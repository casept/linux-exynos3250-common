/*
 * Copyright (C) 2013 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/pagemap.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/poll.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/regs-gpio.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>

#include <linux/sensor/sensors_core.h>
#include <linux/sensor/k6ds3.h>


#define VENDOR_CHIP "ST Microelectronics"
#define SENSOR_NAME "K6DS3TR"

#define I_AM_K6DS3TR	105

#define XL_HM_MODE 1        // normal mode
#define ODR_XL	4			// data rate = 104Hz
#define FS_XL	1			// range +/-16g
#define BW_SCHL_ODR	1 		// band with dertemind by setting BW_XL
#define	BW_XL	0 			// band with = 400hz
#define DYNAMIC_THRESHOLD	300

#define SET_BIT(reg,n)	(reg | (0x01<<n))
#define CLEAR_BIT(reg,n)	(reg & (~(0x01<<n)))

#define REST_DEVICE 0x01
#define ENABLE_OUTPUT_ACCEL_SENSOR 0x38

#define ACC_CAL_PATH	"/csa/sensor/accel_cal_data"
#define GYRO_CAL_PATH	"/csa/sensor/gyro_cal_data"

#define ACC_CAL_TIME	20
#define ACC_IDEAL	4096	/*raw data 1G in range = 8G*/

#define ABS(a)		((a) > 0 ? (a) : -(a))

/* Selftest: 90~1700mg @ 2G */
#define K6DS3_ACC_MIN_ST			((int)(90/0.061f))
#define K6DS3_ACC_MAX_ST			((int)(1700/0.061f) + 1)

/*config for accel sensor*/
static const __s8 position_map[][3][3] = {
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* 0 top/lower-right */
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* 1 top/lower-left */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* 2 top/upper-left */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* 3 top/upper-right */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* 4 bottom/lower-right */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* 5 bottom/lower-left */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* 6 bottom/upper-left */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* 7 bottom/upper-right*/
};

typedef enum {
	ACC_POWER_DOWN,
	ACC_13HZ,
	ACC_26HZ,
	ACC_52HZ,
	ACC_104HZ,
	ACC_208HZ,
	ACC_416HZ,
	ACC_833HZ,
	ACC_1_66KHZ,
	ACC_3_33KHZ,
	ACC_6_66KHZ,
}accel_mode;

typedef enum {
	ACC_RANGE_2G,
	ACC_RANGE_16G,
	ACC_RANGE_4G,
	ACC_RANGE_8G,
}accel_range;

typedef enum {
	ACC_BANDWIDTH_400,
	ACC_BANDWIDTH_200,
	ACC_BANDWIDTH_100,
	ACC_BANDWIDTH_50,
}bandwidth_accel;

typedef enum {
	HIGH_PERFORMANCE_ENABLE,
	HIGH_PERFORMANCE_DISABLE,
}high_performance;

typedef enum  {
	DERTRMINED_BY_ODR,
	DERTRMINED_BY_BW_XL,
}bandwidth_selection;

/*config for gyro sensor*/
typedef enum {
	GYRO_POWER_DOWN,
	GYRO_13HZ,
	GYRO_26HZ,
	GYRO_52HZ,
	GYRO_104HZ,
	GYRO_208HZ,
	GYRO_416HZ,
	GYRO_833HZ,
	GYRO_1_66KHZ,
}gyro_mode;

typedef enum {
	GYRO_RANGE_245DPS,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS,
	GYRO_RANGE_125DPS,
}gyro_range;

typedef enum {
	FREQ_CUTOFF_0_0081HZ,
	FREQ_CUTOFF_0_0032HZ,
	FREQ_CUTOFF_2_07HZ,
	FREQ_CUTOFF_16_32HZ,
}freq_cutoff;

typedef enum {
	INT1_PAD,
	INT2_PAD,
} int_pad;

enum {
	REACTIVE_OFF = 0,
	REACTIVE_ON = 1,
	REACTIVE_FACTORY = 2,
	REACTIVE_MAX,
};
struct k6ds3_data {
	struct i2c_client *i2c_client;
	struct mutex mutex;
	struct input_dev *accel_input;
	struct input_dev *gyro_input;
	struct device *accel_sensor_device;
	struct device *gyro_sensor_device;

	struct delayed_work accel_work;
	struct delayed_work gyro_work;

	int position;
	int int_selection;
	atomic_t accel_delay;
	atomic_t accel_enabled;
	atomic_t gyro_delay;
	atomic_t gyro_enabled;
	s16 acc_cal[3];
	s16 gyro_cal[3];

	accel_mode	accel_data_rate;
	gyro_mode	gyro_data_rate;

	int acc_int;
	int irq_acc;

	struct wake_lock reactive_wake_lock;
	int dyamic_threshold[2];
	int movent_recog_flag;
	atomic_t interrupt_state;
	unsigned long motion_recg_st_time;
};

struct k6ds3_acc {
	s16 x;
	s16 y;
	s16 z;
};

struct k6ds3_gyro {
	s16 gx;
	s16 gy;
	s16 gz;
};

static int k6ds3_i2c_read(struct i2c_client *i2c_client,
			u8 reg_addr, int len, u8 *data)
{
	int ret = 0;
	int err_count = 0;
	struct i2c_msg msg[2];

	msg[0].addr = i2c_client->addr;
	msg[0].flags = i2c_client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = i2c_client->addr;
	msg[1].flags = i2c_client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	if (err_count <= 3) {
		ret = i2c_transfer(i2c_client->adapter, msg, 2);

		if (ret < 0) {
			err_count++;
			pr_err("[SENSOR]%s - i2c read(0x%x) error %d\n",
				__func__, reg_addr, ret);
		}
	} else {
		ret = -1;
	}

	return ret;
}

static int k6ds3_i2c_write(struct i2c_client *i2c_client,
			u8 reg_addr, int len, u8 *data)
{
	u8 send[len + 1];
	int ret;
	struct i2c_msg msg;

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = i2c_client->addr;
	msg.flags = i2c_client->flags;
	msg.len = len;
	msg.buf = send;

	ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("[SENSOR]%s - i2c read(0x%x) error %d\n",
				__func__, reg_addr, ret);

	return ret;
}

static int k6ds3_write_data_with_mask(struct k6ds3_data *sdata,
			u8 reg_addr, u8 mask, u8 data)
{
	int err;
	u8 new_data = 0x00, bak_data;

	err = k6ds3_i2c_read(sdata->i2c_client, reg_addr, 1, &new_data);
	if (err<0){
		pr_err("[SENSOR] %s - read reg %d is failed\n", __func__, reg_addr);
		return err;
	}

	bak_data = new_data;
	new_data = ((new_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data != bak_data){
		err = k6ds3_i2c_write(sdata->i2c_client, reg_addr, 1, &new_data);
		if (err<0){
			pr_err("[SENSOR] %s - write reg %d is failed\n", __func__, reg_addr);
			return err;
		}
	}

	return err;

}

static int k6ds3_set_mode_gyro(struct k6ds3_data *data,
						high_performance enable, gyro_mode mode)
{
	int ret=0;
	u8 reg;

	/*enable/disable hight performent*/
	ret = k6ds3_i2c_read(data->i2c_client, CTRL7_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL7_G is failed\n", __func__);
		return ret;
	}

	switch(enable){
		case HIGH_PERFORMANCE_ENABLE:{		/*enable hight performent*/
			reg = CLEAR_BIT(reg, 7);
			break;
		}

		case HIGH_PERFORMANCE_DISABLE:{		/*disable hight performent*/
			reg = SET_BIT(reg, 7);
			break;
		}

		default:
			pr_err("[SENSOR]: %s - high_performance set up is failed\n", __func__);
			return -1;
	}
	ret = k6ds3_i2c_write(data->i2c_client, CTRL7_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL7_G is failed\n", __func__);
		return ret;
	}


	/*set gyro mode*/
	ret = k6ds3_i2c_read(data->i2c_client, CTRL2_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL2_G is failed\n", __func__);
		return ret;
	}
	reg = (reg & 0xF0) | (mode<<4);
	ret = k6ds3_i2c_write(data->i2c_client, CTRL2_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL2_G is failed\n", __func__);
		return ret;
	}

	return ret;
}

static void k6sd3_poweronoff_gyro_sensor(struct k6ds3_data *data, bool enable)
{
	if (enable){
		k6ds3_set_mode_gyro(data, HIGH_PERFORMANCE_ENABLE, data->gyro_data_rate);
	}else{
		k6ds3_set_mode_gyro(data, HIGH_PERFORMANCE_ENABLE, GYRO_POWER_DOWN);
	}
}

static int k6ds3_set_range_gyro(struct k6ds3_data *data, gyro_range range)
{
	int ret;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, CTRL2_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL2_G is failed\n", __func__);
		return ret;
	}

	if (range == GYRO_RANGE_125DPS){
		reg = SET_BIT(reg,1);
	} else{
		reg = (reg & 0xF3) | (range<<2);
	}

	ret = k6ds3_i2c_write(data->i2c_client, CTRL2_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL2_G is failed\n", __func__);
		return ret;
	}

	return ret;
}


static int k6ds3_set_high_pass_filter_gyro(struct k6ds3_data *data, bool enable,
		freq_cutoff FREQ)
{
	int ret=0;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, CTRL7_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL7_G is failed\n", __func__);
		return ret;
	}

	if (enable){
		reg = SET_BIT(reg,6);
		reg = (reg & 0xCF) | (FREQ<<4);
	} else{
		reg = CLEAR_BIT(reg,6);
	}

	ret = k6ds3_i2c_write(data->i2c_client, CTRL7_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL7_G is failed\n", __func__);
		return ret;
	}

	return ret;
}

static int k6ds3_enable_output_data(struct k6ds3_data *data, bool enable)
{
	int ret=0;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, CTRL10_C, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL10_C is failed\n", __func__);
		return ret;
	}
	if (enable){
		reg = SET_BIT(reg,3);
		reg = SET_BIT(reg,4);
		reg = SET_BIT(reg,5);
	} else{
		reg = CLEAR_BIT(reg,3);
		reg = CLEAR_BIT(reg,4);
		reg = CLEAR_BIT(reg,5);
	}

	ret = k6ds3_i2c_write(data->i2c_client, CTRL10_C, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL10_C is failed\n", __func__);
		return ret;
	}


	return ret;
}


/*accel config*/
static int k6ds3_set_mode_accel(struct k6ds3_data *data,
						high_performance enable, accel_mode mode)
{
	int ret=0;
	u8 reg;

	/*enable/disable hight performent*/
	ret = k6ds3_i2c_read(data->i2c_client, CTRL6_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL6_G is failed\n", __func__);
		return ret;
	}

	switch(enable){
		case HIGH_PERFORMANCE_ENABLE:{		/*enable hight performent*/
			reg = CLEAR_BIT(reg, 4);
			break;
		}

		case HIGH_PERFORMANCE_DISABLE:{		/*disable hight performent*/
			reg = SET_BIT(reg, 4);
			break;
		}

		default:
			pr_err("[SENSOR]: %s - high_performance set up is failed\n", __func__);
			return -1;

	}

	ret = k6ds3_i2c_write(data->i2c_client, CTRL6_G, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL6_G is failed\n", __func__);
		return ret;
	}

	/*set accel mode*/
	ret = k6ds3_i2c_read(data->i2c_client, CTRL1_XL, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL1_XL is failed\n", __func__);
		return ret;
	}
	reg = (reg & 0x0F) | (mode<<4);
	ret = k6ds3_i2c_write(data->i2c_client, CTRL1_XL, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL1_XL is failed\n", __func__);
		return ret;
	}

	return ret;
}

static void k6sd3_poweronoff_accel_sensor(struct k6ds3_data *data, bool enable){

	if (enable){
		k6ds3_set_mode_accel(data, HIGH_PERFORMANCE_ENABLE, data->accel_data_rate);
	} else{
		k6ds3_set_mode_accel(data, HIGH_PERFORMANCE_ENABLE, ACC_POWER_DOWN);
	}
}

static int k6ds3_set_range_accel(struct k6ds3_data *data, accel_range range)
{
	int ret = 0;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, CTRL1_XL, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register CTRL1_XL is failed\n", __func__);
		return ret;
	}
	reg = (reg & 0xF3) | (range<<2);
	ret = k6ds3_i2c_write(data->i2c_client, CTRL1_XL, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL1_XL is failed\n", __func__);
		return ret;
	}

	return ret;
}

static int k6ds3_set_bandwidth_accel(struct k6ds3_data *data, bandwidth_selection bw_select,
					bandwidth_accel bandwidth)
{
	int ret = 0;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, CTRL4_C, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read CTRL4_C is failed\n", __func__);
		return ret;
	}

	switch(bw_select){
		case DERTRMINED_BY_ODR:{
			reg = CLEAR_BIT(reg,7);
			ret = k6ds3_i2c_write(data->i2c_client, CTRL4_C, 1, &reg);
			if (ret<0){
				pr_err("[SENSOR]: %s - write CTRL4_C is failed\n", __func__);
				return ret;
			}
			break;
		}

		case DERTRMINED_BY_BW_XL:{
			reg = SET_BIT(reg,7);
			ret = k6ds3_i2c_write(data->i2c_client, CTRL4_C, 1, &reg);
			if (ret<0){
				pr_err("[SENSOR]: %s - write CTRL4_C is failed\n", __func__);
				return ret;
			}

			/*set bandwidth by BW_XL[1:0] in CTRL1_XL*/
			ret = k6ds3_i2c_read(data->i2c_client, CTRL1_XL, 1, &reg);
			if (ret<0){
				pr_err("[SENSOR]: %s - read CTRL1_XL is failed\n", __func__);
				return ret;
			}
			reg = (reg & 0xFC) | (bandwidth<<0);
			ret = k6ds3_i2c_write(data->i2c_client, CTRL1_XL, 1, &reg);
			if (ret<0){
				pr_err("[SENSOR]: %s - write CTRL1_XL is failed\n", __func__);
				return ret;
			}
			break;
		}

		default:{
			pr_err("[SENSOR]: %s - select bandwidth is failed\n", __func__);
			ret = -1;
		}
	}

	return ret;
}

static void k6ds3_apply_orientation(struct k6ds3_data *data,
	s16 raw[3], s16 orient_raw[3])
{
	int i = 0, j = 0;
	s32 value = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++)
			value += position_map[data->position][i][j] * raw[j];

		/*trim the edge */
		if (value > 32767)
			value = 32767;
		else if (value < -32768)
			value = -32768;

		orient_raw[i] = (s16) value;
		value = 0;
	}
}

static int k6ds3_input_read_gyro_raw_data(struct k6ds3_data *data)
{
	u8 regs[6] = {0, };
	s16 raw[3];
	int result;

	result = k6ds3_i2c_read(data->i2c_client, OUTX_L_G, 6, regs);
	if (result<0){
		pr_err("[SENSOR]: %s - read OUTX_L_G is failed\n", __func__);
		return result;
	}

	raw[0] = ((s16) ((s16) regs[1] << 8)) | regs[0];
	raw[1] = ((s16) ((s16) regs[3] << 8)) | regs[2];
	raw[2] = ((s16) ((s16) regs[5] << 8)) | regs[4];

	input_report_rel(data->gyro_input, REL_RX, raw[0]);
	input_report_rel(data->gyro_input, REL_RY, raw[1]);
	input_report_rel(data->gyro_input, REL_RZ, raw[2]);

	input_sync(data->gyro_input);

	pr_err("[SENSOR]: %s - x=%d  y=%d   z=%d\n", __func__, raw[0], raw[1], raw[2]);
	return 0;
}

static int k6sd3_init_accel_sensor(struct k6ds3_data *data){
	int ret=0;
	u8 reg;

	/*high performance enable - output data rate = power down*/
	ret = k6ds3_set_mode_accel(data, HIGH_PERFORMANCE_ENABLE, ACC_POWER_DOWN);
	if (ret<0){
		pr_err("[SENSOR]: %s - set mode for accel is failed\n", __func__);
		return ret;
	}

	/*set range for accel sensor = 8G*/
	ret = k6ds3_set_range_accel(data, ACC_RANGE_8G);
	if (ret<0){
		pr_err("[SENSOR]: %s - set range for accel is failed\n", __func__);
		return ret;
	}

	/*set bandwidth dertermined by BW_XL[1:0] and bandwidth=400Hz*/
	ret = k6ds3_set_bandwidth_accel(data, DERTRMINED_BY_BW_XL, ACC_BANDWIDTH_400);
	if (ret<0){
		pr_err("[SENSOR]: %s - set bandwidth for accel is failed\n", __func__);
		return ret;
	}

	/*enable output data (x,y,z axis)for accel sensor */
	reg = ENABLE_OUTPUT_ACCEL_SENSOR;
	ret = k6ds3_i2c_write(data->i2c_client, CTRL9_XL, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL9_XL is failed\n", __func__);
		return ret;
	}

	return ret;
}

static int k6ds3_init_gyro_sensor(struct k6ds3_data *data){
	int ret = 0;

	/*high performance enable - output data rate = power down*/
	ret = k6ds3_set_mode_gyro(data, HIGH_PERFORMANCE_ENABLE, GYRO_POWER_DOWN);
	if (ret<0){
		pr_err("[SENSOR]: %s - set mode for accel is failed\n", __func__);
		return ret;
	}

	/*set range for gyro sensor = 1000dps*/
	ret = k6ds3_set_range_gyro(data, GYRO_RANGE_1000DPS);
	if (ret<0){
		pr_err("[SENSOR]: %s - set mode for gyro is failed\n", __func__);
		return ret;
	}

	/*set hight pass filter gyro*/
	ret = k6ds3_set_high_pass_filter_gyro(data, true, FREQ_CUTOFF_2_07HZ);
	if (ret<0){
		pr_err("[SENSOR]: %s - set HPF for gyro is failed\n", __func__);
		return ret;
	}

	/*gyro enable output*/
	ret = k6ds3_enable_output_data(data, true);
	if (ret<0){
		pr_err("[SENSOR]: %s - set output data for gyro is failed\n", __func__);
		return ret;
	}
	return ret;
}


static int k6ds3_reset_device(struct k6ds3_data *data){
	int ret = 0;
	u8 reg;

	reg = REST_DEVICE;
	ret = k6ds3_i2c_write(data->i2c_client, CTRL3_C, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - write register CTRL3_C is failed\n", __func__);
		return ret;
	}

	return ret;
}

static int k6ds3_input_read_accel_raw_data(struct k6ds3_data *data,
	struct k6ds3_acc *acc_raw_data)
{
	u8 regs[6] = {0, };
	s16 raw[3] = {0, }, orien_raw[3] = {0,};
	int result;

	/* set range again */
	result = k6ds3_set_range_accel(data, ACC_RANGE_8G);
	if (result < 0){
		pr_err("[SENSOR]: %s, k6ds3_set_range_accel failed\n", __func__);
		return result;
	}
	result = k6ds3_i2c_read(data->i2c_client, OUTX_L_XL, 6, regs);
	if (result<0){
		pr_err("[SENSOR]: %s - read OUTX_L_XL is failed\n", __func__);
		return result;
	}

	raw[0] = ((s16) ((s16) regs[1] << 8)) | regs[0];
	raw[1] = ((s16) ((s16) regs[3] << 8)) | regs[2];
	raw[2] = ((s16) ((s16) regs[5] << 8)) | regs[4];

	k6ds3_apply_orientation(data, raw, orien_raw);

	/* apply calibration data*/
	acc_raw_data->x = orien_raw[0] -data->acc_cal[0];
	acc_raw_data->y = orien_raw[1] -data->acc_cal[1];
	acc_raw_data->z = orien_raw[2] -data->acc_cal[2];

	pr_err("[SENSOR]: %s - x=%d  y=%d   z=%d\n", __func__, acc_raw_data->x, acc_raw_data->y, acc_raw_data->z);
	return 0;
}

static int acc_data_is_ready(struct k6ds3_data *data){
	int ret = 1;
	u8 reg;

	ret = k6ds3_i2c_read(data->i2c_client, STATUS_REG, 1, &reg);
	if (ret<0){
		pr_err("[SENSOR]: %s - read STATUS_REG is failed\n", __func__);
		return 0;
	}

	reg = reg & 0x01;
	if (reg){
		return 1;	/*data ready*/
	}else{
		return 0;	/*data not ready*/
	}
}

static int k6ds3_selftest_accel_mode_enable(struct k6ds3_data *data,
	int x_diff, int y_diff, int z_diff)
{
	u8 reg;
	int count;
	int i;
	u8 acc_data[6] = {0, };
	s16 OUTX_NOST, OUTY_NOST, OUTZ_NOST;
	s16 OUTX_ST, OUTY_ST, OUTZ_ST;

	/* init sensor, turn on sensor, enable X/Y/Z axis*/
	/* Set BDU=1, FS=2G, ODR=52Hz*/
	reg = 0x30;
	k6ds3_i2c_write(data->i2c_client, CTRL1_XL, 1, &reg);
	reg = 0x00;
	k6ds3_i2c_write(data->i2c_client, CTRL2_G, 1, &reg);
	reg = 0x44;
	k6ds3_i2c_write(data->i2c_client, CTRL3_C, 1, &reg);
	reg = 0x00;
	k6ds3_i2c_write(data->i2c_client, CTRL4_C, 1, &reg);
	k6ds3_i2c_write(data->i2c_client, CTRL5_C, 1, &reg);
	k6ds3_i2c_write(data->i2c_client, CTRL6_G, 1, &reg);
	k6ds3_i2c_write(data->i2c_client, CTRL7_G, 1, &reg);
	k6ds3_i2c_write(data->i2c_client, CTRL8_XL, 1, &reg);
	reg = 0x38;
	k6ds3_i2c_write(data->i2c_client, CTRL9_XL, 1, &reg);
	reg = 0x00;
	k6ds3_i2c_write(data->i2c_client, CTRL10_C, 1, &reg);

	msleep(200); /*wait 200ms*/

	/*check Acc data ready bit*/
	count = 0;
	while (true){
		if (acc_data_is_ready(data)){
			break;
		} else{
			count++;
		}

		if (count>=10){
			pr_err("[SENSOR]: %s - check data ready is time out = %d\n", __func__, count);
			break;
		}
	}

	/*read acc data 5 times - average the stored data for each axis*/
	OUTX_NOST = 0;
	OUTY_NOST = 0;
	OUTZ_NOST = 0;
	for (i=0; i<5; i++){
		k6ds3_i2c_read(data->i2c_client, OUTX_L_XL, 6, acc_data);
		OUTX_NOST += ((s16) ((s16) acc_data[1] << 8)) | acc_data[0];
		OUTY_NOST += ((s16) ((s16) acc_data[3] << 8)) | acc_data[2];
		OUTZ_NOST += ((s16) ((s16) acc_data[5] << 8)) | acc_data[4];
	}
	OUTX_NOST = OUTX_NOST/5;
	OUTY_NOST = OUTY_NOST/5;
	OUTZ_NOST = OUTZ_NOST/5;

	/*Enable Acc Self Test*/
	reg = ST_POSITIVE_MODE_ACC;
	k6ds3_i2c_write(data->i2c_client, CTRL5_C, 1, &reg);

	msleep(200); /*wait 200ms*/

	/*check Acc data ready bit*/
	count = 0;
	while (true){
		if (acc_data_is_ready(data)){
			break;
		} else{
			count++;
		}

		if (count>=10){
			pr_err("[SENSOR]: %s - check data ready is time out (selftest)= %d\n", __func__, count);
			break;
		}
	}

	/*read acc data 5 times - average the stored data for each axis in selftest mode*/
	OUTX_ST = 0;
	OUTY_ST = 0;
	OUTZ_ST = 0;
	for (i=0; i<5; i++){
		k6ds3_i2c_read(data->i2c_client, OUTX_L_XL, 6, acc_data);
		OUTX_ST += ((s16) ((s16) acc_data[1] << 8)) | acc_data[0];
		OUTY_ST += ((s16) ((s16) acc_data[3] << 8)) | acc_data[2];
		OUTZ_ST += ((s16) ((s16) acc_data[5] << 8)) | acc_data[4];
	}
	OUTX_ST = OUTX_ST/5;
	OUTY_ST = OUTY_ST/5;
	OUTZ_ST = OUTZ_ST/5;

	x_diff= ABS(OUTX_ST - OUTX_NOST);
	y_diff= ABS(OUTY_ST - OUTY_NOST);
	z_diff= ABS(OUTZ_ST - OUTZ_NOST);

	if (x_diff > K6DS3_ACC_MIN_ST && x_diff < K6DS3_ACC_MAX_ST &&
	     y_diff > K6DS3_ACC_MIN_ST && y_diff < K6DS3_ACC_MAX_ST &&
	     z_diff > K6DS3_ACC_MIN_ST && z_diff < K6DS3_ACC_MAX_ST)
	{
		return 1;	/*Pass*/
	} else {
		return 0;	/*Fail*/
	};
}

static int k6ds3_gyro_fifo_test(struct k6ds3_data *sdata,
	s16 *zero_rate_lsb, s32 *fifo_cnt, u8 *fifo_pass, s16 *slot_raw)
{
	int err;
	u8 buf[5] = {0x00,};
	s16 nFifoDepth = (ST_LSM6DS3_FIFO_TEST_DEPTH + 1) * 3;
	bool zero_rate_read_2nd = 0;
	s16 raw[3] = {0,}, zero_rate_delta[3] = {0,}, length = 0;
	s16 data[ST_LSM6DS3_FIFO_TEST_DEPTH * 3] = {0,};
	s32 i = 0, j = 0, sum_raw[3] = {0,};

	err = k6ds3_write_data_with_mask(sdata,
			CTRL4_C, 0x40, ST_LSM6DS3_DIS_BIT);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata,
			CTRL4_C, 0x01, ST_LSM6DS3_EN_BIT);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata,
			CTRL2_G, 0xf0, GYRO_104HZ);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata,
			CTRL2_G, 0xf0, GYRO_104HZ);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata,
			CTRL2_G, 0x0C, GYRO_RANGE_2000DPS);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata,
			CTRL3_C, 0x40, ST_LSM6DS3_DIS_BIT);
	if (err<0)
		return err;

	buf[0] = (u8)(nFifoDepth & 0xff);
	err = k6ds3_i2c_write(sdata->i2c_client, FIFO_CTRL1, 1, buf);
	if (err<0)
		return err;

	buf[0] = (u8)((nFifoDepth >> 8) & 0x0f);
	err = k6ds3_write_data_with_mask(sdata,
			FIFO_CTRL2, 0x0f, buf[0]);
	if (err<0)
		return err;

	buf[0] = (0x01 << 3);
	err = k6ds3_i2c_write(sdata->i2c_client, FIFO_CTRL3, 1, buf);
	if (err<0)
		return err;

	buf[0] = 0x00;
	err = k6ds3_i2c_write(sdata->i2c_client, FIFO_CTRL4, 1, buf);
	if (err<0)
		return err;

	buf[0] = (0x04 << 3) | 0x01;
	err = k6ds3_i2c_write(sdata->i2c_client, FIFO_CTRL5, 1, buf);
	if (err<0)
		return err;

	mdelay(800);

read_zero_rate_data:
	err = k6ds3_write_data_with_mask(sdata, FIFO_CTRL5, 0x07, 0x00);
	if (err<0)
		return err;

	err = k6ds3_write_data_with_mask(sdata, FIFO_CTRL5, 0x07, 0x01);
	if (err<0)
		return err;

	length = ST_LSM6DS3_FIFO_TEST_DEPTH * 3;
	while(1) {
		mdelay(20);
		err = k6ds3_i2c_read(sdata->i2c_client, FIFO_STATUS1, 4, buf);
		if (err<0)
			return err;
		else {
			if (buf[1] & 0xA0)
				break;
			else {
				if((--length) == 0){
					pr_err("[SENSOR] %s - FIFO not filled\n",__func__);
					return -EBUSY;
				}
			}
		}
	}

	length = ST_LSM6DS3_FIFO_TEST_DEPTH * 3;
	for ( i= 0; i < length; i++) {
		err = k6ds3_i2c_read(sdata->i2c_client, FIFO_DATA_OUT_L, 2, (u8 *)(data + i));
		if (err<0){
			pr_err("[SENSOR] %s - Reading FIFO output is fail\n",__func__);
			return err;
		}
	}

	*fifo_cnt = 0;
	/*Check fifo pass or fail*/
	for (i = 0; i < length; i += 3){
		*fifo_cnt += 1;

		raw[0] = data[i];
		raw[1] = data[i + 1];
		raw[2] = data[i + 2];

		sum_raw[0] += raw[0];
		sum_raw[1] += raw[1];
		sum_raw[2] += raw[2];

		for (j = 0; j < 3; j++) {
			if (raw[j] < ST_LSM6DS3_GYR_MIN_ZRL || raw[j] > ST_LSM6DS3_GYR_MAX_ZRL){
				slot_raw[0] = raw[0] * 7 / 100;
				slot_raw[1] = raw[1] * 7 / 100;
				slot_raw[2] = raw[2] * 7 / 100;
				pr_err("[SENSOR] %s - fifo fail : %d\n", __func__, *fifo_pass);
				return -EAGAIN;
			}
		}
	}

	for (i=0; i < 3; i++) {
		zero_rate_lsb[i] = (sum_raw[i] + (ST_LSM6DS3_FIFO_TEST_DEPTH / 2)) / ST_LSM6DS3_FIFO_TEST_DEPTH;
	}

	if (zero_rate_read_2nd == 1) {
		*fifo_pass = 1;
		pr_info("[SENSOR] %s - fifo pass: %d\n",__func__, *fifo_pass);
		/* check zero rate second time */
		zero_rate_delta[0] -=zero_rate_lsb[0];
		zero_rate_delta[1] -=zero_rate_lsb[1];
		zero_rate_delta[2] -=zero_rate_lsb[2];
		for (i = 0; i < 3; i++) {
			if (ABS(zero_rate_delta[i]) > ST_LSM6DS3_GYR_ZRL_DELTA) {
				pr_info("[SENSOR] %s - zero_rate_delta fail : [%d] %d\n",
					__func__, i, zero_rate_delta[i]);
				return -EAGAIN;
			}
		}
	} else {
		/* check zero_rate fisrt time, go to check again */
		zero_rate_read_2nd = 1;
		sum_raw[0] = 0;
		sum_raw[1] = 0;
		sum_raw[2] = 0;
		zero_rate_delta[0] = zero_rate_lsb[0];
		zero_rate_delta[1] = zero_rate_lsb[1];
		zero_rate_delta[2] = zero_rate_lsb[2];

		goto read_zero_rate_data;
	}

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;

	err = k6ds3_i2c_write(sdata->i2c_client, FIFO_CTRL5, 1, buf);
	if (err < 0){
		return err;
	}

	return 0;

}

static int k6ds3_gyr_hw_selftest(struct k6ds3_data *sdata,
		s32 NOST[3], s32 ST[3], s32 DIFF_ST[3])
{
	int err;
	u8 buf[1] = {0x00,};
	s16 nOutData[3] = {0,};
	s32 i, retry;
	u8 testset_regs[10] = {0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x38};
	u8 reg;
	pr_info("[SENSOR] %s, start\n", __func__);

	for (i = 0; i < 3; i++){
		NOST[i] = 0;
		ST[i] = 0;
	}

	err = k6ds3_i2c_write(sdata->i2c_client, CTRL1_XL, 10, testset_regs);
	if (err<0)
		goto G_HW_SELF_EXIT;

	mdelay(800);

	retry = 5;
	do {
		mdelay(1);
		err = k6ds3_i2c_read(sdata->i2c_client, STATUS_REG, 1, &reg);
		if (err < 0)
			goto G_HW_SELF_EXIT;
		retry--;
		if (!retry)
			break;
	} while (!(reg & 0x02));

	err = k6ds3_i2c_read(sdata->i2c_client, OUTX_L_G, 6, (u8*)nOutData);
	if (err < 0)
		goto G_HW_SELF_EXIT;

	for (i = 0; i < 6; i++){
		retry = 5;
		do {
			mdelay(1);
			err = k6ds3_i2c_read(sdata->i2c_client, STATUS_REG, 1, &reg);
			if (err<0)
				goto G_HW_SELF_EXIT;
			retry--;
			if (!retry)
				break;
		} while (!(reg & 0x02));
		err = k6ds3_i2c_read(sdata->i2c_client, OUTX_L_G, 6, (u8*)nOutData);
		if (err < 0)
			goto G_HW_SELF_EXIT;
		if (i>0){
			NOST[0] += nOutData[0];
			NOST[1] += nOutData[1];
			NOST[2] += nOutData[2];
		}
	}

	NOST[0]  /= 5;
	NOST[1]  /= 5;
	NOST[2]  /= 5;

	/* enable selftest for gyro*/
	reg = 0x04;
	err = k6ds3_i2c_write(sdata->i2c_client, CTRL5_C, 1, &reg);
	if (err<0)
		goto G_HW_SELF_EXIT;

	mdelay(60);

	retry = 5;
	do {
		mdelay(1);
		err = k6ds3_i2c_read(sdata->i2c_client, STATUS_REG, 1, buf);
		if (err < 0)
			goto G_HW_SELF_EXIT;
		retry--;
		if (!retry)
			break;
	} while (!(buf[0] & 0x02));

	err = k6ds3_i2c_read(sdata->i2c_client, OUTX_L_G, 6, (u8*)nOutData);
	if (err < 0)
		goto G_HW_SELF_EXIT;

	for (i = 0; i < 6; i++){
		retry = 5;
		do {
			mdelay(1);
			err = k6ds3_i2c_read(sdata->i2c_client, STATUS_REG, 1, &reg);
			if (err<0)
				goto G_HW_SELF_EXIT;
			retry--;
			if (!retry)
				break;
		} while (!(reg & 0x02));
		err = k6ds3_i2c_read(sdata->i2c_client, OUTX_L_G, 6, (u8*)nOutData);
		if (err < 0)
			goto G_HW_SELF_EXIT;
		if (i>0){
			ST[0] += nOutData[0];
			ST[1] += nOutData[1];
			ST[2] += nOutData[2];
		}
	}

	k6ds3_reset_device(sdata);

	ST[0]  /= 5;
	ST[1]  /= 5;
	ST[2]  /= 5;

	retry = 0;
	for (i = 0; i < 3; i++) {
		DIFF_ST[i] = ABS(ST[i] - NOST[i]);
		if ((ST_LSM6DS3_GYR_MIN_ST > DIFF_ST[i]) || (ST_LSM6DS3_GYR_MAX_ST < DIFF_ST[i])) {
			retry++;
		}
	}

	if (retry > 0) {
		return 0;	/* FAIL*/
	}
	else {
		return 1;	/* PASS */
	}
G_HW_SELF_EXIT:
	return err;
}

static int k6ds3_gyro_save_calibration(struct k6ds3_data *cdata,
		s32 cal_data[3])
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	pr_info("[SENSOR] %s\n", __func__);

	cdata->gyro_cal[0] = cal_data[0];
	cdata->gyro_cal[1] = cal_data[1];
	cdata->gyro_cal[2] = cal_data[2];

	/* save cal data */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(GYRO_CAL_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		pr_err("[SENSOR] %s Can't open calibration file %d\n", __func__, ret);
		return 0;
	}

	ret = cal_filp->f_op->write(cal_filp,
			(char *)&cdata->gyro_cal, 3 * sizeof(s16),
			&cal_filp->f_pos);
	if (ret != 3 * sizeof(s16)) {
		pr_err("[SENSOR] %s: Can't write the cal data to file %d\n",
			__func__, ret);
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	pr_info("%s: %d, %d, %d\n", __func__, cdata->gyro_cal[0],
		cdata->gyro_cal[1], cdata->gyro_cal[2]);

	return 1;
}

static int k6ds3_selftest_gyro_enable_mode(struct k6ds3_data *data, char *out_str)
{
	int err;
	u8 cal_pass = 0, fifo_pass = 0;
	s16 zero_rate_data[3] = {0,}, slot_raw[3] = {0,};
	s32 NOST[3] = {0,}, ST[3] = {0,}, DIFF_ST[3] = {0,};
	s32 fifo_count = 0, hw_st_ret = 0;

	err = k6ds3_reset_device(data);
	if (err<0){
		pr_err("[SENSOR] reset device fail\n");
		return err;
	}

	/* test fifo gyro*/
	err = k6ds3_gyro_fifo_test(data, zero_rate_data,
		&fifo_count, &fifo_pass, slot_raw);
	if (err < 0) {
		pr_err("[SENSOR] %s - Gyro FIFO test fail\n",__func__);
		pr_err("[SENSOR] %s - fail ra = %d %d %d",__func__, slot_raw[0], slot_raw[1], slot_raw[2]);
		goto reset_dev;
	}

	/* test hardware */
	err = k6ds3_reset_device(data);
	if (err<0){
		pr_err("[SENSOR] reset device fail\n");
		return err;
	}
	hw_st_ret = k6ds3_gyr_hw_selftest(data, NOST, ST, DIFF_ST);
	cal_pass = k6ds3_gyro_save_calibration(data, NOST);
	zero_rate_data[0] = zero_rate_data[0] * 7 / 100;
	zero_rate_data[1] = zero_rate_data[1] * 7 / 100;
	zero_rate_data[2] = zero_rate_data[2] * 7 / 100;
	NOST[0] = NOST[0] * 7 / 100;
	NOST[1] = NOST[1] * 7 / 100;
	NOST[2] = NOST[2] * 7 / 100;
	ST[0] = ST[0] * 7 / 100;
	ST[1] = ST[1] * 7 / 100;
	ST[2] = ST[2] * 7 / 100;
	DIFF_ST[0] = DIFF_ST[0] * 7 / 100;
	DIFF_ST[1] = DIFF_ST[1] * 7 / 100;
	DIFF_ST[2] = DIFF_ST[2] * 7 / 100;

	pr_info("%s, zero rate = %d %d %d\nST = %d %d %d\nNOST = %d %d %d\nst_diff = %d %d %d\nfifo = %d cal = %d\n",
		__func__, zero_rate_data[0], zero_rate_data[1], zero_rate_data[2],
		ST[0], ST[1], ST[2], NOST[0], NOST[1], NOST[2],
		DIFF_ST[0], DIFF_ST[1], DIFF_ST[2], fifo_pass, cal_pass);
	if (hw_st_ret > 0)
		pr_info("%s, Gyro selftest pass\n", __func__);
	else {
		pr_info("%s, Gyro selftest fail\n", __func__);
	}
reset_dev:
	k6ds3_reset_device(data);
	if(!fifo_pass)
		return snprintf(out_str, PAGE_SIZE, "%d,%d,%d\n"
		,slot_raw[0], slot_raw[1], slot_raw[2]);
	else
		return snprintf(out_str, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
		,zero_rate_data[0], zero_rate_data[1], zero_rate_data[2]
		,NOST[0], NOST[1], NOST[2]
		,ST[0], ST[1], ST[2]
		,DIFF_ST[0], DIFF_ST[1], DIFF_ST[2], fifo_pass, cal_pass);

}
static int k6ds3_accel_open_calibration(struct k6ds3_data *data)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACC_CAL_PATH,
		O_RDONLY, S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		pr_err("[SENSOR] Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&data->acc_cal,
			3 * sizeof(s16), &cal_filp->f_pos);
	if (err != 3 * sizeof(s16)) {
		pr_err("[SENSOR] Can't read the cal data from file\n");
		err = -EIO;
	}

	pr_info("[SENSOR] (%d,%d,%d)\n", data->acc_cal[0],
			data->acc_cal[1], data->acc_cal[2]);

	filp_close(cal_filp, current->files);
done:
	set_fs(old_fs);
	return err;
}


static int k6ds3_accel_do_calibrate(struct k6ds3_data *data,
		bool enable)
{
	struct file *cal_filp;
	int sum[3] = {0, };
	int err;
	int i;
	mm_segment_t old_fs;
	struct k6ds3_acc	acc_xyz;

	if (!atomic_read(&data->accel_enabled)){
		k6sd3_poweronoff_accel_sensor(data,true);
		mdelay(100);
	}

	for (i = 0; i < ACC_CAL_TIME; i++){
		err =k6ds3_input_read_accel_raw_data(data, &acc_xyz);
		if (err<0){
			pr_err("[SENSOR]: %s - failed to read data \n", __func__);
			goto done;
		}
		sum[0] += acc_xyz.x;
		sum[1] += acc_xyz.y;
		sum[2] += acc_xyz.z;
	}
	if (!atomic_read(&data->accel_enabled))
		k6sd3_poweronoff_accel_sensor(data,false);

	if (enable) {
		data->acc_cal[0] = sum[0] / ACC_CAL_TIME ;
		data->acc_cal[1] = sum[1] / ACC_CAL_TIME;
		if (sum[2] >= 0){
			data->acc_cal[2] =
				sum[2] / ACC_CAL_TIME - ACC_IDEAL;
		} else
			data->acc_cal[2] =
				sum[2] / ACC_CAL_TIME  + ACC_IDEAL;
	} else {
		data->acc_cal[0] = 0;
		data->acc_cal[1] = 0;
		data->acc_cal[2] = 0;
	}

	pr_info("[SENSOR]: %s - cal data (%d %d %d) \n", __func__, data->acc_cal[0],
				data->acc_cal[1], data->acc_cal[2]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACC_CAL_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[SENSOR] Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&data->acc_cal, 3 * sizeof(s16),
		&cal_filp->f_pos);

	if (err != 3 * sizeof(s16)) {
		pr_err("[SENSOR] Can't write cal data to file\n");
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	done:
		set_fs(old_fs);
		return err;
}

static int k6ds3_set_motion_interrrupt(struct k6ds3_data *data,
			bool enable, int_pad mode)
{
	int err;

	atomic_set(&data->interrupt_state, false);

	if (enable) {
		/*select interrupt pad*/
		switch (mode) {
		case INT1_PAD:
			err = k6ds3_write_data_with_mask(data, INT1_CTRL, 0x40, INT1_SIGN_MOT);
			if (err < 0)
				return err;
			err = k6ds3_write_data_with_mask(data, INT2_CTRL, 0x40, 0x00);
			if (err < 0)
				return err;
			break;
		case INT2_PAD:
			err = k6ds3_write_data_with_mask(data, INT2_CTRL, 0x40, INT2_SIGN_MOT);
			if (err < 0)
				return err;
			err = k6ds3_write_data_with_mask(data, INT1_CTRL, 0x40, 0x00);
			if (err < 0)
				return err;
			break;

		/* enable sign motion interrup*/
		err = k6ds3_write_data_with_mask(data, CTRL10_C, 0x04, ST_LSM6DS3_EN_BIT<<2);
		if (err<0)
			return err;
		err = k6ds3_write_data_with_mask(data, CTRL10_C, 0x01, ST_LSM6DS3_EN_BIT<<0);
		if (err<0)
			return err;

		/* enable TILT_EN, PEDO_EN */
		err = k6ds3_write_data_with_mask(data, TAP_CFG, 0x60, (ST_LSM6DS3_EN_BIT << 5) | (ST_LSM6DS3_EN_BIT << 6));
		if (err < 0)
			return err;

		/* set FS = 2G*/
		err = k6ds3_set_range_accel(data, ACC_RANGE_2G);
		if (err < 0)
			return err;

		data->motion_recg_st_time = jiffies;
		}
	} else {

	}

	return 0;
}

static ssize_t k6ds3_accel_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	struct k6ds3_acc acc_data;

	k6ds3_input_read_accel_raw_data(data, &acc_data);
	return sprintf(buf, "%d, %d, %d\n", acc_data.x, acc_data.y, acc_data.z);
}

static ssize_t accel_vendor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR_CHIP);
}

static ssize_t accel_name_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", SENSOR_NAME);
}

static ssize_t k6ds3_temperature_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	s16 temperature;
	u8 data_temp[2] = {0, };
	struct k6ds3_data *data = dev_get_drvdata(dev);

	ret = k6ds3_i2c_read(data->i2c_client, OUT_TEMP_L_ADDR, 2, data_temp);
	if (ret<0){
		pr_err("[SENSOR]: %s - read register OUT_TEMP_L_ADDR is failed\n", __func__);
	}

	temperature = (data_temp[0] | (data_temp[1] << 8)) /16 + 25;
	return snprintf(buf, 0xff, "%d\n", temperature);
}

static ssize_t k6ds3_selftest_accel_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int result;
	int x_diff, y_diff, z_diff;
	struct k6ds3_data *data = dev_get_drvdata(dev);

	result = k6ds3_selftest_accel_mode_enable(data, x_diff, y_diff, z_diff);

	/*soft reset device*/
	k6ds3_reset_device(data);

	/*init sensors*/
	if (atomic_read(&data->accel_enabled)){
		k6sd3_init_accel_sensor(data);
		k6sd3_poweronoff_accel_sensor(data, true);
	} else{
		k6sd3_init_accel_sensor(data);
	}

	if (atomic_read(&data->gyro_enabled)){
		k6ds3_init_gyro_sensor(data);
		k6sd3_poweronoff_gyro_sensor (data, true);
	} else{
		k6ds3_init_gyro_sensor(data);
	}

	return snprintf(buf, 0xff, "%d\n", result);
}

static ssize_t k6ds3_selftest_gyro_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct k6ds3_data *data = dev_get_drvdata(dev);
	ret = k6ds3_selftest_gyro_enable_mode(data, buf);

	/*init sensors*/
	if (atomic_read(&data->accel_enabled)){
		k6sd3_init_accel_sensor(data);
		k6sd3_poweronoff_accel_sensor(data, true);
	} else{
		k6sd3_init_accel_sensor(data);
	}

	if (atomic_read(&data->gyro_enabled)){
		k6ds3_init_gyro_sensor(data);
		k6sd3_poweronoff_gyro_sensor (data, true);
	} else{
		k6ds3_init_gyro_sensor(data);
	}

	return ret;
}

static ssize_t k6ds3_calibration_accel_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int err;
	struct k6ds3_data *data = dev_get_drvdata(dev);

	err = k6ds3_accel_open_calibration(data);
	if (err<0)
		pr_err("[SENSOR] k6ds3_accel_open_calibration fail\n");

	if (!data->acc_cal[0] && !data->acc_cal[1] && !data->acc_cal[2])
		err = -1;
	pr_info ("[SENSOR] - calib: %d  -  %d  -  %d", data->acc_cal[0], data->acc_cal[1], data->acc_cal[2]);
	return sprintf(buf, "%d %d %d %d\n",
		err, data->acc_cal[0], data->acc_cal[1], data->acc_cal[2]);
}

static ssize_t k6ds3_calibration_accel_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;
	int enable;
	struct k6ds3_data *data = dev_get_drvdata(dev);

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	ret = k6ds3_accel_do_calibrate(data, enable);
	if (ret<0)
		pr_info("[SENSOR] %s - accel do calibration failed\n",__func__);

	return count;
}

static ssize_t k6ds3_power_on_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t k6ds3_power_off_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t k6ds3_accel_reactive_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", atomic_read(&data->interrupt_state));
}

static ssize_t k6ds3_accel_reactive_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int err;
	int reactive_mode = -1;
	struct k6ds3_data *data = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &reactive_mode);

	switch (reactive_mode) {
	case REACTIVE_OFF:
		if (data->movent_recog_flag != REACTIVE_OFF){
			pr_info("[SENSOR] %s, reactive alert off", __func__);
			if (device_may_wakeup(&data->i2c_client->dev))
				disable_irq_wake(data->irq_acc);
			disable_irq(data->irq_acc);
			mutex_lock(&data->mutex);
			k6ds3_set_motion_interrrupt(data, false, INT1_PAD);
			data->movent_recog_flag = REACTIVE_OFF;
			mutex_unlock(&data->mutex);
		}
		break;
	case REACTIVE_ON:
	case REACTIVE_FACTORY:
		if (data->movent_recog_flag == REACTIVE_OFF) {
			pr_info("[SENSOR] %s, reactive alert on", __func__);
			mutex_lock(&data->mutex);
			k6ds3_set_motion_interrrupt(data, true, INT1_PAD);
			data->movent_recog_flag = reactive_mode;
			mutex_unlock(&data->mutex);
			enable_irq(data->irq_acc);
			if (device_may_wakeup(&data->i2c_client->dev))
				enable_irq_wake(data->irq_acc);
		}
		break;
	}

	return count;
}

static DEVICE_ATTR(raw_data, 0666, k6ds3_accel_raw_data_show, NULL);
static DEVICE_ATTR(vendor, 0666, accel_vendor_show, NULL);
static DEVICE_ATTR(name, 0666, accel_name_show, NULL);
static DEVICE_ATTR(temperature, 0666, k6ds3_temperature_show, NULL);
static DEVICE_ATTR(power_on, 0666, k6ds3_power_on_show, NULL);
static DEVICE_ATTR(power_off, 0666, k6ds3_power_off_show, NULL);
static DEVICE_ATTR(reactive_alert, 0666, k6ds3_accel_reactive_enable_show, k6ds3_accel_reactive_enable_store);

static struct device_attribute dev_attr_acc_selftest =
	__ATTR(selftest, S_IRUGO, k6ds3_selftest_accel_show, NULL);

static struct device_attribute dev_attr_acc_calibration =
	__ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
		k6ds3_calibration_accel_show,
		k6ds3_calibration_accel_store);


static struct device_attribute *accel_sensor_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_acc_selftest,
	&dev_attr_acc_calibration,
	&dev_attr_reactive_alert,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};

static struct device_attribute dev_attr_gyro_selftest =
	__ATTR(selftest, S_IRUGO, k6ds3_selftest_gyro_show, NULL);

static struct device_attribute *gyro_sensor_attrs[] = {
	&dev_attr_power_on,
	&dev_attr_power_off,
	&dev_attr_temperature,
	&dev_attr_gyro_selftest,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};


static ssize_t accel_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->accel_delay));
}

static ssize_t accel_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	int64_t new_delay;
	int ret;

	ret = kstrtoll(buf, 10, &new_delay);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	atomic_set(&data->accel_delay, new_delay);
	pr_info("[SENSOR]: %s - poll_delay = %lld\n", __func__, new_delay);

	return size;
}


static ssize_t accel_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->accel_enabled));
}

static ssize_t accel_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	int64_t new_enable;
	int ret;

	ret = kstrtoll(buf, 10, &new_enable);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}
	if(new_enable){
		k6sd3_poweronoff_accel_sensor(data,true);
		mdelay(100);
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(5));
	}else{
		cancel_delayed_work_sync(&data->accel_work);
		k6sd3_poweronoff_accel_sensor(data,false);
	}

	atomic_set(&data->accel_enabled, new_enable);
	pr_info("[SENSOR]: %s - enable = %lld\n", __func__, new_enable);

	return size;
}



static ssize_t gyro_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->gyro_delay));
}

static ssize_t gyro_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	int64_t new_delay;
	int ret;

	ret = kstrtoll(buf, 10, &new_delay);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	atomic_set(&data->gyro_delay, new_delay);
	pr_info("[SENSOR]: %s - poll_delay = %lld\n", __func__, new_delay);

	return size;
}


static ssize_t gyro_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->gyro_enabled));
}

static ssize_t gyro_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k6ds3_data *data = dev_get_drvdata(dev);
	int64_t new_enable;
	int ret;

	ret = kstrtoll(buf, 10, &new_enable);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	if (new_enable){
		k6sd3_poweronoff_gyro_sensor(data, true);
		mdelay(100);
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(5));
	}else{
		cancel_delayed_work_sync(&data->gyro_work);
		k6sd3_poweronoff_gyro_sensor(data, false);
	}

	atomic_set(&data->gyro_enabled, new_enable);
	pr_info("[SENSOR]: %s - poll_delay = %lld\n", __func__, new_enable);

	return size;
}


static struct device_attribute dev_attr_accel_enable =
__ATTR(enable, 0666,
	accel_enable_show,
	accel_enable_store);
static struct device_attribute dev_attr_accel_poll_delay =
__ATTR(poll_delay, 0666,
	accel_poll_delay_show,
	accel_poll_delay_store);

static struct device_attribute dev_attr_gyro_enable =
__ATTR(enable, 0666,
	gyro_enable_show,
	gyro_enable_store);
static struct device_attribute dev_attr_gyro_poll_delay =
__ATTR(poll_delay, 0666,
	gyro_poll_delay_show,
	gyro_poll_delay_store);

static struct attribute *accel_sysfs_attrs[] = {
	&dev_attr_accel_enable.attr,
	&dev_attr_accel_poll_delay.attr,
	NULL,
};

static struct attribute *gyro_sysfs_attrs[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_poll_delay.attr,
	NULL,
};

static struct attribute_group accel_attribute_group = {
	.attrs = accel_sysfs_attrs,
};

static struct attribute_group gyro_attribute_group = {
	.attrs = gyro_sysfs_attrs,
};

static int k6ds3_input_init(struct k6ds3_data *data)
{
	int ret = 0;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;

	/* allocate accel sensor input_device */
	accel_dev = input_allocate_device();
	if (!accel_dev)
		return -ENOMEM;

	accel_dev->name = "accelerometer_sensor";
	accel_dev->id.bustype = BUS_I2C;

	input_set_capability(accel_dev, EV_REL, REL_X);
	input_set_capability(accel_dev, EV_REL, REL_Y);
	input_set_capability(accel_dev, EV_REL, REL_Z);
	input_set_drvdata(accel_dev, data);

	ret = input_register_device(accel_dev);
	if (ret < 0) {
		input_free_device(accel_dev);
		return ret;
	}

	ret = sysfs_create_group(&accel_dev->dev.kobj, &accel_attribute_group);
	if (ret < 0) {
		sensors_remove_symlink(&accel_dev->dev.kobj, accel_dev->name);
		input_unregister_device(accel_dev);
		return ret;
	}

	data->accel_input = accel_dev;


	/* allocate gyro sensor input_device */
	gyro_dev = input_allocate_device();
	if (!gyro_dev)
		return -ENOMEM;

	gyro_dev->name = "gyro_sensor";
	gyro_dev->id.bustype = BUS_I2C;

	input_set_capability(gyro_dev, EV_REL, REL_RX);
	input_set_capability(gyro_dev, EV_REL, REL_RY);
	input_set_capability(gyro_dev, EV_REL, REL_RZ);
	input_set_drvdata(gyro_dev, data);

	ret = input_register_device(gyro_dev);
	if (ret < 0) {
		input_free_device(gyro_dev);
		return ret;
	}

	ret = sysfs_create_group(&gyro_dev->dev.kobj, &gyro_attribute_group);
	if (ret < 0) {
		sensors_remove_symlink(&gyro_dev->dev.kobj, gyro_dev->name);
		input_unregister_device(gyro_dev);
		return ret;
	}

	data->gyro_input = gyro_dev;
	return 0;
}

static void k6ds3_work_func_acc(struct work_struct *work)
{
	struct k6ds3_data *data =
		container_of((struct delayed_work *)work,
			struct k6ds3_data, accel_work);

	struct k6ds3_acc acc_raw_data;
	int result;

	result = k6ds3_input_read_accel_raw_data(data, &acc_raw_data);
	if (result<0){
		pr_err("[SENSOR]: %s - read data failed\n", __func__);
	}

	input_report_rel(data->accel_input, REL_X, acc_raw_data.x);
	input_report_rel(data->accel_input, REL_Y, acc_raw_data.y);
	input_report_rel(data->accel_input, REL_Z, acc_raw_data.z);

	input_sync(data->accel_input);

	if (atomic_read(&data->accel_delay) < 60) {
		usleep_range(atomic_read(&data->accel_delay) * 1000,
			atomic_read(&data->accel_delay) * 1100);
		schedule_delayed_work(&data->accel_work, 0);
	} else {
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(
			atomic_read(&data->accel_delay)));
	}
}

static void k6ds3_work_func_gyro(struct work_struct *work)
{
	struct k6ds3_data *data =
		container_of((struct delayed_work *)work,
			struct k6ds3_data, gyro_work);

	int result;

	result = k6ds3_input_read_gyro_raw_data(data);
	if (result<0){
		pr_err("[SENSOR]: %s - read data failed\n", __func__);
	}

	if (atomic_read(&data->gyro_delay) < 60) {
		usleep_range(atomic_read(&data->gyro_delay) * 1000,
			atomic_read(&data->gyro_delay) * 1100);
		schedule_delayed_work(&data->gyro_work, 0);
	} else {
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(
			atomic_read(&data->gyro_delay)));
	}
}

static irqreturn_t k6ds3_irq_thread(int irq, void *dev)
{
	int err;
	struct k6ds3_data *data = (struct k6ds3_data *)dev;
	unsigned long timediff = 0;

	timediff = jiffies_to_msecs(timediff -
		data->motion_recg_st_time);
	if (timediff < 150 &&
		(data->movent_recog_flag != REACTIVE_FACTORY)) {
		pr_err("[SENSOR] %s, timediff = %ld msec",__func__, timediff);
		goto done;
	}

	if (data->movent_recog_flag == REACTIVE_FACTORY){
		atomic_set(&data->interrupt_state, true);
		pr_info("[SENSOR] %s, motion interrupt is happened\n", __func__);
	}

	err = k6ds3_set_range_accel(data, ACC_RANGE_8G);
	if (err < 0)
		goto done;

	wake_lock_timeout(&data->reactive_wake_lock, 5 * HZ);
done:
	return IRQ_HANDLED;
}

static int k6ds3_init_pin(struct k6ds3_data *data)
{
	int ret=0;
	ret = gpio_request(data->acc_int, "acc_int");
	if (ret<0){
		pr_err("SENSOR]:  failed to request gpio%s\n",__func__);
		goto exit;
	}

	s3c_gpio_cfgpin(data->acc_int, S3C_GPIO_INPUT);
	s3c_gpio_setpull(data->acc_int, S3C_GPIO_PULL_NONE);

	s5p_register_gpio_interrupt(data->acc_int);
	data->irq_acc = gpio_to_irq(data->acc_int);

	ret = request_threaded_irq(data->irq_acc, NULL,
			k6ds3_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"k6ds3_int", data);
	if (ret<0){
		pr_err("SENSOR]:  failed to request interrupt%s\n",__func__);
		goto exit_int_gpio;
	}
	disable_irq(data->irq_acc);

exit_int_gpio:
	gpio_free(data->acc_int);
exit:
	return ret;

}

static void k6ds3_int_var(struct k6ds3_data *data){
	atomic_set(&data->accel_enabled, 0);
	atomic_set(&data->gyro_enabled,0);
	atomic_set(&data->accel_delay, 100);
	atomic_set(&data->gyro_delay, 100);
	data->accel_data_rate = ACC_208HZ;
	data->gyro_data_rate = GYRO_208HZ;

	atomic_set(&data->interrupt_state, 0);
	data->movent_recog_flag = REACTIVE_OFF;
	data->dyamic_threshold[0] = DYNAMIC_THRESHOLD;
	data->dyamic_threshold[1] = DYNAMIC_THRESHOLD;
	wake_lock_init(&data->reactive_wake_lock, WAKE_LOCK_SUSPEND,
		"reactive_wake_lock");
	mutex_init(&data->mutex);
}

static int k6ds3_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct k6ds3_data *data = NULL;
	struct k6ds3_platform_data *p_data;
	unsigned char whoami = 0;

	pr_info("[SENSOR]: %s - Probe Start!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[SENSOR]: %s - i2c_check_functionality error\n",
			__func__);
		goto exit;
	}

	data = kzalloc(sizeof(struct k6ds3_data), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[SENSOR]: %s - kzalloc error\n", __func__);
		ret = -ENOMEM;
		goto exit_kzalloc;
	}

	data->i2c_client = client;
	i2c_set_clientdata(client, data);

	ret = k6ds3_reset_device(data);
	if (ret<0){
		pr_err("[SENSOR]: %s - reset device is fail\n", __func__);
		goto reset_failed;
	}

	ret = k6ds3_i2c_read(client, WHO_AM_I, 1, &whoami);
	if (ret<0){
		pr_err("[SENSOR]: %s - read i2c is failed\n", __func__);
		goto i2c_read_failed;
	} else{
		if (whoami != I_AM_K6DS3TR){
			pr_err("[SENSOR]: %s - this isn't K6DS3TR, whoami = %d\n", __func__, whoami);
			ret = -1;
			goto is_not_k6ds3;
		}
		pr_err("[SENSOR]: %s - whoami = %d\n", __func__, whoami);
	}

	/* get platform data */
	p_data = client->dev.platform_data;
	if (p_data){
		data->position = p_data->position;
		data->acc_int = p_data->irq_gpio;
		data->int_selection = p_data->int_selection;
		pr_info("[SENSOR]:%s - position = %d, acc_int = %d,  int_selection = %d\n",
				__func__,data->position, data->acc_int, data->int_selection);
	} else {
		data->position = 3;
		data->int_selection = 1;
	}

	k6ds3_int_var(data);

	ret = k6sd3_init_accel_sensor(data);
	if (ret<0){
		pr_err("[SENSOR]: %s - init accel sensor fail\n", __func__);
		goto init_accel_failed;
	}

	ret = k6ds3_init_gyro_sensor(data);
	if (ret<0){
		pr_err("[SENSOR]: %s - init gyro sensor fail\n", __func__);
		goto init_gyro_failed;
	}

	ret = k6ds3_input_init(data);
	if (ret < 0)
		goto exit_input_init;

	ret = sensors_register(data->accel_sensor_device,
		data, accel_sensor_attrs,
			"accelerometer_sensor");
	if (ret) {
		pr_err("[SENSOR]: %s - could not register accelerometer sensor device(%d)\n", __func__, ret);
		goto acc_sensor_register_failed;
	}

	ret = sensors_register(data->gyro_sensor_device,
				data, gyro_sensor_attrs, "gyro_sensor");
	if (ret) {
		pr_err("[SENSOR]: %s - could not register gyro sensor device(%d)\n", __func__, ret);
		goto gyro_sensor_register_failed;
	}

	INIT_DELAYED_WORK(&data->accel_work, k6ds3_work_func_acc);
	INIT_DELAYED_WORK(&data->gyro_work, k6ds3_work_func_gyro);

	k6ds3_init_pin(data);

	device_init_wakeup(&data->i2c_client->dev, 1);
	pr_info("[SENSOR]: %s - probe done", __func__);

	return 0;

exit_input_init:
reset_failed:
acc_sensor_register_failed:
gyro_sensor_register_failed:
i2c_read_failed:
is_not_k6ds3:
init_accel_failed:
init_gyro_failed:
	kfree(data);
exit_kzalloc:
exit:
	pr_err("[SENSOR]: %s - Probe fail!\n", __func__);
	return ret;
}

static void k6ds3_shutdown(struct i2c_client *client)
{

}

static int __devexit k6ds3_remove(struct i2c_client *client)
{
	struct k6ds3_data *data = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&data->accel_work);
	cancel_delayed_work_sync(&data->gyro_work);

	/*sensor accel + gyro: power off mode*/
	k6sd3_poweronoff_accel_sensor(data, false);
	k6sd3_poweronoff_gyro_sensor (data, false);

	sensors_unregister(data->accel_sensor_device, accel_sensor_attrs);
	sensors_remove_symlink(&data->accel_input->dev.kobj, data->accel_input->name);
	sysfs_remove_group(&data->accel_input->dev.kobj, &accel_attribute_group);

	sensors_unregister(data->gyro_sensor_device, gyro_sensor_attrs);
	sensors_remove_symlink(&data->gyro_input->dev.kobj, data->gyro_input->name);
	sysfs_remove_group(&data->gyro_input->dev.kobj, &gyro_attribute_group);

	kfree(data);
	return 0;
}


#ifdef CONFIG_OF
static struct of_device_id k6ds3_match_table[] = {
	{ .compatible = "stm,k6ds3",},
	{ },
};
#else
#define k6ds3_match_table	NULL
#endif

static int k6ds3_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct k6ds3_data *data = i2c_get_clientdata(client);

	/*sensor accel + gyro: power off mode*/
	k6sd3_poweronoff_accel_sensor(data, false);
	k6sd3_poweronoff_gyro_sensor (data, false);
	return 0;
}

static int k6ds3_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct k6ds3_data *data = i2c_get_clientdata(client);

	/*sensor accel + gyro: power on mode*/
	k6sd3_poweronoff_accel_sensor(data, true);
	k6sd3_poweronoff_gyro_sensor (data, true);
	return 0;
}

static const struct i2c_device_id k6ds3_device_id[] = {
	{ "k6ds3", 0 },
	{ }
};

static const struct dev_pm_ops k6ds3_pm_ops = {
	.suspend = k6ds3_suspend,
	.resume = k6ds3_resume
};

static struct i2c_driver k6ds3_i2c_driver = {
	.driver = {
		.name = "k6ds3",
		.owner = THIS_MODULE,
		.pm = &k6ds3_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(k6ds3_match_table),
#endif
	},
	.probe = k6ds3_probe,
	.shutdown = k6ds3_shutdown,
	.remove = __devexit_p(k6ds3_remove),
	.id_table = k6ds3_device_id,
};

static int __init k6ds3_init(void)
{
	return i2c_add_driver(&k6ds3_i2c_driver);
}

static void __exit k6ds3_exit(void)
{
	i2c_del_driver(&k6ds3_i2c_driver);
}

module_init(k6ds3_init);
module_exit(k6ds3_exit);

MODULE_DESCRIPTION("K6DS3TR SENSOR");
MODULE_AUTHOR("vancuong.hg@samsung.com");
MODULE_LICENSE("GPL");
