/*
 *
 * Zinitix ztw522 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#include <linux/pinctrl/consumer.h>
#include <linux/i2c/zxt_ztw522_ts.h>
#ifdef CONFIG_SEC_SYSFS
#include <linux/sec_sysfs.h>
#endif
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif
#include <linux/trm.h>

static const char *power_control_string[POWER_ON_SEQUENCE+1] = {
	"POWER_OFF",
	"POWER_ON",
	"POWER_ON_SEQUENCE"
};
#if ZINITIX_MISC_DEBUG
static struct ztw522_ts_info *misc_info;
#endif

#ifndef CONFIG_SEC_SYSFS
extern struct class *sec_class;
#endif

#ifdef CONFIG_SLEEP_MONITOR
#define	PRETTY_MAX	14
#define	STATE_BIT	24
#define	CNT_MARK	0xffff
#define	STATE_MARK	0xff
#endif

#define	TC_SECTOR_SZ		8
#define	I2C_RESUME_DELAY	2000
#define	WAKELOCK_TIME		200

static bool ztw522_power_control(struct ztw522_ts_info *info, u8 ctl);
int ztw522_power(struct i2c_client *client, int on);
static bool ztw522_init_touch(struct ztw522_ts_info *info, bool forced);
static bool ztw522_mini_init_touch(struct ztw522_ts_info *info);
static void ztw522_clear_report_data(struct ztw522_ts_info *info);
#if 0
static int ztw522_pinctrl_configure(struct ztw522_ts_info *info, bool active);
#endif
#ifdef SEC_FACTORY_TEST
static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);

/* Vendor dependant command */
static void run_dnd_read(void *device_data);
static void get_dnd(void *device_data);
static void get_dnd_all_data(void *device_data);
static void run_dnd_v_gap_read(void *device_data);
static void get_dnd_v_gap(void * device_data);
static void run_dnd_h_gap_read(void *device_data);
static void get_dnd_h_gap(void * device_data);
static void run_hfdnd_read(void *device_data);
static void get_hfdnd(void * device_data);
static void get_hfdnd_all_data(void *device_data);
static void run_hfdnd_v_gap_read(void *device_data);
static void get_hfdnd_v_gap(void * device_data);
static void run_hfdnd_h_gap_read(void *device_data);
static void get_hfdnd_h_gap(void * device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);
static void run_reference_read(void * device_data);
static void get_reference(void *device_data);
static void get_delta_all_data(void *device_data);
static void dead_zone_enable(void *device_data);
static void clear_cover_mode(void *device_data);
static void clear_reference_data(void *device_data);
static void run_ref_calibration(void *device_data);
static void run_connect_test(void *device_data);
#endif

static int ztw522_upgrade_sequence(struct ztw522_ts_info *info, const u8 *firmware_data);
static bool ztw522_hw_calibration(struct ztw522_ts_info *info);
static s32 ztw522_write_reg(struct i2c_client *client, u16 reg, u16 value);
static s32 ztw522_write_cmd(struct i2c_client *client, u16 reg);
static bool ztw522_set_touchmode(struct ztw522_ts_info *info, u16 value);
static s32 ztw522_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length);

#ifdef CONFIG_SLEEP_MONITOR
static int ztw522_get_sleep_monitor_cb(void* priv, unsigned int *raw_val, int check_level, int caller_type);
static struct sleep_monitor_ops  ztw522_sleep_monitor_ops = {
	 .read_cb_func =  ztw522_get_sleep_monitor_cb,
};

static int ztw522_get_sleep_monitor_cb(void* priv, unsigned int *raw_val, int check_level, int caller_type)
{
	struct ztw522_ts_info *info = priv;
	struct i2c_client *client = info->client;
	int state = DEVICE_UNKNOWN;
	int pretty = 0;

#if defined(CYTEST_GETPOWERMODE)
	int tsp_mode;
#endif

	*raw_val = -1;

	if (check_level == SLEEP_MONITOR_CHECK_SOFT) {
		if (info->device_enabled)
			state = DEVICE_ON_ACTIVE1;
		else
			state = DEVICE_POWER_OFF;
		*raw_val = state;
	} else if (check_level == SLEEP_MONITOR_CHECK_HARD) {
		if (!info->device_enabled) {
			state = DEVICE_POWER_OFF;
			goto out;
		}

#if defined(CYTEST_GETPOWERMODE)
		tsp_mode = ztw522_test_get_power_mode(info->dev);
		if (tsp_mode < 0) {
			dev_err(info->dev, "%s:get failed chip power mode.[%d]\n",
				__func__, tsp_mode);
			goto out;
		}

		if (tsp_mode == PWRMODE_ACTIVE)
			state = DEVICE_ON_ACTIVE2;
		else if (tsp_mode == PWRMODE_LOOKFORTOUCH)
			state = DEVICE_ON_ACTIVE1;
		else if (tsp_mode == PWRMODE_LOWPOWER)
			state = DEVICE_ON_LOW_POWER;
		else
			state = DEVICE_ERR_1;
#else
		if (info->device_enabled)
			state = DEVICE_ON_ACTIVE1;
		else
			state = DEVICE_POWER_OFF;
#endif
	}

out:
	*raw_val = ((state & STATE_MARK) << STATE_BIT) |\
			(info->release_cnt & CNT_MARK);

	if (info->release_cnt > PRETTY_MAX)
		pretty = PRETTY_MAX;
	else
		pretty = info->release_cnt;

	info->release_cnt = 0;

	dev_dbg(&client->dev, "%s: raw_val[0x%08x], check_level[%d], state[%d], pretty[%d]\n",
		__func__, *raw_val, check_level, state, pretty);

	return pretty;
}
#endif

static void zinitix_delay(unsigned int ms)
{
	if (ms <= 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

static s32 ztw522_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
retry:
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
			dev_err(&client->dev, "%s: failed to send. reg=0x%04x, ret:%d, try:%d\n",
					__func__, reg, ret, count + 1);
			zinitix_delay(I2C_TX_DELAY);
			if (++count < I2C_RETRY_TIMES)
				goto retry;
			return ret;
		}

	usleep_range(DELAY_FOR_TRANSCATION, DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to recv. ret:%d\n", __func__, ret);
		return ret;
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
}

#if TOUCH_POINT_MODE
static s32 ztw522_read_data_only(struct i2c_client *client, u8 *values, u16 length)
{
	s32 ret;

	ret = i2c_master_recv(client , values , length);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to recv. ret:%d\n", __func__, ret);
		return ret;
	}
	usleep_range(DELAY_FOR_TRANSCATION, DELAY_FOR_TRANSCATION);
	return length;
}
#endif

static inline s32 ztw522_write_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
	u8 pkt[10];

	pkt[0] = (reg) & 0xff;
	pkt[1] = (reg >> 8) & 0xff;
	memcpy((u8 *)&pkt[2], values, length);

retry:
	ret = i2c_master_send(client , pkt , length + 2);
	if (ret < 0) {
		zinitix_delay(I2C_TX_DELAY);
		if (++count < I2C_RETRY_TIMES) {
			goto retry;
		} else {
			dev_err(&client->dev, "%s: failed to send. reg= 0x%04x, data= 0x%02x, ret:%d, try:%d\n",
						__func__, reg, *values, ret, count + 1);
			return ret;
		}
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
}

static s32 ztw522_write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (ztw522_write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static s32 ztw522_write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;
	int count = 0;

retry:
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		zinitix_delay(I2C_TX_DELAY);
		if (++count < I2C_RETRY_TIMES) {
			goto retry;
		} else {
			dev_err(&client->dev,
				"%s: failed to send reg 0x%04x. ret:%d\n",
				__func__, reg, ret);
			return ret;
		}
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return I2C_SUCCESS;
}

static inline s32 ztw522_read_raw_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;

	ret = ztw522_write_cmd(client, reg);
	if (ret  != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to send reg 0x%04x.\n", __func__, reg);
		return ret;
	}

	ret = i2c_master_recv(client , values , length);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to recv. ret:%d\n", __func__, ret);
		return ret;
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
}

static s32 ztw522_send_start_event(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret;

	ret = ztw522_write_cmd(client,ztw522_I2C_START_CMD);

	usleep_range(I2C_START_DELAY,I2C_START_DELAY);

	if (ret == I2C_SUCCESS)
		return 0;
	else
		return -1;
}

static void ztw522_set_optional_mode(struct ztw522_ts_info *info, bool force)
{
	static  u32 m_prev_optional_mode = 0;

	if (!info->device_enabled)
		return;
	if (m_prev_optional_mode == info->optional_mode && !force)
		return;

	if (ztw522_write_reg(info->client, ztw522_OPTIONAL_SETTING, info->optional_mode) == I2C_SUCCESS) {
		m_prev_optional_mode = info->optional_mode;
		dev_info(&info->client->dev, "%s: 0x%04x\n",
						__func__, info->optional_mode);
	}
}
static void ztw522_cover_set(struct ztw522_ts_info *info)
{
	if (info->cover_state == COVER_OPEN) {
		zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_SVIEW_DETECT_BIT);
	} else if (info->cover_state == COVER_CLOSED) {
		zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_SVIEW_DETECT_BIT);
	}

	if (info->work_state == SUSPEND || info->work_state == EALRY_SUSPEND || info->work_state == PROBE)
		return;

	ztw522_set_optional_mode(info, true);
}

#if USE_TSP_TA_CALLBACKS
static void ztw522_set_ta_status(struct ztw522_ts_info *info)
{
	if (info->ta_connected)
		zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_USB_DETECT_BIT);
	else
		zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_USB_DETECT_BIT);

	ztw522_set_optional_mode(info, false);
}

static void ztw522_ts_charger_status_cb(struct tsp_callbacks *cb, int status)
{
	struct ztw522_ts_info *info =
		container_of(cb, struct ztw522_ts_info, callbacks);
	struct i2c_client *client = info->client;

	info->ta_connected = !!status;

	if (info->device_enabled) {
		ztw522_send_start_event(info);
		ztw522_set_ta_status(info);
		ztw522_write_cmd(client, ztw522_I2C_END_CMD);
	}

	dev_info(&info->client->dev, "%s: TA %s\n", __func__,
			info->ta_connected ? "connected" : "disconnected");
}

extern struct tsp_callbacks *ztw522_charger_callbacks;
static void ztw522_register_callback(struct tsp_callbacks *cb)
{
	ztw522_charger_callbacks = cb;
	pr_info("%s\n", __func__);
}
#endif

static bool get_raw_data(struct ztw522_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct zxt_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	int i;
	u32 temp_sz;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: other process occupied. (%d)\n",
			__func__, info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);
		return false;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			zinitix_delay(GPIO_GET_DELAY);

		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		zinitix_delay(1);
	}

	sz = total_node * 2;

	while (gpio_get_value(pdata->gpio_int))
		zinitix_delay(1);

	for (i = 0; sz > 0; i++) {
		temp_sz = I2C_BUFFER_SIZE;

		if (sz < I2C_BUFFER_SIZE)
			temp_sz = sz;
		if (ztw522_read_raw_data(client, ztw522_RAWDATA_REG + i,
			(char *)(buff + (i * I2C_BUFFER_SIZE)), temp_sz) < 0) {

			dev_err(&info->client->dev, "%s: read zinitix tc raw data\n", __func__);
			info->work_state = NOTHING;
			enable_irq(info->irq);
			up(&info->work_lock);
			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return true;
}

static bool ztw522_get_raw_data(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	u16 temp_sz;
	int i;

	if (down_trylock(&info->raw_data_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy sema\n", __func__);
		info->touch_info.status = 0;
		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);

	for (i = 0; sz > 0; i++) {
		temp_sz = I2C_BUFFER_SIZE;

		if (sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (ztw522_read_raw_data(info->client, ztw522_RAWDATA_REG + i,
			(char *)((u8*)(info->cur_data)+ (i * I2C_BUFFER_SIZE)), temp_sz) < 0) {

			dev_err(&client->dev, "%s: Failed to read raw data\n", __func__);
			up(&info->raw_data_lock);
			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	info->update = 1;
	memcpy((u8 *)(&info->touch_info),
			(u8 *)&info->cur_data[total_node], sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

#if (TOUCH_POINT_MODE == 0)
#if ZINITIX_I2C_CHECKSUM
#define ZINITIX_I2C_CHECKSUM_WCNT 0x016a
#define ZINITIX_I2C_CHECKSUM_RESULT 0x016c
static bool ztw522_i2c_checksum(struct ztw522_ts_info *info, s16 *pChecksum, u16 wlength)
{
	s16 checksum_result;
	s16 checksum_cur;
	int i;

	checksum_cur = 0;
	for (i = 0; i < wlength; i++)
		checksum_cur += (s16)pChecksum[i];

	if (ztw522_read_data(info->client, ZINITIX_I2C_CHECKSUM_RESULT, (u8 *)(&checksum_result), 2) < 0) {
		dev_err(&info->client->dev, "%s: error read i2c checksum rsult.\n", __func__);
		return false;
	}
	if (checksum_cur != checksum_result) {
		dev_err(&info->client->dev, "%s: checksum error : %d, %d\n",
				__func__, checksum_cur, checksum_result);
		return false;
	}
	return true;
}

#endif
#endif

static bool ztw522_clear_interrupt(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;

	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO)) {
		dev_err(&client->dev, "%s: Invalid must zero bit(%04x)\n", __func__, info->touch_info.status);
		return -1;
	}

	if (ztw522_write_cmd(info->client, ztw522_CLEAR_INT_STATUS_CMD)) {
		dev_err(&client->dev, "%s: failed to write ztw522_CLEAR_INT_STATUS_CMD\n", __func__);
		return -1;
	}

	return 0;
}

static bool ztw522_ts_read_coord(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	u16 status;
#if TOUCH_POINT_MODE
	int i;
#endif

	/* for  Debugging Tool */
	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (info->update == 0) {
			if (!ztw522_get_raw_data(info))
				return false;
		} else
			info->touch_info.status = 0;
		dev_err(&client->dev, "%s: status = 0x%04X\n", __func__, info->touch_info.status);
		goto out;
	}

#if TOUCH_POINT_MODE
	memset(&info->touch_info, 0x0, sizeof(struct point_info));

	if (ztw522_read_data_only(info->client, (u8 *)(&info->touch_info), 10) < 0) {
		dev_err(&client->dev, "%s: error read point info using i2c.-\n", __func__);
		return false;
	}

	if (info->touch_info.event_flag == 0 || info->touch_info.status == 0) {
		ztw522_set_optional_mode(info, false);
		if (ztw522_read_data(info->client, ztw522_OPTIONAL_SETTING, (u8 *)&status, 2) < 0) {
			dev_err(&client->dev, "%s: error read noise mode.-\n", __func__);
			return false;
		}

		ztw522_write_cmd(info->client, ztw522_CLEAR_INT_STATUS_CMD);
		return true;
	}

	for (i = 1; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			usleep_range(20, 20);
			if (ztw522_read_data(info->client, ztw522_POINT_STATUS_REG + 2 + (i * 4),
				(u8 *)(&info->touch_info.coord[i]), sizeof(struct coord)) < 0) {
				dev_err(&client->dev, "%s: error read point info\n", __func__);
				return false;
			}
		}
	}

#else   /* TOUCH_POINT_MODE */
#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum) {
		if (ztw522_write_reg(info->client, ZINITIX_I2C_CHECKSUM_WCNT,
					(sizeof(struct point_info)/2)) != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: error write checksum wcnt.-\n", __func__);
			return false;
		}
	}
#endif
	if (ztw522_read_data(info->client, ztw522_POINT_STATUS_REG,
			(u8 *)(&info->touch_info), sizeof(struct point_info)) < 0) {
		dev_err(&client->dev, "%s: Failed to read point info\n", __func__);
		return false;
	}
#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum) {
		if (ztw522_i2c_checksum(info, (s16 *)(&info->touch_info),
					sizeof(struct point_info)/2) == false)
			return false;
	}
#endif
#endif	/* TOUCH_POINT_MODE */
	ztw522_set_optional_mode(info, false);

	if(zinitix_bit_test(info->touch_info.status, BIT_DEBUG)) {
		if (ztw522_read_data(info->client, ZTW522_DEBUG_00, (u8 *)&status, 2) < 0) {
			dev_err(&client->dev, "%s: error read lpm mode\n", __func__);
			return false;
		}
		info->lpm_mode = zinitix_bit_test(status, BIT_DEVICE_STATUS_LPM);
	}

	return true;

out:
	if (ztw522_clear_interrupt(info)) {
		dev_err(&client->dev, "%s: failed clear interrupt.\n", __func__);
		return false;
	}

	return true;
}

static bool ztw522_power_sequence(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;
	u16 chip_code;

retry_power_sequence:
	if (ztw522_write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to send power sequence(vendor cmd enable)\n", __func__);
		goto fail_power_sequence;
	}
	usleep_range(10, 10);

	if (ztw522_read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		dev_err(&client->dev, "%s: Failed to read chip code\n", __func__);
		goto fail_power_sequence;
	}

	dev_dbg(&client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);
	usleep_range(10, 10);

	if (ztw522_write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to send power sequence(intn clear)\n", __func__);
		goto fail_power_sequence;
	}
	usleep_range(10, 10);

	if (ztw522_write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to send power sequence(nvm init)\n", __func__);
		goto fail_power_sequence;
	}
	zinitix_delay(2);

	if (ztw522_write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to send power sequence(program start)\n", __func__);
		goto fail_power_sequence;
	}

	zinitix_delay(FIRMWARE_ON_DELAY);	/* wait for checksum cal */

	return true;

fail_power_sequence:
	if (retry++ < 3) {
		zinitix_delay(CHIP_ON_DELAY);
		dev_err(&client->dev, "%s: retry = %d\n", __func__, retry);
		goto retry_power_sequence;
	}

	dev_err(&client->dev, "%s: Failed to send power sequence\n", __func__);
	return false;
}

int ztw522_power(struct i2c_client *client, int on)
{
	int ret;
	struct ztw522_ts_info *info = i2c_get_clientdata(client);
	struct zxt_ts_platform_data *pdata = info->pdata;
	static struct regulator *vddo = NULL;
	static struct regulator *avdd = NULL;
	static bool reg_boot_on = true;

	if (on == POWER_ON_SEQUENCE)
		on = POWER_ON;

	if (!pdata) {
		dev_err(&client->dev, "%s: pdata is NULL \n", __func__);
		return -ENODEV;
	}

	if (reg_boot_on && !pdata->reg_boot_on)
		reg_boot_on = false;

	if (pdata->vdd_en) {
		ret = gpio_direction_output(pdata->vdd_en, on);
		if (ret) {
			dev_err(&client->dev, "%s: unable to set_direction for zt_vdd_en [%d]\n",
				__func__, pdata->vdd_en);
			return -EINVAL;
		}
	} else {
		if (!avdd) {
			avdd = regulator_get(&info->client->dev, "tsp_avdd_3.0v");
			if (IS_ERR_OR_NULL(avdd)) {
				dev_err(&client->dev, "%s: could not get tsp_avdd_3.0v, ret = %ld\n",
					__func__, IS_ERR(avdd));
				return -EINVAL;
			} else
				dev_info(&client->dev, "%s: success register tsp_avdd_3.0v\n", __func__);

			ret = regulator_set_voltage(avdd, 3000000, 3000000);
			if (ret) {
				dev_err(&client->dev, "%s: could not set voltage tsp_avdd_3.0v, ret = %d\n",
					__func__, ret);
			} else
				dev_info(&client->dev, "%s: success set_voltage tsp_avdd_3.0v\n", __func__);
		}

		if (on) {
			ret = regulator_enable(avdd);
			if (ret) {
				dev_err(&client->dev, "%s: tsp_avdd_3.0v enable failed (%d)\n", __func__, ret);
				return -EINVAL;
			} else
				dev_dbg(&client->dev, "%s: success enable tsp_avdd_3.0v\n", __func__);
		} else {
			ret = regulator_disable(avdd);
			if (ret) {
				dev_err(&client->dev, "%s: tsp_avdd_3.0v disable failed (%d)\n", __func__, ret);
				return -EINVAL;
			} else
				dev_dbg(&client->dev, "%s: success disable tsp_avdd_3.0v\n", __func__);
		}
	}

	if (!vddo) {
		vddo = regulator_get(&info->client->dev, "tsp_vddo_1.8v");
		if (IS_ERR_OR_NULL(vddo)) {
			dev_err(&client->dev, "%s: could not get tsp_vddo_1.8v, ret = %ld\n",
				__func__, IS_ERR(vddo));
			return -EINVAL;
		} else
			dev_info(&client->dev, "%s: success register tsp_vddo_1.8v\n", __func__);

		ret = regulator_set_voltage(vddo, 1800000, 1800000);
		if (ret) {
			dev_err(&client->dev, "%s: could not set voltage tsp_vddo_1.8v, ret = %d\n",
				__func__, ret);
		} else
			dev_info(&client->dev, "%s: success set_voltage tsp_vddo_1.8v\n", __func__);
	}

	if (on) {
		ret = regulator_enable(vddo);
		if (ret) {
			dev_err(&client->dev, "%s: tsp_vddo_1.8v enable failed (%d)\n", __func__, ret);
			return -EINVAL;
		} else
			dev_dbg(&client->dev, "%s: success enable tsp_vddo_1.8v\n", __func__);
	} else {
		ret = regulator_disable(vddo);
		if (ret) {
			dev_err(&client->dev, "%s: tsp_vddo_1.8v disable failed (%d)\n", __func__, ret);
			return -EINVAL;
		} else
			dev_dbg(&client->dev, "%s: success disable tsp_vddo_1.8v\n", __func__);
	}

	dev_dbg(&client->dev, "%s: %s", __func__, on ? "ON":"OFF");

	if (pdata->vdd_en)
		pr_cont("%s: vdd_en %s, ", __func__, gpio_get_value(pdata->vdd_en) ? "ON":"OFF");
	else if (!IS_ERR_OR_NULL(avdd))
		pr_cont("%s: tsp_avdd_3.0v: %s, ",  __func__, regulator_is_enabled(avdd) ? "ON":"OFF");

	if (!IS_ERR_OR_NULL(vddo))
		pr_cont("tsp_vddo_1.8v %s\n", regulator_is_enabled(vddo) ? "ON":"OFF");

	if (on >= POWER_ON) {
		if (!reg_boot_on)
			zinitix_delay(CHIP_ON_DELAY);
		reg_boot_on = false;
	} else
		zinitix_delay(CHIP_OFF_DELAY);

	return 0;
}

static bool ztw522_power_control(struct ztw522_ts_info *info, u8 ctl)
{
	struct zxt_ts_platform_data *pdata = info->pdata;
	int ret;

	dev_info(&info->client->dev, "%s: %s\n", __func__, power_control_string[ctl]);

	ret = info->pdata->tsp_power(info->client, ctl);
	if (ret) {
		dev_err(&info->client->dev, "%s: Failed to control power\n", __func__);
		return false;
	}

	msleep(POWER_ON_DELAY);

	if (pdata->gpio_config)
		pdata->gpio_config(!!ctl);

	msleep(RESET_DELAY);

	if (ctl == POWER_ON_SEQUENCE)
		return ztw522_power_sequence(info);

	return true;
}

static bool ztw522_check_need_upgrade(struct ztw522_ts_info *info,
	u16 cur_version, u16 cur_minor_version, u16 cur_reg_version, u16 cur_hw_id)
{
	u16	new_version;
	u16	new_minor_version;
	u16	new_reg_version;
#if CHECK_HWID
	u16	new_hw_id;
#endif

	if (!info->fw_data) {
		dev_err(&info->client->dev, "%s: fw_data is NULL\n", __func__);
		return false;
	}

	new_version = (u16) (info->fw_data[52] | (info->fw_data[53]<<8));
	new_minor_version = (u16) (info->fw_data[56] | (info->fw_data[57]<<8));
	new_reg_version = (u16) (info->fw_data[60] | (info->fw_data[61]<<8));

#if CHECK_HWID
	new_hw_id = (u16) (info->fw_data[48] | (info->fw_data[49]<<8));
	dev_info(&info->client->dev, "cur HW_ID = 0x%x, new HW_ID = 0x%x\n",
							cur_hw_id, new_hw_id);
#endif

	dev_info(&info->client->dev, "%s: cur version = 0x%x, new version = 0x%x\n",
							__func__, cur_version, new_version);
	dev_info(&info->client->dev, "%s: cur minor version = 0x%x, new minor version = 0x%x\n",
						__func__, cur_minor_version, new_minor_version);
	dev_info(&info->client->dev, "%s: cur reg data version = 0x%x, new reg data version = 0x%x\n",
						__func__, cur_reg_version, new_reg_version);
	if (info->cal_mode) {
		dev_info(&info->client->dev, "%s: didn't update TSP F/W in CAL MODE\n", __func__);
		return false;
	}

	if (cur_reg_version == 0xffff)
		return true;
	if (cur_version > 0xFF)
		return true;
	if (cur_version < new_version)
		return true;
	else if (cur_version > new_version)
		return false;
#if CHECK_HWID
	if (cur_hw_id != new_hw_id)
		return true;
#endif
	if (cur_minor_version < new_minor_version)
		return true;
	else if (cur_minor_version > new_minor_version)
		return false;
	if (cur_reg_version < new_reg_version)
		return true;

	return false;
}

static bool ztw522_upgrade_firmware(struct ztw522_ts_info *info, const u8 *firmware_data, u32 size)
{
	struct i2c_client *client = info->client;
	u32 flash_addr;
	u8 *verify_data;
	int i;
	int page_sz = 64;
	u16 chip_code;

	if (!firmware_data) {
		dev_err(&client->dev, "%s: firmware is NULL\n", __func__);
		return false;
	}

	dev_info(&client->dev, "%s\n", __func__);

	verify_data = devm_kzalloc(&client->dev, size, GFP_KERNEL);
	if (!verify_data) {
		dev_err(&client->dev, "%s: cannot alloc verify buffer\n", __func__);
		return false;
	}

	ztw522_power_control(info, POWER_OFF);
	ztw522_power_control(info, POWER_ON);

	zinitix_delay(10);

	if (ztw522_write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: power sequence error (vendor cmd enable)\n", __func__);
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (ztw522_read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		dev_err(&client->dev, "%s: failed to read chip code\n", __func__);
		goto fail_upgrade;
	}
	dev_info(&client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);

	usleep_range(10, 10);

	if (ztw522_write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: power sequence error (intn clear)\n", __func__);
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (ztw522_write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: power sequence error (nvm init)\n", __func__);
		goto fail_upgrade;
	}

	zinitix_delay(5);

	dev_info(&client->dev, "%s: init flash\n", __func__);

	if (ztw522_write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to write nvm vpp on\n", __func__);
		goto fail_upgrade;
	}

	if (ztw522_write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to write nvm wp disable\n", __func__);
		goto fail_upgrade;
	}

	if (ztw522_write_cmd(client, ztw522_INIT_FLASH) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to init flash\n", __func__);
		goto fail_upgrade;
	}

	// Mass Erase
	if (ztw522_write_cmd(client, 0x01DF) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to mass erase\n", __func__);
		goto fail_upgrade;
	}
	zinitix_delay(100);

	if (ztw522_write_reg(client, 0x01DE, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to enter upgrade mode\n", __func__);
		goto fail_upgrade;
	}

	zinitix_delay(1);

	if (ztw522_write_reg(client, 0x01D3, 0x0008) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to init upgrade mode\n", __func__);
		goto fail_upgrade;
	}

	dev_info(&client->dev, "%s: writing firmware data\n", __func__);
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			if (ztw522_write_data(client, ztw522_WRITE_FLASH,
						(u8 *)&firmware_data[flash_addr], TC_SECTOR_SZ) < 0) {
				dev_err(&client->dev, "%s: write zinitix tc firmare\n", __func__);
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			usleep_range(100, 100);
		}
		zinitix_delay(8);	/*for fuzing delay*/
	}

	if (ztw522_write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: nvm write vpp off\n", __func__);
		goto fail_upgrade;
	}

	if (ztw522_write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: nvm wp enable\n", __func__);
		goto fail_upgrade;
	}

	dev_info(&client->dev, "%s: init flash\n", __func__);
	if (ztw522_write_cmd(client, ztw522_INIT_FLASH) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to init flash\n", __func__);
		goto fail_upgrade;
	}

	dev_info(&client->dev, "%s: read firmware data\n", __func__);
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			if (ztw522_read_raw_data(client, ztw522_READ_FLASH,
						(u8 *)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
				dev_err(&client->dev, "%s: Failed to read firmare\n", __func__);
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
		}
	}

	/* verify */
	dev_info(&client->dev, "%s: verify firmware data\n", __func__);
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		dev_info(&client->dev, "%s: upgrade finished\n", __func__);
		devm_kfree(&client->dev, verify_data);
		ztw522_power_control(info, POWER_OFF);
		ztw522_power_control(info, POWER_ON_SEQUENCE);
		return true;
	} else
		dev_err(&client->dev, "%s: Failed to verify firmare\n", __func__);

fail_upgrade:
	ztw522_power_control(info, POWER_OFF);
	ztw522_power_control(info, POWER_ON);

	devm_kfree(&client->dev, verify_data);

	dev_info(&client->dev, "%s: Failed to upgrade\n", __func__);

	return false;
}

static bool ztw522_hw_calibration(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	u16 chip_eeprom_info;
	int time_out = 0;

	dev_info(&client->dev, "%s\n", __func__);

	if (ztw522_write_reg(client, ztw522_TOUCH_MODE, 0x07) != I2C_SUCCESS)
		return false;
	zinitix_delay(10);
	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
	zinitix_delay(10);
	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
	zinitix_delay(50);
	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
	zinitix_delay(10);

	if (ztw522_write_cmd(client, ztw522_CALIBRATE_CMD) != I2C_SUCCESS)
		return false;

	if (ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;

	zinitix_delay(10);
	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);

	/* wait for h/w calibration*/
	do {
		zinitix_delay(500);
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);

		if (ztw522_read_data(client, ztw522_EEPROM_INFO_REG, (u8 *)&chip_eeprom_info, 2) < 0)
			return false;

		dev_dbg(&client->dev, "touch eeprom info = 0x%04X\n", chip_eeprom_info);
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;

		if (time_out++ == 4) {
			ztw522_write_cmd(client, ztw522_CALIBRATE_CMD);
			zinitix_delay(10);
			ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
			dev_err(&client->dev, "%s: h/w calibration retry timeout.\n", __func__);
		}

		if (time_out++ > 10) {
			dev_err(&client->dev, "%s: h/w calibration timeout.\n", __func__);
			break;
		}

	} while (true);

	if (ztw522_write_reg(client, ztw522_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		return false;

	if (info->cap_info.ic_int_mask) {
		if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG,
					info->cap_info.ic_int_mask) != I2C_SUCCESS)
			return false;
	}

	usleep_range(100, 100);
	if (ztw522_write_cmd(client, ztw522_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
		return false;

	zinitix_delay(1000);

	return true;
}
static int ztw522_get_ic_fw_size(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	int ret = 0;

	if (info->chip_code == ZTW522_IC_CHIP_CODE)
		cap->ic_fw_size = ZTW522_IC_FW_SIZE;
	else {
		dev_err(&client->dev, "%s: Failed to get ic_fw_size\n", __func__);
		ret = -1;
	}

	return ret;
}

static bool ztw522_init_touch(struct ztw522_ts_info *info, bool fw_skip)
{
	struct capa_info *cap_info = &info->cap_info;
	struct zxt_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;
	u16 chip_code = 0;
#if USE_CHECKSUM
	u16 chip_check_sum;
	bool checksum_err;
#endif
	int retry_cnt = 0;
	const struct firmware *tsp_fw = NULL;
	char fw_path[ZINITIX_MAX_FW_PATH];
	int ret;

	info->lpm_mode = 0;

	if (!info->fw_data) {
		snprintf(fw_path, ZINITIX_MAX_FW_PATH, "%s%s", ZINITIX_FW_PATH, info->pdata->fw_name);
		ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
		if (ret) {
			dev_err(&info->client->dev, "%s: failed to request_firmware %s\n",
						__func__, fw_path);
			return false;
		} else {
			info->fw_data = (unsigned char *)tsp_fw->data;
			dev_info(&info->client->dev, "%s: Success to read_firmware %s\n",
						__func__, fw_path);
		}
	}

	info->ref_scale_factor = TSP_INIT_TEST_RATIO;
retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (ztw522_read_data(client, ztw522_EEPROM_INFO_REG,
						(u8 *)&chip_eeprom_info, 2) < 0) {
			dev_err(&client->dev, "%s: Failed to read eeprom info(%d)\n", __func__, i);
			zinitix_delay(10);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	checksum_err = false;
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (ztw522_read_data(client, ztw522_CHECKSUM_RESULT,
						(u8 *)&chip_check_sum, 2) < 0) {
			dev_err(&client->dev, "%s: Failed to read ztw522_CHECKSUM_RESULT[0x%04x]\n",
				__func__, chip_check_sum);
			zinitix_delay(10);
			continue;
		}

		if (chip_check_sum != ZTW522_CHECK_SUM) {
			dev_err(&client->dev, "%s: Failed chip_check_sum in retry_init[0x%04x]\n",
				__func__, chip_check_sum);
			checksum_err = true;
		}
		break;
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		dev_err(&client->dev, "%s: Failed to check firmware data\n", __func__);
		if (checksum_err && retry_cnt < INIT_RETRY_CNT)
			retry_cnt = INIT_RETRY_CNT;
		goto fail_init;
	}
#endif
	if (ztw522_write_cmd(client, ztw522_SWRESET_CMD) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to write reset command\n", __func__);
		goto fail_init;
	}
#ifdef SUPPORTED_TOUCH_KEY
	cap->button_num = SUPPORTED_BUTTON_NUM;
	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);
#endif

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);

#if SUPPORTED_PALM_TOUCH
	zinitix_bit_set(reg_val, BIT_PALM);
	zinitix_bit_set(reg_val, BIT_PALM_REJECT);
#endif

	cap->ic_int_mask = reg_val;

	if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

	dev_dbg(&client->dev, "%s: Send reset command\n", __func__);
	if (ztw522_write_cmd(client, ztw522_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (ztw522_read_data(client, ztw522_VENDOR_ID, (u8 *)&cap->vendor_id, 2) < 0) {
		dev_err(&client->dev, "%s: failed to read vendor id\n", __func__);
		goto fail_init;
	}

	if (ztw522_read_data(client, ztw522_CHIP_REVISION, (u8 *)&cap->ic_revision, 2) < 0) {
		dev_err(&client->dev, "%s: failed to read chip revision\n", __func__);
		goto fail_init;
	}

	if (ztw522_read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		chip_code = 0;
		dev_err(&client->dev, "%s: Failed to read chip code\n", __func__);
		goto fail_init;
	}
	info->chip_code = chip_code;
	dev_info(&client->dev, "%s: chip code = 0x%X\n", __func__, info->chip_code);

	ret = ztw522_get_ic_fw_size(info);
	if (ret) {
		chip_code = 0;
		dev_err(&client->dev, "%s: Failed to get ic_fw_size\n", __func__);
		goto fail_init;
	}

	if (ztw522_read_data(client, ztw522_HW_ID, (u8 *)&cap->hw_id, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_BUTTON_SENSITIVITY, (u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DUMMY_BUTTON_SENSITIVITY, (u8 *)&cap->dummy_threshold, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_TOTAL_NUMBER_OF_X, (u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_TOTAL_NUMBER_OF_Y, (u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;

	if (ztw522_read_data(client, ztw522_DND_CP_CTRL_L, (u8 *)&cap->cp_ctrl_l, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DND_V_FORCE, (u8 *)&cap->v_force, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DND_AMP_V_SEL, (u8 *)&cap->amp_v_sel, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DND_N_COUNT, (u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DND_U_COUNT, (u8 *)&cap->u_cnt, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_AFE_FREQUENCY, (u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;

	/* get chip firmware version */
	if (ztw522_read_data(client, ztw522_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0)
		goto fail_init;

	if (ztw522_read_data(client, ztw522_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0)
		goto fail_init;

	if (!fw_skip && ztw522_check_need_upgrade(info, cap->fw_version,
			cap->fw_minor_version, cap->reg_data_version, cap->hw_id)) {

		dev_info(&client->dev, "%s: start upgrade firmware\n", __func__);

		if (!ztw522_upgrade_firmware(info, info->fw_data, cap->ic_fw_size))
			goto fail_init;

#if USE_CHECKSUM
		if (ztw522_read_data(client, ztw522_CHECKSUM_RESULT, (u8 *)&chip_check_sum, 2) < 0) {
			dev_err(&client->dev, "%s: Failed to read CHECKSUM_RESULT register\n", __func__);
			goto fail_init;
		}

		if (chip_check_sum != ZTW522_CHECK_SUM) {
			dev_err(&client->dev, "%s: Failed chip_check_sum [0x%04x]\n", __func__, chip_check_sum);
			goto fail_init;
		}
#endif
		if (!ztw522_hw_calibration(info))
			goto fail_init;

		/* disable chip interrupt */
		if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;

		/* get chip firmware version */
		if (ztw522_read_data(client, ztw522_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0)
			goto fail_init;

		if (ztw522_read_data(client, ztw522_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0)
			goto fail_init;

		if (ztw522_read_data(client, ztw522_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0)
			goto fail_init;
	}

	if (ztw522_read_data(client, ztw522_EEPROM_INFO_REG, (u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	if (zinitix_bit_test(chip_eeprom_info, 0)) {
		if (!ztw522_hw_calibration(info))
			goto fail_init;

		/* disable chip interrupt */
		if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;
	}

	/* initialize */
	if (ztw522_write_reg(client, ztw522_X_RESOLUTION, (u16)pdata->x_resolution + ABS_PT_OFFSET) != I2C_SUCCESS)
		goto fail_init;

	if (ztw522_write_reg(client, ztw522_Y_RESOLUTION, (u16)pdata->y_resolution + ABS_PT_OFFSET) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32)0;
	cap->MinY = (u32)0;
	cap->MaxX = (u32)pdata->x_resolution;
	cap->MaxY = (u32)pdata->y_resolution;

#ifdef SUPPORTED_TOUCH_KEY
	if (ztw522_write_reg(client, ztw522_BUTTON_SUPPORTED_NUM, (u16)cap->button_num) != I2C_SUCCESS)
		goto fail_init;
#endif

	if (ztw522_write_reg(client, ztw522_SUPPORTED_FINGER_NUM, (u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;
	cap->gesture_support = 0;

	if (ztw522_write_reg(client, ztw522_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (ztw522_write_reg(client, ztw522_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		goto fail_init;

#if ZINITIX_I2C_CHECKSUM
	if (ztw522_read_data(client, ZINITIX_INTERNAL_FLAG_02, (u8 *)&reg_val, 2) < 0) {
		dev_err(&client->dev, "%s: Failed to read INTERNAL_FLAG_02\n", __func__);
		goto fail_init;
	}

	cap->i2s_checksum = !(!zinitix_bit_test(reg_val, 15));
	dev_info(&client->dev, "%s: i2s_checksum = %d, reg_val = 0x%04x\n",
		__func__, cap->i2s_checksum, reg_val);
#endif

#if USE_TSP_TA_CALLBACKS
	ztw522_set_ta_status(info);
#endif
	ztw522_set_optional_mode(info, true);

	if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG, cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (ztw522_write_reg(client, ztw522_DELAY_RAW_FOR_HOST,
					RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to set DELAY_RAW_FOR_HOST\n", __func__);
			goto fail_init;
		}
	}
	if (info->fw_data) {
		release_firmware(tsp_fw);
		info->fw_data = NULL;
	}

	info->gesture_enable = false;

	dev_info(&client->dev, "%s: firmware version: 0x%02x%02x\n", __func__,
		cap_info->fw_minor_version, cap_info->reg_data_version);
	dev_info(&client->dev, "%s: initialize done!\n", __func__);

	return true;

fail_init:
	if (info->cal_mode) {
		dev_err(&client->dev, "%s: didn't update TSP F/W!! in CAL MODE\n", __func__);
		goto retry_fail_init;
	}
	if (++retry_cnt <= INIT_RETRY_CNT) {
		dev_err(&client->dev, "%s: Retry init_touch. retry_cnt=%d/%d\n",
				__func__, retry_cnt, INIT_RETRY_CNT);
		ztw522_power_control(info, POWER_OFF);
		ztw522_power_control(info, POWER_ON_SEQUENCE);
		goto retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT + 1) {
		if (!chip_code) {
			info->chip_code = ZTW522_IC_CHIP_CODE;
			ret = ztw522_get_ic_fw_size(info);
			if (ret) {
				dev_err(&client->dev, "%s: Failed to get ic_fw_size\n", __func__);
				goto retry_fail_init;
			}
		}
		if (!ztw522_upgrade_firmware(info, info->fw_data, cap->ic_fw_size)) {
			dev_err(&client->dev, "%s: firmware upgrade fail!\n", __func__);
			goto retry_fail_init;
		}
		zinitix_delay(100);

		/* hw calibration and make checksum */
		if (!ztw522_hw_calibration(info)) {
			dev_err(&client->dev, "%s: failed to initiallize\n", __func__);
			goto retry_fail_init;
		}
		goto retry_init;
	}

retry_fail_init:
	if (info->fw_data) {
		release_firmware(tsp_fw);
		info->fw_data = NULL;
	}

	return false;
}

static bool ztw522_mini_init_touch(struct ztw522_ts_info *info)
{
	struct capa_info *cap_info = &info->cap_info;
	struct zxt_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
#if ZINITIX_I2C_CHECKSUM
	u16 reg_val;
#endif
#if USE_CHECKSUM
	u16 chip_check_sum;
	if (ztw522_read_data(client, ztw522_CHECKSUM_RESULT, (u8 *)&chip_check_sum, 2) < 0) {
		dev_err(&client->dev, "%s: Failed to read CHECKSUM_RESULT register\n", __func__);
		goto fail_mini_init;
	}

	if (chip_check_sum != ZTW522_CHECK_SUM) {
		dev_err(&client->dev, "%s: Failed chip_check_sum [0x%04x]\n", __func__, chip_check_sum);
		goto fail_mini_init;
	}
#endif
	info->lpm_mode = 0;

	info->ref_scale_factor = TSP_INIT_TEST_RATIO;

	if (ztw522_write_cmd(client, ztw522_SWRESET_CMD) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to write reset command\n", __func__);
		goto fail_mini_init;
	}

	/* initialize */
	if (ztw522_write_reg(client, ztw522_X_RESOLUTION, (u16)(pdata->x_resolution + ABS_PT_OFFSET)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ztw522_write_reg(client, ztw522_Y_RESOLUTION, (u16)(pdata->y_resolution + ABS_PT_OFFSET)) != I2C_SUCCESS)
		goto fail_mini_init;

#ifdef SUPPORTED_TOUCH_KEY
	if (ztw522_write_reg(client, ztw522_BUTTON_SUPPORTED_NUM, (u16)info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;
#endif

	if (ztw522_write_reg(client, ztw522_SUPPORTED_FINGER_NUM, (u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ztw522_write_reg(client, ztw522_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ztw522_write_reg(client, ztw522_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;

#if ZINITIX_I2C_CHECKSUM
	if (cap_info->i2s_checksum) {
		if (ztw522_read_data(client, ZINITIX_INTERNAL_FLAG_02, (u8 *)&reg_val, 2) < 0) {
			dev_err(&client->dev, "%s: Failed to read INTERNAL_FLAG_02\n", __func__);
			goto fail_mini_init;
		}

		cap_info->i2s_checksum = !(!zinitix_bit_test(reg_val, 15));
		dev_info(&client->dev, "%s: i2s_checksum = %d, reg_val = 0x%04x\n",
			__func__, cap_info->i2s_checksum, reg_val);
	}
#endif

#if USE_TSP_TA_CALLBACKS
	ztw522_set_ta_status(info);
#endif
	ztw522_set_optional_mode(info, true);

	/* soft calibration */
	if (ztw522_write_cmd(client, ztw522_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ztw522_write_reg(client, ztw522_INT_ENABLE_FLAG, info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ztw522_write_reg(client, ztw522_DELAY_RAW_FOR_HOST,
					RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS){
			dev_err(&client->dev, "%s: Failed to set ztw522_DELAY_RAW_FOR_HOST\n", __func__);
			goto fail_mini_init;
		}
	}

	info->gesture_enable = false;

	dev_info(&client->dev, "%s: firmware version: 0x%02x%02x\n", __func__,
		cap_info->fw_minor_version, cap_info->reg_data_version);

	dev_info(&client->dev, "%s: Successfully mini initialized\n", __func__);

	return true;

fail_mini_init:
	dev_err(&client->dev, "%s: Failed to initialize mini init\n", __func__);
	ztw522_power_control(info, POWER_OFF);
	ztw522_power_control(info, POWER_ON_SEQUENCE);

	if (!ztw522_init_touch(info, false)) {
		dev_err(&client->dev, "%s: Failed to initialize\n", __func__);
		return false;
	}

	info->gesture_enable = false;

	return true;
}

static void ztw522_clear_report_data(struct ztw522_ts_info *info)
{
	int i;

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < info->cap_info.button_num; i++) {
		if (info->button[i] == ICON_BUTTON_DOWN) {
			info->button[i] = ICON_BUTTON_UP;
			input_report_key(info->input_dev, info->button_code[i], 0);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			dev_info(&info->client->dev, "%s: key %d up\n", __func__, info->button_code[i]);
#else
			dev_info(&info->client->dev, "%s: key up\n", __func__);
#endif
		}
	}
#endif

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		info->move_cnt[i] = 0;
		input_mt_slot(info->input_dev, i);
#ifdef REPORT_2D_Z
		input_report_abs(info->input_dev, ABS_MT_PRESSURE, 0);
#endif
#if SUPPORTED_PALM_TOUCH
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, 0);
		input_report_abs(info->input_dev, ABS_MT_PALM, 0);
#endif
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
		info->reported_touch_info.coord[i].sub_status = 0;
		info->finger_cnt = 0;
	}
#if SUPPORTED_PALM_TOUCH
	info->palm_flag = false;
#endif
	input_sync(info->input_dev);

	touch_booster_release();

	dev_info(&info->client->dev, "%s\n", __func__);
}

static void  ztw522_send_wakeup_event(struct ztw522_ts_info *info)
{
	struct i2c_client *client = info->client;
	struct zxt_ts_platform_data *pdata = info->pdata;

	dev_info(&client->dev, "%s: send wakeup event\n", __func__);

	input_mt_slot(info->input_dev, 0);
	input_mt_report_slot_state(info->input_dev,
					MT_TOOL_FINGER, 1);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X,
				pdata->x_resolution >>1);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y,
				pdata->y_resolution >>1);
	input_report_key(info->input_dev, KEY_WAKEUP, 1);
	input_sync(info->input_dev);

	msleep(20);

	input_mt_report_slot_state(info->input_dev,
					MT_TOOL_FINGER, 0);
	input_report_key(info->input_dev, KEY_WAKEUP, 0);
	input_sync(info->input_dev);
}

static irqreturn_t ztw522_touch_work(int irq, void *data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)data;
	struct zxt_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i = 0, t;
	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
#ifdef REPORT_2D_Z
	u16 z = 0;
	int ret = 0;
#endif
	u8 palm = 0;

	if (gpio_get_value(info->pdata->gpio_int)) {
		dev_err(&client->dev, "%s: Invalid interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		return IRQ_HANDLED;
	}

	if ((info->gesture_enable) && (info->work_state == SUSPEND)) {
		dev_info(&client->dev, "%s: waiting AP resume..\n", __func__);
		wake_lock_timeout(&info->wake_lock, WAKELOCK_TIME);
		t = wait_event_timeout(info->wait_q, info->wait_until_wake,
			msecs_to_jiffies(I2C_RESUME_DELAY));
		if (!t) {
			info->work_state = NOTHING;
			dev_err(&client->dev, "%s: Timed out I2C resume\n", __func__);
		}
	}

	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied\n", __func__);
		usleep_range(DELAY_FOR_SIGNAL_DELAY, DELAY_FOR_SIGNAL_DELAY);
		if (!gpio_get_value(info->pdata->gpio_int)) {
			ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
			usleep_range(DELAY_FOR_SIGNAL_DELAY, DELAY_FOR_SIGNAL_DELAY);
		}
		goto out;
	}
	info->work_state = NORMAL;

	/* Touch Booster en */
	if ((!info->palm_flag) && (!info->finger_cnt))
		touch_booster_press();

#if ZINITIX_I2C_CHECKSUM
	if (!ztw522_ts_read_coord(info) || info->touch_info.status == 0xffff || info->touch_info.status == 0x1) {
		for (i = 1; i < 4; i++) {
			if (!(!ztw522_ts_read_coord(info) || info->touch_info.status == 0xffff
			|| info->touch_info.status == 0x1))
				break;
		}
	}

	if (i == 4) {
		dev_err(&client->dev, "%s: Failed to read info coord\n", __func__);
		ztw522_power_control(info, POWER_OFF);
		ztw522_power_control(info, POWER_ON_SEQUENCE);
		ztw522_clear_report_data(info);
		ztw522_mini_init_touch(info);
		goto out;
	}
#else
	if (!ztw522_ts_read_coord(info) || info->touch_info.status == 0xffff || info->touch_info.status == 0x1) {
		dev_err(&client->dev, "%s: Failed to read info coord\n", __func__);
		ztw522_power_control(info, POWER_OFF);
		ztw522_power_control(info, POWER_ON_SEQUENCE);
		ztw522_clear_report_data(info);
		ztw522_mini_init_touch(info);
		goto out;
	}
#endif
	/* invalid : maybe periodical repeated int. */
	if (info->touch_info.status == 0x0) {
		dev_err(&client->dev, "%s: maybe periodical repeated interrupt\n", __func__);
		goto out;
	}

	if (info->gesture_enable) {
		if (!zinitix_bit_test(info->touch_info.status, BIT_GESTURE_WAKE)) {
			dev_err(&client->dev, "%s: Can't detect any event (0x%04x)\n",
				__func__, info->touch_info.status);
		}

		if (ztw522_clear_interrupt(info))
			dev_err(&client->dev, "%s: failed clear interrupt.\n", __func__);
		ztw522_send_wakeup_event(info);

		goto out;
	}

#ifdef SUPPORTED_TOUCH_KEY
	if (zinitix_bit_test(info->touch_info.status, BIT_ICON_EVENT)) {
		if (ztw522_read_data(info->client, ztw522_ICON_STATUS_REG,
			(u8 *)(&info->icon_event_reg), 2) < 0) {
			dev_err(&client->dev, "%s: Failed to read button info\n", __func__);
			ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
			goto out;
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg, (BIT_O_ICON0_DOWN + i))) {
				input_report_key(info->input_dev, info->button_code[i], 1);
				if (info->button[i] != ICON_BUTTON_DOWN)
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
					dev_info(&client->dev, "%s: key %d down\n", __func__, info->button_code[i]);
#else
					dev_info(&client->dev, "%s: key down\n", __func__);
#endif
				info->button[i] = ICON_BUTTON_DOWN;
			}
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg, (BIT_O_ICON0_UP + i))) {
				info->button[i] = ICON_BUTTON_UP;
				input_report_key(info->input_dev, info->button_code[i], 0);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				dev_info(&client->dev, "%s: key %d up\n", __func__, info->button_code[i]);
#else
				dev_info(&client->dev, "%s: key up\n", __func__);
#endif
			}
		}
	}
#endif

	if (!info->cap_info.multi_fingers) {
		dev_info(&client->dev, "%s: cap_info.multi_fingers is zero. work_state:[%d], gesture:[%s]\n",
			__func__, info->work_state, info->gesture_enable ? "enable":"disable");
	}

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			 /* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY + info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX + info->cap_info.MinX - x;

				maxX = info->cap_info.MaxX;
				maxY = info->cap_info.MaxY;

				if (x > maxX || y > maxY) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
					dev_err(&client->dev,
						"%s: Invalid coord %d : x=%d, y=%d\n", __func__, i, x, y);
#endif
					continue;
				}

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;
			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN)) {
				info->finger_cnt++;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				dev_info(&client->dev, "%s: [%d][P] x=%3d, y=%3d, p=%d\n",
						__func__, i, x, y, info->palm_flag);
#else
				dev_info(&client->dev,
						"%s: [%d][P] p = %d\n",
						__func__, i, info->palm_flag);
#endif
			} else if (zinitix_bit_test(sub_status, SUB_BIT_MOVE)) {
				info->move_cnt[i]++;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				if (info->debug_enable)
					dev_info(&client->dev, "%s: [%d][M] x=%3d, y=%3d, p=%d\n",
						__func__, i, x, y, info->palm_flag);
				else
					dev_dbg(&client->dev, "%s: [%d][M] x=%3d, y=%3d, p=%d\n",
						 __func__, i, x, y, info->palm_flag);
#else
				dev_dbg(&client->dev,
						"%s: [%d][M] p = %d\n", __func__, i, info->palm_flag);
#endif
			}

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR, (u32)w);


#ifdef REPORT_2D_Z
			ret = ztw522_read_data(client, ztw522_REAL_WIDTH + i, (u8*)&z, 2);
			if (ret < 0)
				dev_err(&client->dev, "%s: Failed to read %d's Real width %s\n", __func__, i, __func__);
			if (z < 1) z = 1;
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, (u32)z);
#endif
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP) ||
			zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			dev_info(&client->dev, "%s: [%d][R] x=%3d, y=%3d, m=%d, v=0x%02x%02x, md=%d\n",
				__func__, i, info->touch_info.coord[i].x, info->touch_info.coord[i].y,
				info->move_cnt[i], info->cap_info.fw_minor_version,
				info->cap_info.reg_data_version, info->optional_mode);
			info->move_cnt[i] = 0;
			if (info->finger_cnt > 0)
				info->finger_cnt--;

#ifdef CONFIG_SLEEP_MONITOR
			if ((!info->finger_cnt) && (info->release_cnt < CNT_MARK))
				info->release_cnt++;
#endif
#if SUPPORTED_PALM_TOUCH
			if ((!info->finger_cnt) && (info->palm_flag)) {
				info->palm_flag = 0;
				input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, 0);
				input_report_abs(info->input_dev, ABS_MT_PALM, info->palm_flag);
				dev_info(&client->dev, "%s: palm: [%d]\n", __func__, info->palm_flag);
			}
#endif
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
		} else
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
	}

#if SUPPORTED_PALM_TOUCH
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM))
		palm = 1;
	else if (zinitix_bit_test(info->touch_info.status, BIT_PALM_REJECT))
		palm = 2;
	else
		palm = 0;

	if (info->palm_flag != palm) {
		info->palm_flag = !!palm;
		if (!info->finger_cnt) {
			input_mt_slot(info->input_dev, 0);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, pdata->x_resolution >>1);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, pdata->y_resolution >>1);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, info->palm_flag);
		}
		input_report_abs(info->input_dev, ABS_MT_PALM, info->palm_flag);
		dev_info(&client->dev, "%s: palm: [%d]\n", __func__, info->palm_flag);
	}
#endif
	memcpy((char *)&info->reported_touch_info,
		(char *)&info->touch_info, sizeof(struct point_info));
	input_sync(info->input_dev);

out:
	/* Touch Booster Off */
	if ((!info->palm_flag) && (!info->finger_cnt))
		touch_booster_release();

	if (ztw522_clear_interrupt(info))
		dev_err(&client->dev, "%s: failed clear interrupt.\n", __func__);

	if (info->work_state == NORMAL) {
		info->work_state = NOTHING;
	}

	up(&info->work_lock);

	return IRQ_HANDLED;
}

#if defined(CONFIG_PM)
static int ztw522_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ztw522_ts_info *info = i2c_get_clientdata(client);

	if (info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: already enabled. work_state:[%d], gesture:[%s]\n",
			__func__, info->work_state, info->gesture_enable ? "enable":"disable");
		return 0;
	}

	dev_info(&client->dev, "%s: work_state:[%d], gesture:[%s]\n",
		__func__, info->work_state, info->gesture_enable ? "enable":"disable");

	if (info->gesture_enable) {
		info->work_state = NOTHING;
		disable_irq_wake(info->irq);
		info->wait_until_wake = true;
		wake_up(&info->wait_q);
	}

	return 0;
}

static int ztw522_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ztw522_ts_info *info = i2c_get_clientdata(client);

	if (info->work_state == SUSPEND) {
		dev_err(&client->dev, "%s: already disabled. gesture:[%s]\n",
			__func__, info->gesture_enable ? "enable":"disable");
		return 0;
	}

	dev_info(&client->dev, "%s: work_state:[%d], gesture:[%s]\n",
		__func__, info->work_state, info->gesture_enable ? "enable":"disable");

	if (info->gesture_enable) {
		info->wait_until_wake = false;
		info->work_state = SUSPEND;
		enable_irq_wake(info->irq);
	}
	return 0;
}
#endif

static int ztw522_input_open(struct input_dev *dev)
{
	struct ztw522_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;

	dev_info(&client->dev, "%s\n", __func__);

	if(!info->init_done) {
		dev_info(&client->dev, "%s: device is not initialized\n", __func__);
		return 0;
	}

	if(info->device_enabled) {
		dev_err(&client->dev, "%s: already enabled\n", __func__);
		return 0;
	}

	down(&info->work_lock);

	ztw522_power_control(info, POWER_ON_SEQUENCE);

	info->device_enabled = true;
	info->work_state = NOTHING;

	if (!ztw522_mini_init_touch(info))
		dev_err(&client->dev, "%s: Failed to resume\n", __func__);

	enable_irq(info->irq);
	up(&info->work_lock);

	return 0;
}
static void ztw522_input_close(struct input_dev *dev)
{
	struct ztw522_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;

	dev_info(&client->dev, "%s\n", __func__);

	if ((!info->device_enabled) || (!info->init_done)){
		dev_err(&client->dev, "%s: already disabled\n", __func__);
		return;
	}

	down(&info->work_lock);

	disable_irq(info->irq);

	info->gesture_enable = false;
	info->device_enabled = false;
	info->work_state = SUSPEND;

	ztw522_clear_report_data(info);

	ztw522_power_control(info, POWER_OFF);

	up(&info->work_lock);

	return;
}

/* For DND*/
static bool ztw522_set_touchmode(struct ztw522_ts_info *info, u16 value)
{
	int i;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&info->client->dev, "%s: other process occupied.\n", __func__);
		enable_irq(info->irq);
		up(&info->work_lock);
		return -1;
	}

	info->work_state = SET_MODE;

	if (value == TOUCH_SEC_MODE)
		info->touch_mode = TOUCH_POINT_MODE;
	else
		info->touch_mode = value;

	dev_info(&info->client->dev, "%s: touchkey_testmode = %d\n",
			__func__, info->touch_mode);

	if (ztw522_write_reg(info->client, ztw522_TOUCH_MODE,
			info->touch_mode) != I2C_SUCCESS)
		dev_err(&info->client->dev,
			"%s: Fail to set ZINITX_TOUCH_MODE %d.\n",
			__func__, info->touch_mode);

	if (ztw522_write_cmd(info->client, ztw522_SWRESET_CMD) != I2C_SUCCESS) {
		dev_err(&info->client->dev,
			"%s: Failed to write reset command\n", __func__);
	}

	/* clear garbage data */
	for (i = 0; i <= INT_CLEAR_RETRY; i++) {
		zinitix_delay(20);
		ztw522_write_cmd(info->client, ztw522_CLEAR_INT_STATUS_CMD);
	}

	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);
	return 1;
}

static int ztw522_upgrade_sequence(struct ztw522_ts_info *info, const u8 *firmware_data)
{
	struct i2c_client *client = info->client;
	bool ret;

	disable_irq(info->irq);
	down(&info->work_lock);
	info->work_state = UPGRADE;

	ztw522_clear_report_data(info);

	dev_info(&client->dev, "%s: start upgrade firmware\n", __func__);

	ret = ztw522_upgrade_firmware(info, firmware_data, info->cap_info.ic_fw_size);
	if (!ret)
		dev_err(&client->dev, "%s: Failed update firmware\n", __func__);

	ztw522_init_touch(info, true);

	enable_irq(info->irq);
	info->work_state = NOTHING;
	up(&info->work_lock);

	return (ret) ? 0 : -1;
}

#ifdef SEC_FACTORY_TEST
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},

	/* vendor dependant command */
	{TSP_CMD("run_dnd_read", run_dnd_read),},
	{TSP_CMD("get_dnd", get_dnd),},
	{TSP_CMD("get_dnd_all_data", get_dnd_all_data),},
	{TSP_CMD("run_dnd_v_gap_read", run_dnd_v_gap_read),},
	{TSP_CMD("get_dnd_v_gap", get_dnd_v_gap),},
	{TSP_CMD("run_dnd_h_gap_read", run_dnd_h_gap_read),},
	{TSP_CMD("get_dnd_h_gap", get_dnd_h_gap),},
	{TSP_CMD("run_hfdnd_read", run_hfdnd_read),},
	{TSP_CMD("get_hfdnd", get_hfdnd),},
	{TSP_CMD("get_hfdnd_all_data", get_hfdnd_all_data),},
	{TSP_CMD("run_hfdnd_v_gap_read", run_hfdnd_v_gap_read),},
	{TSP_CMD("get_hfdnd_v_gap", get_hfdnd_v_gap),},
	{TSP_CMD("run_hfdnd_h_gap_read", run_hfdnd_h_gap_read),},
	{TSP_CMD("get_hfdnd_h_gap", get_hfdnd_h_gap),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("get_delta_all_data", get_delta_all_data),},
	{TSP_CMD("dead_zone_enable", dead_zone_enable),},
	{TSP_CMD("clear_cover_mode", clear_cover_mode),},
	{TSP_CMD("clear_reference_data", clear_reference_data),},
	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_ref_calibration", run_ref_calibration),},
	{TSP_CMD("tsp_connect_test", run_connect_test),},
};

static inline void set_cmd_result(struct tsp_factory_info *finfo, char *buff, int len)
{
	strncat(finfo->cmd_result, buff, len);
}

static inline void set_default_result(struct tsp_factory_info *finfo)
{
	char delim = ':';
	memset(finfo->cmd_result, 0x00, ARRAY_SIZE(finfo->cmd_result));
	memcpy(finfo->cmd_result, finfo->cmd, strlen(finfo->cmd));
	strncat(finfo->cmd_result, &delim, 1);
}

static int get_tsp_connect_data(struct ztw522_ts_info *info, u8 *threshold)
{
	struct i2c_client *client = info->client;
	struct zxt_ts_platform_data	*pdata = info->pdata;
	int i;
	u32 ret;

	down(&info->work_lock);

	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: other process occupied. (%d)\n",
			__func__, info->work_state);
		up(&info->work_lock);
		return -1;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < CONNECT_TEST_RETRY; i++) {
		while (gpio_get_value(pdata->gpio_int))
			zinitix_delay(GPIO_GET_DELAY);

		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		zinitix_delay(GPIO_GET_DELAY);
	}

	ret = ztw522_read_data(client, ztw522_CONNECTED_REG, threshold, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Failed to read  ztw522_CONNECTED_REG(%d)\n",
			__func__, ret);
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		goto out;
	}

	ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
	ret = 0;

out:
	info->work_state = NOTHING;
	up(&info->work_lock);

	return ret;
}


static void run_connect_test(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 threshold = 0;
	bool ret;

	set_default_result(finfo);

	disable_irq(info->irq);

	ztw522_set_touchmode(info, TOUCH_CND_MODE);

	ret = get_tsp_connect_data(info, (u8 *)&threshold);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to read ztw522_CONNECTED_REG\n", __func__);
		ztw522_set_touchmode(info, TOUCH_POINT_MODE);
		finfo->cmd_state = FAIL;
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NG");
		goto out;
	}

	ztw522_set_touchmode(info, TOUCH_POINT_MODE);

	if (threshold >= TSP_CONNECTED_THRETHOLD) {
		finfo->cmd_state = OK;
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");
	} else {
		finfo->cmd_state = FAIL;
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NG");
	}

	dev_info(&client->dev, "%s: threshold=[%d]\n", __func__, threshold);
	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strlen(finfo->cmd_buff));

out:
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	enable_irq(info->irq);

	return;
}

static void fw_update(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	int ret = 0;
	u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	const struct firmware *tsp_fw = NULL;
	char fw_path[ZINITIX_MAX_FW_PATH];

	set_default_result(finfo);
/*
	* 0 : [BUILT_IN] Getting firmware which is for user.
	* 1 : [UMS] Getting firmware from sd card.
	* 2 : [FFU] Getting firmware from air.
*/
	switch (finfo->cmd_param[0]) {
	case BUILT_IN:
		if (!info->fw_data) {
			snprintf(fw_path, ZINITIX_MAX_FW_PATH, "%s%s", ZINITIX_FW_PATH, info->pdata->fw_name);
			ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
			if (ret) {
				dev_err(&info->client->dev, "%s: failed to request_firmware %s\n",
							__func__, fw_path);
				finfo->cmd_state = FAIL;
				return;
			} else {
				info->fw_data = (unsigned char *)tsp_fw->data;
			}
		}

		ret = ztw522_upgrade_sequence(info, (u8 *)info->fw_data);
		if (ret < 0) {
			finfo->cmd_state = FAIL;
			return;
		}

		if (info->fw_data) {
			release_firmware(tsp_fw);
			info->fw_data = NULL;
		}
		break;
	case UMS:
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fp = filp_open(ZINITIX_DEFAULT_UMS_FW, O_RDONLY, S_IRUSR);
		if (IS_ERR(fp)) {
			dev_err(&client->dev, "%s: file(%s) open error:%d\n",
				__func__, ZINITIX_DEFAULT_UMS_FW, (s32)fp);
			finfo->cmd_state = FAIL;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "%s: invalid fw size!! size:%ld\n", __func__, fsize);
			finfo->cmd_state = FAIL;
			goto err_open;
		}

		buff = devm_kzalloc(&client->dev, (size_t)fsize, GFP_KERNEL);
		if (!buff) {
			dev_err(&client->dev, "%s: failed to alloc buffer for fw\n", __func__);
			finfo->cmd_state = FAIL;
			goto err_alloc;
		}

		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			finfo->cmd_state = FAIL;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);
		dev_info(&client->dev, "%s: ums fw is loaded\n", __func__);

		ret = ztw522_upgrade_sequence(info, (u8 *)buff);
		if (ret < 0) {
			devm_kfree(&client->dev, buff);
			finfo->cmd_state = FAIL;
			goto update_fail;
		}
		break;

	case FFU:
		snprintf(fw_path, ZINITIX_MAX_FW_PATH, "%s", ZINITIX_DEFAULT_FFU_FW);

		dev_err(&info->client->dev, "%s: Load firmware : %s\n",
						__func__, fw_path);

		ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
		if (ret) {
			dev_err(&info->client->dev, "%s: failed to request_firmware %s\n",
						__func__, fw_path);
			finfo->cmd_state = FAIL;
			return;
		} else {
			info->fw_data = (unsigned char *)tsp_fw->data;
		}

		ret = ztw522_upgrade_sequence(info, (u8 *)info->fw_data);
		if (ret < 0) {
			finfo->cmd_state = FAIL;
			return;
		}

		if (info->fw_data) {
			release_firmware(tsp_fw);
			info->fw_data = NULL;
		}
		break;

	default:
		dev_err(&client->dev, "%s: invalid fw file type!!\n", __func__);
		goto update_fail;
	}

	finfo->cmd_state = OK;
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff) , "%s", "OK");
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	devm_kfree(&client->dev, buff);

	return;


if (fp != NULL) {
err_fw_size:
	devm_kfree(&client->dev, buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);
}
update_fail:
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff) , "%s", "NG");
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}

static void get_fw_ver_bin(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 hw_id, model_version, fw_version, vendor_id;
	u32 version = 0, length;
	const struct firmware *tsp_fw = NULL;
	char fw_path[ZINITIX_MAX_FW_PATH];
	int ret;

	set_default_result(finfo);

	if (!info->fw_data) {
		snprintf(fw_path, ZINITIX_MAX_FW_PATH, "%s%s", ZINITIX_FW_PATH, info->pdata->fw_name);
		ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
		if (ret) {
			dev_err(&info->client->dev, "%s: failed to request_firmware %s\n",
						__func__, fw_path);
			finfo->cmd_state = FAIL;
			return;
		} else {
			info->fw_data = (unsigned char *)tsp_fw->data;
		}
	}

	hw_id = (u16)(info->fw_data[52] | (info->fw_data[53] << 8));
	model_version = (u16)(info->fw_data[56] | (info->fw_data[57] << 8));
	fw_version = (u16)(info->fw_data[60] | (info->fw_data[61] << 8));

	version = (u32)((hw_id & 0xf) << 16) |((model_version & 0xf) << 8)\
			| (fw_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", "ZI");
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length, "%06X", version);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	if (info->fw_data) {
		release_firmware(tsp_fw);
		info->fw_data = NULL;
	}

	return;
}

static void get_fw_ver_ic(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 hw_id, model_version, fw_version, vendor_id;
	u32 version = 0, length;

	set_default_result(finfo);

/* Read firmware version from IC */
	if(ztw522_read_data(client, ztw522_FIRMWARE_VERSION, (u8 *)&hw_id, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read firmware version\n", __func__);
		goto out;
	}
	if(ztw522_read_data(client, ztw522_MINOR_FW_VERSION, (u8 *)&model_version, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read minor version\n", __func__);
		goto out;
	}
	if(ztw522_read_data(client, ztw522_DATA_VERSION_REG, (u8 *)&fw_version, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read register version\n", __func__);
		goto out;
	}
	
	vendor_id = ntohs(info->cap_info.vendor_id);
	version = (u32)((hw_id & 0xf) << 16)
		| ((model_version & 0xf) << 8) | (fw_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length, "%06X", version);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
out:
	finfo->cmd_state = FAIL;
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff) , "%s", "NG");
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_threshold(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%d", info->cap_info.threshold);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#define ztw522_VENDOR_NAME "ZINITIX"

static void get_chip_vendor(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", ztw522_VENDOR_NAME);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#define ztw522_CHIP_NAME "ztw522"
#define ZT7538_CHIP_NAME "ZT7538"

static void get_chip_name(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	if (info->chip_code == ZTW522_IC_CHIP_CODE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", ztw522_CHIP_NAME);
	else if (info->chip_code == ZT7538_IC_CHIP_CODE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", ZT7538_CHIP_NAME);
	else
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", ztw522_VENDOR_NAME);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_x_num(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%u", info->cap_info.y_node_num);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_y_num(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%u", info->cap_info.x_node_num);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void not_support_cmd(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	sprintf(finfo->cmd_buff, "%s", "NA");
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = NOT_APPLICABLE;

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_all_data(struct ztw522_ts_info *info,
			run_func_t run_func, void *data, enum data_type type)
{
	struct tsp_factory_info *finfo = info->factory_info;
	struct i2c_client *client = info->client;
	char *buff;
	int node_num;
	int page_size, len;
	static int index = 0;

	set_default_result(finfo);
	info->get_all_data = true;

	if (!data) {
		dev_err(&client->dev, "%s: data is NULL\n", __func__);
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "FAIL");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		goto all_data_out;
	}

	if (finfo->cmd_param[0] == 0) {
		run_func(info);
		if (finfo->cmd_state != RUNNING)
			goto all_data_out;
		index = 0;
	} else {
		if (index == 0) {
			dev_info(&client->dev,
				"%s: Please do cmd_param '0' first\n", 	__func__);
			snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
			set_cmd_result(finfo, finfo->cmd_buff,
					strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
			finfo->cmd_state = NOT_APPLICABLE;
			goto all_data_out;
		}
	}

	page_size = TSP_CMD_RESULT_STR_LEN - strlen(finfo->cmd) - 10;
	node_num = info->cap_info.x_node_num * info->cap_info.y_node_num;
	buff = devm_kzalloc(&client->dev, TSP_CMD_RESULT_STR_LEN, GFP_KERNEL);
	if (buff != NULL) {
		char *pBuf = buff;
		for (; index < node_num; index++) {
			switch(type) {
			case DATA_UNSIGNED_CHAR: {
				unsigned char *ddata = data;
				len = snprintf(pBuf, 5, "%u,", ddata[index]);
				break;}
			case DATA_SIGNED_CHAR: {
				char *ddata = data;
				len = snprintf(pBuf, 5, "%d,", ddata[index]);
				break;}
			case DATA_UNSIGNED_SHORT: {
				unsigned short *ddata = data;
				len = snprintf(pBuf, 10, "%u,", ddata[index]);
				break;}
			case DATA_SIGNED_SHORT: {
				short *ddata = data;
				len = snprintf(pBuf, 10, "%d,", ddata[index]);
				break;}
			case DATA_UNSIGNED_INT: {
				unsigned int *ddata = data;
				len = snprintf(pBuf, 15, "%u,", ddata[index]);
				break;}
			case DATA_SIGNED_INT: {
				int *ddata = data;
				len = snprintf(pBuf, 15, "%d,", ddata[index]);
				break;}
			default:
				dev_err(&client->dev,
					"%s: not defined data type\n", __func__);
				snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
				set_cmd_result(finfo, finfo->cmd_buff,
						strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
				finfo->cmd_state = NOT_APPLICABLE;
				devm_kfree(&client->dev, buff);
				goto all_data_out;
			}

			if (page_size - len < 0) {
				snprintf(pBuf, 5, "cont");
				break;
			} else {
				page_size -= len;
				pBuf += len;
			}
		}
		if (index == node_num)
			index = 0;

		set_cmd_result(finfo, buff, TSP_CMD_RESULT_STR_LEN);
		finfo->cmd_state = OK;

		devm_kfree(&client->dev, buff);
	} else {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "kzalloc failed");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = NOT_APPLICABLE;
	}

all_data_out:
	dev_info(&client->dev, "%s: %s\n", __func__, finfo->cmd_result);
	info->get_all_data = false;
}

static void run_dnd_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	u16 min, max;
	s32 i, j,offset;
	bool ret;

	set_default_result(finfo);

	ztw522_set_touchmode(info, TOUCH_DND_MODE);
	ret = get_raw_data(info, (u8 *)raw_data->dnd_data, 2);
	ztw522_set_touchmode(info, TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		pr_info("%s: dnd_data[%2d] :", client->name, i);
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			offset = i * info->cap_info.y_node_num + j;
			pr_cont(" %5d", raw_data->dnd_data[offset]);
			if (raw_data->dnd_data[offset] < min &&raw_data->dnd_data[offset] != 0)
				min = raw_data->dnd_data[offset];
			if (raw_data->dnd_data[offset] > max)
				max = raw_data->dnd_data[offset];
		}
		pr_cont("\n");
	}

	if (ret) {
		finfo->cmd_state = OK;
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");
	} else {
		finfo->cmd_state = FAIL;
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NG");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strlen(finfo->cmd_buff));

	return;
}

static void get_dnd(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	const int y_num = info->cap_info.y_node_num;
	int y_node, i;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= info->cap_info.x_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num; i++) {
		sprintf(tmp, "%d",  raw_data->dnd_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_dnd_all_data(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;

	get_all_data(info, run_dnd_read, info->raw_data->dnd_data, DATA_UNSIGNED_SHORT);
}

static void run_dnd_v_gap_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;
	u16 screen_max = 0x0000;
	u16 touchkey_max = 0x0000;
	set_default_result(finfo);

	memset(raw_data->vgap_data, 0x00, TSP_CMD_NODE_NUM);

	dev_info(&client->dev, "%s: DND V Gap start\n", __func__);

	for (i = 0; i < x_num - 1; i++) {
		pr_info("%s: [%2d] :", client->name, i);
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->dnd_data[offset];
			next_val = raw_data->dnd_data[offset + y_num];
			if ((i >= x_num - 2) && !next_val) {	/* touchkey node */
				raw_data->vgap_data[offset] = next_val;
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			pr_cont(" %d", val);

			raw_data->vgap_data[offset] = val;

			if (raw_data->vgap_data[offset] > screen_max)
				screen_max = raw_data->vgap_data[offset];
		}
		pr_cont("\n");
	}

	dev_info(&client->dev, "%s: DND V Gap screen_max %d touchkey_max %d\n",
			__func__, screen_max, touchkey_max);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d", screen_max, touchkey_max);

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	return;
}

static void run_dnd_h_gap_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;
	u16 screen_max = 0x0000;
	u16 touchkey_max = 0x0000;
	set_default_result(finfo);

	memset(raw_data->hgap_data, 0x00, TSP_CMD_NODE_NUM);

	dev_info(&client->dev, "%s: DND H Gap start\n", __func__);

	for (i = 0; i < x_num ; i++) {
		pr_info("%s: [%2d] :", client->name, i);
		for (j = 0; j < y_num - 1; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->dnd_data[offset];
			if ((i >= x_num - 1) && !cur_val) {	/* touchkey node */
				raw_data->hgap_data[offset] = cur_val;
				continue;
			}

			next_val = raw_data->dnd_data[offset + 1];
			if ((i >= x_num - 1) && !next_val) {	/* touchkey node */
				raw_data->hgap_data[offset] = next_val;
				for (++j; j < y_num - 1; j++) {
					offset = (i * y_num) + j;

					next_val = raw_data->dnd_data[offset];
					if (!next_val) {
						raw_data->hgap_data[offset] = next_val;
						continue;
					}
					break;
				}
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			pr_cont(" %d", val);

			raw_data->hgap_data[offset] = val;

			if (raw_data->hgap_data[offset] > screen_max)
					screen_max = raw_data->hgap_data[offset];
		}
		pr_cont("\n");
	}

	dev_info(&client->dev, "%s: DND H Gap screen_max %d, touchkey_max %d\n",
			__func__, screen_max, touchkey_max);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d", screen_max, touchkey_max);

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	return;
}

static void get_dnd_h_gap(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	int y_node, i;
	const int x_num = info->cap_info.x_node_num;
	const int y_num = info->cap_info.y_node_num;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= x_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num-1; i++) {
		sprintf(tmp, "%d",  raw_data->hgap_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}

static void get_dnd_v_gap(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	int y_node, i;
	const int x_num = info->cap_info.x_node_num;
	const int y_num = info->cap_info.y_node_num;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= x_num-1) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num; i++) {
		sprintf(tmp, "%d",  raw_data->vgap_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}


static void run_hfdnd_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset;
	u16 min = 0xFFFF, max = 0x0000;

	set_default_result(finfo);

	ztw522_set_touchmode(info, TOUCH_HFDND_MODE);
	get_raw_data(info, (u8 *)raw_data->hfdnd_data, 2);
	ztw522_set_touchmode(info, TOUCH_POINT_MODE);

	dev_info(&client->dev, "%s: HF DND start\n", __func__);

	for (i = 0; i < x_num; i++) {
		pr_info("%s: hfdnd_data[%2d] :", client->name, i);
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;
			pr_cont(" %5d", raw_data->hfdnd_data[offset]);
			if (raw_data->hfdnd_data[offset] < min && raw_data->hfdnd_data[offset] != 0)
				min = raw_data->hfdnd_data[offset];
			if (raw_data->hfdnd_data[offset] > max)
				max = raw_data->hfdnd_data[offset];
		}
		pr_cont("\n");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_hfdnd(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	const int y_num = info->cap_info.y_node_num;
	int y_node, i;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= info->cap_info.x_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num; i++) {
		sprintf(tmp, "%d",  raw_data->hfdnd_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_hfdnd_all_data(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;

	get_all_data(info, run_hfdnd_read, info->raw_data->hfdnd_data, DATA_SIGNED_SHORT);
}

static void run_hfdnd_v_gap_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;

	set_default_result(finfo);

	memset(raw_data->hfvgap_data, 0x00, TSP_CMD_NODE_NUM);

	dev_info(&client->dev, "%s: HFDND V Gap start\n", __func__);

	for (i = 0; i < x_num - 1; i++) {
		pr_info("%s: [%2d] :", client->name, i);
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->hfdnd_data[offset];
			next_val = raw_data->hfdnd_data[offset + y_num];
			if ((i >= x_num - 2) && !next_val) {	/* touchkey node */
				raw_data->hfvgap_data[offset] = next_val;
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			pr_cont(" %d", val);

			raw_data->hfvgap_data[offset] = val;
		}
		pr_cont("\n");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	return;
}

static void get_hfdnd_v_gap(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	int y_node, i;
	const int x_num = info->cap_info.x_node_num;
	const int y_num = info->cap_info.y_node_num;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= x_num-1) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num; i++) {
		sprintf(tmp, "%d",  raw_data->hfvgap_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}

static void run_hfdnd_h_gap_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;

	set_default_result(finfo);

	memset(raw_data->hgap_data, 0x00, TSP_CMD_NODE_NUM);

	dev_info(&client->dev, "%s: HFDND H Gap start\n", __func__);

	for (i = 0; i < x_num ; i++) {
		pr_info("%s: [%2d] :", client->name, i);
		for (j = 0; j < y_num - 1; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->hfdnd_data[offset];
			if ((i >= x_num - 1) && !cur_val) {	/* touchkey node */
				raw_data->hfhgap_data[offset] = cur_val;
				continue;
			}

			next_val = raw_data->hfdnd_data[offset + 1];
			if ((i >= x_num - 1) && !next_val) {	/* touchkey node */
				raw_data->hfhgap_data[offset] = next_val;
				for (++j; j < y_num - 1; j++) {
					offset = (i * y_num) + j;

					next_val = raw_data->hfdnd_data[offset];
					if (!next_val) {
						raw_data->hfhgap_data[offset] = next_val;
						continue;
					}
					break;
				}
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			pr_cont(" %d", val);

			raw_data->hfhgap_data[offset] = val;
		}
		pr_cont("\n");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}


static void get_hfdnd_h_gap(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	char tmp[16] = {0};
	int y_node, i;
	const int x_num = info->cap_info.x_node_num;
	const int y_num = info->cap_info.y_node_num;

	set_default_result(finfo);

	y_node = finfo->cmd_param[0];

	if (y_node < 0 || y_node >= x_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		set_cmd_result(finfo, finfo->cmd_buff, strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	for (i=0; i < y_num-1; i++) {
		sprintf(tmp, "%d",  raw_data->hfhgap_data[i + (y_num * y_node)]);
		strcat(finfo->cmd_buff, tmp);
		if (i < y_num-1)
			strcat(finfo->cmd_buff, " ");
	}

	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}

static void run_delta_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s16 min, max;
	s32 i, j;


	set_default_result(finfo);

	ztw522_set_touchmode(info, TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *)raw_data->delta_data, 10);
	ztw522_set_touchmode(info, TOUCH_POINT_MODE);

	min = (s16)0x7FFF;
	max = (s16)0x8000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		pr_info("%s: delta_data[%2d] : ", client->name, i);
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			pr_cont("[%3d]", raw_data->delta_data[i * info->cap_info.y_node_num + j]);
			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->delta_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->delta_data[i * info->cap_info.y_node_num + j];
			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->delta_data[i * info->cap_info.y_node_num + j];

		}
		pr_cont("\n");
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d", min, max);
	if (!info->get_all_data) {
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = OK;
	}

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strlen(finfo->cmd_buff));

	return;
}

static void get_delta(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(finfo);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_delta_all_data(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;

	get_all_data(info, run_delta_read, info->raw_data->delta_data, DATA_SIGNED_SHORT);
}

static void dead_zone_enable(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	if (finfo->cmd_param[0] == 1) {	/* enable */
		zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_EDGE_SELECT);
	} else if (finfo->cmd_param[0] == 0) {
		zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_EDGE_SELECT);
	} else {
		finfo->cmd_state = FAIL;
		sprintf(finfo->cmd_buff, "%s", "NG");
		goto err;
	}
	ztw522_set_optional_mode(info, false);

	finfo->cmd_state = OK;
	sprintf(finfo->cmd_buff, "%s", "OK");
err:
	set_cmd_result(finfo, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void clear_cover_mode(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	int arg = finfo->cmd_param[0];

	set_default_result(finfo);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u",
							(unsigned int) arg);

	info->cover_state = arg;
	ztw522_cover_set(info);
	set_cmd_result(finfo, finfo->cmd_buff,
					strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	finfo->cmd_is_running = false;
	finfo->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_reference_read(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	int min = 0xFFFF, max = 0x0000;
	s32 i, j, touchkey_node = 2;

	set_default_result(finfo);

	ztw522_set_touchmode(info, TOUCH_REFERENCE_MODE);
	get_raw_data(info, (u8 *)raw_data->reference_data, 2);
	ztw522_set_touchmode(info, TOUCH_POINT_MODE);

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		pr_info("%s: ref_data[%2d] : ", client->name, i);
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			pr_cont(" %5d", raw_data->reference_data[i * info->cap_info.y_node_num + j]);

			if (i == (info->cap_info.x_node_num - 1)) {
				if ((j == touchkey_node)||(j == (info->cap_info.y_node_num - 1) - touchkey_node)) {
					if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] < min &&
						raw_data->reference_data[(i * info->cap_info.y_node_num) + j] >= 0)
						min = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];

					if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] > max)
						max = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];
				}
			} else {
				if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] < min &&
					raw_data->reference_data[(i * info->cap_info.y_node_num) + j] >= 0)
					min = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];

				if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] > max)
					max = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];
			}
		}
		pr_cont("\n");
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d", min, max);
	set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strlen(finfo->cmd_buff));

	return;
}

static void get_reference(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(finfo);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(finfo, finfo->cmd_buff,
						strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->reference_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(finfo, finfo->cmd_buff,
					strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d), x=%d, y=%d\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)), x_node, y_node);

	return;
}

static void clear_reference_data(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(finfo);

	ztw522_write_reg(client, ztw522_EEPROM_INFO_REG, 0xffff);

	ztw522_write_reg(client, 0xc003, 0x0001);
	ztw522_write_reg(client, 0xc104, 0x0001);
	usleep_range(100, 100);
	if (ztw522_write_cmd(client, ztw522_SAVE_STATUS_CMD) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: failed to TSP clear calibration bit\n", __func__);
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		goto out;
	}

	zinitix_delay(500);
	ztw522_write_reg(client, 0xc003, 0x0000);
	ztw522_write_reg(client, 0xc104, 0x0000);
	usleep_range(100, 100);

	dev_info(&client->dev, "%s: TSP clear calibration bit\n", __func__);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "OK");
out:
	set_cmd_result(finfo, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		(int)strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	return;
}

static void run_ref_calibration(void *device_data)
{
	struct ztw522_ts_info *info = (struct ztw522_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	int i;
	bool ret;

	set_default_result(finfo);

	disable_irq(info->irq);

	ret = ztw522_hw_calibration(info);
	dev_info(&client->dev, "%s: TSP calibration %s\n",
				__func__, ret ? "Pass" : "Fail");

	for (i = 0; i < 5; i++) {
		ztw522_write_cmd(client, ztw522_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);
	}

	enable_irq(info->irq);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", ret ? "OK" : "NG");
	set_cmd_result(finfo, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		(int)strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
}

static ssize_t ztw522_show_gesture_mode(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%sable\n", info->gesture_enable ? "en":"dis");
}

static ssize_t ztw522_store_gesture_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	bool enable;
	int ret;
	u16 gesture_state = 0;
	int retry = 0;

	ret = strtobool(buf, &enable);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to get parameter.\n", __func__);
		return -EIO;
	}

	if (info->gesture_enable == enable) {
		dev_err(&client->dev, "%s: gesture already %sabled\n",
		__func__, info->gesture_enable ? "en":"dis");
		return size;
	}

	info->gesture_enable = enable;

	dev_info(&client->dev, "%s: gesture %sable\n",
		__func__, info->gesture_enable ? "en":"dis");

	if (!info->device_enabled) {
		dev_err(&client->dev, "%s: Device is not enable.\n", __func__);
		return -EIO;
	}
	ztw522_clear_report_data(info);

gesture_retry:
	ret = ztw522_send_start_event(info);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to write ztw522_I2C_START_CMD\n", __func__);
		ret = -EIO;
		goto out;
	}

	if (enable) {
		ret = ztw522_write_cmd(info->client, ztw522_SLEEP_CMD);
		if (ret != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to write ztw522_SLEEP_CMD\n", __func__);
			ret = -EIO;
			goto out;
		}
	} else {
		ret = ztw522_write_cmd(info->client, ztw522_WAKEUP_CMD);
		if (ret != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to write ztw522_WAKEUP_CMD\n", __func__);
			ret = -EIO;
			goto out;
		}
	}

	if (0 > ztw522_read_raw_data(info->client, ztw522_GESTURE_STATE, (u8 *)&gesture_state, 2)) {
		dev_err(&client->dev, "%s: Failed to get ztw522_GESTURE_STATE\n", __func__);
		ret = -EIO;
		goto out;
	}

	if (enable != gesture_state) {
		dev_err(&client->dev, "%s: Failed to change gesture mode (0x%04x)\n", __func__, gesture_state);
		if (I2C_RETRY_TIMES > retry) {
			retry++;
			ztw522_write_cmd(client, ztw522_I2C_END_CMD);
			goto gesture_retry;
		} else {
			ret = -EIO;
			goto out;
		}
	}

	dev_info(&client->dev, "%s: gesture mode [%s]\n", __func__, gesture_state ? "on":"off");

	ret = size;
out:
	ztw522_write_cmd(client, ztw522_I2C_END_CMD);


	return ret;
}

#ifdef CONFIG_SLP_KERNEL_ENG
static ssize_t ztw522_show_debug_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", info->debug_enable);
}

static ssize_t ztw522_store_debug_enable(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	int ret;
	bool enable;

	ret = strtobool(buf, &enable);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to get enable value.\n", __func__);
		return ret;
	}

	info->debug_enable = enable;

	dev_info(&client->dev, "%s: debug_enable is %s\n",
		__func__, info->debug_enable ? "enable":"disable");

	return size;
}
#endif

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (finfo->cmd_is_running == true) {
		dev_err(&client->dev, "%s: other cmd is running\n", __func__);
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = true;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = RUNNING;

	for (i = 0; i < ARRAY_SIZE(finfo->cmd_param); i++)
		finfo->cmd_param[i] = 0;

	if (count >= TSP_CMD_STR_LEN)
		count = (TSP_CMD_STR_LEN -1);

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(finfo->cmd, 0x00, ARRAY_SIZE(finfo->cmd));
	memcpy(finfo->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	finfo->cmd_buff[0] = 0;

	if (!info->device_enabled) {
		set_default_result(finfo);
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "FAIL");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		dev_err(&client->dev, "%s: TSP is not enabled.\n", __func__);
		goto err_out;
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				finfo->cmd_param[param_cnt] = (int)simple_strtol(buff, NULL, 10);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				if ((TSP_CMD_PARAM_NUM-1) > param_cnt)
					param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "%s: cmd = %s\n", __func__, tsp_cmd_ptr->cmd_name);

	ret = ztw522_send_start_event(info);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to write ztw522_I2C_START_CMD\n", __func__);
		set_default_result(finfo);
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "FAIL");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		goto err_out;
	}

	tsp_cmd_ptr->cmd_func(info);

	ret = ztw522_write_cmd(client, ztw522_I2C_END_CMD);
	if (ret != I2C_SUCCESS) {
		dev_err(&client->dev, "%s: Failed to write ztw522_I2C_END_CMD\n", __func__);
		set_default_result(finfo);
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "FAIL");
		set_cmd_result(finfo, finfo->cmd_buff,
				strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		goto err_out;
	}

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_dbg(&client->dev, "tsp cmd: status:%d\n", finfo->cmd_state);

	if (finfo->cmd_state == WAITING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "WAITING");
	else if (finfo->cmd_state == RUNNING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "RUNNING");
	else if (finfo->cmd_state == OK)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");
	else if (finfo->cmd_state == FAIL)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "FAIL");
	else if (finfo->cmd_state == NOT_APPLICABLE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NOT_APPLICABLE");

	return snprintf(buf, sizeof(finfo->cmd_buff), "%s\n", finfo->cmd_buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "%s: tsp cmd: result: %s\n", __func__, finfo->cmd_result);

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;

	return snprintf(buf, sizeof(finfo->cmd_result), "%s\n", finfo->cmd_result);
}

static ssize_t ztw522_show_fw_ver_ic(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	char buffer[32] = {0, };
	u16 hw_id, model_version, fw_version, vendor_id;
	u32 version = 0, length;

	if (!info->device_enabled) {
		dev_err(&client->dev, "%s: TSP is not enable.\n", __func__);
		return -EIO;
	}

	ztw522_send_start_event(info);

	if(ztw522_read_data(client, ztw522_FIRMWARE_VERSION, (u8 *)&hw_id, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read firmware version\n", __func__);
		ztw522_write_cmd(client, ztw522_I2C_END_CMD);
		return -EIO;
	}
	if(ztw522_read_data(client, ztw522_MINOR_FW_VERSION, (u8 *)&model_version, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read minor version\n", __func__);
		ztw522_write_cmd(client, ztw522_I2C_END_CMD);
		return -EIO;
	}
	if(ztw522_read_data(client, ztw522_DATA_VERSION_REG, (u8 *)&fw_version, 2)<0) {
		dev_err(&client->dev, "%s: Failed to read register version\n", __func__);
		ztw522_write_cmd(client, ztw522_I2C_END_CMD);
		return -EIO;
	}

	ztw522_write_cmd(client, ztw522_I2C_END_CMD);

	vendor_id = ntohs(info->cap_info.vendor_id);
	version = (u32)((hw_id & 0xf) << 16)
		| ((model_version & 0xf) << 8) | (fw_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(buffer, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(buffer + length, sizeof(buffer) - length, "%06X", version);

	dev_info(&client->dev, "%s: %s\n", __func__, buffer);

	return scnprintf(buf, PAGE_SIZE, "%s\n", buffer);
}

static ssize_t ztw522_show_fw_ver_bin(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	const struct firmware *tsp_fw = NULL;
	char fw_path[ZINITIX_MAX_FW_PATH];
	char buffer[32] = {0, };
	u16 hw_id, model_version, fw_version, vendor_id;
	u32 version = 0, length;
	int ret;

	if (!info->fw_data) {
		snprintf(fw_path, ZINITIX_MAX_FW_PATH, "%s%s", ZINITIX_FW_PATH, info->pdata->fw_name);
		ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
		if (ret) {
			dev_err(&info->client->dev, "%s: failed to request_firmware %s\n",
						__func__, fw_path);
			return -EIO;
		} else {
			info->fw_data = (unsigned char *)tsp_fw->data;
		}
	}

	hw_id = (u16)(info->fw_data[52] | (info->fw_data[53] << 8));
	model_version = (u16)(info->fw_data[56] | (info->fw_data[57] << 8));
	fw_version = (u16)(info->fw_data[60] | (info->fw_data[61] << 8));

	version = (u32)((hw_id & 0xf) << 16) |((model_version & 0xf) << 8)\
			| (fw_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(buffer, length + 1, "%s", "ZI");
	snprintf(buffer + length, sizeof(buffer) - length, "%06X", version);

	dev_info(&client->dev, "%s: %s\n", __func__, buffer);

	if (info->fw_data) {
		release_firmware(tsp_fw);
		info->fw_data = NULL;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", buffer);
}

static DEVICE_ATTR(fw_ver_ic, S_IRUGO, ztw522_show_fw_ver_ic, NULL);
static DEVICE_ATTR(fw_ver_bin, S_IRUGO, ztw522_show_fw_ver_bin, NULL);
static DEVICE_ATTR(mode, S_IRUGO | S_IWGRP,\
		ztw522_show_gesture_mode, ztw522_store_gesture_mode);
static DEVICE_ATTR(cmd, S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);
#ifdef CONFIG_SLP_KERNEL_ENG
static DEVICE_ATTR(debug, S_IRUGO | S_IWGRP,\
		ztw522_show_debug_enable, ztw522_store_debug_enable);
#endif

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_mode.attr,
	&dev_attr_fw_ver_ic.attr,
	&dev_attr_fw_ver_bin.attr,
#ifdef CONFIG_SLP_KERNEL_ENG
	&dev_attr_debug.attr,
#endif
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

#ifdef SUPPORTED_TOUCH_KEY
static ssize_t show_touchkey_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);

	dev_info(&client->dev, "%s: key threshold = %d\n", __func__, cap->key_threshold);

	return snprintf(buf, 41, "%d", cap->key_threshold);
}

static ssize_t show_touchkey_sensitivity(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	u16 val;
	int ret;
	int i;

	if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 1;
	else {
		dev_err(&client->dev, "%s: Invalid attribute\n", __func__);
		goto err_out;
	}
	down(&info->work_lock);
	ret = ztw522_read_data(client, ztw522_BTN_WIDTH + i, (u8 *)&val, 2);
	up(&info->work_lock);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read %d's key sensitivity\n", __func__, i);
		goto err_out;
	}

	dev_info(&client->dev, "%s: %d's key sensitivity = %d\n", __func__, i, val);

	return snprintf(buf, 6, "%d", val);

err_out:
	return sprintf(buf, "NG");
}

#ifdef SUPPORTED_KEY_LED
static ssize_t touchkey_led_control(struct device *dev,
		 struct device_attribute *attr, const char *buf,
		 size_t count)
{
	struct ztw522_ts_info *info = dev_get_drvdata(dev);
	int data;
	int ret;

	ret = sscanf(buf, "%d", &data);
	if (ret != 1) {
		dev_err(&info->client->dev, "%s: cmd read err\n", __func__);
		return count;
	}

	if (!(data == 0 || data == 1)) {
		dev_err(&info->client->dev, "%s: wrong command(%d)\n",
			__func__, data);
		return count;
	}

	if (IS_ERR_OR_NULL(info->led_ldo))
		return count;

	if (data == 1)
		ret = regulator_enable(info->led_ldo);
	else
		ret = regulator_disable(info->led_ldo);

	zinitix_delay(20);

	dev_info(&info->client->dev, "%s: data(%d), ret(%d)\n",
		__func__, data, regulator_is_enabled(info->led_ldo));

	return count;
}
#endif

static DEVICE_ATTR(touchkey_threshold, S_IRUGO, show_touchkey_threshold, NULL);
static DEVICE_ATTR(touchkey_recent, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, show_touchkey_sensitivity, NULL);
#ifdef SUPPORTED_KEY_LED
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touchkey_led_control);
#endif
static struct attribute *touchkey_attributes[] = {
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_recent.attr,
#ifdef SUPPORTED_KEY_LED
	&dev_attr_brightness.attr,
#endif
	NULL,
};
static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};
#endif

static int ztw522_init_sec_factory(struct ztw522_ts_info *info)
{
	struct device *factory_ts_dev;
#ifdef SUPPORTED_TOUCH_KEY
	struct device *factory_tk_dev;
#endif
	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
	int ret;
	int i;

	factory_info = devm_kzalloc(&info->client->dev, sizeof(struct tsp_factory_info), GFP_KERNEL);
	if (unlikely(!factory_info)) {
		dev_err(&info->client->dev, "%s: failed to allocate factory_info\n", __func__);
		ret = -ENOMEM;
		goto err_alloc1;
	}

	raw_data = devm_kzalloc(&info->client->dev, sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		dev_err(&info->client->dev, "%s: failed to allocate raw_data\n", __func__);
		ret = -ENOMEM;
		goto err_alloc2;
	}

	INIT_LIST_HEAD(&factory_info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &factory_info->cmd_list_head);

#ifdef CONFIG_SEC_SYSFS
	factory_ts_dev = sec_device_create(info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		dev_err(&info->client->dev, "%s: ailed to create factory dev\n", __func__);
		ret = -ENODEV;
		goto err_create_device;
	}
#else
	factory_ts_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		dev_err(&info->client->dev, "%s: ailed to create factory dev\n", __func__);
		ret = -ENODEV;
		goto err_create_device;
	}
#endif

#ifdef SUPPORTED_TOUCH_KEY
#ifdef CONFIG_SEC_SYSFS
	factory_tk_dev = sec_device_create(info, "sec_touchkey");
	if (IS_ERR(factory_tk_dev)) {
		dev_err(&info->client->dev, "%s: failed to create factory dev\n", __func__);
		ret = -ENODEV;
		goto err_create_device;
	}
#else
	factory_tk_dev = device_create(sec_class, NULL, 0, info, "sec_touchkey");
	if (IS_ERR(factory_tk_dev)) {
		dev_err(&info->client->dev, "%s: failed to create factory dev\n", __func__);
		ret = -ENODEV;
		goto err_create_device;
	}
#endif
#endif

	ret = sysfs_create_group(&factory_ts_dev->kobj, &touchscreen_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "%s: Failed to create touchscreen sysfs group\n", __func__);
		goto err_create_sysfs;
	}

#ifdef SUPPORTED_TOUCH_KEY
	ret = sysfs_create_group(&factory_tk_dev->kobj, &touchkey_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "%s: Failed to create touchkey sysfs group\n", __func__);
		goto err_create_sysfs;
	}
#endif

	ret = sysfs_create_link(&factory_ts_dev->kobj,
		&info->input_dev->dev.kobj, "input");
	if (ret < 0) {
		dev_err(&info->client->dev,
			"%s: Failed to create input symbolic link %d\n",
			__func__, ret);
	}

	mutex_init(&factory_info->cmd_lock);
	factory_info->cmd_is_running = false;

	info->factory_info = factory_info;
	info->raw_data = raw_data;

	return ret;

err_create_sysfs:
err_create_device:
	devm_kfree(&info->client->dev, raw_data);
err_alloc2:
	devm_kfree(&info->client->dev, factory_info);
err_alloc1:

	return ret;
}
#endif /* end of SEC_FACTORY_TEST */

#ifdef CONFIG_OF
static const struct of_device_id tsp_dt_ids[] = {
	{ .compatible = "Zinitix,ztw522_ts", },
	{},
};
MODULE_DEVICE_TABLE(of, tsp_dt_ids);
#else
#define tsp_dt_ids NULL
#endif

static int ztw522_ts_parse_dt(struct device_node *np, struct device *dev,
					struct zxt_ts_platform_data *pdata)
{
	int ret;

	if (!np) {
		dev_err(dev, "%s: np is NULL.\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: ", __func__);

	/* gpio irq */
	pdata->gpio_int = of_get_named_gpio(np, "zinitix,irq-gpio", 0);
	if (pdata->gpio_int < 0) {
		dev_err(dev, "%s: failed to get irq number\n", __func__);
		return -EINVAL;
	}
	pr_cont("int:%d, ", pdata->gpio_int);

	ret = gpio_request(pdata->gpio_int, "ztw522_irq");
	if (ret < 0) {
		dev_err(dev, "%s: failed to request gpio_irq\n", __func__);
		return -EINVAL;
	}
	gpio_direction_input(pdata->gpio_int);

	/* gpio power enable */
	pdata->vdd_en = of_get_named_gpio(np, "zinitix,tsppwr_en", 0);
	if (pdata->vdd_en < 0)
		pdata->vdd_en = -1;
	pr_cont("vdd_en:%d, ", pdata->vdd_en);

	if (gpio_is_valid(pdata->vdd_en)) {
		ret = gpio_request(pdata->vdd_en, "ztw522_vdd_en");
		if (ret < 0) {
			dev_err(dev, "%s: failed to request gpio_vdd_en\n", __func__);
			return -EINVAL;
		}
	}

	ret = of_property_read_string(np, "ztw522,fw_name", &pdata->fw_name);
	if (ret < 0) {
		dev_err(dev, "%s: failed to get firmware path!\n", __func__);
		return -EINVAL;
	}
	pr_cont("fw_name:%s, ", pdata->fw_name);

	ret = of_property_read_u32(np, "ztw522,x_resolution", &pdata->x_resolution);
	if (ret < 0) {
		dev_err(dev, "%s: failed to get x_resolution\n", __func__);
		return ret;
	}
	pr_cont("max_x:%d, ", pdata->x_resolution);

	ret = of_property_read_u32(np, "ztw522,y_resolution", &pdata->y_resolution);
	if (ret < 0) {
		dev_err(dev, "%s: failed to get y_resolution\n", __func__);
		return ret;
	}
	pr_cont("max_y:%d, ", pdata->y_resolution);

	ret = of_property_read_string(np, "ztw522,model_name", &pdata->model_name);
	if (ret < 0) {
		pdata->model_name = "";
	}
	pr_cont("model:%s, ", pdata->model_name);

	pdata->reg_boot_on = of_property_read_bool(np, "ztw522,reg_boot_on");
	pr_cont("reg_boot_on:%d, ", pdata->reg_boot_on);

	pdata->tsp_power = ztw522_power;
	pr_cont("end\n");
	return 0;

}

#if ZINITIX_MISC_DEBUG
static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct ztw522_ts_info *info = misc_info;
	struct raw_ioctl raw_ioctl;
	struct reg_ioctl reg_ioctl;
	void __user *argp = (void __user *)arg;
	static int m_ts_debug_mode = ZINITIX_DEBUG;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 mode;
	u16 val;
	int nval = 0;

	if (!info) {
		dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
		return -1;
	}

	switch (cmd) {

	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			dev_err(&info->client->dev, "%s: copy_from_user\n", __func__);
			return -1;
		}
		if (nval)
			dev_err(&info->client->dev, "%s: on debug mode (%d)\n", __func__, nval);
		else
			dev_err(&info->client->dev, "%s: off debug mode (%d)\n", __func__, nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		dev_info(&info->client->dev, "%s: firmware size = %d\n", __func__, sz);
		if (info->cap_info.ic_fw_size != sz) {
			dev_err(&info->client->dev, "%s: firmware size error\n", __func__);
			return -1;
		}
		break;
/*
	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		if (copy_from_user(m_firmware_data,
			argp, info->cap_info.ic_fw_size))
			return -1;

		version = (u16) (m_firmware_data[52] | (m_firmware_data[53]<<8));

		dev_err(&info->client->dev, "%s: firmware version = %x\n", __func__, version);

		if (copy_to_user(argp, &version, sizeof(version)))
			return -1;
		break;

	case TOUCH_IOCTL_START_UPGRADE:
		return ztw522_upgrade_sequence(info, (u8 *)m_firmware_data);
*/
	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		disable_irq(info->irq);
		down(&info->work_lock);
		if (info->work_state != NOTHING) {
			dev_err(&info->client->dev, "%s: other process occupied.. (%d)\n",
				__func__, info->work_state);
			up(&info->work_lock);
			return -1;
		}
		info->work_state = HW_CALIBRAION;
		zinitix_delay(100);

		/* h/w calibration */
		if (ztw522_hw_calibration(info))
			ret = 0;

		ztw522_send_start_event(info);

		mode = info->touch_mode;
		if (ztw522_write_reg(info->client,
			ztw522_TOUCH_MODE, mode) != I2C_SUCCESS) {
			dev_err(&info->client->dev, "%s: failed to set touch mode %d.\n", __func__, mode);
			goto fail_hw_cal;
		}

		if (ztw522_write_cmd(info->client, ztw522_SWRESET_CMD) != I2C_SUCCESS)
			goto fail_hw_cal;

		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		enable_irq(info->irq);
		info->work_state = NOTHING;
		up(&info->work_lock);
		return ret;
fail_hw_cal:
		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		enable_irq(info->irq);
		info->work_state = NOTHING;
		up(&info->work_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (info == NULL) {
			dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
			return -1;
		}
		if (copy_from_user(&nval, argp, 4)) {
			dev_err(&info->client->dev, " %s: copy_from_user\n", __func__);
			info->work_state = NOTHING;
			return -1;
		}

		ztw522_send_start_event(info);
		ztw522_set_touchmode(info, (u16)nval);
		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		return 0;

	case TOUCH_IOCTL_GET_REG:
		if (info == NULL) {
			dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
			return -1;
		}
		down(&info->work_lock);
		if (info->work_state != NOTHING) {
			dev_err(&info->client->dev, "%s: other process occupied.. (%d)\n",
				__func__, info->work_state);
			up(&info->work_lock);
			return -1;
		}

		info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl,
			argp, sizeof(struct reg_ioctl))) {
			info->work_state = NOTHING;
			up(&info->work_lock);
			dev_err(&info->client->dev, " %s: copy_from_user(1)\n", __func__);
			return -1;
		}

		ztw522_send_start_event(info);

		if (ztw522_read_data(info->client,
			(u16)reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		nval = (int)val;

		if (copy_to_user((void *)reg_ioctl.val, (u8 *)&nval, 4)) {
			info->work_state = NOTHING;
			up(&info->work_lock);
			dev_err(&info->client->dev, " %s: copy_to_user(2)\n", __func__);
			return -1;
		}

		dev_err(&info->client->dev, "%s: reg addr = 0x%x, val = 0x%x\n",
			__func__, reg_ioctl.addr, nval);

		info->work_state = NOTHING;
		up(&info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		down(&info->work_lock);
		if (info->work_state != NOTHING) {
			dev_err(&info->client->dev, "%s: other process occupied.. (%d)\n",
				__func__, info->work_state);
			up(&info->work_lock);
			return -1;
		}

		info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct reg_ioctl))) {
			info->work_state = NOTHING;
			up(&info->work_lock);
			dev_err(&info->client->dev, " %s: copy_from_user(1)\n", __func__);
			return -1;
		}

		if (copy_from_user(&val, (void *)reg_ioctl.val, 4)) {
			info->work_state = NOTHING;
			up(&info->work_lock);
			dev_err(&info->client->dev, " %s: copy_from_user(2)\n", __func__);
			return -1;
		}

		ztw522_send_start_event(info);

		if (ztw522_write_reg(info->client,
			(u16)reg_ioctl.addr, val) != I2C_SUCCESS)
			ret = -1;

		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		dev_err(&info->client->dev, "%s: write: reg addr = 0x%x, val = 0x%x\n",
			__func__, reg_ioctl.addr, val);
		info->work_state = NOTHING;
		up(&info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:
		if (info == NULL) {
			dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
			return -1;
		}
		down(&info->work_lock);
		if (info->work_state != NOTHING) {
			dev_err(&info->client->dev, "%s: other process occupied.. (%d)\n",
				__func__, info->work_state);
			up(&info->work_lock);
			return -1;
		}

		ztw522_send_start_event(info);

		info->work_state = SET_MODE;
		if (ztw522_write_reg(info->client,
			ztw522_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			ret = -1;

		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		dev_err(&info->client->dev, "%s: write: reg addr = 0x%x, val = 0x0\n",
			__func__, ztw522_INT_ENABLE_FLAG);

		info->work_state = NOTHING;
		up(&info->work_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (info == NULL) {
			dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
			return -1;
		}
		down(&info->work_lock);
		if (info->work_state != NOTHING) {
			dev_err(&info->client->dev, "%s: other process occupied.." \
				"(%d)\n", __func__, info->work_state);
			up(&info->work_lock);
			return -1;
		}
		info->work_state = SET_MODE;
		ret = 0;

		ztw522_send_start_event(info);

		if (ztw522_write_cmd(info->client,
			ztw522_SAVE_STATUS_CMD) != I2C_SUCCESS)
			ret =  -1;

		zinitix_delay(1000);	/* for fusing eeprom */

		ztw522_write_cmd(info->client, ztw522_I2C_END_CMD);

		info->work_state = NOTHING;
		up(&info->work_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (info == NULL) {
			dev_err(&info->client->dev, "%s: misc device NULL?\n", __func__);
			return -1;
		}

		if (info->touch_mode == TOUCH_POINT_MODE)
			return -1;

		down(&info->raw_data_lock);
		if (info->update == 0) {
			up(&info->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(struct raw_ioctl))) {
			up(&info->raw_data_lock);
			dev_err(&info->client->dev, "%s: copy_from_user\n", __func__);
			return -1;
		}

		info->update = 0;

		u8Data = (u8 *)&info->cur_data[0];
		if (raw_ioctl.sz > MAX_TRAW_DATA_SZ * 2)
			raw_ioctl.sz = MAX_TRAW_DATA_SZ * 2;
		if (copy_to_user((void *)raw_ioctl.buf, (u8 *)u8Data, raw_ioctl.sz)) {
			up(&info->raw_data_lock);
			return -1;
		}

		up(&info->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};
#endif
#if 0
static int ztw522_pinctrl_configure(struct ztw522_ts_info *info,
							bool active)
{
	struct pinctrl_state *set_state_i2c;
	int retval;

	dev_err(&info->client->dev, "%s: %s\n", __func__, active ? "ACTIVE" : "SUSPEND");

	return 0;

	if (active) {
		set_state_i2c =
			pinctrl_lookup_state(info->pinctrl,
						"tsp_i2c_gpio_active");
		if (IS_ERR(set_state_i2c)) {
			dev_err(&info->client->dev, "%s: cannot get pinctrl(i2c) active state\n", __func__);
			return PTR_ERR(set_state_i2c);
		}
	} else {
		set_state_i2c =
			pinctrl_lookup_state(info->pinctrl,
						"tsp_i2c_suspend");
		if (IS_ERR(set_state_i2c)) {
			dev_err(&info->client->dev, "%s: cannot get pinctrl(i2c) sleep state\n", __func__);
			return PTR_ERR(set_state_i2c);
		}
	}

	retval = pinctrl_select_state(info->pinctrl, set_state_i2c);
	if (retval) {
		dev_err(&info->client->dev, "%s: cannot set pinctrl(i2c) %s state\n",
				__func__, active ? "active" : "suspend");
		return retval;
	}

	return 0;
}
#endif
static int ztw522_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct zxt_ts_platform_data *pdata = client->dev.platform_data;
	struct ztw522_ts_info *info;
	struct input_dev *input_dev;
	struct device_node *np = client->dev.of_node;
	int ret = -1;
#ifdef SUPPORTED_TOUCH_KEY
	int i;
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: Not compatible i2c function\n", __func__);
		ret = -EIO;
		goto err_no_platform_data;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct zxt_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		ret = ztw522_ts_parse_dt(np, &client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "%s: Failed ztw522_ts_parse_dt\n", __func__);
			goto err_no_platform_data;
		}
	}

	info = devm_kzalloc(&client->dev, sizeof(struct ztw522_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_no_platform_data;
	}

	i2c_set_clientdata(client, info);
	info->client = client;
	info->pdata = pdata;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "%s: Failed to allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	info->input_dev = input_dev;
	info->work_state = PROBE;

#if 0
	/* Get pinctrl if target uses pinctrl */
	info->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(info->pinctrl)) {
		if (PTR_ERR(info->pinctrl) == -EPROBE_DEFER) {
			dev_err(&client->dev, "%s: failed to get pinctrl\n", __func__);
			ret = -ENODEV;
			goto err_alloc;
		}

		dev_info(&client->dev, "%s: Target does not use pinctrl\n", __func__);
		info->pinctrl = NULL;
	}

	if (info->pinctrl) {
		ret = ztw522_pinctrl_configure(info, true);
		if (ret)
			dev_err(&client->dev, "%s: cannot set pinctrl state\n", __func__);
	}
#endif

	if (pdata->vdd_en) {
		ret = gpio_direction_output(pdata->vdd_en, 1);
		if (ret) {
			dev_err(&client->dev,"%s: unable to set_direction for zt_vdd_en [%d]\n",
				__func__, pdata->vdd_en);
			ret = -ENODEV;
			goto err_power_sequence;
		}
		zinitix_delay(CHIP_ON_DELAY);
	}

	if (pdata->vdd_en) {
		ret = gpio_direction_output(pdata->vdd_en, 0);
		if (ret) {
			dev_err(&client->dev,"%s: unable to set_direction for zt_vdd_en [%d]\n",
				__func__, pdata->vdd_en);
			ret = -ENODEV;
			goto err_power_sequence;
		}
		zinitix_delay(CHIP_OFF_DELAY);
	}

	/* power on */
	if (!ztw522_power_control(info, POWER_ON_SEQUENCE)) {
		ret = -EPERM;
		goto err_power_sequence;
	}

	memset(&info->reported_touch_info, 0x0, sizeof(struct point_info));
	sema_init(&info->work_lock, 1);

#if ZINITIX_MISC_DEBUG
	misc_info = info;
#endif

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;

	if (!ztw522_init_touch(info, false)) {
		ret = -EPERM;
		goto err_input_unregister_device;
	}

#if USE_TSP_TA_CALLBACKS
	info->callbacks.inform_charger = ztw522_ts_charger_status_cb;
	ztw522_register_callback(&info->callbacks);
#endif

	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));
	input_dev->name = SEC_TSP_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->phys = info->phys;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, info->input_dev->evbit);
	__set_bit(EV_KEY, info->input_dev->evbit);
	__set_bit(BTN_TOUCH, info->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);

	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers);

	init_waitqueue_head(&info->wait_q);
	device_init_wakeup(&client->dev, true);
	wake_lock_init(&info->wake_lock,
			WAKE_LOCK_SUSPEND, "TSP_wake_lock");

	if (pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_Y,
			info->cap_info.MinX, info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_X,
			info->cap_info.MinY, info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinX, info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinY, info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_X,
			info->cap_info.MinX, info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_Y,
			info->cap_info.MinY, info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinX, info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinY, info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);


	input_set_abs_params(info->input_dev, ABS_MT_TRACKING_ID, 0, MAX_SUPPORTED_FINGER_NUM, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_ORIENTATION, -128, 127, 0, 0);

	input_set_abs_params(info->input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

#if SUPPORTED_PALM_TOUCH
	input_set_abs_params(info->input_dev, ABS_MT_SUMSIZE, 0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PALM, 0, 1, 0, 0);
#endif

#ifdef REPORT_2D_Z
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
			0, REAL_Z_MAX, 0, 0);
#endif


#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		info->button[i] = ICON_BUTTON_UNCHANGE;

	info->led_ldo = devm_regulator_get(&client->dev, "key-led");
	if (IS_ERR(info->led_ldo)) {
		if (PTR_ERR(info->led_ldo) == -EPROBE_DEFER) {
			ret = -ENODEV;
			goto err_alloc;
		}

		dev_err(&client->dev, "%s: Target does not use KEY LED\n", __func__);
		info->led_ldo = NULL;
	}
	if (!IS_ERR_OR_NULL(info->led_ldo)) {
		ret = regulator_set_voltage(info->led_ldo, 3300000, 3300000);
		if (ret) {
			dev_err(&client->dev, "%s: could not set voltage led, ret = %d\n",
				__func__, ret);
		}
	}

	set_bit(LED_MISC, info->input_dev->ledbit);
	set_bit(EV_LED, info->input_dev->evbit);

	info->button_code[0] = KEY_PHONE;
	info->button_code[1] = KEY_BACK;

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(info->button_code[i], info->input_dev->keybit);
#endif


	info->input_dev->open = ztw522_input_open;
	info->input_dev->close = ztw522_input_close;

	input_set_drvdata(info->input_dev, info);

#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_register_ops(info, &ztw522_sleep_monitor_ops,
			SLEEP_MONITOR_TSP);
#endif

	ret = input_register_device(info->input_dev);
	if (ret) {
		dev_err(&info->client->dev, "%s: unable to register input device\n", __func__);
		goto err_input_register_device;
	}

	info->work_state = NOTHING;
	info->finger_cnt = 0;

	info->irq = gpio_to_irq(pdata->gpio_int);
	if (info->irq < 0) {
		dev_err(&client->dev, "%s: failed to get gpio_to_irq\n", __func__);
		ret = -ENODEV;
		goto err_gpio_irq;
	}
	ret = request_threaded_irq(info->irq, NULL, ztw522_touch_work,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT , SEC_TSP_NAME, info);

	if (ret) {
		dev_err(&client->dev, "%s: failed to request irq.\n", __func__);
		goto err_request_irq;
	}

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_enable(&client->dev);
#endif

	sema_init(&info->raw_data_lock, 1);

#if ZINITIX_MISC_DEBUG
	ret = misc_register(&touch_misc_device);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to register touch misc device\n", __func__);
		goto err_misc_register;
	}
#endif

#ifdef SEC_FACTORY_TEST
	ret = ztw522_init_sec_factory(info);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to init sec factory device\n", __func__);
		goto err_kthread_create_failed;
	}
#endif
	info->wait_until_wake = true;
	info->device_enabled = true;
	info->init_done = true;
	dev_info(&client->dev, "%s: done.\n", __func__);

	return 0;

#ifdef SEC_FACTORY_TEST
err_kthread_create_failed:
	devm_kfree(&client->dev, info->factory_info);
	devm_kfree(&client->dev, info->raw_data);
#endif
#if ZINITIX_MISC_DEBUG
err_misc_register:
#endif
	free_irq(info->irq, info);
err_request_irq:
err_gpio_irq:
	input_unregister_device(info->input_dev);
err_input_unregister_device:
err_input_register_device:
#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_unregister_ops(SLEEP_MONITOR_TSP);
#endif
#if USE_TSP_TA_CALLBACKS
	ztw522_register_callback(NULL);
#endif
	ztw522_power_control(info, POWER_OFF);
err_power_sequence:
	if (gpio_is_valid(pdata->gpio_int) != 0)
		gpio_free(pdata->gpio_int);
	if (gpio_is_valid(pdata->vdd_en) != 0)
		gpio_free(pdata->vdd_en);
	input_free_device(info->input_dev);
err_alloc:
	wake_lock_destroy(&info->wake_lock);
	devm_kfree(&client->dev, info);
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&client->dev, (void *)pdata);

	dev_err(&client->dev, "%s: Failed to probe\n", __func__);
	return ret;
}

static int ztw522_remove(struct i2c_client *client)
{
	struct ztw522_ts_info *info = i2c_get_clientdata(client);
	struct zxt_ts_platform_data *pdata = info->pdata;

	disable_irq(info->irq);
	down(&info->work_lock);

	info->work_state = REMOVE;
	wake_lock_destroy(&info->wake_lock);

#ifdef SEC_FACTORY_TEST
	devm_kfree(&client->dev, info->factory_info);
	devm_kfree(&client->dev, info->raw_data);
#endif
	if (info->irq)
		free_irq(info->irq, info);

#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_unregister_ops(SLEEP_MONITOR_TSP);
#endif
#if ZINITIX_MISC_DEBUG
	misc_deregister(&touch_misc_device);
#endif

	if (gpio_is_valid(pdata->gpio_int) != 0)
		gpio_free(pdata->gpio_int);

	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	up(&info->work_lock);
	devm_kfree(&client->dev, info);

	return 0;
}

void ztw522_shutdown(struct i2c_client *client)
{
	struct ztw522_ts_info *info = i2c_get_clientdata(client);

	disable_irq(info->irq);
	down(&info->work_lock);
	up(&info->work_lock);
	ztw522_power_control(info, POWER_OFF);
}

static struct i2c_device_id ztw522_idtable[] = {
	{ZTW522_TS_DEVICE, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ztw522_idtable);

const struct dev_pm_ops ztw522_ts_pm_ops = {
#ifdef CONFIG_PM
	.suspend = ztw522_suspend,
	.resume = ztw522_resume,
#endif
};

static struct i2c_driver ztw522_ts_driver = {
	.probe	= ztw522_probe,
	.remove	= ztw522_remove,
	.shutdown = ztw522_shutdown,
	.id_table	= ztw522_idtable,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= ZTW522_TS_DEVICE,
		.pm = &ztw522_ts_pm_ops,
		.of_match_table = tsp_dt_ids,
	},
};

extern unsigned int lpcharge;
static int __init ztw522_init(void)
{
	pr_info("%s: lpcharge: [%d]\n", __func__, lpcharge);

	if (lpcharge)
		return 0;
	else
		return i2c_add_driver(&ztw522_ts_driver);
}

static void __exit ztw522_exit(void)
{
	i2c_del_driver(&ztw522_ts_driver);
}

module_init(ztw522_init);
module_exit(ztw522_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("<mika.kim@samsung.com>");
MODULE_LICENSE("GPL");
