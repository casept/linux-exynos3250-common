#ifndef __K6DS3_INPUT_H_
#define __K6DS3_INPUT_H_

/*register of K6DS3TR*/
#define 	FUNC_CFG_ACCESS	0x01
#define 	SENSOR_SYNC_TIME_FRAME	0x04
#define 	SENSOR_SYNC_ENABLE	0x05
#define 	FIFO_CTRL1	0x06
#define 	FIFO_CTRL2	0x07
#define 	FIFO_CTRL3	0x08
#define 	FIFO_CTRL4	0x09
#define	FIFO_CTRL5	0x0A
#define 	ORIENT_CFG_G	0x0B
#define 	INT1_CTRL	0x0D
#define 	INT2_CTRL	0x0E
#define 	WHO_AM_I	0x0F
#define 	CTRL1_XL	0x10
#define 	CTRL2_G		0x11
#define 	CTRL3_C		0x12
#define	CTRL4_C		0x13
#define	CTRL5_C		0x14
#define	CTRL6_G		0x15
#define	CTRL7_G		0x16
#define	CTRL8_XL	0x17
#define	CTRL9_XL	0x18
#define	CTRL10_C	0x19
#define	MASTER_CONFIG 0x1A
#define	WAKE_UP_SRC	0x1B
#define	TAP_SRC		0x1C
#define	D6D_SRC		0x1D
#define	STATUS_REG	0x1E
#define	OUT_TEMP_L_ADDR	0x20
#define	OUTX_L_G	0x22
#define	OUTX_H_G	0x23
#define	OUTY_L_G	0x24
#define	OUTY_H_G	0x25
#define	OUTZ_L_G	0x26
#define	OUTZ_H_G	0x27
#define	OUTX_L_XL	0x28
#define	OUTX_H_XL	0x29
#define	OUTY_L_XL	0x2A
#define	OUTY_H_XL	0x2B
#define	OUTZ_L_XL	0x2C
#define	OUTZ_H_XL	0x2D
#define 	SENSORHUB1_REG	0x2E
#define	SENSORHUB2_REG	0x2F
#define	SENSORHUB3_REG	0x30
#define	SENSORHUB4_REG	0x31
#define	SENSORHUB5_REG	0x32
#define	SENSORHUB6_REG	0x33
#define	SENSORHUB7_REG	0x34
#define	SENSORHUB8_REG	0x35
#define	SENSORHUB9_REG	0x36
#define	SENSORHUB10_REG 0x37
#define	SENSORHUB11_REG	0x38
#define	SENSORHUB12_REG	0x39
#define	FIFO_STATUS1	0x3A
#define	FIFO_STATUS2	0x3B
#define	FIFO_STATUS3	0x3C
#define	FIFO_STATUS4	0x3D
#define	FIFO_DATA_OUT_L	0x3E
#define	FIFO_DATA_OUT_H	0x3F
#define	TIMESTAMP0_REG	0x40
#define	TIMESTAMP1_REG	0x41
#define	TIMESTAMP2_REG	0x42
#define	STEP_COUNTERR_L	0x4B
#define	STEP_COUNTERR_H	0x4C
#define	SENSORHUB13_REG	0x4D
#define	SENSORHUB14_REG	0x4E
#define	SENSORHUB15_REG	0x4F
#define	SENSORHUB16_REG	0x50
#define	SENSORHUB17_REG	0x51
#define	SENSORHUB18_REG	0x52
#define	FUNC_SRC		0x53
#define	TAP_CFG			0x58
#define	TAP_THS_6D		0x59
#define	INT_DUR2		0x5A
#define	WAKE_UP_THS		0x5B
#define	WAKE_UP_DUR		0x5C
#define	FREE_FALL		0x5D
#define	MD1_CFG			0x5E
#define	MD2_CFG			0x5F


/*selft test mode for accel sensor*/
#define ST_NORMAL_MODE_ACC	0
#define ST_POSITIVE_MODE_ACC	1
#define ST_NOT_ALLOWED_MODE_ACC	2
#define ST_NEGATIVE_MODE_ACC	3

/*selft-test mode for accel sensor*/
#define ST_NORMAL_MODE_GYRO	0
#define ST_POSITIVE_MODE_GYRO	1
#define ST_NAGATIVE_MODE_GYRO	2
#define ST_NOT_ALLOWED_MODE_GYRO	3


/*self-test mode for gyro sensor*/
#define ST_LSM6DS3_FIFO_TEST_DEPTH 32
#define ST_LSM6DS3_DIS_BIT	0x00
#define ST_LSM6DS3_EN_BIT		0x01

/* ZRL: 40dps @ 2000dps */
#define ST_LSM6DS3_GYR_MIN_ZRL			((int)(-40/0.07f) - 1)
#define ST_LSM6DS3_GYR_MAX_ZRL			((int)(40/0.07f) + 1)
#define ST_LSM6DS3_GYR_ZRL_DELTA			((int)(6/0.07f) + 1)
/* Selftest: 250~700dps @ 2000dps */
#define ST_LSM6DS3_GYR_MIN_ST			((int)(200/0.07f))
#define ST_LSM6DS3_GYR_MAX_ST			((int)(700/0.07f) + 1)
#define ST_LSM6DS3_GYR_DA_RETRY_COUNT			5


/* configure for interrupt */
#define INT1_DRDY_XL	1
#define INT1_DRDY_G	2
#define INT1_BOOT		4
#define INT1_FTH		8
#define INT1_FIFO_OVR	16
#define INT1_FULL_FLAG	32
#define INT1_SIGN_MOT	64
#define INT1_PEDO		128

#define INT2_DRDY_XL	1
#define INT2_DRDY_G	2
#define INT2_FTH		8
#define INT2_FIFO_OVR	16
#define INT2_FULL_FLAG	32
#define INT2_SIGN_MOT	64
#define INT2_PEDO		128


struct k6ds3_platform_data {
	int position;
	int irq_gpio;
	int int_selection;
};

#endif