#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>
#include <plat/devs.h>
#include <mach/hs-iic.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <asm/system_info.h>

#include "board-universal3250.h"

#include <linux/ssp_platformdata.h>
#include <linux/sensor/k6ds3.h>

#define GPIO_GYRO_INT		EXYNOS3_GPE0(4)
#define GPIO_GYRO_SDA		EXYNOS3_GPD1(2)
#define GPIO_GYRO_SCL		EXYNOS3_GPD1(3)

struct k6ds3_platform_data k6ds3tr_data = {
	.position = 3,
	.irq_gpio = GPIO_GYRO_INT,
	.int_selection = 1,
};

static struct i2c_board_info i2c_devs_acc_sensor[] = {
		{
			I2C_BOARD_INFO("k6ds3", 0x6B),
			.platform_data = &k6ds3tr_data,
		},
};




void __init exynos3_sensor_init(void)
{
	/*register accel, gyro sensor*/
	s3c_gpio_setpull(GPIO_GYRO_SDA, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_GYRO_SCL, S3C_GPIO_PULL_NONE);
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs_acc_sensor,
		ARRAY_SIZE(i2c_devs_acc_sensor));
	platform_device_register(&s3c_device_i2c1);
}

