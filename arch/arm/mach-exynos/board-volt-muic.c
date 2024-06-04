/* 
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>
#include <linux/mfd/sm5504.h>
#include <mach/map.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>
#include "board-universal3250.h"

extern unsigned int system_rev;

#define GPIO_IF_PMIC_SDA	EXYNOS3_GPX2(2)
#define GPIO_IF_PMIC_SCL	EXYNOS3_GPX2(3)
#define GPIO_IF_PMIC_IRQ	EXYNOS3_GPX1(2)
#define I2C_IF_PMIC_ID		7

#define MFD_DEV_ADDR		(0x28 >> 1)
#define SYSREG_I2C_MUX		(S3C_VA_SYS + 0x0228)

static struct sm5504_platform_data b2_mfd_pdata = {
	.irq_base		= IRQ_BOARD_IFIC_START,
	.irq_gpio		= GPIO_IF_PMIC_IRQ,
	.wakeup			= true,
};

static struct i2c_board_info __initdata i2c_devs_if_pmic[] = {
	{
		I2C_BOARD_INFO("sm5504", MFD_DEV_ADDR),
		.platform_data = &b2_mfd_pdata,
	}
};

#ifdef CONFIG_MUIC_I2C_USE_S3C_DEV_I2C7
static struct s3c2410_platform_i2c __initdata i2c_data_if_pmic = {
	.bus_num	= I2C_IF_PMIC_ID,
	.flags		= 0,
	.slave_addr	= MFD_DEV_ADDR,
	.frequency	= 400*1000,
	.sda_delay	= 100,
};
#else
static struct i2c_gpio_platform_data gpio_i2c_if_pmic = {
	.sda_pin = GPIO_IF_PMIC_SDA,
	.scl_pin = GPIO_IF_PMIC_SCL,
};

static struct platform_device device_i2c_if_pmic = {
	.name = "i2c-gpio",
	.id = I2C_IF_PMIC_ID,
	.dev.platform_data = &gpio_i2c_if_pmic,
};
#endif /* CONFIG_MUIC_I2C_USE_S3C_DEV_I2C7 */

void __init exynos3_b2_mfd_init(void)
{
	if (system_rev < 0x03) {
#ifdef CONFIG_MUIC_I2C_USE_S3C_DEV_I2C7
		pr_info("%s: Use hw i2c 7\n", __func__);

		/*
		 * S3C_I2C7 dedicated to using internal block
		 * If want to use external I2C, set 0x0 to I2C_MUX register
		 */
		__raw_writel(0x0, SYSREG_I2C_MUX);
		s3c_i2c7_set_platdata(&i2c_data_if_pmic);
		i2c_register_board_info(I2C_IF_PMIC_ID,
			i2c_devs_if_pmic, ARRAY_SIZE(i2c_devs_if_pmic));
		platform_device_register(&s3c_device_i2c7);
#else

#ifdef CONFIG_USE_HW_I2C_MUIC
		s3c_i2c7_set_platdata(NULL);

		i2c_register_board_info(I2C_IF_PMIC_ID,
			i2c_devs_if_pmic, ARRAY_SIZE(i2c_devs_if_pmic));

		platform_device_register(&s3c_device_i2c7);
#else
		pr_info("%s: Use sw i2c gpio\n", __func__);

		s3c_gpio_cfgpin(GPIO_IF_PMIC_SDA, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(GPIO_IF_PMIC_SCL, S3C_GPIO_INPUT);

		s3c_gpio_setpull(GPIO_IF_PMIC_SDA, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(GPIO_IF_PMIC_SCL, S3C_GPIO_PULL_NONE);

		s3c_gpio_setpull(GPIO_IF_PMIC_IRQ, S3C_GPIO_PULL_NONE);


		i2c_register_board_info(I2C_IF_PMIC_ID,
			i2c_devs_if_pmic, ARRAY_SIZE(i2c_devs_if_pmic));

		platform_device_register(&device_i2c_if_pmic);
#endif

#endif /* CONFIG_MUIC_I2C_USE_S3C_DEV_I2C7 */
	} else {
		pr_info("%s: No use SM5504 muic\n", __func__);
	}
}
