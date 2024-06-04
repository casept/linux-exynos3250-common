/* linux/arch/arm/mach-exynos/board-orbis-input.c
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/regs-gpio.h>
#include "board-universal3250.h"
#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
#include <mach/regs-pmu.h>
#include <linux/io.h>
#include <linux/i2c/zxt_ztw522_ts.h>
#endif

#ifndef CONFIG_SEC_SYSFS
struct class *sec_class;
EXPORT_SYMBOL(sec_class);
#endif

#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
#define ZTW522_I2C_ADDR		0x20
#define GPX1_3_WAKEUP_MASK_BIT	0x800

extern int ztw522_power(struct i2c_client *client, int on);
static void ztw522_gpio_config(int on);

static struct zxt_ts_platform_data zxt_ts_pdata = {
	.gpio_int = EXYNOS3_GPX1(3),
	.gpio_scl = EXYNOS3_GPA0(7),
	.gpio_sda = EXYNOS3_GPA0(6),
	.gpio_reset = EXYNOS3_GPX1(7),
	.vdd_en = 0,
	.gpio_ldo_en = 0,
	.tsp_power = ztw522_power,
	.gpio_config = ztw522_gpio_config,
	.x_resolution = 216,
	.y_resolution = 432,
	.fw_name = "zinitix_ts.fw",
	.model_name = "ztw522",
	.page_size = 0,
	.orientation = 0,
	.tsp_supply_type = 0,
	.core_num = 0,
	.reg_boot_on = 0,
};

static struct i2c_board_info zxt_ts_i2c_dev[] = {
	{
		I2C_BOARD_INFO(ZTW522_TS_DEVICE, ZTW522_I2C_ADDR),
		.platform_data = &zxt_ts_pdata,
	},
};

static void ztw522_gpio_config(int on)
{
	unsigned int eint_wakeup_mask =  __raw_readl(EXYNOS_EINT_WAKEUP_MASK);

	printk("%s:%s\n", __func__, on ? "resume":"suspend");

	if (on) {
		s3c_gpio_cfgpin(zxt_ts_pdata.gpio_int, S3C_GPIO_SFN(0xf));
		eint_wakeup_mask = eint_wakeup_mask & (~(GPX1_3_WAKEUP_MASK_BIT));
		s3c_gpio_setpull(zxt_ts_pdata.gpio_int, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(zxt_ts_pdata.gpio_reset, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(zxt_ts_pdata.gpio_reset, S3C_GPIO_PULL_NONE);
		gpio_set_value(zxt_ts_pdata.gpio_reset, 1);
	} else {
		s3c_gpio_cfgpin(zxt_ts_pdata.gpio_reset, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(zxt_ts_pdata.gpio_reset, S3C_GPIO_PULL_NONE);
		gpio_set_value(zxt_ts_pdata.gpio_reset, 0);
		s3c_gpio_cfgpin(zxt_ts_pdata.gpio_int, S3C_GPIO_INPUT);
		eint_wakeup_mask = eint_wakeup_mask | GPX1_3_WAKEUP_MASK_BIT;
		s3c_gpio_setpull(zxt_ts_pdata.gpio_int, S3C_GPIO_PULL_NONE);
	}

	__raw_writel(eint_wakeup_mask, EXYNOS_EINT_WAKEUP_MASK);
}
#endif
#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_BACK_BUTTON	EXYNOS3_GPX3(3)
static struct gpio_keys_button keys_button[] = {
	{
		.code = KEY_BACK,
		.gpio = GPIO_BACK_BUTTON,
		.desc = "KEY_BACK",
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 20,
	},
};

static struct gpio_keys_platform_data keys_pdata = {
	keys_button,
	ARRAY_SIZE(keys_button),
};

static struct platform_device gpio_keys_pdev = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data = &keys_pdata,
	},
};

static void gpio_keys_config_setup(void)
{
	int irq;

	s3c_gpio_cfgpin(GPIO_BACK_BUTTON, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_BACK_BUTTON, S3C_GPIO_PULL_UP);

	irq = s5p_register_gpio_interrupt(GPIO_BACK_BUTTON);
	if (IS_ERR_VALUE(irq)) {
		pr_err("%s: Failed to configure BACK GPIO\n", __func__);
		return;
	}
}
#endif

void __init exynos3_universal3250_input_init(void)
{
#ifndef CONFIG_SEC_SYSFS
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("%s: Failed to create sec_class.\n", __func__);
#endif
#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
	platform_device_register(&s3c_device_i2c2);
	s3c_i2c2_set_platdata(NULL);
	ztw522_gpio_config(false);
	i2c_register_board_info(2, zxt_ts_i2c_dev, ARRAY_SIZE(zxt_ts_i2c_dev));
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	gpio_keys_config_setup();
	platform_device_register(&gpio_keys_pdev);
#endif
}
