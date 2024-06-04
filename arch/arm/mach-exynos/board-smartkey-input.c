/* linux/arch/arm/mach-exynos/board-smartkey-input.c
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

#include <linux/i2c/cyttsp5_core.h>

struct class *sec_class;
EXPORT_SYMBOL(sec_class);


#define GPIO_KEY_NAME	"gpio-keys"

#define GPIO_TSP_INT		EXYNOS3_GPX1(3)
#define GPIO_TSP_SDA		EXYNOS3_GPA0(6)
#define GPIO_TSP_SCL		EXYNOS3_GPA0(7)

#define GPIO_POWER_BUTTON	EXYNOS3_GPX2(7)
#define GPIO_BACK_BUTTON	EXYNOS3_GPX3(3)

struct tsp_callbacks *tsp_callbacks;
struct tsp_callbacks {
	void (*inform_charger) (struct tsp_callbacks *, bool);
};

#define CYTTSP5_I2C_TCH_ADR 0x24
#define I2C_CYTTSP5_ID 2
#define CYTTSP5_DEV_ADDR

#define CYTTSP5_HID_DESC_REGISTER 1

#define CY_MAXX 360
#define CY_MAXY 480
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF


extern int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata, struct device *dev);
extern int cyttsp5_power(struct cyttsp5_core_platform_data *pdata, int on, struct device *dev, atomic_t *ignore_irq);
extern int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata, struct device *dev);

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = GPIO_TSP_INT,
	.hid_desc_register = CYTTSP5_HID_DESC_REGISTER,
	.xres = cyttsp5_xres,
	.power = cyttsp5_power,
	.irq_stat = cyttsp5_irq_stat,
	.report_rate = 90,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp5_sett_param_regs, */
		NULL,	/* &cyttsp5_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		NULL, /* &cyttsp5_sett_btn_keys, */	/* button-to-keycode table */
	},
};

static const uint16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -128, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = 0,
	.inp_dev_name = "sec_touchscreen",
};

extern struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data;
static struct cyttsp5_platform_data _cyttsp5_platform_data = {
	.core_pdata = &_cyttsp5_core_platform_data,
	.mt_pdata = &_cyttsp5_mt_platform_data,
	.loader_pdata = &_cyttsp5_loader_platform_data,
};

static struct s3c2410_platform_i2c __initdata i2c_data_cyttsp5 = {
	.bus_num	= I2C_CYTTSP5_ID,
	.flags		= 0,
	.slave_addr	= CYTTSP5_I2C_TCH_ADR,
	.frequency	= 400*1000,
	.sda_delay	= 100,
};

static struct i2c_board_info i2c_devs2[] = {
	{
		I2C_BOARD_INFO("cyttsp5_i2c_adapter", CYTTSP5_I2C_TCH_ADR),
		.platform_data = &_cyttsp5_platform_data,
	},
};

void __init cypress_tsp_set_platdata(struct cyttsp5_platform_data *pdata)
{
	if (!pdata)
		pdata = &_cyttsp5_platform_data;

	i2c_devs2[0].platform_data = pdata;
}

void __init cypress_tsp_init(void)
{
	int ret;
	int irq_gpio;

	irq_gpio = GPIO_TSP_INT;


	_cyttsp5_core_platform_data.irq_gpio = irq_gpio;
	ret = gpio_request(irq_gpio, "TSP_INT");
	if (ret)
		pr_err("failed to request gpio(TSP_INT)(%d)\n", ret);

	s3c_gpio_cfgpin(irq_gpio, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(irq_gpio, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SDA, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL, S3C_GPIO_PULL_NONE);

	s5p_register_gpio_interrupt(irq_gpio);
	i2c_devs2[0].irq = gpio_to_irq(irq_gpio);

	pr_info("%s touch : %d\n", __func__, i2c_devs2[0].irq);
}

static void universal3250_gpio_keys_config_setup(void)
{
	int irq;

	s3c_gpio_cfgpin(GPIO_POWER_BUTTON, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_POWER_BUTTON, S3C_GPIO_PULL_NONE);

	irq = s5p_register_gpio_interrupt(GPIO_POWER_BUTTON);
	if (IS_ERR_VALUE(irq)) {
		pr_err("%s: Failed to configure POWER GPIO\n", __func__);
		return;
	}

	s3c_gpio_cfgpin(GPIO_BACK_BUTTON, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_BACK_BUTTON, S3C_GPIO_PULL_UP);

	irq = s5p_register_gpio_interrupt(GPIO_BACK_BUTTON);
	if (IS_ERR_VALUE(irq)) {
		pr_err("%s: Failed to configure BACK GPIO\n", __func__);
		return;
	}

}

static struct gpio_keys_button rev16_button[] = {
	{
		.code = KEY_POWER,
		.gpio = GPIO_POWER_BUTTON,
		.desc = "key_power",
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 20,
	},
	{
		.code = KEY_BACK,
		.gpio = GPIO_BACK_BUTTON,
		.desc = "key_back",
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 20,
	},
};

static struct gpio_keys_platform_data rev16_gpiokeys_pdata = {
	rev16_button,
	ARRAY_SIZE(rev16_button),
};

static struct platform_device rev16_gpio_keys = {
	.name	= GPIO_KEY_NAME,
	.dev	= {
		.platform_data = &rev16_gpiokeys_pdata,
	},
};


void __init exynos3_universal3250_input_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create sec_class.\n");

	platform_device_register(&s3c_device_i2c2);
	cypress_tsp_init();
	s3c_i2c2_set_platdata(&i2c_data_cyttsp5);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

	universal3250_gpio_keys_config_setup();

	platform_device_register(&rev16_gpio_keys);


}

