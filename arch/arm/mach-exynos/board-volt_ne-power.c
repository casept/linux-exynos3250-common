/* linux/arch/arm/mach-exynos/board-volt_ne-power.c
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/mfd/samsung/s2mpw01.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-pmu.h>
#include <mach/irqs.h>

#include <mach/devfreq.h>
#ifdef CONFIG_EXYNOS_THERMAL
#include <mach/tmu.h>
#endif

#ifdef CONFIG_KEYBOARD_S2MPW01
#include <linux/mfd/samsung/s2mpw01_keys.h>
#include <linux/input.h>
#endif

#ifdef CONFIG_FUELGAUGE_S2MPW01
#include <linux/battery/fuelgauge/s2mpw01_fuelgauge.h>
#endif

#ifdef CONFIG_CHARGER_S2MPW01
#include <linux/battery/charger/s2mpw01_charger.h>
#endif

#define ESPRESSO3250_PMIC_EINT	IRQ_EINT(7)

static struct regulator_consumer_supply s2mpw01_buck1_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);


static struct regulator_consumer_supply s2mpw01_buck2_consumer[] = {
	REGULATOR_SUPPLY("vdd_mif", NULL),
	REGULATOR_SUPPLY("vdd_int", NULL),
        REGULATOR_SUPPLY("vdd_g3d", NULL),
};

static struct regulator_consumer_supply s2mpw01_buck3_consumer[] = {
	REGULATOR_SUPPLY("buck3", NULL),
};

static struct regulator_consumer_supply s2mpw01_ldo2_consumer =
        REGULATOR_SUPPLY("vdd_pll_1v8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo3_consumer =
        REGULATOR_SUPPLY("vcc_ap_1.8v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo4_consumer =
        REGULATOR_SUPPLY("vap_mipi_1.0v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo5_consumer =
        REGULATOR_SUPPLY("vdd_ldo5 range", NULL);

static struct regulator_consumer_supply s2mpw01_ldo6_consumer =
        REGULATOR_SUPPLY("vap_usb_3.0v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo7_consumer =
        REGULATOR_SUPPLY("v_lpddr_1.2v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo10_consumer =
        REGULATOR_SUPPLY("vdd_sensor_io_1.8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo11_consumer =
        REGULATOR_SUPPLY("vdd_gps_2.8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo12_consumer =
        REGULATOR_SUPPLY("hrm_3.3", NULL);

static struct regulator_consumer_supply s2mpw01_ldo13_consumer =
        REGULATOR_SUPPLY("hrm_1.8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo14_consumer =
        REGULATOR_SUPPLY("tsp_avdd_3.0v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo15_consumer =
        REGULATOR_SUPPLY("vdd_a1p8_ap", NULL);

static struct regulator_consumer_supply s2mpw01_ldo16_consumer =
        REGULATOR_SUPPLY("vcc_lcd_3.0", NULL);

static struct regulator_consumer_supply s2mpw01_ldo17_consumer =
        REGULATOR_SUPPLY("vcc_lcd_1.8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo18_consumer =
        REGULATOR_SUPPLY("v_mot_2.7", NULL);

static struct regulator_consumer_supply s2mpw01_ldo19_consumer =
        REGULATOR_SUPPLY("baro_1.8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo20_consumer =
        REGULATOR_SUPPLY("tsp_vddo_1.8v", NULL);

static struct regulator_consumer_supply s2mpw01_ldo21_consumer =
        REGULATOR_SUPPLY("vdd_pll_a1p8", NULL);

static struct regulator_consumer_supply s2mpw01_ldo22_consumer =
        REGULATOR_SUPPLY("vdd_alive_1p0_ap", NULL);

static struct regulator_init_data s2m_buck1_data = {
	.constraints    = {
		.name           = "vdd_arm range",
		.min_uV         =  600000,
		.max_uV         = 1300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_buck1_consumer,
};

static struct regulator_init_data s2m_buck2_data = {
	.constraints    = {
		.name           = "vdd_mif range",
		.min_uV         =  600000,
		.max_uV         = 1300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies  = 3,
	.consumer_supplies      = &s2mpw01_buck2_consumer[0],
};

static struct regulator_init_data s2m_buck3_data = {
	.constraints    = {
		.name           = "buck3",
		.min_uV         = 1200000,
		.max_uV         = 2100000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_buck3_consumer[0],
};

static struct regulator_init_data s2m_ldo2_data = {
	.constraints	= {
		.name		= "vdd_ldo2 range",
		.min_uV		= 400000,
		.max_uV		= 1350000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mpw01_ldo2_consumer,
};



static struct regulator_init_data s2m_ldo3_data = {
	.constraints    = {
		.name           = "vcc_ap_1.8v",
		.min_uV         = 800000,
		.max_uV         = 2000000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo3_consumer,
};

static struct regulator_init_data s2m_ldo4_data = {
	.constraints	= {
		.name		= "vap_mipi_1.0v",
		.min_uV		= 800000,
		.max_uV		= 1200000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.apply_uV       = 1,
		.always_on 	= 1,
		.boot_on 	= 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &s2mpw01_ldo4_consumer,
};

static struct regulator_init_data s2m_ldo5_data = {
	.constraints	= {
		.name		= "vdd_ldo5 range",
		.min_uV		= 800000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &s2mpw01_ldo5_consumer,
};

static struct regulator_init_data s2m_ldo6_data = {
	.constraints    = {
		.name           = "vap_usb_3.0v",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo6_consumer,
};

static struct regulator_init_data s2m_ldo7_data = {
	.constraints	= {
		.name		= "v_lpddr_1.2v",
		.min_uV		= 800000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &s2mpw01_ldo7_consumer,
};

#if 0
static struct regulator_init_data s2m_ldo8_data = {
	.constraints    = {
		.name           = "vap_usb_3.0v",
		.min_uV         = 800000,
		.max_uV         = 2000000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo8_consumer,
};

static struct regulator_init_data s2m_ldo9_data = {
	.constraints    = {
		.name           = "v_lpddr_1.2v",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo9_consumer,
};
#endif

static struct regulator_init_data s2m_ldo10_data = {
	.constraints    = {
		.name           = "vdd_sensor_io_1.8",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo10_consumer,
};

static struct regulator_init_data s2m_ldo11_data = {
	.constraints    = {
		.name           = "vdd_gps_2.8",
		.min_uV         = 2800000,
		.max_uV         = 2800000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo11_consumer,
};
static struct regulator_init_data s2m_ldo12_data = {
	.constraints    = {
		.name           = "hrm_3.3",
		.min_uV         = 3300000,
		.max_uV         = 3300000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo12_consumer,
};

static struct regulator_init_data s2m_ldo13_data = {
	.constraints    = {
		.name           = "hrm_1.8",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo13_consumer,
};

static struct regulator_init_data s2m_ldo14_data = {
	.constraints    = {
		.name           = "tsp_avdd_3.0v",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo14_consumer,
};

static struct regulator_init_data s2m_ldo15_data = {
	.constraints    = {
		.name           = "vdd_a1p8_ap",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo15_consumer,
};

static struct regulator_init_data s2m_ldo16_data = {
	.constraints    = {
		.name           = "vcc_lcd_3.0",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo16_consumer,
};

static struct regulator_init_data s2m_ldo17_data = {
	.constraints    = {
		.name           = "vcc_lcd_1.8",
		.min_uV         = 800000,
		.max_uV         = 2000000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo17_consumer,
};

static struct regulator_init_data s2m_ldo18_data = {
        .constraints    = {
                .name           = "v_mot_2.7",
                .min_uV         = 1800000,
                .max_uV         = 3375000,
                .apply_uV       = 1,
                .always_on      = 0,
                .boot_on        = 0,
                .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = &s2mpw01_ldo18_consumer,
};

#if 0
static struct regulator_init_data s2m_ldo18_data = {
	.constraints    = {
		.name           = "cam_af_2.8",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo18_consumer,
};
#endif

static struct regulator_init_data s2m_ldo19_data = {
	.constraints    = {
		.name           = "baro_1.8",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo19_consumer,
};

static struct regulator_init_data s2m_ldo20_data = {
	.constraints    = {
		.name           = "tsp_vddo_1.8v",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 0,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo20_consumer,
};

static struct regulator_init_data s2m_ldo21_data = {
	.constraints    = {
		.name           = "vdd_pll_a1p8",
		.min_uV         = 800000,
		.max_uV         = 2000000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo21_consumer,
};


static struct regulator_init_data s2m_ldo22_data = {
	.constraints    = {
		.name           = "vdd_alive_1p0_ap",
		.min_uV         = 800000,
		.max_uV         = 1200000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo22_consumer,
};

#if 0
static struct regulator_init_data s2m_ldo23_data = {
	.constraints    = {
		.name           = "vdd_sensor_io_1.8",
		.min_uV         = 1800000,
		.max_uV         = 3375000,
		.apply_uV       = 1,
		.always_on      = 1,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s2mpw01_ldo23_consumer,
};
#endif



static struct sec_regulator_data exynos_regulators[] = {
	{S2MPW01_BUCK1, &s2m_buck1_data},
	{S2MPW01_BUCK2, &s2m_buck2_data},
	{S2MPW01_BUCK3, &s2m_buck3_data},
	{S2MPW01_LDO2, &s2m_ldo2_data},
	{S2MPW01_LDO3, &s2m_ldo3_data},
	{S2MPW01_LDO4, &s2m_ldo4_data},
	{S2MPW01_LDO5, &s2m_ldo5_data},
	{S2MPW01_LDO6, &s2m_ldo6_data},
	{S2MPW01_LDO7, &s2m_ldo7_data},
	{S2MPW01_LDO10, &s2m_ldo10_data},
	{S2MPW01_LDO11, &s2m_ldo11_data},
	{S2MPW01_LDO12, &s2m_ldo12_data},
	{S2MPW01_LDO13, &s2m_ldo13_data},
	{S2MPW01_LDO14, &s2m_ldo14_data},
	{S2MPW01_LDO15, &s2m_ldo15_data},
	{S2MPW01_LDO16, &s2m_ldo16_data},
	{S2MPW01_LDO17, &s2m_ldo17_data},
	{S2MPW01_LDO18, &s2m_ldo18_data},
	{S2MPW01_LDO19, &s2m_ldo19_data},
	{S2MPW01_LDO20, &s2m_ldo20_data},
	{S2MPW01_LDO21, &s2m_ldo21_data},
	{S2MPW01_LDO22, &s2m_ldo22_data},
};

struct sec_opmode_data s2mpw01_opmode_data[S2MPW01_REG_MAX] = {
	[S2MPW01_BUCK1] = {S2MPW01_BUCK1, SEC_OPMODE_STANDBY},
	[S2MPW01_BUCK2] = {S2MPW01_BUCK2, SEC_OPMODE_STANDBY},
	[S2MPW01_BUCK3] = {S2MPW01_BUCK3, SEC_OPMODE_LP},
	[S2MPW01_LDO2] = {S2MPW01_LDO2, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO3] = {S2MPW01_LDO3, SEC_OPMODE_LP},
	[S2MPW01_LDO4] = {S2MPW01_LDO4, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO5] = {S2MPW01_LDO5, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO6] = {S2MPW01_LDO6, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO7] = {S2MPW01_LDO7, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO10] = {S2MPW01_LDO10, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO11] = {S2MPW01_LDO11, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO12] = {S2MPW01_LDO12, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO13] = {S2MPW01_LDO13, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO14] = {S2MPW01_LDO14, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO15] = {S2MPW01_LDO15, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO16] = {S2MPW01_LDO16, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO17] = {S2MPW01_LDO17, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO18] = {S2MPW01_LDO18, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO19] = {S2MPW01_LDO19, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO20] = {S2MPW01_LDO20, SEC_OPMODE_NORMAL},
	[S2MPW01_LDO21] = {S2MPW01_LDO21, SEC_OPMODE_STANDBY},
	[S2MPW01_LDO22] = {S2MPW01_LDO22, SEC_OPMODE_LP},
};

static int sec_cfg_irq(void)
{
	unsigned int pin = irq_to_gpio(ESPRESSO3250_PMIC_EINT);

	s3c_gpio_cfgpin(pin, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pin, S3C_GPIO_PULL_UP);

	return 0;
}


#ifdef CONFIG_KEYBOARD_S2MPW01
static struct power_keys_button power_button[] = {
	{
		.code = KEY_POWER,
		.wakeup = true,
		.desc = "KEY_POWER",
	},
};

static struct power_keys_platform_data keys_pdata = {
	.buttons = power_button,
	.nbuttons = ARRAY_SIZE(power_button),
	.name = "s2mpw01-power-keys",
};
#endif

#ifdef CONFIG_FUELGAUGE_S2MPW01
#define GPIO_FUEL_nALRT		EXYNOS3_GPX1(5)

static s2mpw01_fuelgauge_platform_data_t fuelgauge_pata = {
	.fuelgauge_name = "fuelgauge_name",
	.fg_irq = GPIO_FUEL_nALRT,
	.fuel_alert_soc = 2,
	.repeated_fuelalert = false,
	.capacity_calculation_type =
		SEC_FUELGAUGE_CAPACITY_TYPE_RAW |
		SEC_FUELGAUGE_CAPACITY_TYPE_SCALE |
		SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE |
		SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL,
	.capacity_max = 1000,
	.capacity_max_margin = 40,
	.capacity_min = -8,

	.bat_param_normal = {0x2E, 0xC0, 0x40, 0xE0, 0x10},
	.bat_param_high = {0x80, 0xE0, 0x92, 0xE0, 0x00},
	.bat_param_low = {0x00, 0x40, 0x72, 0xC0, 0x20},
	.bat_param_very_low = {0x00, 0x40, 0x72, 0xB0, 0x20},
};
#endif

#ifdef CONFIG_CHARGER_S2MPW01

sec_charging_current_t charging_current_table_retail[] = {
	{75,	75,	100,	100},
	{0,	0,	0,	0},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{0,	0,	0,	0},
	{75,	75,	100,	100},
	{75,	75,	100,	100},
	{0,	0,	0,	0},
	{0,	0,	0,	0},
};

sec_charging_current_t charging_current_table[] = {
	{150,	150,	25,	10},
	{0,	0,	0,	0},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{0,	0,	0,	0},
	{150,	150,	25,	10},
	{150,	150,	25,	10},
	{0,	0,	0,	0},
	{0,	0,	0,	0},
};

static s2mpw01_charger_platform_data_t charger_pata = {
	.charging_current_table = charging_current_table,
	.charging_current_table_normal = charging_current_table,
	.charging_current_table_retail = charging_current_table_retail,
	.chg_float_voltage = 4400,
	.chg_recharge_voltage = 4150,
	.charger_name = "sec-charger",
	.topoff_timer_enable = 1,
	.full_check_type = SEC_BATTERY_FULLCHARGED_CHGPSY,
	/* 2nd full check */
	.full_check_type_2nd = SEC_BATTERY_FULLCHARGED_CHGPSY,
};
#endif

static struct mfd_cell s2mpw01_devs[] = {
#ifdef CONFIG_REGULATOR_S2MPW01
	{ .name = "s2mpw01-pmic", },
#endif
#ifdef CONFIG_RTC_DRV_S2MP
	{ .name = "s2mp-rtc", },
#endif
#ifdef CONFIG_KEYBOARD_S2MPW01
	{ .name = "s2mpw01-power-keys",
	   .platform_data = &keys_pdata,
	   .pdata_size = sizeof(keys_pdata),
	 },
#endif
#ifdef CONFIG_CHARGER_S2MPW01
	{ .name = "s2mpw01-charger",
		.platform_data = &charger_pata,
		.pdata_size = sizeof(charger_pata),

},
#endif
#ifdef CONFIG_FUELGAUGE_S2MPW01
	{ .name = "s2mpw01-fuelgauge",
	   .platform_data = &fuelgauge_pata,
	   .pdata_size = sizeof(fuelgauge_pata),
},
#endif
};

static struct sec_pmic_platform_data exynos3_s2m_pdata = {
	.device_type		= S2MPW01,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(exynos_regulators),
	.regulators		= exynos_regulators,
	.cfg_pmic_irq		= sec_cfg_irq,
	.wakeup		= 1,
	.wtsr_smpl		= 1,
	.opmode_data		= s2mpw01_opmode_data,
	.buck1234_ramp_delay	= 12,
	.using_rid_detect = true,
	.num_mfd_devs	= ARRAY_SIZE(s2mpw01_devs),
	.mfd_devs = s2mpw01_devs,
};

struct s3c2410_platform_i2c i2c_data_s2mpw01 __initdata = {
	.flags          = 0,
	.slave_addr     = 0x66,
	.frequency      = 400*1000,
	.sda_delay      = 100,
};

static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("sec-pmic", 0x66 >> 1),
		.platform_data	= &exynos3_s2m_pdata,
		.irq		= ESPRESSO3250_PMIC_EINT,
	},
};

#ifdef CONFIG_EXYNOS_THERMAL
static struct exynos_tmu_platform_data exynos3_tmu_data = {
	.trigger_levels[0] = 80,
	.trigger_levels[1] = 85,
	.trigger_levels[2] = 100,
	.trigger_levels[3] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 0,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 900 * 1000,
		.temp_level = 80,
	},
	.freq_tab[1] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 85,
	},
	.freq_tab[2] = {
		.freq_clip_max = 700 * 1000,
		.temp_level = 90,
	},
	.freq_tab[3] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 95,
	},
	.freq_tab[4] = {
		.freq_clip_max = 500 * 1000,
		.temp_level = 100,
	},
	.size[THERMAL_TRIP_ACTIVE] = 4,
	.size[THERMAL_TRIP_PASSIVE] = 1,
	.freq_tab_count = 5,
	.type = SOC_ARCH_EXYNOS3,
	.clk_name = "tmu_apbif",

	/* workaround for uncalibrated chip list */
	.uncalibrated_chips = {
		"NZ4RZ",
		"NZ4T1",
		"NZ4U6",
		"NZ4U7",
		"NZ4UN",
		NULL,
	},
};
#endif /* CONFIG_EXYNOS_THERMAL */

#ifdef CONFIG_PM_DEVFREQ
static struct platform_device exynos3250_mif_devfreq = {
	.name	= "exynos3250-busfreq-mif",
	.id	= -1,
};

static struct exynos_devfreq_platdata universal3250_qos_int_pd __initdata = {
	/* locking the INT min level to L5 : 50000MHz */
	.default_qos = 50000,
};
static struct platform_device exynos3250_int_devfreq = {
	.name	= "exynos3250-busfreq-int",
	.id	= -1,
};

static struct exynos_devfreq_platdata universal3250_qos_mif_pd __initdata = {
	/* locking the MIF min level to L4 : 50000MHz */
	.default_qos = 50000,
};
#endif



static struct platform_device *universal3250_power_devices[] __initdata = {
#ifdef CONFIG_PM_DEVFREQ
	&exynos3250_mif_devfreq,
	&exynos3250_int_devfreq,
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos3250_device_tmu,
#endif
	&s3c_device_i2c0,
};

extern unsigned int system_rev;
void __init exynos3_universal3250_power_init(void)
{
	s3c_i2c0_set_platdata(&i2c_data_s2mpw01);
#ifdef CONFIG_PM_DEVFREQ
	s3c_set_platdata(&universal3250_qos_int_pd,
			sizeof(struct exynos_devfreq_platdata),
			&exynos3250_int_devfreq);
	s3c_set_platdata(&universal3250_qos_mif_pd,
			sizeof(struct exynos_devfreq_platdata),
			&exynos3250_mif_devfreq);
#endif

	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

#ifdef CONFIG_EXYNOS_THERMAL
	exynos_tmu_set_platdata(&exynos3_tmu_data);
#endif

	platform_add_devices(universal3250_power_devices,
			ARRAY_SIZE(universal3250_power_devices));
}
