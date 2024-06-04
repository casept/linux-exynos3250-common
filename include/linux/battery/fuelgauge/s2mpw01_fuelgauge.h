/*
 * s2mpw01_fuelgauge.h
 * Samsung S2MPW01 Fuel Gauge Header
 *
 * Copyright (C) 2015 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __S2MPW01_FUELGAUGE_H
#define __S2MPW01_FUELGAUGE_H __FILE__

#if defined(ANDROID_ALARM_ACTIVATED)
#include <linux/android_alarm.h>
#endif
#include <linux/mfd/samsung/s2mpw01.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/battery/sec_charging_common.h>

/* Slave address should be shifted to the right 1bit.
 * R/W bit should NOT be included.
 */

#define S2MPW01_FG_REG_STATUS  0x00
#define S2MPW01_FG_REG_IRQ  0x02
#define S2MPW01_FG_REG_INTM  0x03
#define S2MPW01_FG_REG_RVBAT  0x04
#define S2MPW01_FG_REG_ROCV  0x06
#define S2MPW01_FG_REG_RSOC  0x08
#define S2MPW01_FG_REG_RTEMP  0x0A
#define S2MPW01_FG_REG_RZADJ_CHG2  0x0B
#define S2MPW01_FG_REG_RBATCAP  0x0C
#define S2MPW01_FG_REG_RZADJ  0x0E
#define S2MPW01_FG_REG_RZADJ_CHG  0x0F
#define S2MPW01_FG_REG_RBATZ0  0x10
#define S2MPW01_FG_REG_RBATZ1  0x12
#define S2MPW01_FG_REG_IRQ_LVL  0x14
#define S2MPW01_FG_REG_START  0x16
#define S2MPW01_FG_REG_MONOUT_CFG  0x19
#define S2MPW01_FG_REG_CURR  0x1A

struct sec_fg_info {
	/* test print count */
	int pr_cnt;
	/* full charge comp */
	/* struct delayed_work     full_comp_work; */
	u32 previous_fullcap;
	u32 previous_vffullcap;
	/* low battery comp */
	int low_batt_comp_flag;
	/* low battery boot */
	int low_batt_boot_flag;
	bool is_low_batt_alarm;

	/* battery info */
	u32 soc;

	/* miscellaneous */
	unsigned long fullcap_check_interval;
	int full_check_flag;
	bool is_first_check;
};

typedef struct s2mpw01_fuelgauge_platform_data {
	int capacity_max;
	int capacity_max_margin;
	int capacity_min;
	int capacity_calculation_type;
	int fuel_alert_soc;
	int fullsocthr;
	int fg_irq;

	char *fuelgauge_name;

	bool repeated_fuelalert;

	struct sec_charging_current *charging_current;

	u8 bat_param_normal[5];
	u8 bat_param_high[5];
	u8 bat_param_low[5];
	u8 bat_param_very_low[5];
} s2mpw01_fuelgauge_platform_data_t;


#endif /* __S2MPW01_FUELGAUGE_H */
