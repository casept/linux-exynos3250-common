/* drivers/battery/s2mpw01_charger.c
 * S2MPW01 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/battery/sec_charging_common.h>
#include <linux/mfd/samsung/s2mpw01.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/mfd/samsung/s2mpw01-irq.h>
#include <linux/battery/charger/s2mpw01_charger.h>
#ifdef CONFIG_SWITCH
#include <linux/misc/muic.h>
#include <linux/switch.h>
extern struct sec_switch_data switch_data;
#endif

extern unsigned int batt_booting_chk;

#define ENABLE_MIVR 1

#define MINVAL(a, b) ((a <= b) ? a : b)

#define EOC_DEBOUNCE_CNT 2
#define HEALTH_DEBOUNCE_CNT 3
#define DEFAULT_CHARGING_CURRENT 500

#define EOC_SLEEP 200
#define EOC_TIMEOUT (EOC_SLEEP * 6)
#ifndef EN_TEST_READ
#define EN_TEST_READ 1
#endif

struct s2mpw01_charger_data {
	struct sec_pmic_dev *iodev;
	struct i2c_client       *client;
	struct device *dev;
	//struct s2mpw01_platform_data *s2mpw01_pdata;
	struct delayed_work	charger_work;
	struct delayed_work init_work;
	struct delayed_work usb_work;
	struct delayed_work rid_work;
	struct delayed_work ta_work;
	struct workqueue_struct *charger_wqueue;
	struct power_supply	psy_chg;
	s2mpw01_charger_platform_data_t *pdata;
	int dev_id;
	int charging_current;
	int siop_level;
	int cable_type;
	bool is_charging;
	bool is_usb_ready;
	struct mutex io_lock;

	/* register programming */
	int reg_addr;
	int reg_data;

	bool full_charged;
	bool ovp;
	bool factory_mode;

	int unhealth_cnt;
	int status;

	/* charger enable, disable data */
	unsigned int chg_en_data;

	/* s2mpw01 */
	int irq_det_bat;
	int irq_chg;
	int irq_tmrout;

	void (*init_callback)(void);
	void (*dock_callback)(uint8_t attached);
	void (*cable_chg_callback)(int32_t cable_type);
	void (*ocp_callback)(void);
	void (*otp_callback)(void);
	void (*ovp_callback)(void);
	void (*usb_callback)(uint8_t attached);
	void (*uart_callback)(uint8_t attached);
	void (*otg_callback)(uint8_t attached);
	void (*jig_callback)(uint8_t attached);
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
	void (*ps_cable_callback)(uint8_t attached);
#endif

	int irq_uart_off;
	int irq_uart_on;
	int irq_usb_off;
	int irq_usb_on;
	int irq_uart_cable;
	int irq_fact_leakage;
	int irq_jigon;
	int irq_acokf;
	int irq_acokr;
};

static enum power_supply_property sec_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_USB_OTG,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int s2mpw01_get_charging_health(struct s2mpw01_charger_data *charger);

static void s2mpw01_test_read(struct sec_pmic_dev *iodev)
{
	unsigned int data;
	char str[1016] = {0,};
	int i;

	for (i = 0x0; i <= 0x11; i++) {
		sec_chg_read(iodev, i, &data);

		sprintf(str+strlen(str), "0x%02x:0x%02x, ", i, data);
	}

	pr_err("[DEBUG]%s: %s\n", __func__, str);
}

static void s2mpw01_enable_charger_switch(struct s2mpw01_charger_data *charger,
		int onoff)
{
	unsigned int data = 0;
	unsigned int acok_stat = 0;

	data = charger->chg_en_data;

	/* ACOK status */
	sec_pmic_read(charger->iodev, S2MPW01_PMIC_REG_STATUS1, &acok_stat);
	pr_err("[DEBUG]%s: onoff[%d], chg_en_data[0x%x], acok[0x%x]\n",
		__func__, onoff, charger->chg_en_data, acok_stat);

	if (onoff > 0) {
		pr_err("[DEBUG]%s: turn on charger\n", __func__);
		sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL1, EN_CHG_MASK, EN_CHG_MASK);
	} else {
		charger->full_charged = false;
		pr_err("[DEBUG] %s: turn off charger\n", __func__);

		if (!charger->factory_mode) {
			data |= 0x40;
			sec_chg_write(charger->iodev, 0x2E, data);
			sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL1, 0, EN_CHG_MASK);
		}

		/* ACOK status : high > keep charger off, low > charger on */
		if (!(acok_stat & ACOK_STATUS_MASK)) {
			data = charger->chg_en_data;
			sec_chg_write(charger->iodev, 0x2E, data);
			sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL1, EN_CHG_MASK, EN_CHG_MASK);
			pr_err("[DEBUG] %s: turn on charger for RID detection\n", __func__);
		}
	}
}

static void s2mpw01_topoff_interrupt_onoff(struct s2mpw01_charger_data *charger, int onoff)
{
	if (onoff > 0) {
		/* Use top-off interrupt. Masking off */
		sec_chg_update(charger->iodev, S2MPW01_CHG_REG_INT1M, 0x00, 0x04);
		charger->iodev->irq_masks_cur[3] &= ~0x04;
		pr_err("[DEBUG]%s: Use top-off interrupt: 0x%x\n", __func__, charger->iodev->irq_masks_cur[3]);
		pm_stay_awake(charger->dev);
		s2mpw01_enable_charger_switch(charger, false);
		msleep(100);
		s2mpw01_enable_charger_switch(charger, true);
		pm_relax(charger->dev);
	} else {
		/* Not use top-off interrupt. Masking */
		sec_chg_update(charger->iodev, S2MPW01_CHG_REG_INT1M, 0x04, 0x04);
		charger->iodev->irq_masks_cur[3] |= 0x04;
		pr_err("[DEBUG]%s: Top-off interrupt Masking: 0x%x\n", __func__, charger->iodev->irq_masks_cur[3]);
	}

}

static void s2mpw01_set_regulation_voltage(struct s2mpw01_charger_data *charger,
		int float_voltage)
{
	unsigned int data;

	pr_err("[DEBUG]%s: float_voltage %d\n", __func__, float_voltage);
	if (float_voltage <= 4200)
		data = 0;
	else if (float_voltage > 4200 && float_voltage <= 4550)
		data = (float_voltage - 4200) / 50;
	else
		data = 0x7;

	sec_chg_update(charger->iodev,
			S2MPW01_CHG_REG_CTRL5, data << SET_VF_VBAT_SHIFT, SET_VF_VBAT_MASK);
}

static void s2mpw01_set_fast_charging_current(struct sec_pmic_dev *iodev,
		int charging_current)
{
	unsigned int data;

	pr_err("[DEBUG]%s: fast charge current  %d\n", __func__, charging_current);
	if (charging_current <= 75)
		data = 0x6;
	else if (charging_current <= 150)
		data = 0;
	else if (charging_current <= 175)
		data = 0x7;
	else if (charging_current > 175 && charging_current <= 400)
		data = (charging_current - 150) / 50;
	else
		data = 0x5;

	sec_chg_update(iodev, S2MPW01_CHG_REG_CTRL2, data << FAST_CHARGING_CURRENT_SHIFT,
			FAST_CHARGING_CURRENT_MASK);
}

static int s2mpw01_get_fast_charging_current(struct sec_pmic_dev *iodev)
{
	int ret;
	unsigned int data;

	ret = sec_chg_read(iodev, S2MPW01_CHG_REG_CTRL2, &data);
	if (ret < 0)
		return ret;

	data = (data & FAST_CHARGING_CURRENT_MASK) >> FAST_CHARGING_CURRENT_SHIFT;

	if (data <= 0x5)
		data = data * 50 + 150;
	else if (data == 0x06)
		data = 75;
	else if (data == 0x07)
		data = 175;
	return data;
}

int eoc_current[16] =
{ 5,10,12,15,20,17,25,30,35,40,50,60,70,80,90,100,};

static int s2mpw01_get_current_eoc_setting(struct s2mpw01_charger_data *charger)
{
	int ret;
	unsigned int data;

	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_CTRL4, &data);
	if (ret < 0)
		return ret;

	data = data & FIRST_TOPOFF_CURRENT_MASK;

	if (data > 0x0f)
		data = 0x0f;

	pr_err("[DEBUG]%s: top-off current  %d\n", __func__, eoc_current[data]);

	return eoc_current[data];
}

static void s2mpw01_set_topoff_current(struct sec_pmic_dev *iodev, int current_limit)
{
	unsigned int data;

	if (current_limit <= 5)
		data = 0;
	else if (current_limit > 5 && current_limit <= 10)
		data = (current_limit - 5) / 5;
	else if (current_limit > 10 && current_limit < 18)
		data = (current_limit - 10) / 5 * 2 + 1;
	else if (current_limit >= 18 && current_limit < 20)
		data = 5;	  /* 17.5 mA */
	else if (current_limit >= 20 && current_limit < 25)
		data = 4;
	else if (current_limit >= 25 && current_limit <= 40)
		data = (current_limit - 25) / 5 + 6;
	else if (current_limit > 40 && current_limit <= 100)
		data = (current_limit - 40) / 10 + 9;
	else
		data = 0x0F;

	pr_err("[DEBUG]%s: top-off current	%d, data=0x%x\n", __func__, current_limit, data);

	sec_chg_update(iodev, S2MPW01_CHG_REG_CTRL4, data << FIRST_TOPOFF_CURRENT_SHIFT,
			FIRST_TOPOFF_CURRENT_MASK);
}

/* eoc reset */
static void s2mpw01_set_charging_current(struct s2mpw01_charger_data *charger)
{
	int adj_current = 0;

	pr_err("[DEBUG]%s: charger->siop_level  %d\n", __func__, charger->siop_level);
	adj_current = charger->charging_current * charger->siop_level / 100;
	s2mpw01_set_fast_charging_current(charger->iodev, adj_current);
}

enum {
	S2MPW01_MIVR_4200MV = 0,
	S2MPW01_MIVR_4300MV,
	S2MPW01_MIVR_4400MV,
	S2MPW01_MIVR_4500MV,
	S2MPW01_MIVR_4600MV,
	S2MPW01_MIVR_4700MV,
	S2MPW01_MIVR_4800MV,
	S2MPW01_MIVR_4900MV,
};

#if ENABLE_MIVR
/* charger input regulation voltage setting */
static void s2mpw01_set_mivr_level(struct s2mpw01_charger_data *charger)
{
	int mivr = S2MPW01_MIVR_4600MV;

	sec_chg_update(charger->iodev,
			S2MPW01_CHG_REG_CTRL4, mivr << SET_VIN_DROP_SHIFT, SET_VIN_DROP_MASK);
}
#endif /*ENABLE_MIVR*/

static void s2mpw01_configure_charger(struct s2mpw01_charger_data *charger)
{
	struct device *dev = charger->dev;
	int eoc = 0;
	union power_supply_propval chg_mode;

	dev_err(dev, "%s() set configure charger \n", __func__);

	if (charger->charging_current < 0) {
		dev_info(dev, "%s() OTG is activated. Ignore command!\n",
				__func__);
		return;
	}

	if (!charger->pdata->charging_current_table) {
		dev_err(dev, "%s() table is not exist\n",__func__);
		return;
	}

#if ENABLE_MIVR
	s2mpw01_set_mivr_level(charger);
#endif /*DISABLE_MIVR*/

	/* msleep(200); */

	s2mpw01_set_regulation_voltage(charger,
			charger->pdata->chg_float_voltage);

	charger->charging_current = charger->pdata->charging_current_table
		[charger->cable_type].fast_charging_current;

	s2mpw01_set_charging_current(charger);

	dev_err(dev, "%s: full_check_type [%d][%d]\n", __func__,
			charger->pdata->full_check_type, charger->pdata->full_check_type_2nd);

	if (charger->pdata->full_check_type == SEC_BATTERY_FULLCHARGED_CHGPSY) {
		if (charger->pdata->full_check_type_2nd == SEC_BATTERY_FULLCHARGED_CHGPSY) {
				psy_do_property("battery", get,
						POWER_SUPPLY_PROP_CHARGE_NOW,
						chg_mode);

				if (chg_mode.intval == SEC_BATTERY_CHARGING_2ND) {
					//s2mpw01_enable_charger_switch(charger, 0);
					charger->full_charged = false;
					eoc = charger->pdata->charging_current_table
						[charger->cable_type].full_check_current_2nd;
				} else {
					eoc = charger->pdata->charging_current_table
						[charger->cable_type].full_check_current_1st;
				}
			} else {
				eoc = charger->pdata->charging_current_table
					[charger->cable_type].full_check_current_1st;
			}
		s2mpw01_set_topoff_current(charger->iodev, eoc);

		/* use TOP-OFF interrupt */
		schedule_delayed_work(&charger->ta_work, msecs_to_jiffies(200));
	}
	s2mpw01_enable_charger_switch(charger, 1);
}

static void s2mpw01_set_recharge_voltage(struct s2mpw01_charger_data *charger,
		int recharge_voltage)
{
	unsigned int data;
	dev_info(charger->dev, "%s : recharge voltage: %dmV\n",
				__func__, recharge_voltage);
	if (recharge_voltage <= 4150)
		data = 0;
	else if (recharge_voltage > 4150 && recharge_voltage <= 4500)
		data = (recharge_voltage - 4150) / 50;
	else
		data = 0x7;

	/* Enable recharge voltage setting bit */
	data |= EN_RECHARGE_VOLTAGE_MASK;
	sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL3,
			data, EN_RECHARGE_VOLTAGE_MASK | RECHARGE_VOLTAGE_MASK);
}

/* here is set init charger data */
static bool s2mpw01_chg_init(struct s2mpw01_charger_data *charger)
{
	dev_info(charger->dev, "%s : DEV ID : 0x%x\n", __func__,
			charger->dev_id);

	/* change Top-off detection debounce time (0x56 to 0x76) */
	sec_chg_write(charger->iodev, 0x2C, 0x76);

#if !(ENABLE_MIVR)
	/* voltage regulatio disable does not exist mu005 */
#endif

	if (!charger->pdata->topoff_timer_enable) {
		dev_info(charger->dev, "%s: Top-off timer disable\n", __func__);
		sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL8,
				NO_TIMEOUT_30M_MASK, NO_TIMEOUT_30M_MASK);
	}

	if (charger->pdata->chg_recharge_voltage)
		s2mpw01_set_recharge_voltage(charger,
					charger->pdata->chg_recharge_voltage);

	/* Factory_mode initialization */
	charger->factory_mode = false;

	return true;
}

static int s2mpw01_get_charging_status(struct s2mpw01_charger_data *charger)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int ret;
	unsigned int chg_sts;
	struct device *dev = charger->dev;

	dev_err(dev, "%s() get charging status \n", __func__);
	s2mpw01_test_read(charger->iodev);

	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS1, &chg_sts);
	if (ret < 0)
		return status;
	dev_info(charger->dev, "%s : charger status : 0x%x\n", __func__, chg_sts);

	if (charger->full_charged) {
			dev_info(charger->dev, "%s : POWER_SUPPLY_STATUS_FULL : 0x%x\n", __func__, chg_sts);
			return POWER_SUPPLY_STATUS_FULL;
	}

	switch (chg_sts & 0x12) {
	case 0x00:
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case 0x10:	/*charge state */
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x12:	/* Input is invalid */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		break;
	}

	return status;
}

static int s2mpw01_get_charge_type(struct sec_pmic_dev *iodev)
{
	int status = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int ret;
	unsigned int data;

	ret = sec_chg_read(iodev, S2MPW01_CHG_REG_STATUS1, &data);
	if (ret < 0) {
		pr_err("{DEBUG] %s : status read fail\n", __func__);
		return ret;
	}

	switch (data & CHG_STS_STATUS_MASK) {
	case 0x10:
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		/* 005 does not need to do this */
		/* pre-charge mode */
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	}

	return status;
}

static bool s2mpw01_get_batt_present(struct sec_pmic_dev *iodev)
{
	int ret;
	unsigned int data;

	ret = sec_chg_read(iodev, S2MPW01_CHG_REG_STATUS2, &data);
	if (ret < 0)
		return false;

	return (data & DET_BAT_STATUS_MASK) ? true : false;
}

static int s2mpw01_get_charging_health(struct s2mpw01_charger_data *charger)
{
	int ret;
	unsigned int data;

	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS1, &data);

	if (ret < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (data & CHGVIN_STATUS_MASK) {
		charger->ovp = false;
		return POWER_SUPPLY_HEALTH_GOOD;
	}

	/* 005 need to check ovp & health count */
	charger->unhealth_cnt = HEALTH_DEBOUNCE_CNT;
	if (charger->ovp)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
}

static int sec_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct s2mpw01_charger_data *charger =
		container_of(psy, struct s2mpw01_charger_data, psy_chg);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->charging_current ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mpw01_get_charging_status(charger);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s2mpw01_get_charging_health(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 2000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (charger->charging_current) {
			val->intval = s2mpw01_get_fast_charging_current(charger->iodev);
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = s2mpw01_get_charge_type(charger->iodev);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = charger->pdata->chg_float_voltage;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s2mpw01_get_batt_present(charger->iodev);
		break;

	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = charger->is_charging;
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sec_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct s2mpw01_charger_data *charger =
		container_of(psy, struct s2mpw01_charger_data, psy_chg);
	struct device *dev = charger->dev;
	int eoc;
/*	int previous_cable_type = charger->cable_type; */

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		charger->status = val->intval;
		break;
		/* val->intval : type */
	case POWER_SUPPLY_PROP_ONLINE:
		charger->cable_type = val->intval;
		if (charger->cable_type == POWER_SUPPLY_TYPE_BATTERY ||
				charger->cable_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			dev_info(dev, "%s() [BATT] Type Battery\n", __func__);
#if 0
			if (!charger->pdata->charging_current_table)
				return -EINVAL;

			charger->charging_current = charger->pdata->charging_current_table
					[POWER_SUPPLY_TYPE_USB].fast_charging_current;

			s2mpw01_set_charging_current(charger);
			s2mpw01_set_topoff_current(charger->iodev,
					charger->pdata->charging_current_table
					[POWER_SUPPLY_TYPE_USB].full_check_current_1st);
#endif
			charger->is_charging = false;
			charger->full_charged = false;
			s2mpw01_enable_charger_switch(charger, 0);
		} else if (charger->cable_type == POWER_SUPPLY_TYPE_OTG) {
			dev_info(dev, "%s() OTG mode not supported\n", __func__);
		} else {
			dev_info(dev, "%s() Set charging, Cable type = %d\n",
				 __func__, charger->cable_type);
			charger->is_charging = true;
			/* Enable charger */
			s2mpw01_configure_charger(charger);
		}
#if EN_TEST_READ
		msleep(100);
		s2mpw01_test_read(charger->iodev);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		dev_info(dev, "%s() is_charging %d\n", __func__, charger->is_charging);
		/* set charging current */
		if (charger->is_charging) {
			/* decrease the charging current according to siop level */
			charger->siop_level = val->intval;
			dev_info(dev, "%s() SIOP level = %d, chg current = %d\n", __func__,
					val->intval, charger->charging_current);
			eoc = s2mpw01_get_current_eoc_setting(charger);
			s2mpw01_set_charging_current(charger);
			//s2mpw01_set_topoff_current(charger->iodev, 0);
		}
		break;
	/* val->intval : charging current */
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		dev_info(dev, "%s() set current[%d]\n", __func__, val->intval);
		charger->charging_current = val->intval;
		s2mpw01_set_charging_current(charger);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:

		dev_info(dev, "%s() float voltage(%d)\n", __func__, val->intval);
		charger->pdata->chg_float_voltage = val->intval;
		s2mpw01_set_regulation_voltage(charger,
				charger->pdata->chg_float_voltage);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		eoc = s2mpw01_get_current_eoc_setting(charger);
		dev_info(dev, "%s() Set Power Now -> chg current = %d mA, eoc = %d mA\n",
				__func__, val->intval, eoc);
		s2mpw01_set_charging_current(charger);
		//s2mpw01_set_topoff_current(charger->iodev, 0);
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		dev_err(dev, "%s() OTG mode not supported\n", __func__);
		/* s2mpw01_charger_otg_control(charger, val->intval); */
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		dev_info(dev, "%s() CHARGING_ENABLE\n", __func__);
		/* charger->is_charging = val->intval; */
		s2mpw01_enable_charger_switch(charger, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if(val->intval == SEC_BATTERY_RETAIL_MODE) {
			charger->pdata->chg_float_voltage = 4000;
			charger->pdata->full_check_type= SEC_BATTERY_FULLCHARGED_SOC;
			charger->pdata->full_check_type_2nd = SEC_BATTERY_FULLCHARGED_SOC;
			charger->pdata->charging_current_table = charger->pdata->charging_current_table_retail;
			s2mpw01_set_regulation_voltage(charger, charger->pdata->chg_float_voltage);
		} else {
			charger->pdata->chg_float_voltage = 4400;
			charger->pdata->full_check_type= SEC_BATTERY_FULLCHARGED_CHGPSY;
			charger->pdata->full_check_type_2nd = SEC_BATTERY_FULLCHARGED_CHGPSY;
			charger->pdata->charging_current_table = charger->pdata->charging_current_table_normal;
			s2mpw01_set_regulation_voltage(charger, charger->pdata->chg_float_voltage);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#if 0
/* s2mpw01 interrupt service routine */
static irqreturn_t s2mpw01_det_bat_isr(int irq, void *data)
{
	struct s2mpw01_charger_data *charger = data;
	unsigned int val;

	sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &val);
	if ((val & DET_BAT_STATUS_MASK) == 0) {
		s2mpw01_enable_charger_switch(charger, 0);
		pr_err("charger-off if battery removed\n");
	}
	return IRQ_HANDLED;
}
#endif

static void s2mpw01_factory_mode_setting(struct s2mpw01_charger_data *charger)
{
	unsigned int stat_val;

	/* ACOK status */
	stat_val = charger->chg_en_data;
	sec_chg_write(charger->iodev, 0x2E, stat_val);
	s2mpw01_enable_charger_switch(charger, true);
	charger->factory_mode = true;
	pr_err("%s, factory mode\n", __func__);
}

static irqreturn_t s2mpw01_chg_isr(int irq, void *data)
{
	struct s2mpw01_charger_data *charger = data;
	unsigned int val;

	sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS1, &val);
	pr_info("[DEBUG]%s , %02x\n " , __func__, val);

	if (val & TOP_OFFSTATUS_MASK) {
		pr_info("%s : top_off status!!\n", __func__);
		charger->full_charged = true;
		/* TOP-OFF interrupt masking */
		s2mpw01_topoff_interrupt_onoff(charger, 0);
	}

	return IRQ_HANDLED;
}

static irqreturn_t s2mpw01_tmrout_isr(int irq, void *data)
{
	struct s2mpw01_charger_data *charger = data;
	unsigned int val;

	sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &val);
	if (val & 0x10) {
		/* Timer out status */
		pr_err("%s, fast-charging timeout, timer clear\n", __func__);
		s2mpw01_enable_charger_switch(charger, false);
		msleep(100);
		s2mpw01_enable_charger_switch(charger, true);
	}
	return IRQ_HANDLED;
}

static void s2mpw01_muic_init_detect(struct work_struct *work)
{
	struct s2mpw01_charger_data *charger =
		container_of(work, struct s2mpw01_charger_data, init_work.work);

	int ret;
	unsigned int status, chg_sts2, chg_sts3;

	/* check when booting after USB connected */
	ret = sec_pmic_read(charger->iodev, S2MPW01_PMIC_REG_STATUS1, &status);
	if (ret < 0) {
		pr_err("{DEBUG] %s : pm status read fail\n", __func__);
	}
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status2 read fail\n", __func__);
	}
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status3 read fail\n", __func__);
	}

	pr_info("%s: pm status, chg status3: 0x%x,0x%x,0x%x\n", __func__, status, chg_sts2, chg_sts3);
	if(status & ACOK_STATUS_MASK) {
		if(chg_sts3 & UART_CABLE_STATUS_MASK) {
			if (charger->usb_callback && charger->is_usb_ready)
				charger->usb_callback(1);

			if (charger->cable_chg_callback)
				charger->cable_chg_callback(MUIC_CABLE_TYPE_USB);

			pr_info("%s: USB connected\n", __func__);
		} else {
			if (!(chg_sts2 & JIGON_STATUS_MASK)) {
				if (charger->cable_chg_callback)
					charger->cable_chg_callback(MUIC_CABLE_TYPE_REGULAR_TA);
				pr_info("%s: TA connected\n", __func__);
			} else {
				if (charger->jig_callback)
					charger->jig_callback(1);
				pr_info("%s: JIG connected\n", __func__);
			}
		}
	}
}

static void s2mpw01_ta_detect(struct work_struct *work)
{
	struct s2mpw01_charger_data *charger =
		container_of(work, struct s2mpw01_charger_data, ta_work.work);

	if (charger->is_charging) {
		if (batt_booting_chk)
			s2mpw01_topoff_interrupt_onoff(charger, 1);
	}
}

static void s2mpw01_muic_usb_detect(struct work_struct *work)
{
	struct s2mpw01_charger_data *charger =
		container_of(work, struct s2mpw01_charger_data, usb_work.work);

	int ret;
	unsigned int status, chg_sts2, chg_sts3;

	charger->is_usb_ready = true;
	/* check when booting after USB connected */
	ret = sec_pmic_read(charger->iodev, S2MPW01_PMIC_REG_STATUS1, &status);
	if (ret < 0) {
		pr_err("{DEBUG] %s : pm status read fail\n", __func__);
	}
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status2 read fail\n", __func__);
	}
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status3 read fail\n", __func__);
	}

	pr_info("%s: pm status, chg status2 status3: 0x%x,0x%x,0x%x\n", __func__, status, chg_sts2, chg_sts3);
	if(status & ACOK_STATUS_MASK) {
		if(chg_sts3 & UART_CABLE_STATUS_MASK) {
			if (charger->usb_callback && charger->is_usb_ready)
				charger->usb_callback(1);

			pr_info("%s: USB connected\n", __func__);
		}
	} else if(chg_sts2 & USB_BOOT_ON_STATUS_MASK) {
		if (charger->usb_callback && charger->is_usb_ready)
				charger->usb_callback(1);
		pr_info("%s: JIG USB connected\n", __func__);
	}
}

static void s2mpw01_muic_rid_check(struct work_struct *work)
{
	struct s2mpw01_charger_data *charger =
		container_of(work, struct s2mpw01_charger_data, rid_work.work);
	unsigned int chg_sts2, chg_sts3;

	sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
	sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
	pr_info("%s: acokr irq stat2: 0x%x stat3: 0x%x\n", __func__, chg_sts2, chg_sts3);

	if (!((chg_sts2 & USB_BOOT_ON_STATUS_MASK) || (chg_sts2 & USB_BOOT_OFF_STATUS_MASK) ||
		(chg_sts3 & UART_BOOT_ON_STATUS_MASK) || (chg_sts2 & UART_BOOT_OFF_STATUS_MASK) ||
		(chg_sts3 & UART_CABLE_STATUS_MASK))) {
		charger->factory_mode = false;
		pr_err("%s: factory mode[%d]\n", __func__, charger->factory_mode);
	}

}

static irqreturn_t s2mpw01_muic_isr(int irq, void *data)
{
	unsigned int chg_sts2, chg_sts3;
	struct s2mpw01_charger_data *charger = data;
	unsigned int stat_val;

	pr_info("%s: irq:%d\n", __func__, irq);

	if(irq == charger->irq_acokr) {
		/* ACOK status */
		if (!charger->factory_mode) {
			stat_val = charger->chg_en_data;
			stat_val |= 0x40;
			sec_chg_write(charger->iodev, 0x2E, stat_val);
		}

		/* acokr */
		sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
		sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
		pr_info("%s: acokr irq stat2: 0x%x stat3: 0x%x\n", __func__, chg_sts2, chg_sts3);
		if(chg_sts3 & UART_CABLE_STATUS_MASK) {
			if (charger->usb_callback && charger->is_usb_ready)
				charger->usb_callback(1);

			if (charger->cable_chg_callback)
				charger->cable_chg_callback(MUIC_CABLE_TYPE_USB);

			pr_info("%s: USB connected. status3: 0x%x\n", __func__, chg_sts3);
		} else {
			if (!(chg_sts2 & JIGON_STATUS_MASK)) {
				if (charger->cable_chg_callback)
					charger->cable_chg_callback(MUIC_CABLE_TYPE_REGULAR_TA);
				pr_info("%s: TA connected\n", __func__);
			} else {
				if ((chg_sts2 & USB_BOOT_ON_STATUS_MASK) || (chg_sts2 & USB_BOOT_OFF_STATUS_MASK) ||
					(chg_sts3 & UART_BOOT_ON_STATUS_MASK) || (chg_sts2 & UART_BOOT_OFF_STATUS_MASK)) {
					if (charger->jig_callback)
						charger->jig_callback(1);
				}
				if ((chg_sts2 & USB_BOOT_ON_STATUS_MASK) ||
					(chg_sts2 & USB_BOOT_OFF_STATUS_MASK)) {
					if (charger->usb_callback && charger->is_usb_ready)
						charger->usb_callback(1);
				}
				pr_info("%s: JIG connected.\n", __func__);
			}
		}
	} else if(irq == charger->irq_acokf) {
		/* ACOK status */
		stat_val= charger->chg_en_data;
		sec_chg_write(charger->iodev, 0x2E, stat_val);

		/* acokf */
		sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
		sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
		pr_info("%s: acokf irq:sts2[0x%x],sts3[0x%x]\n", __func__, chg_sts2, chg_sts3);

		if (charger->usb_callback)
			charger->usb_callback(0);

		if (charger->cable_chg_callback)
			charger->cable_chg_callback(MUIC_CABLE_TYPE_NONE);

		/* TOP-OFF interrupt masking */
		s2mpw01_topoff_interrupt_onoff(charger, 0);
#if !defined(CONFIG_SLP_KERNEL_ENG)
		if (charger->jig_callback)
			charger->jig_callback(0);
#endif
	} else if(irq == charger->irq_usb_on) {
		/* usb boot on */
		pr_info("%s: usb boot on irq\n", __func__);
		if (charger->jig_callback)
			charger->jig_callback(1);
		if (charger->usb_callback && charger->is_usb_ready)
			charger->usb_callback(1);
		s2mpw01_factory_mode_setting(charger);
	} else if(irq == charger->irq_uart_off) {
		/* uart boot off */
		pr_info("%s: uart boot off irq:%d\n", __func__, irq);
		s2mpw01_factory_mode_setting(charger);
	} else if(irq == charger->irq_uart_on) {
		/* uart boot on */
		pr_info("%s: uart boot on irq:%d\n", __func__, irq);
		s2mpw01_factory_mode_setting(charger);
	} else if(irq == charger->irq_usb_off) {
		/* usb boot off */
		pr_info("%s: usb boot off irq:%d\n", __func__, irq);
		s2mpw01_factory_mode_setting(charger);
	} else if(irq == charger->irq_jigon) {
		schedule_delayed_work(&charger->rid_work, msecs_to_jiffies(200));

		/* jigon */
		pr_info("%s: jigon irq:%d\n", __func__, irq);
	}
#if 0
	else if(irq == charger->irq_fact_leakage) {

		/* fact leakage */
		pr_info("%s: fact leakage irq:%d\n", __func__, irq);
	} else if(irq == charger->irq_uart_cable) {

		/* uart cable */
		pr_info("%s: uart cable irq:%d\n", __func__, irq);
	}
#endif

	return IRQ_HANDLED;
}

#define REQUEST_IRQ(_irq, _dev_id, _name)				\
do {									\
	ret = request_threaded_irq(_irq, NULL, s2mpw01_muic_isr,	\
				0, _name, _dev_id);	\
	if (ret < 0) {							\
		pr_err("%s: Failed to request s2mpw01 muic IRQ #%d: %d\n",		\
				__func__, _irq, ret);	\
		_irq = 0;						\
	}								\
} while (0)

static int s2mpw01_muic_irq_init(struct s2mpw01_charger_data *charger)
{
	int ret = 0;

	if (charger->iodev && (charger->iodev->irq_base > 0)) {
		int irq_base = charger->iodev->irq_base;

		/* request MUIC IRQ */
#if 0
		charger->irq_fact_leakage = irq_base + S2MPW01_CHG_IRQ_FACT_LEAKAGE_INT3;
		REQUEST_IRQ(charger->irq_fact_leakage, charger, "muic-fact_leakage");

		charger->irq_uart_cable = irq_base + S2MPW01_CHG_IRQ_UART_CABLE_INT3;
		REQUEST_IRQ(charger->irq_uart_cable, charger, "muic-uart_cable");
#endif
		charger->irq_jigon = irq_base + S2MPW01_CHG_IRQ_JIGON_INT2;
		REQUEST_IRQ(charger->irq_jigon, charger, "muic-jigon");

		charger->irq_usb_off = irq_base + S2MPW01_CHG_IRQ_USB_BOOT_OFF_INT2;
		REQUEST_IRQ(charger->irq_usb_off, charger, "muic-usb_off");

		charger->irq_usb_on = irq_base + S2MPW01_CHG_IRQ_USB_BOOT_ON_INT2;
		REQUEST_IRQ(charger->irq_usb_on, charger, "muic-usb_on");

		charger->irq_uart_off = irq_base + S2MPW01_CHG_IRQ_UART_BOOT_OFF_INT2;
		REQUEST_IRQ(charger->irq_uart_off, charger, "muic-uart_off");

		charger->irq_uart_on = irq_base + S2MPW01_CHG_IRQ_UART_BOOT_ON_INT3;
		REQUEST_IRQ(charger->irq_uart_on, charger, "muic-uart_on");

		charger->irq_acokf = irq_base + S2MPW01_PMIC_IRQ_ACOKBF_INT1;
		REQUEST_IRQ(charger->irq_acokf, charger, "muic-acokf");

		charger->irq_acokr = irq_base + S2MPW01_PMIC_IRQ_ACOKBR_INT1;
		REQUEST_IRQ(charger->irq_acokr, charger, "muic-acokr");
	}

#if 0
	pr_err("%s: usb_off(%d), usb_on(%d), uart_off(%d), uart_on(%d), fact_leak(%d), 150K(%d)",
		__func__, charger->irq_usb_off, charger->irq_usb_on, charger->irq_uart_off,
		charger->irq_uart_on, charger->irq_fact_leakage, charger->irq_uart_cable);
#endif
	pr_err("%s:usb_off(%d), usb_on(%d), uart_off(%d), uart_on(%d), jig_on(%d), muic-acokf(%d), muic-acokr(%d)\n",
		__func__, charger->irq_usb_off, charger->irq_usb_on, charger->irq_uart_off, charger->irq_uart_on,
		charger->irq_jigon, charger->irq_acokf, charger->irq_acokr);

	return ret;
}

#define FREE_IRQ(_irq, _dev_id, _name)					\
do {									\
	if (_irq) {							\
		free_irq(_irq, _dev_id);				\
		pr_info("%s: IRQ(%d):%s free done\n",	\
				__func__, _irq, _name);			\
	}								\
} while (0)

static void s2mpw01_muic_free_irqs(struct s2mpw01_charger_data *charger)
{
	pr_info("%s\n", __func__);

	/* free MUIC IRQ */
	FREE_IRQ(charger->irq_uart_off, charger, "muic-uart_off");
	FREE_IRQ(charger->irq_uart_on, charger, "muic-uart_on");
	FREE_IRQ(charger->irq_usb_off, charger, "muic-usb_off");
	FREE_IRQ(charger->irq_usb_on, charger, "muic-usb_on");
	FREE_IRQ(charger->irq_uart_cable, charger, "muic-uart_cable");
	FREE_IRQ(charger->irq_fact_leakage, charger, "muic-fact_leakage");
	FREE_IRQ(charger->irq_jigon, charger, "muic-jigon");
}
#if 0
#ifdef CONFIG_OF
static int s2mpw01_charger_parse_dt(struct device *dev,
		struct s2mpw01_charger_platform_data *pdata)
{
	struct device_node *np = of_find_node_by_name(NULL, "s2mpw01-charger");
	const u32 *p;
	int ret, i , len;

	/* SC_CTRL11 , SET_OSC_BUCK , Buck switching frequency setting
	* 0 : 500kHz
	* 1 : 750kHz
	* 2 : 1MHz
	* 3 : 2MHz
	*/

	/* SC_CTRL5 , SET_VF_VBAT , Battery regulation voltage setting */
	ret = of_property_read_u32(np, "battery,chg_float_voltage",
				&pdata->chg_float_voltage);

	np = of_find_node_by_name(NULL, "battery");
	if (!np) {
		pr_err("%s np NULL\n", __func__);
	} else {
		ret = of_property_read_string(np,
			"battery,charger_name", (char const **)&pdata->charger_name);

		ret = of_property_read_u32(np, "battery,full_check_type_2nd",
				&pdata->full_check_type_2nd);
		if (ret)
			pr_info("%s : Full check type 2nd is Empty\n", __func__);

		p = of_get_property(np, "battery,input_current_limit", &len);
		if (!p)
			return 1;

		len = len / sizeof(u32);

		pdata->charging_current_table = kzalloc(sizeof(sec_charging_current_t) * len,
				GFP_KERNEL);

		for (i = 0; i < len; i++) {
			ret = of_property_read_u32_index(np,
					"battery,input_current_limit", i,
					&pdata->charging_current_table[i].input_current_limit);
			ret = of_property_read_u32_index(np,
					"battery,fast_charging_current", i,
					&pdata->charging_current_table[i].fast_charging_current);
			ret = of_property_read_u32_index(np,
					"battery,full_check_current_1st", i,
					&pdata->charging_current_table[i].full_check_current_1st);
			ret = of_property_read_u32_index(np,
					"battery,full_check_current_2nd", i,
					&pdata->charging_current_table[i].full_check_current_2nd);
		}
	}

	dev_info(dev, "s2mpw01 charger parse dt retval = %d\n", ret);
	return ret;
}
/* if need to set s2mpw01 pdata */
static struct of_device_id s2mpw01_charger_match_table[] = {
	{ .compatible = "swa,s2mpw01-charger",},
	{},
};
#else
static int s2mpw01_charger_parse_dt(struct device *dev,
		struct s2mpw01_charger_platform_data *pdata)
{
	return -ENOSYS;
}
#define s2mpw01_charger_match_table NULL
#endif /* CONFIG_OF */
#endif

static int s2mpw01_charger_probe(struct platform_device *pdev)
{
	s2mpw01_charger_platform_data_t *pdata = dev_get_platdata(&pdev->dev);
	struct sec_pmic_dev *s2mpw01 = dev_get_drvdata(pdev->dev.parent);
/*	struct s2mpw01_platform_data *pdata = dev_get_platdata(s2mpw01->dev); */
	struct s2mpw01_charger_data *charger;
	int ret = 0;
	unsigned int acok_stat = 0, data = 0;
	unsigned int chg_sts2, chg_sts3;

	pr_info("%s:[BATT] S2MPW01 Charger driver probe\n", __func__);
	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->io_lock);
	device_init_wakeup(&pdev->dev, true);

	charger->dev = &pdev->dev;
	charger->iodev = s2mpw01;

	//charger->client = s2mpw01->charger;

#if 0
	charger->pdata = devm_kzalloc(&pdev->dev, sizeof(*(charger->pdata)),
			GFP_KERNEL);
	if (!charger->pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_parse_dt_nomem;
	}
#endif

#if 0
	ret = s2mpw01_charger_parse_dt(&pdev->dev, charger->pdata);
	if (ret < 0)
		goto err_parse_dt;
#else
	charger->pdata = pdata;
#endif

	platform_set_drvdata(pdev, charger);

	if (charger->pdata->charger_name == NULL)
		charger->pdata->charger_name = "sec-charger";

	charger->psy_chg.name           = charger->pdata->charger_name;
	charger->psy_chg.type           = POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg.get_property   = sec_chg_get_property;
	charger->psy_chg.set_property   = sec_chg_set_property;
	charger->psy_chg.properties     = sec_charger_props;
	charger->psy_chg.num_properties = ARRAY_SIZE(sec_charger_props);

	charger->dev_id = s2mpw01->rev_num;

	/* need to check siop level */
	charger->siop_level = 100;

	s2mpw01_chg_init(charger);

	ret = power_supply_register(&pdev->dev, &charger->psy_chg);
	if (ret) {
		pr_err("%s: Failed to Register psy_chg\n", __func__);
		goto err_power_supply_register;
	}

#if 0
	/*
	 * irq request
	 * if you need to add irq , please refer below code.
	 */
	charger->irq_det_bat = pdata->irq_base + S2MPW01_CHG_IRQ_BATDET_INT2;
	ret = request_threaded_irq(charger->irq_det_bat, NULL,
			s2mpw01_det_bat_isr, 0 , "det-bat-in-irq", charger);
	if (ret < 0) {
		dev_err(s2mpw01->dev, "%s: Fail to request det bat in IRQ: %d: %d\n",
					__func__, charger->irq_det_bat, ret);
		goto err_reg_irq;
	}
#endif
	charger->irq_chg = charger->iodev->irq_base + S2MPW01_CHG_IRQ_TOPOFF_INT1;
	ret = request_threaded_irq(charger->irq_chg, NULL,
			s2mpw01_chg_isr, 0 , "chg-irq", charger);
	if (ret < 0) {
		dev_err(s2mpw01->dev, "%s: Fail to request charger irq in IRQ: %d: %d\n",
					__func__, charger->irq_chg, ret);
		goto err_power_supply_register;
	}

	charger->irq_tmrout = charger->iodev->irq_base + S2MPW01_CHG_IRQ_TMROUT_INT3;
	ret = request_threaded_irq(charger->irq_tmrout, NULL,
			s2mpw01_tmrout_isr, 0 , "tmrout-irq", charger);
	if (ret < 0) {
		dev_err(s2mpw01->dev, "%s: Fail to request charger irq in IRQ: %d: %d\n",
					__func__, charger->irq_tmrout, ret);
		goto err_power_supply_register;
	}

	s2mpw01_test_read(charger->iodev);

	if (s2mpw01->using_rid_detect) {
		ret = s2mpw01_muic_irq_init(charger);
		if (ret) {
			pr_err( "[muic] %s: failed to init muic irq(%d)\n", __func__, ret);
			goto fail_init_irq;
		}
#if defined(CONFIG_SWITCH)
		charger->init_callback = switch_data.init_cb;
		charger->dock_callback = switch_data.dock_cb;
		charger->usb_callback  = switch_data.usb_cb;
		charger->otg_callback  = switch_data.otg_cb;
		charger->jig_callback = switch_data.set_jig_state_cb;
		charger->cable_chg_callback = switch_data.cable_chg_cb;
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
		charger->ps_cable_callback = switch_data.ps_cable_cb;
#endif
		if (charger->init_callback)
			charger->init_callback();
#endif

		INIT_DELAYED_WORK(&charger->init_work, s2mpw01_muic_init_detect);
		schedule_delayed_work(&charger->init_work, msecs_to_jiffies(3000));

		charger->is_usb_ready = false;
		INIT_DELAYED_WORK(&charger->usb_work, s2mpw01_muic_usb_detect);
		schedule_delayed_work(&charger->usb_work, msecs_to_jiffies(13000));

		INIT_DELAYED_WORK(&charger->rid_work, s2mpw01_muic_rid_check);
	}

	/* charger topoff on/off work */
	INIT_DELAYED_WORK(&charger->ta_work, s2mpw01_ta_detect);

	/* initially TOP-OFF interrupt masking */
	s2mpw01_topoff_interrupt_onoff(charger, 0);

	ret = sec_pmic_read(charger->iodev, 0x41, &charger->chg_en_data);
	if (ret < 0) {
		pr_err("%s: failed to read PM addr 0x41(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

	/* factory_mode setting */
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS2, &chg_sts2);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status2 read fail\n", __func__);
	}
	ret = sec_chg_read(charger->iodev, S2MPW01_CHG_REG_STATUS3, &chg_sts3);
	if (ret < 0) {
		pr_err("{DEBUG] %s : chg status3 read fail\n", __func__);
	}

	if ((chg_sts2 & USB_BOOT_ON_STATUS_MASK) || (chg_sts2 & USB_BOOT_OFF_STATUS_MASK) ||
		(chg_sts3 & UART_BOOT_ON_STATUS_MASK) || (chg_sts2 & UART_BOOT_OFF_STATUS_MASK))
		s2mpw01_factory_mode_setting(charger);

	/* make bit[6] to 0 (if 0x41 is 0, old rev.) */
	if (charger->chg_en_data == 0)
		charger->chg_en_data = 0x21;
	else
		charger->chg_en_data &= 0xBF;

	ret = sec_pmic_read(charger->iodev, S2MPW01_PMIC_REG_STATUS1, &acok_stat);
	if (ret < 0) {
		pr_err("%s: failed to read S2MPW01_PMIC_REG_STATUS1(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

	data = charger->chg_en_data;
	/* if acok is high, set 1 to bit[6]. if acok is low, set 0 to bit[6] */
	if (!charger->factory_mode) {
		if (acok_stat & ACOK_STATUS_MASK)
			data |= 0x40;
		sec_chg_write(charger->iodev, 0x2E, data);
	}
	pr_info("%s:[BATT] S2MPW01 charger driver loaded OK\n", __func__);

	return 0;

fail_init_irq:
	if (s2mpw01->using_rid_detect)
		s2mpw01_muic_free_irqs(charger);
err_power_supply_register:
	destroy_workqueue(charger->charger_wqueue);
/* err_create_wq: */
	power_supply_unregister(&charger->psy_chg);
	device_init_wakeup(&pdev->dev, false);
	mutex_destroy(&charger->io_lock);
	kfree(charger);
	return ret;
}

static int s2mpw01_charger_remove(struct platform_device *pdev)
{
	struct s2mpw01_charger_data *charger =
		platform_get_drvdata(pdev);

	power_supply_unregister(&charger->psy_chg);
	device_init_wakeup(&pdev->dev, false);
	mutex_destroy(&charger->io_lock);
	kfree(charger);
	return 0;
}

#if defined CONFIG_PM
static int s2mpw01_charger_suspend(struct device *dev)
{
	return 0;
}

static int s2mpw01_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mpw01_charger_suspend NULL
#define s2mpw01_charger_resume NULL
#endif

static void s2mpw01_charger_shutdown(struct device *dev)
{
	struct s2mpw01_charger_data *charger = dev_get_drvdata(dev);
	unsigned int stat_val = 0;

	/* ACOK status */
	stat_val= charger->chg_en_data;
	sec_chg_write(charger->iodev, 0x2E, stat_val);
	sec_chg_update(charger->iodev, S2MPW01_CHG_REG_CTRL1, EN_CHG_MASK, EN_CHG_MASK);
	pr_info("%s: S2MPW01 Charger driver shutdown\n", __func__);
}

static SIMPLE_DEV_PM_OPS(s2mpw01_charger_pm_ops, s2mpw01_charger_suspend,
		s2mpw01_charger_resume);

static struct platform_driver s2mpw01_charger_driver = {
	.driver         = {
		.name   = "s2mpw01-charger",
		.owner  = THIS_MODULE,
		//.of_match_table = s2mpw01_charger_match_table,
		.pm     = &s2mpw01_charger_pm_ops,
		.shutdown = s2mpw01_charger_shutdown,
	},
	.probe          = s2mpw01_charger_probe,
	.remove		= s2mpw01_charger_remove,
};

static int __init s2mpw01_charger_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&s2mpw01_charger_driver);

	return ret;
}

subsys_initcall(s2mpw01_charger_init);

static void __exit s2mpw01_charger_exit(void)
{
	platform_driver_unregister(&s2mpw01_charger_driver);
}
module_exit(s2mpw01_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Charger driver for S2MPW01");
