/*
 * arch/arm/mach-sc/sec-switch.c
 *
 * c source file supporting MUIC common platform device register
 *
 * Copyright (C) 2014 Samsung Electronics
 * tyung.kim <tyung.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/module.h>
#include <linux/usb/gadget.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/sm5504.h>

#include <plat/devs.h>
#include <linux/usb/gadget.h>
#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
#include <linux/i2c/zxt_ztw522_ts.h>
#endif

#ifdef CONFIG_SWITCH
#include <linux/switch.h>
static struct switch_dev switch_dock = {
	.name = "dock",
};

static struct switch_dev switch_usb = {
	.name = "usb_cable",
};

static struct switch_dev switch_otg = {
	.name = "otg",
};

static struct switch_dev switch_jig = {
	.name = "jig_cable",
};
/* Samsung's Power Sharing Cable EP-SG900 */
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
static struct switch_dev switch_ps_cable = {
        .name = "ps_cable",
};
#endif
#endif /* CONFIG_SWITCH */

extern struct class *sec_class;
struct device *switch_device;
EXPORT_SYMBOL(switch_device);

bool is_cable_attached;
bool init_cb_done = false;
#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
struct tsp_callbacks *ztw522_charger_callbacks;
void ztw522_tsp_charger_infom(int cable_type, int attach)
{
	if (!ztw522_charger_callbacks) {
		pr_err("%s, ztw522_charger_callbacks is NULL \n",__func__);
		return;
	}

	if (!ztw522_charger_callbacks->inform_charger) {
		pr_err("%s, inform_charger is NULL \n",__func__);
		return;
	}

	switch(cable_type) {

	case MUIC_SM5504_CABLE_TYPE_NONE:
	case MUIC_SM5504_CABLE_TYPE_UART:
	case MUIC_SM5504_CABLE_TYPE_USB:
	case MUIC_SM5504_CABLE_TYPE_REGULAR_TA:
	case MUIC_SM5504_CABLE_TYPE_ATT_TA:
	case MUIC_SM5504_CABLE_TYPE_0x15:
	case MUIC_SM5504_CABLE_TYPE_TYPE1_CHARGER:
	case MUIC_SM5504_CABLE_TYPE_0x1A:
	case MUIC_SM5504_CABLE_TYPE_JIG_USB_OFF:
	case MUIC_SM5504_CABLE_TYPE_JIG_USB_ON:
	case MUIC_SM5504_CABLE_TYPE_JIG_UART_ON:
	case MUIC_SM5504_CABLE_TYPE_JIG_UART_ON_WITH_VBUS:
	case MUIC_SM5504_CABLE_TYPE_CDP:
	case MUIC_SM5504_CABLE_TYPE_L_USB:
	case MUIC_SM5504_CABLE_TYPE_UNKNOWN:
	case MUIC_SM5504_CABLE_TYPE_INVALID:
	case MUIC_SM5504_CABLE_TYPE_OTG_WITH_VBUS:
	case MUIC_SM5504_CABLE_TYPE_SAMSUNG_PS:
		ztw522_charger_callbacks->inform_charger(ztw522_charger_callbacks, attach);
		break;
	case MUIC_SM5504_CABLE_TYPE_OTG:
	case MUIC_SM5504_CABLE_TYPE_JIG_UART_OFF:
	case MUIC_SM5504_CABLE_TYPE_JIG_UART_OFF_WITH_VBUS:
	default:
		ztw522_charger_callbacks->inform_charger(ztw522_charger_callbacks, false);
		break;
	}

	return;
}
#endif

static void muic_init_cb(void)
{
#ifdef CONFIG_SWITCH
	int ret;
	pr_info("func:%s[done:%d]\n", __func__, init_cb_done);
	if (!init_cb_done) {
		ret = switch_dev_register(&switch_dock);
		if (ret < 0)
			pr_err("%s Failed to register dock switch(%d)\n", __func__, ret);

		ret = switch_dev_register(&switch_usb);
		if (ret < 0)
			pr_err("%s Failed to register usb switch(%d)\n", __func__, ret);

		ret = switch_dev_register(&switch_otg);
		if (ret < 0)
			pr_err("%s Failed to register otg switch(%d)\n", __func__, ret);

		ret = switch_dev_register(&switch_jig);
		if (ret < 0)
			pr_err("%s Failed to register jig switch(%d)\n", __func__, ret);

#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
		ret = switch_dev_register(&switch_ps_cable);
		if (ret < 0)
			pr_err("%s Failed to register ps_cable switch(%d)\n", __func__, ret);
#endif
	}
	init_cb_done = true;
#endif
}

extern void usb_notify_cb(int plug_in);

static void muic_usb_cb(u8 attached)
{
	u8 onoff = !!attached;
#ifdef CONFIG_USB_GADGET_SWITCH
	struct usb_gadget *gadget = platform_get_drvdata(&s3c_device_usb_hsotg);
	int g_state;
#endif

	pr_info("%s: usb_mode:%d\n", __func__, attached);

	if (onoff) {
		pr_info("usb: muic: USB_CABLE_ATTACHED(%d)\n", onoff);
#ifdef CONFIG_USB_GADGET_SWITCH
		if (gadget) {
			g_state = usb_gadget_vbus_connect(gadget);
			if (g_state < 0) {
				pr_err("%s:gadget_vbus connect failed(%d)\n",
					__func__, g_state);
				return;
			}
		}
#endif
	} else {
		pr_info("usb: muic: USB_CABLE_DETACHED(%d)\n", onoff);
#ifdef CONFIG_USB_GADGET_SWITCH
		if (gadget) {
			g_state = usb_gadget_vbus_disconnect(gadget);
			if (g_state < 0)
				pr_err("%s:gadget_vbus dis-connect failed(%d)\n",
					__func__, g_state);
		}
#endif
	}

#ifdef CONFIG_SWITCH
	switch_set_state(&switch_usb, onoff);
#endif

	return;
}

static void muic_otg_cb(u8 attached)
{
	pr_info("%s: otg_mode:%d\n", __func__, attached);



#ifdef CONFIG_SWITCH
	switch_set_state(&switch_otg, attached);
#endif

#ifdef CONFIG_MFD_SM5504
	if (attached) {
		SM5701_set_bstout(SM5701_BSTOUT_5P0);
		SM5701_set_operationmode(SM5701_OPERATIONMODE_OTG_ON);
	}
	else {
		SM5701_set_bstout(SM5701_BSTOUT_4P5);
		SM5701_clear_operationmode(SM5701_OPERATIONMODE_OTG_ON);
	}
#endif

	return;
}

bool is_jig_on;
extern int current_cable_type;

static void muic_charger_cb(int cable_type)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union  power_supply_propval value;

	pr_info("%s: cable type (0x%02x)\n", __func__, cable_type);

	switch (cable_type) {
		case MUIC_SM5504_CABLE_TYPE_NONE:
		case MUIC_SM5504_CABLE_TYPE_UNKNOWN:
			current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
			is_jig_on = false;
			is_cable_attached = false;
			break;
		case MUIC_SM5504_CABLE_TYPE_USB:
		case MUIC_SM5504_CABLE_TYPE_CDP:
		case MUIC_SM5504_CABLE_TYPE_L_USB:
		case MUIC_SM5504_CABLE_TYPE_0x15:
		case MUIC_SM5504_CABLE_TYPE_TYPE1_CHARGER:
			current_cable_type = POWER_SUPPLY_TYPE_USB;
			is_jig_on = false;
			is_cable_attached = true;
			break;
		case MUIC_SM5504_CABLE_TYPE_REGULAR_TA:
		case MUIC_SM5504_CABLE_TYPE_ATT_TA:
			current_cable_type = POWER_SUPPLY_TYPE_MAINS;
			is_jig_on = false;
			is_cable_attached = true;
			break;
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
		case MUIC_SM5504_CABLE_TYPE_SAMSUNG_PS:
			current_cable_type = POWER_SUPPLY_TYPE_POWER_SHARING;
			is_jig_on = false;
			is_cable_attached = false;
			break;
#endif
		case MUIC_SM5504_CABLE_TYPE_OTG:
#if 0 /*def CONFIG_MACH_KIRAN*/
			current_cable_type = POWER_SUPPLY_TYPE_USB;
			is_jig_on = false;
			is_cable_attached = false;
			break;
#else
			goto skip;
#endif
		case MUIC_SM5504_CABLE_TYPE_JIG_UART_OFF_WITH_VBUS:
		case MUIC_SM5504_CABLE_TYPE_JIG_UART_ON_WITH_VBUS:
			current_cable_type = POWER_SUPPLY_TYPE_UARTOFF;
			is_jig_on = true;
			is_cable_attached = true;
			break;
		case MUIC_SM5504_CABLE_TYPE_JIG_UART_OFF:
		case MUIC_SM5504_CABLE_TYPE_JIG_UART_ON:
			current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
			is_jig_on = true;
			is_cable_attached = false;
			break;
		case MUIC_SM5504_CABLE_TYPE_JIG_USB_ON:
		case MUIC_SM5504_CABLE_TYPE_JIG_USB_OFF:
			current_cable_type = POWER_SUPPLY_TYPE_USB;
			is_jig_on = true;
			is_cable_attached = true;
			break;
		case MUIC_SM5504_CABLE_TYPE_0x1A:
		case MUIC_SM5504_CABLE_TYPE_UART:
			current_cable_type = POWER_SUPPLY_TYPE_MAINS;
			is_jig_on = false;
			is_cable_attached = true;
			break;
		default:
			pr_err("%s: invalid type for charger:%d\n",
					__func__, cable_type);
			current_cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
			goto skip;
	}

	if (!psy || !psy->set_property)
		pr_err("%s: fail to get battery psy\n", __func__);
	else {
		value.intval = current_cable_type;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	}

#ifdef CONFIG_TOUCHSCREEN_ZINITIX_ZTW522
	ztw522_tsp_charger_infom(cable_type, is_cable_attached);
#endif

skip:
	return;
}

static void muic_dock_cb(u8 type)
{
	pr_info("%s: type: %d\n", __func__, type);

#ifdef CONFIG_SWITCH
	switch_set_state(&switch_dock, type);
#endif
}

bool is_jig_attached;
int muic_get_jig_state(void)
{
	return is_jig_attached;
}
EXPORT_SYMBOL(muic_get_jig_state);

static void muic_set_jig_state(u8 attached)
{
	pr_info("%s: attached: %d\n", __func__, attached);
	is_jig_attached = !!attached;

#ifdef CONFIG_SWITCH
	switch_set_state(&switch_jig, !!attached);
#endif
}

#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
static void muic_ps_cable_cb(u8 attached)
{
        pr_info("%s: ps_cable: %d\n", __func__, attached);

#ifdef CONFIG_SWITCH
        switch_set_state(&switch_ps_cable, attached);
#endif
}
#endif

struct sec_switch_data switch_data = {
	.init_cb = muic_init_cb,
	.dock_cb = muic_dock_cb,
	.usb_cb  = muic_usb_cb,
	.otg_cb  = muic_otg_cb,
	.cable_chg_cb = muic_charger_cb,
	.set_jig_state_cb = muic_set_jig_state,
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
	.ps_cable_cb = muic_ps_cable_cb,
#endif
};

static int __init sec_switch_init(void)
{
	if (!sec_class) {
		pr_err("%s: sec_class is null\n", __func__);
		return -ENODEV;
	}

	switch_device = device_create(sec_class, NULL, 0, NULL, "switch");
	if (IS_ERR(switch_device)) {
		pr_err("%s: Failed to create device(switch)!\n", __func__);
		return -ENODEV;
	}

	return 0;
};

device_initcall(sec_switch_init);
