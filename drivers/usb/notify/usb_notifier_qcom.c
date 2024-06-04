/*
 * Copyright (C) 2014 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) "usb_notifier: " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/usb_notify.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/battery/sec_charging_common.h>

#ifdef CONFIG_MUIC_NOTIFIER
#include <linux/muic/muic.h>
#include <linux/muic/muic_notifier.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif /* CONFIG_SWITCH */
#endif


struct gadget_notify_dev {
	struct device	*dev;
	int	gadget_state;
	bool	is_ready;
};

struct usb_notifier_platform_data {
	struct	gadget_notify_dev g_ndev;
	struct	notifier_block usb_nb;
	int	gpio_redriver_en;
	bool unsupport_host;
};

enum {
	GADGET_NOTIFIER_DETACH,
	GADGET_NOTIFIER_ATTACH,
	GADGET_NOTIFIER_DEFAULT,
};

extern void sec_otg_set_vbus_state(int online);
#ifdef CONFIG_USB
extern int sec_set_host(int enable);
#endif

static int of_usb_notifier_dt(struct device *dev,
		struct usb_notifier_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;

	pdata->unsupport_host = of_property_read_bool(np, "qcom,unsupport_host");

	pr_info("usb: %s: qcom,un-support-host is %d\n", __func__, pdata->unsupport_host);

	return 0;
}

#if defined(CONFIG_MUIC_NOTIFIER)
#ifdef CONFIG_SWITCH
static struct switch_dev switch_usb = {
	.name = "usb_cable",
};
#endif /* CONFIG_SWITCH */

static int usb_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	struct otg_notify *o_notify;

	o_notify = get_otg_notify();

	pr_info("%s action=%lu, attached_dev=%d\n",
		__func__, action, attached_dev);

	switch (attached_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH) {
#ifdef CONFIG_SWITCH
			switch_set_state(&switch_usb, 0);
#endif
			send_otg_notify(o_notify, NOTIFY_EVENT_VBUS, 0);
		} else if (action == MUIC_NOTIFY_CMD_ATTACH) {
			send_otg_notify(o_notify, NOTIFY_EVENT_VBUS, 1);
#ifdef CONFIG_SWITCH
			switch_set_state(&switch_usb, 1);
#endif
		} else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_OTG_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_HOST, 0);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_HOST, 1);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_HMT_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_HMT, 0);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_HMT, 1);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			pr_info("%s - USB_HOST_TEST_DETACHED\n", __func__);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			pr_info("%s - USB_HOST_TEST_ATTACHED\n", __func__);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_SMARTDOCK_TA_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_SMARTDOCK_TA, 0);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_SMARTDOCK_TA, 1);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_SMARTDOCK_USB_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH) {
#ifdef CONFIG_SWITCH
			switch_set_state(&switch_usb, 0);
#endif
			send_otg_notify
				(o_notify, NOTIFY_EVENT_SMARTDOCK_USB, 0);
		} else if (action == MUIC_NOTIFY_CMD_ATTACH) {
			send_otg_notify
				(o_notify, NOTIFY_EVENT_SMARTDOCK_USB, 1);
#ifdef CONFIG_SWITCH
			switch_set_state(&switch_usb, 1);
#endif
		} else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_AUDIODOCK_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_AUDIODOCK, 0);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_AUDIODOCK, 1);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	case ATTACHED_DEV_UNIVERSAL_MMDOCK_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_MMDOCK, 0);
		else if (action == MUIC_NOTIFY_CMD_ATTACH)
			send_otg_notify(o_notify, NOTIFY_EVENT_MMDOCK, 1);
		else
			pr_err("%s - ACTION Error!\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}
#endif /*CONFIG_MUIC_NOTIFIER*/

static int otg_accessory_power(bool enable)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int on = !!enable;
	int current_cable_type;
	int ret = 0;
	/* otg psy test */
#if defined (CONFIG_SEC_Z3LTE_CIS_PROJECT) || defined (CONFIG_SEC_Z3LTE_CHN_PROJECT)
	psy = power_supply_get_by_name("sm5703-charger");
#else
	psy = power_supply_get_by_name("battery");
#endif

	if (!psy) {
		pr_err("%s: Fail to get psy battery\n", __func__);
		return -1;
	}

#if defined(CONFIG_MUIC_SM5502_SUPPORT_LANHUB_TA)
	if (enable) {
			current_cable_type = lanhub_ta_case ? POWER_SUPPLY_TYPE_LAN_HUB : POWER_SUPPLY_TYPE_OTG;
			pr_info("%s: LANHUB+TA Case cable type change for the (%d) \n",
									__func__,current_cable_type);
	}
#else
	if (enable)
		current_cable_type = POWER_SUPPLY_TYPE_OTG;
#endif
	else
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;

	val.intval = current_cable_type;
	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);

	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	} else {
		pr_info("otg accessory power = %d\n", on);
	}

	return ret;
}

static int sec_set_peripheral(bool enable)
{
	sec_otg_set_vbus_state((int)enable);

	return 0;
}

static struct otg_notify sec_otg_notify = {
	.vbus_drive	= otg_accessory_power,
	.set_peripheral	= sec_set_peripheral,
#ifdef CONFIG_USB
	.set_host = sec_set_host,
#endif
	.vbus_detect_gpio = -1,
	.is_wakelock = 1,
	.redriver_en_gpio = -1,
	.unsupport_host = 0,
};

static int usb_notifier_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct usb_notifier_platform_data *pdata = NULL;

	pr_info("notifier_probe\n");

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct usb_notifier_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = of_usb_notifier_dt(&pdev->dev, pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to get device of_node\n");
			return ret;
		}

		pdev->dev.platform_data = pdata;
	} else
		pdata = pdev->dev.platform_data;

	if(pdata->unsupport_host)
		sec_otg_notify.unsupport_host = 1;
	set_otg_notify(&sec_otg_notify);
	set_notify_data(&sec_otg_notify, pdata);

#ifdef CONFIG_MUIC_NOTIFIER
#ifdef CONFIG_SWITCH
	ret = switch_dev_register(&switch_usb);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register usb switch(%d)\n", ret);
		return ret;
	}
#endif
	muic_notifier_register(&pdata->usb_nb, usb_handle_notification,
			       MUIC_NOTIFY_DEV_USB);
#endif

	dev_info(&pdev->dev, "usb notifier probe\n");
	return 0;
}

static int usb_notifier_remove(struct platform_device *pdev)
{
	struct usb_notifier_platform_data *pdata = dev_get_platdata(&pdev->dev);

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_unregister(&pdata->usb_nb);
#endif
	return 0;
}

static const struct of_device_id usb_notifier_dt_ids[] = {
	{ .compatible = "samsung,usb-notifier",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, usb_notifier_dt_ids);

static struct platform_driver usb_notifier_driver = {
	.probe		= usb_notifier_probe,
	.remove		= usb_notifier_remove,
	.driver		= {
		.name	= "usb_notifier",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(usb_notifier_dt_ids),
	},
};

static int __init usb_notifier_init(void)
{
	return platform_driver_register(&usb_notifier_driver);
}

static void __init usb_notifier_exit(void)
{
	platform_driver_unregister(&usb_notifier_driver);
}

late_initcall(usb_notifier_init);
module_exit(usb_notifier_exit);

MODULE_AUTHOR("Samsung USB Team");
MODULE_DESCRIPTION("USB notifier");
MODULE_LICENSE("GPL");
