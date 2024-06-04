/*
 * include/linux/misc/muic.h
 *
 * muic header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_MUIC_SM5504_H
#define LINUX_MUIC_SM5504_H

#include <linux/misc/muic.h>

#if !defined(CONFIG_USB_SPRD_SWITCH)
extern struct device *switch_device;
#endif

struct sm5504_platform_data {
	int irq_base;
	int irq_gpio;
	bool wakeup;

    void (*init_callback)(void);
    void (*dock_callback)(uint8_t attached);
    void (*cable_chg_callback)(int32_t cable_type);
    void (*ocp_callback)(void);
    void (*otp_callback)(void);
    void (*ovp_callback)(void);
    void (*usb_callback)(uint8_t attached);
    void (*uart_callback)(uint8_t attached);
    void (*otg_callback)(uint8_t attached);
    void (*jig_callback)(jig_type_t type, uint8_t attached);
#ifdef CONFIG_MUIC_SUPPORT_PS_CABLE
    void (*ps_cable_callback)(uint8_t attached);
#endif
};

bool sm5504_get_vbus_status(void);
bool sm5504_get_otg_status(void);
#endif // LINUX_MUIC_SM5504_H

