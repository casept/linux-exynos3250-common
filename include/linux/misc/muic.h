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

#ifndef LINUX_MISC_MUIC_H
#define LINUX_MISC_MUIC_H

#if !defined(CONFIG_USB_SPRD_SWITCH)
extern struct device *switch_device;
#endif

enum {
    MUIC_CABLE_TYPE_NONE = 0,
    MUIC_CABLE_TYPE_UART,            //adc 0x16
    MUIC_CABLE_TYPE_USB,             //adc 0x1f (none id)
    MUIC_CABLE_TYPE_OTG,             //adc 0x00, regDev1&0x01
    /* TA Group */
    MUIC_CABLE_TYPE_REGULAR_TA,      //adc 0x1f (none id, D+ short to D-)
    MUIC_CABLE_TYPE_ATT_TA,          //adc 0x1f (none id, only VBUS)AT&T
    MUIC_CABLE_TYPE_0x15,            //adc 0x15
    MUIC_CABLE_TYPE_TYPE1_CHARGER,   //adc 0x17 (id : 200k)
    MUIC_CABLE_TYPE_0x1A,            //adc 0x1A
    /* JIG Group */
    MUIC_CABLE_TYPE_JIG_USB_OFF,     //adc 0x18
    MUIC_CABLE_TYPE_JIG_USB_ON,      //adc 0x19
    MUIC_CABLE_TYPE_JIG_UART_OFF,    //adc 0x1C
    MUIC_CABLE_TYPE_JIG_UART_ON,     //adc 0x1D
    // JIG type with VBUS
    MUIC_CABLE_TYPE_JIG_UART_OFF_WITH_VBUS,    //adc 0x1C
    MUIC_CABLE_TYPE_JIG_UART_ON_WITH_VBUS,     //adc 0x1D

    MUIC_CABLE_TYPE_CDP, // USB Charging downstream port, usually treated as SDP
    MUIC_CABLE_TYPE_L_USB, // L-USB cable
    MUIC_CABLE_TYPE_UNKNOWN,
    MUIC_CABLE_TYPE_INVALID, // Un-initialized

    MUIC_CABLE_TYPE_OTG_WITH_VBUS,
    MUIC_CABLE_TYPE_SAMSUNG_PS,
};

typedef enum {
    JIG_USB_BOOT_OFF,
    JIG_USB_BOOT_ON,
    JIG_UART_BOOT_OFF,
    JIG_UART_BOOT_ON,
} jig_type_t;

int muic_get_jig_state(void);

#endif // LINUX_MISC_MUIC_H

