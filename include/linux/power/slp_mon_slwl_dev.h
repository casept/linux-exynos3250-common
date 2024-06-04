/*
 * To log which slave wakelock prevent AP sleep.
 *
 * Copyright (C) 2015 SAMSUNG, Inc.
 * Sanghyeon Lee <sirano06.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _SLP_MON_SLWL_DEV_H
#define _SLP_MON_SLWL_DEV_H

#include <linux/ktime.h>

#define SLEEP_MON_SLWL_ARRAY_SIZE 4
#define SLEEP_MON_SLWL_NAME_LENGTH 15

#define SLEEP_MON_SLWL_IDX_BIT 24
#define SLEEP_MON_SLWL_PREVENT_TIME_MAX BIT(SLEEP_MON_SLWL_IDX_BIT) - 1

#ifdef CONFIG_SLEEP_MONITOR_SLAVE_WAKELOCK
extern int add_slp_mon_slwl_list(char *name);
extern int slp_mon_slwl_lock(const char *buf);
extern int slp_mon_slwl_unlock(const char *buf);
#else
static int add_slp_mon_slwl_list(char *name){}
static int slp_mon_slwl_lock(const char *buf){}
static int slp_mon_slwl_unlock(const char *buf){}
#endif

#ifdef CONFIG_ENERGY_MONITOR
struct slp_mon_slave_wakelocks {
	char slwl_name[SLEEP_MON_SLWL_NAME_LENGTH];
	ktime_t prevent_time;
};

#ifdef CONFIG_SLEEP_MONITOR_SLAVE_WAKELOCK
void get_sleep_monitor_slave_wakelock(struct slp_mon_slave_wakelocks *slwl, int size);

#else
void get_sleep_monitor_slave_wakelock(struct slp_mon_slave_wakelocks *slwl, int size){}
#endif
#endif

#endif /* _SLP_MON_SLWL_DEV_H */

