/*
 * To log which interrupt wake AP up.
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

#include <linux/irq.h>
#include <linux/power/sleep_monitor.h>
#include <linux/power/slp_mon_irq_dev.h>
#include <linux/rcupdate.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>


struct slp_mon_irq {
	struct list_head entry;
	char irq_name[SLEEP_MON_IRQ_NAME_LENGTH];
	int irq;
	int active;
};

static LIST_HEAD(slp_mon_irq_list);

int add_slp_mon_irq_list(int irq, char *name)
{
	struct slp_mon_irq *irq_ins, *iter;
	struct irq_desc *desc = irq_to_desc(irq);

	if (!strncmp(name, UNKNOWN_IRQ_NAME, UNKNOWN_IRQ_NAME_LENGTH)) {
		if (desc && desc->action && desc->action->name) {
			strncpy(name, (char*)desc->action->name, SLEEP_MON_IRQ_NAME_LENGTH);
		}
	}

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slp_mon_irq_list, entry) {
		if (!strcmp(iter->irq_name, name)) {
			iter->active = SLP_MON_IRQ_ACTIVE;
			rcu_read_unlock();
			return 0;
		}
	}
	rcu_read_unlock();
	if ((irq_ins = kmalloc(sizeof(struct slp_mon_irq), GFP_KERNEL)) == NULL)
		return 0;

	irq_ins->irq = irq;
	strncpy(irq_ins->irq_name, name, SLEEP_MON_IRQ_NAME_LENGTH);
	irq_ins->active = SLP_MON_IRQ_ACTIVE;
	list_add_tail(&irq_ins->entry, &slp_mon_irq_list);

	return 1;
}


static int slp_mon_irq_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slp_mon_irq *iter;
	int hit = 0;
	int count = 0;

	if (caller_type == SLEEP_MONITOR_CALL_RESUME) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_irq_list, entry) {
			if (iter->active == SLP_MON_IRQ_ACTIVE) {
				*raw_val = count;
				iter->active = SLP_MON_IRQ_NOT_ACTIVE;
				hit = 1;
				break;
			}
			count += 0x1;
		}
		rcu_read_unlock();
	}

	return hit;
}

static struct sleep_monitor_ops slp_mon_irq_dev = {
	.read_cb_func = slp_mon_irq_cb,
};

static ssize_t slp_mon_read_irq_list(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int i = 0;
	char *buf = NULL;
	struct slp_mon_irq *iter;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = (char*)kmalloc(4 * SLEEP_MONITOR_ONE_LINE_RAW_SIZE * sizeof(char), GFP_KERNEL);

	if (!buf)
		return -1;

	memset(buf, 0, SLEEP_MONITOR_ONE_LINE_RAW_SIZE * sizeof(char));

	if (*ppos == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%08x]%s", special_key,
					get_type_marker(SLEEP_MONITOR_CALL_IRQ_LIST));
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_irq_list, entry) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%s/", iter->irq_name);
			i++;
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret,"\n");
		rcu_read_unlock();
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);
	return ret;
}



static const struct file_operations slp_mon_irq_list_ops = {
        .read           = slp_mon_read_irq_list,
};

static int slp_mon_irq_dev_init(void)
{
	pr_info("%s\n", __func__);
	sleep_monitor_register_ops(NULL, &slp_mon_irq_dev,	SLEEP_MONITOR_IRQ);

	if (slp_mon_d) {
		if (!debugfs_create_file("slp_mon_irq", S_IRUSR
			, slp_mon_d, NULL, &slp_mon_irq_list_ops)) \
				pr_err("%s : debugfs_create_file, error\n", "slp_mon_irq");
	} else
		pr_info("%s - dentry not defined\n", __func__);

	return 0;
}

static void slp_mon_irq_dev_exit(void)
{
	pr_info("%s\n", __func__);

}

module_init(slp_mon_irq_dev_init);
module_exit(slp_mon_irq_dev_exit);

