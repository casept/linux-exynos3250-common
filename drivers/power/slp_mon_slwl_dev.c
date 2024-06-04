/*
 * To log which wakeup source prevent AP sleep.
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


#include <linux/power/sleep_monitor.h>
#include <linux/power/slp_mon_slwl_dev.h>
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
#ifdef CONFIG_ENERGY_MONITOR
#include <linux/power/energy_monitor.h>
#endif


struct slp_mon_slwl {
	struct list_head entry;
	char slwl_name[SLEEP_MON_SLWL_NAME_LENGTH];
	ktime_t lock_time;
	ktime_t unlock_time;
	ktime_t prevent_time;
};

static LIST_HEAD(slp_mon_slwl_list);

static int slwl_idx[SLEEP_MON_SLWL_ARRAY_SIZE];
static ktime_t slwl_prev_time[SLEEP_MON_SLWL_ARRAY_SIZE];
static char slwl_name[SLEEP_MON_SLWL_ARRAY_SIZE][SLEEP_MON_SLWL_NAME_LENGTH];
static int slwl_num;


#ifdef CONFIG_ENERGY_MONITOR
static LIST_HEAD(energy_mon_slwl_list);

static int e_slwl_idx[SLEEP_MON_SLWL_ARRAY_SIZE];
static ktime_t e_slwl_prev_time[SLEEP_MON_SLWL_ARRAY_SIZE];
static char e_slwl_name[SLEEP_MON_SLWL_ARRAY_SIZE][SLEEP_MON_SLWL_NAME_LENGTH];
static int e_slwl_num;

static void init_energy_mon_slwl_data(void)
{
	struct slp_mon_slwl* iter;
	ktime_t ktime_zero = ktime_set(0, 0);
	int i = 0;

	for(i = 0; i < e_slwl_num; i++) {
		memset(e_slwl_name[i], 0, SLEEP_MON_SLWL_ARRAY_SIZE);
		e_slwl_prev_time[i] = ktime_set(0, 0);
		e_slwl_idx[i] = 0;
	}
	e_slwl_num = 0;

	list_for_each_entry_rcu(iter, &energy_mon_slwl_list, entry) {
		iter->prevent_time = ktime_zero;
		iter->lock_time = ktime_zero;
		iter->unlock_time = ktime_zero;
	}
}

static int set_energy_mon_slwl_prv_time(char *name)
{
	struct slp_mon_slwl* iter;
	ktime_t temp = ktime_set(0, 0);

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &energy_mon_slwl_list, entry) {
		if (!(strncmp(iter->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH - 1))) {
			iter->unlock_time = ktime_get();
			temp = ktime_sub(iter->unlock_time, iter->lock_time);
			iter->prevent_time = ktime_add(iter->prevent_time, temp);
			rcu_read_unlock();
			return 0;
		}
	}
	rcu_read_unlock();

	return 1;
}

int add_energy_mon_slwl_list(char *name)
{
	struct slp_mon_slwl *e_slwl_ins, *iter;
	int i = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &energy_mon_slwl_list, entry) {
		if (!strncmp(iter->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH - 1)) {
			iter->lock_time = ktime_get();
			iter->unlock_time = ktime_set(0, 0);
			rcu_read_unlock();
			return 1;
		}
		i++;
	}
	rcu_read_unlock();
	if ((e_slwl_ins = kmalloc(sizeof(struct slp_mon_slwl), GFP_KERNEL)) == NULL)
		return 0;

	memset(e_slwl_ins->slwl_name, 0, SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(e_slwl_ins->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH);
	e_slwl_ins->slwl_name[SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
	e_slwl_ins->lock_time = ktime_get();
	e_slwl_ins->unlock_time = ktime_set(0, 0);
	e_slwl_ins->prevent_time = ktime_set(0, 0);
	list_add_tail(&e_slwl_ins->entry, &energy_mon_slwl_list);

	return 1;
}

static void energy_mon_swap(int i, int j)
{
	ktime_t temp_time = ktime_set(0, 0);
	char temp_name[SLEEP_MON_SLWL_NAME_LENGTH] = {0, };
	int temp_idx = 0;

	temp_time = e_slwl_prev_time[i];
	e_slwl_prev_time[i] = e_slwl_prev_time[j];
	e_slwl_prev_time[j] = temp_time;

	strncpy(temp_name, e_slwl_name[i], SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(e_slwl_name[i], e_slwl_name[j], SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(e_slwl_name[j], temp_name, SLEEP_MON_SLWL_NAME_LENGTH);
	e_slwl_name[i][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
	e_slwl_name[j][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;

	temp_idx = e_slwl_idx[i];
	e_slwl_idx[i] = e_slwl_idx[j];
	e_slwl_idx[j] = temp_idx;
}

static void energy_mon_sort_prevent_time(void)
{
	int i = 0, idx = 0, j = 0;
	struct slp_mon_slwl *iter;

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &energy_mon_slwl_list, entry) {
		if (e_slwl_num < SLEEP_MON_SLWL_ARRAY_SIZE) {
			if (ktime_to_ms(iter->prevent_time) > 0) {
				e_slwl_prev_time[e_slwl_num] = iter->prevent_time;
				e_slwl_idx[e_slwl_num] = idx;
				strncpy(e_slwl_name[e_slwl_num], iter->slwl_name, SLEEP_MON_SLWL_NAME_LENGTH);
				e_slwl_name[e_slwl_num][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
				e_slwl_num++;
			}
		} else
			break;

		idx++;
	}
	rcu_read_unlock();

	for (i = 0; i < SLEEP_MON_SLWL_ARRAY_SIZE; i++) {
		for (j = i + 1; j < SLEEP_MON_SLWL_ARRAY_SIZE; j++) {
			if (i == j)
				continue;

			if (ktime_to_ms(e_slwl_prev_time[i]) < ktime_to_ms(e_slwl_prev_time[j]))
				energy_mon_swap(i, j);
		}
	}

	if (e_slwl_num < SLEEP_MON_SLWL_ARRAY_SIZE)
		return ;

	idx = 0;
	rcu_read_lock();
	list_for_each_entry_rcu(iter, &energy_mon_slwl_list, entry) {
		if (ktime_to_ms(iter->prevent_time) > 0) {
			for (i = 0; i < SLEEP_MON_SLWL_ARRAY_SIZE; i++) {
				if (!strncmp(iter->slwl_name, e_slwl_name[i],
						(strlen(iter->slwl_name) > strlen(e_slwl_name[i])) ?
							strlen(e_slwl_name[i]) : strlen(iter->slwl_name)))
					break;

				if (ktime_to_ms(iter->prevent_time) > ktime_to_ms(e_slwl_prev_time[i])) {
					for (j = SLEEP_MON_SLWL_ARRAY_SIZE - 1; j > i; j--) {
						strncpy(e_slwl_name[j], e_slwl_name[j - 1], SLEEP_MON_SLWL_NAME_LENGTH);
						e_slwl_prev_time[j] = e_slwl_prev_time[j - 1];
						e_slwl_idx[j] = e_slwl_idx[j - 1];
					}
					e_slwl_prev_time[i] = iter->prevent_time;
					e_slwl_idx[i] = idx;
					strncpy(e_slwl_name[i], iter->slwl_name, SLEEP_MON_SLWL_NAME_LENGTH);
					break;
				}
			}
		}
		idx++;
	}
	rcu_read_unlock();
}

void get_sleep_monitor_slave_wakelock(struct slp_mon_slave_wakelocks *slwl, int size)
{
	int i;
	ktime_t time_zero = ktime_set(0, 0);

	energy_mon_sort_prevent_time();
	for (i = 0; i < size; i++) {
		if (ktime_to_ms(e_slwl_prev_time[i]) > ktime_to_ms(time_zero)) {
			strncpy(slwl[i].slwl_name, e_slwl_name[i], SLEEP_MON_SLWL_NAME_LENGTH);
			slwl[i].prevent_time = e_slwl_prev_time[i];
		}
	}

	init_energy_mon_slwl_data();
}
#endif
static void init_slwl_data(void)
{
	struct slp_mon_slwl* iter;
	ktime_t ktime_zero = ktime_set(0, 0);
	int i = 0;

	for(i = 0; i < slwl_num; i++) {
		memset(slwl_name[i], 0, SLEEP_MON_SLWL_ARRAY_SIZE);
		slwl_prev_time[i] = ktime_set(0, 0);
		slwl_idx[i] = 0;
	}
	slwl_num = 0;

	list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
		iter->prevent_time = ktime_zero;
		iter->lock_time = ktime_zero;
		iter->unlock_time = ktime_zero;
	}
}

static int set_slp_mon_slwl_prv_time(char *name)
{
	struct slp_mon_slwl* iter;
	ktime_t temp = ktime_set(0, 0);

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
		if (!(strncmp(iter->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH - 1))) {
			iter->unlock_time = ktime_get();
			temp = ktime_sub(iter->unlock_time, iter->lock_time);
			iter->prevent_time = ktime_add(iter->prevent_time, temp);
			rcu_read_unlock();
			return 0;
		}
	}
	rcu_read_unlock();

	return 1;
}

int slp_mon_slwl_lock(const char *buf)
{
	const char *str = buf;
	struct task_struct *task;
	int pid = 0;
	size_t len;
	int ret = 0;

	while (*str && !isspace(*str))
		str++;

	len = str - buf;
	if (!len)
		return -EINVAL;

	if (*str && *str != '\n') {
		/* Find out if there's a pid string appended. */
		ret = kstrtos32(skip_spaces(str), 10, &pid);
		if (ret)
			return -EINVAL;
	}
	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}
	rcu_read_unlock();
	add_slp_mon_slwl_list(task->comm);
#ifdef CONFIG_ENERGY_MONITOR
	add_energy_mon_slwl_list(task->comm);
#endif

	return 0;
}

int slp_mon_slwl_unlock(const char *buf)
{
	const char *str = buf;
	struct task_struct *task;
	int pid = 0;
	size_t len;
	int ret = 0;

	while (*str && !isspace(*str))
		str++;

	len = str - buf;
	if (!len)
		return -EINVAL;

	if (*str && *str != '\n') {
		/* Find out if there's a pid string appended. */
		ret = kstrtos32(skip_spaces(str), 10, &pid);
		if (ret)
			return -EINVAL;
	}

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}
	rcu_read_unlock();
	ret = set_slp_mon_slwl_prv_time(task->comm);
	if (ret)
		pr_info("%s : can't find slave wake lock\n", __func__);
#ifdef CONFIG_ENERGY_MONITOR
	ret = set_energy_mon_slwl_prv_time(task->comm);
	if (ret)
		pr_info("%s : can't find energy_mon slave wake lock\n", __func__);
#endif

	return -EINVAL;
}

int add_slp_mon_slwl_list(char *name)
{
	struct slp_mon_slwl *slwl_ins, *iter;
	int i = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
		if (!strncmp(iter->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH - 1)) {
			iter->lock_time = ktime_get();
			iter->unlock_time = ktime_set(0, 0);
			rcu_read_unlock();
			return 1;
		}
		i++;
	}
	rcu_read_unlock();
	if ((slwl_ins = kmalloc(sizeof(struct slp_mon_slwl), GFP_KERNEL)) == NULL)
		return 0;

	memset(slwl_ins->slwl_name, 0, SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(slwl_ins->slwl_name, name, SLEEP_MON_SLWL_NAME_LENGTH);
	slwl_ins->slwl_name[SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
	slwl_ins->lock_time = ktime_get();
	slwl_ins->unlock_time = ktime_set(0, 0);
	slwl_ins->prevent_time = ktime_set(0, 0);
	list_add_tail(&slwl_ins->entry, &slp_mon_slwl_list);

	return 1;
}

static void slp_mon_swap(int i, int j)
{
	ktime_t temp_time = ktime_set(0, 0);
	char temp_name[SLEEP_MON_SLWL_NAME_LENGTH] = {0, };
	int temp_idx = 0;

	temp_time = slwl_prev_time[i];
	slwl_prev_time[i] = slwl_prev_time[j];
	slwl_prev_time[j] = temp_time;

	strncpy(temp_name, slwl_name[i], SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(slwl_name[i], slwl_name[j], SLEEP_MON_SLWL_NAME_LENGTH);
	strncpy(slwl_name[j], temp_name, SLEEP_MON_SLWL_NAME_LENGTH);
	slwl_name[i][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
	slwl_name[j][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;

	temp_idx = slwl_idx[i];
	slwl_idx[i] = slwl_idx[j];
	slwl_idx[j] = temp_idx;
}

static void slp_mon_sort_prevent_time(void)
{
	int i = 0, idx = 0, j = 0;
	struct slp_mon_slwl *iter;

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
		if (slwl_num < SLEEP_MON_SLWL_ARRAY_SIZE) {
			if (ktime_to_ms(iter->prevent_time) > 0) {
				slwl_prev_time[slwl_num] = iter->prevent_time;
				slwl_idx[slwl_num] = idx;
				strncpy(slwl_name[slwl_num], iter->slwl_name, SLEEP_MON_SLWL_NAME_LENGTH);
				slwl_name[slwl_num][SLEEP_MON_SLWL_NAME_LENGTH - 1] = 0;
				slwl_num++;
			}
		} else
			break;

		idx++;
	}
	rcu_read_unlock();

	for (i = 0; i < SLEEP_MON_SLWL_ARRAY_SIZE; i++) {
		for (j = i + 1; j < SLEEP_MON_SLWL_ARRAY_SIZE; j++) {
			if (i == j)
				continue;

			if (ktime_to_ms(slwl_prev_time[i]) < ktime_to_ms(slwl_prev_time[j]))
				slp_mon_swap(i, j);
		}
	}

	if (slwl_num < SLEEP_MON_SLWL_ARRAY_SIZE)
			return ;

	idx = 0;
	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
		if (ktime_to_ms(iter->prevent_time) > 0) {
			for (i = 0; i < SLEEP_MON_SLWL_ARRAY_SIZE; i++) {
				if (!strncmp(iter->slwl_name, slwl_name[i],
						(strlen(iter->slwl_name) > strlen(slwl_name[i])) ?
							strlen(slwl_name[i]) : strlen(iter->slwl_name))) {
					break;
				}

				if (ktime_to_ms(iter->prevent_time) > ktime_to_ms(slwl_prev_time[i])) {
					for (j = SLEEP_MON_SLWL_ARRAY_SIZE - 1; j > i; j--) {
						strncpy(slwl_name[j], slwl_name[j - 1], SLEEP_MON_SLWL_NAME_LENGTH);
						slwl_prev_time[j] = slwl_prev_time[j - 1];
						slwl_idx[j] = slwl_idx[j - 1];
					}
					slwl_prev_time[i] = iter->prevent_time;
					slwl_idx[i] = idx;
					strncpy(slwl_name[i], iter->slwl_name, SLEEP_MON_SLWL_NAME_LENGTH);
					break;
				}
			}
		}
		idx++;
	}
	rcu_read_unlock();
}

static int slp_mon_slwl_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slp_mon_slwl *iter;
	int i = 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND)
		slp_mon_sort_prevent_time();

	if (slwl_num < 1)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
			if (slwl_idx[0] == i) {
				*raw_val |= i << SLEEP_MON_SLWL_IDX_BIT;
				if (ktime_to_ms(iter->prevent_time) > SLEEP_MON_SLWL_PREVENT_TIME_MAX)
					iter->prevent_time =
					ktime_set(0, SLEEP_MON_SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->prevent_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (slwl_num == 1)
		init_slwl_data();

	return 1;
}

static int slp_mon_slwl1_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slp_mon_slwl *iter;
	int i = 0;

	if (slwl_num < 2)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
			if (slwl_idx[1] == i) {
				*raw_val |= i << SLEEP_MON_SLWL_IDX_BIT;
				if (ktime_to_ms(iter->prevent_time) > SLEEP_MON_SLWL_PREVENT_TIME_MAX)
					iter->prevent_time =
					ktime_set(0, SLEEP_MON_SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->prevent_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (slwl_num == 2)
		init_slwl_data();

	return 1;
}

static int slp_mon_slwl2_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slp_mon_slwl *iter;
	int i = 0;

	if (slwl_num < 3)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
			if (slwl_idx[2] == i) {
				*raw_val |= i << SLEEP_MON_SLWL_IDX_BIT;
				if (ktime_to_ms(iter->prevent_time) > SLEEP_MON_SLWL_PREVENT_TIME_MAX)
					iter->prevent_time =
					ktime_set(0, SLEEP_MON_SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->prevent_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (slwl_num == 3)
		init_slwl_data();

	return 1;
}


static int slp_mon_slwl3_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slp_mon_slwl *iter;
	int i = 0;

	if (slwl_num < 4)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
			if (slwl_idx[3] == i) {
				*raw_val |= i << SLEEP_MON_SLWL_IDX_BIT;
				if (ktime_to_ms(iter->prevent_time) > SLEEP_MON_SLWL_PREVENT_TIME_MAX)
					iter->prevent_time =
					ktime_set(0, SLEEP_MON_SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->prevent_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (slwl_num == 4)
		init_slwl_data();

	return 1;
}


static struct sleep_monitor_ops slp_mon_slwl_dev = {
	.read_cb_func = slp_mon_slwl_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev1 = {
	.read_cb_func = slp_mon_slwl1_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev2 = {
	.read_cb_func = slp_mon_slwl2_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev3 = {
	.read_cb_func = slp_mon_slwl3_cb,
};

static ssize_t slp_mon_read_slwl_list(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int i = 0;
	char *buf = NULL;
	struct slp_mon_slwl *iter;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = (char*)kmalloc(4 * SLEEP_MONITOR_ONE_LINE_RAW_SIZE * sizeof(char), GFP_KERNEL);

	if (!buf)
		return -1;

	memset(buf, 0, SLEEP_MONITOR_ONE_LINE_RAW_SIZE * sizeof(char));

	if (*ppos == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%08x]%s", special_key,
						get_type_marker(SLEEP_MONITOR_CALL_SLWL_LIST));
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slp_mon_slwl_list, entry) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%s/", iter->slwl_name);
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

static int slp_mon_slwl_pm_post_suspend_cb(void)
{
	struct slp_mon_slwl *iter;

	rcu_read_lock();
	list_for_each_entry(iter, &slp_mon_slwl_list, entry) {
		iter->prevent_time = ktime_set(0, 0);
	}
	rcu_read_unlock();

	return 0;
}

static int slp_mon_slwl_notifier(struct notifier_block *nb,
								unsigned long event, void *dummy)
{
	switch (event) {
		case PM_POST_SUSPEND:
			slp_mon_slwl_pm_post_suspend_cb();
			init_slwl_data();
			break;
		default:
			break;
	}
	return 0;
}

static struct notifier_block slp_mon_slwl_notifier_ops = {
	.notifier_call = slp_mon_slwl_notifier,
};


static const struct file_operations slp_mon_slwl_list_ops = {
        .read           = slp_mon_read_slwl_list,
};

static int slp_mon_slwl_dev_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev, SLEEP_MONITOR_SLWL);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev1, SLEEP_MONITOR_SLWL1);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev2, SLEEP_MONITOR_SLWL2);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev3, SLEEP_MONITOR_SLWL3);

	ret = register_pm_notifier(&slp_mon_slwl_notifier_ops);
	if (ret)
		pr_info("%s - error register_pm_notifier\n", __func__);


	if (slp_mon_d) {
		if (!debugfs_create_file("slp_mon_slwl", S_IRUSR
			, slp_mon_d, NULL, &slp_mon_slwl_list_ops)) \
				pr_err("%s : debugfs_create_file, error\n", "slp_mon_slwl");
	} else
		pr_info("%s - entry not defined\n", __func__);

	return 0;
}

static void slp_mon_slwl_dev_exit(void)
{
	pr_info("%s\n", __func__);

}

module_init(slp_mon_slwl_dev_init);
module_exit(slp_mon_slwl_dev_exit);
