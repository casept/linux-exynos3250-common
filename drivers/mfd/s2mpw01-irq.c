/*
 * s2mpw01-irq.c
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/mfd/samsung/s2mpw01-irq.h>
#include <linux/mfd/samsung/s2mpw01.h>
#if defined(CONFIG_CHARGER_S2MPW01)
#include <linux/battery/charger/s2mpw01_charger.h>
#endif
#if defined(CONFIG_FUELGAUGE_S2MPW01)
#include <linux/battery/fuelgauge/s2mpw01_fuelgauge.h>
#endif
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/slp_mon_irq_dev.h>
#endif
#ifdef CONFIG_PM_SLEEP_HISTORY
#include <linux/power/sleep_history.h>
#endif
#ifdef CONFIG_ENERGY_MONITOR
#include <linux/power/energy_monitor.h>
#endif

static const u8 s2mpw01_mask_reg[] = {
	/* TODO: Need to check other INTMASK */
	[PMIC_INT1] = S2MPW01_PMIC_REG_INT1M,
	[PMIC_INT2] = S2MPW01_PMIC_REG_INT2M,
	[PMIC_INT3] = S2MPW01_PMIC_REG_INT3M,
#if defined(CONFIG_CHARGER_S2MPW01)
	[CHG_INT1] = S2MPW01_CHG_REG_INT1M,
	[CHG_INT2] = S2MPW01_CHG_REG_INT2M,
	[CHG_INT3] = S2MPW01_CHG_REG_INT3M,
#endif
};

struct s2mpw01_irq_data {
	int mask;
	enum s2mpw01_irq_source group;
};

short g_check_pmic_int;

#define DECLARE_IRQ(idx, _group, _mask)	\
	[(idx)] = {.group = (_group), .mask = (_mask)}
static const struct s2mpw01_irq_data s2mpw01_irqs[] = {
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_PWRONR_INT1,	PMIC_INT1, 1 << 1),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_PWRONF_INT1,	PMIC_INT1, 1 << 0),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_JIGONBF_INT1,	PMIC_INT1, 1 << 2),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_JIGONBR_INT1,	PMIC_INT1, 1 << 3),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_ACOKBF_INT1,	PMIC_INT1, 1 << 4),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_ACOKBR_INT1,	PMIC_INT1, 1 << 5),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_PWRON1S_INT1,	PMIC_INT1, 1 << 6),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_MRB_INT1,		PMIC_INT1, 1 << 7),

	DECLARE_IRQ(S2MPW01_PMIC_IRQ_RTC60S_INT2,	PMIC_INT2, 1 << 0),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_RTCA1_INT2,	PMIC_INT2, 1 << 1),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_RTCA0_INT2,	PMIC_INT2, 1 << 2),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_SMPL_INT2,		PMIC_INT2, 1 << 3),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_RTC1S_INT2,	PMIC_INT2, 1 << 4),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_WTSR_INT2,		PMIC_INT2, 1 << 5),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_WRSTB_INT2,	PMIC_INT2, 1 << 7),

	DECLARE_IRQ(S2MPW01_PMIC_IRQ_120C_INT3,		PMIC_INT3, 1 << 0),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_140C_INT3,		PMIC_INT3, 1 << 1),
	DECLARE_IRQ(S2MPW01_PMIC_IRQ_TSD_INT3,		PMIC_INT3, 1 << 2),
#if defined(CONFIG_CHARGER_S2MPW01)
	DECLARE_IRQ(S2MPW01_CHG_IRQ_RECHG_INT1,		CHG_INT1, 1 << 0),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGDONE_INT1,	CHG_INT1, 1 << 1),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_TOPOFF_INT1,	CHG_INT1, 1 << 2),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_PREECHG_INT1,	CHG_INT1, 1 << 3),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGSTS_INT1,	CHG_INT1, 1 << 4),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CIN2BAT_INT1,	CHG_INT1, 1 << 5),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGVINOVP_INT1,	CHG_INT1, 1 << 6),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGVIN_INT1,	CHG_INT1, 1 << 7),

	DECLARE_IRQ(S2MPW01_CHG_IRQ_UART_BOOT_OFF_INT2,	CHG_INT2, 1 << 0),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_USB_BOOT_ON_INT2,		CHG_INT2, 1 << 1),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_USB_BOOT_OFF_INT2,	CHG_INT2, 1 << 2),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_JIGON_INT2,	CHG_INT2, 1 << 3),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_ADPATH_INT2,	CHG_INT2, 1 << 4),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_FCHG_INT2,		CHG_INT2, 1 << 5),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGIN_INPUT_INT2,		CHG_INT2, 1 << 6),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_BATDET_INT2,	CHG_INT2, 1 << 7),

	DECLARE_IRQ(S2MPW01_CHG_IRQ_CVOK_INT3,		CHG_INT3, 1 << 0),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_UART_CABLE_INT3,		CHG_INT3, 1 << 1),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_FACT_LEAKAGE_INT3,		CHG_INT3, 1 << 2),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_UART_BOOT_ON_INT3,		CHG_INT3, 1 << 3),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_TMROUT_INT3,	CHG_INT3, 1 << 4),
	DECLARE_IRQ(S2MPW01_CHG_IRQ_CHGTSDINT3,		CHG_INT3, 1 << 5),
#endif
};

static void s2mpw01_irq_lock(struct irq_data *data)
{
	struct sec_pmic_dev *s2mpw01 = irq_get_chip_data(data->irq);

	mutex_lock(&s2mpw01->irqlock);
}

static void s2mpw01_irq_sync_unlock(struct irq_data *data)
{
	struct sec_pmic_dev *s2mpw01 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < S2MPW01_IRQ_GROUP_NR; i++) {
		s2mpw01->irq_masks_cache[i] = s2mpw01->irq_masks_cur[i];

		switch (i) {
	 	case PMIC_INT1 ... PMIC_INT3:
	 		sec_pmic_write(s2mpw01, s2mpw01_mask_reg[i],
				s2mpw01->irq_masks_cur[i]);
			break;
#if defined(CONFIG_CHARGER_S2MPW01)
	 	case CHG_INT1 ... CHG_INT3:
	 		sec_chg_write(s2mpw01, s2mpw01_mask_reg[i],
				s2mpw01->irq_masks_cur[i]);
			break;
#endif
	 	default:
	 		dev_err(s2mpw01->dev, "%s() Failed to sync unlock interrupts\n",
				__func__);
		}

	}

	mutex_unlock(&s2mpw01->irqlock);
}

static const inline struct s2mpw01_irq_data *
irq_to_s2mpw01_irq(struct sec_pmic_dev *s2mpw01, int irq)
{
	return &s2mpw01_irqs[irq - s2mpw01->irq_base];
}

static void s2mpw01_irq_mask(struct irq_data *data)
{
	struct sec_pmic_dev *s2mpw01 = irq_get_chip_data(data->irq);
	const struct s2mpw01_irq_data *irq_data =
	    irq_to_s2mpw01_irq(s2mpw01, data->irq);

	if (irq_data->group >= S2MPW01_IRQ_GROUP_NR)
		return;

	s2mpw01->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void s2mpw01_irq_unmask(struct irq_data *data)
{
	struct sec_pmic_dev *s2mpw01 = irq_get_chip_data(data->irq);
	const struct s2mpw01_irq_data *irq_data =
	    irq_to_s2mpw01_irq(s2mpw01, data->irq);

	if (irq_data->group >= S2MPW01_IRQ_GROUP_NR)
		return;

	s2mpw01->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip s2mpw01_irq_chip = {
	.name			= "sec-pmic",
	.irq_bus_lock		= s2mpw01_irq_lock,
	.irq_bus_sync_unlock	= s2mpw01_irq_sync_unlock,
	.irq_mask		= s2mpw01_irq_mask,
	.irq_unmask		= s2mpw01_irq_unmask,
};

static irqreturn_t s2mpw01_irq_thread(int irq, void *data)
{
	struct sec_pmic_dev *s2mpw01 = data;
	struct device *dev = s2mpw01->dev;
	u8 irq_reg[S2MPW01_IRQ_GROUP_NR] = {[0 ... S2MPW01_IRQ_GROUP_NR-1] = 0};
	unsigned int irq_src;
	unsigned int chg_st1;
	int i, ret;
	short is_find = 0;

	ret = sec_reg_read(s2mpw01, S2MPW01_PMIC_REG_INTSRC, &irq_src);
	if (ret) {
		dev_err(dev, "%s() Failed to read interrupt source: %d\n",
				__func__, ret);
		return IRQ_NONE;
	}

	dev_info(dev, "%s() intr src(0x%02x)\n", __func__, irq_src);

	if (irq_src & S2MPW01_IRQSRC_PMIC) {
		/* PMIC_INT */
		ret = sec_pmic_bulk_read(s2mpw01, S2MPW01_PMIC_REG_INT1,
				S2MPW01_NUM_IRQ_PMIC_REGS, &irq_reg[PMIC_INT1]);
		if (ret) {
			dev_err(dev, "%s() Failed to read pmic interrupt: %d\n",
				__func__, ret);
			return IRQ_NONE;
		}

		dev_info(dev, "%s() PMM intr(0x%02x, 0x%02x, 0x%02x)\n", __func__,
			irq_reg[PMIC_INT1], irq_reg[PMIC_INT2], irq_reg[PMIC_INT3]);

		if ((irq_reg[PMIC_INT1] & 0x30) == 0x30) {
			sec_chg_read(s2mpw01, S2MPW01_CHG_REG_STATUS1, &chg_st1);
			irq_reg[PMIC_INT1] &= 0xCF;
			if (chg_st1 & CHGVIN_STATUS_MASK)
				irq_reg[PMIC_INT1] |= 0x20;
			else
				irq_reg[PMIC_INT1] |= 0x10;
		}

		if (g_check_pmic_int == 1) {
			if ((irq_reg[PMIC_INT1] & 0x02) | (irq_reg[PMIC_INT1] & 0x01)) {
				pr_info("Resume caused by key_power\n");
#ifdef CONFIG_SLEEP_MONITOR
				add_slp_mon_irq_list(irq, "key_power");
#endif
			} else if ((irq_reg[PMIC_INT1] & 0x04) | (irq_reg[PMIC_INT1] & 0x08)) {
				pr_info("Resume caused by jig\n");
#ifdef CONFIG_SLEEP_MONITOR
				add_slp_mon_irq_list(irq, "jig");
#endif
			} else if ((irq_reg[PMIC_INT2] & 0x02) | (irq_reg[PMIC_INT2] & 0x04)) {
				pr_info("Resume caused by pmic-rtc\n");
#ifdef CONFIG_SLEEP_MONITOR
				add_slp_mon_irq_list(irq, "pmic-rtc");
#endif
			} else {
				pr_info("Resume caused by other-pmic-int\n");
#ifdef CONFIG_SLEEP_MONITOR
				add_slp_mon_irq_list(irq, "other-pmic-int");
#endif
			}
		}
	}
#ifdef CONFIG_CHARGER_S2MPW01
	if (irq_src & S2MPW01_IRQSRC_CHG) {
		/* CHG_INT */
		ret = sec_chg_bulk_read(s2mpw01, S2MPW01_CHG_REG_INT1,
				S2MPW01_NUM_IRQ_CHG_REGS, &irq_reg[CHG_INT1]);
		if (ret) {
			dev_err(dev, "%s() Failed to read charger interrupt: %d\n",
				__func__, ret);
			return IRQ_NONE;
		}

		dev_info(dev, "%s() CHAGER intr(0x%02x, 0x%02x, 0x%02x)\n",
				__func__, irq_reg[CHG_INT1], irq_reg[CHG_INT2],
				irq_reg[CHG_INT3]);
	}
#endif


	/* Apply masking */
	for (i = 0; i < S2MPW01_IRQ_GROUP_NR; i++)
		irq_reg[i] &= ~s2mpw01->irq_masks_cur[i];

	/* Report */
	for (i = 0; i < S2MPW01_IRQ_NR; i++) {
		if (irq_reg[s2mpw01_irqs[i].group] & s2mpw01_irqs[i].mask) {
			handle_nested_irq(s2mpw01->irq_base + i);
			if (is_find == 0 && g_check_pmic_int == 1) {
#ifdef CONFIG_PM_SLEEP_HISTORY
{
				int slp_his_irq = s2mpw01->irq_base + i;
				sleep_history_marker(SLEEP_HISTORY_WAKEUP_IRQ, NULL, (void *)&slp_his_irq);
}
#endif
#ifdef CONFIG_ENERGY_MONITOR
				energy_monitor_record_wakeup_reason(s2mpw01->irq_base + i);
#endif
				is_find = 1;
			}
		}
	}
	g_check_pmic_int = 0;

	return IRQ_HANDLED;
}

int s2mpw01_irq_init(struct sec_pmic_dev *s2mpw01)
{
	struct device *dev = s2mpw01->dev;
	int i;
	int ret;
	unsigned int i2c_data;

	if (!s2mpw01->irq) {
		dev_warn(dev, "No interrupt specified.\n");
		s2mpw01->irq_base = 0;
		return 0;
	}

	if (!s2mpw01->irq_base) {
		dev_err(dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&s2mpw01->irqlock);
	dev_info(dev, "%s() irq_base=%d, irq=%d\n", __func__,
			s2mpw01->irq_base, s2mpw01->irq);

	/* Mask individual interrupt sources */
	for (i = 0; i < S2MPW01_IRQ_GROUP_NR; i++) {
		s2mpw01->irq_masks_cur[i] = 0xff;
		s2mpw01->irq_masks_cache[i] = 0xff;

	 	switch (i) {
	 	case PMIC_INT1 ... PMIC_INT3:
	 		sec_pmic_write(s2mpw01, s2mpw01_mask_reg[i], 0xff);
			break;
#if defined(CONFIG_CHARGER_S2MPW01)
	 	case CHG_INT1 ... CHG_INT3:
	 		sec_chg_write(s2mpw01, s2mpw01_mask_reg[i], 0xff);
			break;
#endif
	 	default:
	 		dev_err(s2mpw01->dev, "%s() Failed to init interrupts\n",
				__func__);
		}

	}

	/* Register with genirq */
	for (i = 0; i < S2MPW01_IRQ_NR; i++) {
		int cur_irq;
		cur_irq = i + s2mpw01->irq_base;
		irq_set_chip_data(cur_irq, s2mpw01);
		irq_set_chip_and_handler(cur_irq, &s2mpw01_irq_chip,
					handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	ret = sec_reg_read(s2mpw01, S2MPW01_PMIC_REG_INTSRC_MASK,
			  &i2c_data);
	if (ret) {
		pr_err("%s: fail to read intsrc mask reg\n", __func__);
		return ret;
	}

	/* Unmask pmic interrupt : already sub-masked its section. */
	i2c_data &= ~(S2MPW01_IRQSRC_PMIC);
	i2c_data |= (S2MPW01_IRQSRC_FG);
#if defined(CONFIG_CHARGER_S2MPW01)
	 /* Unmask charger interrupt */
	i2c_data &= ~(S2MPW01_IRQSRC_CHG);
#endif

	sec_reg_write(s2mpw01, S2MPW01_PMIC_REG_INTSRC_MASK, i2c_data);

	ret = request_threaded_irq(s2mpw01->irq, NULL, s2mpw01_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "s2mpw01-irq", s2mpw01);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d: %d\n",
			s2mpw01->irq, ret);
		return ret;
	}

	return 0;
}

void s2mpw01_irq_exit(struct sec_pmic_dev *s2mpw01)
{
	if (s2mpw01->irq)
		free_irq(s2mpw01->irq, s2mpw01);
}

