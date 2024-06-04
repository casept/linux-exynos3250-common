/*
 * s2mpw01-core.c
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
#define S2MPW01_CORE_C

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/mfd/samsung/s2mpw01.h>
#include <linux/mfd/samsung/rtc-s2mp.h>
#include <linux/regmap.h>

#ifdef CONFIG_DISPLAY_EARLY_DPMS
#include <drm/exynos_drm.h>
#endif

int sec_reg_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
	return regmap_read(sec_pmic->regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sec_reg_read);

int sec_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count, u8 *buf)
{
	return regmap_bulk_read(sec_pmic->regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_bulk_read);

int sec_reg_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
	return regmap_write(sec_pmic->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sec_reg_write);

int sec_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count, u8 *buf)
{
	return regmap_raw_write(sec_pmic->regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_bulk_write);

int sec_reg_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val, u32 mask)
{
	return regmap_update_bits(sec_pmic->regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(sec_reg_update);

/* -------- PMIC -------------*/
int sec_pmic_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
	return regmap_read(sec_pmic->pmic_regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sec_pmic_read);

int sec_pmic_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_bulk_read(sec_pmic->pmic_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_pmic_bulk_read);

int sec_pmic_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
	return regmap_write(sec_pmic->pmic_regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sec_pmic_write);

int sec_pmic_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_raw_write(sec_pmic->pmic_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_pmic_bulk_write);

int sec_pmic_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val,
		u32 mask)
{
	return regmap_update_bits(sec_pmic->pmic_regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(sec_pmic_update);

/* -------- RTC -------------*/
int sec_rtc_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
	return regmap_read(sec_pmic->rtc_regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sec_rtc_read);

int sec_rtc_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_bulk_read(sec_pmic->rtc_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_rtc_bulk_read);

int sec_rtc_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
	return regmap_write(sec_pmic->rtc_regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sec_rtc_write);

int sec_rtc_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_raw_write(sec_pmic->rtc_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_rtc_bulk_write);

int sec_rtc_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val,
		u32 mask)
{
	return regmap_update_bits(sec_pmic->rtc_regmap, reg, mask, val);
}

/* -------- CHG -------------*/
int sec_chg_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
	return regmap_read(sec_pmic->chg_regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sec_chg_read);

int sec_chg_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_bulk_read(sec_pmic->chg_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_chg_bulk_read);

int sec_chg_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
	return regmap_write(sec_pmic->chg_regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sec_chg_write);

int sec_chg_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_raw_write(sec_pmic->chg_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_chg_bulk_write);

int sec_chg_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val,
		u32 mask)
{
	return regmap_update_bits(sec_pmic->chg_regmap, reg, mask, val);
}

/* -------- FG -------------*/
int sec_fg_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
	return regmap_read(sec_pmic->fg_regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sec_fg_read);

int sec_fg_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_bulk_read(sec_pmic->fg_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_fg_bulk_read);

int sec_fg_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
	return regmap_write(sec_pmic->fg_regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sec_fg_write);

int sec_fg_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
	return regmap_raw_write(sec_pmic->fg_regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sec_fg_bulk_write);

int sec_fg_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val,
		u32 mask)
{
	return regmap_update_bits(sec_pmic->fg_regmap, reg, mask, val);
}


EXPORT_SYMBOL_GPL(sec_rtc_update);

static struct regmap_config sec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int sec_pmic_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct sec_pmic_platform_data *pdata = i2c->dev.platform_data;
	struct sec_pmic_dev *sec_pmic;
	unsigned int reg_data=0;
	int ret = 0;

	sec_pmic = devm_kzalloc(&i2c->dev, sizeof(struct sec_pmic_dev),
				GFP_KERNEL);
	if (sec_pmic == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sec_pmic);
	sec_pmic->dev = &i2c->dev;
	sec_pmic->i2c = i2c;
	sec_pmic->irq = i2c->irq;
	sec_pmic->type = id->driver_data;
	if (pdata) {
		sec_pmic->device_type = pdata->device_type;
		sec_pmic->ono = pdata->ono;
		sec_pmic->irq_base = pdata->irq_base;
		sec_pmic->wakeup = pdata->wakeup;
		sec_pmic->using_rid_detect = pdata->using_rid_detect;
	}

	sec_pmic->regmap = regmap_init_i2c(i2c, &sec_regmap_config);
	if (IS_ERR(sec_pmic->regmap)) {
		ret = PTR_ERR(sec_pmic->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		goto err4;
	}

	mutex_init(&sec_pmic->iolock);
	if (sec_reg_read(sec_pmic, S2MPW01_PMIC_REG_PMICID, &reg_data) < 0) {
		dev_err(sec_pmic->dev,
			"device not found on this channel (this is not an error)\n");
		ret = -ENODEV;
		goto err2;
	} else {
		/* print rev_num */
		sec_pmic->rev_num = (reg_data & 0x0F);
		dev_info(&i2c->dev, "%s() device found: rev_num : 0x%x\n",
				__func__, sec_pmic->rev_num);
	}

	sec_pmic->pmic = i2c_new_dummy(i2c->adapter, PMIC_I2C_ADDR);
	if (IS_ERR(sec_pmic->pmic)) {
		ret = PTR_ERR(sec_pmic->pmic);
		dev_err(&i2c->dev, "Address %02x unavailable\n", PMIC_I2C_ADDR);
		goto err3;
	}

	i2c_set_clientdata(sec_pmic->pmic, sec_pmic);
	sec_pmic->pmic_regmap = regmap_init_i2c(sec_pmic->pmic, &sec_regmap_config);
	if (IS_ERR(sec_pmic->pmic_regmap)) {
		ret = PTR_ERR(sec_pmic->pmic_regmap);
		dev_err(&sec_pmic->pmic->dev,
				"Failed to allocate register map: %d\n", ret);
		goto err2;
	}

	sec_pmic->rtc = i2c_new_dummy(i2c->adapter, RTC_I2C_ADDR);
	if (IS_ERR(sec_pmic->rtc)) {
		ret = PTR_ERR(sec_pmic->rtc);
		dev_err(&i2c->dev, "Address %02x unavailable\n", RTC_I2C_ADDR);
		goto err3;
	}

	i2c_set_clientdata(sec_pmic->rtc, sec_pmic);
	sec_pmic->rtc_regmap = regmap_init_i2c(sec_pmic->rtc, &sec_regmap_config);
	if (IS_ERR(sec_pmic->rtc_regmap)) {
		ret = PTR_ERR(sec_pmic->rtc_regmap);
		dev_err(&sec_pmic->rtc->dev,
				"Failed to allocate register map: %d\n", ret);
		goto err2;
	}

	sec_pmic->chg = i2c_new_dummy(i2c->adapter, CHG_I2C_ADDR);
	if (IS_ERR(sec_pmic->chg)) {
		ret = PTR_ERR(sec_pmic->chg);
		dev_err(&i2c->dev, "Address %02x unavailable\n", CHG_I2C_ADDR);
		goto err3;
	}

	i2c_set_clientdata(sec_pmic->chg, sec_pmic);
	sec_pmic->chg_regmap = regmap_init_i2c(sec_pmic->chg, &sec_regmap_config);
	if (IS_ERR(sec_pmic->chg_regmap)) {
		ret = PTR_ERR(sec_pmic->chg_regmap);
		dev_err(&sec_pmic->chg->dev,
				"Failed to allocate register map: %d\n", ret);
		goto err2;
	}

	sec_pmic->fg = i2c_new_dummy(i2c->adapter, FG_I2C_ADDR);
	if (IS_ERR(sec_pmic->fg)) {
		ret = PTR_ERR(sec_pmic->fg);
		dev_err(&i2c->dev, "Address %02x unavailable\n", FG_I2C_ADDR);
		goto err3;
	}

	i2c_set_clientdata(sec_pmic->fg, sec_pmic);
	sec_pmic->fg_regmap = regmap_init_i2c(sec_pmic->fg, &sec_regmap_config);
	if (IS_ERR(sec_pmic->fg_regmap)) {
		ret = PTR_ERR(sec_pmic->fg_regmap);
		dev_err(&sec_pmic->fg->dev,
				"Failed to allocate register map: %d\n", ret);
		goto err2;
	}

	if (pdata && pdata->cfg_pmic_wrst)
		pdata->cfg_pmic_wrst();

	if (pdata && pdata->cfg_pmic_irq)
		pdata->cfg_pmic_irq();

	s2mpw01_irq_init(sec_pmic);

	pm_runtime_set_active(sec_pmic->dev);
	switch (sec_pmic->device_type) {
	case S2MPW01:
		ret = mfd_add_devices(sec_pmic->dev, -1, pdata->mfd_devs,
					pdata->num_mfd_devs, NULL, 0);
		break;
	default:
		/* If this happens the probe function is problem */
		BUG();
	}
	if (ret < 0)
		goto err1;

#ifdef CONFIG_DISPLAY_EARLY_DPMS
	device_set_early_complete(sec_pmic->dev, EARLY_COMP_SLAVE);
#endif

	return ret;

err1:
	s2mpw01_irq_exit(sec_pmic);
	regmap_exit(sec_pmic->pmic_regmap);
	regmap_exit(sec_pmic->rtc_regmap);
	regmap_exit(sec_pmic->chg_regmap);
	regmap_exit(sec_pmic->fg_regmap);
err2:
	i2c_unregister_device(sec_pmic->pmic);
	i2c_unregister_device(sec_pmic->rtc);
	i2c_unregister_device(sec_pmic->chg);
	i2c_unregister_device(sec_pmic->fg);
err3:
	regmap_exit(sec_pmic->regmap);
err4:
	return ret;
}

static int sec_pmic_remove(struct i2c_client *i2c)
{
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	mfd_remove_devices(sec_pmic->dev);
	s2mpw01_irq_exit(sec_pmic);
	regmap_exit(sec_pmic->rtc_regmap);
	i2c_unregister_device(sec_pmic->rtc);
	regmap_exit(sec_pmic->regmap);
	return 0;
}

static const struct i2c_device_id sec_pmic_id[] = {
	{ "sec-pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_pmic_id);

#ifdef CONFIG_PM
static int sec_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	if (sec_pmic->wakeup)
		enable_irq_wake(sec_pmic->irq);

	disable_irq(sec_pmic->irq);
	return 0;
}

static int sec_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	if (sec_pmic->wakeup)
		disable_irq_wake(sec_pmic->irq);

	enable_irq(sec_pmic->irq);
	return 0;
}
#else
#define sec_suspend	NULL
#define sec_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops sec_pmic_apm = {
	.suspend_late = sec_suspend,
	.resume_early = sec_resume,
};

static struct i2c_driver sec_pmic_driver = {
	.driver = {
		   .name = "sec-pmic",
		   .owner = THIS_MODULE,
		   .pm = &sec_pmic_apm,
	},
	.probe = sec_pmic_probe,
	.remove = sec_pmic_remove,
	.id_table = sec_pmic_id,
};

static int __init sec_pmic_init(void)
{
	return i2c_add_driver(&sec_pmic_driver);
}

subsys_initcall(sec_pmic_init);

static void __exit sec_pmic_exit(void)
{
	i2c_del_driver(&sec_pmic_driver);
}
module_exit(sec_pmic_exit);

MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("Core support for the SAMSUNG MFD");
MODULE_LICENSE("GPL");
