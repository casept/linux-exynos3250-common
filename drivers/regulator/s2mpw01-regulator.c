/*
 * s2mpw01-regulator.c
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

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/samsung/s2mpw01-core.h>
#include <linux/mfd/samsung/s2mpw01.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif


struct s2mpw01_info {
	struct device *dev;
	struct sec_pmic_dev *iodev;
	int num_regulators;
	struct regulator_dev **rdev;
	struct sec_opmode_data *opmode_data;
	int ramp_delay1234;

#ifdef CONFIG_DEBUG_FS
	struct dentry			*reg_debugfs_dir;
#endif
};

struct s2mpw01_voltage_desc {
	int max;
	int min;
	int step;
};

static const struct s2mpw01_voltage_desc buck_voltage_val1 = {
	.max = 1300000,
	.min =  400000,
	.step =   6250,
};

static const struct s2mpw01_voltage_desc buck_voltage_val2 = {
	.max = 2100000,
	.min =  600000,
	.step =  12500,
};

static const struct s2mpw01_voltage_desc ldo_voltage_val1 = {
	.max = 1200000,
	.min =  800000,
	.step =  12500,
};

static const struct s2mpw01_voltage_desc ldo_voltage_val2 = {
	.max = 1350000,
	.min =  400000,
	.step =  25000,
};

static const struct s2mpw01_voltage_desc ldo_voltage_val3 = {
	.max = 2000000,
	.min =  800000,
	.step =  25000,
};

static const struct s2mpw01_voltage_desc ldo_voltage_val4 = {
	.max = 3375000,
	.min = 1800000,
	.step =  25000,
};

static const struct s2mpw01_voltage_desc *reg_voltage_map[] = {
	[S2MPW01_LDO1] = &ldo_voltage_val2,
	[S2MPW01_LDO2] = &ldo_voltage_val2,
	[S2MPW01_LDO3] = &ldo_voltage_val3,
	[S2MPW01_LDO4] = &ldo_voltage_val1,
	[S2MPW01_LDO5] = &ldo_voltage_val1,
	[S2MPW01_LDO6] = &ldo_voltage_val4,
	[S2MPW01_LDO7] = &ldo_voltage_val1,
	[S2MPW01_LDO8] = &ldo_voltage_val3,
	[S2MPW01_LDO9] = &ldo_voltage_val4,
	[S2MPW01_LDO10] = &ldo_voltage_val3,
	[S2MPW01_LDO11] = &ldo_voltage_val4,
	[S2MPW01_LDO12] = &ldo_voltage_val4,
	[S2MPW01_LDO13] = &ldo_voltage_val3,
	[S2MPW01_LDO14] = &ldo_voltage_val4,
	[S2MPW01_LDO15] = &ldo_voltage_val3,
	[S2MPW01_LDO16] = &ldo_voltage_val4,
	[S2MPW01_LDO17] = &ldo_voltage_val3,
	[S2MPW01_LDO18] = &ldo_voltage_val4,
	[S2MPW01_LDO19] = &ldo_voltage_val4,
	[S2MPW01_LDO20] = &ldo_voltage_val4,
	[S2MPW01_LDO21] = &ldo_voltage_val3,
	[S2MPW01_LDO22] = &ldo_voltage_val1,
	[S2MPW01_LDO23] = &ldo_voltage_val4,
	[S2MPW01_BUCK1] = &buck_voltage_val1,
	[S2MPW01_BUCK2] = &buck_voltage_val1,
	[S2MPW01_BUCK3] = &buck_voltage_val2,
	[S2MPW01_BUCK4] = &buck_voltage_val2,
};

#ifdef CONFIG_DEBUG_FS

static char* reg_description[S2MPW01_PMIC_REG_LDO_DSCH3 + 1] = {
	"INT1    ", "INT2    ", "INT3    ", "INT1M   ", "INT2M   ", "INT3M   ", "STATUS1 ",
	"STATUS2 ", "PWRONSRC", "OFFSRC  ", "B/U CHG ", "RTC BUF ", "CTRL1   ", "CTRL2   ",
	"CTRL3   ", "ETC_OTP ", "UVLO_OTP", "CFG1    ", "CFG2    ", "B1CTRL1 ", "B1CTRL2 ",
	"B1CTRL3 ", "B1CTRL4 ", "B2CTRL1 ", "B2CTRL2 ", "B3CTRL1 ", "B3CTRL2 ", "B4CTRL1 ",
	"B4CTRL2 ", "RAMP    ", "BSTCTRL ", "L1DVS   ", "L1CTRL1 ", "L1CTRL2 ", "L2CTRL1 ",
	"L2CTRL2 ", "L3CTRL  ", "L4CTRL  ", "L5CTRL  ", "L6CTRL  ", "L7CTRL  ", "L8CTRL  ",
	"L9CTRL  ", "L10CTRL ", "L11CTRL ", "L12CTRL ", "L13CTRL ", "L14CTRL ", "L15CTRL ",
	"L16CTRL ", "L17CTRL ", "L18CTRL ", "L19CTRL ", "L20CTRL ", "L21CTRL ", "L22CTRL ",
	"L23CTRL ", "LDODSCH1", "LDODSCH2", "LDODSCH3",
};

static int s2mpw01_debugfs_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

static ssize_t s2mpw01_debugfs_read_registers(struct file *filp,
	char __user *buffer, size_t count, loff_t *ppos)
{
	struct s2mpw01_info *s2mpw01 = filp->private_data;
	int i;
	u8 regs_value[S2MPW01_PMIC_REG_LDO_DSCH3+1];
	u32 reg = 0;
	char *buf;
	size_t len = 0;
	ssize_t ret;

	if (!s2mpw01) {
		pr_err("%s : s2mpw01 is null\n", __func__);
		return 0;
	}

	if (*ppos != 0)
		return 0;

	if (count < sizeof(buf))
		return -ENOSPC;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += snprintf(buf + len, PAGE_SIZE - len, "Address Value Description\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "------- ----- ---------------------\n");

	reg = S2MPW01_PMIC_REG_INT1;
	ret = sec_pmic_bulk_read(s2mpw01->iodev, reg,
			S2MPW01_PMIC_REG_LDO_DSCH3 + 1, regs_value);
	if (!ret) {
		for (i = 0; i < S2MPW01_PMIC_REG_LDO_DSCH3+1; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
						" 0x%02x   0x%02x %s\n",
						i, regs_value[i], reg_description[i]);
		}
	}

	ret = simple_read_from_buffer(buffer, len, ppos, buf, PAGE_SIZE);
	kfree(buf);

	return ret;
}

static const struct file_operations s2mpw01_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = s2mpw01_debugfs_open,
	.read = s2mpw01_debugfs_read_registers,
};
#endif

static int s2mpw01_list_voltage(struct regulator_dev *rdev,
				unsigned int selector)
{
	const struct s2mpw01_voltage_desc *desc;
	int reg_id = rdev_get_id(rdev);
	int val;

	if (reg_id >= ARRAY_SIZE(reg_voltage_map) || reg_id < 0)
		return -EINVAL;

	desc = reg_voltage_map[reg_id];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val;
}

unsigned int s2mpw01_opmode_reg[][3] = {
	{0x3, 0x2, 0x1}, /* LDO1 */
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1}, /* LDO23 */
	{0x3, 0x2, 0x1}, /* BUCK1 */
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1},
	{0x3, 0x2, 0x1}, /* BUCK4 */
};

static int s2mpw01_get_register(struct regulator_dev *rdev,
	unsigned int *reg, int *pmic_en)
{
	int reg_id = rdev_get_id(rdev);
	unsigned int mode;
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);

	switch (reg_id) {
	case S2MPW01_LDO1 ... S2MPW01_LDO2:
		*reg = S2MPW01_PMIC_REG_L1CTRL1 + (reg_id - S2MPW01_LDO1) * 2;
		break;
	case S2MPW01_LDO3 ... S2MPW01_LDO23:
		*reg = S2MPW01_PMIC_REG_L3CTRL + (reg_id - S2MPW01_LDO3);
		break;
	case S2MPW01_BUCK1:
		*reg = S2MPW01_PMIC_REG_B1CTRL1;
		break;
	case S2MPW01_BUCK2 ... S2MPW01_BUCK4:
		*reg = S2MPW01_PMIC_REG_B2CTRL1 + (reg_id - S2MPW01_BUCK2) * 2;
		break;
	default:
		return -EINVAL;
	}

	mode = s2mpw01->opmode_data[reg_id].mode;
	*pmic_en = s2mpw01_opmode_reg[reg_id][mode] << S2MPW01_PMIC_EN_SHIFT;

	return 0;
}

static int s2mpw01_reg_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	unsigned int reg, val;
	int ret, mask = 0xc0, pmic_en;

	ret = s2mpw01_get_register(rdev, &reg, &pmic_en);
	if (ret == -EINVAL)
		return 1;
	else if (ret)
		return ret;

	ret = sec_pmic_read(s2mpw01->iodev, reg, &val);
	if (ret)
		return ret;

	switch (reg_id) {
	case S2MPW01_LDO1 ... S2MPW01_BUCK4:
		mask = 0xc0;
		break;
	default:
		return -EINVAL;
	}

	return (val & mask) == pmic_en;
}

static int s2mpw01_reg_enable(struct regulator_dev *rdev)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	unsigned int reg;
	int ret, mask, pmic_en;

	ret = s2mpw01_get_register(rdev, &reg, &pmic_en);
	if (ret)
		return ret;

	switch (reg_id) {
	case S2MPW01_LDO1 ... S2MPW01_BUCK4:
		mask = 0xc0;
		break;
	default:
		return -EINVAL;
	}

	return sec_pmic_update(s2mpw01->iodev, reg, pmic_en, mask);
}

static int s2mpw01_reg_disable(struct regulator_dev *rdev)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	unsigned int reg;
	int ret, mask, pmic_en;

	ret = s2mpw01_get_register(rdev, &reg, &pmic_en);
	if (ret)
		return ret;

	switch (reg_id) {
	case S2MPW01_LDO1 ... S2MPW01_BUCK4:
		mask = 0xc0;
		break;
	default:
		return -EINVAL;
	}

	return sec_pmic_update(s2mpw01->iodev, reg, ~mask, mask);
}

static int s2mpw01_get_voltage_register(struct regulator_dev *rdev,
		unsigned int *_reg)
{
	int reg_id = rdev_get_id(rdev);
	unsigned int reg;

	switch (reg_id) {
	case S2MPW01_LDO1 ... S2MPW01_LDO2:
		reg = S2MPW01_PMIC_REG_L1CTRL2 + (reg_id - S2MPW01_LDO1);
		break;
	case S2MPW01_LDO3 ... S2MPW01_LDO23:
		reg = S2MPW01_PMIC_REG_L3CTRL + (reg_id - S2MPW01_LDO3);
		break;
	case S2MPW01_BUCK1:
		reg = S2MPW01_PMIC_REG_B1CTRL4;
		break;
	case S2MPW01_BUCK2 ... S2MPW01_BUCK4:
		reg = S2MPW01_PMIC_REG_B2CTRL2 + (reg_id - S2MPW01_BUCK2) * 2;
		break;
	default:
		return -EINVAL;
	}

	*_reg = reg;

	return 0;
}

static int s2mpw01_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int mask, ret;
	int reg_id = rdev_get_id(rdev);
	unsigned int reg, val;

	ret = s2mpw01_get_voltage_register(rdev, &reg);
	if (ret)
		return ret;

	switch (reg_id) {
	case S2MPW01_BUCK1 ... S2MPW01_BUCK4:
		mask = 0xff;
		break;
	case S2MPW01_LDO1 ... S2MPW01_LDO23:
		mask = 0x3f;
		break;
	default:
		return -EINVAL;
	}

	ret = sec_pmic_read(s2mpw01->iodev, reg, &val);
	if (ret)
		return ret;

	val &= mask;

	return val;
}

static inline int s2mpw01_convert_voltage_to_sel(
		const struct s2mpw01_voltage_desc *desc,
		int min_vol, int max_vol)
{
	int selector = 0;

	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	selector = (min_vol - desc->min) / desc->step;

	if (desc->min + desc->step * selector > max_vol)
		return -EINVAL;

	return selector;
}

static int s2mpw01_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int min_vol = min_uV, max_vol = max_uV;
	const struct s2mpw01_voltage_desc *desc;
	int ret, reg_id = rdev_get_id(rdev);
	unsigned int reg, mask;
	int sel;

	mask = (reg_id < S2MPW01_BUCK1) ? 0x3f : 0xff;

	desc = reg_voltage_map[reg_id];

	sel = s2mpw01_convert_voltage_to_sel(desc, min_vol, max_vol);
	if (sel < 0)
		return sel;

	ret = s2mpw01_get_voltage_register(rdev, &reg);
	if (ret)
		return ret;

	ret = sec_pmic_update(s2mpw01->iodev, reg, sel, mask);
	*selector = sel;

	return ret;
}

static int s2mpw01_set_voltage_time_sel(struct regulator_dev *rdev,
					     unsigned int old_sel,
					     unsigned int new_sel)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	const struct s2mpw01_voltage_desc *desc;
	int reg_id = rdev_get_id(rdev);
	int ramp_delay = 0;

	switch (reg_id) {
	case S2MPW01_BUCK1 ... S2MPW01_BUCK4:
		ramp_delay = s2mpw01->ramp_delay1234;
		break;
	default:
		return -EINVAL;
	}

	desc = reg_voltage_map[reg_id];

	if (((old_sel < new_sel) && (reg_id >= S2MPW01_BUCK1)) && ramp_delay) {
		return DIV_ROUND_UP(desc->step * (new_sel - old_sel),
			ramp_delay * 1000);
	}

	return 0;
}

static int s2mpw01_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	struct s2mpw01_info *s2mpw01 = rdev_get_drvdata(rdev);
	int mask, ret;
	int reg_id = rdev_get_id(rdev);
	unsigned int reg;

	ret = s2mpw01_get_voltage_register(rdev, &reg);
	if (ret)
		return ret;

	switch (reg_id) {
	case S2MPW01_BUCK1 ... S2MPW01_BUCK4:
		mask = 0xff;
		break;
	case S2MPW01_LDO1 ... S2MPW01_LDO23:
		mask = 0x3f;
		break;
	default:
		return -EINVAL;
	}

	return sec_pmic_update(s2mpw01->iodev, reg, selector, mask);
}
#if 0
static int get_ramp_delay(int ramp_delay)
{
	unsigned char cnt = 0;

	ramp_delay /= 6;

	while (true) {
		ramp_delay = ramp_delay >> 1;
		if (ramp_delay == 0)
			break;
		cnt++;
	}
	return cnt;
}
#endif
static struct regulator_ops s2mpw01_ldo_ops = {
	.list_voltage		= s2mpw01_list_voltage,
	.is_enabled		= s2mpw01_reg_is_enabled,
	.enable			= s2mpw01_reg_enable,
	.disable		= s2mpw01_reg_disable,
	.get_voltage_sel	= s2mpw01_get_voltage_sel,
	.set_voltage		= s2mpw01_set_voltage,
	.set_voltage_time_sel	= s2mpw01_set_voltage_time_sel,
};

static struct regulator_ops s2mpw01_buck_ops = {
	.list_voltage		= s2mpw01_list_voltage,
	.is_enabled		= s2mpw01_reg_is_enabled,
	.enable			= s2mpw01_reg_enable,
	.disable		= s2mpw01_reg_disable,
	.get_voltage_sel	= s2mpw01_get_voltage_sel,
	.set_voltage_sel	= s2mpw01_set_voltage_sel,
	.set_voltage_time_sel	= s2mpw01_set_voltage_time_sel,
};

#define regulator_desc_ldo(num)		{	\
	.name		= "LDO"#num,		\
	.id		= S2MPW01_LDO##num,	\
	.ops		= &s2mpw01_ldo_ops,	\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,		\
}
#define regulator_desc_buck(num)	{	\
	.name		= "BUCK"#num,		\
	.id		= S2MPW01_BUCK##num,	\
	.ops		= &s2mpw01_buck_ops,	\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,		\
}

static struct regulator_desc regulators[] = {
	regulator_desc_ldo(1),
	regulator_desc_ldo(2),
	regulator_desc_ldo(3),
	regulator_desc_ldo(4),
	regulator_desc_ldo(5),
	regulator_desc_ldo(6),
	regulator_desc_ldo(7),
	regulator_desc_ldo(8),
	regulator_desc_ldo(9),
	regulator_desc_ldo(10),
	regulator_desc_ldo(11),
	regulator_desc_ldo(12),
	regulator_desc_ldo(13),
	regulator_desc_ldo(14),
	regulator_desc_ldo(15),
	regulator_desc_ldo(16),
	regulator_desc_ldo(17),
	regulator_desc_ldo(18),
	regulator_desc_ldo(19),
	regulator_desc_ldo(20),
	regulator_desc_ldo(21),
	regulator_desc_ldo(22),
	regulator_desc_ldo(23),
	regulator_desc_buck(1),
	regulator_desc_buck(2),
	regulator_desc_buck(3),
	regulator_desc_buck(4),
};

static __devinit int s2mpw01_pmic_probe(struct platform_device *pdev)
{
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sec_pmic_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct regulator_dev **rdev;
	struct s2mpw01_info *s2mpw01;
	int i, ret, size;

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpw01 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpw01_info),
				GFP_KERNEL);
	if (!s2mpw01)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	s2mpw01->rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!s2mpw01->rdev)
		return -ENOMEM;

	rdev = s2mpw01->rdev;
	s2mpw01->dev = &pdev->dev;
	s2mpw01->iodev = iodev;
	s2mpw01->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, s2mpw01);

	s2mpw01->ramp_delay1234 = pdata->buck1234_ramp_delay;
	s2mpw01->opmode_data = pdata->opmode_data;

	for (i = 0; i < pdata->num_regulators; i++) {
		const struct s2mpw01_voltage_desc *desc;
		int id = pdata->regulators[i].id;

		desc = reg_voltage_map[id];
		if (desc)
			regulators[id].n_voltages =
				(desc->max - desc->min) / desc->step + 1;

		rdev[i] = regulator_register(&regulators[id], s2mpw01->dev,
				pdata->regulators[i].initdata, s2mpw01, NULL);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(s2mpw01->dev, "regulator init failed for %d\n",
					id);
			rdev[i] = NULL;
			goto err;
		}
	}

#ifdef CONFIG_DEBUG_FS
		s2mpw01->reg_debugfs_dir =
			debugfs_create_dir("s2mpw01_debug", NULL);
		if (s2mpw01->reg_debugfs_dir) {
			if (!debugfs_create_file("s2mpw01_regs", 0644,
				s2mpw01->reg_debugfs_dir,
				s2mpw01, &s2mpw01_debugfs_fops))
				pr_err("%s : debugfs_create_file, error\n", __func__);
		} else
			pr_err("%s : debugfs_create_dir, error\n", __func__);
#endif

	return 0;
err:
	for (i = 0; i < s2mpw01->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	return ret;
}

static int __devexit s2mpw01_pmic_remove(struct platform_device *pdev)
{
	struct s2mpw01_info *s2mpw01 = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = s2mpw01->rdev;
	int i;

	for (i = 0; i < s2mpw01->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	return 0;
}

static const struct platform_device_id s2mpw01_pmic_id[] = {
	{ "s2mpw01-pmic", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mpw01_pmic_id);

static struct platform_driver s2mpw01_pmic_driver = {
	.driver = {
		.name = "s2mpw01-pmic",
		.owner = THIS_MODULE,
	},
	.probe = s2mpw01_pmic_probe,
	.remove = __devexit_p(s2mpw01_pmic_remove),
	.id_table = s2mpw01_pmic_id,
};

static int __init s2mpw01_pmic_init(void)
{
	return platform_driver_register(&s2mpw01_pmic_driver);
}
subsys_initcall(s2mpw01_pmic_init);

static void __exit s2mpw01_pmic_exit(void)
{
	platform_driver_unregister(&s2mpw01_pmic_driver);
}
module_exit(s2mpw01_pmic_exit);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_AUTHOR("Dongsu Ha <dsfine.ha@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG S2MPW01 Regulator Driver");
MODULE_LICENSE("GPL");
