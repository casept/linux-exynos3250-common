#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#define GPIO_GPS_PWR_EN		EXYNOS3_GPX1(6)

static struct device *gps_dev;
#ifndef CONFIG_SEC_SYSFS
extern struct class *sec_class;
#endif

static struct platform_device bcm4774 = {
    .name = "bcm4774",
    .id = -1,
};

#ifdef CONFIG_OF
static ssize_t show_gps_pwr_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;

	ret = gpio_get_value(GPIO_GPS_PWR_EN);

	pr_info("%s:%d GPIO_GPS_PWR_EN is %d\n", __func__, __LINE__, ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t set_gps_pwr_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int enable;

	sscanf(buf, "%u", &enable);
	//if (kstrtoll(buf, 10, &enable) < 0)
	//	return -EINVAL;

	pr_info("%s:%d endable:%d \n", __func__, __LINE__, enable);
	if (enable)
		gpio_direction_output(GPIO_GPS_PWR_EN, 1);
	else
		gpio_direction_output(GPIO_GPS_PWR_EN, 0);

	return size;
}

static DEVICE_ATTR(gps_pwr_en, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gps_pwr_en, set_gps_pwr_en);
#endif

static int __init gps_bcm4774_init(void)
{
    int ret = 0;

    pr_info("%s\n", __func__);

    platform_device_register(&bcm4774);

#ifdef CONFIG_SEC_SYSFS
	gps_dev = sec_device_create(NULL, "gps");
#else
    BUG_ON(!sec_class);
    gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
    BUG_ON(!gps_dev);
#endif

	ret = gpio_request(GPIO_GPS_PWR_EN, "GPS_PWR_EN");
	if (ret) {
		pr_err("%s, fail to request gpio(GPS_PWR_EN)\n", __func__);
		goto err_find_node;
	}

    gpio_direction_output(GPIO_GPS_PWR_EN, 0);

#ifdef CONFIG_OF
	device_create_file(gps_dev, &dev_attr_gps_pwr_en);
#else
    gpio_export(GPIO_GPS_PWR_EN , 1);
    gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN);
#endif

    return 0;

err_find_node:
#ifdef CONFIG_SEC_SYSFS
	sec_device_destroy(gps_dev->devt);
#else
    device_destroy(sec_class, gps_dev->devt);
#endif
    return ret;
}

device_initcall(gps_bcm4774_init);
