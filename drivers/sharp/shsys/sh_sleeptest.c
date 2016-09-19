/* drivers/sharp/shsys/sh_sleeptest.c
 *
 * Copyright (C) 2012 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <sharp/sh_sleeptest.h>

static int sleep_test_mode = 0;

int sleep_test_is_enabled(void)
{
	return sleep_test_mode;
}

static ssize_t show_sleep_test_mode(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	int len;

	len = scnprintf(buf, PAGE_SIZE, "%d\n", sleep_test_mode);

	return len;
}

static ssize_t store_sleep_test_mode(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	long new_mode;

	new_mode = simple_strtol(buf, NULL, 16);
	sleep_test_mode = (new_mode) ? 1 : 0;

	return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_sleep_test_mode, store_sleep_test_mode);

static struct attribute *sh_sleeptest_attrs[] = {
	&dev_attr_mode.attr,
	NULL,
};

static struct attribute_group sh_sleeptest_attr_group = {
	.attrs = sh_sleeptest_attrs,
};

static int sh_sleeptest_probe(struct platform_device *pdev)
{
	int err;

	err = sysfs_create_group(&pdev->dev.kobj, &sh_sleeptest_attr_group);
	if (err) {
		return err;
	}

	return 0;
}

static int sh_sleeptest_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &sh_sleeptest_attr_group);

	return 0;
}

static struct platform_device sh_sleeptest_device = {
	.name = "sh_sleeptest",
	.id   = -1,
};

static struct platform_driver sh_sleeptest_driver = {
	.probe	      = sh_sleeptest_probe,
	.remove	      = sh_sleeptest_remove,
	.driver	      = {
		.name = "sh_sleeptest",
	},
};

static int __init sh_sleeptest_init(void)
{
	platform_device_register(&sh_sleeptest_device);
	platform_driver_register(&sh_sleeptest_driver);

	return 0;
}

static void __exit sh_sleeptest_exit(void)
{
	platform_driver_unregister(&sh_sleeptest_driver);
	platform_device_unregister(&sh_sleeptest_device);
}

module_init(sh_sleeptest_init);
module_exit(sh_sleeptest_exit);

EXPORT_SYMBOL(sleep_test_is_enabled);

MODULE_DESCRIPTION("SHARP SLEEPTEST DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
