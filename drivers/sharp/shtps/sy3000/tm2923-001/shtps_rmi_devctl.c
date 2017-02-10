/* drivers/sharp/shtps/sy3000/tm2923-001/shtps_rmi_spi.c
 *
 * Copyright (c) 2013, Sharp. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <sharp/shtps_dev.h>
#include "shtps_rmi_devctl.h"

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_PWR_PMIC_PORT_NAME				"8941_l22"

#define SHTPS_HWRESET_TIME_US					1
#define SHTPS_HWRESET_AFTER_TIME_MS				1
#define SHTPS_HWRESET_WAIT_MS					290

/* -----------------------------------------------------------------------------------
 */
static int msm_shtps_gpio_setup(int irq, int rst)
{
    int rc = 0;

#if 0
    rc = gpio_request(irq, "shtps_irq");
    if (rc) {
        pr_err("%s() request gpio failed (irq)\n", __func__);
        return rc;
    }
#endif

    rc = gpio_request(rst, "shtps_rst");
    if (rc) {
        pr_err("%s() request gpio failed (rst)\n", __func__);
        return rc;
    }

    return 0;
}

static void msm_shtps_gpio_teardown(int irq, int rst)
{
#if 0
    gpio_free(irq);
#endif
    gpio_free(rst);
}

/* -----------------------------------------------------------------------------------
 */
int shtps_device_setup(int irq, int rst)
{
	return msm_shtps_gpio_setup(irq, rst);
}
EXPORT_SYMBOL(shtps_device_setup);

void shtps_device_teardown(int irq, int rst)
{
	msm_shtps_gpio_teardown(irq, rst);
}
EXPORT_SYMBOL(shtps_device_teardown);

void shtps_device_reset(int rst)
{
	gpio_set_value(rst, 0);
	udelay(SHTPS_HWRESET_TIME_US);

	gpio_set_value(rst, 1);
	mdelay(SHTPS_HWRESET_AFTER_TIME_MS);
}
EXPORT_SYMBOL(shtps_device_reset);

void shtps_device_sleep(struct device* dev)
{
	int ret = 0;
	int enabled = 0;
	struct regulator *reg;

	reg = regulator_get(dev, SHTPS_PWR_PMIC_PORT_NAME);
	if (IS_ERR(reg)) {
		pr_err("Unable to get %s regulator\n", SHTPS_PWR_PMIC_PORT_NAME);
		return;
	}

	enabled = regulator_is_enabled(reg);

	if (enabled){
		ret = regulator_set_mode(reg, REGULATOR_MODE_IDLE);
	}else{
		WARN_ON(!enabled);
	}

	if(ret != 0) {
		pr_err("regulator_set_mode fail, ret=%d, mode=%d\n", ret, REGULATOR_MODE_IDLE);
	}

	regulator_put(reg);
}
EXPORT_SYMBOL(shtps_device_sleep);

void shtps_device_wakeup(struct device* dev)
{
	int ret = 0;
	int enabled = 0;
	struct regulator *reg;

	reg = regulator_get(dev, SHTPS_PWR_PMIC_PORT_NAME);
	if (IS_ERR(reg)) {
		pr_err("Unable to get %s regulator\n", SHTPS_PWR_PMIC_PORT_NAME);
		return;
	}

	enabled = regulator_is_enabled(reg);

	if (enabled){
		ret = regulator_set_mode(reg, REGULATOR_MODE_NORMAL);
	}else{
		WARN_ON(!enabled);
	}

	if(ret != 0) {
		pr_err("regulator_set_mode fail, ret=%d, mode=%d\n", ret, REGULATOR_MODE_NORMAL);
	}

	regulator_put(reg);
}
EXPORT_SYMBOL(shtps_device_wakeup);

MODULE_DESCRIPTION("SHARP TOUCHPANEL DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
