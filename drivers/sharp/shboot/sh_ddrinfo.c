/* drivers/sharp/shboot/sh_ddrinfo.c
 *
 * Copyright (C) 2013 Sharp Corporation
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/io.h>

#ifdef CONFIG_ANDROID_ENGINEERING
#define EBICS0_MR4_CMD	0x00A80014
#define EBICS0_MR4_RES	0x00A80020

static unsigned int sdram_temp;

static unsigned int get_sdram_temp(void)
{
	unsigned int stat;
	unsigned int count = 0;
	unsigned int ret = 0;
	void __iomem *regadr = NULL;

	regadr = ioremap_nocache(EBICS0_MR4_CMD, 4);
	iowrite32(0x04060000, regadr);

	while (1) {
		udelay(1);
		stat = ioread32(regadr);
		pr_debug("%s() status %d\n", __func__, stat);
		if (!(stat & BIT(17)))
			break;
		if (count++ > 1000) {
			pr_err("%s() timeout 1 sec\n", __func__);
			goto sdram_error;
		}
	}

	if (regadr != NULL)
		iounmap(regadr);

	regadr = ioremap_nocache(EBICS0_MR4_RES, 4);
	ret = ioread32(regadr);
	if (regadr != NULL)
		iounmap(regadr);
sdram_error:
	pr_debug("%s() return %d\n", __func__, ret);
	return ret;
}

static int get_sdram_temp_to_user(char *buf, const struct kernel_param *kp)
{
	sdram_temp = get_sdram_temp();
	return param_get_uint(buf, kp);
}

static struct kernel_param_ops param_ops_sdram_temp = {
	.get = get_sdram_temp_to_user,
};

module_param_cb(ramtemp, &param_ops_sdram_temp, &sdram_temp, S_IRUGO);

#endif /* CONFIG_ANDROID_ENGINEERING */

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sharp Corporation");
MODULE_DESCRIPTION("DDR information driver");
