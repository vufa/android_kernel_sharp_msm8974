/* drivers/sharp/shsys/sh_regaxs.c
 *
 * Copyright (C) 2012 Sharp Corporation
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
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/io.h>
#include <asm/io.h>

#include <sharp/sh_regaxs.h>

static int sh_regaxs_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int sh_regaxs_ioctl_read(unsigned long arg)
{
    int rc = 0;
    struct sh_regaxs_read_write regaxs_req;
    void __iomem *regadr = NULL;

    if (arg == 0) {
        return -EINVAL;
    }

    if (copy_from_user(&regaxs_req, (void __user *)arg, sizeof(regaxs_req)) != 0) {
        return -EINVAL;
    }

    regadr = ioremap_nocache(regaxs_req.physaddr, 4);
    regaxs_req.data = ioread32(regadr);

    if (copy_to_user((u8*)arg, (u8*)&regaxs_req, sizeof(regaxs_req)) != 0) {
        rc = -EFAULT;
    }

    if (regadr != NULL) {
        iounmap(regadr);
    }
    return rc;
}

static int sh_regaxs_ioctl_write(unsigned long arg)
{
    struct sh_regaxs_read_write regaxs_req;
    void __iomem *regadr = NULL;

    if (arg == 0) {
        return -EINVAL;
    }

    if (copy_from_user(&regaxs_req, (void __user *)arg, sizeof(regaxs_req)) != 0) {
        return -EINVAL;
    }

    regadr = ioremap_nocache(regaxs_req.physaddr, 4);
    iowrite32(regaxs_req.data, regadr);
    if (regadr != NULL) {
        iounmap(regadr);
    }

    return 0;
}

static long sh_regaxs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int rc = 0;

    switch (cmd) {
    case SH_REGAXS_READ:
        rc = sh_regaxs_ioctl_read(arg);
        break;
    case SH_REGAXS_WRITE:
        rc = sh_regaxs_ioctl_write(arg);
        break;
    default:
        rc = -EPERM;
        break;
    }
    return rc;
}

static int sh_regaxs_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations sh_regaxs_fops = {
    .owner          = THIS_MODULE,
    .open           = sh_regaxs_open,
    .release        = sh_regaxs_release,
    .unlocked_ioctl = sh_regaxs_ioctl,
};

static struct miscdevice sh_regaxs_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sh_regaxs",
    .fops = &sh_regaxs_fops,
};

static int __init sh_regaxs_init( void )
{
    int ret;

    ret = misc_register(&sh_regaxs_dev);
    if (ret != 0) {
        printk("sh_regaxs_init: fail to misc_register ret %d\n", ret);
    }

    return ret;
}

module_init(sh_regaxs_init);

MODULE_DESCRIPTION("sh_regaxs");
MODULE_LICENSE("GPL v2");

