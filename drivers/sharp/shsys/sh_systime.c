/* drivers/sharp/shsys/sh_systime.c
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

#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/io.h>
#include <asm/io.h>

#include <sharp/sh_systime.h>

static int timestamp_point = 0;

static int sh_systime_set_timestamp(const char *val, struct kernel_param *kp)
{
    int ret;
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    unsigned long long systime = 0;
    int i;

    ret = param_set_int(val, kp);
    if (ret)
        return ret;

    ret = sh_systime_read_current(&systime);
    if (ret)
        return ret;

    p_sh_smem_common_type = sh_smem_get_common_address();
    if (p_sh_smem_common_type != NULL) {
        if ((timestamp_point == SHSYS_TIMEMSTAMP_SHUTDOWN_START) || (timestamp_point == SHSYS_TIMEMSTAMP_HOTBOOT_START)
            || ((SHSYS_TIMEMSTAMP_FREE <= timestamp_point) && (timestamp_point < SHSYS_TIMEMSTAMP_MAX_NUM))) {
            p_sh_smem_common_type->shsys_timestamp[timestamp_point] = systime;
        }
        else if (timestamp_point == SHSYS_TIMEMSTAMP_KEYGUARD_START) {
            if (p_sh_smem_common_type->shsys_timestamp[timestamp_point] == 0) {
                p_sh_smem_common_type->shsys_timestamp[timestamp_point] = systime;
            }
        }
        else {
            printk("=== Current shsys_timestamp list (MAX_NUM %d) ===\n", SHSYS_TIMEMSTAMP_MAX_NUM);
            for (i = 0; i < SHSYS_TIMEMSTAMP_MAX_NUM; i++) {
                printk("shsys_timestamp[%d] = %llu[us]\n", i, p_sh_smem_common_type->shsys_timestamp[i]);
            }
        }
    }

    return 0;
}

module_param_call(timestamp_point, sh_systime_set_timestamp, param_get_int, &timestamp_point, 0664);

void sh_systime_log_shutdown_complete_time(void)
{
    int ret;
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    unsigned long long systime = 0;

    p_sh_smem_common_type = sh_smem_get_common_address();
    if (p_sh_smem_common_type != NULL) {
        if (p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_SHUTDOWN_START] == 0) {
            return;
        }

        ret = sh_systime_read_current(&systime);
        if (ret != 0) {
            return;
        }

        if (systime >= p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_SHUTDOWN_START]) {
            systime = systime - p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_SHUTDOWN_START];
        }
        else {
            systime = systime + (0xFFFFFFFFFFFFFFFF - p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_SHUTDOWN_START]);
        }

        p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_SHUTDOWN_START] = 0;

        printk("shutdown complete time %llu[us]\n", systime);
    }
}
EXPORT_SYMBOL(sh_systime_log_shutdown_complete_time);

static int sh_systime_open(struct inode *inode, struct file *filp)
{
    return 0;
}

int sh_systime_read_current(unsigned long long *systime)
{
    void __iomem *regadr = NULL;
    unsigned int curr_timetick, last_timetick;

    regadr = ioremap_nocache(TIMETICK_CLK_OFFSET, 4);
    if (regadr != NULL) {
        curr_timetick = ioread32(regadr);
    }
    else {
        return -EFAULT;
    }

    /* Keep grabbing the time until a stable count is given */
    do {
        last_timetick = curr_timetick;
        curr_timetick = ioread32(regadr);
    } while (curr_timetick != last_timetick);

    iounmap(regadr);

    *systime = curr_timetick;
    /* The following calculation is more exact than  "CALCULATE_TIMESTAMP" */
    *systime *= 30517;
    do_div(*systime, 1000);

    return 0;
}
EXPORT_SYMBOL(sh_systime_read_current);

static int sh_systime_read_timestamp(unsigned long long *timestamp)
{
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    unsigned long long systime = 0;
    int ret;

    p_sh_smem_common_type = sh_smem_get_common_address();
    if (p_sh_smem_common_type != NULL) {
        if (p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_BOOT_END] == 0) {
            ret = sh_systime_read_current(&systime);
            if (ret == 0) {
                p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_BOOT_END] = systime;
            }
        }
        if (p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_START] != 0) {
            ret = sh_systime_read_current(&systime);
            if (ret == 0) {
                if (systime >= p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_START]) {
                    systime = systime - p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_START];
                }
                else {
                    systime = systime + (0xFFFFFFFFFFFFFFFF - p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_START]);
                }
                p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_COMP] = systime;
                p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_HOTBOOT_START] = 0;
            }
        }
        memcpy(timestamp, p_sh_smem_common_type->shsys_timestamp, sizeof(p_sh_smem_common_type->shsys_timestamp));
    }
    else {
        return -EFAULT;
    }

    return 0;
}

static ssize_t sh_systime_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    unsigned long long systime;
    unsigned long long timestamp[32];
    int ret;

    if (count == sizeof(systime)) {
        ret = sh_systime_read_current(&systime);
        if (ret != 0) {
            return ret;
        }
        if (copy_to_user(buf, (void *)&systime, count)) {
            return -EFAULT;
        }
    }
    else if (count == sizeof(timestamp)) {
        ret = sh_systime_read_timestamp(timestamp);
        if (ret != 0) {
            return ret;
        }
        if (copy_to_user(buf, (void *)timestamp, count)) {
            return -EFAULT;
        }
    }
    else {
        return -EFAULT;
    }

    return count;
}

static int sh_systime_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations sh_systime_fops = {
    .owner          = THIS_MODULE,
    .open           = sh_systime_open,
    .read           = sh_systime_read,
    .release        = sh_systime_release,
};

static struct miscdevice sh_systime_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sh_systime",
    .fops = &sh_systime_fops,
};

static int __init sh_systime_init( void )
{
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    unsigned long long systime = 0;
    unsigned long long t = 0;
    int ret;

    p_sh_smem_common_type = sh_smem_get_common_address();
    if (p_sh_smem_common_type != NULL) {
        ret = sh_systime_read_current(&systime);
        t = cpu_clock(smp_processor_id());
        if ((ret == 0) && (t != 0)) {
            do_div(t, 1000);
            p_sh_smem_common_type->shsys_timestamp[SHSYS_TIMEMSTAMP_KERNEL_START] = systime - t;
        }
        else {
            printk("sh_systime_init: sh_systime_read_current() or cpu_clock() failed\n");
        }
    }
    else {
        printk("sh_systime_init: sh_smem_get_common_address() failed\n");
    }

    ret = misc_register(&sh_systime_dev);
    if (ret != 0) {
        printk("sh_systime_init: fail to misc_register ret %d\n", ret);
    }

    return ret;
}

module_init(sh_systime_init);

MODULE_DESCRIPTION("sh_systime");
MODULE_LICENSE("GPL v2");

