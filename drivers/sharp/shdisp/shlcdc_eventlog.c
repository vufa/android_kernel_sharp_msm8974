/* drivers/sharp/shdisp/shlcdc_eventlog.c  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <sharp/shlcdc_eventlog.h>

/* ------------------------------------------------------------------------- */
/* MACRAOS                                                                   */
/* ------------------------------------------------------------------------- */

#define SHLCDC_EVENTLOG_ALL 512
#define SHLCDC_EVENTLOG_NUM 368
#define SHLCDC_BUSYLOG_NUM  128
#define SHLCDC_ERRLOG_NUM   16

#ifndef SH_BUILD_ID
#define SH_BUILD_ID "S0000"
#endif

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

struct st_shlcdc_eventlog {
    unsigned long sec;
    unsigned long usec;
    unsigned long event;
    unsigned long param;
};

static struct st_shlcdc_eventlog st_shlcdc_logbuffer[SHLCDC_EVENTLOG_ALL];
struct st_shlcdc_eventlog *shlcdc_eventlog=st_shlcdc_logbuffer;
struct st_shlcdc_eventlog *shlcdc_busylog=st_shlcdc_logbuffer+SHLCDC_EVENTLOG_NUM;
struct st_shlcdc_eventlog *shlcdc_errlog=st_shlcdc_logbuffer+SHLCDC_EVENTLOG_NUM+SHLCDC_BUSYLOG_NUM;
unsigned long shlcdc_eventlog_cnt=0;
unsigned long shlcdc_busylog_cnt=0;
unsigned long shlcdc_errlog_cnt=0;
unsigned long shlcdc_eventlog_size=sizeof(struct st_shlcdc_eventlog)*SHLCDC_EVENTLOG_NUM;
unsigned long shlcdc_busylog_size=sizeof(struct st_shlcdc_eventlog)*SHLCDC_BUSYLOG_NUM;
unsigned long shlcdc_errlog_size=sizeof(struct st_shlcdc_eventlog)*SHLCDC_ERRLOG_NUM;
static unsigned long shlcdc_eventlog_max=SHLCDC_EVENTLOG_NUM;
static unsigned long shlcdc_busylog_max=SHLCDC_BUSYLOG_NUM;
static unsigned long shlcdc_errlog_max=SHLCDC_ERRLOG_NUM;

#ifdef CONFIG_ANDROID_ENGINEERING
static unsigned long shlcdc_eventlog_num=SHLCDC_EVENTLOG_NUM;
static unsigned long shlcdc_busylog_num=SHLCDC_BUSYLOG_NUM;
static unsigned long shlcdc_errlog_num=SHLCDC_ERRLOG_NUM;
module_param(shlcdc_eventlog_cnt, ulong, 0400);
module_param(shlcdc_busylog_cnt, ulong, 0400);
module_param(shlcdc_errlog_cnt, ulong, 0400);
module_param(shlcdc_eventlog_num, ulong, 0600);
module_param(shlcdc_busylog_num, ulong, 0600);
module_param(shlcdc_errlog_num, ulong, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static void shlcdc_eventlog_initialize(void);

#ifdef CONFIG_ANDROID_ENGINEERING
static int shlcdc_eventlog_view_set(const char *str, struct kernel_param *kp);
static int shlcdc_eventlog_view_get(char *str, struct kernel_param *kp);
static int shlcdc_logreset_set(const char *str, struct kernel_param *kp);
static int shlcdc_logreset_get(char *str, struct kernel_param *kp);
#endif /* CONFIG_ANDROID_ENGINEERING */

static int shlcdc_eventlog_userlog_rec(struct stEventlog_st *data);
static int shlcdc_eventlog_open(struct inode *inode, struct file *file);
static int shlcdc_eventlog_release(struct inode *inode, struct file *file);
static long shlcdc_eventlog_ioctl(struct file *file, unsigned int cmd, unsigned long arg);


static struct file_operations shlcdc_eventlog_fops = {
  .owner = THIS_MODULE,
  .open = shlcdc_eventlog_open,
  .release = shlcdc_eventlog_release,
  .unlocked_ioctl = shlcdc_eventlog_ioctl,
};

static struct miscdevice shlcdc_eventlog_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME_EVENTLOG,
    .fops  = &shlcdc_eventlog_fops,
};

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_rec                                                       */
/* ------------------------------------------------------------------------- */

void shlcdc_eventlog_rec(unsigned long event, unsigned long param)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;

    if(strncmp(SH_BUILD_ID, "S", 1) == 0)
        return;

    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = shlcdc_eventlog_cnt%shlcdc_eventlog_max;
    shlcdc_eventlog_cnt ++;
    shlcdc_eventlog[idx].sec = (unsigned long)t;
    shlcdc_eventlog[idx].usec = (unsigned long)(nanosec_rem / 1000);
    shlcdc_eventlog[idx].event = event;
    shlcdc_eventlog[idx].param = param;
}
EXPORT_SYMBOL(shlcdc_eventlog_rec);


/* ------------------------------------------------------------------------- */
/* shlcdc_busylog_rec                                                        */
/* ------------------------------------------------------------------------- */

void shlcdc_busylog_rec(unsigned long event, unsigned long param)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;

    if(strncmp(SH_BUILD_ID, "S", 1) == 0)
        return;

    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = shlcdc_busylog_cnt%shlcdc_busylog_max;
    shlcdc_busylog_cnt ++;
    shlcdc_busylog[idx].sec = (unsigned long)t;
    shlcdc_busylog[idx].usec = (unsigned long)(nanosec_rem / 1000);
    shlcdc_busylog[idx].event = event;
    shlcdc_busylog[idx].param = param;
}
EXPORT_SYMBOL(shlcdc_busylog_rec);


/* ------------------------------------------------------------------------- */
/* shlcdc_errlog_rec                                                         */
/* ------------------------------------------------------------------------- */

void shlcdc_errlog_rec(unsigned long event, unsigned long param)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;

    if(strncmp(SH_BUILD_ID, "S", 1) == 0)
        return;

    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = shlcdc_errlog_cnt;
    shlcdc_errlog_cnt ++;
    if(shlcdc_errlog_cnt > shlcdc_errlog_max) return;
    shlcdc_errlog[idx].sec = (unsigned long)t;
    shlcdc_errlog[idx].usec = (unsigned long)(nanosec_rem / 1000);
    shlcdc_errlog[idx].event = event;
    shlcdc_errlog[idx].param = param;
}
EXPORT_SYMBOL(shlcdc_errlog_rec);


/* ------------------------------------------------------------------------- */
/* INITIALIZE                                                                */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_initialize                                                */
/* ------------------------------------------------------------------------- */

static void shlcdc_eventlog_initialize(void)
{
    shlcdc_eventlog_cnt=0;
    shlcdc_busylog_cnt=0;
    shlcdc_errlog_cnt=0;
    memset(st_shlcdc_logbuffer, 0x00, sizeof(st_shlcdc_logbuffer));

    shlcdc_eventlog=st_shlcdc_logbuffer;
    shlcdc_busylog=st_shlcdc_logbuffer+shlcdc_eventlog_max;
    shlcdc_errlog=st_shlcdc_logbuffer+shlcdc_eventlog_max+shlcdc_busylog_max;

    shlcdc_eventlog_size=sizeof(struct st_shlcdc_eventlog)*shlcdc_eventlog_max;
    shlcdc_busylog_size=sizeof(struct st_shlcdc_eventlog)*shlcdc_busylog_max;
    shlcdc_errlog_size=sizeof(struct st_shlcdc_eventlog)*shlcdc_errlog_max;
}


#ifdef CONFIG_ANDROID_ENGINEERING
/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_view_set                                                  */
/* ------------------------------------------------------------------------- */

static unsigned long shlcdc_eventlog_view=0;
static int shlcdc_eventlog_view_set(const char *str, struct kernel_param *kp)
{
    int lp;
    pr_info("[SHLCDC_EVENTLOG(%s:cnt=%lu)]\n", SH_BUILD_ID, shlcdc_eventlog_cnt);
    for(lp=0 ; lp<shlcdc_eventlog_cnt && lp<shlcdc_eventlog_max ; lp++)
        pr_info("[%5lu.%06lu] 0x%08lx 0x%08lx\n", shlcdc_eventlog[lp].sec, shlcdc_eventlog[lp].usec, shlcdc_eventlog[lp].event, shlcdc_eventlog[lp].param);

    pr_info("[SHLCDC_BUSYLOG(%s:cnt=%lu)]\n", SH_BUILD_ID, shlcdc_busylog_cnt);
    for(lp=0 ; lp<shlcdc_busylog_cnt && lp<shlcdc_busylog_max ; lp++)
        pr_info("[%5lu.%06lu] 0x%08lx 0x%08lx\n", shlcdc_busylog[lp].sec, shlcdc_busylog[lp].usec, shlcdc_busylog[lp].event, shlcdc_busylog[lp].param);

    pr_info("[SHLCDC_ERRLOG(%s:cnt=%lu)]\n", SH_BUILD_ID, shlcdc_errlog_cnt);
    for(lp=0 ; lp<shlcdc_errlog_cnt && lp<shlcdc_errlog_max ; lp++)
        pr_info("[%5lu.%06lu] 0x%08lx 0x%08lx\n", shlcdc_errlog[lp].sec, shlcdc_errlog[lp].usec, shlcdc_errlog[lp].event, shlcdc_errlog[lp].param);

    return 0;
}


/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_view_get                                                  */
/* ------------------------------------------------------------------------- */

static int shlcdc_eventlog_view_get(char *str, struct kernel_param *kp)
{
    int ret;
    ret = sprintf(str, "%lu", shlcdc_eventlog_view);
    return ret;
}

module_param_call(shlcdc_eventlog_view, shlcdc_eventlog_view_set, shlcdc_eventlog_view_get, NULL, 0600);


/* ------------------------------------------------------------------------- */
/* shlcdc_logreset_set                                                       */
/* ------------------------------------------------------------------------- */

static unsigned long shlcdc_logreset=0;
static int shlcdc_logreset_set(const char *str, struct kernel_param *kp)
{
    if(shlcdc_eventlog_num < 1) shlcdc_eventlog_num=1;
    if(shlcdc_eventlog_num > SHLCDC_EVENTLOG_ALL-2) shlcdc_eventlog_num=SHLCDC_EVENTLOG_ALL-2;
    if(shlcdc_busylog_num < 1) shlcdc_busylog_num=1;
    if(shlcdc_busylog_num > SHLCDC_EVENTLOG_ALL-2) shlcdc_busylog_num=SHLCDC_EVENTLOG_ALL-2;
    if(shlcdc_errlog_num < 1) shlcdc_errlog_num=1;
    if(shlcdc_errlog_num > SHLCDC_EVENTLOG_ALL-2) shlcdc_errlog_num=SHLCDC_EVENTLOG_ALL-2;

    if(shlcdc_eventlog_num + shlcdc_busylog_num + shlcdc_errlog_num > SHLCDC_EVENTLOG_ALL){
        shlcdc_eventlog_num = SHLCDC_EVENTLOG_NUM;
        shlcdc_busylog_num = SHLCDC_BUSYLOG_NUM;
        shlcdc_errlog_num = SHLCDC_ERRLOG_NUM;
    }
    shlcdc_eventlog_max = shlcdc_eventlog_num;
    shlcdc_busylog_max = shlcdc_busylog_num;
    shlcdc_errlog_max = shlcdc_errlog_num;

    shlcdc_eventlog_initialize();
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shlcdc_logreset_get                                                       */
/* ------------------------------------------------------------------------- */

static int shlcdc_logreset_get(char *str, struct kernel_param *kp)
{
    int ret;
    ret = sprintf(str, "%lu %lu %lu", shlcdc_eventlog_max, shlcdc_busylog_max, shlcdc_errlog_max);
    shlcdc_logreset = ret;
    return ret;
}

module_param_call(shlcdc_logreset, shlcdc_logreset_set, shlcdc_logreset_get, NULL, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */


/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_userlog_rec                                               */
/* ------------------------------------------------------------------------- */

static int shlcdc_eventlog_userlog_rec(struct stEventlog_st *data)
{
    if(data->eventID < SHLCDC_EVENTLOG_USEREVENT){
        data->eventID |= SHLCDC_EVENTLOG_USEREVENT;
    }

    if(data->kind == EEVENTLOG_USER_EVENT){
        shlcdc_eventlog_rec(data->eventID, data->param);
    }
    else if(data->kind == EEVENTLOG_USER_BUSY){
        shlcdc_busylog_rec(data->eventID, data->param);
    }
    else if(data->kind == EEVENTLOG_USER_ERR){
        shlcdc_errlog_rec(data->eventID, data->param);
    }

    return 0;
}


/* ------------------------------------------------------------------------- */
/* FOPS                                                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_open                                                      */
/* ------------------------------------------------------------------------- */

static int shlcdc_eventlog_open(struct inode *inode, struct file *file)
{
    
    return(0);
    
}


/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_release                                                   */
/* ------------------------------------------------------------------------- */

static int shlcdc_eventlog_release(struct inode *inode, struct file *file)
{
    
    return(0);
    
}


/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_ioctl                                                     */
/* ------------------------------------------------------------------------- */

static long shlcdc_eventlog_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret;
    struct stEventlog_st data;
    void __user *argp = (void __user *)arg;

    if(strncmp(SH_BUILD_ID, "S", 1) == 0)
        return 0;

    switch(cmd) {
    case SHLCDC_EVENTLOG_USER :
        if(copy_from_user(&data, argp, sizeof(data))) {
            return(-EFAULT);
        }
        ret = shlcdc_eventlog_userlog_rec(&data);
        break;
    default :
        ret = 0;
        break;
    }
    return(ret);
}


/* ------------------------------------------------------------------------- */
/* shlcdc_eventlog_init                                                      */
/* ------------------------------------------------------------------------- */

static int __init shlcdc_eventlog_init(void)
{
    int ret;
    ret = misc_register(&shlcdc_eventlog_device);
    if(ret < 0) {
        pr_err("%s:misc_register() failed:ret=%d.\n", __func__, ret);
        return(ret);
    }

    if(shlcdc_eventlog_cnt == 0 && shlcdc_busylog_cnt == 0 && shlcdc_errlog_cnt == 0)
        shlcdc_eventlog_initialize();

    return 0;

}
module_init(shlcdc_eventlog_init);


MODULE_DESCRIPTION("SHARP LCD EVENTLOG DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("2.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
