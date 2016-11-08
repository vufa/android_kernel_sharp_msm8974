/* drivers/sharp/mfc/snfc.c (NFC driver)
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
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

#ifdef CONFIG_SHSNFC

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/qpnp/pin.h>
#include <sharp/snfc_ucc.h>
#include <sharp/felica_cen.h>
#include "mfc.h"

/* RFS */
#define D_RFS_DEVS				(1)
#define D_RFS_DEV_NAME			("snfc_rfs")

/* CEN */
#define D_CEN_DEVS				(1)
#define D_CEN_DEV_NAME			("snfc_cen")

/* INTU */
#define D_INTU_DEV_LOW			(0)
#define D_INTU_DEV_HIGH			(1)

/* INTU_POLL */
#define D_INTU_POLL_DEVS		(1)
#define D_INTU_POLL_DEV_NAME	("snfc_intu_poll")
#define D_INTU_POLL_DELAY_MSEC	(3)

/* UART Collision Control */
#define D_SUCC_DEVS				(1)
#define D_SUCC_DEV_NAME			("snfc_ucc")
#define D_SUCC_OPEN_RETRY_INTERVAL_USEC	10000
#define D_SUCC_OPEN_RETRY_NUM	30
#define D_SUCC_PON_WAIT_USEC	10000

/* NFC Available */
#define D_AVAILABLE_DEVS		(1)
#define D_AVAILABLE_DEV_NAME	("snfc_available_poll")
#define D_AVAILABLE_DELAY_MSEC	(3)

/*
 * prototype
 */
static __init int snfc_init(void);
static void __exit snfc_exit(void);

static int succ_actNop(int flags);
static int succ_actBusy(int flags);
static int succ_actAbnormal(int flags);
static int succ_actOpenNfc(int flags);
static int succ_actCloseNfc(int flags);
static int succ_actOpenFelica(int flags);
static int succ_actOpenFelicaRetry(int flags);
static int succ_actCloseFelica(int flags);

typedef struct {
	int (*execFunc)(int);
	int nextStatus;
} succ_fsm_item;

/*
 * global variable
 */
static struct class *snfc_class = NULL;

static struct cdev rfs_cdev;
static struct cdev cen_cdev;
static struct cdev intu_poll_cdev;
static struct cdev succ_cdev;
static struct cdev available_cdev;

static struct poll_data g_intu_d;
static struct poll_data g_available_d;
static int g_current_state = SUCC_STATE_IDLE;

int g_snfc_hsel_gpio_no = 0;
int g_snfc_intu_gpio_no = 0;
int g_snfc_vfel_gpio_no = 0;
extern unsigned int mfc_rws_sts;

/*
 * function_hsel
 */
int set_hsel_value(int value)
{
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	gpio_set_value(D_HSEL_GPIO_NO, value);
	return 0;
}

/*
 * function_rfs
 */
extern const struct file_operations mfc_rfs_fileops;

static int rfs_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev, 0, D_RFS_DEVS, D_RFS_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&rfs_cdev, &mfc_rfs_fileops);
	rfs_cdev.owner = THIS_MODULE;

	result = cdev_add(&rfs_cdev, dev, D_RFS_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_RFS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(snfc_class, NULL, dev, NULL, D_RFS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_cdev);
		unregister_chrdev_region(dev, D_RFS_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void rfs_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	cdev_del(&rfs_cdev);
	unregister_chrdev_region(dev, D_RFS_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/*
 * function_cen
 */
extern const struct file_operations mfc_cen_fileops;

static int cen_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev, 0, D_CEN_DEVS, D_CEN_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&cen_cdev, &mfc_cen_fileops);
	cen_cdev.owner = THIS_MODULE;

	result = cdev_add(&cen_cdev, dev, D_CEN_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_CEN_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(snfc_class, NULL, dev, NULL, D_CEN_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&cen_cdev);
		unregister_chrdev_region(dev, D_CEN_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void cen_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	cdev_del(&cen_cdev);
	unregister_chrdev_region(dev, D_CEN_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/*
 * function_intu_poll
 */
static void intu_poll_work_func(struct work_struct *work)
{
	int read_value = 0, old_value = 0;
	unsigned long irqflag = 0;

	MFC_DRV_DBG_LOG("START");

	old_value = g_intu_d.device_status;

	read_value = gpio_get_value_cansleep(D_INTU_GPIO_NO);

	MFC_DRV_DBG_LOG("read_value = %d old_value = %d", read_value, old_value);

	/* read error */
	if (read_value < 0) {
		g_intu_d.read_error = read_value;
	/* read changed data */
	} else if (read_value != old_value) {
		g_intu_d.device_status = read_value;
		g_intu_d.read_error = 0;

		/* change irq flag */
		if (g_intu_d.device_status == D_INTU_DEV_LOW)
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		else
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
/* COORDINATOR Qualcomm1021 BUILDERR MODIFY start */
		if (irq_set_irq_type(gpio_to_irq(D_INTU_GPIO_NO), irqflag))
			MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
/* COORDINATOR Qualcomm1021 BUILDERR MODIFY end */
	}

	/* enable irq handler */
	enable_irq(gpio_to_irq(D_INTU_GPIO_NO));

	/* read changed data or error */
	if (read_value != old_value || g_intu_d.read_error) {
		if ((snfc_available()) && (snfc_available_wake_up())) {
			/* wakeup poll and read */
			g_intu_d.irq_handler_done = 1;
			wake_up_interruptible(&g_intu_d.read_wait);
		}
	}

	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d, g_intu_d.read_error = %d"
					, read_value, old_value, g_intu_d.read_error);
}

static irqreturn_t intu_poll_irq_handler(int irq, void *dev_id)
{
	MFC_DRV_DBG_LOG("START irq = %d", irq);

	disable_irq_nosync(gpio_to_irq(D_INTU_GPIO_NO));
	/* set workqueue */
	schedule_delayed_work(&g_intu_d.work, msecs_to_jiffies(D_INTU_POLL_DELAY_MSEC));

	MFC_DRV_DBG_LOG("END");

	return IRQ_HANDLED;
}

static unsigned int intu_poll_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	MFC_DRV_DBG_LOG("START");

	/* wait irq handler */
	poll_wait(filp, &g_intu_d.read_wait, wait);
	if (g_intu_d.irq_handler_done)
		mask = POLLIN | POLLRDNORM;

	MFC_DRV_DBG_LOG("END mask = %d", mask);

	return mask;
}

static ssize_t intu_poll_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char on[2];

	MFC_DRV_DBG_LOG("START");

	/* length check */
	if (len < 1) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}

	if (!g_intu_d.irq_handler_done) {
		if (filp->f_flags & O_NONBLOCK) {
			MFC_DRV_ERR_LOG("NONBLOCK");
			return -EAGAIN;
		}
		/* wait irq handler */
		ret = wait_event_interruptible(g_intu_d.read_wait,
		                               g_intu_d.irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			MFC_DRV_DBG_LOG("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}

	/* read failed */
	if (g_intu_d.read_error) {
		g_intu_d.irq_handler_done = 0;
		g_intu_d.read_error = 0;
		MFC_DRV_ERR_LOG("g_intu_d.read_error = %d", g_intu_d.read_error);
		return -EIO;
	}

	/* set readed data */
	if (g_intu_d.device_status == D_INTU_DEV_HIGH) {
		on[0] = D_INTU_DEV_HIGH;
	} else {
		on[0] = D_INTU_DEV_LOW;
	}
	on[1] = 0x00;

	if (len > 2)
		len = 2;

	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	g_intu_d.irq_handler_done = 0;

	MFC_DRV_DBG_LOG("END len = %d, on[0] = %d", len, on[0]);

	return len;
}

static int intu_poll_open(struct inode *inode, struct file *filp)
{
	unsigned long irqflag = 0;
	int ret = 0;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	/* only one time success */
	if (g_intu_d.open_flag) {
		MFC_DRV_ERR_LOG("only one time");
		return -EBUSY;
	}
	g_intu_d.open_flag = 1;

	/* preparation of anti-chattering */
	ret = gpio_get_value_cansleep(D_INTU_GPIO_NO);
	if (ret < 0) {
		g_intu_d.open_flag = 0;
		MFC_DRV_ERR_LOG("gpio_get_value_cansleep ret = %d", ret);
		return -EIO;
	}
	g_intu_d.device_status = ret;

	/* set irq handler */
	if (g_intu_d.device_status == D_INTU_DEV_LOW) {
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	} else {
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	if (request_irq(gpio_to_irq(D_INTU_GPIO_NO),
					intu_poll_irq_handler,
					irqflag,
					D_INTU_POLL_DEV_NAME,
					(void*)&g_intu_d)) {
		g_intu_d.open_flag = 0;
		MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}

	if (enable_irq_wake(gpio_to_irq(D_INTU_GPIO_NO))) {
		MFC_DRV_ERR_LOG("enable_irq_wake");
		free_irq(gpio_to_irq(D_INTU_GPIO_NO), (void *)&g_intu_d);
		return -EIO;
	}

	g_intu_d.irq_handler_done = 0;

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static int intu_poll_release(struct inode *inode, struct file *filp)
{
	MFC_DRV_DBG_LOG("START");

	/* clear workqueue */
	cancel_delayed_work(&g_intu_d.work);

	if (disable_irq_wake(gpio_to_irq(D_INTU_GPIO_NO)))
		MFC_DRV_ERR_LOG("disable_irq_wake");

	free_irq(gpio_to_irq(D_INTU_GPIO_NO), (void *)&g_intu_d);

	g_intu_d.open_flag = 0;

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static const struct file_operations intu_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = intu_poll_read,
	.open    = intu_poll_open,
	.release = intu_poll_release,
	.poll    = intu_poll_poll,
};

static int intu_poll_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	/* get major number */
	result = alloc_chrdev_region(&dev, 0, D_INTU_POLL_DEVS, D_INTU_POLL_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	/* initialize INTU */
	cdev_init(&intu_poll_cdev, &intu_poll_fileops);
	intu_poll_cdev.owner = THIS_MODULE;

	/* add INTU */
	result = cdev_add(&intu_poll_cdev, dev, D_INTU_POLL_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_INTU_POLL_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	/* create INTU */
	class_dev = device_create(snfc_class, NULL, dev, NULL, D_INTU_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&intu_poll_cdev);
		unregister_chrdev_region(dev, D_INTU_POLL_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	/* initialize poll_data */
	memset(&g_intu_d, 0x00, sizeof(struct poll_data));
	/* initialize workqueue */
	INIT_DELAYED_WORK(&g_intu_d.work, intu_poll_work_func);
	/* initialize waitqueue */
	init_waitqueue_head(&g_intu_d.read_wait);

	g_intu_d.open_flag = 0;

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void intu_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	MFC_DRV_DBG_LOG("START");

	cdev_del(&intu_poll_cdev);
	unregister_chrdev_region(dev, D_INTU_POLL_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/*
 * function_succ
 */
static long succ_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int event, result, flags;
	MFC_DRV_DBG_LOG("START cmd = %u, arg = %lu", cmd, arg);

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	if (copy_from_user(&flags, (int __user *)arg, sizeof(flags))) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}

	switch (cmd) {
	case SHSNFC_UCC_REQ_AUTOPOLL:
		event = SUCC_EVENT_AUTOPOLL;
		break;
	case SHSNFC_UCC_REQ_START_NFC:
		event = SUCC_EVENT_START_NFC;
		break;
	case SHSNFC_UCC_REQ_END_PROC:
		event = SUCC_EVENT_END_NFC;
		break;
	default:
		MFC_DRV_ERR_LOG("cmd unhandled");
		return -EINVAL;
	}

	result = succ_handle_event(event, flags);

	if (copy_to_user((int __user *)arg, &result, sizeof(result))) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static int succ_open(struct inode *inode, struct file *filp)
{
	MFC_DRV_DBG_LOG("");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	return 0;
}

static int succ_release(struct inode *inode, struct file *filp)
{
	MFC_DRV_DBG_LOG("");
	succ_handle_event(SUCC_EVENT_END_NFC, 0);
	return 0;
}

static const struct file_operations succ_fileops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= succ_ioctl,
	.open			= succ_open,
	.release		= succ_release,
};

static int succ_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev, 0, D_SUCC_DEVS, D_SUCC_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&succ_cdev, &succ_fileops);
	succ_cdev.owner = THIS_MODULE;

	result = cdev_add(&succ_cdev, dev, D_SUCC_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_SUCC_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(snfc_class, NULL, dev, NULL, D_SUCC_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&succ_cdev);
		unregister_chrdev_region(dev, D_SUCC_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void succ_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	cdev_del(&succ_cdev);
	unregister_chrdev_region(dev, D_SUCC_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/* UART Collision Control FSM table */
static const succ_fsm_item succ_fsm_table[SUCC_STATE_NUM][SUCC_EVENT_NUM] = {
	{	/* SUCC_STATE_IDLE */
		{ &succ_actOpenNfc, SUCC_STATE_AUTOPOLL }, /* SUCC_EVENT_AUTOPOLL */
		{ &succ_actOpenNfc, SUCC_STATE_NFC }, /* SUCC_EVENT_START_NFC */
		{ &succ_actOpenFelica, SUCC_STATE_FELICA }, /* SUCC_EVENT_START_FELICA */
		{ &succ_actNop, -1 }, /* SUCC_EVENT_END_NFC */
		{ &succ_actNop, -1 }  /* SUCC_EVENT_END_FELICA */
	},
	{	/* SUCC_STATE_AUTOPOLL */
		{ &succ_actAbnormal, -1 }, /* SUCC_EVENT_AUTOPOLL */
		{ &succ_actAbnormal, -1 }, /* SUCC_EVENT_START_NFC */
		{ &succ_actOpenFelicaRetry, SUCC_STATE_NFC }, /* SUCC_EVENT_START_FELICA */
		{ &succ_actCloseNfc, SUCC_STATE_IDLE }, /* SUCC_EVENT_END_NFC */
		{ &succ_actAbnormal, -1 }  /* SUCC_EVENT_END_FELICA */
	},
	{	/* SUCC_STATE_NFC */
		{ &succ_actAbnormal, -1 }, /* SUCC_EVENT_AUTOPOLL */
		{ &succ_actAbnormal, -1 }, /* SUCC_EVENT_START_NFC */
		{ &succ_actBusy, -1 }, /* SUCC_EVENT_START_FELICA */
		{ &succ_actCloseNfc, SUCC_STATE_IDLE }, /* SUCC_EVENT_END_NFC */
		{ &succ_actAbnormal, -1 }  /* SUCC_EVENT_END_FELICA */
	},
	{	/* SUCC_STATE_FELICA */
		{ &succ_actBusy, -1 }, /* SUCC_EVENT_AUTOPOLL */
		{ &succ_actBusy, -1 }, /* SUCC_EVENT_START_NFC */
		{ &succ_actNop, -1 }, /* SUCC_EVENT_START_FELICA */
		{ &succ_actAbnormal, -1 }, /* SUCC_EVENT_END_NFC */
		{ &succ_actCloseFelica, SUCC_STATE_IDLE }  /* SUCC_EVENT_END_FELICA */
	}
};

static const succ_fsm_item* succ_getFsmItem(int state, int event)
{
	if ((state < 0) || (state >= SUCC_STATE_NUM)
	 || (event < 0) || (event >= SUCC_EVENT_NUM)) {
		return NULL;
	}
	return &succ_fsm_table[state][event];
}

int succ_handle_event(int event, int flags)
{
	int result = SUCC_RETVAL_ABNORMAL;
	const succ_fsm_item *item;
	static DEFINE_MUTEX(lock);

	MFC_DRV_DBG_LOG("START event = %d", event);

	mutex_lock(&lock);

	MFC_DRV_DBG_LOG("state = %d", g_current_state);

	item = succ_getFsmItem(g_current_state, event);

	if (item && (item->execFunc != 0)) {
		if (item->nextStatus >= 0) {
			g_current_state = item->nextStatus;
		}

		mutex_unlock(&lock);

		result = item->execFunc(flags);
	} else {
		mutex_unlock(&lock);
	}

	MFC_DRV_DBG_LOG("END state = %d, result = %d", g_current_state, result);
	return result;
}

static int succ_actNop(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	return SUCC_RETVAL_OK;
}

static int succ_actBusy(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	return SUCC_RETVAL_BUSY;
}

static int succ_actAbnormal(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	return SUCC_RETVAL_ABNORMAL;
}

static int succ_actOpenNfc(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");

	if (flags & SHSNFC_UCC_FLAG_RWS_CHECK) {
		if (mfc_rws_sts == D_RWS_RW_DISABLE) {
			MFC_DRV_DBG_LOG("rws disable");
			succ_handle_event(SUCC_EVENT_END_NFC, flags);
			return SUCC_RETVAL_BUSY;
		}
	}

	set_hsel_value(D_HSEL_DEV_HIGH);
	if (set_pon_value(D_PON_DEV_HIGH) != 0) {
		MFC_DRV_ERR_LOG("set pon failed");
		succ_handle_event(SUCC_EVENT_END_NFC, flags);
		return SUCC_RETVAL_BUSY;
	}

	if (flags & SHSNFC_UCC_FLAG_PON_WAIT) {
		usleep(D_SUCC_PON_WAIT_USEC);
	} else if (flags & SHSNFC_UCC_FLAG_PON_WAIT_NO_RFS) {
		if (get_rfs_value() == D_RFS_DEV_HIGH) {
			usleep(D_SUCC_PON_WAIT_USEC);
		}
	} else {
		/* nothing to do */
	}

	return SUCC_RETVAL_OK;
}

static int succ_actCloseNfc(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	set_pon_value(D_PON_DEV_LOW);
	set_hsel_value(D_HSEL_DEV_LOW);
	return SUCC_RETVAL_OK;
}

static int succ_actOpenFelica(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	set_hsel_value(D_HSEL_DEV_LOW);
	notify_nfc_avalable_change(NFC_AVAILABLE_FELICA, AVAILABLE_FALSE);
	return SUCC_RETVAL_OK;
}

static int succ_actOpenFelicaRetry(int flags)
{
	int result, i;
	MFC_DRV_DBG_LOG("START");

	for (i = 0 ; i < D_SUCC_OPEN_RETRY_NUM; i++) {
		usleep(D_SUCC_OPEN_RETRY_INTERVAL_USEC);

		result = succ_handle_event(SUCC_EVENT_START_FELICA, flags);
		MFC_DRV_DBG_LOG("loop count = %d, result = %d", i, result);

		if (result == SUCC_RETVAL_OK) {
			break;
		}
	}

	MFC_DRV_DBG_LOG("END result = %d", result);
	return result;
}

static int succ_actCloseFelica(int flags)
{
	MFC_DRV_DBG_LOG("ENTER");
	notify_nfc_avalable_change(NFC_AVAILABLE_FELICA, AVAILABLE_TRUE);
	return SUCC_RETVAL_OK;
}

/*
 * function_available
 */
static inline int is_nfc_available(void)
{
	return (g_available_d.device_status == (NFC_AVAILABLE_RFS | NFC_AVAILABLE_CEN | NFC_AVAILABLE_FELICA | NFC_AVAILABLE_RWS));
}

void notify_nfc_avalable_change(int type, int status)
{
	int available;

	MFC_DRV_DBG_LOG("START device_status = %d, type = %d", g_available_d.device_status, type);

	switch (type) {
	case NFC_AVAILABLE_RFS:
	case NFC_AVAILABLE_CEN:
	case NFC_AVAILABLE_FELICA:
	case NFC_AVAILABLE_RWS:
		break;
	default:
		return;
	}

	if (status) {
		g_available_d.device_status |= type;
	} else {
		g_available_d.device_status &= ~type;
	}

	available = is_nfc_available();
	if (available) {
		if ((snfc_available()) && (snfc_available_wake_up())) {
			g_available_d.irq_handler_done = 1;
			wake_up_interruptible(&g_available_d.read_wait);
		}
	}

	MFC_DRV_DBG_LOG("END device_status = %d", g_available_d.device_status);
}

static void available_on_rfs_irq(int rfs_status)
{
	MFC_DRV_DBG_LOG("START rfs_status = %d", rfs_status);

	if (rfs_status < 0) {
		MFC_DRV_DBG_LOG("read_error");
		g_available_d.read_error = 1;
		g_available_d.irq_handler_done = 1;
		wake_up_interruptible(&g_available_d.read_wait);
	} else {
		notify_nfc_avalable_change(NFC_AVAILABLE_RFS, rfs_status);
	}

	MFC_DRV_DBG_LOG("END");
}

static unsigned int available_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	int available;

	MFC_DRV_DBG_LOG("START");

	/* wait irq handler */
	poll_wait(filp, &g_available_d.read_wait, wait);
	available = is_nfc_available();
	if (available) {
		mask = POLLIN | POLLRDNORM;
	}

	MFC_DRV_DBG_LOG("END mask = %d", mask);

	return mask;
}

static ssize_t available_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	int ret, available;
	char on[2];

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	/* length check */
	if (len < 1) {
		MFC_DRV_ERR_LOG("len = %d", len);
		return -EIO;
	}

	available = is_nfc_available();
	if (!available) {
		g_available_d.irq_handler_done = 0;

		MFC_DRV_DBG_LOG("start read_wait");

		if (filp->f_flags & O_NONBLOCK) {
			MFC_DRV_ERR_LOG("NONBLOCK");
			return -EAGAIN;
		}

		if (g_available_d.irq_handler_done == 0) {
			/* wait irq handler */
			ret = wait_event_interruptible(g_available_d.read_wait,
											g_available_d.irq_handler_done == 1);
			if (ret == -ERESTARTSYS) {
				MFC_DRV_DBG_LOG("wait_event_interruptible ret = %d", ret);
				return -EINTR;
			}
		}

		/* read failed */
		if (g_available_d.read_error) {
			g_available_d.read_error = 0;
			MFC_DRV_ERR_LOG("read_error = %d", g_available_d.read_error);
			return -EIO;
		}

		available = is_nfc_available();
		MFC_DRV_DBG_LOG("end read_wait available = %d", available);
	}

	/* set readed data */
	if (available) {
		on[0] = 1;
	} else {
		on[0] = 0;
	}
	on[1] = 0;

	if (len > 2)
		len = 2;

	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	MFC_DRV_DBG_LOG("END len = %d, on[0] = %d", len, on[0]);

	return len;
}

static int available_open(struct inode *inode, struct file *filp)
{
	int ret = 0, status = 0;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	/* only one time success */
	if (g_available_d.open_flag) {
		MFC_DRV_ERR_LOG("only one time");
		return -EBUSY;
	}

	ret = get_cen_value();
	if (ret == CEN_LOCK_OFF) {
		status |= NFC_AVAILABLE_CEN;
	} else if (ret == CEN_LOCK_ON) {
		/* nothing to do */
	} else {
		MFC_DRV_ERR_LOG("get_cen_value ret = %d", ret);
		return -EIO;
	}

	if (g_current_state != SUCC_STATE_FELICA) {
		status |= NFC_AVAILABLE_FELICA;
	}

	if (mfc_rws_sts == D_RWS_RW_ENABLE) {
		status |= NFC_AVAILABLE_RWS;
	}

	/* request rfs irq */
	ret = request_notify_rfs_irq(&available_on_rfs_irq);
	if (ret == D_RFS_DEV_HIGH) {
		status |= NFC_AVAILABLE_RFS;
	} else if (ret == D_RFS_DEV_LOW) {
		/* nothing to do */
	} else {
		MFC_DRV_ERR_LOG("request_notify_rfs_irq ret = %d", ret);
		return -EIO;
	}

	g_available_d.device_status = status;
	g_available_d.open_flag = 1;

	MFC_DRV_DBG_LOG("END status = %d", status);

	return 0;
}

static int available_release(struct inode *inode, struct file *filp)
{
	MFC_DRV_DBG_LOG("START");

	/* free rfs irq */
	free_notify_rfs_irq(&available_on_rfs_irq);

	g_available_d.open_flag = 0;

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static const struct file_operations available_fileops = {
	.owner   = THIS_MODULE,
	.read    = available_read,
	.open    = available_open,
	.release = available_release,
	.poll    = available_poll,
};

static int available_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	/* get major number */
	result = alloc_chrdev_region(&dev, 0, D_AVAILABLE_DEVS, D_AVAILABLE_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	/* initialize Available */
	cdev_init(&available_cdev, &available_fileops);
	available_cdev.owner = THIS_MODULE;

	/* add Available */
	result = cdev_add(&available_cdev, dev, D_AVAILABLE_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_AVAILABLE_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	/* create Available */
	class_dev = device_create(snfc_class, NULL, dev, NULL, D_AVAILABLE_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&available_cdev);
		unregister_chrdev_region(dev, D_AVAILABLE_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	/* initialize available_data */
	memset(&g_available_d, 0, sizeof(g_available_d));
	/* initialize waitqueue */
	init_waitqueue_head(&g_available_d.read_wait);

	g_available_d.open_flag = 0;

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void available_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	MFC_DRV_DBG_LOG("START");

	cdev_del(&available_cdev);
	unregister_chrdev_region(dev, D_AVAILABLE_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/*
 * snfc_init
 */
static __init int snfc_init(void)
{
	int ret;

	snfc_class = class_create(THIS_MODULE, "snfc");
	if (IS_ERR(snfc_class)) {
		return PTR_ERR(snfc_class);
	}

	g_snfc_hsel_gpio_no = qpnp_pin_map("pm8941-gpio", 7);
	g_snfc_intu_gpio_no = qpnp_pin_map("pm8941-gpio", 6);
	g_snfc_vfel_gpio_no = qpnp_pin_map("pm8941-mpp", 8);

	ret = rfs_init();
	if (ret < 0) {
		return ret;
	}

	ret = cen_init();
	if (ret < 0) {
		return ret;
	}

	ret = intu_poll_init();
	if (ret < 0) {
		return ret;
	}

	ret = succ_init();
	if (ret < 0) {
		return ret;
	}

	ret = available_init();
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * snfc_exit
 */
static void __exit snfc_exit(void)
{
	class_destroy(snfc_class);

	rfs_exit();

	cen_exit();

	intu_poll_exit();

	succ_exit();

	available_exit();
}

MODULE_LICENSE("GPL v2");

module_init(snfc_init);
module_exit(snfc_exit);

#endif /* CONFIG_SHSNFC */

