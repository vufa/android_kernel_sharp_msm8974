/* drivers/sharp/mfc/felica.c (FeliCa driver)
 *
 * Copyright (C) 2010-2013 SHARP CORPORATION
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

#ifdef CONFIG_SHFELICA

/***************header***************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/current.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>


#include <sharp/felica_cen_diag.h>
#include <sharp/felica_cen.h>
#include <sharp/felica_pon.h>
#include <sharp/felica_poll.h>
#include <sharp/felica_rws.h>


#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>

#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#include "mfc.h"

static struct class *felica_class = NULL;

static struct cdev rfs_cdev;
static struct cdev int_cdev;
static struct cdev pon_cdev;
static struct cdev cen_cdev;
static struct cdev rfs_poll_cdev;
static struct cdev int_poll_cdev;
static struct cdev rws_cdev;

/*
 * Definition
*/
/* RFS */
#define D_RFS_DEVS		(1)
#define D_RFS_DEV_NAME	("felica_rfs")

/* INT */
#define D_INT_DEVS		(1)
#define D_INT_DEV_NAME	("felica_int")

/* PON */
#define D_PON_DEVS		(1)
#define D_PON_DEV_NAME	("felica_pon")
#define D_PON_HIGH_ENABLE	(1)
#define D_PON_HIGH_DISABLE	(0)

static unsigned int pon_high_sts = D_PON_HIGH_ENABLE;

/* CEN */
#define D_CEN_DEVS		(1)
#define D_CEN_DEV_NAME	("felica_cen")

/* RFS_POLL */
#define D_RFS_POLL_DEVS		D_RFS_DEVS
#define D_RFS_POLL_DEV_NAME	("rfs_poll")

#define D_RFS_POLL_SLEEP_NUM	(3)
#define D_RFS_POLL_SLEEP_USEC	(1000)
#define D_RFS_POLL_DELAY_MSEC	(3)
#define D_RFS_IRQ_NAME			("rfs_irq")

/* INT_POLL */
#define D_INT_POLL_DEVS		D_INT_DEVS
#define D_INT_POLL_DEV_NAME	("int_poll")

#define D_INT_POLL_SLEEP_NUM	(3)
#define D_INT_POLL_SLEEP_USEC	(1000)
#define D_INT_POLL_DELAY_MSEC	(3)
#define D_INT_DEV_LOW			(0)
#define D_INT_DEV_HIGH			(1)
#define D_INT_WAKE_LOCK_TIMEOUT	(HZ/2)	/* 500msec */

/* RWS */
#define D_RWS_DEVS				(1)
#define D_RWS_DEV_NAME		("felica_rws")


/* CEN */
static struct i2c_client *this_client;

struct cen_data {
	struct input_dev *input_dev;
};

/* RFS_IRQ */
struct rfs_irq_data {
	int req_count;
	rfs_irq_handler on_irq[D_RFS_IRQ_MAX_NUM];
	struct delayed_work work;
	int rfs_status;
};

static struct rfs_irq_data g_rfs_irq_d;

/* INT */
static struct wake_lock g_int_wake_lock;

/*
 * prototype
*/
static __init int felica_init(void);
static void __exit felica_exit(void);

static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid);

static int felica_ppc_read_E2PROM(unsigned char item_id, unsigned char* ppc_data_ptr);
static int felica_ppc_read_register(unsigned char item_id, unsigned char* ppc_data_ptr);
static int felica_ppc_write_E2PROM(unsigned char item_id, unsigned char set_ppc_data);
static int felica_ppc_write_register(unsigned char item_id, unsigned char set_ppc_data);
static int felica_ppc_reload(void);
static int felica_ppc_reverse_output(void);
static int felica_ppc_write_protect(int write_protect);

static int mfd_LD_ppc_read_status_register_process(unsigned char* ppc_data_ptr);
static int mfd_LD_ppc_read_process(unsigned char cmd, unsigned char* read_data_ptr);
static int mfd_LD_ppc_change_accessmode_process(int accessmode);
static int mfd_LD_ppc_write_process(unsigned char cmd, unsigned char set_data, int wait_flg);
static int mfd_LD_ppc_check_set_data(unsigned char cmd, unsigned char set_data);
static int mfd_LD_ppc_reload_process(void);
static int mfd_LD_ppc_reverse_output_process(void);
static int mfd_LD_ppc_E2PROM_write_protect_process(int write_protect);

static int mfd_PD_ppc_read_status_register(unsigned char* ppc_data_ptr);
static int mfd_PD_ppc_change_accessmode(int accessmode);
static int mfd_PD_ppc_read(unsigned char cmd, unsigned char* read_data_ptr);
static int mfd_PD_ppc_write(unsigned char cmd, unsigned char set_data, int wait_flg);
static int mfd_PD_ppc_E2PROM_write_protect(int write_protect);	
static int mfd_PD_ppc_reload(void);
static int mfd_PD_ppc_reverse_output(void);

static int mfd_PD_ppc_i2c_read(unsigned char cmd, unsigned char* read_data_ptr);
static int mfd_PD_ppc_i2c_write(unsigned char cmd, unsigned char set_data, int wait_flg);
static int mfd_PD_ppc_extract_read_data(unsigned char cmd, unsigned char* read_data_ptr);
static int mfd_PD_ppc_i2c_write_no_data(unsigned char cmd);

/*
 * function_rfs
 */
int get_rfs_value(void)
{
	int ret;

	MFC_DRV_DBG_LOG("START");

	ret = gpio_get_value(D_RFS_GPIO_NO);

	MFC_DRV_DBG_LOG("END gpio_ret = %d", ret);
	
	return (ret < 0) ? 0 : ret;
	
}

ssize_t rfs_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	ret = gpio_get_value(D_RFS_GPIO_NO);
	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return ret;
	}
	
	if (ret == D_RFS_DEV_HIGH)
		on[0] = SHMFD_RFS_STATUS_HIGH;
	else
		on[0] = SHMFD_RFS_STATUS_LOW;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	
	MFC_DRV_DBG_LOG("END on[0] = %d, len = %d", on[0], len);
	
	return len;
}

static int rfs_open(struct inode *inode, struct file *file)
{	
	MFC_DRV_DBG_LOG("");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	return 0;
}

static int rfs_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

const struct file_operations mfc_rfs_fileops = {
	.owner   = THIS_MODULE,
	.read    = rfs_read,
	.open    = rfs_open,
	.release = rfs_release,
};

static int rfs_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_RFS_DEVS, D_RFS_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rfs_cdev, &mfc_rfs_fileops);
	rfs_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rfs_cdev, dev, D_RFS_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_RFS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_RFS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_cdev);
		unregister_chrdev_region(dev, D_RFS_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
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
 * function_int
 */
ssize_t int_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	ret = gpio_get_value(D_INT_GPIO_NO);
	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return ret;
	}
	
	if (ret == D_INT_DEV_HIGH)
		on[0] = SHMFD_INT_STATUS_HIGH;
	else
		on[0] = SHMFD_INT_STATUS_LOW;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	
	MFC_DRV_DBG_LOG("END on[0] = %d, len = %d", on[0], len);
	
	return len;
}

static int int_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	return 0;
}

static int int_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static const struct file_operations int_fileops = {
	.owner   = THIS_MODULE,
	.read    = int_read,
	.open    = int_open,
	.release = int_release,
};

static int int_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_INT_DEVS, D_INT_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&int_cdev, &int_fileops);
	int_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&int_cdev, dev, D_INT_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_INT_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_INT_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&int_cdev);
		unregister_chrdev_region(dev, D_INT_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void int_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&int_cdev);
	unregister_chrdev_region(dev, D_INT_DEVS);
	
	MFC_DRV_DBG_LOG("END");
}

/*
 * function_pon
 */
int set_pon_value(int value)
{
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* pon high status check */
	if ((value == SHMFD_PON_STATUS_HIGH) && (pon_high_sts == D_PON_HIGH_DISABLE)) {
		return -1;
	}

	/* set pon value */
	gpio_set_value(D_PON_GPIO_NO, value);
	return 0;
}

ssize_t pon_write(struct file *file, const char __user *data,
		       size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}
	
	if (on == SHMFD_PON_STATUS_HIGH)
		seton = D_PON_DEV_HIGH;
	else if (on == SHMFD_PON_STATUS_LOW)
		seton = D_PON_DEV_LOW;
	else {
		MFC_DRV_ERR_LOG("on = %d", on); 
		return -EFAULT;
	}
	
	if (set_pon_value(seton) != 0) {
		MFC_DRV_ERR_LOG("High Sts on = %d, pon_high_sts = %d", on, pon_high_sts); 
		return -EBUSY;
	}
	
	MFC_DRV_DBG_LOG("END on = %d, seton = %d", on, seton);
	
	return len;
}

static long pon_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	MFC_DRV_DBG_LOG("START");
	
	switch(cmd){
	case SHMFD_PON_REQ_HIGH_ENABLE :
		pon_high_sts = D_PON_HIGH_ENABLE;
		break;
	
	case SHMFD_PON_REQ_HIGH_DISABLE :
		pon_high_sts = D_PON_HIGH_DISABLE;
		break;
	
	default:
		MFC_DRV_ERR_LOG("cmd = %d, pon_high_sts = %d", cmd, pon_high_sts);
		return -EINVAL;
	}
	
	MFC_DRV_DBG_LOG("END cmd = %d, pon_high_sts = %d", cmd, pon_high_sts);
	
	return 0;
}

static int pon_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	return 0;
}

static int pon_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return 0;
	}
	
	gpio_set_value(D_PON_GPIO_NO , SHMFD_PON_STATUS_LOW);
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static const struct file_operations pon_fileops = {
	.owner   = THIS_MODULE,
	.write   = pon_write,
	.unlocked_ioctl = pon_ioctl,
	.open    = pon_open,
	.release = pon_release,
};

static int pon_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_PON_DEVS, D_PON_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&pon_cdev, &pon_fileops);
	pon_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&pon_cdev, dev, D_PON_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_PON_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_PON_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&pon_cdev);
		unregister_chrdev_region(dev, D_PON_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void pon_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&pon_cdev);
	unregister_chrdev_region(dev, D_PON_DEVS);
	
	MFC_DRV_DBG_LOG("END");
}

static struct poll_data g_rfs_data, g_int_data;
static struct poll_data *g_rfs_d = &g_rfs_data, *g_int_d = &g_int_data;

/*
 * function_rfs_poll
 */
static void notify_rfs_irq_work_func(struct work_struct *work)
{
	int read_value = 0, old_value = 0, i = 0;
	unsigned long irqflag = 0;

	MFC_DRV_DBG_LOG("START");

	/* anti-chattering */
	old_value = g_rfs_irq_d.rfs_status;
	for (i = 0; i < D_RFS_POLL_SLEEP_NUM; i++) {
		read_value = gpio_get_value(D_RFS_GPIO_NO);
		if ((read_value < 0) || (read_value == old_value)) {
			break;
		}
		usleep(D_RFS_POLL_SLEEP_USEC);
	}

	MFC_DRV_DBG_LOG("read_value = %d old_value = %d", read_value, old_value);

	/* read error */
	if ((read_value >= 0) && (read_value != old_value)) {
		g_rfs_irq_d.rfs_status = read_value;

		/* change irq flag */
		if (read_value == D_RFS_DEV_LOW) {
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		} else {
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
		}

/* COORDINATOR Qualcomm1021 BUILDERR MODIFY start */
		if (irq_set_irq_type(gpio_to_irq(D_RFS_GPIO_NO), irqflag)) {
			MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
		}
/* COORDINATOR Qualcomm1021 BUILDERR MODIFY end */
	}

	/* enable irq handler */
	enable_irq(gpio_to_irq(D_RFS_GPIO_NO));

	/* read changed data or error */
	if ((read_value != old_value) || (read_value < 0)) {
		/* notify rfs irq */
		for (i = 0; i < D_RFS_IRQ_MAX_NUM; i++) {
			if (g_rfs_irq_d.on_irq[i]) {
				(*g_rfs_irq_d.on_irq[i])(read_value);
			}
		}
	}

	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d", read_value, old_value);
}

static irqreturn_t notify_rfs_irq_handler(int irq, void *dev_id)
{
	MFC_DRV_DBG_LOG("START irq = %d", irq);

	disable_irq_nosync(gpio_to_irq(D_RFS_GPIO_NO));
	/* set workqueue */
	schedule_delayed_work(&g_rfs_irq_d.work, msecs_to_jiffies(D_RFS_POLL_DELAY_MSEC));

	MFC_DRV_DBG_LOG("END");

	return IRQ_HANDLED;
}

int request_notify_rfs_irq(rfs_irq_handler on_irq)
{
	int ret, i;
	unsigned long irqflag = 0;

	MFC_DRV_DBG_LOG("START req_count = %d", g_rfs_irq_d.req_count);

	if (!on_irq) {
		MFC_DRV_DBG_LOG("on_irq is null");
		return -1;
	}

	ret = gpio_get_value(D_RFS_GPIO_NO);
	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -1;
	}

	if (g_rfs_irq_d.req_count == 0) {
		g_rfs_irq_d.rfs_status = ret;

		/* set irq handler */
		if (ret == D_RFS_DEV_LOW) {
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		} else {
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
		}

		if (request_irq(gpio_to_irq(D_RFS_GPIO_NO),
						notify_rfs_irq_handler,
						irqflag,
						D_RFS_IRQ_NAME,
						(void *)&g_rfs_irq_d)) {
			MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
			return -1;
		}

		if (enable_irq_wake(gpio_to_irq(D_RFS_GPIO_NO))) {
			free_irq(gpio_to_irq(D_RFS_GPIO_NO), (void *)&g_rfs_irq_d);
			MFC_DRV_ERR_LOG("enable_irq_wake");
			return -1;
		}
	}

	for (i = 0; i < D_RFS_IRQ_MAX_NUM; i++) {
		if (g_rfs_irq_d.on_irq[i] == NULL) {
			g_rfs_irq_d.on_irq[i] = on_irq;
			g_rfs_irq_d.req_count++;
			MFC_DRV_DBG_LOG("END ret = %d, req_count = %d, index = %d", ret, g_rfs_irq_d.req_count, i);
			return ret;
		}
	}

	MFC_DRV_ERR_LOG("END failed");
	return -1;
}

void free_notify_rfs_irq(rfs_irq_handler on_irq)
{
	int i;

	MFC_DRV_DBG_LOG("START");

	if (!on_irq) {
		MFC_DRV_DBG_LOG("on_irq is null");
		return;
	}

	if (g_rfs_irq_d.req_count == 0) {
		MFC_DRV_DBG_LOG("req_count is none");
		return;
	}

	for (i = 0; i < D_RFS_IRQ_MAX_NUM; i++) {
		if (g_rfs_irq_d.on_irq[i] == on_irq) {
			g_rfs_irq_d.on_irq[i] = NULL;
			g_rfs_irq_d.req_count--;
			break;
		}
	}

	if (g_rfs_irq_d.req_count > 0) {
		MFC_DRV_DBG_LOG("req_count = %d", g_rfs_irq_d.req_count);
		return;
	}

	/* clear workqueue */
	cancel_delayed_work(&g_rfs_irq_d.work);

	if (disable_irq_wake(gpio_to_irq(D_RFS_GPIO_NO))) {
		MFC_DRV_ERR_LOG("disable_irq_wake");
	}

	free_irq(gpio_to_irq(D_RFS_GPIO_NO), (void *)&g_rfs_irq_d);

	MFC_DRV_DBG_LOG("END");
}

static void rfs_poll_on_irq(int rfs_status)
{
	struct poll_data *rfs_d = g_rfs_d;

	if (rfs_status < 0) {
		rfs_d->read_error = 1;
	} else {
		rfs_d->device_status = rfs_status;
	}

	rfs_d->irq_handler_done = 1;
	wake_up_interruptible(&rfs_d->read_wait);
}

unsigned int rfs_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *rfs_d = g_rfs_d;
	unsigned int mask = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	/* wait irq handler */
	poll_wait(file, &rfs_d->read_wait, wait);
	if (rfs_d->irq_handler_done)
		mask = POLLIN | POLLRDNORM;
	
	MFC_DRV_DBG_LOG("END mask = %d", mask);
	
	return (mask);
}

ssize_t rfs_poll_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	struct poll_data *rfs_d = g_rfs_d;
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");

	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (!rfs_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			MFC_DRV_ERR_LOG("NONBLOCK");
			return -EAGAIN;
		}
		/* wait irq handler */
		ret = wait_event_interruptible(rfs_d->read_wait,
		                               rfs_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			MFC_DRV_DBG_LOG("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	/* read failed */
	if (rfs_d->read_error) {
		rfs_d->irq_handler_done = 0;
		rfs_d->read_error = 0;
		MFC_DRV_ERR_LOG("rfs_d->read_error = %d", rfs_d->read_error);
		return -EIO;
	}
	
	/* set readed data */
	if (rfs_d->device_status == D_RFS_DEV_HIGH)
		on[0] = SHMFD_RFS_STATUS_HIGH;
	else
		on[0] = SHMFD_RFS_STATUS_LOW;

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		on[0] = SHMFD_RFS_STATUS_HIGH;
	}

	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	rfs_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END len = %d, on[0] = %d", len, on[0]);
	
	return len;
}

static int rfs_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;
	int ret = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* only one time success */
	if (rfs_d->open_flag) {
		MFC_DRV_ERR_LOG("only one time");
		return -EBUSY;
	}
	rfs_d->open_flag = 1;
	
	/* request rfs irq */
	ret = request_notify_rfs_irq(&rfs_poll_on_irq);
	if (ret < 0) {
		rfs_d->open_flag = 0;
		MFC_DRV_ERR_LOG("request_notify_rfs_irq ret = %d", ret);
		return -EIO;
	}
	
	rfs_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static int rfs_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;
	
	MFC_DRV_DBG_LOG("START");
	
	/* free rfs irq */
	free_notify_rfs_irq(&rfs_poll_on_irq);
	
	rfs_d->open_flag = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static const struct file_operations rfs_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = rfs_poll_read,
	.open    = rfs_poll_open,
	.release = rfs_poll_release,
	.poll    = rfs_poll_poll,
};

static int rfs_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	/* get major number */
	sdResult = alloc_chrdev_region(&dev , 0 , D_RFS_POLL_DEVS, D_RFS_POLL_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	/* initialize RFS */
	cdev_init(&rfs_poll_cdev, &rfs_poll_fileops);
	rfs_poll_cdev.owner = THIS_MODULE;
	
	/* add RFS */
	sdResult = cdev_add(&rfs_poll_cdev, dev, D_RFS_POLL_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_RFS_POLL_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	/* create RFS */
	class_dev = device_create(felica_class, NULL, dev, NULL, D_RFS_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_poll_cdev);
		unregister_chrdev_region(dev, D_RFS_POLL_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	/* initialize rfs irq data */
	memset(&g_rfs_irq_d, 0, sizeof(g_rfs_irq_d));
	/* initialize rfsirq workqueue */
	INIT_DELAYED_WORK(&g_rfs_irq_d.work, notify_rfs_irq_work_func);
	
	/* initialize poll_data */
	memset(g_rfs_d, 0x00, sizeof(struct poll_data));
	/* initialize waitqueue */
	init_waitqueue_head(&g_rfs_d->read_wait);
	
	g_rfs_d->open_flag = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void rfs_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&rfs_poll_cdev);
	unregister_chrdev_region(dev, D_RFS_POLL_DEVS);
	MFC_DRV_DBG_LOG("END");
}

/*
 * function_int_poll
 */
static irqreturn_t int_poll_irq_handler(int irq, void *dev_id);
void int_poll_work_func(struct work_struct *work)
{
	struct poll_data *int_d = g_int_d;
	int read_value = 0, old_value = 0;
	unsigned long irqflag = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	old_value = int_d->device_status;

	read_value = gpio_get_value(D_INT_POLL_GPIO_NO);

	MFC_DRV_DBG_LOG("read_value = %d old_value = %d", read_value, old_value);
	
	/* read error */
	if (read_value < 0) {
		int_d->read_error = read_value;
	/* read changed data */
	} else if (read_value != old_value) {
		int_d->device_status = read_value;
		int_d->read_error = 0;
		
		/* change irq flag */
		if (int_d->device_status == D_INT_DEV_LOW)
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		else
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
/* COORDINATOR Qualcomm1021 BUILDERR MODIFY start */
		if (irq_set_irq_type(gpio_to_irq(D_INT_POLL_GPIO_NO), irqflag))
			MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
/* COORDINATOR Qualcomm1021 BUILDERR MODIFY end */
	}

	if (wake_lock_active(&g_int_wake_lock)) {
		wake_unlock(&g_int_wake_lock);
	}
	wake_lock_timeout(&g_int_wake_lock, D_INT_WAKE_LOCK_TIMEOUT);
	
	/* enable irq handler */
	enable_irq(gpio_to_irq(D_INT_POLL_GPIO_NO));
	
	/* read changed data or error */
	if (read_value != old_value || int_d->read_error) {
		if ((snfc_available()) && (snfc_available_wake_up())) {
			/* wakeup poll and read */
			int_d->irq_handler_done = 1;
			wake_up_interruptible(&int_d->read_wait);
		}
	}
	
	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d, int_d->read_error = %d"
					, read_value, old_value, int_d->read_error);
}

static irqreturn_t int_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *int_d = g_int_d;
	
	MFC_DRV_DBG_LOG("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(D_INT_POLL_GPIO_NO));
	/* set workqueue */
	schedule_delayed_work(&int_d->work, msecs_to_jiffies(D_INT_POLL_DELAY_MSEC));
	
	MFC_DRV_DBG_LOG("END");
	
	return IRQ_HANDLED;
}

unsigned int int_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *int_d = g_int_d;
	unsigned int mask = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	/* wait irq handler */
	poll_wait(file, &int_d->read_wait, wait);
	if (int_d->irq_handler_done)
		mask = POLLIN | POLLRDNORM;
	
	MFC_DRV_DBG_LOG("END mask = %d", mask);
	
	return (mask);
}

ssize_t int_poll_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	struct poll_data *int_d = g_int_d;
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");

	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (!int_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			MFC_DRV_ERR_LOG("NONBLOCK");
			return -EAGAIN;
		}
		/* wait irq handler */
		ret = wait_event_interruptible(int_d->read_wait,
		                               int_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			MFC_DRV_DBG_LOG("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	/* read failed */
	if (int_d->read_error) {
		int_d->irq_handler_done = 0;
		int_d->read_error = 0;
		MFC_DRV_ERR_LOG("int_d->read_error = %d", int_d->read_error);
		return -EIO;
	}
	
	/* set readed data */
	if (int_d->device_status == D_INT_DEV_HIGH)
		on[0] = SHMFD_INT_STATUS_HIGH;
	else
		on[0] = SHMFD_INT_STATUS_LOW;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	int_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END len = %d, on[0] = %d", len, on[0]);
	
	return len;
}

static int int_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* only one time success */
	if (int_d->open_flag) {
		MFC_DRV_ERR_LOG("only one time");
		return -EBUSY;
	}
	int_d->open_flag = 1;
	
	/* preparation of anti-chattering */
	ret = gpio_get_value(D_INT_POLL_GPIO_NO);
	if (ret < 0) {
		int_d->open_flag = 0;
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	int_d->device_status = ret;
	
	/* set irq handler */
	if (int_d->device_status == D_INT_DEV_LOW)
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	else
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	
	if (request_irq(gpio_to_irq(D_INT_POLL_GPIO_NO),
	                int_poll_irq_handler,
	                irqflag,
	                D_INT_POLL_DEV_NAME,
	                (void*)int_d)) {
		
		int_d->open_flag = 0;
		
		MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
		
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(D_INT_POLL_GPIO_NO))){
		
		MFC_DRV_ERR_LOG("enable_irq_wake");
		
		free_irq(gpio_to_irq(D_INT_POLL_GPIO_NO), (void *)int_d);
		
		return -EIO;
	}
	
	int_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static int int_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	
	MFC_DRV_DBG_LOG("START");
	
	/* clear workqueue */
	cancel_delayed_work(&int_d->work);
	
	if (disable_irq_wake(gpio_to_irq(D_INT_POLL_GPIO_NO)))
		MFC_DRV_ERR_LOG("disable_irq_wake");
	
	free_irq(gpio_to_irq(D_INT_POLL_GPIO_NO), (void *)int_d);
	
	int_d->open_flag = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static const struct file_operations int_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = int_poll_read,
	.open    = int_poll_open,
	.release = int_poll_release,
	.poll    = int_poll_poll,
};

static int int_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	/* get major number */
	sdResult = alloc_chrdev_region(&dev , 0 , D_INT_POLL_DEVS, D_INT_POLL_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	/* initialize INT */
	cdev_init(&int_poll_cdev, &int_poll_fileops);
	int_poll_cdev.owner = THIS_MODULE;
	
	/* add INT */
	sdResult = cdev_add(&int_poll_cdev, dev, D_INT_POLL_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_INT_POLL_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	/* create INT */
	class_dev = device_create(felica_class, NULL, dev, NULL, D_INT_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&int_poll_cdev);
		unregister_chrdev_region(dev, D_INT_POLL_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	/* initialize poll_data */
	memset(g_int_d, 0x00, sizeof(struct poll_data));
	/* initialize workqueue */
	INIT_DELAYED_WORK(&g_int_d->work, int_poll_work_func);
	/* initialize waitqueue */
	init_waitqueue_head(&g_int_d->read_wait);
	
	g_int_d->open_flag = 0;
	
    wake_lock_init(&g_int_wake_lock, WAKE_LOCK_SUSPEND, "int_wake_lock");

	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void int_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&int_poll_cdev);
	unregister_chrdev_region(dev, D_INT_POLL_DEVS);

	if (wake_lock_active(&g_int_wake_lock)) {
	    wake_unlock(&g_int_wake_lock);
	}
    wake_lock_destroy(&g_int_wake_lock);

	MFC_DRV_DBG_LOG("END");
}

/*
 * function_clf_rws
 */
extern const struct file_operations mfc_rws_fileops;

static int rws_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_RWS_DEVS, D_RWS_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rws_cdev, &mfc_rws_fileops);
	rws_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rws_cdev, dev, D_RWS_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_RWS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_RWS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rws_cdev);
		unregister_chrdev_region(dev, D_RWS_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void rws_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&rws_cdev);
	unregister_chrdev_region(dev, D_RWS_DEVS);
	
	MFC_DRV_DBG_LOG("END");
}

/*
 * function_cen
 */
int felica_ppc_read_E2PROM(unsigned char item_id, unsigned char* ppc_data_ptr)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START item_id = %d", item_id);
	
	/* NULL check */
	if (ppc_data_ptr == NULL) {
		MFC_DRV_ERR_LOG("ppc_data_ptr NULL");
		
		return FALSE;
	}
	
	/* check item_id	*/
	if ((item_id < FELICA_PPC_E2PROM_ITEM_ID_MIN) ||
		(item_id > FELICA_PPC_E2PROM_ITEM_ID_MAX)) {
		MFC_DRV_ERR_LOG("item_id = %d", item_id);
		
		return FALSE;
	}
	
	/* read_status */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check_PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	switch(item_id) {
	case FELICA_PPC_CHANGE_ACCESSMODE_COMMAND:						/* change access mode	*/
		get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
		*ppc_data_ptr = get_status_register >> 1;
		MFC_DRV_DBG_LOG("END CHANGE_ACCESSMODE_COMMAND");
		return TRUE;
	case FELICA_PPC_READ_STATUS_REGISTER_COMMAND:					/* read status register	*/
		*ppc_data_ptr = get_status_register & 0x0F;
		MFC_DRV_DBG_LOG("END READ_STATUS_REGISTER_COMMAND");
		return TRUE;
	case FELICA_PPC_SET_PORT_DATA_COMMAND:							/* set port							*/
	case FELICA_PPC_SET_SLAVEADDRESS_COMMAND:						/* set slave address				*/
		break;
	default:
		return FALSE;
	}
	
	/* change access mode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	/* check access mode */
	if (get_status_register != FELICA_PPC_STATUS_E2PROM_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_E2PROM_ACCESSMODE);
		
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* read ppc data */
	i2c_ret = mfd_LD_ppc_read_process(item_id, ppc_data_ptr);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read");
		
		return FALSE;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_read_register(unsigned char item_id, unsigned char* ppc_data_ptr)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START item_id = %d", item_id);
	
	/* NULL check */
	if (ppc_data_ptr == NULL) {
		MFC_DRV_ERR_LOG("ppc_data_ptr NULL");
		
		return FALSE;
	}
	
	if ((item_id < FELICA_PPC_E2PROM_ITEM_ID_MIN) ||
		(item_id > FELICA_PPC_E2PROM_ITEM_ID_MAX)) {
		MFC_DRV_ERR_LOG("item_id = %d", item_id);
		
		return FALSE;
	}
	
	/* read_status */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	switch(item_id) {
	case FELICA_PPC_CHANGE_ACCESSMODE_COMMAND:				/* change access mode	*/
		get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
		*ppc_data_ptr = get_status_register >> 1;
		MFC_DRV_DBG_LOG("END CHANGE_ACCESSMODE_COMMAND");
		return TRUE;
	case FELICA_PPC_READ_STATUS_REGISTER_COMMAND:			/* read status register	*/
		*ppc_data_ptr = get_status_register & 0x0F;
		MFC_DRV_DBG_LOG("END READ_STATUS_REGISTER_COMMAND");
		return TRUE;
	case FELICA_PPC_SET_PORT_DATA_COMMAND:					/* set port				*/
		*ppc_data_ptr = get_status_register & FELICA_PPC_STATUS_OUTPUT_MASK;
		MFC_DRV_DBG_LOG("END SET_PORT_DATA_COMMAND");
		return TRUE;
	case FELICA_PPC_SET_SLAVEADDRESS_COMMAND:				/* set slave address	*/
		MFC_DRV_DBG_LOG("SET_SLAVEADDRESS_COMMAND");
		break;
	default:
		return FALSE;
	}
	
	/* change access mode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	/* check access mode */
	if (get_status_register != FELICA_PPC_STATUS_REGISTER_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_REGISTER_ACCESSMODE);
		if(i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* read ppc data */
	i2c_ret = mfd_LD_ppc_read_process(item_id, ppc_data_ptr);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read");
		
		return FALSE;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_write_E2PROM(unsigned char item_id, unsigned char set_ppc_data)
{
	int				i2c_ret				= FALSE;
	int				check_data_ret		= FALSE;
	unsigned char	get_status_register	= 0xFF;
	int				wait_flg			= FELICA_PPC_NO_WAIT;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START item_id = %d, set_ppc_data = %d", item_id, set_ppc_data);
	
	if ((item_id == FELICA_PPC_SET_PORT_DATA_COMMAND) ||
		(item_id == FELICA_PPC_SET_SLAVEADDRESS_COMMAND)) {
		wait_flg = FELICA_PPC_NEED_WAIT;				/*  write E2PROM	*/
		MFC_DRV_DBG_LOG("item_id = 0x06 or 0x07 SET_WAIT");
	} else if (item_id == FELICA_PPC_CHANGE_ACCESSMODE_COMMAND) {
		MFC_DRV_DBG_LOG("item_id = 0x04 SET_NO_WAIT");	/* write register 		*/
	} else {
		MFC_DRV_ERR_LOG("item_id = %d", item_id);
		return FALSE;
	}
	
	check_data_ret = mfd_LD_ppc_check_set_data(item_id, set_ppc_data);	/* check set_ppc_data	*/
	
	if (check_data_ret == FALSE) {
		MFC_DRV_ERR_LOG("set_ppc_data = %d", set_ppc_data);
		
		return FALSE;
	}
	
	/* read status */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	/* change accessmode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	/* check access mode				*/
	if (get_status_register != FELICA_PPC_STATUS_E2PROM_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_E2PROM_ACCESSMODE);
		
		if(i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* E2PROM write enable */
	i2c_ret = mfd_LD_ppc_E2PROM_write_protect_process(FELICA_PPC_WRITE_ENABLE);	
	
	if (i2c_ret < TRUE) {
		i2c_ret = mfd_LD_ppc_E2PROM_write_protect_process(FELICA_PPC_WRITE_DISABLE);
		MFC_DRV_ERR_LOG("E2PROM_write_protect");
		
		return FALSE;
	}
	
	/* write ppc data */
	i2c_ret = mfd_LD_ppc_write_process(item_id, set_ppc_data, wait_flg);
	
	if (i2c_ret < TRUE) {
		i2c_ret = mfd_LD_ppc_E2PROM_write_protect_process(FELICA_PPC_WRITE_DISABLE);
		MFC_DRV_ERR_LOG("write");
		
		return FALSE;
	}
	/* check accessmode */
	/* case : command = 0x04 setdata = 0 (registaer accessmode)	*/
	if ((item_id == FELICA_PPC_CHANGE_ACCESSMODE_COMMAND) &&
		(set_ppc_data == FELICA_PPC_WRITE_DISABLE)) {
		i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("read_status_register(registaer)");
			
			return FALSE;
		}
		get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
		/* case : registermode				*/
		if(get_status_register == FELICA_PPC_STATUS_E2PROM_ACCESSMODE) {
			MFC_DRV_ERR_LOG("accsessmode(registaer)");
			
			return FALSE;
		}
	} else {
		i2c_ret = mfd_LD_ppc_E2PROM_write_protect_process(FELICA_PPC_WRITE_DISABLE);	/* E2PROM write disable */
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("E2PROM_write_protect");
			
			return FALSE;
		}
	}
	
#ifdef CONFIG_SHSNFC
	notify_nfc_avalable_change(NFC_AVAILABLE_CEN, ((set_ppc_data & FELICA_CONTROL_LOCK_MASK) == CEN_LOCK_OFF));
#endif /* CONFIG_SHSNFC */
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_write_register(unsigned char item_id, unsigned char set_ppc_data)
{
	int				i2c_ret				= FALSE;
	int				check_data_ret		= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START item_id = %d, set_ppc_data = %d", item_id, set_ppc_data);
	
	if ((item_id < FELICA_PPC_E2PROM_ITEM_ID_MIN) ||
		(item_id > FELICA_PPC_E2PROM_ITEM_ID_MAX) ||
		(item_id == FELICA_PPC_READ_STATUS_REGISTER_COMMAND)) {
		MFC_DRV_ERR_LOG("item_id = %d", item_id);
		
		return FALSE;
	}
	
	/* check ppc_data		*/
	check_data_ret = mfd_LD_ppc_check_set_data(item_id, set_ppc_data);
	
	if (check_data_ret == FALSE) {
		MFC_DRV_ERR_LOG("set_ppc_data = %d", set_ppc_data);
		
		return FALSE;
	}
	
	/* read status */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	/* change access mode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	if (get_status_register != FELICA_PPC_STATUS_REGISTER_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_REGISTER_ACCESSMODE);
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* write ppc data */
	i2c_ret = mfd_LD_ppc_write_process(item_id, set_ppc_data, FELICA_PPC_NO_WAIT);
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("write");
		
		return FALSE;
	}
	
#ifdef CONFIG_SHSNFC
	notify_nfc_avalable_change(NFC_AVAILABLE_CEN, ((set_ppc_data & FELICA_CONTROL_LOCK_MASK) == CEN_LOCK_OFF));
#endif /* CONFIG_SHSNFC */
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_reload(void)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START");
	
	/* read status */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	/* change access mode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	if (get_status_register != FELICA_PPC_STATUS_REGISTER_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_REGISTER_ACCESSMODE);
		
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* reload */
	i2c_ret = mfd_LD_ppc_reload_process();
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("reload");
		
		return FALSE;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_reverse_output(void)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START");
	
	/* read status register */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	/* change access mode  */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	if (get_status_register != FELICA_PPC_STATUS_REGISTER_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_REGISTER_ACCESSMODE);
		
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
			
			return FALSE;
		}
	}
	
	/* reverse output	*/
	i2c_ret = mfd_LD_ppc_reverse_output_process();
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("reverse_output");
		
		return FALSE;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

int felica_ppc_write_protect(int write_protect)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	ppc_check_por_state	= 0x0F;
	
	MFC_DRV_DBG_LOG("START");
	
	/* read status register */
	i2c_ret = mfd_LD_ppc_read_status_register_process(&get_status_register);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("read_status_register");
		
		return FALSE;
	}
	
	/* check PoR */
	ppc_check_por_state = get_status_register & FELICA_PPC_STATUS_POR_MASK;
	
	if (ppc_check_por_state == FELICA_PPC_STATUS_POR) {
		MFC_DRV_ERR_LOG("PPC_state_of_POR");
		
		return FALSE;
	}
	
	/* change access mode */
	get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
	
	if (get_status_register != FELICA_PPC_STATUS_E2PROM_ACCESSMODE) {
		i2c_ret = mfd_LD_ppc_change_accessmode_process(FELICA_PPC_CHANGE_E2PROM_ACCESSMODE);
		if (i2c_ret < TRUE) {
			MFC_DRV_ERR_LOG("change_accessmode");
	
			return FALSE;
		}
	}
	
	/* write protect */
	i2c_ret = mfd_LD_ppc_E2PROM_write_protect_process(write_protect);
	
	if (i2c_ret < TRUE) {
		MFC_DRV_ERR_LOG("write_protect");
		
		return FALSE;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return TRUE;
}

static int mfd_LD_ppc_E2PROM_write_protect_process(int write_protect)
{
	int					i2c_ret					= FALSE;
	unsigned char		get_status_register		= 0xFF;
	unsigned char		check_write_protect		= 0x00;
	unsigned char		count					= 0;		/* retry count	*/
	
	MFC_DRV_DBG_LOG("START write_protect = %d", write_protect);
	
	check_write_protect = write_protect << 2;
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_E2PROM_write_protect(write_protect);
		/* success write_protect				*/
		if (i2c_ret > FALSE) {
			i2c_ret = mfd_PD_ppc_read_status_register(&get_status_register);
			if (i2c_ret > FALSE) {
				get_status_register &= FELICA_PPC_STATUS_WRITE_PROTECT_MASK;	/* get status_write_protect*/
				if (get_status_register == check_write_protect) {
					break;
				} else {
					i2c_ret = FALSE;
				}
			}
		}
		MFC_DRV_DBG_LOG("i2c_retry count = %d", count);
	}
	MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_read_status_register_process(unsigned char* ppc_data_ptr)
{
	int				i2c_ret		= FALSE;
	unsigned char	count		= 0;		/* retry count					*/
	
	MFC_DRV_DBG_LOG("START");
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_read_status_register(ppc_data_ptr);
		if (i2c_ret > FALSE) {
			break;
		} else {
			MFC_DRV_DBG_LOG("i2c_retry_count = %d ", count);
		}
	}
	
	MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_read_process(unsigned char cmd, unsigned char* read_data_ptr)
{
	int				i2c_ret		= FALSE;
	unsigned char	count		= 0;		/* retry_count		*/
	
	MFC_DRV_DBG_LOG("START cmd = %d", cmd);
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_read(cmd, read_data_ptr);			/* read ppc_data			*/
		if(i2c_ret > FALSE) {
			break;
		} else {
			MFC_DRV_DBG_LOG("i2c_retry_count = %d ", count);
		}
	}
	
	MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_change_accessmode_process(int accessmode)
{
	int				i2c_ret				= FALSE;
	unsigned char	get_status_register	= 0xFF;
	unsigned char	check_accessmode	= 0x00;
	unsigned char	count				= 0;
	
	MFC_DRV_DBG_LOG("START accessmode = %d", accessmode);
	
	check_accessmode = accessmode << 1;
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_change_accessmode(accessmode);
		if (i2c_ret > FALSE) {
			i2c_ret = mfd_PD_ppc_read_status_register(&get_status_register);
			if (i2c_ret > FALSE) {
				get_status_register &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
				/* set access mode		*/
				if(get_status_register == check_accessmode) {
					break;
				} else {
					i2c_ret = FALSE;
				}
			}
		}
		MFC_DRV_DBG_LOG("i2c_retry_count = %d ", count);
	}
	
	MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_write_process(unsigned char cmd, unsigned char set_data, int wait_flg)
{
	int				i2c_ret			= FALSE;
	unsigned char	get_ppc_data	= 0xFF;
	unsigned char	count			= 0;
	
	MFC_DRV_DBG_LOG("START cmd = %d, set_data = %d, wait_flg = %d", cmd, set_data, wait_flg);
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_write(cmd, set_data, wait_flg);		/* write ppc_data			*/
		
		if (i2c_ret > FALSE) {
			if (cmd != FELICA_PPC_SET_SLAVEADDRESS_COMMAND) {
				i2c_ret = mfd_PD_ppc_read_status_register(&get_ppc_data);
			} else {
				/* case set_slave_address	*/
				i2c_ret = mfd_PD_ppc_read(cmd, &get_ppc_data);
			}
			
			if (i2c_ret > FALSE) {
				switch (cmd) {
				case FELICA_PPC_CHANGE_ACCESSMODE_COMMAND:		/* change access mode	*/
					MFC_DRV_DBG_LOG("CHANGE_ACCESSMODE_COMMAND");
					get_ppc_data &= FELICA_PPC_STATUS_ACCESSMODE_MASK;
					get_ppc_data >>= 1;
					break;
				case FELICA_PPC_SET_PORT_DATA_COMMAND:			/* set port				*/
					MFC_DRV_DBG_LOG("SET_PORT_DATA_COMMAND");
					get_ppc_data &= FELICA_PPC_STATUS_OUTPUT_MASK;
					break;
				case FELICA_PPC_SET_SLAVEADDRESS_COMMAND:		/* set slave address	*/
					MFC_DRV_DBG_LOG("SET_SLAVEADDRESS_COMMAND");
					break;
				default:
					MFC_DRV_ERR_LOG("cmd = %d", cmd);
					return FALSE;
				}
				
				if (get_ppc_data == set_data) {
					/* success	*/
					break;
				} else {
					i2c_ret = FALSE;
				}
			}
		}
		MFC_DRV_DBG_LOG("i2c_retry count = %d", count);
	}
	
	MFC_DRV_DBG_LOG("END i2c_ret = %d ", i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_check_set_data(unsigned char cmd, unsigned char set_data)
{
	unsigned char	ppc_set_max_data	= 0x00;
	
	MFC_DRV_DBG_LOG("START cmd = %d, set_data = %d" ,cmd, set_data);
	
	switch (cmd) {
	case FELICA_PPC_SET_PORT_DATA_COMMAND:			/* set port					*/
	case FELICA_PPC_CHANGE_ACCESSMODE_COMMAND:		/* change access mode		*/
		ppc_set_max_data = 0x01;
		break;
	case FELICA_PPC_READ_STATUS_REGISTER_COMMAND:	/* read status				*/
		ppc_set_max_data = 0x0F;
		break;
	case FELICA_PPC_SET_SLAVEADDRESS_COMMAND:		/* set slave address		*/
		ppc_set_max_data = 0x07;
		break;
	default:										/* command error			*/
		MFC_DRV_ERR_LOG("cmd = %d", cmd);
		return FALSE;
	}
	
	if(set_data <= ppc_set_max_data) {
		MFC_DRV_DBG_LOG("END");
		return TRUE;
	} else {
		MFC_DRV_ERR_LOG("set_data = %d, ppc_set_max_data = %d", set_data, ppc_set_max_data);
		return FALSE;
	}
}

static int mfd_LD_ppc_reload_process(void)
{
	int				i2c_ret		= FALSE;
	unsigned char	count		= 0;
	
	MFC_DRV_DBG_LOG("START");
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_reload();
		
		if (i2c_ret > FALSE) {
			break;
		} else {
			MFC_DRV_DBG_LOG("i2c_retry count = %d", count);
		}
	}
	
	MFC_DRV_DBG_LOG("END i2c_ret = %d",i2c_ret);
	
	return i2c_ret;
}

static int mfd_LD_ppc_reverse_output_process(void)
{

	int				i2c_ret		= FALSE;
	unsigned char	count		= 0;
	
	MFC_DRV_DBG_LOG("START");
	
	for (count = 0; count < FELICA_PPC_I2C_TRY_COUNT; count++) {
		i2c_ret = mfd_PD_ppc_reverse_output();
		
		if (i2c_ret > FALSE) {
			break;
		} else {
			MFC_DRV_DBG_LOG("i2c_retry count = %d", count);
		}
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return i2c_ret;
}

static int mfd_PD_ppc_read_status_register(unsigned char* ppc_data_ptr)
{
	int i2c_ret	= FALSE;
	
	MFC_DRV_DBG_LOG("START");
	
	i2c_ret = mfd_PD_ppc_i2c_read(FELICA_PPC_READ_STATUS_REGISTER_COMMAND, ppc_data_ptr);
	
	MFC_DRV_DBG_LOG("END");
	
	return i2c_ret;
}

static int mfd_PD_ppc_change_accessmode(int accessmode)
{
	int				i2c_ret		= FALSE;
	unsigned char	set_data	= FELICA_PPC_SET_REGISTER_ACCESSMODE;
	
	MFC_DRV_DBG_LOG("START accessmode = %d", accessmode);
	
	if (accessmode == FELICA_PPC_CHANGE_E2PROM_ACCESSMODE) {
		set_data = FELICA_PPC_SET_E2PROM_ACCESSMODE;
	} else {
		set_data = FELICA_PPC_SET_REGISTER_ACCESSMODE;
	}
	
	i2c_ret = mfd_PD_ppc_i2c_write(FELICA_PPC_CHANGE_ACCESSMODE_COMMAND, set_data, FELICA_PPC_NO_WAIT);
	
	if (i2c_ret > FALSE) {
		MFC_DRV_DBG_LOG("END");
		return TRUE;
	} else {
		MFC_DRV_ERR_LOG("mfd_PD_ppc_i2c_write i2c_ret = %d", i2c_ret);
		return FALSE;
	}
}

static int mfd_PD_ppc_read(unsigned char cmd, unsigned char* read_data_ptr)
{
	int read_ppc_ret		= FALSE;
	int extract_data_ret	= FALSE;
	
	MFC_DRV_DBG_LOG("START cmd = %d", cmd);
	
	if (read_data_ptr == NULL) {
		MFC_DRV_ERR_LOG("read_data_ptr NULL");
		
		return FALSE;
	}
	
	read_ppc_ret = mfd_PD_ppc_i2c_read(cmd, read_data_ptr);
	
	if (read_ppc_ret < TRUE) {
		MFC_DRV_ERR_LOG("ppc_read");
		
		return FALSE;
	}
	
	extract_data_ret = mfd_PD_ppc_extract_read_data(cmd, read_data_ptr);
	
	if (extract_data_ret > FALSE) {
		MFC_DRV_DBG_LOG("END");
		return TRUE;
	} else {
		MFC_DRV_ERR_LOG("extract_read_data extract_data_ret = %d",extract_data_ret);
		return FALSE;
	}
}

static int mfd_PD_ppc_write(unsigned char cmd, unsigned char set_data, int wait_flg)
{
	int write_ppc_ret	= FALSE;
	
	MFC_DRV_DBG_LOG("START cmd = %d, set_data = %d, wait_flg = %d", cmd, set_data, wait_flg);
	
	write_ppc_ret = mfd_PD_ppc_i2c_write(cmd, set_data, wait_flg);
	
	MFC_DRV_DBG_LOG("END write_ppc_ret = %d", write_ppc_ret);
	
	return write_ppc_ret;
}

static int mfd_PD_ppc_E2PROM_write_protect(int write_protect)
{

	int	i2c_ret		= FALSE;
	
	MFC_DRV_DBG_LOG("START write_protect = %d", write_protect);
	
	if (write_protect != FELICA_PPC_WRITE_ENABLE) {
		i2c_ret = mfd_PD_ppc_i2c_write_no_data(FELICA_PPC_E2PROM_WRITE_DISABLE_COMMAND);
	} else {
		i2c_ret = mfd_PD_ppc_i2c_write_no_data(FELICA_PPC_E2PROM_WRITE_ENABLE_COMMAND);
	}
	
	if (i2c_ret > FALSE) {
		MFC_DRV_DBG_LOG("END");
		
		return TRUE;
	} else {
		MFC_DRV_ERR_LOG("i2c_ret = %d", i2c_ret);
		
		return FALSE;
	}
}

static int mfd_PD_ppc_reload(void)
{
	int reload_ret = FALSE;
	
	MFC_DRV_DBG_LOG("START");
	
	reload_ret = mfd_PD_ppc_i2c_write_no_data(FELICA_PPC_RELOAD_COMMAND);
	
	MFC_DRV_DBG_LOG("END reload_ret = %d", reload_ret);
	
	return reload_ret;
}

static int mfd_PD_ppc_reverse_output(void)
{
	int reverse_ret = FALSE;
	
	MFC_DRV_DBG_LOG("START");
	
	reverse_ret = mfd_PD_ppc_i2c_write_no_data(FELICA_PPC_REVERSE_OUTPUT_COMMAND);
	
	MFC_DRV_DBG_LOG("END reverse_ret = %d", reverse_ret);
	
	return reverse_ret;
}

static int mfd_PD_ppc_i2c_read(unsigned char cmd, unsigned char* read_data_ptr)
{
	int	i2c_ret	= I2C_FAILURE;
	
	/* i2c_transfer(read) */
	struct i2c_msg mesgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= read_data_ptr,
		},
	};
	
	MFC_DRV_DBG_LOG("START cmd = %d", cmd);
	
	if (read_data_ptr == NULL) {
		MFC_DRV_ERR_LOG("read_data_ptr NULL");
		
		return FALSE;
	}
	
	i2c_ret = i2c_transfer(this_client->adapter, mesgs, 2);
	
	if (i2c_ret <= I2C_FAILURE) {
		MFC_DRV_ERR_LOG("i2c_ret = %d", i2c_ret);
		
		return FALSE;
	} else {
		MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
		
		return TRUE;
	}
}

static int mfd_PD_ppc_i2c_write(unsigned char cmd, unsigned char set_data, int wait_flg)
{
	int		i2c_ret		= I2C_FAILURE;
	unsigned char	txData[2]	= {cmd, set_data};
	
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= txData,
		},
	};
	
	MFC_DRV_DBG_LOG("START cmd = %d, set_data = %d, wait_flg = %d", cmd, set_data, wait_flg);
	
	i2c_ret = i2c_transfer(this_client->adapter, msg, 1);
	
	if (wait_flg == FELICA_PPC_NEED_WAIT) {
		msleep(I2C_WRITE_WAIT_TIME);
	}
	
	if (i2c_ret <= I2C_FAILURE) {
		MFC_DRV_ERR_LOG("i2c_ret = %d", i2c_ret);
		
		return FALSE;
	} else {
		MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
		
		return TRUE;
	}
}

static int mfd_PD_ppc_extract_read_data(unsigned char cmd, unsigned char* read_data_ptr)
{
	MFC_DRV_DBG_LOG("START cmd = %d", cmd);
	
	/* NULL check	*/
	if (read_data_ptr == NULL) {
		MFC_DRV_ERR_LOG("read_data_ptr NULL");
		
		return FALSE;
	}
	
	switch (cmd) {
	case FELICA_PPC_SET_PORT_DATA_COMMAND:			/* set port					*/
	case FELICA_PPC_CHANGE_ACCESSMODE_COMMAND:		/* change accessmode		*/
		*read_data_ptr &= 0x01;
		break;
	case FELICA_PPC_READ_STATUS_REGISTER_COMMAND:	/* read status				*/
		*read_data_ptr &= 0x0F;
		break;
	case FELICA_PPC_SET_SLAVEADDRESS_COMMAND:		/* set slave address		*/
		*read_data_ptr &= 0x07;
		break;
	default:										/* command error			*/
		MFC_DRV_ERR_LOG("cmd = %d", cmd);
		return FALSE;
	}
	MFC_DRV_DBG_LOG("END read_data = %d", *read_data_ptr);
	
	return TRUE;
}

static int mfd_PD_ppc_i2c_write_no_data(unsigned char cmd)
{

	int		i2c_ret		= I2C_FAILURE;
	
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &cmd,
		},
	};
	
	MFC_DRV_DBG_LOG("START cmd = %d", cmd);
	
	i2c_ret = i2c_transfer(this_client->adapter, msg, 1);
	
	if (i2c_ret <= I2C_FAILURE) {
		MFC_DRV_ERR_LOG("i2c_ret = %d", i2c_ret);
		
		return FALSE;
	} else {
		MFC_DRV_DBG_LOG("END i2c_ret = %d", i2c_ret);
		
		return TRUE;
	}
}

static int cen_open( struct inode *inode, struct file *filp )
{
	MFC_DRV_DBG_LOG("");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	return 0;
}

static int cen_release( struct inode *inode, struct file *filp )
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

int get_cen_value(void)
{
	int result;
	unsigned char ppc_data = 0x00;

	/* read status register	*/
	result = felica_ppc_read_register(FELICA_PPC_READ_STATUS_REGISTER_COMMAND, &ppc_data);
	if (result == FALSE) {
		return -1;
	}

	ppc_data &= FELICA_CONTROL_LOCK_MASK;
	return ppc_data;
}

ssize_t cen_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	int				ret					= FALSE;
	unsigned char lock_status;
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* check NULL_ptr		*/
	if (buf == NULL) {
		MFC_DRV_ERR_LOG("buf_ptr_NULL");
		return -EFAULT;
	}
	
	ret = get_cen_value();
	if (ret == CEN_LOCK_OFF) {
		lock_status = SHMFD_CEN_READ_LOCK_OFF;				/* status : UnLock	*/
	} else if (ret == CEN_LOCK_ON) {
		lock_status = SHMFD_CEN_READ_LOCK_ON;				/* status : Lock	*/
	} else {
		MFC_DRV_ERR_LOG("get_cen_value");
		return -EIO;
	}
	
	if (copy_to_user(buf, &lock_status, sizeof(lock_status))) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}
	
	MFC_DRV_DBG_LOG("END lock_status = %d ,sizeof(lock_status) = %d", lock_status, sizeof(lock_status));
	
	return (sizeof(lock_status));
}

ssize_t cen_write(struct file *file, const char *buf,
				size_t count, loff_t *pos)
{
	int				lock_retern		= FALSE;
	unsigned char	lock_request	= SHMFD_CEN_WRITE_LOCK_OFF;
	char			lock_status;		/* select Lock_status	*/
	int				writemode		= FELICA_WRITE_REGISTER_ACCESS_MODE;
	static 			DEFINE_MUTEX(lock);		/* mutex */
	
	MFC_DRV_DBG_LOG("START");
	
	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
		
	/* mutex_lock start	*/
	mutex_lock(&lock);
	
	/* check NULL_ptr	*/
	if (buf == NULL) {
		mutex_unlock(&lock);
		MFC_DRV_ERR_LOG("buf_ptr_NULL");
		return -EFAULT;
	}
	
	if (copy_from_user(&lock_status, buf, sizeof(lock_status))) {
		mutex_unlock(&lock);
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}
	
	switch (lock_status) {
	case SHMFD_CEN_WRITE_LOCK_ON:							/* FeliCa Lock	*/
		writemode		= FELICA_WRITE_E2PROM_ACCESS_MODE;
		lock_request	= CEN_LOCK_ON;
		break;
	case SHMFD_CEN_WRITE_LOCK_OFF:							/* FeliCa UnLock	*/
		writemode		= FELICA_WRITE_E2PROM_ACCESS_MODE;
		lock_request	= CEN_LOCK_OFF;
		break;
	case SHMFD_CEN_WRITE_LOCK_TEMPOFF:						/* FeliCa TempUnLock	*/
		writemode		= FELICA_WRITE_REGISTER_ACCESS_MODE;
		lock_request	= CEN_LOCK_OFF;
		break;
	default:
		mutex_unlock(&lock);
		MFC_DRV_ERR_LOG("lock_status = %d", lock_status);
		return -EINVAL;
	}
	
	if (writemode == FELICA_WRITE_E2PROM_ACCESS_MODE) {
		lock_retern = felica_ppc_write_E2PROM(FELICA_PPC_SET_PORT_DATA_COMMAND, lock_request);
	} else {
		lock_retern = felica_ppc_write_register(FELICA_PPC_SET_PORT_DATA_COMMAND, lock_request);
	}
	mutex_unlock(&lock);
	
	if (lock_retern == TRUE) {
		MFC_DRV_DBG_LOG("END sizeof(lock_request) = %d", sizeof(lock_request));
		return (sizeof(lock_request));
	} else {
		MFC_DRV_ERR_LOG("lock_retern = %d", lock_retern);
		return -EIO;
	}
}

static long cen_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned char	item_id;
	unsigned char	read_data_ptr;
	unsigned char	write_data_ppc;
	int				write_protect = 0;
	
	struct mfd_cen felica_lock_status;
	
	MFC_DRV_DBG_LOG("START");
	
	if ((cmd != FELICA_PPC_RELOAD) || 
		(cmd != FELICA_PPC_REVERSE_OUTPUT) ) {
		if ((struct mfd_cen __user *)arg == NULL) {
			MFC_DRV_ERR_LOG("arg NULL");
			return -EFAULT;
		}
	}
	
	switch (cmd) {
	/* 1 : read E2PROM					*/
	case (FELICA_PPC_READ_E2PROM):
		MFC_DRV_DBG_LOG("START FELICA_PPC_READ_E2PROM");
		if (copy_from_user(&felica_lock_status, (struct mfd_cen __user *)arg, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_from_user read E2PROM");
			return -EFAULT;
		}
		
		item_id			= felica_lock_status.item_id;
		if (felica_ppc_read_E2PROM(item_id, &read_data_ptr) == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_read_E2PROM");
			return -EIO;
		}
		
		MFC_DRV_DBG_LOG("read_data = 0x%02x read E2PROM", read_data_ptr);
		
		felica_lock_status.data = read_data_ptr;
		
		if (copy_to_user((struct mfd_cen __user *)arg, &felica_lock_status, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_to_user read_E2PROM");
			return -EFAULT;
		}
		break;
	
	/* 2 : read register		*/
	case (FELICA_PPC_READ_REGISTER):
		MFC_DRV_DBG_LOG("START FELICA_PPC_READ_REGISTER");
		if (copy_from_user(&felica_lock_status, (struct mfd_cen __user *)arg, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_from_user read register");
			return -EFAULT;
		}
		
		item_id	= felica_lock_status.item_id;
		
		if (felica_ppc_read_register(item_id, &read_data_ptr) == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_read_register");
			return -EIO;
		}
		
		MFC_DRV_DBG_LOG("read_data = 0x%02x read_register", read_data_ptr);
		
		felica_lock_status.data = read_data_ptr;
		
		if (copy_to_user((struct mfd_cen __user *)arg, &felica_lock_status, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_to_user read_register");
			return -EFAULT;
		}
		break;
	
	/* 3 : write E2PROM					*/
	case (FELICA_PPC_WRITE_E2PROM):	
		
		MFC_DRV_DBG_LOG("START FELICA_PPC_WRITE_E2PROM");
		if (copy_from_user(&felica_lock_status, (struct mfd_cen __user *)arg, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_from_user write_E2PROM");
			return -EFAULT;
		}
		
		item_id			= felica_lock_status.item_id;
		write_data_ppc	= felica_lock_status.data;
		
		if (felica_ppc_write_E2PROM(item_id, write_data_ppc) == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_write_E2PROM");
			return -EIO;
		}
		break;
	
	/* 4 : write register					*/
	case (FELICA_PPC_WRITE_REGISTER):
		
		MFC_DRV_DBG_LOG("START FELICA_PPC_WRITE_REGISTER");
		if (copy_from_user(&felica_lock_status, (struct mfd_cen __user *)arg, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_from_user write_register");
			return -EFAULT;
		}
		
		item_id			= felica_lock_status.item_id;
		write_data_ppc	= felica_lock_status.data;
		
		if (felica_ppc_write_register(item_id, write_data_ppc) == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_write_register");
			return -EIO;
		}
		
		break;
	
	/* 5 : reload						*/
	case (FELICA_PPC_RELOAD):
		
		MFC_DRV_DBG_LOG("START FELICA_PPC_RELOAD");
		
		if (felica_ppc_reload() == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_reload");
			return -EIO;
		}
		
		break;
	
	/* 6 : reverse output						*/
	case (FELICA_PPC_REVERSE_OUTPUT):
		
		MFC_DRV_DBG_LOG("START FELICA_PPC_REVERSE_OUTPUT");
		
		if (felica_ppc_reverse_output() == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_reverse_output");
			return -EIO;
		}
		
		break;
	
	/* 7 : write protect				*/
	case (FELICA_PPC_WRITE_PROTECT):
		
		MFC_DRV_DBG_LOG("START FELICA_PPC_WRITE_PROTECT");
		
		if (copy_from_user(&felica_lock_status, (struct mfd_cen __user *)arg, sizeof(felica_lock_status))) {
			MFC_DRV_ERR_LOG("copy_from_user write_protect");
			return -EFAULT;
		}
		
		write_protect	= felica_lock_status.data;
		
		if (write_protect < FELICA_PPC_WRITE_DISABLE ||
			write_protect > FELICA_PPC_WRITE_ENABLE) {
			MFC_DRV_ERR_LOG("write_protect = %d", write_protect);
			return -EINVAL;
		}
		
		if (felica_ppc_write_protect(write_protect) == FALSE) {
			MFC_DRV_ERR_LOG("felica_ppc_write_protect");
			return -EIO;
		}
		break;
	
	/* command error 					*/
	default:
		MFC_DRV_ERR_LOG("cmd = %d", cmd);
		return -EINVAL;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return (0);
}

const struct file_operations mfc_cen_fileops = {
	.owner	 = THIS_MODULE,
	.open	 = cen_open,
	.release = cen_release,
	.read	 = cen_read,
	.write	 = cen_write,
	.unlocked_ioctl	 = cen_ioctl,
};

static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct cen_data *cen;
	int alloc_ret = 0;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cen = kzalloc(sizeof(struct cen_data), GFP_KERNEL);
	if (!cen) {
		MFC_DRV_ERR_LOG("kzalloc");
		return -ENOMEM;								/* got no mem */
	}
	
	i2c_set_clientdata(client, cen);
	
	this_client = client;
	
	cen->input_dev = input_allocate_device();
	
	alloc_ret = alloc_chrdev_region(&dev , 0 , D_CEN_DEVS, D_CEN_DEV_NAME);
	if (alloc_ret) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region");
		return alloc_ret;
	}
	
	cdev_init(&cen_cdev, &mfc_cen_fileops);
	cen_cdev.owner = THIS_MODULE;
	
	cdev_add(&cen_cdev, dev, D_CEN_DEVS);
	
	device_create(felica_class, NULL, dev, NULL, D_CEN_DEV_NAME);
	
	MFC_DRV_DBG_LOG("END");
	
	return (0);
}

static int cen_remove(struct i2c_client *client)
{
	struct cen_data *cen = i2c_get_clientdata(client);
	
	MFC_DRV_DBG_LOG("START");
	
	input_unregister_device(cen->input_dev);
	
	kfree(cen);
	
	MFC_DRV_DBG_LOG("END");
	
	return (0);
}

static const struct i2c_device_id cen_id[] = {
	{ SH_MFD_CEN, 0 },
	{ }
};

static struct i2c_driver ak6921f_driver =
{
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SH_MFD_CEN,
	},
	.class	 = I2C_CLASS_HWMON,
	.probe	 = cen_probe,
	.id_table = cen_id,
	.remove	 = cen_remove,
};

/*
 * felica_init
 */
static __init int felica_init(void)
{
	int ret;
	
	felica_class = class_create(THIS_MODULE, "felica");
	
	if (IS_ERR(felica_class)) {
		return PTR_ERR(felica_class);
	}
	
	ret = pon_init();
	if( ret < 0 )
		return ret;
	
	ret = int_init();
	if( ret < 0 )
		return ret;
	
	ret = rfs_init();
	if( ret < 0 )
		return ret;
	
	ret = i2c_add_driver(&ak6921f_driver);
	if( ret < 0 )
		return ret;
	
	ret = int_poll_init();
	if( ret < 0 )
		return ret;
	
	ret = rfs_poll_init();
	if( ret < 0 )
		return ret;
	
	ret = rws_init();
	if( ret < 0 )
		return ret;

#ifdef CONFIG_SHSNFC
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION
	if( gpio_get_value(D_MVDD_GPIO_NO) ) {
#endif
#endif
	/* reload CEN */
	felica_ppc_reload();
#ifdef CONFIG_SHSNFC
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION
	}
#endif
#endif

	return 0;
}

/*
 * felica_exit
 */
static void __exit felica_exit(void)
{
	class_destroy( felica_class );
	
	pon_exit();
	
	int_exit();
	
	rfs_exit();
	
	i2c_del_driver(&ak6921f_driver);
	
	int_poll_exit();
	
	rfs_poll_exit();
	
	rws_exit();
	
	return;
}


MODULE_LICENSE("GPL v2");

module_init(felica_init);
module_exit(felica_exit);

#endif /* CONFIG_SHFELICA */

