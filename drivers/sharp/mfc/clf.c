/* drivers/sharp/mfc/clf.c (NFC/FeliCa driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

/***************header***************/
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>

#include <sharp/felica_rws.h>
#include "mfc.h"

static struct class *clf_class = NULL;
static struct cdev rws_cdev;
unsigned int mfc_rws_sts = 0;

/*
 * Definition
*/

/* CLF RWS */
#define D_CLF_RWS_DEVS			(1)
#define D_CLF_RWS_DEV_NAME		("clf_rws")


/*
 * function_rws
 */
static ssize_t rws_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	char on[2];
	
	MFC_DRV_DBG_LOG("START");
	
	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (mfc_rws_sts == D_RWS_RW_DISABLE)
		on[0] = SHMFD_RWS_RW_DISABLE;
	else
		on[0] = SHMFD_RWS_RW_ENABLE;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	
	MFC_DRV_DBG_LOG("END on = %d, len = %d", on[0], len);
	
	return len;
}

static ssize_t rws_write(struct file *file, const char __user *data,
		       size_t len, loff_t *ppos)
{
	char on;
	
	MFC_DRV_DBG_LOG("START");
	
	/* length check */
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}
	
	if (on == SHMFD_RWS_RW_ENABLE)
		mfc_rws_sts = D_RWS_RW_ENABLE;
	else if (on == SHMFD_RWS_RW_DISABLE)
		mfc_rws_sts = D_RWS_RW_DISABLE;
	else {
		MFC_DRV_ERR_LOG("on = %d", on);
		return -EFAULT;
	}

#ifdef CONFIG_SHSNFC
	notify_nfc_avalable_change(NFC_AVAILABLE_RWS, (mfc_rws_sts == D_RWS_RW_ENABLE));
#endif /* CONFIG_SHSNFC */

	MFC_DRV_DBG_LOG("END on = %d mfc_rws_sts = %d", on, mfc_rws_sts);
	
	return len;
}

static int rws_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static int rws_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}


/* clf_rws */
const struct file_operations mfc_rws_fileops = {
	.owner   = THIS_MODULE,
	.write   = rws_write,
	.read    = rws_read,
	.open    = rws_open,
	.release = rws_release,
};

static int rws_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_CLF_RWS_DEVS, D_CLF_RWS_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rws_cdev, &mfc_rws_fileops);
	rws_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rws_cdev, dev, D_CLF_RWS_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_CLF_RWS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(clf_class, NULL, dev, NULL, D_CLF_RWS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rws_cdev);
		unregister_chrdev_region(dev, D_CLF_RWS_DEVS);
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
	unregister_chrdev_region(dev, D_CLF_RWS_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/* clf_init */
static __init int clf_init(void)
{
	int ret;
	
	clf_class = class_create(THIS_MODULE, "clf");
	
	if (IS_ERR(clf_class)) {
		return PTR_ERR(clf_class);
	}
	
	ret = rws_init();
	if( ret < 0 )
		return ret;

	return 0;
}

static void __exit clf_exit(void)
{
	class_destroy( clf_class );
	
	rws_exit();
	
	return;
}


MODULE_LICENSE("GPL v2");

module_init(clf_init);
module_exit(clf_exit);

