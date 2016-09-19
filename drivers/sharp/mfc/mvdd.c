/* drivers/sharp/mfc/mvdd.c (NFC driver)
 *
 * Copyright (C) 2012-2013 SHARP CORPORATION
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
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
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
#include <asm/uaccess.h>
#include <sharp/snfc_en.h>
#include "mfc.h"

#define D_MVDD_ENABLE 			0x01
#define D_MVDD_DISABLE 			0x00

#ifdef CONFIG_SHSNFC_BATTERY_FIXATION
#define D_MVDD_DEVS 			(1)
#define D_MVDD_DEV_LOW 			(0)
#define D_MVDD_DEV_HIGH 		(1)
#define D_MVDD_DEV_NAME 		("snfc_en")
#define D_MVDD_DELAY_MSEC 		(0)

/* for snfc_output_disable */
#define D_INTU_PULL_DN			(0)
#define D_INTU_PULL_UP			(1)

/* for vfel-rst control */
#define D_VFEL_DEV_LOW 			(0)
#define D_VFEL_DEV_HIGH 		(1)

#define D_VFEL_HIGH_SLEEP_USEC	20*1000+500*1000
#define D_VFEL_LOW_SLEEP_USEC	100*1000+100*1000

//#define D_MVDD_REF_INT
#ifdef D_MVDD_REF_INT
#define D_MVDD_DEV_FIX	 		(2)
#define D_INT_DEV_HIGH			(1)

#endif /* D_MVDD_REF_INT */

/*
 * prototype
 */
static __init int mvdd_init(void);
static void __exit mvdd_exit(void);

/*
 * global variable
 */
static struct poll_data g_mvdd_data;
static struct poll_data *g_mvdd_d = &g_mvdd_data;

static struct class *snfc_en_class = NULL;
static struct cdev snfc_en_cdev;

#ifdef CONFIG_SHSNFC
static struct qpnp_pin_cfg intu_gpio_cfg = {
	.mode             = QPNP_PIN_MODE_DIG_IN,
	.output_type      = QPNP_PIN_OUT_BUF_CMOS,
	.invert           = QPNP_PIN_INVERT_DISABLE,
	.pull             = QPNP_PIN_GPIO_PULL_UP_30,
	.vin_sel          = QPNP_PIN_VIN2,
	.out_strength     = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel          = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en        = QPNP_PIN_MASTER_ENABLE,
	.aout_ref         = 0,
	.ain_route        = 0,
	.cs_out           = 0,
};
#endif /* CONFIG_SHSNFC */

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.

unsigned int snfc_available(void)
{
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION

	unsigned int ret;
	ret = (g_mvdd_d->device_status != D_MVDD_DEV_LOW) ? D_MVDD_ENABLE : D_MVDD_DISABLE;

	return ret;

#else	//CONFIG_SHSNFC_BATTERY_FIXATION.

	return D_MVDD_ENABLE;

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.
}


unsigned int snfc_available_wake_up(void)
{
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION

	unsigned int ret;
#ifdef D_MVDD_REF_INT
	ret = ((g_mvdd_d->device_status == D_MVDD_DEV_FIX) || (gpio_get_value(D_MVDD_GPIO_NO) == D_MVDD_DEV_HIGH)) ? D_MVDD_ENABLE : D_MVDD_DISABLE;
#else
	ret = (gpio_get_value(D_MVDD_GPIO_NO) == D_MVDD_DEV_HIGH) ? D_MVDD_ENABLE : D_MVDD_DISABLE;
#endif /* D_MVDD_REF_INT */

	return ret;

#else	//CONFIG_SHSNFC_BATTERY_FIXATION.

	return D_MVDD_ENABLE;

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.
}

#ifdef CONFIG_SHSNFC_BATTERY_FIXATION
#ifdef CONFIG_SHSNFC
static void set_intu_gpio_pull(int pull_type)
{
	int ret;
	
	intu_gpio_cfg.pull = (pull_type == D_INTU_PULL_UP) ? QPNP_PIN_GPIO_PULL_UP_30 : QPNP_PIN_GPIO_PULL_DN;
	ret = qpnp_pin_config(D_INTU_GPIO_NO, &intu_gpio_cfg);
	if(ret) {
		MFC_DRV_ERR_LOG("qpnp_pin_config ret = %d", ret);
	}
}
#endif /* CONFIG_SHSNFC */


static void snfc_gpio_free(unsigned gpio)
{
	int ret;
	
	MFC_DRV_DBG_LOG("START gpio = %u", gpio);
	
	gpio_free(gpio);
	ret = gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret) {
		MFC_DRV_ERR_LOG("gpio_tlmm_config ret = %d", ret);
	}
}


static void snfc_output_disable(void)
{
	MFC_DRV_DBG_LOG("START");
	
	/* UART-TX */
	snfc_gpio_free(D_UART_TX_GPIO_NO);

	/* UART-RX */
	snfc_gpio_free(D_UART_RX_GPIO_NO);

	/* RFS */
	snfc_gpio_free(D_RFS_GPIO_NO);

	/* INT */
	snfc_gpio_free(D_INT_GPIO_NO);

#ifdef CONFIG_SHSNFC
	/* INTU */
	set_intu_gpio_pull(D_INTU_PULL_DN);

	/* HSEL */
	gpio_set_value(D_HSEL_GPIO_NO, D_MVDD_DEV_LOW);
#endif /* CONFIG_SHSNFC */
	
	/* PON */
	gpio_set_value(D_PON_GPIO_NO, D_MVDD_DEV_LOW);

	MFC_DRV_DBG_LOG("END");
}

static void snfc_output_enable(void)
{
	int ret;
	
	MFC_DRV_DBG_LOG("START");

	/* UART-TX */
	ret = gpio_request(D_UART_TX_GPIO_NO,"UART_TX_GPIO request");
	if(ret) {
		MFC_DRV_ERR_LOG("UART_TX_GPIO ret = %d", ret);
	}
	
	/* UART-RX */
	ret = gpio_request(D_UART_RX_GPIO_NO,"UART_RX_GPIO request");
	if(ret) {
		MFC_DRV_ERR_LOG("UART_RX_GPIO ret = %d", ret);
	}

	/* RFS */
	ret = gpio_request(D_RFS_GPIO_NO,"RFS_GPIO request");
	if(ret) {
		MFC_DRV_ERR_LOG("RFS_GPIO ret = %d", ret);
	}

	/* INT */
	ret = gpio_request(D_INT_GPIO_NO,"INT_GPIO request");
	if(ret) {
		MFC_DRV_ERR_LOG("INT_GPIO ret = %d", ret);
	}

#ifdef CONFIG_SHSNFC
	/* INTU */
	set_intu_gpio_pull(D_INTU_PULL_UP);
#endif /* CONFIG_SHSNFC */

	MFC_DRV_DBG_LOG("END");
}

static void mvdd_work_func(struct work_struct *work)
{
	struct poll_data *mvdd_d = g_mvdd_d;
	int read_value = 0, old_value = 0;
	unsigned long irqflag = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	old_value = mvdd_d->device_status;
	read_value = gpio_get_value(D_MVDD_GPIO_NO);
	
	MFC_DRV_DBG_LOG("read_value = %d, old_value = %d", read_value, old_value);

	/* read error */
	if (read_value < 0) {
		mvdd_d->read_error = read_value;
	} else if (read_value != old_value) {

		if (read_value == D_MVDD_DEV_LOW) {
			snfc_output_disable();
			mvdd_d->device_status = read_value;
			mvdd_d->read_error = 0;
			irqflag = IRQF_TRIGGER_HIGH;
			
		} else {
			msleep(100);
			
			read_value = gpio_get_value(D_MVDD_GPIO_NO);
			if(read_value == D_MVDD_DEV_HIGH) {
				snfc_output_enable();
			}
			mvdd_d->device_status = read_value;
			mvdd_d->read_error = 0;
			irqflag = (read_value == D_MVDD_DEV_HIGH) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
		}

		irqflag |= IRQF_SHARED;
		if (irq_set_irq_type(gpio_to_irq(D_MVDD_GPIO_NO), irqflag))
			MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);

	}

	/* enable irq handler */
	enable_irq(gpio_to_irq(D_MVDD_GPIO_NO));

	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d, mvdd_d->read_error = %d"
					, read_value, old_value, mvdd_d->read_error);
}


static irqreturn_t mvdd_irq_handler(int irq, void *dev_id)
{	
	MFC_DRV_DBG_LOG("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(D_MVDD_GPIO_NO));
	/* set workqueue */
	schedule_delayed_work(&g_mvdd_d->work, msecs_to_jiffies(D_MVDD_DELAY_MSEC));
	
	MFC_DRV_DBG_LOG("END");
	
	return IRQ_HANDLED;
}

void snfc_chip_reset(void)
{
	MFC_DRV_ERR_LOG("abnormal state");
	
	/* vfel-rst[H] = mvdd[L] */
	gpio_set_value(D_VFEL_GPIO_NO, D_VFEL_DEV_HIGH);

	usleep(D_VFEL_HIGH_SLEEP_USEC);
	
	/* vfel-rst[L] = mvdd[H] */
	gpio_set_value(D_VFEL_GPIO_NO, D_VFEL_DEV_LOW);

	usleep(D_VFEL_LOW_SLEEP_USEC);

}

static int snfc_en_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static int snfc_en_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

ssize_t snfc_en_write(struct file *file, const char __user *data,
		       size_t len, loff_t *ppos)
{
	MFC_DRV_DBG_LOG("");
	return len;
}

static long snfc_en_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	MFC_DRV_DBG_LOG("START cmd = %u", cmd);

	switch (cmd) {
	case SHSNFC_EN_REQ_CHIPRESET:
		snfc_chip_reset();
		break;
	default:
		MFC_DRV_ERR_LOG("cmd unhandled");
		return -EINVAL;
	}

	MFC_DRV_DBG_LOG("END");

	return 0;
}


static const struct file_operations snfc_en_fileops = {
	.owner          = THIS_MODULE,
	.open           = snfc_en_open,
	.release        = snfc_en_release,
	.write          = snfc_en_write,
	.unlocked_ioctl = snfc_en_ioctl,
};


/*
 * mvdd_init
 */
static __init int mvdd_init(void)
{
	int ret = 0;
	struct poll_data *mvdd_d = g_mvdd_d;
	unsigned long irqflag = 0;

	struct device *class_dev;
	dev_t dev;

	MFC_DRV_DBG_LOG("START");

	snfc_en_class = class_create(THIS_MODULE, "mvdd");
	if (IS_ERR(snfc_en_class)) {
		return PTR_ERR(snfc_en_class);
	}

	dev = MKDEV(MISC_MAJOR, 0);

	ret = alloc_chrdev_region(&dev, 0, D_MVDD_DEVS, D_MVDD_DEV_NAME);
	if (ret) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region ret = %d", ret);
		return ret;
	}

	cdev_init(&snfc_en_cdev, &snfc_en_fileops);
	snfc_en_cdev.owner = THIS_MODULE;

	ret = cdev_add(&snfc_en_cdev, dev, D_MVDD_DEVS);
	if (ret) {
		unregister_chrdev_region(dev, D_MVDD_DEVS);
		MFC_DRV_ERR_LOG("cdev_add ret = %d", ret);
		return ret;
	}

	class_dev = device_create(snfc_en_class, NULL, dev, NULL, D_MVDD_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&snfc_en_cdev);
		unregister_chrdev_region(dev, D_MVDD_DEVS);
		ret = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create ret = %d", ret);
		return ret;
	}

	/* initialize poll_data */
	memset(g_mvdd_d, 0x00, sizeof(struct poll_data));
	/* initialize workqueue */
	INIT_DELAYED_WORK(&g_mvdd_d->work, mvdd_work_func);
	/* initialize waitqueue */
	init_waitqueue_head(&g_mvdd_d->read_wait);

	ret = gpio_get_value(D_MVDD_GPIO_NO);
	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -EIO;
	}

	MFC_DRV_DBG_LOG("MVDD gpio_get_value[%u] = %d", D_MVDD_GPIO_NO, ret);

	mvdd_d->device_status = ret;

#ifdef D_MVDD_REF_INT
	if (mvdd_d->device_status == D_MVDD_DEV_LOW) {
		ret = gpio_get_value(D_INT_GPIO_NO);
		if (ret == D_INT_DEV_HIGH) {
			mvdd_d->device_status = D_MVDD_DEV_FIX;
		}
	}
#endif /* D_MVDD_REF_INT */

	irqflag = (mvdd_d->device_status == D_MVDD_DEV_HIGH) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	irqflag |= IRQF_SHARED;

	if (request_irq(gpio_to_irq(D_MVDD_GPIO_NO),
	                mvdd_irq_handler,
	                irqflag,
	                D_MVDD_DEV_NAME,
	                (void*)mvdd_d)) {

		MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
		
	if(enable_irq_wake(gpio_to_irq(D_MVDD_GPIO_NO))) {
		MFC_DRV_ERR_LOG("enable_irq_wake");
		free_irq(gpio_to_irq(D_MVDD_GPIO_NO), (void *)mvdd_d);
		return -EIO;
	}
	mvdd_d->irq_handler_done = 0;
	mvdd_d->open_flag = 1;

#ifdef CONFIG_SHSNFC
	ret = gpio_request(D_INTU_GPIO_NO, "mvdd_intu");
	if(ret) {
		MFC_DRV_ERR_LOG("gpio_request ret = %d", ret);
	}
#endif /* CONFIG_SHSNFC */

	if(mvdd_d->device_status != D_MVDD_DEV_LOW) {
		snfc_output_enable();
	}

	MFC_DRV_DBG_LOG("END");
	
	return 0;
}


/*
 * mvdd_exit
 */
static void __exit mvdd_exit(void)
{
	struct poll_data *mvdd_d = g_mvdd_d;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");
	
	/* clear workqueue */
	cancel_delayed_work(&mvdd_d->work);
	
	if(mvdd_d->open_flag) {
		if(disable_irq_wake(gpio_to_irq(D_MVDD_GPIO_NO)))
			MFC_DRV_ERR_LOG("disable_irq_wake");
		
		free_irq(gpio_to_irq(D_MVDD_GPIO_NO), (void *)mvdd_d);
	}
	
#ifdef CONFIG_SHSNFC
	gpio_free(D_INTU_GPIO_NO);
#endif /* CONFIG_SHSNFC */
	
	cdev_del(&snfc_en_cdev);
	unregister_chrdev_region(dev, D_MVDD_DEVS);
	class_destroy(snfc_en_class);

	MFC_DRV_DBG_LOG("END");
}

MODULE_LICENSE("GPL v2");

module_init(mvdd_init);
module_exit(mvdd_exit);

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.

#endif	//CONFIG_SHFELICA

