/* drivers/sharp/mfc/mfcuart.c (MFC UART Wrapper)
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

#ifdef CONFIG_SHFELICA

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <asm/termios.h>
#include <asm/uaccess.h>
#include "mfc.h"

/* Common UART */
#define D_UART_DEV_FILE_PATH	("/dev/ttyHSL1")

/* FeliCa UART */
#define D_FELICA_UART_DEVS		(1)
#define D_FELICA_UART_DEV_NAME	("felica")

#ifdef CONFIG_SHSNFC
/* NFC UART */
#define D_NFC_UART_DEVS			(1)
#define D_NFC_UART_DEV_NAME		("snfc_uart")
#endif /* CONFIG_SHSNFC */

/*
 * prototype
 */
static __init int mfcuart_init(void);
static void __exit mfcuart_exit(void);

/*
 * global variable
 */
static struct class *mfcuart_class = NULL;

static struct cdev felica_uart_cdev;
#ifdef CONFIG_SHSNFC
static struct cdev nfc_uart_cdev;

static int g_felica_open_count = 0;
#endif /* CONFIG_SHSNFC */

/*
 * function_common
 */
static inline struct tty_struct *file_tty(struct file *filp)
{
	return ((struct tty_file_private *)filp->private_data)->tty;
}

#ifdef CONFIG_SHSNFC
static void uart_clear_buffer(struct file *filp)
{
	struct file *tty;

	MFC_DRV_DBG_LOG("ENTER");

	tty = (struct file *)filp->private_data;
	if (tty) {
		tty_ldisc_flush(file_tty(tty));
		tty_driver_flush_buffer(file_tty(tty));
	}
}
#endif /* CONFIG_SHSNFC */

/*
 * function_common_uart
 */
static ssize_t uart_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	struct file *tty;
	ssize_t result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->read) {
		MFC_DRV_DBG_LOG("tty read len = %d", len);
		result = tty->f_op->read(tty, buf, len, ppos);
	}

	MFC_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static ssize_t uart_write(struct file *filp, const char __user *data, size_t len, loff_t *ppos)
{
	struct file *tty;
	ssize_t result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->write) {
		MFC_DRV_DBG_LOG("tty write len = %d", len);
		result = tty->f_op->write(tty, data, len, ppos);
	}

	MFC_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static unsigned int uart_poll(struct file *filp, poll_table *wait)
{
	struct file *tty;
	unsigned int result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->poll) {
		MFC_DRV_DBG_LOG("tty poll");
		result = tty->f_op->poll(tty, wait);
	}

	MFC_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static long uart_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct file *tty;
	long result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->unlocked_ioctl) {
		MFC_DRV_DBG_LOG("tty unlocked_ioctl cmd = %d", cmd);
		result = tty->f_op->unlocked_ioctl(tty, cmd, arg);
	}

	MFC_DRV_DBG_LOG("END result = %ld", result);

	return result;
}

#ifdef CONFIG_COMPAT
static long uart_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct file *tty;
	long result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->compat_ioctl) {
		MFC_DRV_DBG_LOG("tty compat_ioctl cmd = %d", cmd);
		result = tty->f_op->compat_ioctl(tty, cmd, arg);
	}

	MFC_DRV_DBG_LOG("END result = %ld", result);

	return result;
}
#else
#define uart_compat_ioctl NULL
#endif

static int uart_open(struct inode *inode, struct file *filp)
{
	int result;
	struct file *tty;
	struct termios termios;

	MFC_DRV_DBG_LOG("START");

	if (!(snfc_available())) {
		MFC_DRV_ERR_LOG("snfc_available");
		return -EIO;
	}
	
	/* open tty driver */
	tty = filp_open(D_UART_DEV_FILE_PATH, filp->f_flags, 0);
	if (IS_ERR(tty)) {
		filp->private_data = NULL;
		result = PTR_ERR(tty);
		MFC_DRV_ERR_LOG("filp_open result = %d", result);
		return result;
	}

	if (tty->f_op->unlocked_ioctl) {
		mm_segment_t oldfs = get_fs();
		set_fs(KERNEL_DS);

		/* set UART speed */
		tty->f_op->unlocked_ioctl(tty, TCGETS, (unsigned long)&termios);

		termios.c_cflag = (termios.c_cflag & ~CBAUD) | (B460800 & CBAUD);
		termios.c_cflag &= ~(PARENB | CSTOPB | CSIZE | PARODD);
		termios.c_cflag |= (CLOCAL | CREAD | CS8);
		termios.c_lflag &= ~(ICANON | IEXTEN | ISIG | ECHO);
		termios.c_oflag &= ~(OPOST | ONLCR);
		termios.c_iflag &= ~(ISTRIP | IUCLC | IGNCR | ICRNL | INLCR | IXON | PARMRK | BRKINT | INPCK);
		termios.c_iflag |= (IGNPAR | IGNBRK);
		termios.c_cc[VMIN] = 0;
		termios.c_cc[VTIME] = 0;

		tty->f_op->unlocked_ioctl(tty, TCSETS, (unsigned long)&termios);

		filp->private_data = tty;
		result = 0;

		set_fs(oldfs);
	} else  {
		filp_close(tty, 0);
		filp->private_data = NULL;
		result = -ENOSYS;
	}

	MFC_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static int uart_release(struct inode *inode, struct file *filp)
{
	struct file *tty;

	MFC_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty) {
		MFC_DRV_DBG_LOG("tty close");
		filp_close(tty, 0);
		filp->private_data = NULL;
	}

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static int uart_fasync(int fd, struct file *filp, int on)
{
	struct file *tty;
	int result = -ENOSYS;

	MFC_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->fasync) {
		MFC_DRV_DBG_LOG("tty fasync");
		result = tty->f_op->fasync(fd, tty, on);
	}

	MFC_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static int uart_fsync(struct file *file, loff_t start, loff_t end, int datasync)
{
	MFC_DRV_DBG_LOG("ENTER");
	return 0;
}

/*
 * function_felica_uart
 */
static ssize_t felica_uart_write(struct file *filp, const char __user *data, size_t len, loff_t *ppos)
{
	ssize_t result;
	struct file *tty;

	result = uart_write(filp, data, len, ppos);
	if (result > 0) {
		MFC_DRV_DBG_LOG("tty_wait_until_sent");
		tty = (struct file *)filp->private_data;
		if (tty) {
			tty_wait_until_sent(file_tty(tty), 0);
		}
	}

	return result;
}

static int felica_uart_open(struct inode *inode, struct file *filp)
{
	int result;
#ifdef CONFIG_SHSNFC
	result = succ_handle_event(SUCC_EVENT_START_FELICA, 0);
	if (result != SUCC_RETVAL_OK) {
		return -EBUSY;
	}
#endif /* CONFIG_SHSNFC */
	result = uart_open(inode, filp);
#ifdef CONFIG_SHSNFC
	if (result == 0) {
		if (++g_felica_open_count == 1) {
			uart_clear_buffer(filp);
		}
	} else {
		if (g_felica_open_count == 0) {
			succ_handle_event(SUCC_EVENT_END_FELICA, 0);
		}
	}
#endif /* CONFIG_SHSNFC */
	return result;
}

static int felica_uart_release(struct inode *inode, struct file *filp)
{
	int result;
#ifdef CONFIG_SHSNFC
	if (g_felica_open_count == 1) {
		uart_clear_buffer(filp);
	}
#endif /* CONFIG_SHSNFC */
	result = uart_release(inode, filp);
#ifdef CONFIG_SHSNFC
	if (result == 0) {
		if ((g_felica_open_count > 0) && (--g_felica_open_count == 0)) {
			succ_handle_event(SUCC_EVENT_END_FELICA, 0);
		}
	}
#endif /* CONFIG_SHSNFC */
	return result;
}

static const struct file_operations felica_uart_fileops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.read			= uart_read,
	.write			= felica_uart_write,
	.poll			= uart_poll,
	.unlocked_ioctl	= uart_ioctl,
	.compat_ioctl	= uart_compat_ioctl,
	.open			= felica_uart_open,
	.release		= felica_uart_release,
	.fasync			= uart_fasync,
	.fsync			= uart_fsync,
};

static int felica_uart_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev , 0 , D_FELICA_UART_DEVS, D_FELICA_UART_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&felica_uart_cdev, &felica_uart_fileops);
	felica_uart_cdev.owner = THIS_MODULE;

	result = cdev_add(&felica_uart_cdev, dev, D_FELICA_UART_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_FELICA_UART_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(mfcuart_class, NULL, dev, NULL, D_FELICA_UART_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felica_uart_cdev);
		unregister_chrdev_region(dev, D_FELICA_UART_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void felica_uart_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	cdev_del(&felica_uart_cdev);
	unregister_chrdev_region(dev, D_FELICA_UART_DEVS);

	MFC_DRV_DBG_LOG("END");
}

#ifdef CONFIG_SHSNFC
/*
 * function_nfc_uart
 */
static const struct file_operations nfc_uart_fileops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.read			= uart_read,
	.write			= uart_write,
	.poll			= uart_poll,
	.unlocked_ioctl	= uart_ioctl,
	.compat_ioctl	= uart_compat_ioctl,
	.open			= uart_open,
	.release		= uart_release,
	.fasync			= uart_fasync,
	.fsync			= uart_fsync,
};

static int nfc_uart_init(void)
{
	int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev , 0 , D_NFC_UART_DEVS, D_NFC_UART_DEV_NAME);
	if (result) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&nfc_uart_cdev, &nfc_uart_fileops);
	nfc_uart_cdev.owner = THIS_MODULE;

	result = cdev_add(&nfc_uart_cdev, dev, D_NFC_UART_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_NFC_UART_DEVS);
		MFC_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(mfcuart_class, NULL, dev, NULL, D_NFC_UART_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&nfc_uart_cdev);
		unregister_chrdev_region(dev, D_NFC_UART_DEVS);
		result = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFC_DRV_DBG_LOG("END");

	return result;
}

static void nfc_uart_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFC_DRV_DBG_LOG("START");

	cdev_del(&nfc_uart_cdev);
	unregister_chrdev_region(dev, D_NFC_UART_DEVS);

	MFC_DRV_DBG_LOG("END");
}
#endif /* CONFIG_SHSNFC */

/*
 * mfcuart_init
 */
static __init int mfcuart_init(void)
{
	int ret;

	mfcuart_class = class_create(THIS_MODULE, "mfcuart");
	if (IS_ERR(mfcuart_class)) {
		return PTR_ERR(mfcuart_class);
	}

	ret = felica_uart_init();
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SHSNFC
	ret = nfc_uart_init();
	if (ret < 0) {
		return ret;
	}
#endif /* CONFIG_SHSNFC */

	return 0;
}

/*
 * mfcuart_exit
 */
static void __exit mfcuart_exit(void)
{
	class_destroy(mfcuart_class);

	felica_uart_exit();

#ifdef CONFIG_SHSNFC
	nfc_uart_exit();
#endif /* CONFIG_SHSNFC */
}

MODULE_LICENSE("GPL v2");

module_init(mfcuart_init);
module_exit(mfcuart_exit);

#endif /* CONFIG_SHFELICA */

