/* drivers/usb/gadget/f_sh_msc.c
 * 
 * Gadget Driver for Android MSC
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 * Copyright (C) 2013 SHARP CORPORATION
 *
 * This code also borrows from f_adb.c, which is
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#define D_MSC_TRANSPORT_WRITE_WAIT_COMP
#define D_MSC_TRANSPORT_READ_REQ_ON_TIME
#define D_MSC_STORE_MOUNT_FILE_PATH

#define BULK_RX_BUFFER_SIZE		512
#define BULK_TX_BUFFER_SIZE		4096

#ifdef D_MSC_STORE_MOUNT_FILE_PATH
	#define D_MSC_MOUNT_PATH_LEHGTH_MAX	(128)
#endif /* D_MSC_STORE_FILE_PATH */

/* number of rx and tx requests to allocate */
#define MSC_RX_REQ_MAX			32
#define TX_REQ_MAX			4

static const char msc_shortname[] = "msc_transport";

#define D_MSC_FUNCTION_NAME		"mass_storage"

#define USB_MSC_IOC_MAGIC 0xFF

#define USB_MSC_FUNC_IOC_SET_STALL	_IOW(USB_MSC_IOC_MAGIC, 0x20, int)
#define USB_MSC_FUNC_IOC_GET_MOUNT_STS	_IOW(USB_MSC_IOC_MAGIC, 0x21, int)
#define USB_MSC_FUNC_IOC_GET_VENDOR_ID	_IOW(USB_MSC_IOC_MAGIC, 0x22, int)
#define USB_MSC_FUNC_IOC_GET_PRODUCT_ID	_IOW(USB_MSC_IOC_MAGIC, 0x23, int)
#ifdef D_MSC_STORE_MOUNT_FILE_PATH
#define USB_MSC_FUNC_IOC_GET_MOUNT_PATH	_IOR(USB_MSC_IOC_MAGIC, 0x24, unsigned char*)
#endif /* D_MSC_STORE_FILE_PATH */

/* MSC setup class requests */
#define USB_MSC_GET_MAX_LUN_REQUEST	0xFE
#define USB_MSC_RESET_REQUEST		0xFF

#define	USB_MSC_NTY_ONLINE_STATE	(1)
#define	USB_MSC_NTY_OFFLINE_STATE	(2)
#define USB_MSC_NTY_MOUNT_ENABLE	(3)
#define USB_MSC_NTY_MOUNT_DISABLE	(4)
#define	USB_MSC_NTY_RESET_STATE		(5)

#define	USB_CTRL_NTY_MAX_STR_LEN	(48)
#define USB_MSC_VENDOR_ID_LEN		(8)
#define USB_MSC_VENDOR_ID_INDEX		(0)
#define USB_MSC_PRODUCT_ID_LEN		(16)
#define USB_MSC_PRODUCT_ID_INDEX	(8)
#define USB_MSC_INQUIRY_STRING		"SHARP   Mobile  microSD "
#define USB_MSC_INQUIRY_STRING_LEN	(8 + 16 + 1)
#define	USB_ONLINE_STATE_CHANGE_STR	"online"
#define	USB_OFFLINE_STATE_CHANGE_STR	"offline"
#define	USB_MOUNT_ENABLE_STR		"mount"
#define	USB_MOUNT_DISABLE_STR		"umount"
#define	USB_RESET_STATE_STR		"reset"

#define USB_BULK_RESET_REQUEST		0xff
#define USB_BULK_GET_MAX_LUN_REQUEST	0xfe
/* USB protocol value = the transport method */
#define USB_PR_BULK			0x50	// Bulk-only

/* USB subclass value = the protocol encapsulation */
#define USB_SC_SCSI			0x06	// Transparent SCSI

/* String IDs */
#define MSC_STRING_INDEX		0

/* MOUNT_FILE */
struct lun {
	struct device	dev;
};

struct msc_dev {
	struct usb_function		function;
	struct usb_composite_dev	*cdev;
	spinlock_t lock;

	struct usb_ep			*ep_in;
	struct usb_ep			*ep_out;

	atomic_t			online;
	atomic_t			error;

	atomic_t			read_excl;
	atomic_t			write_excl;
	atomic_t			open_excl;

	struct list_head		tx_idle;
	struct list_head		rx_idle;
	struct list_head		rx_done;

	wait_queue_head_t		read_wq;
	wait_queue_head_t		write_wq;

#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	volatile unsigned long		sleep_flag;
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */

	/* the request we're currently reading from */
	struct usb_request		*read_req;
	unsigned char			*read_buf;
	unsigned			read_count;

	/* for msc_control */
	atomic_t			open_ctrl_excl;
	atomic_t			ctrl_read_excl;
	wait_queue_head_t		ctrl_read_wq;
	unsigned char			ctrl_set_size;
	unsigned char			*ctrl_read_buf;

	struct lun			*lun;	/* MOUNT_FILE */

	unsigned int			mount_enb;

#ifdef D_MSC_STORE_MOUNT_FILE_PATH
	unsigned char			mount_path[D_MSC_MOUNT_PATH_LEHGTH_MAX];
#endif /* D_MSC_STORE_FILE_PATH */

	/*
	 * Vendor (8 chars), product (16 chars) and NUL byte
	 */
	char				msc_inquiry_string[USB_MSC_INQUIRY_STRING_LEN];

	struct workqueue_struct		*wq;
	struct work_struct		suspend_work;
};

static struct usb_interface_descriptor msc_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass     = USB_SC_SCSI,
	.bInterfaceProtocol     = USB_PR_BULK,
	/* .iInterface = DYNAMIC */
};

static struct usb_endpoint_descriptor msc_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor msc_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor msc_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor msc_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_msc_descs[] = {
	(struct usb_descriptor_header *) &msc_interface_desc,
	(struct usb_descriptor_header *) &msc_fullspeed_in_desc,
	(struct usb_descriptor_header *) &msc_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_msc_descs[] = {
	(struct usb_descriptor_header *) &msc_interface_desc,
	(struct usb_descriptor_header *) &msc_highspeed_in_desc,
	(struct usb_descriptor_header *) &msc_highspeed_out_desc,
	NULL,
};


/* string descriptors: */
static struct usb_string msc_string_defs[] = {
	/* msc_iInterface declared in sh_string.c */
	[0].s = msc_iInterface,
	{  } /* end of list */
};

static struct usb_gadget_strings msc_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		msc_string_defs,
};

static struct usb_gadget_strings *msc_strings[] = {
	&msc_string_table,
	NULL,
};

/* Module */
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");


/* temporary variable used between msc_open() and msc_gadget_bind() */
static struct msc_dev *_msc_dev;

static inline struct msc_dev *msc_func_to_dev(struct usb_function *f)
{
	return container_of(f, struct msc_dev, function);
}

static inline int usb_ept_get_max_packet(struct usb_ep *ep) 
{
	return ep->maxpacket;
}

static struct usb_request *msc_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void msc_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void msc_req_put(struct msc_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *msc_req_get(struct msc_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void msc_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct msc_dev *dev = _msc_dev;

	if (req->status != 0)
		atomic_set(&dev->error, 1);

	msc_req_put(dev, &dev->tx_idle, req);

#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	dev->sleep_flag = 1;
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */

	wake_up(&dev->write_wq);
}

static void msc_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct msc_dev *dev = _msc_dev;
	if (req->status != 0) {
		atomic_set(&dev->error, 1);
		msc_req_put(dev, &dev->rx_idle, req);
	} else {
		msc_req_put(dev, &dev->rx_done, req);
	}

	wake_up(&dev->read_wq);
}

static int msc_create_bulk_endpoints(struct msc_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "msc_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	dev->ep_in = ep;
	dev->ep_in->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for msc ep_out got %s\n", ep->name);
	dev->ep_out = ep;
	dev->ep_out->driver_data = cdev;

	/* now allocate requests for our endpoints */
	for (i = 0; i < MSC_RX_REQ_MAX; i++) {
		req = msc_request_new(dev->ep_out, BULK_RX_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = msc_complete_out;
		msc_req_put(dev, &dev->rx_idle, req);
	}

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = msc_request_new(dev->ep_in, BULK_TX_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = msc_complete_in;
		msc_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR "msc_bind() could not allocate requests\n");
	return -1;
}

static ssize_t msc_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct msc_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	int	timeout_onflg = 0;	/* timeout-flg */
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	int req_q_cnt =0;
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

	DBG(cdev, "msc_read(%d)\n", count);

	if (_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		DBG(cdev, "msc_read: waiting for online state\n");
		ret = wait_event_interruptible_timeout(dev->read_wq,
			(atomic_read(&dev->online) || atomic_read(&dev->error)),
			HZ / 10);
		if (ret < 0) {
			_unlock(&dev->read_excl);
			return ret;
		}
		else if (!atomic_read(&dev->online)) {
			_unlock(&dev->read_excl);
			return -EIO;
		}
	}

#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	req_q_cnt = count % usb_ept_get_max_packet(dev->ep_out);
	if(req_q_cnt == 0)
		req_q_cnt = count / usb_ept_get_max_packet(dev->ep_out);
	else
		req_q_cnt = count / usb_ept_get_max_packet(dev->ep_out) + 1;
	DBG(cdev,"msc_read: count = %d , req_q_cnt = %d \n",count , req_q_cnt);
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

	while (count > 0) {
		if (atomic_read(&dev->error)) {
			DBG(cdev, "msc_read dev->error\n");
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = msc_req_get(dev, &dev->rx_idle))) {
requeue_req:
			req->length = usb_ept_get_max_packet(dev->ep_out);

#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
			if(req_q_cnt == 0) {
				DBG(cdev,"msc_read: req_q_cnt == 0 ,so break \n");
				msc_req_put(dev, &dev->rx_idle, req);
				break;
			}
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

			ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);

			if (ret < 0) {
				r = -EIO;
				atomic_set(&dev->error, 1);
				msc_req_put(dev, &dev->rx_idle, req);
				goto fail;
			} else {
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
				req_q_cnt--;
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */
				DBG(cdev, "rx %p queue\n", req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (dev->read_count > 0) {
			if (dev->read_count < count)
				xfer = dev->read_count;
			else
				xfer = count;

			if (copy_to_user(buf, dev->read_buf, xfer)) {
				r = -EFAULT;
				break;
			}
			dev->read_buf += xfer;
			dev->read_count -= xfer;
			buf += xfer;
			count -= xfer;

			/* if we've emptied the buffer, release the request */
			if (dev->read_count == 0) {
				msc_req_put(dev, &dev->rx_idle, dev->read_req);
				dev->read_req = 0;

				timeout_onflg = 1;
				/* for short packet  */
				if( (r - count) % usb_ept_get_max_packet(dev->ep_out) != 0) {
					r = r - count;
					break;
				}
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		if( timeout_onflg == 0 )
			ret = wait_event_interruptible(dev->read_wq,
				((req = msc_req_get(dev, &dev->rx_done)) ||
				 atomic_read(&dev->error)));
		else
			ret = wait_event_interruptible_timeout(dev->read_wq,
				((req = msc_req_get(dev, &dev->rx_done)) ||
				 atomic_read(&dev->error)),  HZ / 10 );
		
		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0)
				goto requeue_req;

			dev->read_req = req;
			dev->read_count = req->actual;
			dev->read_buf = req->buf;
			DBG(cdev, "rx %p %d\n", req, req->actual);
		}
		else if((ret == 0) && (!atomic_read(&dev->error))) { /* for timeout */
			r = r - count;
			break;
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	if(r < 0) {
		DBG(cdev, "msc_read: case r< 0 \n");
		/* refresh out-direction buffer */
		usb_ep_fifo_flush(dev->ep_out);

		/* if we have a stale request being read, recycle it */
		dev->read_buf = 0;
		dev->read_count = 0;
		if (dev->read_req) {
			msc_req_put(dev, &dev->rx_idle, dev->read_req);
			dev->read_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = msc_req_get(dev, &dev->rx_done)))
			msc_req_put(dev, &dev->rx_idle, req);
	}
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

	_unlock(&dev->read_excl);
	/* interrupts are handled as errs */
	if (r == -ERESTARTSYS)
		r = -EIO;
	DBG(cdev, "msc_read returning %d\n", r);
	return r;
}

static ssize_t msc_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct msc_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	int req_q_cnt;
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */

	DBG(cdev, "msc_write(%d)\n", count);

	if (_lock(&dev->write_excl))
		return -EBUSY;

	if (!atomic_read(&dev->online)) {
		_unlock(&dev->write_excl);
		return -EIO;
	}

	while (count > 0) {
		if (atomic_read(&dev->error)) {
			DBG(cdev, "msc_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			(atomic_read(&dev->error) ||
			 (req = msc_req_get(dev, &dev->tx_idle))));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > BULK_TX_BUFFER_SIZE)
				xfer = BULK_TX_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				DBG(cdev, "msc_write: xfer error %d\n", ret);
				atomic_set(&dev->error, 1);
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}

	if (req)
		msc_req_put(dev, &dev->tx_idle, req);

#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	if( r > 0 ) {
		do {
			unsigned long flags;

			req_q_cnt = 0;

			dev->sleep_flag = 0;
			spin_lock_irqsave(&dev->lock, flags);
			list_for_each_entry(req,&dev->tx_idle,list)
				req_q_cnt++;
			spin_unlock_irqrestore(&dev->lock, flags);

			DBG(cdev, "msc_write: wait complete q_cnt=%d \n", req_q_cnt);

			if(req_q_cnt != TX_REQ_MAX) {
				ret = wait_event_interruptible_timeout( dev->write_wq, 
					(dev->sleep_flag == 1), HZ / 10 );
				if (ret < 0) {
					r = ret;
					break;
				}
			}
			else {
				break;
			}

		} while (!atomic_read(&dev->error));

	}
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */
	_unlock(&dev->write_excl);
	/* interrupts are handled as errs */
	if (r == -ERESTARTSYS)
		r = -EIO;
	DBG(cdev, "msc_write returning %d\n", r);
	return r;
}

static int msc_open(struct inode *ip, struct file *fp)
{
	if (!_msc_dev)
		return -EIO;

	if (_lock(&_msc_dev->open_excl))
		return -EBUSY;

	fp->private_data = _msc_dev;

	/* clear the error latch */
	atomic_set(&_msc_dev->error, 0);

	return 0;
}

static int msc_release(struct inode *ip, struct file *fp)
{
	struct msc_dev *dev = fp->private_data;
	struct usb_request *req_out;

	/* retire any completed rx requests from previous session */
	while ((req_out = msc_req_get(dev, &dev->rx_done)))
			msc_req_put(dev, &dev->rx_idle, req_out);
	if (_msc_dev)
		_unlock(&_msc_dev->open_excl);
	return 0;
}

static long msc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct msc_dev *dev = file->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req_out;
	int ret = 0;

	DBG(cdev, "msc_ioctl(cmd=%d)\n", cmd);

	switch (cmd) {
	case USB_MSC_FUNC_IOC_SET_STALL: {
			int direction = arg; /* in:1 */
			DBG(cdev, "msc_ioctl(USB_MSC_FUNC_IOC_SET_STALL)(dir=%d)\n", direction);
			if( direction == 1 ) {
				usb_ep_set_halt(dev->ep_in);
			}
			else {
				usb_ep_set_halt(dev->ep_out);
				
				/* refresh out-direction buffer */
				usb_ep_fifo_flush(dev->ep_out);
				
				/* if we have a stale request being read, recycle it */
				dev->read_buf = 0;
				dev->read_count = 0;
				if (dev->read_req) {
					msc_req_put(dev, &dev->rx_idle, dev->read_req);
					dev->read_req = 0;
				}

				/* retire any completed rx requests from previous session */
				while ((req_out = msc_req_get(dev, &dev->rx_done)))
					msc_req_put(dev, &dev->rx_idle, req_out);
			}
		}
		break;
	case USB_MSC_FUNC_IOC_GET_MOUNT_STS:
		ret = _msc_dev->mount_enb;
		break;
	case USB_MSC_FUNC_IOC_GET_VENDOR_ID:
		if (!arg) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_VENDOR_ID arg NULL err\n");
			ret = -EFAULT;
			break;
		}
		if (copy_to_user((void __user*)arg,
				&(_msc_dev->msc_inquiry_string[USB_MSC_VENDOR_ID_INDEX]),
				USB_MSC_VENDOR_ID_LEN)) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_VENDOR_ID copy_to_user err\n");
			ret = -EFAULT;
			break;
		}
		ret = USB_MSC_VENDOR_ID_LEN;
		break;
	case USB_MSC_FUNC_IOC_GET_PRODUCT_ID:
		if (!arg) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_PRODUCT_ID arg NULL err\n");
			ret = -EFAULT;
			break;
		}
		if (copy_to_user((void __user*)arg,
				&(_msc_dev->msc_inquiry_string[USB_MSC_PRODUCT_ID_INDEX]),
				USB_MSC_PRODUCT_ID_LEN)) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_PRODUCT_ID copy_to_user err\n");
			ret = -EFAULT;
			break;
		}
		ret = USB_MSC_PRODUCT_ID_LEN;
		break;

#ifdef D_MSC_STORE_MOUNT_FILE_PATH
	case USB_MSC_FUNC_IOC_GET_MOUNT_PATH:
		if (!arg) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_MOUNT_PATH arg NULL err\n");
			ret = -EFAULT;
			break;
		}
		if (copy_to_user((void __user*)arg, _msc_dev->mount_path, strlen(_msc_dev->mount_path))) {
			printk(KERN_ERR "msc_ioctl:USB_MSC_FUNC_IOC_GET_MOUNT_PATH copy_to_user err\n");
			ret = -EFAULT;
			break;
		}
		ret = strlen(_msc_dev->mount_path);
		break;
#endif /* D_MSC_STORE_FILE_PATH */

	default:
		return -ENOTTY;
	}

	return ret;
}


/* file operations for ADB device /dev/android_msc */
static struct file_operations msc_fops = {
	.owner = THIS_MODULE,
	.read = msc_read,
	.write = msc_write,
	.open = msc_open,
	.release = msc_release,
	.unlocked_ioctl = msc_ioctl,
};

static struct miscdevice msc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = msc_shortname,
	.fops = &msc_fops,
};

static void msc_control_notify( unsigned char type,void* opt )
{
	if (!_msc_dev)
		return;

	if (!_msc_dev->ctrl_read_buf)
		return;

	switch (type) {
	case USB_MSC_NTY_ONLINE_STATE:
		_msc_dev->ctrl_set_size = strlen(USB_ONLINE_STATE_CHANGE_STR);
		memcpy(_msc_dev->ctrl_read_buf, USB_ONLINE_STATE_CHANGE_STR , _msc_dev->ctrl_set_size );
		break;
	case USB_MSC_NTY_OFFLINE_STATE:
		_msc_dev->ctrl_set_size = strlen(USB_OFFLINE_STATE_CHANGE_STR);
		memcpy(_msc_dev->ctrl_read_buf, USB_OFFLINE_STATE_CHANGE_STR ,  _msc_dev->ctrl_set_size );
		break;
	case USB_MSC_NTY_MOUNT_ENABLE:
		_msc_dev->ctrl_set_size = strlen(USB_MOUNT_ENABLE_STR);
		memcpy(_msc_dev->ctrl_read_buf, USB_MOUNT_ENABLE_STR ,  _msc_dev->ctrl_set_size );
		break;
	case USB_MSC_NTY_MOUNT_DISABLE:
		_msc_dev->ctrl_set_size = strlen(USB_MOUNT_DISABLE_STR);
		memcpy(_msc_dev->ctrl_read_buf, USB_MOUNT_DISABLE_STR ,  _msc_dev->ctrl_set_size );
		break;
	case USB_MSC_NTY_RESET_STATE:
		_msc_dev->ctrl_set_size = strlen(USB_RESET_STATE_STR);
		memcpy(_msc_dev->ctrl_read_buf, USB_RESET_STATE_STR ,  _msc_dev->ctrl_set_size );
		break;
		
	default:
		return;
	}

	wake_up(&_msc_dev->ctrl_read_wq);

	return;
}


static ssize_t msc_control_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	int	size;
	int	ret = 0;

	if (!_msc_dev)
		return -EIO;

	if (_lock(&_msc_dev->ctrl_read_excl))
		return -EBUSY;

	ret = wait_event_interruptible(_msc_dev->ctrl_read_wq, _msc_dev->ctrl_set_size );
	if (ret < 0) {
		_unlock(&_msc_dev->ctrl_read_excl);
		return ret;
	}

	if (count < _msc_dev->ctrl_set_size) {
		_unlock(&_msc_dev->ctrl_read_excl);
		return -EFAULT;
	}

	if (copy_to_user(buf, _msc_dev->ctrl_read_buf, _msc_dev->ctrl_set_size))
		size =  -EFAULT;
	else {
		size = _msc_dev->ctrl_set_size;
		_msc_dev->ctrl_set_size = 0;
	}

	_unlock(&_msc_dev->ctrl_read_excl);

	return size;
}

static int msc_control_open(struct inode *ip, struct file *fp)
{
	if (!_msc_dev)
		return -EIO;

	if (_lock(&_msc_dev->open_ctrl_excl))
		return -EBUSY;

	fp->private_data = _msc_dev;

	_msc_dev->ctrl_set_size = 0;
	if (_msc_dev->ctrl_read_buf)
		kfree(_msc_dev->ctrl_read_buf);
	_msc_dev->ctrl_read_buf = kzalloc( USB_CTRL_NTY_MAX_STR_LEN, GFP_KERNEL);

	if (atomic_read( &_msc_dev->online ))
		msc_control_notify(USB_MSC_NTY_ONLINE_STATE , NULL);
	else
		msc_control_notify(USB_MSC_NTY_OFFLINE_STATE , NULL);

	return 0;
}

static int msc_control_release(struct inode *ip, struct file *fp)
{
	if (!_msc_dev)
		return 0;

	_unlock(&_msc_dev->open_ctrl_excl);

	_msc_dev->ctrl_set_size = 0;
	if (_msc_dev->ctrl_read_buf) {
		kfree(_msc_dev->ctrl_read_buf);
		_msc_dev->ctrl_read_buf = 0;
	}

	return 0;
}


static struct file_operations msc_control_fops = {
	.owner		= THIS_MODULE,
	.read		= msc_control_read,
	.open		= msc_control_open,
	.release	= msc_control_release,
};

static struct miscdevice msc_control_device = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "msc_control",
	.fops		= &msc_control_fops,
};

/*-------------------------------------------------------------------------*/
/* MOUNT_FILE */
static int msc_function_set_mount_enable(int enable)
{
	if (!_msc_dev)
		return -EIO;

	_msc_dev->mount_enb = !!enable;
	
	if (enable != 0)
		msc_control_notify(USB_MSC_NTY_MOUNT_ENABLE , NULL);
	else
		msc_control_notify(USB_MSC_NTY_MOUNT_DISABLE , NULL);

	return 0;
}

#ifndef D_MSC_STORE_MOUNT_FILE_PATH
static int msc_function_get_mount_enable(void)
{
	if (!_msc_dev)
		return -EIO;

	return _msc_dev->mount_enb;
}
#endif /* D_MSC_STORE_FILE_PATH */

static ssize_t show_msc_file(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t		rc;
#ifndef D_MSC_STORE_MOUNT_FILE_PATH
	int 		mnt;
#endif /* D_MSC_STORE_FILE_PATH */

#ifdef D_MSC_STORE_MOUNT_FILE_PATH
	if (!_msc_dev)
		return -EIO;

	rc = snprintf(buf, PAGE_SIZE, "%s", _msc_dev->mount_path);
#else /* D_MSC_STORE_FILE_PATH */
	mnt = msc_function_get_mount_enable();

	if (mnt < 0)
		rc = -EIO;
	else
		rc = snprintf(buf, PAGE_SIZE, "%d", mnt);
#endif /* D_MSC_STORE_FILE_PATH */

	return rc;
}

static ssize_t store_msc_file(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	/* Remove a trailing newline */
	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;

#ifdef D_MSC_STORE_MOUNT_FILE_PATH
	if (count > 0 && buf[0]) {
		int len;

		if (!_msc_dev)
			return -EIO;

		len = strlen(buf);
		if (len >= D_MSC_MOUNT_PATH_LEHGTH_MAX) {
			((char *) buf)[D_MSC_MOUNT_PATH_LEHGTH_MAX-1] = 0;
		}

		len = snprintf(_msc_dev->mount_path, D_MSC_MOUNT_PATH_LEHGTH_MAX, "%s", buf);
		if (count != len)
			printk(KERN_ERR "%s:cant store all characters\n", __func__);
		msc_function_set_mount_enable(1);
	}
	else {
		_msc_dev->mount_path[0] = 0;
		msc_function_set_mount_enable(0);
	}
#else /* D_MSC_STORE_FILE_PATH */
	if (count > 0 && buf[0]) 
		msc_function_set_mount_enable(1);
	else
		msc_function_set_mount_enable(0);
#endif /* D_MSC_STORE_FILE_PATH */

	return count;
}

static struct device_attribute dev_attr_msc_file = __ATTR(file, 0444, show_msc_file, store_msc_file);

/*-------------------------------------------------------------------------*/

static void msc_lun_release(struct device *dev)
{
	struct msc_dev	*mdev = dev_get_drvdata(dev);

	kfree(mdev->lun);
	mdev->lun = 0;
	return;
}


/*-------------------------------------------------------------------------*/

static void
msc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct msc_dev		*dev = msc_func_to_dev(f);
	struct usb_request	*req;

	while ((req = msc_req_get(dev, &dev->rx_idle)))
		msc_request_free(req, dev->ep_out);
	while ((req = msc_req_get(dev, &dev->tx_idle)))
		msc_request_free(req, dev->ep_in);
}

static int
msc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev	*cdev = c->cdev;
	struct msc_dev			*dev = msc_func_to_dev(f);
	int				id;
	int				ret;

	dev->cdev = cdev;
	DBG(cdev, "msc_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	msc_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = msc_create_bulk_endpoints(dev, &msc_fullspeed_in_desc,
			&msc_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		msc_highspeed_in_desc.bEndpointAddress =
			msc_fullspeed_in_desc.bEndpointAddress;
		msc_highspeed_out_desc.bEndpointAddress =
			msc_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);

	return 0;
}

static int msc_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct msc_dev			*dev = msc_func_to_dev(f);
	struct usb_composite_dev	*cdev = f->config->cdev;
	int ret;

	DBG(cdev, "msc_function_set_alt intf: %d alt: %d\n", intf, alt);
	
	if( atomic_read(&dev->online) )
		return 0;
	
	/* Enable the endpoints */
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_in);
	if (ret)
		return ret;

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	atomic_set(&dev->online, 1);

	msc_control_notify(USB_MSC_NTY_ONLINE_STATE , NULL);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void msc_function_disable(struct usb_function *f)
{
	struct msc_dev			*dev = msc_func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "msc_function_disable\n");

	if( !cdev )
		return;

	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);

	wake_up(&dev->read_wq);

	usb_ep_fifo_flush(dev->ep_in);
	usb_ep_fifo_flush(dev->ep_out);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	msc_control_notify(USB_MSC_NTY_OFFLINE_STATE , NULL);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

int msc_function_setup(struct usb_function *f,
	const struct usb_ctrlrequest *ctrl)
{
	struct msc_dev			*dev = msc_func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct usb_request		*req = cdev->req;
	int				value = -EOPNOTSUPP;
	u16				w_index = le16_to_cpu(ctrl->wIndex);
	u16				w_value = le16_to_cpu(ctrl->wValue);

	DBG(cdev, "msc_function_setup\n");
	if (w_index != msc_interface_desc.bInterfaceNumber)
		return value;

	/* Handle Bulk-only class-specific requests */
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		DBG(cdev, "USB_TYPE_CLASS\n");
		switch (ctrl->bRequest) {
		case USB_BULK_RESET_REQUEST:
			if (ctrl->bRequestType != (USB_DIR_OUT |
					USB_TYPE_CLASS | USB_RECIP_INTERFACE))
				break;
			if (w_value != 0) {
				value = -EDOM;
				break;
			}

			/* Raise an exception to stop the current operation
			 * and reinitialize our state. */
			DBG(cdev, "bulk reset request\n");
			msc_control_notify(USB_MSC_NTY_RESET_STATE ,NULL);
			value = 0;
			break;

		case USB_BULK_GET_MAX_LUN_REQUEST:
			if (ctrl->bRequestType != (USB_DIR_IN |
					USB_TYPE_CLASS | USB_RECIP_INTERFACE))
				break;
			if (w_value != 0) {
				value = -EDOM;
				break;
			}
			VDBG(cdev, "get max LUN\n");
			*(u8 *)cdev->req->buf = 0;	/* we have only one disk */
			value = 1;
			break;
		}
	}

	if (value >= 0) {
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			req->status = 0;
	}
	return value;
}

static void msc_reset_descriptor(struct usb_configuration *c, 
		struct usb_function *f,
		struct usb_descriptor_header **descriptors ,
		u8 bInterfaceNumber)
{
	msc_interface_desc.bInterfaceNumber = bInterfaceNumber;
	return;
}

static void msc_function_suspend(struct usb_function *f)
{
	msc_function_disable(&_msc_dev->function);
}

static int msc_bind_config(struct usb_configuration *c)
{
	struct msc_dev *dev = _msc_dev;
	int ret;

	/* maybe allocate device-global string ID */
	if (msc_string_defs[0].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		msc_string_defs[0].id = ret;
		msc_interface_desc.iInterface = ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = D_MSC_FUNCTION_NAME;
	dev->function.descriptors = fs_msc_descs;
	dev->function.hs_descriptors = hs_msc_descs;
	dev->function.bind = msc_function_bind;
	dev->function.unbind = msc_function_unbind;
	dev->function.set_alt = msc_function_set_alt;
	dev->function.disable = msc_function_disable;

	/* ADD */
	dev->function.strings = msc_strings;
	dev->function.setup = msc_function_setup;
	dev->function.reset_descriptor = msc_reset_descriptor;
	dev->function.suspend = msc_function_suspend;

	return usb_add_function(c, &dev->function);
}

static int msc_setup(struct usb_composite_dev *cdev, struct device *device)
{
	struct msc_dev	*dev;
	struct lun	*curlun;
	int		ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->rx_done);
	INIT_LIST_HEAD(&dev->tx_idle);

	/* reset any added variaties */
	atomic_set(&dev->open_ctrl_excl, 0);
	atomic_set(&dev->ctrl_read_excl, 0);

	init_waitqueue_head(&dev->ctrl_read_wq);

	dev->ctrl_set_size = 0;
	dev->ctrl_read_buf = NULL;

	dev->mount_enb = 0;

	/* _msc_dev must be set before calling usb_gadget_register_driver */
	_msc_dev = dev;

	/* register for msc_device */
	ret = misc_register(&msc_device);
	if (ret) {
		printk(KERN_ERR "msc_device regist failure %d", ret);
		goto err1;
	}

	/* register for msc_control */
	ret = misc_register(&msc_control_device);
	if (ret) {
		printk(KERN_ERR "msc_control_device regist failure %d", ret);
		goto err2;
	}

	/* MOUNT_FILE */
	dev_attr_msc_file.attr.mode = 0644;

	curlun = kzalloc(sizeof(struct lun), GFP_KERNEL);
	if (!curlun) {
		printk(KERN_ERR "lun cannot allocate");
		goto err2;
	}

	curlun->dev.release = msc_lun_release;
	curlun->dev.parent = device;
	dev_set_drvdata(&curlun->dev, dev);
	dev_set_name(&curlun->dev, "lun");
	ret = device_register(&curlun->dev);
	if (ret != 0) {
		printk(KERN_ERR "%s:device_register(lun) failed\n", __func__);
		goto err3;
	}
	ret = device_create_file(&curlun->dev, &dev_attr_msc_file);
	if (ret != 0) {
		printk(KERN_ERR "%s:device_create_file(lun) failed\n", __func__);
		device_unregister(&curlun->dev);
		goto err3;
	}

	return ret;

err3:
	kfree(curlun);
err2:
	printk(KERN_ERR "msc_device deregist");
	misc_deregister(&msc_device);
err1:
	kfree(dev);
	_msc_dev = NULL;
	printk(KERN_ERR "msc gadget driver failed to initialize\n");
	return ret;
}

static void msc_cleanup(void)
{
	if (_msc_dev && _msc_dev->lun) {
		/* MOUNT_FILE */ /* remove lun */
		device_remove_file(&_msc_dev->lun->dev, &dev_attr_msc_file);
		device_unregister(&_msc_dev->lun->dev);
	}

	misc_deregister(&msc_control_device);
	misc_deregister(&msc_device);
	if (_msc_dev) {
		kfree(_msc_dev);
		_msc_dev = NULL;
	}
}

