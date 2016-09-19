/* drivers/usb/gadget/lismo_buf_control.h
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

#ifndef _LISMO_BUF_CONTROL_H_
#define _LISMO_BUF_CONTROL_H_


#define SC_VENDOR_START			0xe4
#define SC_VENDOR_END			0xef
#define VENDOR_CMD_NR	(SC_VENDOR_END - SC_VENDOR_START + 1)

struct op_desc {
	struct device	dev;
	unsigned long	flags;
/* flag symbols are bit numbers */
#define FLAG_IS_READ	0	/* not use */
#define FLAG_IS_WRITE	1	/* not use */
#define FLAG_EXPORT	2	/* protected by sysfs_lock */
	char			*buffer;
	size_t			len;
	struct bin_attribute	dev_bin_attr_buffer;
	unsigned long 		update;
	struct work_struct	work;
	struct sysfs_dirent	*value_sd;
};

/* buffer size alloc at __init() */
#define ALLOC_INI_SIZE  0x101000
#define ALLOC_CMD_CNT   1

static void op_release(struct device *dev);

static DEFINE_MUTEX(sysfs_lock);

struct fsg_lun {
	struct device	dev;
	struct op_desc *op_desc[VENDOR_CMD_NR];
	char   *reserve_buf[VENDOR_CMD_NR];
};

#define fsg_lun_is_open(curlun)	((curlun)->filp != NULL)

static struct fsg_lun *fsg_lun_from_dev(struct device *dev)
{
	return container_of(dev, struct fsg_lun, dev);
}

static struct op_desc *dev_to_desc(struct device *dev)
{
	return container_of(dev, struct op_desc, dev);
}

#endif /* _LISMO_BUF_CONTROL_H_ */
