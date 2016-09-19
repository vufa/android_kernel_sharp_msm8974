/* drivers/sharp/shirda/shirda_kdrv.h (sharp IrDA driver)
 *
 * Copyright (C) 2011 - 2013 SHARP CORPORATION All rights reserved.
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
#ifndef __SHIRDA_CMN__
#define __SHIRDA_CMN__

#define SHIRDA_KERNEL_DRIVER_MASTER_VERSION	"2.11.00a"







#ifndef SHIRDA_DRIVER_NAME
#ifdef SHIRDA_LDISC_DRIVER_NAME
#define SHIRDA_DRIVER_NAME	SHIRDA_LDISC_DRIVER_NAME
#else
#define SHIRDA_DRIVER_NAME	"shirda"
#endif
#endif

#define IRDALOG_FATAL(format, args...)	\
	printk(KERN_ALERT "[%s]%s() " format, SHIRDA_DRIVER_NAME, \
							__func__, ##args)
#define IRDALOG_ERROR(format, args...)	\
	printk(KERN_ERR "[%s]%s() " format, SHIRDA_DRIVER_NAME, \
							__func__, ##args)
#define IRDALOG_WARNING(format, args...)	\
	printk(KERN_WARNING "[%s]%s() " format, SHIRDA_DRIVER_NAME, \
							__func__, ##args)
#define	IRDALOG_INFO(format, args...)

#endif
