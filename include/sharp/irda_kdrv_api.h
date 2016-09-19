/* include/sharp/irda_kdrv_api.h (sharp IrDA driver)
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
#ifndef _IRDA_KDRV_API_H
#define _IRDA_KDRV_API_H

#include <linux/ioctl.h>
#include <linux/tty.h>

#include "irda_common.h"


#define SHIRDA_UARTDM_ID		(0)

#define TTYHS_DEVFILE_NAME		"ttyHSL3"
#define TTYHS_DEVFILE			"/dev/"TTYHS_DEVFILE_NAME

#define SHIRDA_DEVFILE_NAME		"msm_shirda"
#define SHIRDA_DEVFILE			"/dev/"SHIRDA_DEVFILE_NAME



#ifdef N_IRDA
#define N_SHIRDA N_IRDA
#else
#define N_SHIRDA 11
#endif

#define IRDA_DRV_IOCTL_MAGIC	'i'
#define IRDA_DRV_IOCTL_SET_QOS		_IOW(IRDA_DRV_IOCTL_MAGIC, 1, \
						irda_qos_info)
#define IRDA_DRV_IOCTL_GET_QOS		_IOR(IRDA_DRV_IOCTL_MAGIC, 2, \
						irda_qos_info)
#define IRDA_DRV_IOCTL_READ_WAKEUP	_IO (IRDA_DRV_IOCTL_MAGIC, 3)
#define IRDA_DRV_IOCTL_GET_ERR		_IOR(IRDA_DRV_IOCTL_MAGIC, 4, int)
#define IRDA_DRV_IOCTL_LOOPBACK 	_IO (IRDA_DRV_IOCTL_MAGIC, 5)
#define IRDA_DRV_IOCTL_GET_CAPABILITY 	_IOR(IRDA_DRV_IOCTL_MAGIC, 6, \
						irda_kdrv_capa_notify)
#define IRDA_DRV_IOCTL_GET_MEDIABUSY	_IOR(IRDA_DRV_IOCTL_MAGIC, 7, \
						int)
#define IRDA_DRV_IOCTL_CLR_MEDIABUSY	_IO(IRDA_DRV_IOCTL_MAGIC, 8)

#define IRDA_LDISC_NO_ERR		(1)
#define IRDA_LDISC_READ_CANCELED	(2)
#define IRDA_LDISC_LOGICAL_ERR		(3)
#define IRDA_LDISC_PERMISSION_ERR	(4)
#define IRDA_LDISC_TX_TIMEOUT		(5)
#define IRDA_LDISC_RX_TIMEOUT		(6)
#define IRDA_LDISC_CLOSED		(7)
#define IRDA_LDISC_RX_BUFFER_OVERFLOW	(8)
#define IRDA_LDISC_TX_SEND_ERR		(9)

#define IRDA_LDISC_MEDIA_FREE		(0)
#define IRDA_LDISC_MEDIA_BUSY		(1)

#endif
