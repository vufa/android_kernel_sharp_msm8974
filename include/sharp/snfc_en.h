/* include/sharp/snfc_en.h (NFC driver)
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
#ifndef _LINUX_SNFC_EN_H
#define _LINUX_SNFC_EN_H

/* for ioctl */
#define SNFC_EN_IOC_MAGIC 's'
#define SHSNFC_EN_REQ_CHIPRESET			_IO(SNFC_EN_IOC_MAGIC, 1)

/* for write(nop) */
#define SHNFC_EN_CHIP_POWER_OFF			1			/* VFEL-RST HIGH */
#define SHNFC_EN_CHIP_POWER_ON			0			/* VFEL-RST LOW  */

#endif /* _LINUX_SNFC_EN_H */

