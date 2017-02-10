/* include/sharp/sh_regaxs.h
 *
 * Copyright (C) 2012 Sharp Corporation
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
 
#ifndef __SH_REGAXS_H__
#define __SH_REGAXS_H__

#include <linux/types.h>
#include <linux/ioctl.h>


#define SH_REGAXS_DEVFILE "/dev/sh_regaxs"

struct sh_regaxs_read_write {
    /* Memory address or register address depending on ioctl */
    unsigned int physaddr;
    unsigned int data;
};

#define SH_REGAXS_IOCTL (0x81) /* Magic number for SH_REGAXS Iocts */

#define SH_REGAXS_READ  _IOR(SH_REGAXS_IOCTL, 0x1, struct sh_regaxs_read_write)
#define SH_REGAXS_WRITE _IOW(SH_REGAXS_IOCTL, 0x2, struct sh_regaxs_read_write)

#endif // __SH_REGAXS_H__
