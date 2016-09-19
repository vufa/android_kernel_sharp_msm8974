/* include/sharp/felica_pon.h  (FeliCa driver)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
#ifndef _LINUX_FELICA_PON_H
#define _LINUX_FELICA_PON_H

#define IOC_MAGIC	'f'

#define SHMFD_PON_REQ_HIGH_ENABLE			_IO(IOC_MAGIC, 1)			/* PON HIGH ENABLE			*/
#define SHMFD_PON_REQ_HIGH_DISABLE			_IO(IOC_MAGIC, 2)			/* PON HIGH DISABLE			*/

#define SHMFD_PON_STATUS_LOW				0							/* PON STATUS LOW			*/
#define SHMFD_PON_STATUS_HIGH				1							/* PON STATUS HIGH			*/

#endif /* _LINUX_FELICA_PON_H */
