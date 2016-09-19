/* include/sharp/felica_cen_diag.h  (FeliCa driver)
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

#ifndef FELICA_CEN_DIAG_H
#define FELICA_CEN_DIAG_H

/* ioctl				*/
struct ioctl_cmd
{
	unsigned int reg;
	unsigned int offset;
	unsigned int val;
};

#define IOC_MAGIC	'f'

#define FELICA_PPC_READ_E2PROM				_IOWR(IOC_MAGIC, 1, struct ioctl_cmd)	/* read E2PROM process				*/
#define FELICA_PPC_READ_REGISTER			_IOWR(IOC_MAGIC, 2, struct ioctl_cmd)	/* read register process			*/
#define FELICA_PPC_WRITE_E2PROM				_IOWR(IOC_MAGIC, 3, struct ioctl_cmd)	/* write E2PROM process				*/
#define FELICA_PPC_WRITE_REGISTER			_IOWR(IOC_MAGIC, 4, struct ioctl_cmd)	/* write register process			*/
#define FELICA_PPC_RELOAD					_IO(IOC_MAGIC, 5)						/* reload process					*/
#define FELICA_PPC_REVERSE_OUTPUT			_IO(IOC_MAGIC, 6)						/* reverse output process			*/
#define FELICA_PPC_WRITE_PROTECT			_IOWR(IOC_MAGIC, 7, struct ioctl_cmd)	/* write protect process			*/

struct mfd_cen
{
	unsigned char item_id;					/* command number		*/
	unsigned char data;						/* set_data				*/
};
#endif /* FELICA_CEN_DIAG_H */
