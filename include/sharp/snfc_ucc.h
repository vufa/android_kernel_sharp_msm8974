/* include/sharp/snfc_ucc.h (NFC driver)
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
#ifndef _LINUX_SNFC_UCC_H
#define _LINUX_SNFC_UCC_H

#define IOC_MAGIC 's'

#define SHSNFC_UCC_REQ_AUTOPOLL			_IOWR(IOC_MAGIC, 1, int)
#define SHSNFC_UCC_REQ_START_NFC		_IOWR(IOC_MAGIC, 2, int)
#define SHSNFC_UCC_REQ_END_PROC			_IOWR(IOC_MAGIC, 3, int)

#define SHSNFC_UCC_FLAG_PON_WAIT		(0x01)		/* wait after PON High */
#define SHSNFC_UCC_FLAG_PON_WAIT_NO_RFS	(0x02)		/* wait after PON High, if no RFS */
#define SHSNFC_UCC_FLAG_RWS_CHECK		(0x04)		/* check RWS */

enum
{
	SHSNFC_UCC_RETVAL_OK = 0,
	SHSNFC_UCC_RETVAL_BUSY = -1,
	SHSNFC_UCC_RETVAL_ABNORMAL = -2
};

#endif /* _LINUX_SNFC_UCC_H */

