/* drivers/sharp/shdisp/shdisp_clmr_fw.h  (Display Driver)
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

/* eDRAM Base Address */
const unsigned short arm_fw_base = 0x7CE0;
/* Start address */
const unsigned long arm_fw_start = 0x00000000;
/* Code size counted by 128bit (16Byte) */
const unsigned long arm_fw_size = 3638;
/* Code image */
#include "shdisp_clmr_fw_default.h"

