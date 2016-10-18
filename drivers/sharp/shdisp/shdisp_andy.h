/* drivers/sharp/shdisp/shdisp_andy.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_ANDY_H
#define SHDISP_ANDY_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

struct shdisp_panel_operations *shdisp_andy_API_create(void);

int shdisp_andy_API_mipi_start_display(void);

int shdisp_andy_API_mipi_lcd_on_after_black_screen(void);
int shdisp_andy_API_mipi_lcd_off_black_screen_on(void);
#if defined (CONFIG_ANDROID_ENGINEERING)
int shdisp_andy_API_dump_reg(int cog);
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif /* SHDISP_ANDY_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

