/* drivers/video/msm/mdss/mdss_shdisp.c  (Display Driver)
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
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
#include <linux/types.h>
#include <mach/board.h>
#include <sharp/shdisp_kerl.h>
#include "mdss_fb.h"
#include "mdss_shdisp.h"

static int lcd_disp_on = 0;

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_disp_status(void)
{
	shdisp_api_get_boot_context();

	if( shdisp_api_get_boot_disp_status() ) {
		return true;
	}

	return false;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_on( void )
{
	shdisp_api_lcd_on();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_off( void )
{
    int mode;
    mode = mdss_fb_shutdown_in_progress();
    pr_debug("caal shdisp_api_lcd_off() mode=%d", mode);
    shdisp_api_lcd_off(mode);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bkl_ctl( u32 bl_level )
{
	struct shdisp_main_bkl_ctl bkl;

	pr_debug("%s: called bl_level=%u\n", __func__, bl_level);

	if( bl_level == 0 ) {
		shdisp_api_main_bkl_off();
	} else {
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
int mdss_shdisp_is_disp_on( void )
{
	return lcd_disp_on;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_start_display( void )
{
	shdisp_api_start_display();
	lcd_disp_on = 1;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lcd_power_on( void )
{
//	shdisp_api_main_lcd_power_on();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lcd_disp_off( void )
{
//	shdisp_api_main_disp_off();
	lcd_disp_on = 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_pll_ctl(int ctl)
{
	return shdisp_api_main_pll_ctl(ctl);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_shutdown( void )
{
	shdisp_api_shutdown();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_bright_to_bl(enum led_brightness value)
{
    int bl_lvl;

    bl_lvl = 0;

    if (value == 0) {
        bl_lvl = 0;
    } else if ((value >= 1) && (value <= 20)) {
        bl_lvl = 1;
    } else if ((value >= 21) && (value <= 30)) {
        bl_lvl = 2;
    } else if ((value >= 31) && (value <= 40)) {
        bl_lvl = 3;
    } else if ((value >= 41) && (value <= 50)) {
        bl_lvl = 4;
    } else if ((value >= 51) && (value <= 60)) {
        bl_lvl = 5;
    } else if ((value >= 61) && (value <= 70)) {
        bl_lvl = 6;
    } else if ((value >= 71) && (value <= 80)) {
        bl_lvl = 7;
    } else if ((value >= 81) && (value <= 90)) {
        bl_lvl = 8;
    } else if ((value >= 91) && (value <= 100)) {
        bl_lvl = 9;
    } else if ((value >= 101) && (value <= 110)) {
        bl_lvl = 10;
    } else if ((value >= 111) && (value <= 120)) {
        bl_lvl = 11;
    } else if ((value >= 121) && (value <= 130)) {
        bl_lvl = 12;
    } else if ((value >= 131) && (value <= 140)) {
        bl_lvl = 13;
    } else if ((value >= 141) && (value <= 150)) {
        bl_lvl = 14;
    } else if ((value >= 151) && (value <= 160)) {
        bl_lvl = 15;
    } else if ((value >= 161) && (value <= 170)) {
        bl_lvl = 16;
    } else if ((value >= 171) && (value <= 180)) {
        bl_lvl = 17;
    } else if ((value >= 181) && (value <= 190)) {
        bl_lvl = 18;
    } else if ((value >= 191) && (value <= 200)) {
        bl_lvl = 19;
    } else if ((value >= 201) && (value <= 210)) {
        bl_lvl = 20;
    } else if ((value >= 211) && (value <= 230)) {
        bl_lvl = 21;
    } else {
        bl_lvl = 22;
    }

    pr_debug("%s: called value=%d bl_lvl=%d\n", __func__, value, bl_lvl);

    return bl_lvl;
}

