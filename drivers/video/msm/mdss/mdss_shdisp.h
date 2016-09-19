/* drivers/video/msm/mdss/mdss_shdisp.h  (Display Driver)
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

#ifndef MDSS_SHDISP_H
#define MDSS_SHDISP_H

#include <linux/types.h>
#include "mdss_panel.h"
#include <linux/leds.h>

extern bool mdss_shdisp_get_disp_status(void);

extern void mdss_shdisp_lcd_power_on( void );
extern void mdss_shdisp_lcd_disp_off( void );
extern void mdss_shdisp_dsi_panel_on( void );
extern void mdss_shdisp_dsi_panel_off( void );

extern void mdss_shdisp_start_display( void );

extern void mdss_shdisp_bkl_ctl( u32 bl_level );
extern int mdss_shdisp_is_disp_on( void );

extern int mdss_shdisp_pll_ctl(int ctl);

extern void mdss_shdisp_shutdown( void );

extern int mdss_shdisp_bright_to_bl(enum led_brightness value);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00031 */
#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...) \
                pr_debug(",[SHDISP_PERFORM]" fmt, ## args);
#else /* CONFIG_ANDROID_ENGINEERING */
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...)
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif

#endif /* MDSS_SHDISP_H */
