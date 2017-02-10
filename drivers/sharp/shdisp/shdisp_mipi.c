/* drivers/sharp/shdisp/shdisp_mipi.c  (Display Driver)
 *
 * Copyright (C) 2012-2013 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */

#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/sh_boot_manager.h>
#include "shdisp_mipi.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shdisp_api_lcd_on(void);
int shdisp_api_lcd_off(int para);
int shdisp_api_start_display(void);

void shdisp_mipi_api_check_blackscreen_timeout(int type);
#ifdef CONFIG_SHDISP_PANEL_IGZO
static void shdisp_mipi_blackscreen_wq_handler(struct work_struct *work);
static int shdisp_mipi_check_diagmode(void);
#endif /* CONFIG_SHDISP_PANEL_IGZO */

extern void shdisp_mipi_recover_semaphore_start(void);
extern void shdisp_mipi_recover_semaphore_end(void);
/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */
enum {
    LCD_RECOVERY_STOP,
    LCD_RECOVERY_ACTIVE,
    NUM_RECOVERY_STATUS
};

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
int fb_suspend_cond = FB_SUSPEND_COND_DISP_ON;
static struct work_struct shdisp_mipi_blackscreen_worker;
static int shdisp_mipi_blackscreen_type = 0;
static int lcd_recovering = LCD_RECOVERY_STOP;

DEFINE_MUTEX(shdisp_blackscreen_sem);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_api_lcd_on                                                         */
/* ------------------------------------------------------------------------- */
int shdisp_api_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_DEBUG("in. cond=%d\n", fb_suspend_cond);
    shdisp_mipi_recover_semaphore_start();

#ifdef CONFIG_SHDISP_PANEL_IGZO
    if (fb_suspend_cond == FB_SUSPEND_COND_1HZ){
        ;
    }
    else {
        shdisp_api_main_lcd_power_on();
        shdisp_api_main_disp_on();
        fb_suspend_cond = FB_SUSPEND_COND_NORMAL_ON;
    }
#else /* CONFIG_SHDISP_PANEL_IGZO */
    shdisp_api_main_lcd_power_on();
    shdisp_api_main_disp_on();
    fb_suspend_cond = FB_SUSPEND_COND_NORMAL_ON;
#endif /* CONFIG_SHDISP_PANEL_IGZO */

    shdisp_mipi_recover_semaphore_end();
    SHDISP_DEBUG("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_lcd_off                                                        */
/* ------------------------------------------------------------------------- */
int shdisp_api_lcd_off(int para)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef CONFIG_SHDISP_PANEL_IGZO
    int get_music_flg = FALSE;
    int fw_timeout_flg = FALSE;
    int diagmode_flg = FALSE;
#endif /* CONFIG_SHDISP_PANEL_IGZO */

    SHDISP_DEBUG("in. cond=%d\n", fb_suspend_cond);
    shdisp_mipi_recover_semaphore_start();
 
#ifdef CONFIG_SHDISP_PANEL_IGZO
    get_music_flg = shterm_get_music_info();
    fw_timeout_flg = shdisp_api_main_mipi_cmd_is_fw_timeout();
    diagmode_flg = shdisp_mipi_check_diagmode();
    SHDISP_DEBUG("get_music_flg=%d\n", get_music_flg);
    SHDISP_DEBUG("fw_timeout_flg=%d\n", fw_timeout_flg);
    SHDISP_DEBUG("diagmode_flg=%d\n", diagmode_flg);

    if (para || get_music_flg || fw_timeout_flg || diagmode_flg || (fb_suspend_cond == FB_SUSPEND_COND_NORMAL_ON)){
        shdisp_api_main_mipi_cmd_lcd_stop_prepare();
        shdisp_api_main_disp_off();
        shdisp_api_main_lcd_power_off(SHDISP_PANEL_POWER_NORMAL_OFF);
        fb_suspend_cond = FB_SUSPEND_COND_OFF;
    }
    else {
        if (fb_suspend_cond == FB_SUSPEND_COND_1HZ){
            SHDISP_DEBUG("already suspend 1Hz out ret=%04x\n", ret);
            goto shdisp_api_lcd_off_skip;
        }
        shdisp_api_main_mipi_cmd_lcd_off_black_screen_on();
        fb_suspend_cond = FB_SUSPEND_COND_1HZ;
    }
#else /* CONFIG_SHDISP_PANEL_IGZO */
    shdisp_api_main_mipi_cmd_lcd_stop_prepare();
    shdisp_api_main_disp_off();
    if (para){
        shdisp_api_main_lcd_power_off(SHDISP_PANEL_POWER_SHUTDOWN_OFF);
    } else {
        shdisp_api_main_lcd_power_off(SHDISP_PANEL_POWER_NORMAL_OFF);
    }
    fb_suspend_cond = FB_SUSPEND_COND_OFF;
#endif /* CONFIG_SHDISP_PANEL_IGZO */

#ifdef CONFIG_SHDISP_PANEL_IGZO
shdisp_api_lcd_off_skip:
#endif /* CONFIG_SHDISP_PANEL_IGZO */
    shdisp_mipi_recover_semaphore_end();
    SHDISP_DEBUG("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_start_display                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_api_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_DEBUG("in. cond=%d\n", fb_suspend_cond);

#ifdef CONFIG_SHDISP_PANEL_IGZO
    if (fb_suspend_cond == FB_SUSPEND_COND_1HZ){
        shdisp_api_main_mipi_cmd_lcd_on_after_black_screen();
    }
    else {
        shdisp_api_main_mipi_cmd_lcd_start_display();

        if( lcd_recovering != LCD_RECOVERY_ACTIVE ){
            if( shdisp_api_check_det() != SHDISP_RESULT_SUCCESS ){
                shdisp_api_requestrecovery();
            }
        }
    }
    fb_suspend_cond = FB_SUSPEND_COND_DISP_ON;
#else /* CONFIG_SHDISP_PANEL_IGZO */
    shdisp_api_main_mipi_cmd_lcd_start_display();
    fb_suspend_cond = FB_SUSPEND_COND_DISP_ON;
    if( lcd_recovering != LCD_RECOVERY_ACTIVE ){
        if( shdisp_api_check_det() != SHDISP_RESULT_SUCCESS ){
            shdisp_api_requestrecovery();
        }
    }
#endif /* CONFIG_SHDISP_PANEL_IGZO */

    SHDISP_DEBUG("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_mipi_check_blackscreen_timeout                                 */
/* ------------------------------------------------------------------------- */
void shdisp_mipi_api_check_blackscreen_timeout(int type)
{
    SHDISP_DEBUG("in type=%d\n", type);
    shdisp_mipi_blackscreen_type = type;
    schedule_work(&shdisp_mipi_blackscreen_worker);
    SHDISP_DEBUG("out\n");
}

#ifdef CONFIG_SHDISP_PANEL_IGZO
/* ------------------------------------------------------------------------- */
/* shdisp_mipi_blackscreen_wq_handler                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_mipi_blackscreen_wq_handler(struct work_struct *work)
{
}

/* ------------------------------------------------------------------------- */
/* shdisp_mipi_api_check_diagmode                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_mipi_check_diagmode(void)
{
    if(sh_boot_get_bootmode() != SH_BOOT_D && sh_boot_get_bootmode() != SH_BOOT_F_F){
        return FALSE;
    }
    else {
        return TRUE;
    }
}

static int shdisp_prepare(struct device *dev)
{
    SHDISP_DEBUG("in\n");
    if (fb_suspend_cond == FB_SUSPEND_COND_1HZ){
        shdisp_api_lcd_off(1);
    }
    SHDISP_DEBUG("out\n");
	return 0;
}
static struct dev_pm_ops shdisp_dev_pm_ops = {
    .prepare = shdisp_prepare,
};

#ifdef CONFIG_OF
static const struct of_device_id shdisp_igzo_dt_match[] = {
	{ .compatible = "sharp,shdisp_igzo",},
	{}
};
#else
#define shdisp_igzo_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_driver = {
	.driver = {
		/*
		 * Driver name must match the device name added in
		 * platform.c.
		 */
		.name = "shdisp_igzo",
		.pm = &shdisp_dev_pm_ops,
		.of_match_table = shdisp_igzo_dt_match,
	},
};

static int shdisp_register_driver(void)
{
    SHDISP_DEBUG("shdisp_register_driver. in\n");
	return platform_driver_register(&shdisp_driver);
}
#endif /* CONFIG_SHDISP_PANEL_IGZO */

/* ------------------------------------------------------------------------- */
/* shdisp_mipi_init                                                          */
/* ------------------------------------------------------------------------- */
int shdisp_mipi_init(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef CONFIG_SHDISP_PANEL_IGZO
    SHDISP_DEBUG("in\n");

    shdisp_register_driver();

    INIT_WORK(&shdisp_mipi_blackscreen_worker, shdisp_mipi_blackscreen_wq_handler);

#endif /* CONFIG_SHDISP_PANEL_IGZO */
    SHDISP_DEBUG("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
