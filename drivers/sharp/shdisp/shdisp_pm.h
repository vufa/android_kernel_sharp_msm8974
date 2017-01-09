/* drivers/sharp/shdisp/shdisp_pm.h  (Display Driver)
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
#ifndef SHDISP_PM_H
#define SHDISP_PM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_DEV_TYPE_NONE        (0x00000000)
#define SHDISP_DEV_TYPE_LCD         (0x00000001)
#define SHDISP_DEV_TYPE_BKL         (0x00000002)
#define SHDISP_DEV_TYPE_LED_NMR     (0x00000004)
#define SHDISP_DEV_TYPE_LED_ANI     (0x00000008)
#define SHDISP_DEV_TYPE_PS          (0x00000010)
#define SHDISP_DEV_TYPE_ALS_APP     (0x00000020)
#define SHDISP_DEV_TYPE_ALS_BKL     (0x00000040)
#define SHDISP_DEV_TYPE_ALS_LUX     (0x00000080)
#define SHDISP_DEV_TYPE_DIAG        (0x00000100)
#define SHDISP_DEV_TYPE_RECOVER     (0x00000200)
#define SHDISP_DEV_TYPE_INIT        (0x00010000)
#define SHDISP_DEV_TYPE_MASK        (0x000103ff)
#define SHDISP_DEV_TYPE_ALS_MASK    (SHDISP_DEV_TYPE_ALS_APP|SHDISP_DEV_TYPE_ALS_BKL|SHDISP_DEV_TYPE_ALS_LUX|SHDISP_DEV_TYPE_DIAG)
/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DEV_STATE_NOINIT = 0,
    SHDISP_DEV_STATE_OFF,
    SHDISP_DEV_STATE_ON,
    NUM_SHDISP_DEV_STATE
};
/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_pm_init(struct shdisp_boot_context *shdisp_boot_ctx);
int shdisp_pm_clmr_power_manager(int type, int state);
int shdisp_pm_bdic_power_manager(int type, int state);
int shdisp_pm_psals_power_manager(int type, int state);
int shdisp_pm_is_als_active(void);
int shdisp_pm_is_ps_active(void);
int shdisp_pm_is_bdic_active(void);
int shdisp_pm_is_clmr_on(void);
void shdisp_pm_set_als_sensor_param(int als_mode);
void shdisp_pm_get_als_sensor_param(int *als_mode);
int  shdisp_pm_check_bdic_practical(void);
void shdisp_pm_bdic_shutdown(void);
int  shdisp_pm_bdic_resume(void);
int shdisp_pm_is_bkl_active(void);
unsigned long shdisp_pm_get_clmr_users(void);
void shdisp_pm_power_manager_users_clear(void);
#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_pm_power_manager_users_dump(void);
#endif /* CONFIG_ANDROID_ENGINEERING */
void shdisp_pm_psals_error_power_off(void);
int shdisp_pm_psals_error_power_recovery(void);
#endif /* SHDISP_PM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
