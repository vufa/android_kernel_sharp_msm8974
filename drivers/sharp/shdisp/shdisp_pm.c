/* drivers/sharp/shdisp/shdisp_pm.c  (Display Driver)
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
#include <linux/spi/spi.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pwm.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>


#include <sharp/shdisp_kerl.h>

#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_clmr.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_NAME "shdisp"
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
struct shdisp_psals_status {
    int power_status;
    int sensor_state;
    int als_mode;
    unsigned long users;
};
struct shdisp_pm_context {
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    struct shdisp_prox_params prox_params;
    struct shdisp_clmr_status clmr_status;
    struct shdisp_bdic_status bdic_status;
    struct shdisp_psals_status psals_status;
};
static struct shdisp_pm_context shdisp_pm_ctx;
static struct shdisp_pm_context shdisp_pm_ctx_recovery;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int  shdisp_pm_bdic_set_active(void);
static int  shdisp_pm_bdic_set_standby(void);
static void shdisp_pm_psals_get_state(int *state);
static void shdisp_pm_psals_set_state(int state);
static int  shdisp_pm_psals_power_on(void);
static int  shdisp_pm_psals_power_off(void);
static int  shdisp_pm_psals_ps_init(void);
static int  shdisp_pm_psals_ps_deinit(void);
static int  shdisp_pm_psals_als_init(void);
static int  shdisp_pm_psals_als_deinit(void);
static void shdisp_pm_clmr_power_notice(void);
extern void shdisp_api_lcdc_fwlog_timer_ctl(void);
extern void shdisp_lcdc_image_preptg_dump_ctl( void );
extern void shdisp_psals_recovery_subscribe( void );
extern void shdisp_psals_recovery_unsubscribe(void);

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_pm_init                                                            */
/* ------------------------------------------------------------------------- */
void shdisp_pm_init(struct shdisp_boot_context *shdisp_boot_ctx)
{
    memcpy(&(shdisp_pm_ctx.photo_sensor_adj),   &(shdisp_boot_ctx->photo_sensor_adj),   sizeof(struct shdisp_photo_sensor_adj));
    memcpy(&(shdisp_pm_ctx.clmr_status),        &(shdisp_boot_ctx->clmr_status),        sizeof(struct shdisp_clmr_status));
    memcpy(&(shdisp_pm_ctx.bdic_status),        &(shdisp_boot_ctx->bdic_status),        sizeof(struct shdisp_bdic_status));
    memset(&(shdisp_pm_ctx.prox_params), 0, sizeof( struct shdisp_prox_params));

    shdisp_pm_ctx.psals_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx.psals_status.sensor_state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF;
    shdisp_pm_ctx.psals_status.als_mode     = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    shdisp_pm_ctx.psals_status.users        = SHDISP_DEV_TYPE_NONE;

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.psals_status.users        = SHDISP_DEV_TYPE_NONE;
}
/* ------------------------------------------------------------------------- */
/* shdisp_pm_clmr_power_manager                                              */
/* ------------------------------------------------------------------------- */
int shdisp_pm_clmr_power_manager(int type, int state)
{
    int ret;
    unsigned long users_wk;

    if (shdisp_pm_ctx.clmr_status.clmr_is_exist != SHDISP_CLMR_IS_EXIST) {
        SHDISP_ERR("clmr does not exist.\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("in type:0x%08X, state:%s, clmr_status:%s, users:0x%08X\n", type, ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"),
            (shdisp_pm_ctx.clmr_status.power_status ? ((shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) ? "off":"on") : "noinit"),
            (int)shdisp_pm_ctx.clmr_status.users
        );

#if 0   /* temporaly spec clmr do not OFF */
    if (state == SHDISP_DEV_STATE_OFF) {
        shdisp_pm_ctx.clmr_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
        SHDISP_TRACE("out clmr_status:%d, users:0x%08X\n", shdisp_pm_ctx.clmr_status.power_status, (int)shdisp_pm_ctx.clmr_status.users);
        return SHDISP_RESULT_SUCCESS;
    }
#endif

    if (shdisp_pm_ctx.clmr_status.power_status != state) {
        if (state == SHDISP_DEV_STATE_ON) {
            ret = shdisp_clmr_api_power_on();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_clmr_api_power_on.\n");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.clmr_status.power_status = state;
            shdisp_clmr_api_init_fw_lcae();
            shdisp_pm_clmr_power_notice();
        }
        else {
            users_wk = shdisp_pm_ctx.clmr_status.users;
            users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_clmr_api_power_off();
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_clmr_api_power_off.\n");
                    return SHDISP_RESULT_FAILURE;
                }
                shdisp_pm_ctx.clmr_status.power_status = state;
                shdisp_pm_clmr_power_notice();
            }
        }
    }
    if (state == SHDISP_DEV_STATE_ON) {
        shdisp_pm_ctx.clmr_status.users |= (unsigned long)(type & SHDISP_DEV_TYPE_MASK);
    }
    else {
        shdisp_pm_ctx.clmr_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
    }
    SHDISP_TRACE("out clmr_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.clmr_status.power_status ? ((shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) ? "off":"on") : "noinit"),
            (int)shdisp_pm_ctx.clmr_status.users
        );
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_clmr_on                                                      */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_clmr_on(void)
{
    return shdisp_pm_ctx.clmr_status.power_status;
}

/* ------------------------------------------------------------------------- */
/* shdisp_shdisp_pm_clmr_power_notice                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_pm_clmr_power_notice(void)
{
#if defined (CONFIG_ANDROID_ENGINEERING)
    
    shdisp_api_lcdc_fwlog_timer_ctl();
#endif /* defined (CONFIG_ANDROID_ENGINEERING) */
    shdisp_lcdc_image_preptg_dump_ctl();
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_power_manager                                              */
/* ------------------------------------------------------------------------- */
int shdisp_pm_bdic_power_manager(int type, int state)
{
    int ret;
    unsigned long users_wk;

    if (shdisp_pm_ctx.clmr_status.clmr_is_exist != SHDISP_CLMR_IS_EXIST) {
        SHDISP_ERR("clmr does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_ctx.bdic_status.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_TRACE("in type:0x%08X, state:%s, bdic_status:%s, users:0x%08X\n", type, ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"),
            (shdisp_pm_ctx.bdic_status.power_status ? ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );

    if (shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("cannot bdic %s because clmr power off\n", ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"));
        return SHDISP_RESULT_FAILURE;
    }

#if 0   /* temporaly spec bdic do not STANDBY */
    if (state == SHDISP_DEV_STATE_OFF) {
        shdisp_pm_ctx.bdic_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
        SHDISP_TRACE("out bdic_status:%d, users:0x%08X\n", shdisp_pm_ctx.bdic_status.power_status, (int)shdisp_pm_ctx.bdic_status.users);
        return SHDISP_RESULT_SUCCESS;
    }
#endif

    if (shdisp_pm_ctx.bdic_status.power_status != state) {
        if (state == SHDISP_DEV_STATE_ON) {
            ret = shdisp_pm_bdic_set_active();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_active.\n");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.bdic_status.power_status = state;
        }
        else {
            users_wk = shdisp_pm_ctx.bdic_status.users;
            users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_bdic_set_standby();
                shdisp_pm_ctx.bdic_status.power_status = state;
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_standby.\n");
                    return SHDISP_RESULT_FAILURE;
                }
            }
        }
    }
    if (state == SHDISP_DEV_STATE_ON) {
        shdisp_pm_ctx.bdic_status.users |= (unsigned long)(type & SHDISP_DEV_TYPE_MASK);
    }
    else {
        shdisp_pm_ctx.bdic_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
    }
    SHDISP_TRACE("out bdic_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.bdic_status.power_status ? ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_manager                                             */
/* ------------------------------------------------------------------------- */
int shdisp_pm_psals_power_manager(int type, int state)
{
    int ret;
    unsigned long users_wk, als_users_wk;

    SHDISP_TRACE("in type:0x%08X, state:%s, psals_status:%s, users:0x%08X\n", type, ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"),
            (shdisp_pm_ctx.psals_status.power_status ? ((shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.psals_status.users
        );

    if (shdisp_pm_ctx.clmr_status.clmr_is_exist != SHDISP_CLMR_IS_EXIST) {
        SHDISP_ERR("clmr does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }

#ifndef SHDISP_USE_LEDC
    if (shdisp_pm_ctx.bdic_status.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }
#endif  /* SHDISP_USE_LEDC */

    if (shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("cannot sensor %s because clmr power off\n", ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"));
        return SHDISP_RESULT_FAILURE;
    }

#ifndef SHDISP_USE_LEDC
    if (shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("cannot sensor %s because bdic power off\n", ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"));
        return SHDISP_RESULT_FAILURE;
    }
#endif  /* SHDISP_USE_LEDC */

    if ((type != SHDISP_DEV_TYPE_PS) && ((type & SHDISP_DEV_TYPE_ALS_MASK) == SHDISP_DEV_TYPE_NONE)) {
        return SHDISP_RESULT_FAILURE;
    }

    als_users_wk = (unsigned long)(shdisp_pm_ctx.psals_status.users & SHDISP_DEV_TYPE_ALS_MASK);
    als_users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_ALS_MASK));

    if (shdisp_pm_ctx.psals_status.power_status != state) {
        if ((state == SHDISP_DEV_STATE_ON) && (shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_INIT)) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_DEV_STATE_ON;
        }
        else if ((state == SHDISP_DEV_STATE_INIT) && (shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_ON)) {
            ;
        }
        else if ((state == SHDISP_DEV_STATE_ON) || (state == SHDISP_DEV_STATE_INIT)) { 
            ret = shdisp_pm_psals_power_on();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_on.\n");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.psals_status.power_status = state;
#ifndef SHDISP_USE_LEDC
            if(shdisp_pm_ctx_recovery.psals_status.users == SHDISP_DEV_TYPE_NONE)
            {
                shdisp_psals_recovery_subscribe();
            }
#endif /* SHDISP_USE_LEDC */
        }
        else if (state == SHDISP_DEV_STATE_OFF) {
            if (type == SHDISP_DEV_TYPE_PS) {
                shdisp_pm_psals_ps_deinit();
            }
            else {
                if (als_users_wk == SHDISP_DEV_TYPE_NONE) {
                    shdisp_pm_psals_als_deinit();
                }
            }
            users_wk = shdisp_pm_ctx.psals_status.users;
            users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_psals_power_off();
#ifndef SHDISP_USE_LEDC
                if(shdisp_pm_ctx_recovery.psals_status.users == SHDISP_DEV_TYPE_NONE)
                {
                    shdisp_psals_recovery_unsubscribe();
                }
#endif /* SHDISP_USE_LEDC */
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_off.\n");
                    return SHDISP_RESULT_FAILURE;
                }
                shdisp_pm_ctx.psals_status.power_status = state;
            }
        }
    }

    if (state == SHDISP_DEV_STATE_ON) {
        shdisp_pm_ctx.psals_status.users |= (unsigned long)(type & SHDISP_DEV_TYPE_MASK);
        if (type == SHDISP_DEV_TYPE_PS) {
            shdisp_pm_psals_ps_init();
        }
        else {
            if (als_users_wk == SHDISP_DEV_TYPE_NONE) {
                shdisp_pm_psals_als_init();
            }
        }
    }
    else if(state == SHDISP_DEV_STATE_OFF) {
        shdisp_pm_ctx.psals_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
    }

    SHDISP_TRACE("out psals_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.psals_status.power_status ? ((shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.psals_status.users
        );

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_bdic_active                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_bdic_active(void)
{
    return shdisp_pm_ctx.bdic_status.power_status;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_active(void)
{
    int ret;

    SHDISP_TRACE("in bdic_status:%d\n", shdisp_pm_ctx.bdic_status.power_status);

    ret = shdisp_bdic_PD_set_active(shdisp_pm_ctx.bdic_status.power_status);

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");

    ret = shdisp_bdic_PD_set_standby();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_get_state                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_pm_psals_get_state(int *state)
{
    *state = shdisp_pm_ctx.psals_status.sensor_state;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_set_state                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_pm_psals_set_state(int state)
{
    shdisp_pm_ctx.psals_status.sensor_state = state;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_on();
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_off();
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_init                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_init(void)
{
    int ret;
    int state;

    SHDISP_TRACE("in\n");

    shdisp_pm_psals_get_state(&state);
    ret = shdisp_bdic_PD_psals_ps_init(&state);
    shdisp_pm_psals_set_state(state);

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_deinit                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_deinit(void)
{
    int ret;
    int state;

    SHDISP_TRACE("in\n");

    shdisp_pm_psals_get_state(&state);
    ret = shdisp_bdic_PD_psals_ps_deinit(&state);
    shdisp_pm_psals_set_state(state);

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_init                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_init(void)
{
    int ret;
    int state;

    SHDISP_TRACE("in\n");

    shdisp_pm_psals_get_state(&state);
    ret = shdisp_bdic_PD_psals_als_init(&state);
    shdisp_pm_psals_set_state(state);

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;

}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_deinit                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_deinit(void)
{
    int ret;
    int state;

    SHDISP_TRACE("in\n");

    shdisp_pm_psals_get_state(&state);
    ret = shdisp_bdic_PD_psals_als_deinit(&state);
    shdisp_pm_psals_set_state(state);

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_shutdown                                                   */
/* ------------------------------------------------------------------------- */

void shdisp_pm_bdic_shutdown(void)
{
    shdisp_pm_ctx_recovery.bdic_status.power_status = shdisp_pm_ctx.bdic_status.power_status;
    shdisp_pm_ctx_recovery.bdic_status.users        = shdisp_pm_ctx.bdic_status.users;

    shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_MASK, SHDISP_DEV_STATE_OFF);

    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);

    shdisp_pm_ctx.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_resume                                                     */
/* ------------------------------------------------------------------------- */
int  shdisp_pm_bdic_resume(void)
{
    if (shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("cannot bdic on because clmr power off\n");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);

    shdisp_pm_bdic_power_manager(shdisp_pm_ctx_recovery.bdic_status.users, SHDISP_DEV_STATE_ON);

    shdisp_pm_ctx_recovery.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.bdic_status.users        = SHDISP_DEV_TYPE_NONE;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_error_power_off                                           */
/* ------------------------------------------------------------------------- */
void shdisp_pm_psals_error_power_off(void)
{
    SHDISP_TRACE("in\n");
    shdisp_pm_ctx_recovery.psals_status.power_status = shdisp_pm_ctx.psals_status.power_status;
    shdisp_pm_ctx_recovery.psals_status.users        = shdisp_pm_ctx.psals_status.users;

    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_MASK, SHDISP_DEV_STATE_OFF);
    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_STATE_OFF);

    shdisp_pm_ctx.psals_status.power_status = SHDISP_DEV_STATE_NOINIT;
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_error_power_recovery                                      */
/* ------------------------------------------------------------------------- */
int shdisp_pm_psals_error_power_recovery(void)
{
    int ret;
    unsigned long user;

    SHDISP_TRACE("in\n");
    user = shdisp_pm_ctx_recovery.psals_status.users & SHDISP_DEV_TYPE_PS;
    if (user == SHDISP_DEV_TYPE_PS) {
        ret = shdisp_pm_psals_power_manager(user, SHDISP_DEV_STATE_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE\n");
            return SHDISP_RESULT_FAILURE;
        }
    }

    user = shdisp_pm_ctx_recovery.psals_status.users & SHDISP_DEV_TYPE_ALS_MASK;
    if (user != SHDISP_DEV_TYPE_NONE) {
        ret = shdisp_pm_psals_power_manager(user, SHDISP_DEV_STATE_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE\n");
            return SHDISP_RESULT_FAILURE;
        }
    }

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.psals_status.users        = SHDISP_DEV_TYPE_NONE;

    SHDISP_TRACE("out ret = SHDISP_RESULT_SUCCESS\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_als_active                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_als_active(void)
{
    unsigned long user;

    user = shdisp_pm_ctx.psals_status.users & SHDISP_DEV_TYPE_ALS_MASK;
    if (user == SHDISP_DEV_TYPE_NONE) {
        return SHDISP_DEV_STATE_OFF;
    }
    return SHDISP_DEV_STATE_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_ps_active                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_ps_active(void)
{
    unsigned long user;

    user = shdisp_pm_ctx.psals_status.users & SHDISP_DEV_TYPE_PS;
    if (user == SHDISP_DEV_TYPE_NONE) {
        return SHDISP_DEV_STATE_OFF;
    }
    return SHDISP_DEV_STATE_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_bkl_active                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_bkl_active(void)
{
    unsigned long user;

    user = shdisp_pm_ctx.bdic_status.users & SHDISP_DEV_TYPE_BKL;
    if (user == SHDISP_DEV_TYPE_NONE) {
        return SHDISP_DEV_STATE_OFF;
    }
    return SHDISP_DEV_STATE_ON;
}

#if 0
/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_bias_als_adj                                               */
/* ------------------------------------------------------------------------- */
static unsigned char shdisp_pm_bdic_bias_als_adj(void)
{
    unsigned char als_tmp;
    unsigned char als_shift;

    als_tmp = shdisp_pm_ctx.photo_sensor_adj.als_adjust[0].als_shift & 0x1f;

    switch (als_tmp) {
    case 0x1E:
        als_shift = 0x00;
        break;
    case 0x1F:
        als_shift = 0x01;
        break;
    case 0x0E:
        als_shift = 0x0F;
        break;
    case 0x0F:
        als_shift = 0x0F;
        break;
    default:
        als_shift = als_tmp + 2;
        break;
    }
    return als_shift;
}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_pm_get_clmr_users                                                  */
/* ------------------------------------------------------------------------- */
unsigned long shdisp_pm_get_clmr_users(void)
{
    return shdisp_pm_ctx.clmr_status.users;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_power_manager_users_clear                                       */
/* ------------------------------------------------------------------------- */
void shdisp_pm_power_manager_users_clear(void)
{
    SHDISP_ERR("clear devices status and users bit!!\n" );
    shdisp_pm_ctx.psals_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx.psals_status.users        = SHDISP_DEV_TYPE_NONE;

    shdisp_pm_ctx.bdic_status.power_status  = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx.bdic_status.users         = SHDISP_DEV_TYPE_NONE;

    shdisp_pm_ctx.clmr_status.power_status  = SHDISP_DEV_STATE_OFF;
    shdisp_pm_ctx.clmr_status.users         = SHDISP_DEV_TYPE_NONE;
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_pm_power_manager_users_dump                                        */
/* ------------------------------------------------------------------------- */
void shdisp_pm_power_manager_users_dump(void)
{
    printk("[SHDISP] psals_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.psals_status.power_status ? ((shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.psals_status.users
        );
    printk("[SHDISP] bdic_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.bdic_status.power_status ? ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );
    printk("[SHDISP] clmr_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.clmr_status.power_status ? ((shdisp_pm_ctx.clmr_status.power_status == SHDISP_DEV_STATE_OFF) ? "off":"on") : "noinit"),
            (int)shdisp_pm_ctx.clmr_status.users
        );
}
#endif /* CONFIG_ANDROID_ENGINEERING */

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
