/* drivers/sharp/shdisp/shdisp_bl69y6.c  (Display Driver)
 *
 * Copyright (C) 2013-2014 SHARP CORPORATION
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
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_bl69y6.h"
#include "shdisp_dbg.h"

#if defined(CONFIG_MACH_LYNX_DL40)
  #include "./data/shdisp_bl69y6_data_dl40.h"
#elif defined(CONFIG_MACH_LYNX_DL45)
  #include "./data/shdisp_bl69y6_data_dl45.h"
#elif defined(CONFIG_MACH_DECKARD_AS97)
  #include "./data/shdisp_bl69y6_data_as97.h"
#elif defined(CONFIG_MACH_ATK)
  #include "./data/shdisp_bl69y6_data_pa21.h"
#elif defined(CONFIG_MACH_TBS)
  #include "./data/shdisp_bl69y6_data_pa23.h"
#elif defined(CONFIG_MACH_DECKARD_AS87)
  #include "./data/shdisp_bl69y6_data_as87.h"
#else /* CONFIG_MACH_DEFAULT */
  #include "./data/shdisp_bl69y6_data_default.h"
#endif /* CONFIG_MACH_ */

#include "shdisp_pm.h"

#include <sharp/sh_boot_manager.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_BKL_MODE_OFF                (0)
#define SHDISP_BDIC_BKL_MODE_FIX                (1)
#define SHDISP_BDIC_BKL_MODE_AUTO               (2)

#define SHDISP_BDIC_BKL_DTV_OFF                 (0)
#define SHDISP_BDIC_BKL_DTV_ON                  (1)

#define SHDISP_BDIC_BKL_EMG_OFF                 (0)
#define SHDISP_BDIC_BKL_EMG_ON                  (1)

#define SHDISP_BDIC_BKL_ECO_OFF                 (0)
#define SHDISP_BDIC_BKL_ECO_ON                  (1)

#define SHDISP_BDIC_BKL_CHG_OFF                 (0)
#define SHDISP_BDIC_BKL_CHG_ON                  (1)

#define SHDISP_BDIC_TRI_LED_MODE_NORMAL         (0)
#define SHDISP_BDIC_TRI_LED_MODE_BLINK          (1)
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY        (2)
#define SHDISP_BDIC_TRI_LED_MODE_HISPEED        (3)
#define SHDISP_BDIC_TRI_LED_MODE_STANDARD       (4)
#define SHDISP_BDIC_TRI_LED_MODE_BREATH         (5)
#define SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH    (6)
#define SHDISP_BDIC_TRI_LED_MODE_WAVE           (7)
#define SHDISP_BDIC_TRI_LED_MODE_FLASH          (8)
#define SHDISP_BDIC_TRI_LED_MODE_AURORA         (9)
#define SHDISP_BDIC_TRI_LED_MODE_RAINBOW       (10)


#define SHDISP_BDIC_I2C_W_START             0x01
#define SHDISP_BDIC_I2C_R_START             0x02
#define SHDISP_BDIC_I2C_R_TIMRE_ST          0x10
#define SHDISP_BDIC_I2C_ADO_UPDATE_MASK     0x60
#define SHDISP_BDIC_I2C_SEND_WAIT           1000
#define SHDISP_BDIC_I2C_SEND_RETRY          200


#define SHDISP_BDIC_I2C_SEND_RETRY_WAIT     100

#define SHDISP_FW_STACK_EXCUTE

#define SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE    (ARRAY_SIZE(shdisp_bdic_bkl_lux_tbl[0]))

#define SHDISP_BDIC_REGSET(x)         (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x), 0, 0))
#define SHDISP_PSALS_REGSET(x)        (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x), bkl_mode, lux_mode))

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_BDIC_STR,
    SHDISP_BDIC_SET,
    SHDISP_BDIC_CLR,
    SHDISP_BDIC_RMW,
    SHDISP_BDIC_STRM,
    SHDISP_BDIC_WAIT,
    SHDISP_ALS_STR,
    SHDISP_ALS_RMW,
    SHDISP_ALS_STRM,
    SHDISP_ALS_STRMS,
    SHDISP_BKL_MODE,
    SHDISP_LUX_MODE
};

typedef struct {
    unsigned char   addr;
    unsigned char   flg;
    unsigned char   data;
    unsigned char   mask;
    unsigned long   wait;
} shdisp_bdicRegSetting_t;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_seq_backlight_off(void);
static void shdisp_bdic_seq_backlight_fix_on(int param);
static void shdisp_bdic_seq_backlight_auto_on(int param);
static int  shdisp_bdic_seq_led_off(void);
static int  shdisp_bdic_seq_led_normal_on(unsigned char color);
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count);
#if defined(CONFIG_MACH_LYNX_DL45)
static void shdisp_bdic_seq_led_high_speed_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_standard_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_breath_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_long_breath_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_wave_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_flash_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_aurora_on(int interval, int count);
static void shdisp_bdic_seq_led_rainbow_on(int interval, int count);
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux);
static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg);
static void shdisp_bdic_seq_psals_active(unsigned long dev_type);
static void shdisp_bdic_seq_psals_standby(unsigned long dev_type);
static void shdisp_bdic_seq_ps_background(unsigned long state);
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void);
static void shdisp_bdic_LD_LCD_BKL_emg_on(void);
static void shdisp_bdic_LD_LCD_BKL_emg_off(void);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);
static void shdisp_bdic_LD_LCD_BKL_eco_on(void);
static void shdisp_bdic_LD_LCD_BKL_eco_off(void);
static void shdisp_bdic_LD_LCD_BKL_chg_on(void);
static void shdisp_bdic_LD_LCD_BKL_chg_off(void);

static void shdisp_bdic_PD_LCD_POS_PWR_on(void);
static void shdisp_bdic_PD_LCD_POS_PWR_off(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_on(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_off(void);
static void shdisp_bdic_PD_LCD_VO2_ON(void);
static void shdisp_bdic_PD_LCD_VO2_OFF(void);
static void shdisp_bdic_seq_bdic_active_for_led(int);
static void shdisp_bdic_seq_bdic_standby_for_led(int);
static void shdisp_bdic_seq_clmr_off_for_led(int);
static int  shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size, unsigned char bkl_mode, unsigned char lux_mode);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
static void shdisp_bdic_PD_TRI_LED_set_anime(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig(void);
static void shdisp_bdic_PWM_set_value(int pwm_param);
static void shdisp_bdic_bkl_adj_clmr_write_cmd(int mode, unsigned char* value, int len );


static void shdisp_bdic_PD_REG_OPT_set_value( int table );
static int  shdisp_bdic_PHOTO_SENSOR_lux_init(void);
static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value);
static void shdisp_bdic_PD_REG_ADO_FW_get_opt(unsigned short *value);

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_photo_sensor_change_range( int bkl_opt );
static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_als_clmr_write_cmd(unsigned char reg, unsigned char val);
static int shdisp_bdic_als_clmr_msk_write_cmd(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_bdic_als_clmr_read_cmd(unsigned char reg, unsigned char *val);
static int shdisp_bdic_als_clmr_burst_write_cmd(unsigned char addr, unsigned char *val, unsigned char size);

static int shdisp_bdic_PD_ps_req_mask( unsigned char *remask );
static int shdisp_bdic_PD_ps_req_restart( unsigned char remask );
static int shdisp_bdic_PD_psals_write_threshold(shdisp_bdicRegSetting_t *pThresh);
static int shdisp_bdic_register_driver(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_bdic_state_str s_state_str;

static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_bkl_param;
static int shdisp_bdic_bkl_before_mode;

static int shdisp_bdic_dtv;

static int shdisp_bdic_emg;

static int shdisp_bdic_eco;

static int shdisp_bdic_chg;

static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;
static int shdisp_bdic_tri_led_count;

static int shdisp_bdic_i2c_errflag;

static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_0   },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_0   }
};

static unsigned int shdisp_bdic_irq_fac = 0;
static unsigned int shdisp_bdic_irq_fac_exe = 0;

static int  shdisp_bdic_irq_prioriy[SHDISP_IRQ_MAX_KIND];

static unsigned char shdisp_backup_irq_reg[6];
static unsigned char shdisp_backup_irq_photo_req[3];

static int shdisp_bdic_irq_det_flag = 0;

static unsigned char lux_mode_recovery;
static unsigned char bkl_mode_recovery;
static int psals_recovery_flag = 0;
static const shdisp_bdicRegSetting_t shdisp_bdic_init[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM4,             SHDISP_BDIC_STR,    BDIC_REG_SYSTEM4_VAL,       0xFF,      0}
    ,{BDIC_REG_I2C_TIMER,           SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
    ,{BDIC_REG_I2C_SYS,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_I2C_DATA0,           SHDISP_BDIC_STR,    0x72,                       0xFF,      0}

    ,{BDIC_REG_OPT_MODE,            SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_SENSOR,              SHDISP_BDIC_STR,    0x20,                       0xFF,      0}
    ,{BDIC_REG_SENSOR2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_STR,    0xE0,                       0xFF,      0}

    ,{BDIC_REG_MODE_M1,             SHDISP_BDIC_STR,    0x03,                       0xFF,      0}
    ,{BDIC_REG_PSDATA_SET,          SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_PS_HT_LSB,           SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{BDIC_REG_PS_HT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_LSB,           SHDISP_BDIC_STR,    0x0F,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_DCDC1_VLIM,          SHDISP_BDIC_STR,    BDIC_REG_DCDC1_VLIM_VAL,    0xFF,      0}
    ,{BDIC_REG_DCDC_SYS,            SHDISP_BDIC_STR,    BDIC_REG_DCDC_SYS_VAL,      0xFF,      0}
    ,{BDIC_REG_DCDC2_VO,            SHDISP_BDIC_STR,    BDIC_REG_DCDC2_VO_VAL,      0xFF,      0}
    ,{BDIC_REG_DCDC2_SYS,           SHDISP_BDIC_STR,    0x0C,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_RESERVE,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_CPM,                 SHDISP_BDIC_STR,    0x0E,                       0xFF,      0}
    ,{BDIC_REG_TEST61,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_GPOD1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GDIR1,               SHDISP_BDIC_STR,    0x33,                       0xFF,      0}
    ,{BDIC_REG_GPPD1,               SHDISP_BDIC_STR,    0x08,                       0xFF,      0}
    ,{BDIC_REG_GPOMODE1,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPOMODE2,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_INT_CTRL,            SHDISP_BDIC_STR,    0x04,                       0xFF,      0}
    ,{BDIC_REG_DETECTOR,            SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMR2,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMR3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF2,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_active[] = {
     {BDIC_REG_SYSTEM2 ,            SHDISP_BDIC_SET,    0x80,                       0x80,   1000}
    ,{BDIC_REG_M1LED,               SHDISP_BDIC_STR,    BDIC_REG_M1LED_VAL,         0xFF,      0}
    ,{BDIC_REG_M2LED,               SHDISP_BDIC_STR,    BDIC_REG_M2LED_VAL,         0xFF,      0}
    ,{BDIC_REG_SYSTEM4,             SHDISP_BDIC_RMW,    0x05,                       0x55,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_standby[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x80,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on[] = {
#if defined(CONFIG_SHDISP_PANEL_RYOMA)
     {BDIC_REG_CPM,                 SHDISP_BDIC_SET,    0x01,                       0x01,   2000}
#else
     {BDIC_REG_CPM,                 SHDISP_BDIC_SET,    0x01,                       0x01,   1000}
#endif
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsn_off[] = {
     {BDIC_REG_CPM,                 SHDISP_BDIC_CLR,    0x00,                       0x01,   1000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsp_on[] = {
#if defined(CONFIG_SHDISP_PANEL_RYOMA)
     {BDIC_REG_DCDC2_SYS,           SHDISP_BDIC_SET,    0x40,                       0x40,   2000}
#else
     {BDIC_REG_DCDC2_SYS,           SHDISP_BDIC_SET,    0x40,                       0x40,   1000}
#endif
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsp_off[] = {
     {BDIC_REG_DCDC2_SYS,           SHDISP_BDIC_CLR,    0x00,                       0x40,   2000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vo2_on[] = {
     {BDIC_REG_DCDC2_VO,            SHDISP_BDIC_STR,    BDIC_REG_DCDC2_VO_VAL,      0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vo2_off[] = {
     {BDIC_REG_DCDC2_VO,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_on[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    BDIC_REG_SYSTEM2_BKL,       0x03,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_post_start[] = {
     {BDIC_REG_SYSTEM4,             SHDISP_BDIC_RMW,    0x04,                       0x05,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_off[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x03,  15000}
    ,{BDIC_REG_SYSTEM4,             SHDISP_BDIC_RMW,    0x01,                       0x05,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_CH0_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_SET,    0x01,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_SYSTEM3,             SHDISP_BDIC_SET,    0x01,                       0x01,      0}
    ,{BDIC_REG_TIMERSTART,          SHDISP_BDIC_SET,    0x01,                       0x01,   6000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_LPOSC,               SHDISP_BDIC_STR,    0x01,                       0x01,      0}
};

#if defined(CONFIG_MACH_LYNX_DL45)
static const shdisp_bdicRegSetting_t shdisp_bdic_led_high_speed_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_standard_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_breath_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_long_breath_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_wave_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_flash_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_aurora_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_rainbow_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x46,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_TIMERSTART,          SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_CLR,    0x00,                       0x01,   5500}
    ,{BDIC_REG_LPOSC,               SHDISP_BDIC_STR,    0x00,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_on[] = {
     {BDIC_REG_SENSOR,              SHDISP_BDIC_SET,    0x10,                       0x10,   5000}
    ,{BDIC_REG_GIMR3,               SHDISP_BDIC_SET,    0x08,                       0x08,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_off[] = {
     {BDIC_REG_SENSOR,              SHDISP_BDIC_CLR,    0x00,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_psals_init[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x1C,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,   1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xE0,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_psals_standby[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,      0}
    ,{CLMR_SET_BKL_LUT_MODE,        SHDISP_BKL_MODE,    0x00,                       0x04,      0}
    ,{BDIC_REG_PS_HT_LSB,           SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_PS_HT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_LSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_off[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,       0}
    ,{SENSOR_REG_PS_LT_LSB,         SHDISP_ALS_STRMS,   0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_LT_MSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_HT_LSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_HT_MSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xE0,                       0xFF,    1000}
    ,{CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x00,                       0x01,       0}
    ,{BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{BDIC_REG_GIMR3,               SHDISP_BDIC_CLR,    0x00,                       0x20,       0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,   20000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_off[] = {
     {BDIC_REG_GIMR3,               SHDISP_BDIC_CLR,    0x00,                       0x28,       0}
    ,{BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x00,                       0x01,       0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_SET,    0x60,                       0x60,       0}
    ,{CLMR_SET_BKL_LUT_MODE,        SHDISP_BKL_MODE,    0x04,                       0x04,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x80,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_SET,    0x60,                       0x60,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x20,                       0xF8,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STR,     0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STR,     0xEC,                       0xFF,       0}
    ,{SENSOR_REG_PS_LT_LSB,         SHDISP_ALS_STRMS,   0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_LT_MSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_HT_LSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_PS_HT_MSB,         SHDISP_ALS_STRM,    0x00,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xC0,                       0xFF,    1000}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_CLR,    0x00,                       0x60,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_on[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x18,                       0xF8,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STR,     0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STR,     0x00,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,    1000}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_CLR,    0x00,                       0x60,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_off[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x1C,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,    1000}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_CLR,    0x00,                       0x60,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_on[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_SET,    0x60,                       0x60,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x0C,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x24,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xCC,                       0xFF,    1000}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_CLR,    0x00,                       0x60,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_clmr[] = {
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_clmr_wait[] = {
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,       0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,   20000}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_bdic[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_bdic_wait[] = {
     {BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,   20000}
};

#define shdisp_bdic_als_deinit_ps_off   (shdisp_bdic_ps_deinit_als_off)

static const shdisp_bdicRegSetting_t shdisp_bdic_als_deinit_ps_on[] = {
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x00,                       0x01,       0}
    ,{BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_SET,    0x60,                       0x60,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x0C,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xEC,                       0xFF,    1000}
    ,{BDIC_REG_I2C_START,           SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{BDIC_REG_GIMR3,               SHDISP_BDIC_CLR,    0x00,                       0x20,       0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,   20000}
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_boot_init( void )
{

    SHDISP_TRACE("in\n")
    shdisp_bdic_bkl_mode        = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_param       = 0;

    shdisp_bdic_dtv             = SHDISP_BDIC_BKL_DTV_OFF;

    shdisp_bdic_emg             = SHDISP_BDIC_BKL_EMG_OFF;

    shdisp_bdic_eco             = SHDISP_BDIC_BKL_ECO_OFF;

    shdisp_bdic_chg             = SHDISP_BDIC_BKL_CHG_OFF;

    shdisp_bdic_tri_led_color    = 0;
    shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
    shdisp_bdic_tri_led_ontime   = 0;
    shdisp_bdic_tri_led_interval = 0;
    shdisp_bdic_tri_led_count    = 0;

    shdisp_bdic_register_driver();

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);

#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

    SHDISP_TRACE("out\n")
    return SHDISP_BDIC_IS_EXIST;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_bdic_exist                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_bdic_exist(int* bdic_is_exist)
{
    *bdic_is_exist = s_state_str.bdic_is_exist;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str)
{
    s_state_str.bdic_is_exist                   = state_str->bdic_is_exist;
    s_state_str.bdic_main_bkl_opt_mode_output   = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.bdic_main_bkl_opt_mode_ado      = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.shdisp_lux_change_level1        = SHDISP_LUX_CHANGE_LEVEL1;
    s_state_str.shdisp_lux_change_level2        = SHDISP_LUX_CHANGE_LEVEL2;
    s_state_str.clmr_is_exist                   = state_str->clmr_is_exist;

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    if (s_state_str.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
    }
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
    s_state_str.bdic_clrvari_index              = state_str->bdic_clrvari_index;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_on                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_on(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_on();
    SHDISP_DEBUG("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_off                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_off(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_off();
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_on(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_on();
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_off(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_off();
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_on                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_on(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_VO2_ON();
    SHDISP_DEBUG("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_off(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_PD_LCD_VO2_OFF();
    SHDISP_DEBUG("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_off(void)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_seq_backlight_off();
    SHDISP_DEBUG("out\n")
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_seq_backlight_fix_on(param);
    SHDISP_DEBUG("out\n")
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_auto_on                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_auto_on(int param)
{
    SHDISP_DEBUG("in\n")
    shdisp_bdic_seq_backlight_auto_on(param);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param)
{
    int mode = 0;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
            *param = shdisp_main_dtv_bkl_tbl[shdisp_bdic_bkl_param][mode];
        }
        else {
            *param = shdisp_main_bkl_tbl[shdisp_bdic_bkl_param][mode];
        }
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        *param = 0x100;
        break;

    default:
        *param = 0;
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;

    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode     = tmp->led_mode;
    shdisp_bdic_tri_led_color    = color;
    shdisp_bdic_tri_led_ontime   = tmp->ontime;
    shdisp_bdic_tri_led_interval = tmp->interval;
    shdisp_bdic_tri_led_count    = tmp->count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;


    SHDISP_DEBUG("shdisp_bdic_API_LCD_BKL_get_request  tmp->mode %d \n",tmp->mode)
    SHDISP_DEBUG("shdisp_bdic_API_LCD_BKL_get_request  tmp->param %d \n",tmp->param)

    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    else if (((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
                 (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_1))) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    else if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param == SHDISP_MAIN_BKL_PARAM_1) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param;
    }
    else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_on(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_off(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_on(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_off(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_off(void)
{
    int ret;
    ret = shdisp_bdic_seq_led_off();
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */

unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led )
{
    int i;
    unsigned char color = 0xFF;

    for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
        if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
            shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
            shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
            color = shdisp_triple_led_color_index_tbl[i].color;
            break;
        }
    }

    if (color == 0xFF) {
        if (tri_led->red > 1) {
            tri_led->red = 1;
        }
        if (tri_led->green > 1) {
            tri_led->green = 1;
        }
        if (tri_led->blue > 1) {
            tri_led->blue = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
            if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
                color = shdisp_triple_led_color_index_tbl[i].color;
                break;
            }
        }
        if (color == 0xFF) {
            color = 0;
        }
    }
    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    int ret;
    ret = shdisp_bdic_seq_led_normal_on(color);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_blink_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_firefly_on                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_firefly_on(color, ontime, interval, count);
    return;
}

#if defined(CONFIG_MACH_LYNX_DL45)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_high_speed_on                                     */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_high_speed_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_high_speed_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_HISPEED, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_standard_on                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_standard_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_standard_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_STANDARD, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_breath_on                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_breath_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_BREATH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_long_breath_on                                    */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_long_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_long_breath_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_LONG_BREATH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_wave_on                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_wave_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_wave_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_WAVE, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_flash_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_flash_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_flash_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_FLASH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_aurora_on                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_aurora_on(int interval, int count)
{
    shdisp_bdic_seq_led_aurora_on(SHDISP_BDIC_TRI_LED_INTERVAL_AURORA, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_rainbow_on                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_rainbow_on(int interval, int count)
{
    shdisp_bdic_seq_led_rainbow_on(SHDISP_BDIC_TRI_LED_INTERVAL_RAINBOW, count);
    return;
}
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_clrvari_index                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari )
{
    int i=0;

    for( i=0; i<SHDISP_COL_VARI_KIND ; i++) {
        if( (int)shdisp_clrvari_index[i] == clrvari )
            break;
    }
    if( i >= SHDISP_COL_VARI_KIND )
        i = 0;

    return i;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_lux                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(value, lux);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind                               */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode)
{
    if(s_state_str.bdic_main_bkl_opt_mode_ado == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
        *mode = SHDISP_LUX_MODE_LOW;
    }
    else {
        *mode = SHDISP_LUX_MODE_HIGH;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PHOTO_SENSOR_lux_init                                         */
/* ------------------------------------------------------------------------- */

static int  shdisp_bdic_PHOTO_SENSOR_lux_init(void)
{
    int           ret= SHDISP_RESULT_SUCCESS;

#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] shdisp_bdic_PHOTO_SENSOR_lux_init : start\n");
#endif

    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_OFF );

    shdisp_photo_sensor_change_range( SHDISP_BDIC_MAIN_BKL_OPT_LOW );

    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x03 );
    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );

    shdisp_bdic_IO_set_bit_reg( BDIC_REG_ADO_SYS, 0x01 );

    shdisp_bdic_PD_REG_OPT_set_value(SHDISP_BDIC_MAIN_BKL_OPT_LOW);

    shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );

    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_ON );

    shdisp_bdic_API_SENSOR_start(SHDISP_LUX_MODE_LOW);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char value = 0x00;
    int i2c_rtimer_restart_flg = 0;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);

    if (value & SHDISP_BDIC_I2C_R_START) {
        SHDISP_ERR("<OTHER> i2c read start.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (value & SHDISP_BDIC_I2C_R_TIMRE_ST) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : stop i2c rtimer.\n");
#endif
        i2c_rtimer_restart_flg = 1;
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_SYS_delay_us(1000);
    }

    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
        ret = shdisp_bdic_LD_i2c_write(msg);
        break;
    case SHDISP_BDIC_I2C_M_R:
        ret = shdisp_bdic_LD_i2c_read_mode0(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
        ret = shdisp_bdic_LD_i2c_read_mode1(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE2:
        ret = shdisp_bdic_LD_i2c_read_mode2(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = shdisp_bdic_LD_i2c_read_mode3(msg);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    if (i2c_rtimer_restart_flg == 1) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : restart i2c rtimer.\n");
#endif
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
        else {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x30);
        }
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
    }
    else {
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_I2C_start_judge                                           */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_bdic_API_I2C_start_judge(void)
{
    unsigned char value = 0x00;

    shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);

    return value;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_I2C_start_ctl                                             */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_I2C_start_ctl(int flg)
{
    if (flg == 1)  {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
    } else {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
    }

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_ALS_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_ALS_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }

    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
          ret = shdisp_bdic_als_clmr_write_cmd( msg->wbuf[0] , msg->wbuf[1] );
        break;
    case SHDISP_BDIC_I2C_M_R:
          ret = shdisp_bdic_als_clmr_read_cmd( msg->wbuf[0] , msg->rbuf );
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
        ret = shdisp_bdic_LD_i2c_read_mode1(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE2:
        ret = shdisp_bdic_LD_i2c_read_mode2(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = shdisp_bdic_LD_i2c_read_mode3(msg);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    ret = shdisp_bdic_IO_write_reg(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    } else {
        shdisp_SYS_bdic_i2c_cmd_reset();
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    ret = shdisp_bdic_IO_read_reg(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    } else {
        shdisp_SYS_bdic_i2c_cmd_reset();
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    unsigned char dummy=0;

    shdisp_bdic_IO_read_reg(BDIC_REG_GINF3, &dummy);

    if (dummy & 0x04) {
        return SHDISP_RESULT_SUCCESS;
    }
    else {
        return SHDISP_RESULT_FAILURE;
    }
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   value;

    printk("[SHDISP] BL69Y6 INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist              = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_output = %d. (0:low/1:high)\n", s_state_str.bdic_main_bkl_opt_mode_output);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_ado = %d. (0:low/1:high)\n", s_state_str.bdic_main_bkl_opt_mode_ado);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level1   = %d.\n", s_state_str.shdisp_lux_change_level1);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level2   = %d.\n", s_state_str.shdisp_lux_change_level2);
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_dtv                        = %d.\n", shdisp_bdic_dtv);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_eco                        = %d.\n", shdisp_bdic_eco);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);
    printk("[SHDISP] shdisp_bdic_tri_led_color              = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode               = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime             = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval           = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count              = %d.\n", shdisp_bdic_tri_led_count);

    for ( idx = 0; idx < NUM_SHDISP_MAIN_BKL_DEV_TYPE; idx++ ) {
        printk("[SHDISP] shdisp_bkl_priority_table[%d]       = (mode:%d, param:%d).\n", idx, shdisp_bkl_priority_table[idx].mode, shdisp_bkl_priority_table[idx].param);
    }

    for ( idx = 0; idx <= 0xFF; idx++ ) {
        shdisp_bdic_API_DIAG_read_reg(idx, &value);
        printk("[SHDISP] IR2E69Y6 reg = (add=0x%02x, value=0x%02x).\n",idx,value );
    }

    printk("[SHDISP] bdic_clrvari_index                     = %d.\n", s_state_str.bdic_clrvari_index );

    printk("[SHDISP] BL69Y6 INFO <<-\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_INFO_output                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    pbuf = (unsigned char*)kzalloc((((BDIC_REG_CH2_C - BDIC_REG_TIMERSTART + 1)/8)*8), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (((BDIC_REG_CH2_C - BDIC_REG_TIMERSTART + 1)/8)*8));
        return;
    }
    p = pbuf;
    for (idx = BDIC_REG_TIMERSTART; idx <= BDIC_REG_CH2_C; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] TRI-LED INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist      = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] shdisp_bdic_tri_led_color      = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode       = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime     = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval   = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count      = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] shdisp_bdic_clrvari_index      = %d.\n", s_state_str.bdic_clrvari_index);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x %02x\n", BDIC_REG_TIMERSTART, *p, *(p+1), *(p+2), *(p+3));
    p += 4;
    printk("[SHDISP] BDIC_REG_LED_SETTING   0x%2X: %02x %02x %02x %02x %02x %02x %02x\n", BDIC_REG_CH0_SET1, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED_CURRENT   0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", BDIC_REG_CH0_A, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7), *(p+8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED INFO <<-\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_psals_API_DBG_INFO_output                                          */
/* ------------------------------------------------------------------------- */

void shdisp_psals_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    SHDISP_DEBUG("PSALS SENSOR INFO ->>");
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc((((SENSOR_REG_D2_MSB+7)/8)*8), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (((SENSOR_REG_D2_MSB+7)/8)*8));
        return;
    }

    shdisp_SYS_delay_us(1000*1000);

    p = pbuf;
    for (idx = SENSOR_REG_COMMAND1; idx <= SENSOR_REG_D2_MSB; idx++ ) {
        *p = 0x00;
        shdisp_photo_sensor_IO_read_reg(idx, p);
    }
    p = pbuf;
    SHDISP_DEBUG("SENSOR_REG_DUMP 0x00: %02x %02x %02x %02x %02x %02x %02x %02x\n", *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    SHDISP_DEBUG("SENSOR_REG_DUMP 0x08: %02x %02x %02x %02x %02x %02x %02x %02x\n", *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    SHDISP_DEBUG("SENSOR_REG_DUMP 0x10: %02x %02x                              \n", *p, *(p+1));
    kfree(pbuf);

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_OFF);

    SHDISP_DEBUG("PSALS SENSOR INFO <<-\n");
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_set_reg                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_set_reg( int irq_type, int onoff )
{
    unsigned char   val1, val2,val3,val4,val5;

    switch ( irq_type ) {
    case SHDISP_IRQ_TYPE_ALS:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_ALS ){
            val1 = 0x01;
            if( onoff == SHDISP_IRQ_NO_MASK ){
                shdisp_bdic_PHOTO_SENSOR_lux_init();
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT,  0x14);
            }
        }
        break;

    case SHDISP_IRQ_TYPE_PS:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS ){
            val1 = 0x08;
            if( onoff == SHDISP_IRQ_NO_MASK ){
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR1, val1 );
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF1, val1 );
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, val1 );
            }
        }
        break;

    case SHDISP_IRQ_TYPE_DET:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
            val1 = 0x04;
            if( onoff == SHDISP_IRQ_NO_MASK){
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF3, val1 );
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, val1 );
            }
        }
        break;
    default:
        return;
    }

    if( onoff == SHDISP_IRQ_MASK ){
        val1 = 0;
        val2 = 0;
        val3 = 0;
        val4 = 0;
        val5 = 0;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &val1);
        val1 &= 0x01;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &val2);
        val2 &= 0x01;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &val3);
        val3 &= 0x20;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR1, &val4);
        val4 &= 0x08;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF1, &val5);
        val5 &= 0x08;
        if((val1 == 0) && (val2 == 0) && (val3 == 0) && (val4 == 0) && (val5 == 0)){
            if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
            }
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP]I2C_ERR Interrupt Mask\n");
#endif /* SHDISP_SW_BDIC_IRQ_LOG */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_type( int irq_type )
{
    if(( irq_type < SHDISP_IRQ_TYPE_ALS ) || ( irq_type >= NUM_SHDISP_IRQ_TYPE ))
        return SHDISP_RESULT_FAILURE;

    if( irq_type == SHDISP_IRQ_TYPE_DET ){
        if( !( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )){
           return SHDISP_RESULT_FAILURE;
        }
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_save_fac                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_save_fac(void)
{
    unsigned char value1=0, value2=0, value3=0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC1, &value1);
    value1 &= 0x1F;
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC2, &value2);
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC3, &value3);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP] shdisp_bdic_AIP_save_irq_fac:GFAC3=%02x GFAC2=%02x GFAC1=%02x\n", value3, value2, value1);
#endif /* SHDISP_SW_BDIC_IRQ_LOG */
    shdisp_bdic_irq_fac = (unsigned int)value1 | ((unsigned int)value2 << 8 ) | ((unsigned int)value3 << 16);

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ){
        SHDISP_ERR("[SHDISP] ps_als error : INT_PS_REQ(GFAC2[1]) detect\n" );
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PS_REQ;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        SHDISP_ERR("[SHDISP] ps_als error : INT_I2C_ERR_REQ(GFAC3[3]) detect\n" );
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_I2C_ERROR;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
    }

    if( shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_DET | SHDISP_BDIC_INT_GFAC_PS2 | SHDISP_BDIC_INT_GFAC_I2C_ERR) ){
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &shdisp_backup_irq_photo_req[0]);
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &shdisp_backup_irq_photo_req[1]);
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &shdisp_backup_irq_photo_req[2]);
    }

    if((shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
    }

    if(shdisp_bdic_irq_det_flag == 1) {
        shdisp_bdic_irq_fac |= SHDISP_BDIC_INT_GFAC_DET;
        shdisp_bdic_irq_det_flag = 2;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_DET(void)
{
    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        return SHDISP_BDIC_IRQ_TYPE_DET;
    }
    else{
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_fac(void)
{
    int i;
    if( shdisp_bdic_irq_fac == 0 ){
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_irq_fac_exe = (shdisp_bdic_irq_fac & SHDISP_INT_ENABLE_GFAC);
    if( shdisp_bdic_irq_fac_exe == 0 ){
        return SHDISP_RESULT_FAILURE;
    }

    for(i=0; i<SHDISP_IRQ_MAX_KIND ; i++){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_NONE;
    }

    i=0;
    if(( shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_PS | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_PS2 )) == SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_PS;
        i++;
    }

    if(( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_DET ) != 0 ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_DET;
        i++;
    }
    else if(( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_I2C_ERR ) != 0
         || ( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_PS2 ) != 0){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
        i++;
    }
    else if((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0 ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS;
        i++;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_get_fac( int iQueFac )
{
    if( iQueFac >= SHDISP_IRQ_MAX_KIND )
        return SHDISP_BDIC_IRQ_TYPE_NONE;

    return shdisp_bdic_irq_prioriy[iQueFac];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_Clear                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_Clear(void)
{
    unsigned char out1,out2,out3;


    if( shdisp_bdic_irq_fac == 0 )
        return;

    out1 = (unsigned char)(shdisp_bdic_irq_fac & 0x000000FF);
    out2 = (unsigned char)((shdisp_bdic_irq_fac >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((shdisp_bdic_irq_fac >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);


    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS )){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }



    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR)){
    }

    if((( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )) ||
       (( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 )) ||
       (( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR))){
        if( shdisp_backup_irq_photo_req[0] & 0x01 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );
        }
        if( shdisp_backup_irq_photo_req[1] & 0x01 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x01 );
        }
        if( shdisp_backup_irq_photo_req[2] & 0x20 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x20 );
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_i2c_error_Clear(void)
{
    unsigned char out2=0,out3=0;

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        out3 = 0x08;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 ){
        out2 = 0x02;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    }
    psals_recovery_flag = 1;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_fac_Clear                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_fac_Clear(void)
{
    unsigned char out3=0;

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
        out3 = 0x04;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_backup_irq_reg                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_backup_irq_reg(void)
{
    memset(&shdisp_backup_irq_reg, 0, sizeof(shdisp_backup_irq_reg));
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR1, &shdisp_backup_irq_reg[0]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &shdisp_backup_irq_reg[1]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &shdisp_backup_irq_reg[2]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF1, &shdisp_backup_irq_reg[3]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &shdisp_backup_irq_reg[4]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF3, &shdisp_backup_irq_reg[5]);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] backup_irq_reg: R1=0x%02X,R2=0x%02X,R3=0x%02X,F1=0x%02X,F2=0x%02X,F3=0x%02X\n",
        shdisp_backup_irq_reg[0], shdisp_backup_irq_reg[1], shdisp_backup_irq_reg[2],
        shdisp_backup_irq_reg[3], shdisp_backup_irq_reg[4], shdisp_backup_irq_reg[5]);
#endif
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_return_irq_reg                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_return_irq_reg(void)
{
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR1, shdisp_backup_irq_reg[0]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR2, shdisp_backup_irq_reg[1]);
    if (psals_recovery_flag != 2) {
        shdisp_bdic_IO_write_reg( BDIC_REG_GIMR3, shdisp_backup_irq_reg[2]);
    }
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF1, shdisp_backup_irq_reg[3]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF2, shdisp_backup_irq_reg[4]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF3, shdisp_backup_irq_reg[5]);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_irq_reg_i2c_mask                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_irq_reg_i2c_mask(void)
{
    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
    }

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_ctrl                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl)
{
    if( ctrl ){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }
    else {
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_mask                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_irq_mask(void)
{
    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_clr_det_flag                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_clr_det_flag(void)
{
    if(shdisp_bdic_irq_det_flag != 1) {
        shdisp_bdic_irq_det_flag = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_det_flag                                          */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_get_det_flag(void)
{
    return shdisp_bdic_irq_det_flag;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_Clear_All                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_Clear_All(void)
{
    unsigned char out1,out2,out3;

    out1 = (unsigned char)(SHDISP_INT_ENABLE_GFAC & 0x000000FF);
    out2 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_set_fac                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC)
{
    shdisp_bdic_irq_fac = nGFAC;

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_OPTSEL ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_photo_param                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2)
{
    s_state_str.shdisp_lux_change_level1 = (unsigned char)( level1 & 0x00FF );
    s_state_str.shdisp_lux_change_level2 = (unsigned char)( level2 & 0x00FF );
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_lux_timer                                           */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_PHOTO_lux_timer( int onoff )
{
    int ret = SHDISP_DEV_STATE_ON;
    unsigned char   remask;

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        return SHDISP_RESULT_FAILURE;
    }

    if( onoff == SHDISP_BDIC_PHOTO_LUX_TIMER_OFF ){
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x00);
        shdisp_bdic_PD_ps_req_mask(&remask);
    }
    else{
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_SENSOR_start                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_SENSOR_start( int new_opt_mode )
{
    int ret;

    ret = shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SENSOR, 0x01);
    s_state_str.bdic_main_bkl_opt_mode_ado = new_opt_mode;
    shdisp_bdic_PD_ps_req_restart(1);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x04);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_als_sensor_pow_ctl                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_api_als_sensor_pow_ctl(int dev_type, int power_mode)
{
    unsigned long type = 0;
    int param_chk = 0;

    switch (dev_type) {
    case SHDISP_PHOTO_SENSOR_TYPE_APP:
        type = SHDISP_DEV_TYPE_ALS_APP;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_CAMERA:
    case SHDISP_PHOTO_SENSOR_TYPE_KEYLED:
    case SHDISP_PHOTO_SENSOR_TYPE_DIAG:
        return SHDISP_RESULT_FAILURE;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->type(%d).\n", dev_type);
        break;
    }

    switch (power_mode) {
    case SHDISP_PHOTO_SENSOR_DISABLE:
        shdisp_bdic_seq_psals_standby(type);
        break;
    case SHDISP_PHOTO_SENSOR_ENABLE:
        shdisp_bdic_seq_psals_active(type);
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->power(%d).\n", power_mode);
        break;
    }

    if (param_chk == 1) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_default_sensor_param                                  */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_api_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj)
{
    unsigned long tmp_param1;
    unsigned long tmp_param2;

    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ0_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ0_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ1_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ1_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[0].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_A;
    tmp_adj->als_adjust[0].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_A;
    tmp_adj->als_adjust[0].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_A;

    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ0_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ0_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ1_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ1_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[1].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_B;
    tmp_adj->als_adjust[1].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_B;
    tmp_adj->als_adjust[1].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_B;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_prox_sensor_param                                     */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_api_set_prox_sensor_param( struct shdisp_prox_params *prox_params)
{
    memcpy( &s_state_str.prox_params, prox_params, sizeof( struct shdisp_prox_params));
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_get_LC_MLED01                                             */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_bdic_api_get_LC_MLED01(void)
{
    return ((BDIC_REG_M1LED_VAL<<8) | BDIC_REG_M2LED_VAL);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_lux_data                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_get_lux_data(void)
{
    unsigned char para[1] = { 0x05 };
    unsigned char rtnbuf[4] = {0};

    SHDISP_DEBUG("in\n");

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_safe_finishanddoKick();
    }

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXPARAM_READ, 1, para);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 4, rtnbuf);

    SHDISP_DEBUG("out\n");
    return (rtnbuf[3] << 24) | (rtnbuf[2] << 16) | (rtnbuf[1] << 8) | rtnbuf[0];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_bkl_mode                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk)
{
    unsigned char value[1];

    SHDISP_DEBUG("in\n");

    value[0] = bkl_mode;
    value[0] = value[0] & ~msk;
    value[0] |= data;

    SHDISP_DEBUG("[SHDISP]%s bkl_mode=0x%02x\n", __func__, value[0]);

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, 1, value);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, 1, value);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }

    bkl_mode_recovery = value[0];

    SHDISP_DEBUG("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_lux_mode                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk)
{
    unsigned char value[1];

    SHDISP_DEBUG("in\n");

    value[0] = lux_mode;
    value[0] = value[0] & ~msk;
    value[0] |= data;

    SHDISP_DEBUG("lux_mode=0x%02x\n", value[0]);

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, value);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, value);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }

    lux_mode_recovery = value[0];

    SHDISP_DEBUG("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_lux_mode_modify                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk)
{
    unsigned char lux_mode[1];
    int lux_data;

    SHDISP_DEBUG("in\n");

    lux_data = shdisp_bdic_API_get_lux_data();
    lux_mode[0] = (( lux_data  & 0x0000ff00 ) >> 8);
    SHDISP_DEBUG("before lux_mode=0x%02x\n", lux_mode[0]);

    lux_mode[0] = lux_mode[0] & ~msk;
    lux_mode[0] |= data;

    SHDISP_DEBUG("after lux_mode=0x%02x\n", lux_mode[0]);

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, lux_mode);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 0, 0);

    lux_mode_recovery = lux_mode[0];

    SHDISP_DEBUG("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_lux_data_backup                                  */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_RECOVERY_lux_data_backup(void)
{
    int lux_data;
    unsigned char bkl_mode;
    unsigned char lux_mode;

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_bdic_API_set_bkl_mode(0x00 , 0x00 , 0xff);
    shdisp_bdic_API_set_lux_mode(0x00 , 0x00 , 0xff);

    bkl_mode_recovery = bkl_mode;
    lux_mode_recovery = lux_mode;

    SHDISP_DEBUG("backup bkl_mode_recovery = 0x%02x\n",bkl_mode_recovery);
    SHDISP_DEBUG("backup lux_mode_recovery = 0x%02x\n",lux_mode_recovery);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_lux_data_restore                                 */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_RECOVERY_lux_data_restore(void)
{
    SHDISP_DEBUG("restore bkl_mode_recovery = 0x%02x\n",bkl_mode_recovery);
    SHDISP_DEBUG("restore lux_mode_recovery = 0x%02x\n",lux_mode_recovery);
    shdisp_bdic_API_set_bkl_mode(0x00, bkl_mode_recovery , 0xff);
    shdisp_bdic_API_set_lux_mode(0x00, lux_mode_recovery , 0xff);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_active                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_psals_active(unsigned long dev_type)
{
    SHDISP_DEBUG("in\n");
    shdisp_bdic_seq_psals_active(dev_type);
    SHDISP_DEBUG("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_standby                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_psals_standby(unsigned long dev_type)
{
    SHDISP_DEBUG("in\n");
    shdisp_bdic_seq_psals_standby(dev_type);
    SHDISP_DEBUG("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_ps_background                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_ps_background(unsigned long state)
{
    SHDISP_DEBUG("in\n");
    shdisp_bdic_seq_ps_background(state);
    SHDISP_DEBUG("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD4;
        break;

    default:
        return;;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_off                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_off(void)
{
    int ret;

    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    if (shdisp_pm_is_als_active() == SHDISP_DEV_STATE_ON) {
        shdisp_bdic_API_set_lux_mode_modify(0x00 , 0x01);
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        SHDISP_DEBUG("clmr polling stop and bdic polling start.\n");
    }
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_fix_on                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_fix_on(int param)
{
    int ret;

    SHDISP_DEBUG("param:%d\n", param);
    (void)shdisp_pm_clmr_power_manager((SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager((SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 START\n");
    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 END\n");

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    if (shdisp_pm_is_als_active() == SHDISP_DEV_STATE_ON) {
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_bdic_API_set_lux_mode_modify(0x01 , 0x01);
        SHDISP_DEBUG("bdic polling stop and clmr polling start.\n");
    }
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_POST_START_FIX, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    SHDISP_DEBUG("Completed.\n");
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_auto_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_auto_on(int param)
{
    int ret;

    (void)shdisp_pm_clmr_power_manager((SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager((SHDISP_DEV_TYPE_BKL|SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 START\n");
    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_PRE_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 END\n");

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    if (shdisp_pm_is_als_active() == SHDISP_DEV_STATE_ON) {
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_bdic_API_set_lux_mode_modify(0x01 , 0x01);
        SHDISP_DEBUG("bdic polling stop and clmr polling start.\n");
    }
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_POST_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_off                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_off(void)
{
    int ret;

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_NMR|SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_STOP, 0);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED_NMR|SHDISP_DEV_TYPE_LED_ANI);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_normal_on                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_normal_on(unsigned char color)
{
    int ret;

    SHDISP_DEBUG("color:%d\n", color);
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_NMR);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_NMR);
    SHDISP_DEBUG("End.(%d)\n",ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_blink_on                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_firefly_on                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

#if defined(CONFIG_MACH_LYNX_DL45)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_high_speed_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_high_speed_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_standard_on                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_standard_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_breath_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_long_breath_on                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_long_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_wave_on                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_wave_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_flash_on                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_flash_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_aurora_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_aurora_on(int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA, 0x07);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_rainbow_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_rainbow_on(int interval, int count)
{
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED_ANI);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW, 0x07);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED_ANI);
}
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lux                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int mode = s_state_str.bdic_main_bkl_opt_mode_ado;
    unsigned long ret_lux;
    int ext_flag, ok_lux;
    int i,j;
    int ret;
    int clmr_power;

    SHDISP_DEBUG("in lux_table_size=%d\n", SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE);

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }

    clmr_power = shdisp_pm_is_clmr_on();
    if (clmr_power == SHDISP_DEV_STATE_OFF ) {
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_ALS_LUX, SHDISP_DEV_STATE_ON);
        shdisp_bdic_PD_REG_ADO_FW_get_opt(value);
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_ALS_LUX, SHDISP_DEV_STATE_OFF);
    }
    else {
        shdisp_bdic_PD_REG_ADO_get_opt(value);
    }

    ret_lux = shdisp_bdic_bkl_lux_tbl[1][SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE-1].lux;
    for(j=0; j<2 ; j++){
        ok_lux = 0;
        for(i=0; i<SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE; i++) {
            if (*value <= shdisp_bdic_bkl_lux_tbl[mode][i].ado_range) {
                ext_flag = shdisp_bdic_bkl_lux_tbl[mode][i].ext_flag;
                if( ext_flag == mode ){
                    ret_lux = shdisp_bdic_bkl_lux_tbl[mode][i].lux;
                    ok_lux = 1;
                }
                else{
                    mode = ext_flag;
                }
                break;
            }
        }
        if ((ok_lux) || (i >= SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE)){
            break;
        }
    }

    *lux = ret_lux;

#ifdef SHDISP_SW_BDIC_IRQ_LOG
    if (s_state_str.bdic_main_bkl_opt_mode_ado == mode){
        printk("[SHDISP] shdisp_bdic_LD_PHOTO_SENSOR_get_lux : mode=%d / ado=0x%04X, lux=%lu\n", mode, *value, ret_lux);
    }
    else{
        printk("[SHDISP] shdisp_bdic_LD_PHOTO_SENSOR_get_lux : ext_mode=%d / ado=0x%04X, lux=%lu\n", mode, *value, ret_lux);
    }
#endif /* SHDISP_SW_BDIC_IRQ_LOG */

    SHDISP_DEBUG("out mode=%d, ado=0x%04X, lux=%lu\n", mode, *value, ret_lux);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_write                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i;

#ifdef SHDISP_SW_BDIC_I2C_RWLOG
    int t;
    if (msg->wlen <= 0) {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d)\n", msg->addr, msg->wlen);
    }
    else {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d Wbuf=", msg->addr, msg->wlen);
        for (t=0; t<(msg->wlen-1); t++) {
            printk("0x%02X,", (msg->wbuf[t] & 0xFF));
        }
        printk("0x%02X)\n", (msg->wbuf[t] & 0xFF));
    }
#endif

    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->wlen > SHDISP_BDIC_I2C_WBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));

    for (i=0; i<msg->wlen; i++) {
        shdisp_bdic_IO_write_reg((BDIC_REG_I2C_DATA1 + i), msg->wbuf[i]);
    }

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, msg->wlen);

    shdisp_bdic_i2c_errflag = 0;

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_START, SHDISP_BDIC_I2C_W_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);

    for (i=0; i<=SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else {
            shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);
            if ((value & SHDISP_BDIC_I2C_W_START) == 0x00) {
                return SHDISP_RESULT_SUCCESS;
            }
            else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }

    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode0                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i, t;

    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->wlen != 0x01) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->rbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->rlen > SHDISP_BDIC_I2C_RBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->rlen(%d).\n", msg->rlen);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SYS, 0x00);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, msg->wbuf[0]);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, (unsigned char)(msg->rlen << 4));

    shdisp_bdic_i2c_errflag = 0;

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_START, SHDISP_BDIC_I2C_R_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);

    for (i=0; i<=SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else {
            shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);
            if ((value & SHDISP_BDIC_I2C_R_START) == 0x00) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP] bdic i2c Read(addr=0x%02x, reg=0x%02X, size=%d, ", msg->addr, msg->wbuf[0], msg->rlen);
#endif
                for (t=0; t<msg->rlen; t++) {
                    shdisp_bdic_IO_read_reg((BDIC_REG_I2C_READDATA0 + t), &value);
                    msg->rbuf[t] = value;
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                    if (t < (msg->rlen - 1)) {
                        printk("0x%02X,", value);
                    }
                    else {
                        printk("0x%02X)\n", value);
                    }
#endif
                }
                return SHDISP_RESULT_SUCCESS;
            }
            else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }

    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode1                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode2                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode3                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_active                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_psals_active(unsigned long dev_type)
{
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    if (dev_type == SHDISP_DEV_TYPE_ALS_APP) {
        (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_standby                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_psals_standby(unsigned long dev_type)
{
    if (dev_type == SHDISP_DEV_TYPE_ALS_APP) {
        (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    }
    (void)shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_ps_background                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_ps_background(unsigned long state)
{
    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_PS, state);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_on(void)
{
    int ret;

    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_off(void)
{
    int ret;

    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_on(void)
{
    int ret;

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_off(void)
{
    int ret;

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY;
    }
    else if ((shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_FIX) && (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON)) {
        *mode = SHDISP_BKL_TBL_MODE_CHARGE;
    }
    else if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        *mode = SHDISP_BKL_TBL_MODE_ECO;
    }
    else {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    if (*mode >= NUM_SHDISP_BKL_TBL_MODE) {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_on(void)
{
    int ret;

    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_off(void)
{
    int ret;

    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_on(void)
{
    int ret;

    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_off(void)
{
    int ret;

    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_on(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_on);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_off(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_off);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_on(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_off(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsn_off);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_VO2_ON                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_VO2_ON(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vo2_on);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_VO2_OFF                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_VO2_OFF(void)
{
    SHDISP_DEBUG("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vo2_off);
    SHDISP_DEBUG("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_active_for_led                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_active_for_led(int dev_type)
{
    SHDISP_DEBUG("dev_type:%d\n", dev_type);
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    SHDISP_DEBUG("Completed\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_standby_for_led                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_standby_for_led(int dev_type)
{
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_clmr_off_for_led                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_clmr_off_for_led(int dev_type)
{
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_bdic_seq_regset                                               */
/*---------------------------------------------------------------------------*/
static int shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size, unsigned char bkl_mode, unsigned char lux_mode)
{
    int i, cnt_bdic, cnt_als;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t* tbl;
    unsigned char top_addr_bdic;
    unsigned char top_addr_als;
    unsigned char bBuf_bdic[16];
    unsigned char bBuf_als[8];

    cnt_bdic = 0;
    cnt_als  = 0;
    top_addr_bdic = 0x00;
    top_addr_als  = 0x00;

    tbl = (shdisp_bdicRegSetting_t*)regtable;
    for (i = 0; i < size; i++) {
        if (((cnt_bdic > 0) && (tbl->flg != SHDISP_BDIC_STRM)) || (cnt_bdic == sizeof(bBuf_bdic))) {
            ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            cnt_bdic = 0;
            top_addr_bdic = 0x00;
        }
        if (((cnt_als > 0) && (tbl->flg != SHDISP_ALS_STRM)) || (cnt_als == sizeof(bBuf_als))) {
            ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
            cnt_als = 0;
            top_addr_als = 0x00;
        }
        switch(tbl->flg) {
        case SHDISP_BDIC_STR:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
            ret = shdisp_bdic_IO_set_bit_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CLR:
            ret = shdisp_bdic_IO_clr_bit_reg(tbl->addr, tbl->mask);
            break;
        case SHDISP_BDIC_RMW:
            ret = shdisp_bdic_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BDIC_STRM:
            if (cnt_bdic == 0) {
                top_addr_bdic = tbl->addr;
            }
            bBuf_bdic[cnt_bdic] = tbl->data;
            cnt_bdic++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            }
            break;
        case SHDISP_BDIC_WAIT:
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
            ret = shdisp_bdic_als_clmr_write_cmd(tbl->addr, tbl->data);
            break;
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            if (cnt_als == 0) {
                top_addr_als = tbl->addr;
            }
            bBuf_als[cnt_als] = tbl->data;
            cnt_als++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
            }
            break;
        case SHDISP_ALS_RMW:
            ret = shdisp_bdic_als_clmr_msk_write_cmd(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BKL_MODE:
            shdisp_bdic_API_set_bkl_mode(bkl_mode, tbl->data , tbl->mask);
            break;
        case SHDISP_LUX_MODE:
            shdisp_bdic_API_set_lux_mode(lux_mode, tbl->data , tbl->mask);
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic R/W Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            return ret;
        }
        if (tbl->wait > 0) {
            if ((cnt_bdic > 0) && (tbl->flg == SHDISP_BDIC_STRM)) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            if ((cnt_als > 0) && (tbl->flg == SHDISP_ALS_STRM)) {
                ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
                cnt_als = 0;
                top_addr_als = 0x00;
            }
            shdisp_SYS_cmd_delay_us(tbl->wait);
        }
        tbl++;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_active                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_set_active(int power_status)
{
    int ret;

    SHDISP_TRACE("[S] \n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    if (power_status == SHDISP_DEV_STATE_NOINIT) {
        SHDISP_BDIC_REGSET(shdisp_bdic_init);
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_active);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_standby                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_set_standby(void)
{
    int ret;

    SHDISP_TRACE("[S] \n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    SHDISP_BDIC_REGSET(shdisp_bdic_standby);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{
    unsigned char value[1];

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF){
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
        }
        else if(shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX){

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
        }
        else if(shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO){
            if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX ) {
                shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

                value[0] = 0x01;
                shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);

                value[0] = 0x01;
                shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
            }
            else if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO ) {
                shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);
            }
        }
        break;

    case SHDISP_BDIC_REQ_PRE_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF){
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

            value[0] = 0x03;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
        }
        break;

    case SHDISP_BDIC_REQ_POST_START_FIX:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF){
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_post_start);
        }
        break;

    case SHDISP_BDIC_REQ_POST_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF){
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_post_start);
        }
        else if(shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX){
            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

            value[0] = 0x03;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);
        }
        else if(shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO){
            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        value[0] = 0x00;
        shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value, 1);

        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = 0;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_off);
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_AUTO;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_ON;
        break;

    default:
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char    reg;
    unsigned char    bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPOD1;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPOD1;
        bit = 0x02;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPOD1;
        bit = 0x04;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPOD1;
        bit = 0x08;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPOD1;
        bit = 0x10;
        break;
    case SHDISP_BDIC_GPIO_GPOD5:
        reg = BDIC_REG_GPOD1;
        bit = 0x20;
        break;
    default:
        return;
    }
    if ( status == SHDISP_BDIC_GPIO_HIGH ) {
        shdisp_bdic_IO_set_bit_reg( reg, bit );
    }
    else
    {
        shdisp_bdic_IO_clr_bit_reg( reg, bit );
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    unsigned char *pTriLed;

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            pTriLed = (unsigned char*)(&(shdisp_triple_led_tbl[s_state_str.bdic_clrvari_index][shdisp_bdic_tri_led_color]));
            shdisp_bdic_led_fix_on[0].data = *(pTriLed+0);
            shdisp_bdic_led_fix_on[1].data = *(pTriLed+1);
            shdisp_bdic_led_fix_on[2].data = *(pTriLed+2);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
        }
        else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_TRI_LED_set_chdig();
            shdisp_bdic_PD_TRI_LED_set_anime();
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color    = 0;
        shdisp_bdic_tri_led_ontime   = 0;
        shdisp_bdic_tri_led_interval = 0;
        shdisp_bdic_tri_led_count    = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

#if defined(CONFIG_MACH_LYNX_DL45)
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_HISPEED;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_STANDARD;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_WAVE;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FLASH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_AURORA;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_RAINBOW;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char timeer1_val;
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;

    timeer1_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
    timeer1_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER1, (unsigned char)timeer1_val, 0xF7);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        ch_set1_val = 0x06;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER1, (unsigned char)timeer1_val, 0xF7);
        break;

#if defined(CONFIG_MACH_LYNX_DL45)
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_high_speed_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_standard_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_breath_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_long_breath_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_wave_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_flash_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_aurora_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        shdisp_bdic_IO_write_reg(BDIC_REG_TIMEER1, timeer1_val);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_rainbow_on);
        break;
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    int clrvari = s_state_str.bdic_clrvari_index;
    unsigned char wBuf[9];
#if defined(CONFIG_MACH_LYNX_DL45)
    unsigned char anime_tbl1[SHDISP_COL_VARI_KIND][3][SHDISP_TRI_LED_COLOR_TBL_NUM][3];
    unsigned char anime_tbl2[SHDISP_COL_VARI_KIND][3][3];
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

    memset( wBuf, 0, sizeof( wBuf ));

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        wBuf[0] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[1] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[2] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        wBuf[3] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][0];
        wBuf[4] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][1];
        wBuf[5] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][2];
        wBuf[6] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[7] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[8] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;

#if defined(CONFIG_MACH_LYNX_DL45)
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        wBuf[3] = shdisp_triple_led_anime_high_speed_tbl[clrvari][shdisp_bdic_tri_led_color][0];
        wBuf[4] = shdisp_triple_led_anime_high_speed_tbl[clrvari][shdisp_bdic_tri_led_color][1];
        wBuf[5] = shdisp_triple_led_anime_high_speed_tbl[clrvari][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_BREATH){
            memcpy(anime_tbl1, shdisp_triple_led_anime_breath_tbl, sizeof(anime_tbl1));
        }
        else if(shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH){
            memcpy(anime_tbl1, shdisp_triple_led_anime_long_breath_tbl, sizeof(anime_tbl1));
        }
        else if(shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_WAVE){
            memcpy(anime_tbl1, shdisp_triple_led_anime_wave_tbl, sizeof(anime_tbl1));
        }
        else{
            memcpy(anime_tbl1, shdisp_triple_led_anime_flash_tbl, sizeof(anime_tbl1));
        }

        wBuf[0] = anime_tbl1[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[1] = anime_tbl1[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[2] = anime_tbl1[clrvari][0][shdisp_bdic_tri_led_color][2];
        wBuf[3] = anime_tbl1[clrvari][1][shdisp_bdic_tri_led_color][0];
        wBuf[4] = anime_tbl1[clrvari][1][shdisp_bdic_tri_led_color][1];
        wBuf[5] = anime_tbl1[clrvari][1][shdisp_bdic_tri_led_color][2];
        wBuf[6] = anime_tbl1[clrvari][2][shdisp_bdic_tri_led_color][0];
        wBuf[7] = anime_tbl1[clrvari][2][shdisp_bdic_tri_led_color][1];
        wBuf[8] = anime_tbl1[clrvari][2][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_AURORA){
            memcpy(anime_tbl2, shdisp_triple_led_anime_aurora_tbl, sizeof(anime_tbl2));
        }
        else{
            memcpy(anime_tbl2, shdisp_triple_led_anime_rainbow_tbl, sizeof(anime_tbl2));
        }
        wBuf[0] = anime_tbl2[clrvari][0][0];
        wBuf[1] = anime_tbl2[clrvari][0][1];
        wBuf[2] = anime_tbl2[clrvari][0][2];
        wBuf[3] = anime_tbl2[clrvari][1][0];
        wBuf[4] = anime_tbl2[clrvari][1][1];
        wBuf[5] = anime_tbl2[clrvari][1][2];
        wBuf[6] = anime_tbl2[clrvari][2][0];
        wBuf[7] = anime_tbl2[clrvari][2][1];
        wBuf[8] = anime_tbl2[clrvari][2][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PWM_set_value                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PWM_set_value(int pwm_param)
{
    unsigned char value[3] = { 0, 0, 0 };
    unsigned short temp_val;
    int mode = 0;
    unsigned char *pwm_value;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_OFF:
        return;

    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
            temp_val = shdisp_main_dtv_bkl_tbl[shdisp_bdic_bkl_param][mode];
        }
        else {
            temp_val = shdisp_main_bkl_tbl[shdisp_bdic_bkl_param][mode];
        }
        value[0] = 0x5D;
        value[1] = ( temp_val & 0x00ff );
        value[2] = ( temp_val >> 8 );

        shdisp_bdic_bkl_adj_clmr_write_cmd( SHDISP_BDIC_BL_PARAM_WRITE, value, sizeof(value) );

        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:

        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

        if( pwm_param == SHDISP_BDIC_BL_PWM_FIX_PARAM){
            if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
                temp_val = shdisp_main_dtv_bkl_tbl[SHDISP_MAIN_BKL_PARAM_15][mode];
            }
            else {
                temp_val = shdisp_main_bkl_tbl[SHDISP_MAIN_BKL_PARAM_15][mode];
            }
            value[0] = 0x5D;
            value[1] = ( temp_val & 0x00ff );
            value[2] = ( temp_val >> 8 );

            shdisp_bdic_bkl_adj_clmr_write_cmd( SHDISP_BDIC_BL_PARAM_WRITE, value, sizeof(value) );

        }
        else{

            if( s_state_str.bdic_main_bkl_opt_mode_output == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){

                pwm_value = (unsigned char *)&shdisp_main_bkl_pwm_low_tbl[mode][0];
                shdisp_bdic_bkl_adj_clmr_write_cmd( SHDISP_BDIC_BL_PARAM_WRITE,
                                                    pwm_value+1,
                                                    129 );

            }
            else{
                pwm_value = (unsigned char *)&shdisp_main_bkl_pwm_high_tbl[mode][0];
                shdisp_bdic_bkl_adj_clmr_write_cmd( SHDISP_BDIC_BL_PARAM_WRITE,
                                                    pwm_value+1,
                                                    129 );
            }
        }
        break;

    default:
        return;

    }
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_on                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_power_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;
    int sensor_state;
    int i;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    SHDISP_DEBUG("[S]\n");

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = ((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_on);
    SHDISP_PSALS_REGSET(shdisp_bdic_psals_init);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    sensor_state = shdisp_bdic_API_get_sensor_state();
    if ( sensor_state == 0x00 ) {
        SHDISP_ERR("psals poweron(first) failed!! sensor_state = %02x\n",sensor_state);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PSALS_ON_NG;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */

        for(i=0; i<3 ; i++){
            shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
            SHDISP_PSALS_REGSET(shdisp_bdic_psals_init);
            ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

            sensor_state = shdisp_bdic_API_get_sensor_state();
            if (sensor_state != 0x00) {
                break;
            }
            SHDISP_ERR("try psals poweron failed(%d)!! sensor_state = %02x\n", i+1,sensor_state);
        }

        if( i == 3 ){
            SHDISP_ERR("psals poweron retry over!!\n");
            if (psals_recovery_flag == 1) {
                psals_recovery_flag = 2;
            }
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PSALS;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_POWER_ON_NG;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        }
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_SYS_cmd_delay_us(20000);
    SHDISP_PSALS_REGSET(shdisp_bdic_psals_standby);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_off                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_power_off(void)
{
    int ret;

    SHDISP_DEBUG("[S]\n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_off);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    psals_recovery_flag = 0;
    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_ps_init(int *state)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_DEBUG("[S] state = %d\n", *state);

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);

    switch (*state) {
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
        shdisp_bdic_PD_psals_write_threshold(&shdisp_bdic_ps_init_als_off[4]);
        SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_off);
        *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF;
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
        shdisp_bdic_PD_psals_write_threshold(&shdisp_bdic_ps_init_als_on[6]);
        SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_on);
        if (shdisp_pm_is_bkl_active() != SHDISP_DEV_STATE_ON) {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_bdic_wait);
        }
        else {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_clmr_wait);
        }
        *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_ON;
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        break;
    default:
        break;
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    if( *state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF) {
        shdisp_bdic_API_psals_error_ctrl(1,0);
    }
    else if( *state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON) {
        shdisp_bdic_API_psals_error_ctrl(1,1);
    }

    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_ps_deinit(int *state)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_DEBUG("[S] state = %d\n", *state);

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);

    switch (*state) {
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        SHDISP_PSALS_REGSET(shdisp_bdic_ps_deinit_als_off);
        *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF;
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        SHDISP_PSALS_REGSET(shdisp_bdic_ps_deinit_als_on);
        if (shdisp_pm_is_bkl_active() != SHDISP_DEV_STATE_ON) {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_bdic);
        }
        else {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_clmr);
        }
        *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON;
        break;
    default:
        break;
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    if( *state == SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF) {
        shdisp_bdic_API_psals_error_ctrl(0,0);
    }
    else if( *state == SHDISP_SENSOR_STATE_PROX_ON_ALC_ON) {
        shdisp_bdic_API_psals_error_ctrl(0,1);
    }

    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init                                             */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_als_init(int *state)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_DEBUG("[S] state = %d\n", *state);

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);

    switch (*state) {
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
        SHDISP_PSALS_REGSET(shdisp_bdic_als_init_ps_off);
        if (shdisp_pm_is_bkl_active() != SHDISP_DEV_STATE_ON) {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_bdic);
        }
        else {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_clmr);
        }
        *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON;
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        SHDISP_PSALS_REGSET(shdisp_bdic_als_init_ps_on);
        if (shdisp_pm_is_bkl_active() != SHDISP_DEV_STATE_ON) {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_bdic_wait);
        }
        else {
            SHDISP_PSALS_REGSET(shdisp_bdic_als_auto_light_clmr_wait);
        }
        *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_ON;
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        break;
    default:
        break;
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    if( *state == SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF) {
        shdisp_bdic_API_psals_error_ctrl(1,1);
    }
    else if( *state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF) {
        shdisp_bdic_API_psals_error_ctrl(0,1);
    }

    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit                                           */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_PD_psals_als_deinit(int *state)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_DEBUG("[S] state = %d\n", *state);

    lux_data = shdisp_bdic_API_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);

    switch (*state) {
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
        SHDISP_PSALS_REGSET(shdisp_bdic_als_deinit_ps_off);
        *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF;
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        SHDISP_PSALS_REGSET(shdisp_bdic_als_deinit_ps_on);
        *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF;
        break;
    default:
        break;
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    if( *state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON) {
        shdisp_bdic_API_psals_error_ctrl(0,0);
    }
    else if( *state == SHDISP_SENSOR_STATE_PROX_ON_ALC_ON) {
        shdisp_bdic_API_psals_error_ctrl(1,0);
    }

    SHDISP_DEBUG("[E] ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_write_threshold                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_write_threshold(shdisp_bdicRegSetting_t *pThresh)
{
    if (!pThresh) {
        return SHDISP_RESULT_FAILURE;
    }
    pThresh->data = (unsigned char)(s_state_str.prox_params.threshold_low & 0x00FF);
    pThresh++;
    pThresh->data = (unsigned char)((s_state_str.prox_params.threshold_low >> 8) & 0x00FF);
    pThresh++;
    pThresh->data = (unsigned char)(s_state_str.prox_params.threshold_high & 0x00FF);
    pThresh++;
    pThresh->data = (unsigned char)((s_state_str.prox_params.threshold_high >> 8) & 0x00FF);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_write_cmd                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_write_cmd(unsigned char reg, unsigned char val)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[2];

    ary[0] = reg;
    ary[1] = val;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_WRITE, 2, ary);
    }
    else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_WRITE, 2, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_msk_write_cmd                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_msk_write_cmd(unsigned char reg, unsigned char val, unsigned char msk)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[3];

    ary[0] = reg;
    ary[1] = msk;
    ary[2] = val;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_MASK_WRITE, 3, ary);
    }
    else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_MASK_WRITE, 3, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_error_ctrl                               */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_psals_error_ctrl(int psctrl, int alsctl)
{
    if (psals_recovery_flag == 2) {
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
        return;
    }
    if( alsctl ){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x08 );
    }
    else {
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08 );
    }

    if( psctrl ){
    }
    else {
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_bkl_adj_clmr_write_cmd                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_bkl_adj_clmr_write_cmd(int mode, unsigned char* value, int len )
{
#ifndef SHDISP_FW_STACK_EXCUTE
    if(shdisp_FWCMD_buf_get_nokick()){
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
    }
#endif
    switch (mode) {
    case SHDISP_BDIC_BL_PARAM_WRITE:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_WRITE, len, value);
        break;
    case SHDISP_BDIC_BL_MODE_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, len, value);
        bkl_mode_recovery = *value;
        break;
    case SHDISP_BDIC_ALS_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, len, value);
        lux_mode_recovery = *value;
        break;
    case SHDISP_BDIC_ALS_PARAM_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_PARAM_REFLECT, len, value);
        break;
    default:
        break;
    }
#ifndef SHDISP_FW_STACK_EXCUTE
    if(shdisp_FWCMD_buf_get_nokick()){
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
    }
#endif
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_read_cmd                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_read_cmd(unsigned char reg, unsigned char *val)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[1];

    ary[0] = reg;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, ary);
    }
    else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 1, val);
    }

    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_burst_write_cmd                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_burst_write_cmd(unsigned char addr, unsigned char *val, unsigned char size)
{
    int result = SHDISP_RESULT_SUCCESS;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add_burst(SHDISP_CLMR_FWCMD_ALS_BURST_WRITE, addr, size, val);
    }
    else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
        shdisp_FWCMD_buf_add_burst(SHDISP_CLMR_FWCMD_ALS_BURST_WRITE, addr, size, val);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_OPT_set_value                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_OPT_set_value( int table )
{
    int idx;
    int mode = 0;
    unsigned char shdisp_bkl_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE];

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    if( table == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG]opt low table write(mode=%d)\n", mode);
#endif
        s_state_str.bdic_main_bkl_opt_mode_output = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
        shdisp_pm_set_als_sensor_param(SHDISP_BDIC_MAIN_BKL_OPT_LOW);
        shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT, s_state_str.shdisp_lux_change_level1);
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON)
            memcpy( shdisp_bkl_tbl, shdisp_main_dtv_bkl_opt_low_tbl, sizeof( shdisp_main_dtv_bkl_opt_low_tbl ));
        else
            memcpy( shdisp_bkl_tbl, shdisp_main_bkl_opt_low_tbl, sizeof( shdisp_main_bkl_opt_low_tbl ));
    }
    else if( table == SHDISP_BDIC_MAIN_BKL_OPT_HIGH ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG]opt high table write(mode=%d)\n", mode);
#endif
        s_state_str.bdic_main_bkl_opt_mode_output = SHDISP_BDIC_MAIN_BKL_OPT_HIGH;
        shdisp_pm_set_als_sensor_param(SHDISP_BDIC_MAIN_BKL_OPT_HIGH);
        shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT, s_state_str.shdisp_lux_change_level2);
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON)
            memcpy( shdisp_bkl_tbl, shdisp_main_dtv_bkl_opt_high_tbl, sizeof( shdisp_main_dtv_bkl_opt_high_tbl ));
        else
            memcpy( shdisp_bkl_tbl, shdisp_main_bkl_opt_high_tbl, sizeof( shdisp_main_bkl_opt_high_tbl ));
    }
    else
        return;


    for( idx = 0; idx < SHDISP_BKL_AUTO_TBL_NUM; idx++ ){
        shdisp_bdic_IO_write_reg( shdisp_bkl_tbl[idx][0],
                                  shdisp_bkl_tbl[idx][1+mode]);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_get_opt                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value)
{
    unsigned char  reg_sensor;
    static unsigned short before_val = 0x0000;
    int InfoReg2;

    shdisp_bdic_IO_read_reg(BDIC_REG_SENSOR, &reg_sensor);
    if( reg_sensor & 0x01 ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG] ADO_get_opt XEN_SENSOR=ON before_ADO=0x%04X\n", before_val);
#endif
    }
    else{
        InfoReg2 = shdisp_SYS_getInfoReg2();
        SHDISP_DEBUG("[SHDISP]%s InfoReg2=0x%08x\n", __func__, InfoReg2);

        before_val = (InfoReg2 & 0x0000ffff);
        SHDISP_DEBUG("[SHDISP]%s ADO=0x%04x\n", __func__, before_val);
    }
    *value = before_val;
    SHDISP_DEBUG("[SHDISP] ADO_get_opt value = 0x%04X\n", *value);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_FW_get_opt                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_ADO_FW_get_opt(unsigned short *value)
{
    unsigned char  reg_sensor;
    static unsigned short before_val = 0x0000;
    int i,lux_data;
    unsigned char para[1] = { 0x06 };
    unsigned char rtnbuf[4] = {0};

    shdisp_bdic_IO_read_reg(BDIC_REG_SENSOR, &reg_sensor);
    if( reg_sensor & 0x01 ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG] ADO_get_opt XEN_SENSOR=ON before_ADO=0x%04X\n", before_val);
#endif
    }
    else{
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);
        shdisp_bdic_API_set_lux_mode_modify( 0x01, 0x01 );

        for (i = 0; i < 10; i++) {
            shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
            shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXPARAM_READ, 1, para);
            shdisp_FWCMD_buf_finish();
            shdisp_FWCMD_doKick(1, 4, rtnbuf);

            lux_data = (rtnbuf[3] << 24) | (rtnbuf[2] << 16) | (rtnbuf[1] << 8) | rtnbuf[0];

            if( (lux_data & 0x00ff0000) == 0x00010000 ){
                break;
            }
            shdisp_SYS_cmd_delay_us(10*1000);
        }
        before_val = (lux_data & 0x0000ffff);
        SHDISP_DEBUG("[SHDISP]%s ADO=0x%04x\n", __func__, before_val);

        shdisp_bdic_API_set_lux_mode_modify( 0x00, 0x01 );
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, BDIC_REG_I2C_START_R_TIMER_START);

    }
    *value = before_val;
    SHDISP_DEBUG("[SHDISP] ADO_get_opt value = 0x%04X\n", *value);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_sensor_state                                          */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_get_sensor_state(void)
{
    unsigned char para[1] = { 0x00 };
    unsigned char rtnbuf[4] = {0};

    SHDISP_DEBUG("in\n");

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, para);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 4, rtnbuf);

    SHDISP_DEBUG("out\n");
    return (rtnbuf[3] << 24) | (rtnbuf[2] << 16) | (rtnbuf[1] << 8) | rtnbuf[0];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_ps_req_mask                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_ps_req_mask(unsigned char *remask)
{
    int ret;
    unsigned char val;

    *remask = 0;
    if ((SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) == 0){
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_bdic_IO_read_reg(BDIC_REG_GIMF2, &val);
    if ((val & BDIC_REG_GIFM2_PS_REQ_IMF) == BDIC_REG_GIFM2_PS_REQ_IMF) {
        *remask = 1;
        val &= (unsigned char)(~BDIC_REG_GIFM2_PS_REQ_IMF);
        ret = shdisp_bdic_IO_write_reg(BDIC_REG_GIMF2, val);
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_ps_req_restart                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_ps_req_restart(unsigned char remask)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_write_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_write(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_write_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_multi_write_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_multi_write(reg, wval, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_read_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((reg == BDIC_REG_TEST_B3)
    ||  (reg == BDIC_REG_I2C_START)) {
        ret = shdisp_bdic_IO_read_no_check_reg(reg, val);
    }
    else {
        ret = shdisp_bdic_IO_read_check_reg(reg, val);
    }

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_no_check_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_read(reg, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_check_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    int retry = 0;
    unsigned char try_1st, try_2nd;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    try_1st = 0;
    try_2nd = 0;

    for (retry = 0; retry < 3; retry++) {
        ret = shdisp_SYS_bdic_i2c_read(reg, &try_1st);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        ret = shdisp_SYS_bdic_i2c_read(reg, &try_2nd);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        if (try_1st == try_2nd) {
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        }
        else if (retry == 2) {
            SHDISP_ERR("<OTHER> i2c read retry over! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n", reg, try_1st, try_2nd);
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_BDIC;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_I2C_READ;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        }
        else {
            SHDISP_ERR("<OTHER> i2c read retry (%d)! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n", retry, reg, try_1st, try_2nd);
        }
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;

    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }

    if ((reg + (unsigned char)(size-1)) > BDIC_REG_TEST6) {
        SHDISP_ERR("<OTHER> register address overflow.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SYS_bdic_i2c_multi_read(reg, val, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_read.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, 0x00, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_change_range                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_change_range( int bkl_opt )
{
    int           ret;
    unsigned char val=0;

    ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, &val);
    val &= 0xF8;

    if( bkl_opt == SHDISP_BDIC_MAIN_BKL_OPT_LOW )
        val |= 0x04;
    else
        val |= 0x06;
    ret = shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND2, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, msk);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_write_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[2];

    wbuf[0] = reg;
    wbuf[1] = val;

    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = 2;
    msg.rlen = 0;
    msg.wbuf = &wbuf[0];
    msg.rbuf = NULL;

    ret = shdisp_bdic_API_i2c_transfer(&msg);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_read_reg                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];
    unsigned char rbuf[1];

    wbuf[0] = reg;
    rbuf[0] = 0x00;

    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = 1;
    msg.wbuf = &wbuf[0];
    msg.rbuf = &rbuf[0];

    ret = shdisp_bdic_API_i2c_transfer(&msg);
    if (ret == SHDISP_RESULT_SUCCESS) {
        *val = rbuf[0];
    }

    return ret;
}

static int shdisp_bdic_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    int rc = 0;

    SHDISP_DEBUG(" pdev = 0x%p\n", pdev );

    if( pdev ){


        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if( !res ){
            SHDISP_ERR("irq resouce err!!\n");
            rc = 0;
            goto probe_done;
        }
        else {
            shdisp_SYS_set_irq_port(res->start, pdev);
        }
    }

probe_done:
    SHDISP_DEBUG(" rc = %d\n", rc );

    return rc;
#else
    return 0;
#endif /* CONFIG_OF */
}

static int shdisp_bdic_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_bdic_dt_match[] = {
    { .compatible = "sharp,shdisp_bdic",},
    {}
};
#else
#define shdisp_bdic_dt_match NULL
#endif /* CONFIG_OF */

static struct platform_driver shdisp_bdic_driver = {
    .probe = shdisp_bdic_probe,
    .remove = shdisp_bdic_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_bdic",
        .of_match_table = shdisp_bdic_dt_match,
    },
};

static int shdisp_bdic_register_driver(void)
{
    return platform_driver_register(&shdisp_bdic_driver);
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
