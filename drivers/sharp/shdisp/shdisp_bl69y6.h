/* drivers/sharp/shdisp/shdisp_bl69y6.h  (Display Driver)
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

#ifndef SHDISP_BL69Y6_H
#define SHDISP_BL69Y6_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define BDIC_REG_ADOL           0x00
#define BDIC_REG_ADOH           0x01
#define BDIC_REG_SYSTEM1        0x02
#define BDIC_REG_SYSTEM2        0x03
#define BDIC_REG_SYSTEM3        0x04
#define BDIC_REG_SYSTEM4        0x05
#define BDIC_REG_OPT_MODE       0x06
#define BDIC_REG_SLOPE          0x08
#define BDIC_REG_MODE_M1        0x09
#define BDIC_REG_M1LED          0x0A
#define BDIC_REG_M2LED          0x0B
#define BDIC_REG_PWM_M          0x0C
#define BDIC_REG_NLED1          0x0E
#define BDIC_REG_NLED2          0x0F
#define BDIC_REG_OPT0           0x10
#define BDIC_REG_OPT1           0x11
#define BDIC_REG_OPT2           0x12
#define BDIC_REG_OPT3           0x13
#define BDIC_REG_OPT4           0x14
#define BDIC_REG_OPT5           0x15
#define BDIC_REG_OPT6           0x16
#define BDIC_REG_OPT7           0x17
#define BDIC_REG_OPT8           0x18
#define BDIC_REG_OPT9           0x19
#define BDIC_REG_OPT10          0x1A
#define BDIC_REG_OPT11          0x1B
#define BDIC_REG_OPT12          0x1C
#define BDIC_REG_OPT13          0x1D
#define BDIC_REG_OPT14          0x1E
#define BDIC_REG_OPT15          0x1F
#define BDIC_REG_EXOPT0         0x20
#define BDIC_REG_EXOPT1         0x21
#define BDIC_REG_EXOPT2         0x22
#define BDIC_REG_EXOPT3         0x23
#define BDIC_REG_DCDC1_VLIM     0x24
#define BDIC_REG_DCDC_SYS       0x25
#define BDIC_REG_DCDC2_VO       0x27
#define BDIC_REG_DCDC2_SYS      0x28
#define BDIC_REG_DCDC2_RESERVE  0x29
#define BDIC_REG_CPM            0x2B
#define BDIC_REG_TIMERSTART     0x2C
#define BDIC_REG_SEQ_ANIME      0x2D
#define BDIC_REG_TIMEER1        0x2E
#define BDIC_REG_CH0_SET1       0x30
#define BDIC_REG_CH0_SET2       0x31
#define BDIC_REG_CH1_SET1       0x32
#define BDIC_REG_CH1_SET2       0x33
#define BDIC_REG_CH2_SET1       0x34
#define BDIC_REG_CH2_SET2       0x35
#define BDIC_REG_CH_TEST        0x36
#define BDIC_REG_CH0_A          0x37
#define BDIC_REG_CH1_A          0x38
#define BDIC_REG_CH2_A          0x39
#define BDIC_REG_CH0_B          0x3A
#define BDIC_REG_CH1_B          0x3B
#define BDIC_REG_CH2_B          0x3C
#define BDIC_REG_CH0_C          0x3D
#define BDIC_REG_CH1_C          0x3E
#define BDIC_REG_CH2_C          0x3F
#define BDIC_REG_TEST_ADC0      0x59
#define BDIC_REG_TEST_ADC1      0x5A
#define BDIC_REG_TEST_ADC2      0x5B
#define BDIC_REG_TEST_ADC4      0x5C
#define BDIC_REG_TEST_ADC5      0x5D
#define BDIC_REG_TEST_ADC6      0x5E
#define BDIC_REG_TEST_ADC7      0x5F
#define BDIC_REG_TEST60         0x60
#define BDIC_REG_TEST61         0x61
#define BDIC_REG_ADO_SYS        0x62
#define BDIC_REG_ALS_DATA0_SET  0x63
#define BDIC_REG_ALS_ADJ0_L     0x64
#define BDIC_REG_ALS_ADJ0_H     0x65
#define BDIC_REG_ALS_DATA1_SET  0x66
#define BDIC_REG_ALS_ADJ1_L     0x67
#define BDIC_REG_ALS_ADJ1_H     0x68
#define BDIC_REG_ALS_SHIFT      0x69
#define BDIC_REG_ALS_INT        0x6A
#define BDIC_REG_ADO_INDEX      0x6B
#define BDIC_REG_PSDATA_SYS     0x6C
#define BDIC_REG_PSDATA_SET     0x6D
#define BDIC_REG_PS_HT_LSB      0x6E
#define BDIC_REG_PS_HT_MSB      0x6F
#define BDIC_REG_PS_LT_LSB      0x70
#define BDIC_REG_PS_LT_MSB      0x71
#define BDIC_REG_I2C_START      0x72
#define BDIC_REG_I2C_SET        0x73
#define BDIC_REG_I2C_TIMER      0x74
#define BDIC_REG_I2C_SYS        0x75
#define BDIC_REG_I2C_DATA0      0x76
#define BDIC_REG_I2C_DATA1      0x77
#define BDIC_REG_I2C_DATA2      0x78
#define BDIC_REG_I2C_DATA3      0x79
#define BDIC_REG_I2C_DATA4      0x7A
#define BDIC_REG_I2C_DATA5      0x7B
#define BDIC_REG_I2C_DATA6      0x7C
#define BDIC_REG_I2C_READDATA0  0x7D
#define BDIC_REG_I2C_READDATA1  0x7E
#define BDIC_REG_I2C_READDATA2  0x7F
#define BDIC_REG_I2C_READDATA3  0x80
#define BDIC_REG_I2C_READDATA4  0x81
#define BDIC_REG_I2C_READDATA5  0x82
#define BDIC_REG_I2C_SLAVE_SET  0x87
#define BDIC_REG_UART1          0x88
#define BDIC_REG_UART2          0x89
#define BDIC_REG_UART3          0x8A
#define BDIC_REG_SENSOR         0x8B
#define BDIC_REG_SENSOR2        0x8C
#define BDIC_REG_LPOSC          0x8D
#define BDIC_REG_ETC            0x8E
#define BDIC_REG_DETECTOR       0x8F
#define BDIC_REG_OPT0_HT_LSB    0x92
#define BDIC_REG_OPT0_HT_MSB    0x93
#define BDIC_REG_OPT1_LT_LSB    0x94
#define BDIC_REG_OPT1_LT_MSB    0x95
#define BDIC_REG_OPT1_HT_LSB    0x96
#define BDIC_REG_OPT1_HT_MSB    0x97
#define BDIC_REG_OPT2_LT_LSB    0x98
#define BDIC_REG_OPT2_LT_MSB    0x99
#define BDIC_REG_OPT2_HT_LSB    0x9A
#define BDIC_REG_OPT2_HT_MSB    0x9B
#define BDIC_REG_OPT3_LT_LSB    0x9C
#define BDIC_REG_OPT3_LT_MSB    0x9D
#define BDIC_REG_OPT3_HT_LSB    0x9E
#define BDIC_REG_OPT3_HT_MSB    0x9F
#define BDIC_REG_OPT4_LT_LSB    0xA0
#define BDIC_REG_OPT4_LT_MSB    0xA1
#define BDIC_REG_OPT4_HT_LSB    0xA2
#define BDIC_REG_OPT4_HT_MSB    0xA3
#define BDIC_REG_OPT5_LT_LSB    0xA4
#define BDIC_REG_OPT5_LT_MSB    0xA5
#define BDIC_REG_OPT5_HT_LSB    0xA6
#define BDIC_REG_OPT5_HT_MSB    0xA7
#define BDIC_REG_OPT6_LT_LSB    0xA8
#define BDIC_REG_OPT6_LT_MSB    0xA9
#define BDIC_REG_OPT6_HT_LSB    0xAA
#define BDIC_REG_OPT6_HT_MSB    0xAB
#define BDIC_REG_OPT7_LT_LSB    0xAC
#define BDIC_REG_OPT7_LT_MSB    0xAD
#define BDIC_REG_TEST_B0        0xB0
#define BDIC_REG_TEST_B1        0xB1
#define BDIC_REG_TEST_B2        0xB2
#define BDIC_REG_TEST_B3        0xB3
#define BDIC_REG_TEST_B4        0xB4
#define BDIC_REG_TEST_B5        0xB5
#define BDIC_REG_TRIM0          0xC0
#define BDIC_REG_TRIM1          0xC1
#define BDIC_REG_TRIM2          0xC2
#define BDIC_REG_TRIM3          0xC3
#define BDIC_REG_TRIM4          0xC4
#define BDIC_REG_TRIM5          0xC5
#define BDIC_REG_TRIM6          0xC6
#define BDIC_REG_TRIM7          0xC7
#define BDIC_REG_TRIM8          0xC8
#define BDIC_REG_TRIM9          0xC9
#define BDIC_REG_TRIM10         0xCA
#define BDIC_REG_TRIM11         0xCB
#define BDIC_REG_TRIM12         0xCC
#define BDIC_REG_TRIM13         0xCD
#define BDIC_REG_TRIM14         0xCE
#define BDIC_REG_TRIM15         0xCF
#define BDIC_REG_GINF1          0xD0
#define BDIC_REG_GINF2          0xD2
#define BDIC_REG_GINF3          0xD3
#define BDIC_REG_GFAC1          0xD4
#define BDIC_REG_GFAC2          0xD6
#define BDIC_REG_GFAC3          0xD7
#define BDIC_REG_GPOD1          0xD8
#define BDIC_REG_GDIR1          0xDA
#define BDIC_REG_GPPD1          0xDC
#define BDIC_REG_GIMR1          0xE0
#define BDIC_REG_GIMR2          0xE2
#define BDIC_REG_GIMR3          0xE3
#define BDIC_REG_GIMF1          0xE4
#define BDIC_REG_GIMF2          0xE6
#define BDIC_REG_GIMF3          0xE7
#define BDIC_REG_GSCR1          0xE8
#define BDIC_REG_GSCR2          0xEA
#define BDIC_REG_GSCR3          0xEB
#define BDIC_REG_GPOMODE1       0xED
#define BDIC_REG_GPOMODE2       0xEE
#define BDIC_REG_GPIMSK1        0xF0
#define BDIC_REG_GPIMSK2        0xF1
#define BDIC_REG_INT_CTRL       0xF3
#define BDIC_REG_GPIO_SYS       0xF4
#define BDIC_REG_ADC_LCLIP      0xF5
#define BDIC_REG_CLEAR_OFFSET   0xF6
#define BDIC_REG_IR_OFFSET      0xF7
#define BDIC_REG_TEST0          0xF8
#define BDIC_REG_TEST1          0xF9
#define BDIC_REG_TEST2          0xFA
#define BDIC_REG_TEST3          0xFB
#define BDIC_REG_TEST4          0xFC
#define BDIC_REG_TEST5          0xFD
#define BDIC_REG_TEST6          0xFE

#define BDIC_REG_NONE                       (0x00)

#define BDIC_REG_GIFM2_PS_REQ_IMF           (0x02)
#define BDIC_REG_I2C_START_W_START          (0x01)
#define BDIC_REG_I2C_START_R_START          (0x02)
#define BDIC_REG_I2C_START_R_TIMER_START    (0x10)

#define SENSOR_REG_COMMAND1     0x00
#define SENSOR_REG_COMMAND2     0x01
#define SENSOR_REG_COMMAND3     0x02
#define SENSOR_REG_COMMAND4     0x03
#define SENSOR_REG_INT_LT_LSB   0x04
#define SENSOR_REG_INT_LT_MSB   0x05
#define SENSOR_REG_INT_HT_LSB   0x06
#define SENSOR_REG_INT_HT_MSB   0x07
#define SENSOR_REG_PS_LT_LSB    0x08
#define SENSOR_REG_PS_LT_MSB    0x09
#define SENSOR_REG_PS_HT_LSB    0x0A
#define SENSOR_REG_PS_HT_MSB    0x0B
#define SENSOR_REG_D0_LSB       0x0C
#define SENSOR_REG_D0_MSB       0x0D
#define SENSOR_REG_D1_LSB       0x0E
#define SENSOR_REG_D1_MSB       0x0F
#define SENSOR_REG_D2_LSB       0x10
#define SENSOR_REG_D2_MSB       0x11

#define CLMR_SET_BKL_LUT_MODE               (0x00)

#define SHDISP_BDIC_GPIO_COG_RESET                  0

#define SHDISP_BDIC_GPIO_LOW                        0
#define SHDISP_BDIC_GPIO_HIGH                       1

#define SHDISP_BDIC_GPIO_GPOD0                      0
#define SHDISP_BDIC_GPIO_GPOD1                      1
#define SHDISP_BDIC_GPIO_GPOD2                      2
#define SHDISP_BDIC_GPIO_GPOD3                      3
#define SHDISP_BDIC_GPIO_GPOD4                      4
#define SHDISP_BDIC_GPIO_GPOD5                      5

#define SHDISP_BDIC_DEVICE_NONE                     0x00000000
#define SHDISP_BDIC_DEVICE_LCD_BKL                  0x00000001
#define SHDISP_BDIC_DEVICE_LCD_PWR                  0x00000010
#define SHDISP_BDIC_DEVICE_TRI_LED                  0x00000100
#define SHDISP_BDIC_DEVICE_TRI_LED_ANIME            0x00000200
#define SHDISP_BDIC_DEVICE_PHOTO_SENSOR_APP         0x00001000
#define SHDISP_BDIC_DEVICE_PHOTO_SENSOR_LUX         0x00002000
#define SHDISP_BDIC_DEVICE_PHOTO_SENSOR_BKL         0x00004000
#define SHDISP_BDIC_DEVICE_PROX_SENSOR              0x00008000
#define NUM_SHDISP_BDIC_DEVICE                      8

#define SHDISP_BDIC_REQ_ACTIVE                      1
#define SHDISP_BDIC_REQ_STANDBY                     2
#define SHDISP_BDIC_REQ_STOP                        3
#define SHDISP_BDIC_REQ_START                       4

#define SHDISP_BDIC_REQ_BKL_SET_MODE_OFF            5
#define SHDISP_BDIC_REQ_BKL_SET_MODE_FIX            6
#define SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO           7
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL     8
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK      9
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY    10
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED        11
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD       12
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH         13
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH    14
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE           15
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH          16
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA         17
#define SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW        18
#define SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME          19
#define SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL        20
#define SHDISP_BDIC_REQ_TRI_LED_SET_COUNT           21
#define SHDISP_BDIC_REQ_PHOTO_SENSOR_CONFIG         22
#define SHDISP_BDIC_REQ_BKL_DTV_OFF                 25
#define SHDISP_BDIC_REQ_BKL_DTV_ON                  26
#define SHDISP_BDIC_REQ_BKL_EMG_OFF                 27
#define SHDISP_BDIC_REQ_BKL_EMG_ON                  28
#define SHDISP_BDIC_REQ_BKL_ECO_OFF                 29
#define SHDISP_BDIC_REQ_BKL_ECO_ON                  30
#define SHDISP_BDIC_REQ_BKL_CHG_OFF                 31
#define SHDISP_BDIC_REQ_BKL_CHG_ON                  32

#define SHDISP_BDIC_REQ_PRE_START                   33
#define SHDISP_BDIC_REQ_POST_START                  34
#define SHDISP_BDIC_REQ_POST_START_FIX              35

#define SHDISP_BDIC_I2C_SLAVE_ADDR                  (0xA8)
#define SHDISP_BDIC_I2C_WBUF_MAX                    6
#define SHDISP_BDIC_I2C_RBUF_MAX                    6

#define SHDISP_BDIC_INT_GFAC_GFAC0                  0x00000001
#define SHDISP_BDIC_INT_GFAC_GFAC1                  0x00000002
#define SHDISP_BDIC_INT_GFAC_GFAC2                  0x00000004
#define SHDISP_BDIC_INT_GFAC_PS                     0x00000008
#define SHDISP_BDIC_INT_GFAC_GFAC4                  0x00000010
#define SHDISP_BDIC_INT_GFAC_ALS                    0x00000100
#define SHDISP_BDIC_INT_GFAC_PS2                    0x00000200
#define SHDISP_BDIC_INT_GFAC_OPTON                  0x00000400
#define SHDISP_BDIC_INT_GFAC_CPON                   0x00000800
#define SHDISP_BDIC_INT_GFAC_ANIME                  0x00001000
#define SHDISP_BDIC_INT_GFAC_TEST1                  0x00002000
#define SHDISP_BDIC_INT_GFAC_DCDC2_ERR              0x00004000
#define SHDISP_BDIC_INT_GFAC_TSD                    0x00008000
#define SHDISP_BDIC_INT_GFAC_TEST2                  0x00010000
#define SHDISP_BDIC_INT_GFAC_TEST3                  0x00020000
#define SHDISP_BDIC_INT_GFAC_DET                    0x00040000
#define SHDISP_BDIC_INT_GFAC_I2C_ERR                0x00080000
#define SHDISP_BDIC_INT_GFAC_TEST4                  0x00100000
#define SHDISP_BDIC_INT_GFAC_OPTSEL                 0x00200000
#define SHDISP_BDIC_INT_GFAC_TEST5                  0x00400000
#define SHDISP_BDIC_INT_GFAC_TEST6                  0x00800000

#define SHDISP_BKL_TBL_MODE_NORMAL                  0
#define SHDISP_BKL_TBL_MODE_ECO                     1
#define SHDISP_BKL_TBL_MODE_EMERGENCY               2
#define SHDISP_BKL_TBL_MODE_CHARGE                  3

#define SHDISP_BDIC_SENSOR_TYPE_PHOTO               0x01
#define SHDISP_BDIC_SENSOR_TYPE_PROX                0x02
#define SHDISP_BDIC_SENSOR_SLAVE_ADDR               (0x39)

#define SHDISP_OPT_CHANGE_WAIT_TIME                 150

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_PWR_STATUS_OFF,
    SHDISP_BDIC_PWR_STATUS_STANDBY,
    SHDISP_BDIC_PWR_STATUS_ACTIVE,
    NUM_SHDISP_BDIC_PWR_STATUS
};

enum {
    SHDISP_BDIC_DEV_TYPE_LCD_BKL,
    SHDISP_BDIC_DEV_TYPE_LCD_PWR,
    SHDISP_BDIC_DEV_TYPE_TRI_LED,
    SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL,
    SHDISP_BDIC_DEV_TYPE_PROX_SENSOR,
    NUM_SHDISP_BDIC_DEV_TYPE
};

enum {
    SHDISP_BDIC_DEV_PWR_OFF,
    SHDISP_BDIC_DEV_PWR_ON,
    NUM_SHDISP_BDIC_DEV_PWR
};

enum {
    SHDISP_MAIN_BKL_DEV_TYPE_APP,
    SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO,
    NUM_SHDISP_MAIN_BKL_DEV_TYPE
};

enum {
    SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF,
    SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF,
    SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON,
    SHDISP_SENSOR_STATE_PROX_ON_ALC_ON,
    NUM_SHDISP_SENSOR_STATE
};

enum {
    SHDISP_BDIC_IRQ_TYPE_NONE,
    SHDISP_BDIC_IRQ_TYPE_ALS,
    SHDISP_BDIC_IRQ_TYPE_PS,
    SHDISP_BDIC_IRQ_TYPE_DET,
    SHDISP_BDIC_IRQ_TYPE_I2C_ERR,
    NUM_SHDISP_BDIC_IRQ_TYPE
};

enum {
    SHDISP_IRQ_MASK,
    SHDISP_IRQ_NO_MASK,
    NUM_SHDISP_IRQ_SWITCH
};

enum {
    SHDISP_MAIN_BKL_ADJ_RETRY0,
    SHDISP_MAIN_BKL_ADJ_RETRY1,
    SHDISP_MAIN_BKL_ADJ_RETRY2,
    SHDISP_MAIN_BKL_ADJ_RETRY3,
    NUM_SHDISP_MAIN_BKL_ADJ
};

enum {
    SHDISP_BDIC_MAIN_BKL_OPT_LOW,
    SHDISP_BDIC_MAIN_BKL_OPT_HIGH,
    NUM_SHDISP_BDIC_MAIN_BKL_OPT_MODE
};

enum {
    SHDISP_BDIC_PHOTO_LUX_TIMER_ON,
    SHDISP_BDIC_PHOTO_LUX_TIMER_OFF,
    NUM_SHDISP_BDIC_PHOTO_LUX_TIMER_SWITCH
};

enum {
    SHDISP_BDIC_LUX_JUDGE_IN,
    SHDISP_BDIC_LUX_JUDGE_IN_CONTI,
    SHDISP_BDIC_LUX_JUDGE_OUT,
    SHDISP_BDIC_LUX_JUDGE_OUT_CONTI,
    SHDISP_BDIC_LUX_JUDGE_ERROR,
    NUM_SHDISP_BDIC_LUX_JUDGE
};

enum {
    SHDISP_BDIC_BL_PARAM_WRITE = 0,
    SHDISP_BDIC_BL_PARAM_READ,
    SHDISP_BDIC_BL_MODE_SET,
    SHDISP_BDIC_ALS_SET,
    SHDISP_BDIC_ALS_PARAM_WRITE,
    SHDISP_BDIC_ALS_PARAM_READ,
    SHDISP_BDIC_ALS_PARAM_SET,
    SHDISP_BDIC_CABC_CTL,
    SHDISP_BDIC_CABC_CTL_TIME_SET
};

enum {
    SHDISP_BDIC_BL_PWM_FIX_PARAM = 0,
    SHDISP_BDIC_BL_PWM_AUTO_PARAM
};


struct shdisp_bdic_state_str{
    int bdic_is_exist;
    int bdic_main_bkl_opt_mode_output;
    int bdic_main_bkl_opt_mode_ado;
    unsigned char shdisp_lux_change_level1;
    unsigned char shdisp_lux_change_level2;
    int bdic_clrvari_index;
    int clmr_is_exist;
    struct shdisp_prox_params   prox_params;
};

struct shdisp_bdic_bkl_lux_str{
    unsigned int   ext_flag;
    unsigned short ado_range;
    unsigned long  lux;
};

struct shdisp_bdic_led_color_index {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    unsigned char color;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_boot_init( void );
void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str);
void shdisp_bdic_API_bdic_exist(int* bdic_is_exist);
void shdisp_bdic_API_LCD_release_hw_reset(void);
void shdisp_bdic_API_LCD_set_hw_reset(void);
void shdisp_bdic_API_LCD_power_on(void);
void shdisp_bdic_API_LCD_power_off(void);
void shdisp_bdic_API_LCD_m_power_on(void);
void shdisp_bdic_API_LCD_m_power_off(void);
void shdisp_bdic_API_LCD_vo2_on(void);
void shdisp_bdic_API_LCD_vo2_off(void);
void shdisp_bdic_API_LCD_BKL_off(void);
void shdisp_bdic_API_LCD_BKL_fix_on(int param);
void shdisp_bdic_API_LCD_BKL_auto_on(int param);
void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param);
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp);
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp);
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req);
void shdisp_bdic_API_LCD_BKL_dtv_on(void);
void shdisp_bdic_API_LCD_BKL_dtv_off(void);
void shdisp_bdic_API_LCD_BKL_emg_on(void);
void shdisp_bdic_API_LCD_BKL_emg_off(void);
void shdisp_bdic_API_LCD_BKL_eco_on(void);
void shdisp_bdic_API_LCD_BKL_eco_off(void);
void shdisp_bdic_API_LCD_BKL_chg_on(void);
void shdisp_bdic_API_LCD_BKL_chg_off(void);

int  shdisp_bdic_API_TRI_LED_off(void);
unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led );
int  shdisp_bdic_API_TRI_LED_normal_on(unsigned char color);
void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count);
void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count);
#if defined(CONFIG_MACH_LYNX_DL45)
void shdisp_bdic_API_TRI_LED_high_speed_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_standard_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_breath_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_long_breath_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_wave_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_flash_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_aurora_on(int interval, int count);
void shdisp_bdic_API_TRI_LED_rainbow_on(int interval, int count);
#endif  /* defined(CONFIG_MACH_LYNX_DL45) */
int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari );
int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux);
int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode);
int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg);
int shdisp_bdic_API_ALS_transfer(struct shdisp_bdic_i2c_msg *msg);
unsigned char shdisp_bdic_API_I2C_start_judge(void);
void shdisp_bdic_API_I2C_start_ctl(int flg);

int  shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val);
int  shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size);
int  shdisp_bdic_API_RECOVERY_check_restoration(void);
int  shdisp_bdic_API_RECOVERY_check_bdic_practical(void);
#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_bdic_API_DBG_INFO_output(void);
void shdisp_bdic_API_TRI_LED_INFO_output(void);
void shdisp_psals_API_DBG_INFO_output(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

int shdisp_bdic_API_IRQ_set_reg( int irq_type, int onoff );
int  shdisp_bdic_API_IRQ_check_type( int irq_type );
void shdisp_bdic_API_IRQ_save_fac(void);
int  shdisp_bdic_API_IRQ_check_fac(void);
int  shdisp_bdic_API_IRQ_get_fac( int iQueFac );
void shdisp_bdic_API_IRQ_Clear(void);
void shdisp_bdic_API_IRQ_i2c_error_Clear(void);
void shdisp_bdic_API_IRQ_det_fac_Clear(void);
int  shdisp_bdic_API_IRQ_check_DET(void);
void shdisp_bdic_API_IRQ_dbg_Clear_All(void);
void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC);
void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2);
int  shdisp_bdic_api_als_sensor_pow_ctl(int dev_type, int power_mode);
int  shdisp_bdic_PD_set_active(int power_status);
int  shdisp_bdic_PD_set_standby(void);
int  shdisp_bdic_PD_psals_power_on(void);
int  shdisp_bdic_PD_psals_power_off(void);
int  shdisp_bdic_PD_psals_ps_init(int *state);
int  shdisp_bdic_PD_psals_ps_deinit(int *state);
int  shdisp_bdic_PD_psals_als_init(int *state);
int  shdisp_bdic_PD_psals_als_deinit(int *state);
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl);
void shdisp_bdic_api_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj);
void shdisp_bdic_api_set_prox_sensor_param( struct shdisp_prox_params *prox_params);
unsigned short shdisp_bdic_api_get_LC_MLED01(void);
int  shdisp_bdic_API_get_lux_data(void);
void shdisp_bdic_API_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk);
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk);
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk);
int shdisp_bdic_API_psals_is_recovery_successful(void);
int  shdisp_bdic_API_get_sensor_state(void);
void shdisp_bdic_API_RECOVERY_lux_data_backup(void);
void shdisp_bdic_API_RECOVERY_lux_data_restore(void);
void shdisp_bdic_API_psals_active(unsigned long dev_type);
void shdisp_bdic_API_psals_standby(unsigned long dev_type);
void shdisp_bdic_API_ps_background(unsigned long state);
#endif /* SHDISP_BL69Y6_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
