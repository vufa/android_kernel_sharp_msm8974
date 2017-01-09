/* drivers/sharp/shdisp/shdisp_clmr.h  (Display Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

/*---------------------------------------------------------------------------*/
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/*---------------------------------------------------------------------------*/
#ifndef SHDISP_CLMR_H
#define SHDISP_CLMR_H

/*---------------------------------------------------------------------------*/
/* INCLUDE FILES                                                             */
/*---------------------------------------------------------------------------*/

#include <sharp/shdisp_kerl.h>



/*---------------------------------------------------------------------------*/
/* MACROS                                                                    */
/*---------------------------------------------------------------------------*/
#define SHDISP_CLMR_PLL_OFF         0
#define SHDISP_CLMR_PLL_ON          1
#define SHDISP_CLMR_PLL_FORCE_ON    2
#define SHDISP_CLMR_PLL_FORCE_OFF   3

#define SHDISP_CLMR_GPCLK_ON    1
#define SHDISP_CLMR_GPCLK_OFF   0

#define SHDISP_REG_DUMP_DEBUG

#define SHDISP_CLMR_EWB_LUT_NO_0        0
#define SHDISP_CLMR_EWB_LUT_NO_1        1
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_RYOMA) || defined(CONFIG_SHDISP_PANEL_GEMINI)
#define SHDISP_CLMR_EWB_LUT_NUM         2
#else
#define SHDISP_CLMR_EWB_LUT_NUM         1
#endif

#define SHDISP_CLMR_EDRAM_C7    0
#define SHDISP_CLMR_EDRAM_C8    2
#define SHDISP_CLMR_EDRAM_C9    1
#define SHDISP_CLMR_EDRAM_CA    3

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    #define CALI_PLL1CTL_VAL        0x0000B220
  #if defined(CONFIG_MACH_LYNX_DL40) || defined(CONFIG_MACH_LYNX_DL45)
    #define CALI_PLL1CTL2_VAL       0x10901442
    #define CALI_PLL1CTL3_VAL       0x00042001
    #define CALI_PLL1CTL_VAL_B      0x0000B220
    #define CALI_PLL1CTL2_VAL_B     0x01E0143F
    #define CALI_PLL1CTL3_VAL_B     0x00042001
  #elif defined(CONFIG_MACH_DECKARD_AS97) || defined(CONFIG_MACH_DECKARD_AS87)
    #define CALI_PLL1CTL2_VAL       0x28501440
    #define CALI_PLL1CTL3_VAL       0x00043001
    #define CALI_PLL1CTL_VAL_B      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_B     0x1A10143E
    #define CALI_PLL1CTL3_VAL_B     0x00042001
  #elif defined(CONFIG_MACH_TBS)
    #define CALI_PLL1CTL2_VAL       0x1A601440
    #define CALI_PLL1CTL3_VAL       0x00042001
    #define CALI_PLL1CTL_VAL_B      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_B     0x22201441
    #define CALI_PLL1CTL3_VAL_B     CALI_PLL1CTL3_VAL
  #else
    #define CALI_PLL1CTL2_VAL       0x01E0143F
    #define CALI_PLL1CTL3_VAL       0x00042001
    #define CALI_PLL1CTL_VAL_B      0x0000B220
    #define CALI_PLL1CTL2_VAL_B     0x10901442
    #define CALI_PLL1CTL3_VAL_B     0x00042001
  #endif
  #if defined(CONFIG_MACH_TBS)
    #define CALI_PLL1CTL_VAL_C      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_C     0x09801443
    #define CALI_PLL1CTL3_VAL_C     CALI_PLL1CTL3_VAL
  #else
    #define CALI_PLL1CTL_VAL_C      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_C     CALI_PLL1CTL2_VAL
    #define CALI_PLL1CTL3_VAL_C     CALI_PLL1CTL3_VAL
  #endif
#elif defined(CONFIG_SHDISP_PANEL_MARCO) || defined(CONFIG_SHDISP_PANEL_CARIN)
    #define CALI_PLL1CTL_VAL        0x0000B220
    #define CALI_PLL1CTL2_VAL       0x1A601440
    #define CALI_PLL1CTL3_VAL       0x00042001
    #define CALI_PLL1CTL_VAL_B      0x0000B220
    #define CALI_PLL1CTL2_VAL_B     0x22201441
    #define CALI_PLL1CTL3_VAL_B     0x00042001
    #define CALI_PLL1CTL_VAL_C      0x0000B220
    #define CALI_PLL1CTL2_VAL_C     0x09801443
    #define CALI_PLL1CTL3_VAL_C     0x00042001
#elif defined(CONFIG_SHDISP_PANEL_GEMINI)
    #define CALI_PLL1CTL_VAL        (0x0000B220)
    #define CALI_PLL1CTL2_VAL       (0x01E01447)
    #define CALI_PLL1CTL3_VAL       (0x00042001)
    #define CALI_PLL1CTL_VAL_B      (0x0000B220)
    #define CALI_PLL1CTL2_VAL_B     (0x09001446)
    #define CALI_PLL1CTL3_VAL_B     (0x00042001)
    #define CALI_PLL1CTL_VAL_C      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_C     CALI_PLL1CTL2_VAL
    #define CALI_PLL1CTL3_VAL_C     CALI_PLL1CTL3_VAL
#else /* #elif defined(CONFIG_SHDISP_PANEL_RYOMA) */
    #define CALI_PLL1CTL_VAL        0x0000B220
    #define CALI_PLL1CTL2_VAL       0x2520143E
    #define CALI_PLL1CTL3_VAL       0x00043001
    #define CALI_PLL1CTL_VAL_B      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_B     CALI_PLL1CTL2_VAL
    #define CALI_PLL1CTL3_VAL_B     CALI_PLL1CTL3_VAL
    #define CALI_PLL1CTL_VAL_C      CALI_PLL1CTL_VAL
    #define CALI_PLL1CTL2_VAL_C     CALI_PLL1CTL2_VAL
    #define CALI_PLL1CTL3_VAL_C     CALI_PLL1CTL3_VAL
#endif /* defined(CONFIG_SHDISP_PANEL_ANDY) */

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    #define CALI_GPDIV_VAL        0x00000000
    #define CALI_GPDIV_VAL_B      0x00000000
    #define CALI_GPDIV_VAL_C      CALI_GPDIV_VAL
#elif defined(CONFIG_SHDISP_PANEL_MARCO) || defined(CONFIG_SHDISP_PANEL_CARIN)
    #define CALI_GPDIV_VAL        0x00001029
    #define CALI_GPDIV_VAL_B      0x00001027
    #define CALI_GPDIV_VAL_C      0x00001028
#elif defined(CONFIG_SHDISP_PANEL_GEMINI)
    #define CALI_GPDIV_VAL        0x0000100B
    #define CALI_GPDIV_VAL_B      0x0000100B
    #define CALI_GPDIV_VAL_C      CALI_GPDIV_VAL
#else /* #elif defined(CONFIG_SHDISP_PANEL_RYOMA) */
    #define CALI_GPDIV_VAL        0x00000000
    #define CALI_GPDIV_VAL_B      CALI_GPDIV_VAL
    #define CALI_GPDIV_VAL_C      CALI_GPDIV_VAL
#endif /* defined(CONFIG_SHDISP_PANEL_ANDY) */

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    #define CALI_PTGHP_VAL          16
    #define CALI_PTGHB_VAL          44
  #if defined(CONFIG_MACH_LYNX_DL40) || defined(CONFIG_MACH_LYNX_DL45)
    #define CALI_PTGHF_VAL          157
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          6
    #define CALI_PTGVF_VAL          14
    #define CALI_PTGHP_VAL_B        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_B        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_B        95
    #define CALI_PTGVP_VAL_B        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_B        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_B        CALI_PTGVF_VAL
  #elif defined(CONFIG_MACH_DECKARD_AS97) || defined(CONFIG_MACH_DECKARD_AS87)
    #define CALI_PTGHF_VAL          126
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          6
    #define CALI_PTGVF_VAL          14
    #define CALI_PTGHP_VAL_B        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_B        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_B        83
    #define CALI_PTGVP_VAL_B        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_B        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_B        CALI_PTGVF_VAL
  #elif defined(CONFIG_MACH_TBS)
    #define CALI_PTGHF_VAL          122
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          6
    #define CALI_PTGVF_VAL          14
    #define CALI_PTGHP_VAL_B        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_B        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_B        143
    #define CALI_PTGVP_VAL_B        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_B        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_B        CALI_PTGVF_VAL
  #else
    #define CALI_PTGHF_VAL          95
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          9
    #define CALI_PTGVF_VAL          11
    #define CALI_PTGHP_VAL_B        16
    #define CALI_PTGHB_VAL_B        44
    #define CALI_PTGHF_VAL_B        157
    #define CALI_PTGVP_VAL_B        2
    #define CALI_PTGVB_VAL_B        9
    #define CALI_PTGVF_VAL_B        11
  #endif
    #define CALI_PTGHP_VAL_C        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_C        CALI_PTGHB_VAL
  #if defined(CONFIG_MACH_TBS)
    #define CALI_PTGHF_VAL_C        174
  #else
    #define CALI_PTGHF_VAL_C        CALI_PTGHF_VAL
  #endif
    #define CALI_PTGVP_VAL_C        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_C        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_C        CALI_PTGVF_VAL
    #define CALI_PTGHW_VAL          1080
    #define CALI_PTGVW_VAL          1920
    #define CALI_PTGVRAMHW_VAL      101
    #define CALI_PTGSRCXSIZE_VAL    0x00650438
    #define CALI_PTGSRCYSIZE_VAL    0x07850780
    #define CALI_PTGDESXSIZE_VAL    1080
    #define CALI_PTGDESYSIZE_VAL    1920
#elif defined(CONFIG_SHDISP_PANEL_MARCO) || defined(CONFIG_SHDISP_PANEL_CARIN)
    #define CALI_PTGHP_VAL          16
    #define CALI_PTGHB_VAL          44
    #define CALI_PTGHF_VAL          120
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          10
    #define CALI_PTGVF_VAL          10
    #define CALI_PTGHP_VAL_B        16
    #define CALI_PTGHB_VAL_B        44
    #define CALI_PTGHF_VAL_B        140
    #define CALI_PTGVP_VAL_B        2
    #define CALI_PTGVB_VAL_B        10
    #define CALI_PTGVF_VAL_B        10
    #define CALI_PTGHP_VAL_C        16
    #define CALI_PTGHB_VAL_C        44
    #define CALI_PTGHF_VAL_C        172
    #define CALI_PTGVP_VAL_C        2
    #define CALI_PTGVB_VAL_C        10
    #define CALI_PTGVF_VAL_C        10
    #define CALI_PTGHW_VAL          1080
    #define CALI_PTGVW_VAL          1920
    #define CALI_PTGVRAMHW_VAL      101
    #define CALI_PTGSRCXSIZE_VAL    0x00650438
    #define CALI_PTGSRCYSIZE_VAL    0x07850780
    #define CALI_PTGDESXSIZE_VAL    1080
    #define CALI_PTGDESYSIZE_VAL    1920
#elif defined(CONFIG_SHDISP_PANEL_GEMINI)
    #define CALI_PTGHP_VAL          (4)
    #define CALI_PTGHB_VAL          (16)
    #define CALI_PTGHF_VAL          (170)
    #define CALI_PTGVP_VAL          (2)
    #define CALI_PTGVB_VAL          (7)
    #define CALI_PTGVF_VAL          (12)

    #define CALI_PTGHP_VAL_B        (4)
    #define CALI_PTGHB_VAL_B        (16)
    #define CALI_PTGHF_VAL_B        (152)
    #define CALI_PTGVP_VAL_B        (2)
    #define CALI_PTGVB_VAL_B        (7)
    #define CALI_PTGVF_VAL_B        (12)

    #define CALI_PTGHP_VAL_C        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_C        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_C        CALI_PTGHF_VAL
    #define CALI_PTGVP_VAL_C        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_C        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_C        CALI_PTGVF_VAL

    #define CALI_PTGHW_VAL          (1200)
    #define CALI_PTGVW_VAL          (1920)
    #define CALI_PTGVRAMHW_VAL      (139)
    #define CALI_PTGSRCXSIZE_VAL    (0x008B04B0)
    #define CALI_PTGSRCYSIZE_VAL    (0x06120780)
    #define CALI_PTGDESXSIZE_VAL    (1200)
    #define CALI_PTGDESYSIZE_VAL    (1920)
#else /* #elif defined(CONFIG_SHDISP_PANEL_RYOMA) */
    #define CALI_PTGHP_VAL          16
    #define CALI_PTGHB_VAL          44
    #define CALI_PTGHF_VAL          129
    #define CALI_PTGVP_VAL          2
    #define CALI_PTGVB_VAL          6
    #define CALI_PTGVF_VAL          14
    #define CALI_PTGHP_VAL_B        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_B        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_B        CALI_PTGHF_VAL
    #define CALI_PTGVP_VAL_B        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_B        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_B        CALI_PTGVF_VAL
    #define CALI_PTGHP_VAL_C        CALI_PTGHP_VAL
    #define CALI_PTGHB_VAL_C        CALI_PTGHB_VAL
    #define CALI_PTGHF_VAL_C        CALI_PTGHF_VAL
    #define CALI_PTGVP_VAL_C        CALI_PTGVP_VAL
    #define CALI_PTGVB_VAL_C        CALI_PTGVB_VAL
    #define CALI_PTGVF_VAL_C        CALI_PTGVF_VAL
    #define CALI_PTGHW_VAL          720
    #define CALI_PTGVW_VAL          1280
    #define CALI_PTGVRAMHW_VAL      135
    #define CALI_PTGSRCXSIZE_VAL    0x008702D0
    #define CALI_PTGSRCYSIZE_VAL    0x05000500
    #define CALI_PTGDESXSIZE_VAL    720
    #define CALI_PTGDESYSIZE_VAL    1280
#endif /* defined(CONFIG_SHDISP_PANEL_ANDY) */

#if defined (CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_DEBUG_PROCFS_FW_ERROR
    enum {
        SHDISP_CLMR_DBG_FW_NONE,
        SHDISP_CLMR_DBG_FW_TIMEOUT,
        SHDISP_CLMR_DBG_FW_HANDSHAKE,
        SHDISP_CLMR_DBG_FW_CMDNO_DIFFER,
        SHDISP_CLMR_DBG_FW_INTSET1,
        SHDISP_CLMR_DBG_FW_BOOT_NOT_COMP,
        SHDISP_CLMR_DBG_FW_RST_NOT_COMP,
        SHDISP_CLMR_DBG_FW_ERROR_BIT_ON
    };
    enum {
        SHDISP_CLMR_DBG_FW_OK,
        SHDISP_CLMR_DBG_FW_ERROR
    };
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined (CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_GPIO_NUM_PMIC_GPIO35  (35)
#endif /* CONFIG_ANDROID_ENGINEERING */

#define SHDISP_CLMR_RATE_CHECK_MODE_ON    1
#define SHDISP_CLMR_RATE_CHECK_MODE_OFF   0

/*---------------------------------------------------------------------------*/
/* TYPES                                                                     */
/*---------------------------------------------------------------------------*/
enum {
    CALI_STR,
    CALI_OR,
    CALI_AND,
    CALI_RMW,
    CALI_STRM,
};

typedef struct {
    unsigned short addr;
    unsigned char  flg;
    unsigned long  data;
    unsigned long  mask;
    int            wait;
} shdisp_clmrRegSetting_t;

/*---------------------------------------------------------------------------*/
/* VARIABLES                                                                 */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* PROTOTYPES                                                                */
/*---------------------------------------------------------------------------*/
int shdisp_clmr_api_init(struct shdisp_kernel_context *shdisp_kerl_ctx);
#ifndef SHDISP_NOT_SUPPORT_NO_OS
int shdisp_clmr_api_probe(void);
int shdisp_clmr_api_regulator_on(void);
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
void shdisp_clmr_api_exit(void);
int shdisp_clmr_api_power_on(void);
int shdisp_clmr_api_init_fw_lcae(void);
int shdisp_clmr_api_disp_init(void);
int shdisp_clmr_api_power_off(void);
void shdisp_clmr_api_pll_ctrl_reg(int ctrl);
int shdisp_clmr_api_pll_ctrl_gpio(int ctrl);
void shdisp_clmr_api_display_stop(void);
void shdisp_clmr_api_clock_stop(void);
void shdisp_clmr_api_eDramPtr_rst_start(void);
int shdisp_clmr_api_wait4fw_boot_comp(void);
int shdisp_clmr_api_wait4fw_cmd_comp(unsigned char cmdno);
int shdisp_clmr_api_wait4eDramPtr_rst_comp(void);
void shdisp_clmr_api_prepare_handshake(unsigned char cmdno, unsigned char* rbuf, int size);
int shdisp_clmr_api_get_handhake_error(void);

void shdisp_clmr_api_gpclk_ctrl(int ctrl);
int shdisp_clmr_api_lcdc_devcheck(void);

void shdisp_clmr_api_data_transfer_starts(void);
void shdisp_clmr_api_fw_detlcdandbdic_ctrl(int ctrl);
void shdisp_clmr_api_fw_panel_control(int ctrl);
void shdisp_clmr_api_tx_stop(void);
void shdisp_clmr_api_tx_setttingandon(void);

#if defined(SHDISP_REG_DUMP_DEBUG)
    void shdisp_clmr_api_RegDump(int type);
#endif
void shdisp_clmr_api_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, struct shdisp_photo_sensor_adj *adj_out);
int shdisp_clmr_api_custom_blk_init(void);
int shdisp_clmr_api_custom_blk_bkl_on(void);
#if defined(CONFIG_SHDISP_PANEL_GEMINI)
int shdisp_clmr_api_mipi_skew_set(void);
#endif
#if defined(CONFIG_SHDISP_PANEL_RYOMA) || defined(CONFIG_SHDISP_PANEL_GEMINI)
void shdisp_clmr_api_set_device(void);
#endif

void shdisp_clmr_api_rate_check_mode_ctrl(int ctrl);
int shdisp_clmr_api_is_rate_check_mode_ctrl_on(void);
void shdisp_clmr_api_vcom_tracking(void);

void shdisp_clmr_api_set_context_ewb(struct shdisp_clmr_ewb *ewb_param_diffs);
int shdisp_clmr_api_set_ewb_tbl(struct shdisp_clmr_ewb_accu *clmr_ewb_accu, unsigned char no);
int shdisp_clmr_api_diag_set_ewb(struct shdisp_diag_set_ewb *ewb);
int shdisp_clmr_api_diag_read_ewb(struct shdisp_diag_read_ewb *rewb);
void shdisp_clmr_api_convert_ewb_param(struct shdisp_clmr_ewb *ewb_param_diff, struct shdisp_clmr_ewb_accu *ewb_param_accu);
struct shdisp_clmr_ewb_accu* shdisp_clmr_api_get_ewb_accu(unsigned char no);
int shdisp_clmr_api_set_pic_adj_param(struct shdisp_main_pic_adj *pic_adj);
struct shdisp_clmr_trv_info* shdisp_clmr_api_get_trv_info(void);
int shdisp_clmr_api_set_trv_param(struct shdisp_trv_param *trv_param);
int shdisp_clmr_api_set_dbc_param(struct shdisp_main_dbc *dbc);
int shdisp_clmr_api_set_ae_param(struct shdisp_main_ae *ae);
int shdisp_clmr_api_set_pic_adj_ap_type(unsigned short ap_type);
int shdisp_clmr_api_set_flicker_trv(struct shdisp_flicker_trv *flicker_trv);
void shdisp_clmr_api_vsp_on(void);
void shdisp_clmr_api_hsclk_on(void);
void shdisp_clmr_api_hsclk_off(void);
void shdisp_clmr_api_mipi_dsi_tx_circuit_on(void);
void shdisp_clmr_api_mipi_dsi_tx_circuit_off(void);
void shdisp_clmr_api_auto_pat_ctrl(int sw);
void shdisp_clmr_api_SETINT2_0_on(void);
int shdisp_clmr_recover_for_extraordinary(void);

#ifdef SHDISP_USE_LEDC
void shdisp_clmr_api_lcd_vsp_power_on(void);
void shdisp_clmr_api_lcd_vsp_power_off(void);
void shdisp_clmr_api_lcd_vsn_power_on(void);
void shdisp_clmr_api_lcd_vsn_power_off(void);
#endif  /* SHDISP_USE_LEDC */

void shdisp_clmr_set_fw_chg_flg(unsigned int flg);

#if defined (SHDISP_DEBUG_PROCFS_FW_ERROR)
extern int shdisp_debug_clmr_is_fw_break(int count_type);
extern int shdisp_debug_clmr_fw_error_pattern(void);
#endif /* SHDISP_DEBUG_PROCFS_FW_ERROR */

#ifdef SHDISP_GPIO_NUM_PMIC_GPIO35
int shdisp_clmr_api_pmic_gpio35_check(void);
int shdisp_clmr_api_pmic_gpio35_number(void);
#endif

#if defined (CONFIG_ANDROID_ENGINEERING)
unsigned int shdisp_clmr_FWLog_readAndDump(int iseDramPtrRst);
void shdisp_clmr_api_set_pwroff_fwlog_dump_flg(int flg);
#endif /* defined (CONFIG_ANDROID_ENGINEERING) */

void shdisp_clmr_image_preptg_ReadAndDump(void);

/*---------------------------------------------------------------------------*/
/* Register Address                                                          */
/*---------------------------------------------------------------------------*/
#define SHDISP_CLMR_REG_SYSCTL              0x0000
#define SHDISP_CLMR_REG_MCLKDIV             0x0008
#define SHDISP_CLMR_REG_XCLKDIV             0x000C
#define SHDISP_CLMR_REG_REGDIV              0x0010
#define SHDISP_CLMR_REG_REFDIV              0x0014
#define SHDISP_CLMR_REG_VRMDIV              0x0018
#define SHDISP_CLMR_REG_PREDIV              0x001C
#define SHDISP_CLMR_REG_PTGDIV              0x0020
#define SHDISP_CLMR_REG_LAYDIV              0x0024
#define SHDISP_CLMR_REG_TPSTGDIV            0x0028
#define SHDISP_CLMR_REG_TCONTGDIV           0x002C
#define SHDISP_CLMR_REG_TXMPDIV             0x0030
#define SHDISP_CLMR_REG_RXMPDIV             0x0034
#define SHDISP_CLMR_REG_CALDIV              0x0038
#define SHDISP_CLMR_REG_PWMDIV              0x003C
#define SHDISP_CLMR_REG_LUXDIV              0x0040
#define SHDISP_CLMR_REG_ARMTMRDIV           0x0044
#define SHDISP_CLMR_REG_MOSRADIV            0x0048
#define SHDISP_CLMR_REG_I2CDIV              0x004C
#define SHDISP_CLMR_REG_GPDIV               0x0050
#define SHDISP_CLMR_REG_CLKSYS              0x0054
#define SHDISP_CLMR_REG_CLKSYS2             0x0058
#define SHDISP_CLMR_REG_CLKSYS3             0x005C

#define SHDISP_CLMR_REG_CLKSELMASK          0x0070

#define SHDISP_CLMR_REG_PLL0CTL             0x0090
#define SHDISP_CLMR_REG_PLL0CTL2            0x0094
#define SHDISP_CLMR_REG_PLL0CTL3            0x0098

#define SHDISP_CLMR_REG_PLL1CTL             0x00A0
#define SHDISP_CLMR_REG_PLL1CTL2            0x00A4
#define SHDISP_CLMR_REG_PLL1CTL3            0x00A8

#define SHDISP_CLMR_REG_PLLSTAT             0x00B0

#define SHDISP_CLMR_REG_OSCCTL              0x00C0
#define SHDISP_CLMR_REG_OSCCTL2             0x00C4

#define SHDISP_CLMR_REG_INTR                0x0100
#define SHDISP_CLMR_REG_INTM                0x0104
#define SHDISP_CLMR_REG_INTRAW              0x0108

#define SHDISP_CLMR_REG_INTSET1             0x0110
#define SHDISP_CLMR_REG_INTSET2             0x0114

#define SHDISP_CLMR_REG_SETINT1             0x0120
#define SHDISP_CLMR_REG_SETINT2             0x0124

#define SHDISP_CLMR_REG_INFOREG0            0x0140
#define SHDISP_CLMR_REG_INFOREG1            0x0144
#define SHDISP_CLMR_REG_INFOREG2            0x0148
#define SHDISP_CLMR_REG_INFOREG3            0x014C

#define SHDISP_CLMR_REG_SOFTRESET           0x0200

#define SHDISP_CLMR_REG_DEVCODE             0x0300
#define SHDISP_CLMR_REG_SUBDEVCODE          0x0304

#define SHDISP_CLMR_REG_REFCTL              0x0400

#define SHDISP_CLMR_REG_CALCNTCTL           0x0410
#define SHDISP_CLMR_REG_CALCNTA             0x0414
#define SHDISP_CLMR_REG_CALCNTB             0x0418

#define SHDISP_CLMR_REG_PLLONCTL            0x0420
#define SHDISP_CLMR_REG_PLLONCTLREG         0x0424

#define SHDISP_CLMR_REG_EDRAMERR            0x0438
#define SHDISP_CLMR_REG_EDRAMERRMASK        0x043C
#define SHDISP_CLMR_REG_EDRAMCONF           0x0440

#define SHDISP_CLMR_REG_VRAMMODECTL         0x0444

#define SHDISP_CLMR_REG_CSTMSYS             0x0480
#define SHDISP_CLMR_REG_TRVHW               0x0484

#define SHDISP_CLMR_REG_GIOSEL              0x0500
#define SHDISP_CLMR_REG_PFSEL               0x0504
#define SHDISP_CLMR_REG_DSCTL               0x0508

#define SHDISP_CLMR_REG_TSCTL               0x0510

#define SHDISP_CLMR_REG_RXBUFCTL            0x0600
#define SHDISP_CLMR_REG_RXBUFCTL2           0x0604

#define SHDISP_CLMR_REG_TXBUFCTL            0x0620
#define SHDISP_CLMR_REG_TXBUFCTL2           0x0624
#define SHDISP_CLMR_REG_TXBUFCTL3           0x0628

#define SHDISP_CLMR_REG_MDRMSYS             0x0800
#define SHDISP_CLMR_REG_MDRMCTL1            0x0804
#define SHDISP_CLMR_REG_MDRMCTL2            0x0808
#define SHDISP_CLMR_REG_MDRMDPHYCL          0x0810
#define SHDISP_CLMR_REG_MDRMDPHYDL          0x0814
#define SHDISP_CLMR_REG_MDRMDPHYCLCTL       0x0828
#define SHDISP_CLMR_REG_MDRMDPHYDLCTL1      0x082C
#define SHDISP_CLMR_REG_MDRMDPHYDLCTL2      0x0830
#define SHDISP_CLMR_REG_MDRMTEST1           0x0838
#define SHDISP_CLMR_REG_MDRMTEST2           0x083C

#define SHDISP_CLMR_REG_MDRMERRREPSTAT      0x0840
#define SHDISP_CLMR_REG_MDRMERRREPMASK      0x0844

#define SHDISP_CLMR_REG_TXSYS               0x0A00
#define SHDISP_CLMR_REG_MDTMCTL             0x0A04
#define SHDISP_CLMR_REG_MDTMCTL2            0x0A08
#define SHDISP_CLMR_REG_MDTMCMDCTL          0x0A0C
#define SHDISP_CLMR_REG_MDTMCMDADR          0x0A10
#define SHDISP_CLMR_REG_MDTMCMDDATA         0x0A14
#define SHDISP_CLMR_REG_MDTMTRIG            0x0A18

#define SHDISP_CLMR_REG_MDTMHSABYTE         0x0A1C
#define SHDISP_CLMR_REG_MDTMHBPBYTE         0x0A20
#define SHDISP_CLMR_REG_MDTMHFPBYTE         0x0A24
#define SHDISP_CLMR_REG_MDTMHBLBYTE         0x0A28
#define SHDISP_CLMR_REG_MDTMHWBYTE          0x0A2C

#define SHDISP_CLMR_REG_MDTMTHB             0x0A30
#define SHDISP_CLMR_REG_MDTMTHW             0x0A34
#define SHDISP_CLMR_REG_MDTMTVB             0x0A38
#define SHDISP_CLMR_REG_MDTMTVW             0x0A3C
#define SHDISP_CLMR_REG_MDTMTACTL           0x0A40

#define SHDISP_CLMR_REG_MDTMCLKCTL          0x0A54

#define SHDISP_CLMR_REG_MDTMCTCTL           0x0A68
#define SHDISP_CLMR_REG_MDTMCTXTIME         0x0A6C
#define SHDISP_CLMR_REG_MDTMCTYTIME         0x0A70

#define SHDISP_CLMR_REG_MDTMDPHYCL1         0x0A80
#define SHDISP_CLMR_REG_MDTMDPHYCL2         0x0A84
#define SHDISP_CLMR_REG_MDTMDPHYDL1         0x0A88
#define SHDISP_CLMR_REG_MDTMDPHYDL2         0x0A8C

#define SHDISP_CLMR_REG_MDTMLPRXSTAT        0x0A90
#define SHDISP_CLMR_REG_MDTMLPRXMASK        0x0A94
#define SHDISP_CLMR_REG_MDTMLPRXHDR         0x0AA0
#define SHDISP_CLMR_REG_MDTMLPRXADR         0x0AA4
#define SHDISP_CLMR_REG_MDTMLPRXDATA        0x0AA8

#define SHDISP_CLMR_REG_MDTM2CTL            0x0B04
#define SHDISP_CLMR_REG_MDTM2CTL2           0x0B08

#define SHDISP_CLMR_REG_MDTM2CMDCTL         0x0B0C
#define SHDISP_CLMR_REG_MDTM2CMDADR         0x0B10
#define SHDISP_CLMR_REG_MDTM2CMDDATA        0x0B14
#define SHDISP_CLMR_REG_MDTM2TRIG           0x0B18
#define SHDISP_CLMR_REG_MDTM2HSABYTE        0x0B1C
#define SHDISP_CLMR_REG_MDTM2HBPBYTE        0x0B20
#define SHDISP_CLMR_REG_MDTM2HFPBYTE        0x0B24
#define SHDISP_CLMR_REG_MDTM2HBLBYTE        0x0B28
#define SHDISP_CLMR_REG_MDTM2HWBYTE         0x0B2C

#define SHDISP_CLMR_REG_MDTM2THB            0x0B30
#define SHDISP_CLMR_REG_MDTM2THW            0x0B34
#define SHDISP_CLMR_REG_MDTM2TVB            0x0B38
#define SHDISP_CLMR_REG_MDTM2TVW            0x0B3C
#define SHDISP_CLMR_REG_MDTM2TACTL          0x0B40

#define SHDISP_CLMR_REG_MDTM2CLKCTL         0x0B54

#define SHDISP_CLMR_REG_MDTM2CTCTL          0x0B68
#define SHDISP_CLMR_REG_MDTM2CTXTIME        0x0B6C
#define SHDISP_CLMR_REG_MDTM2CTYTIME        0x0B70

#define SHDISP_CLMR_REG_MDTM2DPHYCL1        0x0B80
#define SHDISP_CLMR_REG_MDTM2DPHYCL2        0x0B84
#define SHDISP_CLMR_REG_MDTM2DPHYDL1        0x0B88
#define SHDISP_CLMR_REG_MDTM2DPHYDL2        0x0B8C
#define SHDISP_CLMR_REG_MDTM2LPRXSTAT       0x0B90
#define SHDISP_CLMR_REG_MDTM2LPRXMASK       0x0B94

#define SHDISP_CLMR_REG_MDTM2LPRXHDR        0x0BA0
#define SHDISP_CLMR_REG_MDTM2LPRXADR        0x0BA4
#define SHDISP_CLMR_REG_MDTM2LPRXDATA       0x0BA8

#define SHDISP_CLMR_REG_HOSTSYS             0x0E00
#define SHDISP_CLMR_REG_HOSTCTL             0x0E04
#define SHDISP_CLMR_REG_HOSTASX             0x0E08
#define SHDISP_CLMR_REG_HOSTAEX             0x0E0C
#define SHDISP_CLMR_REG_HOSTASY             0x0E10
#define SHDISP_CLMR_REG_HOSTAEY             0x0E14
#define SHDISP_CLMR_REG_HOSTHW              0x0E18
#define SHDISP_CLMR_REG_HOSTBASE            0x0E1C

#define SHDISP_CLMR_REG_PREMIFCTL           0x1000
#define SHDISP_CLMR_REG_PRECSX              0x1004
#define SHDISP_CLMR_REG_PRECSY              0x1008
#define SHDISP_CLMR_REG_PRECEX              0x100C
#define SHDISP_CLMR_REG_PRECEY              0x1010
#define SHDISP_CLMR_REG_PREXSIZE            0x1014
#define SHDISP_CLMR_REG_PREYSIZE            0x1018
#define SHDISP_CLMR_REG_PREMOCOECTL1        0x101C
#define SHDISP_CLMR_REG_PREMOCOECTL2        0x1020
#define SHDISP_CLMR_REG_PREMOCOECTL3        0x1024
#define SHDISP_CLMR_REG_PREMOCOECTL4        0x1028
#define SHDISP_CLMR_REG_PREMOCOETBL         0x102C
#define SHDISP_CLMR_REG_PREMOCOETBH         0x1030
#define SHDISP_CLMR_REG_PREVRAMHW           0x1034
#define SHDISP_CLMR_REG_PREVRAMBASE         0x1038
#define SHDISP_CLMR_REG_PREINTCTL           0x103C
#define SHDISP_CLMR_REG_PREINT1X            0x1040
#define SHDISP_CLMR_REG_PREINT1Y            0x1044
#define SHDISP_CLMR_REG_PREINT2X            0x1048
#define SHDISP_CLMR_REG_PREINT2Y            0x104C
#define SHDISP_CLMR_REG_PREFMODECTL         0x1050
#define SHDISP_CLMR_REG_PREFSIZE            0x1054

#define SHDISP_CLMR_REG_PREXSTAT            0x1060
#define SHDISP_CLMR_REG_PREYSTAT            0x1064

#define SHDISP_CLMR_REG_NACOCTL             0x1070

#define SHDISP_CLMR_REG_PRESYS              0x10FC

#define SHDISP_CLMR_REG_PSTCTL              0x1200
#define SHDISP_CLMR_REG_PSTVALTRAN          0x1204
#define SHDISP_CLMR_REG_PTGCTL              0x1208
#define SHDISP_CLMR_REG_PTGHP               0x120C
#define SHDISP_CLMR_REG_PTGHB               0x1210
#define SHDISP_CLMR_REG_PTGHW               0x1214
#define SHDISP_CLMR_REG_PTGHF               0x1218
#define SHDISP_CLMR_REG_PTGVP               0x121C
#define SHDISP_CLMR_REG_PTGVB               0x1220
#define SHDISP_CLMR_REG_PTGVW               0x1224
#define SHDISP_CLMR_REG_PTGVF               0x1228
#define SHDISP_CLMR_REG_PTGSTEPCTL          0x122C
#define SHDISP_CLMR_REG_PTGVFS0             0x1230
#define SHDISP_CLMR_REG_PTGVFS1             0x1234
#define SHDISP_CLMR_REG_PTGVFS2             0x1238
#define SHDISP_CLMR_REG_PTGVFS3             0x123C
#define SHDISP_CLMR_REG_PTGVFS4             0x1240
#define SHDISP_CLMR_REG_PTGVFS5             0x1244
#define SHDISP_CLMR_REG_PTGREFCTL           0x1248
#define SHDISP_CLMR_REG_PTGOUTDLY           0x124C
#define SHDISP_CLMR_REG_PTGOUTDLYREF        0x1250
#define SHDISP_CLMR_REG_PTGDANGERSTART      0x1254
#define SHDISP_CLMR_REG_PTGDANGEREND        0x1258
#define SHDISP_CLMR_REG_PTGPVINTLN1         0x125C
#define SHDISP_CLMR_REG_PTGPVINTLN2         0x1260
#define SHDISP_CLMR_REG_PTGVBLKCTL          0x1264
#define SHDISP_CLMR_REG_PTGVBLKS            0x1268
#define SHDISP_CLMR_REG_PTGVBLKE            0x126C
#define SHDISP_CLMR_REG_PTGDCSTECTL         0x1270
#define SHDISP_CLMR_REG_PTGDCSTESCANLINE    0x1274
#define SHDISP_CLMR_REG_PTGTPSYNCCTL        0x1278
#define SHDISP_CLMR_REG_PTGTPSYNCS          0x127C
#define SHDISP_CLMR_REG_PTGTPSYNCE          0x1280
#define SHDISP_CLMR_REG_PTGSLOWVFCTL        0x1284
#define SHDISP_CLMR_REG_PTGSLOWVFMAX        0x1288
#define SHDISP_CLMR_REG_PTGSLOWVFINTCNT1    0x128C
#define SHDISP_CLMR_REG_PTGSLOWVFINTCNT2    0x1290
#define SHDISP_CLMR_REG_PTGSLOWVFDRVPWRLOW  0x1294
#define SHDISP_CLMR_REG_PTGSLOWVFDRVPWRHI   0x1298
#define SHDISP_CLMR_REG_PTGSLOWTPSYNCCTL    0x129C
#define SHDISP_CLMR_REG_PTGSLOWTPSYNCMAX    0x12A0
#define SHDISP_CLMR_REG_PTGSLOWTPSYNCHI     0x12A4
#define SHDISP_CLMR_REG_PTGSLOWTPSYNCLOW    0x12A8
#define SHDISP_CLMR_REG_PTGSLOWTECTL        0x12AC
#define SHDISP_CLMR_REG_PTGSLOWTEMAX        0x12B0
#define SHDISP_CLMR_REG_PTGSLOWTEHI         0x12B4
#define SHDISP_CLMR_REG_PTGSLOWTELOW        0x12B8

#define SHDISP_CLMR_REG_PTGMIFCTL           0x12C0
#define SHDISP_CLMR_REG_PTGSRCXSIZE         0x12C4
#define SHDISP_CLMR_REG_PTGSRCYSIZE         0x12C8
#define SHDISP_CLMR_REG_PTGDESXSIZE         0x12CC
#define SHDISP_CLMR_REG_PTGDESYSIZE         0x12D0
#define SHDISP_CLMR_REG_PTGMOCODCTL1        0x12D4
#define SHDISP_CLMR_REG_PTGMOCODCTL2        0x12D8
#define SHDISP_CLMR_REG_PTGMOCODCTL3        0x12DC
#define SHDISP_CLMR_REG_PTGMOCODCTL4        0x12E0
#define SHDISP_CLMR_REG_PTGMOCODTBL         0x12E4
#define SHDISP_CLMR_REG_PTGMOCODTBH         0x12E8

#define SHDISP_CLMR_REG_PTGFILCTL           0x1300
#define SHDISP_CLMR_REG_PTGFILADR           0x1304
#define SHDISP_CLMR_REG_PTGFILDATA          0x1308
#define SHDISP_CLMR_REG_PTGFMODECTL         0x130C
#define SHDISP_CLMR_REG_PTGFSIZE            0x1310
#define SHDISP_CLMR_REG_PTGPSCX1CTL         0x1314
#define SHDISP_CLMR_REG_PTGPSCX1START       0x1318
#define SHDISP_CLMR_REG_PTGPSCX1END         0x131C
#define SHDISP_CLMR_REG_PTGPSCX2CTL         0x1320
#define SHDISP_CLMR_REG_PTGPSCX2START       0x1324
#define SHDISP_CLMR_REG_PTGPSCX2END         0x1328
#define SHDISP_CLMR_REG_PTGPSCXERAT         0x132C
#define SHDISP_CLMR_REG_PTGPSCYCTL          0x1330
#define SHDISP_CLMR_REG_PTGPSCYSTART        0x1334
#define SHDISP_CLMR_REG_PTGPSCYEND          0x1338
#define SHDISP_CLMR_REG_PTGPSCYERAT         0x133C
#define SHDISP_CLMR_REG_PTGPSCRATADR        0x1340
#define SHDISP_CLMR_REG_PTGPSCRATDAT        0x1344
#define SHDISP_CLMR_REG_PTGCUTCTL           0x1348
#define SHDISP_CLMR_REG_PTGCUTCSX           0x134C
#define SHDISP_CLMR_REG_PTGCUTCEX           0x1350
#define SHDISP_CLMR_REG_PTGCUTCSY           0x1354
#define SHDISP_CLMR_REG_PTGCUTCEY           0x1358
#define SHDISP_CLMR_REG_PTGYOFFSET          0x135C
#define SHDISP_CLMR_REG_PTGVRAMHW           0x1360
#define SHDISP_CLMR_REG_PTGVRAMBASE         0x1364
#define SHDISP_CLMR_REG_PTGLAYRSTX          0x1368
#define SHDISP_CLMR_REG_PTGLAYRSTY          0x136C

#define SHDISP_CLMR_REG_PTGXSTAT            0x1380
#define SHDISP_CLMR_REG_PTGYSTAT            0x1384
#define SHDISP_CLMR_REG_PTGPREFSTAT         0x1388
#define SHDISP_CLMR_REG_PTGHREFSTAT         0x138C
#define SHDISP_CLMR_REG_PTGFIFOSTAT         0x1390
#define SHDISP_CLMR_REG_PTGSTEPSTAT         0x1394
#define SHDISP_CLMR_REG_PTGVBLKSTAT         0x1398
#define SHDISP_CLMR_REG_PTGSLOWSTAT         0x139C
#define SHDISP_CLMR_REG_PTGSLOWCNTSTAT      0x13A0
#define SHDISP_CLMR_REG_PTGSLOWTPCNTSTAT    0x13A4
#define SHDISP_CLMR_REG_PTGSLOWTECNTSTAT    0x13A8

#define SHDISP_CLMR_REG_PSTSYS              0x13FC

#define SHDISP_CLMR_REG_TCSYS               0x1600
#define SHDISP_CLMR_REG_TCCTL               0x1604
#define SHDISP_CLMR_REG_TCTHP               0x1608
#define SHDISP_CLMR_REG_TCTHB               0x160C
#define SHDISP_CLMR_REG_TCTHW0              0x1610
#define SHDISP_CLMR_REG_TCTHW1              0x1614
#define SHDISP_CLMR_REG_TCDELAY             0x1618
#define SHDISP_CLMR_REG_TCTHFMIN            0x161C
#define SHDISP_CLMR_REG_TCHW0OFS            0x1620
#define SHDISP_CLMR_REG_TCHW1OFS            0x1624
#define SHDISP_CLMR_REG_TCTVP               0x1628

#define SHDISP_CLMR_REG_ARMHOSTSYS          0x1800
#define SHDISP_CLMR_REG_ARMDEBEN            0x1804
#define SHDISP_CLMR_REG_ARMCONTCLKEN        0x1808
#define SHDISP_CLMR_REG_ARMTMRCLKEN         0x180C
#define SHDISP_CLMR_REG_ARMSLPCLKEN         0x1810
#define SHDISP_CLMR_REG_ARMTIM0SYS          0x1814
#define SHDISP_CLMR_REG_ARMTIM0CNT          0x1818
#define SHDISP_CLMR_REG_ARMTIM1SYS          0x181C
#define SHDISP_CLMR_REG_ARMTIM1CNT          0x1820
#define SHDISP_CLMR_REG_ARMDMA              0x1824
#define SHDISP_CLMR_REG_ARMSRAMBASE         0x1828
#define SHDISP_CLMR_REG_ARMEDRAMBASE        0x182C
#define SHDISP_CLMR_REG_ARMEDRAMSTART       0x1830
#define SHDISP_CLMR_REG_ARMDMASIZE          0x1834
#define SHDISP_CLMR_REG_ARMDMAHW            0x1838

#define SHDISP_CLMR_REG_ARMUARTCTL          0x1840
#define SHDISP_CLMR_REG_ARMUARTBORT         0x1844
#define SHDISP_CLMR_REG_ARMUARTSTAT         0x1848
#define SHDISP_CLMR_REG_ARMUARTDAT          0x184C

#define SHDISP_CLMR_REG_ARMSRAMCTL          0x1880
#define SHDISP_CLMR_REG_ARMSRAMADR          0x1884
#define SHDISP_CLMR_REG_ARMSRAMDAT          0x1888

#define SHDISP_CLMR_REG_ARMINTR             0x1900
#define SHDISP_CLMR_REG_ARMINTM             0x1904
#define SHDISP_CLMR_REG_ARMINTRAW           0x1908

#define SHDISP_CLMR_REG_ARMINTSET1          0x1910
#define SHDISP_CLMR_REG_ARMINTSET2          0x1914

#define SHDISP_CLMR_REG_ARMSETINT1          0x1920
#define SHDISP_CLMR_REG_ARMSETINT2          0x1924

#define SHDISP_CLMR_REG_ARMINFOREG0         0x1940
#define SHDISP_CLMR_REG_ARMINFOREG1         0x1944
#define SHDISP_CLMR_REG_ARMINFOREG2         0x1948
#define SHDISP_CLMR_REG_ARMINFOREG3         0x194C

#define SHDISP_CLMR_REG_ARMMMU0             0x1980
#define SHDISP_CLMR_REG_ARMMMU1             0x1984
#define SHDISP_CLMR_REG_ARMMMU2             0x1988
#define SHDISP_CLMR_REG_ARMMMU3             0x198C
#define SHDISP_CLMR_REG_ARMMMU4             0x1990
#define SHDISP_CLMR_REG_ARMMMU5             0x1994
#define SHDISP_CLMR_REG_ARMMMU6             0x1998
#define SHDISP_CLMR_REG_ARMMMU7             0x199C
#define SHDISP_CLMR_REG_ARMMMU8             0x19A0
#define SHDISP_CLMR_REG_ARMMMU9             0x19A4
#define SHDISP_CLMR_REG_ARMMMU10            0x19A8
#define SHDISP_CLMR_REG_ARMMMU11            0x19AC
#define SHDISP_CLMR_REG_ARMMMU12            0x19B0
#define SHDISP_CLMR_REG_ARMMMU13            0x19B4
#define SHDISP_CLMR_REG_ARMMMU14            0x19B8
#define SHDISP_CLMR_REG_ARMMMU15            0x19BC
#define SHDISP_CLMR_REG_ARMMMU16            0x19C0
#define SHDISP_CLMR_REG_ARMMMU17            0x19C4
#define SHDISP_CLMR_REG_ARMMMU18            0x19C8
#define SHDISP_CLMR_REG_ARMMMU19            0x19CC
#define SHDISP_CLMR_REG_ARMMMU20            0x19D0
#define SHDISP_CLMR_REG_ARMMMU21            0x19D4
#define SHDISP_CLMR_REG_ARMMMU22            0x19D8
#define SHDISP_CLMR_REG_ARMMMU23            0x19DC
#define SHDISP_CLMR_REG_ARMMMU24            0x19E0
#define SHDISP_CLMR_REG_ARMMMU25            0x19E4
#define SHDISP_CLMR_REG_ARMMMU26            0x19E8
#define SHDISP_CLMR_REG_ARMMMU27            0x19EC
#define SHDISP_CLMR_REG_ARMMMU28            0x19F0
#define SHDISP_CLMR_REG_ARMMMU29            0x19F4
#define SHDISP_CLMR_REG_ARMMMU30            0x19F8
#define SHDISP_CLMR_REG_ARMMMU31            0x19FC

#define SHDISP_CLMR_REG_MOSSYS              0x1A00
#define SHDISP_CLMR_REG_MOSCTL              0x1A04
#define SHDISP_CLMR_REG_MOSCTL2             0x1A08
#define SHDISP_CLMR_REG_MOSDBADR            0x1A0C
#define SHDISP_CLMR_REG_MOSDBDAT            0x1A10
#define SHDISP_CLMR_REG_MOSDBG1             0x1A14
#define SHDISP_CLMR_REG_MOSDBG2             0x1A18
#define SHDISP_CLMR_REG_MOSDBG3             0x1A1C
#define SHDISP_CLMR_REG_MOSDBG4             0x1A20
#define SHDISP_CLMR_REG_MOSSRAMCTL          0x1A24
#define SHDISP_CLMR_REG_MOSSRAMADR          0x1A28
#define SHDISP_CLMR_REG_MOSSRAMDAT          0x1A2C

#define SHDISP_CLMR_REG_I2CSYS              0x1C00
#define SHDISP_CLMR_REG_I2CCTL              0x1C04
#define SHDISP_CLMR_REG_I2CCTL2             0x1C08
#define SHDISP_CLMR_REG_I2CPWM              0x1C0C
#define SHDISP_CLMR_REG_I2CADR              0x1C10
#define SHDISP_CLMR_REG_I2CDRW0             0x1C14
#define SHDISP_CLMR_REG_I2CDRW1             0x1C18
#define SHDISP_CLMR_REG_I2CDRW2             0x1C1C
#define SHDISP_CLMR_REG_I2CDRW3             0x1C20
#define SHDISP_CLMR_REG_I2CSTLIM            0x1C24
#define SHDISP_CLMR_REG_I2CINTSTAT          0x1C28
#define SHDISP_CLMR_REG_I2CINTMASK          0x1C2C
#define SHDISP_CLMR_REG_I2CINTRAW           0x1C30

#define SHDISP_CLMR_REG_EEPSYS              0x1E00
#define SHDISP_CLMR_REG_EEPCTL              0x1E04
#define SHDISP_CLMR_REG_EEPCTL2             0x1E08
#define SHDISP_CLMR_REG_EEPCTL3             0x1E0C
#define SHDISP_CLMR_REG_EEPSTADR            0x1E10
#define SHDISP_CLMR_REG_EEPC32ADR           0x1E14
#define SHDISP_CLMR_REG_EEPC16ADR           0x1E18
#define SHDISP_CLMR_REG_EEPA32ADR           0x1E1C
#define SHDISP_CLMR_REG_EEPA16ADR           0x1E20
#define SHDISP_CLMR_REG_EEPSTAT             0x1E24
#define SHDISP_CLMR_REG_EEPINTSTAT          0x1E28
#define SHDISP_CLMR_REG_EEPINTMASK          0x1E2C
#define SHDISP_CLMR_REG_EEPINTRAW           0x1E30

#define SHDISP_CLMR_REG_GIO00               0x2000
#define SHDISP_CLMR_REG_GIO01               0x2004
#define SHDISP_CLMR_REG_GIO02               0x2008
#define SHDISP_CLMR_REG_GIO03               0x200C
#define SHDISP_CLMR_REG_GIO04               0x2010
#define SHDISP_CLMR_REG_GIO05               0x2014
#define SHDISP_CLMR_REG_GIO06               0x2018
#define SHDISP_CLMR_REG_GIO07               0x201C
#define SHDISP_CLMR_REG_GIO08               0x2020
#define SHDISP_CLMR_REG_GIO09               0x2024
#define SHDISP_CLMR_REG_GIO10               0x2028
#define SHDISP_CLMR_REG_GIO11               0x202C
#define SHDISP_CLMR_REG_GIO12               0x2030
#define SHDISP_CLMR_REG_GIO13               0x2034
#define SHDISP_CLMR_REG_GIO14               0x2038

#define SHDISP_CLMR_REG_TESTMODE00          0x4000
#define SHDISP_CLMR_REG_TESTMODE01          0x4004
#define SHDISP_CLMR_REG_TESTMODE02          0x4008
#define SHDISP_CLMR_REG_TESTMODE03          0x400C
#define SHDISP_CLMR_REG_TESTMODE04          0x4010
#define SHDISP_CLMR_REG_TESTMODE05          0x4014
#define SHDISP_CLMR_REG_TESTMODE06          0x4018
#define SHDISP_CLMR_REG_TESTMODE07          0x401C
#define SHDISP_CLMR_REG_TESTMODE08          0x4020
#define SHDISP_CLMR_REG_TESTMODE09          0x4024
#define SHDISP_CLMR_REG_TESTMODE10          0x4028
#define SHDISP_CLMR_REG_TESTMODE11          0x402C
#define SHDISP_CLMR_REG_TESTMODE12          0x4030
#define SHDISP_CLMR_REG_TESTMODE13          0x4034
#define SHDISP_CLMR_REG_TESTMODE14          0x4038
#define SHDISP_CLMR_REG_TESTMODE15          0x403C

#define SHDISP_CLMR_CUST_AWHS               0x8004

#define SHDISP_CLMR_CUST_OUTPUTSEL          0x802C
#define SHDISP_CLMR_CUST_VSPREGON           0x8030

#define SHDISP_CLMR_CUST_VSPCTRL1           0x8038
#define SHDISP_CLMR_CUST_VSPCTRL2           0x803C
#define SHDISP_CLMR_CUST_VSPCTRL3           0x8040

#define SHDISP_CLMR_CUST_VSPCTRL5           0x8048
#define SHDISP_CLMR_CUST_VSPCTRL6           0x804C

#define SHDISP_CLMR_CUST_VSPCTRL8           0x8054
#define SHDISP_CLMR_CUST_VSPCTRL9           0x8058

#define SHDISP_CLMR_CUST_INTSTAT0           0x8060

#define SHDISP_CLMR_CUST_INTMASK0           0x8068

#define SHDISP_CLMR_CUST_INTSET0            0x8070

#define SHDISP_CLMR_CUST_INTRST0            0x8078

#define SHDISP_CLMR_CUST_STINT              0x8090

#define SHDISP_CLMR_CUST_TUNE1              0x8098

#define SHDISP_CLMR_CUST_STCNT1             0x80A0

#define SHDISP_CLMR_CUST_VSP_TEST           0x80EC

#define SHDISP_CLMR_CUST_CLKGATEOFF         0x80F8
#define SHDISP_CLMR_CUST_CUSTOM_INT         0x80FC


#define SHDISP_CLMR_CUST_LUXCTRL            0x8100
#define SHDISP_CLMR_CUST_LUXDATA0           0x8104
#define SHDISP_CLMR_CUST_LUXDATA1           0x8108
#define SHDISP_CLMR_CUST_LUXNC              0x810C

#define SHDISP_CLMR_CUST_NP1CTL             0x8140
#define SHDISP_CLMR_CUST_NP1RSLT            0x8144
#define SHDISP_CLMR_CUST_NP1THRE            0x8148
#define SHDISP_CLMR_CUST_NP1DLY             0x814C
#define SHDISP_CLMR_CUST_NP1LONG            0x8150

#define SHDISP_CLMR_CUST_PWMCTRL            0x815C
#define SHDISP_CLMR_CUST_PWMDLYSEL          0x8160
#define SHDISP_CLMR_CUST_PWMOUT             0x8164

#define SHDISP_CLMR_CUST_SBLFMT             0x8180
#define SHDISP_CLMR_CUST_SBLSIZE0           0x8184
#define SHDISP_CLMR_CUST_SBLSIZE1           0x8188
#define SHDISP_CLMR_CUST_SBLSET0            0x818C
#define SHDISP_CLMR_CUST_SBLSET1            0x8190
#define SHDISP_CLMR_CUST_SBLSET2            0x8194
#define SHDISP_CLMR_CUST_SBLLUT_FI          0x8198
#define SHDISP_CLMR_CUST_SBLLUT_CC          0x819C
#define SHDISP_CLMR_CUST_APISET0            0x81A0
#define SHDISP_CLMR_CUST_APISET1            0x81A4
#define SHDISP_CLMR_CUST_APISET2            0x81A8
#define SHDISP_CLMR_CUST_CALIBRATION        0x81AC
#define SHDISP_CLMR_CUST_DRC                0x81B0
#define SHDISP_CLMR_CUST_APIMON             0x81B4
#define SHDISP_CLMR_CUST_ALCALIBLUT         0x81B8
#define SHDISP_CLMR_CUST_LOGO               0x81BC
#define SHDISP_CLMR_CUST_SBLTEST            0x81C0


#define SHDISP_CLMR_CUST_EDIF               0x8200

#define SHDISP_CLMR_CUST_ITLC               0x8230

#define SHDISP_CLMR_CUST_Y2R_R2Y            0x8240

#define SHDISP_CLMR_CUST_CPFAEXY            0x8280
#define SHDISP_CLMR_CUST_CPFASXY            0x8284
#define SHDISP_CLMR_CUST_CPFCTRL0           0x8288
#define SHDISP_CLMR_CUST_CPFCTRL1           0x828C
#define SHDISP_CLMR_CUST_CPFCTRL2           0x8290
#define SHDISP_CLMR_CUST_CPFCTRL3           0x8294
#define SHDISP_CLMR_CUST_CPFSTRDAT0         0x8298
#define SHDISP_CLMR_CUST_CPFSTRDAT1         0x829C
#define SHDISP_CLMR_CUST_CPFSTRDAT2         0x82A0
#define SHDISP_CLMR_CUST_CPFFIXLUT          0x82A4
#define SHDISP_CLMR_CUST_CPFLUTVFVAL0       0x82A8
#define SHDISP_CLMR_CUST_CPFLUTVFVAL1       0x82AC
#define SHDISP_CLMR_CUST_CPFLUTVFVAL2       0x82B0

#define SHDISP_CLMR_CUST_SPCOLCNT           0x82C0
#define SHDISP_CLMR_CUST_SPCOLR             0x82C4
#define SHDISP_CLMR_CUST_SPCOLG             0x82C8
#define SHDISP_CLMR_CUST_SPCOLB             0x82CC
#define SHDISP_CLMR_CUST_EWBPAT0            0x82D0
#define SHDISP_CLMR_CUST_EWBPAT1            0x82D4
#define SHDISP_CLMR_CUST_EWBPAT2            0x82D8
#define SHDISP_CLMR_CUST_EWBPAT3            0x82DC
#define SHDISP_CLMR_CUST_EWBLUTADR          0x82E0
#define SHDISP_CLMR_CUST_EWBWRDAT           0x82E4

#define SHDISP_CLMR_CUST_EWBRDDAT0          0x82EC
#define SHDISP_CLMR_CUST_EWBRDDAT1          0x82F0
#define SHDISP_CLMR_CUST_EWBWREXEC          0x82F4
#define SHDISP_CLMR_CUST_EWBEX              0x82F8


#define SHDISP_CLMR_CUST_TRVASAXY0          0x8300
#define SHDISP_CLMR_CUST_TRVAWHS0           0x8304
#define SHDISP_CLMR_CUST_TRVTEXADR0         0x8308
#define SHDISP_CLMR_CUST_TRVTEXADR1         0x830C
#define SHDISP_CLMR_CUST_TRVTEXADR2         0x8310
#define SHDISP_CLMR_CUST_TRVTEXADR3         0x8314
#define SHDISP_CLMR_CUST_TRVDATA            0x8318
#define SHDISP_CLMR_CUST_TRVLUTADR          0x831C
#define SHDISP_CLMR_CUST_TRVCTL             0x8320
#define SHDISP_CLMR_CUST_TRVCFG             0x8324
#define SHDISP_CLMR_CUST_TRVTEXOFT          0x8328
#define SHDISP_CLMR_CUST_TRVPAT0            0x832C
#define SHDISP_CLMR_CUST_TRVPAT1            0x8330
#define SHDISP_CLMR_CUST_TRVFPAT0           0x8334
#define SHDISP_CLMR_CUST_TRVFPAT1           0x8338
#define SHDISP_CLMR_CUST_TRVCOLMAP0         0x833C
#define SHDISP_CLMR_CUST_TRVCOLMAP1         0x8340
#define SHDISP_CLMR_CUST_TRVSHIFT           0x8344
#define SHDISP_CLMR_CUST_TRVANGLX           0x8348
#define SHDISP_CLMR_CUST_TRVANGLY           0x834C
#define SHDISP_CLMR_CUST_TRVANGLH           0x8350
#define SHDISP_CLMR_CUST_TRVOSTFLT0         0x8354
#define SHDISP_CLMR_CUST_TRVOSTFLT1         0x8358
#define SHDISP_CLMR_CUST_TRVCSCREF          0x835C
#define SHDISP_CLMR_CUST_TRVADPPAL          0x8360
#define SHDISP_CLMR_CUST_TRVSTEP            0x8364
#define SHDISP_CLMR_CUST_TRVADPSET          0x8368
#define SHDISP_CLMR_CUST_TRVEX              0x836C
#define SHDISP_CLMR_CUST_TRVREQ             0x8370
#define SHDISP_CLMR_CUST_TRVRAMHW           0x8374


#define SHDISP_CLMR_CUST_SVCTASAXU          0x8400
#define SHDISP_CLMR_CUST_SVCTASAYU          0x8404
#define SHDISP_CLMR_CUST_SVCTAWSU           0x8408
#define SHDISP_CLMR_CUST_SVCTAHSU           0x840C
#define SHDISP_CLMR_CUST_SVCTASAXL          0x8410
#define SHDISP_CLMR_CUST_SVCTASAYL          0x8414
#define SHDISP_CLMR_CUST_SVCTAWSL           0x8418
#define SHDISP_CLMR_CUST_SVCTAHSL           0x841C

#define SHDISP_CLMR_CUST_SVCTYCCRSU         0x8440
#define SHDISP_CLMR_CUST_SVCTYCBLSU         0x8444
#define SHDISP_CLMR_CUST_SVCTYCBRSU         0x8448
#define SHDISP_CLMR_CUST_SVCTYCMPAR0U       0x844C
#define SHDISP_CLMR_CUST_SVCTYCMPAR1U       0x8450
#define SHDISP_CLMR_CUST_SVCTYCSFSU         0x8454
#define SHDISP_CLMR_CUST_SVCTYCUVCSU        0x8458
#define SHDISP_CLMR_CUST_SVCTYCDEGU         0x845C
#define SHDISP_CLMR_CUST_SVCTYCRGBA1U       0x8460
#define SHDISP_CLMR_CUST_SVCTYCCMYA1U       0x8464
#define SHDISP_CLMR_CUST_SVCTYCRGBA2U       0x8468
#define SHDISP_CLMR_CUST_SVCTYCCMYA2U       0x846C
#define SHDISP_CLMR_CUST_SVCTYCRGBA3U       0x8470
#define SHDISP_CLMR_CUST_SVCTYCCMYA3U       0x8474
#define SHDISP_CLMR_CUST_SVCTYCMCSU         0x8478

#define SHDISP_CLMR_CUST_SVCTYCCRSL         0x8480
#define SHDISP_CLMR_CUST_SVCTYCBLSL         0x8484
#define SHDISP_CLMR_CUST_SVCTYCBRSL         0x8488
#define SHDISP_CLMR_CUST_SVCTYCMPAR0L       0x848C
#define SHDISP_CLMR_CUST_SVCTYCMPAR1L       0x8490
#define SHDISP_CLMR_CUST_SVCTYCSFSL         0x8494
#define SHDISP_CLMR_CUST_SVCTYCUVCSL        0x8498
#define SHDISP_CLMR_CUST_SVCTYCDEGL         0x849C
#define SHDISP_CLMR_CUST_SVCTYCRGBA1L       0x84A0
#define SHDISP_CLMR_CUST_SVCTYCCMYA1L       0x84A4
#define SHDISP_CLMR_CUST_SVCTYCRGBA2L       0x84A8
#define SHDISP_CLMR_CUST_SVCTYCCMYA2L       0x84AC
#define SHDISP_CLMR_CUST_SVCTYCRGBA3L       0x84B0
#define SHDISP_CLMR_CUST_SVCTYCCMYA3L       0x84B4
#define SHDISP_CLMR_CUST_SVCTYCMCSL         0x84B8


#define SHDISP_CLMR_CUST_PCAPRM0            0x8500
#define SHDISP_CLMR_CUST_PCAPRM1            0x8504
#define SHDISP_CLMR_CUST_PCAPRM2            0x8508
#define SHDISP_CLMR_CUST_PCAPRM3            0x850C
#define SHDISP_CLMR_CUST_PCAPRM4            0x8510
#define SHDISP_CLMR_CUST_PCAPRM5            0x8514
#define SHDISP_CLMR_CUST_PCAPRM6            0x8518
#define SHDISP_CLMR_CUST_PCAPRM7            0x851C
#define SHDISP_CLMR_CUST_PCAPRM8            0x8520
#define SHDISP_CLMR_CUST_PCAPRM9            0x8524
#define SHDISP_CLMR_CUST_PCAPRM10           0x8528
#define SHDISP_CLMR_CUST_PCAPRM11           0x852C
#define SHDISP_CLMR_CUST_PCAPRM12           0x8530
#define SHDISP_CLMR_CUST_PCAPRM13           0x8534
#define SHDISP_CLMR_CUST_PCAPRM14           0x8538
#define SHDISP_CLMR_CUST_PCAPRM15           0x853C
#define SHDISP_CLMR_CUST_PCAPRM16           0x8540
#define SHDISP_CLMR_CUST_PCAPRM17           0x8544
#define SHDISP_CLMR_CUST_PCAPRM18           0x8548

#define SHDISP_CLMR_CUST_HSV0               0x85A0
#define SHDISP_CLMR_CUST_HSV1               0x85A4
#define SHDISP_CLMR_CUST_HSV2               0x85A8
#define SHDISP_CLMR_CUST_HSV3               0x85AC
#define SHDISP_CLMR_CUST_HSV4               0x85B0
#define SHDISP_CLMR_CUST_HSV5               0x85B4
#define SHDISP_CLMR_CUST_HSV6               0x85B8
#define SHDISP_CLMR_CUST_HSV7               0x85BC
#define SHDISP_CLMR_CUST_HSV8               0x85C0
#define SHDISP_CLMR_CUST_HSV9               0x85C4
#define SHDISP_CLMR_CUST_HSV10              0x85C8
#define SHDISP_CLMR_CUST_HSV11              0x85CC
#define SHDISP_CLMR_CUST_HSV12              0x85D0
#define SHDISP_CLMR_CUST_HSV13              0x85D4
#define SHDISP_CLMR_CUST_HSV14              0x85D8
#define SHDISP_CLMR_CUST_EACT0              0x85DC
#define SHDISP_CLMR_CUST_EACT1              0x85E0
#define SHDISP_CLMR_CUST_EACT2              0x85E4
#define SHDISP_CLMR_CUST_EACT3              0x85E8
#define SHDISP_CLMR_CUST_EACT4              0x85EC


#define SHDISP_CLMR_CUST_SVCTDSP1U          0x8600
#define SHDISP_CLMR_CUST_SVCTDSP2U          0x8604
#define SHDISP_CLMR_CUST_SVCTDSP3U          0x8608
#define SHDISP_CLMR_CUST_SVCTDSP4U          0x860C
#define SHDISP_CLMR_CUST_SVCTDSPHU          0x8610
#define SHDISP_CLMR_CUST_SVCTDHUEU          0x8614
#define SHDISP_CLMR_CUST_SVCTDAXISU         0x8618
#define SHDISP_CLMR_CUST_SVCTDUNITU         0x861C
#define SHDISP_CLMR_CUST_SVCTDSP1L          0x8620
#define SHDISP_CLMR_CUST_SVCTDSP2L          0x8624
#define SHDISP_CLMR_CUST_SVCTDSP3L          0x8628
#define SHDISP_CLMR_CUST_SVCTDSP4L          0x862C
#define SHDISP_CLMR_CUST_SVCTDSPHL          0x8630
#define SHDISP_CLMR_CUST_SVCTDHUEL          0x8634
#define SHDISP_CLMR_CUST_SVCTDAXISL         0x8638
#define SHDISP_CLMR_CUST_SVCTDUNITL         0x863C
#define SHDISP_CLMR_CUST_SVCTRCCSRU         0x8640
#define SHDISP_CLMR_CUST_SVCTRCCSGU         0x8644
#define SHDISP_CLMR_CUST_SVCTRCCSBU         0x8648
#define SHDISP_CLMR_CUST_SVCTRCCRVU         0x864C
#define SHDISP_CLMR_CUST_SVCTRCCSRL         0x8650
#define SHDISP_CLMR_CUST_SVCTRCCSGL         0x8654
#define SHDISP_CLMR_CUST_SVCTRCCSBL         0x8658
#define SHDISP_CLMR_CUST_SVCTRCCRVL         0x865C

#define SHDISP_CLMR_CUST_SVCTYBBLRU         0x8680
#define SHDISP_CLMR_CUST_SVCTYBBRRU         0x8684
#define SHDISP_CLMR_CUST_SVCTYBSCRU         0x8688
#define SHDISP_CLMR_CUST_SVCTYBCRVU         0x868C
#define SHDISP_CLMR_CUST_SVCTYBBLRL         0x8690
#define SHDISP_CLMR_CUST_SVCTYBBRRL         0x8694
#define SHDISP_CLMR_CUST_SVCTYBSCRL         0x8698
#define SHDISP_CLMR_CUST_SVCTYBCRVL         0x869C
#define SHDISP_CLMR_CUST_SVCTYBBLRS0        0x86A0
#define SHDISP_CLMR_CUST_SVCTYBBLRS1        0x86A4
#define SHDISP_CLMR_CUST_SVCTYBBLRS2        0x86A8
#define SHDISP_CLMR_CUST_SVCTYBBLRS3        0x86AC
#define SHDISP_CLMR_CUST_SVCTYBBLRS4        0x86B0
#define SHDISP_CLMR_CUST_SVCTYBBLRS5        0x86B4
#define SHDISP_CLMR_CUST_SVCTYBBLRS6        0x86B8
#define SHDISP_CLMR_CUST_SVCTYBBLRS7        0x86BC
#define SHDISP_CLMR_CUST_SVCTYBBRRS0        0x86C0
#define SHDISP_CLMR_CUST_SVCTYBBRRS1        0x86C4
#define SHDISP_CLMR_CUST_SVCTYBBRRS2        0x86C8
#define SHDISP_CLMR_CUST_SVCTYBBRRS3        0x86CC
#define SHDISP_CLMR_CUST_SVCTYBBRRS4        0x86D0
#define SHDISP_CLMR_CUST_SVCTYBBRRS5        0x86D4
#define SHDISP_CLMR_CUST_SVCTYBBRRS6        0x86D8
#define SHDISP_CLMR_CUST_SVCTYBBRRS7        0x86DC
#define SHDISP_CLMR_CUST_SVCTYBSCRS0        0x86E0
#define SHDISP_CLMR_CUST_SVCTYBSCRS1        0x86E4
#define SHDISP_CLMR_CUST_SVCTYBSCRS2        0x86E8
#define SHDISP_CLMR_CUST_SVCTYBSCRS3        0x86EC
#define SHDISP_CLMR_CUST_SVCTYBSCRS4        0x86F0
#define SHDISP_CLMR_CUST_SVCTYBSCRS5        0x86F4
#define SHDISP_CLMR_CUST_SVCTYBSCRS6        0x86F8
#define SHDISP_CLMR_CUST_SVCTYBSCRS7        0x86FC


#define SHDISP_CLMR_CUST_SMITHSYNC          0x8728

#define SHDISP_CLMR_CUST_SVINTMASK0         0x8730

#define SHDISP_CLMR_CUST_HISTINTF           0x8740
#define SHDISP_CLMR_CUST_HISTINTE           0x8744
#define SHDISP_CLMR_CUST_HISTINTW           0x8748

#define SHDISP_CLMR_CUST_HISTCTRL           0x8750
#define SHDISP_CLMR_CUST_HISTMODE           0x8754
#define SHDISP_CLMR_CUST_HISTAREAX          0x8758
#define SHDISP_CLMR_CUST_HISTAREAY          0x875C
#define SHDISP_CLMR_CUST_HISTAREAW          0x8760
#define SHDISP_CLMR_CUST_HISTAREAH          0x8764
#define SHDISP_CLMR_CUST_HISTBLEND          0x8768

#define SHDISP_CLMR_CUST_HISTDAT0           0x8780
#define SHDISP_CLMR_CUST_HISTDAT1           0x8784
#define SHDISP_CLMR_CUST_HISTDAT2           0x8788
#define SHDISP_CLMR_CUST_HISTDAT3           0x878C
#define SHDISP_CLMR_CUST_HISTDAT4           0x8790
#define SHDISP_CLMR_CUST_HISTDAT5           0x8794
#define SHDISP_CLMR_CUST_HISTDAT6           0x8798
#define SHDISP_CLMR_CUST_HISTDAT7           0x879C
#define SHDISP_CLMR_CUST_HISTDAT8           0x87A0
#define SHDISP_CLMR_CUST_HISTDAT9           0x87A4
#define SHDISP_CLMR_CUST_HISTDAT10          0x87A8
#define SHDISP_CLMR_CUST_HISTDAT11          0x87AC
#define SHDISP_CLMR_CUST_HISTDAT12          0x87B0
#define SHDISP_CLMR_CUST_HISTDAT13          0x87B4
#define SHDISP_CLMR_CUST_HISTDAT14          0x87B8
#define SHDISP_CLMR_CUST_HISTDAT15          0x87BC
#define SHDISP_CLMR_CUST_HISTDAT16          0x87C0
#define SHDISP_CLMR_CUST_HISTDAT17          0x87C4
#define SHDISP_CLMR_CUST_HISTDAT18          0x87C8
#define SHDISP_CLMR_CUST_HISTDAT19          0x87CC
#define SHDISP_CLMR_CUST_HISTDAT20          0x87D0
#define SHDISP_CLMR_CUST_HISTDAT21          0x87D4
#define SHDISP_CLMR_CUST_HISTDAT22          0x87D8
#define SHDISP_CLMR_CUST_HISTDAT23          0x87DC
#define SHDISP_CLMR_CUST_HISTDAT24          0x87E0
#define SHDISP_CLMR_CUST_HISTDAT25          0x87E4
#define SHDISP_CLMR_CUST_HISTDAT26          0x87E8
#define SHDISP_CLMR_CUST_HISTDAT27          0x87EC
#define SHDISP_CLMR_CUST_HISTDAT28          0x87F0
#define SHDISP_CLMR_CUST_HISTDAT29          0x87F4
#define SHDISP_CLMR_CUST_HISTDAT30          0x87F8
#define SHDISP_CLMR_CUST_HISTDAT31          0x87FC

#endif
/*---------------------------------------------------------------------------*/
/* END OF FILE                                                               */
/*---------------------------------------------------------------------------*/
