/* drivers/sharp/shdisp/shdisp_andy.c  (Display Driver)
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
#include <linux/regulator/consumer.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_panel.h"
#include "shdisp_andy.h"
#include "shdisp_system.h"
#include "shdisp_bl69y6.h"
#include "shdisp_type.h"
#include "shdisp_pm.h"
#include "shdisp_clmr.h"
#include "shdisp_dbg.h"



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_DET
#if defined(CONFIG_MACH_LYNX_DL45) || defined(CONFIG_MACH_DECKARD_AS87) || defined(CONFIG_MACH_TBS)
#define SHDISP_POWER_MODE_CHK
#endif
#endif
#define SHDISP_FW_STACK_EXCUTE

#define SHDISP_ANDY_RESET      32
#define SHDISP_ANDY_DISABLE    0
#define SHDISP_ANDY_ENABLE     1

#define SHDISP_ANDY_1V_WAIT                 18000
#define SHDISP_ANDY_VCOM_MIN                0x0000
#define SHDISP_ANDY_VCOM_MAX                0x0190
   static unsigned char current_page = 0;
#define ANDY_VCOM_REG_NUM                   (6)

#define SHDISP_ANDY_GAMMA_SETTING_SIZE      60
#define SHDISP_ANDY_GAMMA_LEVEL_MIN         1
#define SHDISP_ANDY_GAMMA_LEVEL_MAX         30
#define SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET   30
#define SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL 2
#define SHDISP_ANDY_GAMMA_GROUP_BELONG_ADDR  4
#define SHDISP_ANDY_VGH         2
#define SHDISP_ANDY_VGL         3
#define SHDISP_ANDY_VDD_Reg     4
#define SHDISP_ANDY_GVDDP       6
#define SHDISP_ANDY_GVDDN       7
#define SHDISP_ANDY_GVDDP2      8
#define SHDISP_ANDY_VGHO        10
#define SHDISP_ANDY_VGLO        11
#define SHDISP_ANDY_AVDDR       15
#define SHDISP_ANDY_AVEER       16

#define MIPI_SHARP_CLMR_1HZ_BLACK_ON    1
#define MIPI_SHARP_CLMR_AUTO_PAT_OFF    0

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_andy_API_init_io(void);
static int shdisp_andy_API_exit_io(void);
static int shdisp_andy_API_power_on(int mode);
static int shdisp_andy_API_power_off(int mode);
static int shdisp_andy_API_disp_on(void);
static int shdisp_andy_API_disp_off(void);
static int shdisp_andy_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_andy_API_diag_write_reg(int cog, unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_andy_API_diag_read_reg(int cog, unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_andy_API_diag_set_flicker_param(struct shdisp_diag_flicker_param alpha);
static int shdisp_andy_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_andy_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_andy_API_check_recovery(void);
static int shdisp_andy_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_andy_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_andy_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);
static int shdisp_andy_sqe_set_drive_freq(int type);
static int shdisp_andy_API_set_drive_freq(int type);
static int shdisp_andy_API_get_drive_freq(void);
static int shdisp_andy_API_shutdown(void);
static int shdisp_andy_mipi_cmd_lcd_on_after_black_screen(void);
static int shdisp_andy_API_start_display(void);

static int shdisp_andy_mipi_cmd_lcd_on(void);
static int shdisp_andy_mipi_cmd_lcd_off(void);

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_panel_andy_power_mode_chk(unsigned char addr);
#endif

int shdisp_andy_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma);



/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char andy_wdata[8];
static unsigned char andy_rdata[8];
static char Andy_VCOM_Reg[6] = { 0x13, 0x14, 0x15, 0x5C, 0x5D, 0x5E };
#endif
static struct dsi_buf shdisp_mipi_andy_tx_buf;
static struct dsi_buf shdisp_mipi_andy_rx_buf;

static char mipi_sh_andy_set_val_GAMMAREDposi[60][2];
static char mipi_sh_andy_set_val_GAMMAREDnega[60][2];
static char mipi_sh_andy_set_val_GAMMAGREENposi[60][2];
static char mipi_sh_andy_set_val_GAMMAGREENnega[60][2];
static char mipi_sh_andy_set_val_GAMMABLUEposi[60][2];
static char mipi_sh_andy_set_val_GAMMABLUEnega[60][2];
static char mipi_sh_andy_set_val_RegulatorPumpSetting[25][2];

#define NOT_SUPPORT_ANDY_VDDI

static struct shdisp_diag_gamma_info diag_tmp_gamma_info;
static int diag_tmp_gamma_info_set = 0;

static int drive_freq_last_type = SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A;

/* ------------------------------------------------------------------------- */
/*      packet header                                                        */
/* ------------------------------------------------------------------------- */
/*      LCD ON                                                              */
/*      Initial Setting                                                     */

#if defined(CONFIG_MACH_LYNX_DL40)
  #include "./data/shdisp_andy_data_dl40.h"
#elif defined(CONFIG_MACH_LYNX_DL45)
  #include "./data/shdisp_andy_data_dl45.h"
#elif defined(CONFIG_MACH_TBS)
  #include "./data/shdisp_andy_data_pa23.h"
#elif defined(CONFIG_MACH_DECKARD_AS87)
  #include "./data/shdisp_andy_data_as87.h"
#else
  #include "./data/shdisp_andy_data_as97.h"
#endif

const static unsigned char freq_drive_a[] = {
    (CALI_PLL1CTL_VAL & 0x000000FF) >>  0,
    (CALI_PLL1CTL_VAL & 0x0000FF00) >>  8,
    (CALI_PLL1CTL_VAL & 0x00FF0000) >> 16,
    (CALI_PLL1CTL_VAL & 0xFF000000) >> 24,
    (CALI_PLL1CTL2_VAL & 0x000000FF) >>  0,
    (CALI_PLL1CTL2_VAL & 0x0000FF00) >>  8,
    (CALI_PLL1CTL2_VAL & 0x00FF0000) >> 16,
    (CALI_PLL1CTL2_VAL & 0xFF000000) >> 24,
    (CALI_PLL1CTL3_VAL & 0x000000FF) >>  0,
    (CALI_PLL1CTL3_VAL & 0x0000FF00) >>  8,
    (CALI_PLL1CTL3_VAL & 0x00FF0000) >> 16,
    (CALI_PLL1CTL3_VAL & 0xFF000000) >> 24,
    (CALI_GPDIV_VAL & 0x00FF) >> 0,
    (CALI_GPDIV_VAL & 0xFF00) >> 8,
    (CALI_PTGHF_VAL & 0x00FF) >> 0,
    (CALI_PTGHF_VAL & 0xFF00) >> 8,
    (CALI_PTGHP_VAL & 0x00FF) >> 0,
    (CALI_PTGHP_VAL & 0xFF00) >> 8,
    (CALI_PTGHB_VAL & 0x00FF) >> 0,
    (CALI_PTGHB_VAL & 0xFF00) >> 8,
    (CALI_PTGVF_VAL & 0x00FF) >> 0,
    (CALI_PTGVF_VAL & 0xFF00) >> 8,
    (CALI_PTGVP_VAL & 0x00FF) >> 0,
    (CALI_PTGVP_VAL & 0xFF00) >> 8,
    (CALI_PTGVB_VAL & 0x00FF) >> 0,
    (CALI_PTGVB_VAL & 0xFF00) >> 8,
    RTN,
    GIP,
};

const static unsigned char freq_drive_b[] = {
    (CALI_PLL1CTL_VAL_B & 0x000000FF) >>  0,
    (CALI_PLL1CTL_VAL_B & 0x0000FF00) >>  8,
    (CALI_PLL1CTL_VAL_B & 0x00FF0000) >> 16,
    (CALI_PLL1CTL_VAL_B & 0xFF000000) >> 24,
    (CALI_PLL1CTL2_VAL_B & 0x000000FF) >>  0,
    (CALI_PLL1CTL2_VAL_B & 0x0000FF00) >>  8,
    (CALI_PLL1CTL2_VAL_B & 0x00FF0000) >> 16,
    (CALI_PLL1CTL2_VAL_B & 0xFF000000) >> 24,
    (CALI_PLL1CTL3_VAL_B & 0x000000FF) >>  0,
    (CALI_PLL1CTL3_VAL_B & 0x0000FF00) >>  8,
    (CALI_PLL1CTL3_VAL_B & 0x00FF0000) >> 16,
    (CALI_PLL1CTL3_VAL_B & 0xFF000000) >> 24,
    (CALI_GPDIV_VAL_B & 0x00FF) >> 0,
    (CALI_GPDIV_VAL_B & 0xFF00) >> 8,
    (CALI_PTGHF_VAL_B & 0x00FF) >> 0,
    (CALI_PTGHF_VAL_B & 0xFF00) >> 8,
    (CALI_PTGHP_VAL_B & 0x00FF) >> 0,
    (CALI_PTGHP_VAL_B & 0xFF00) >> 8,
    (CALI_PTGHB_VAL_B & 0x00FF) >> 0,
    (CALI_PTGHB_VAL_B & 0xFF00) >> 8,
    (CALI_PTGVF_VAL_B & 0x00FF) >> 0,
    (CALI_PTGVF_VAL_B & 0xFF00) >> 8,
    (CALI_PTGVP_VAL_B & 0x00FF) >> 0,
    (CALI_PTGVP_VAL_B & 0xFF00) >> 8,
    (CALI_PTGVB_VAL_B & 0x00FF) >> 0,
    (CALI_PTGVB_VAL_B & 0xFF00) >> 8,
    RTN_B,
    GIP_B,
};

#if defined(CONFIG_MACH_TBS)
const static unsigned char freq_drive_c[] = {
    (CALI_PLL1CTL_VAL_C & 0x000000FF) >>  0,
    (CALI_PLL1CTL_VAL_C & 0x0000FF00) >>  8,
    (CALI_PLL1CTL_VAL_C & 0x00FF0000) >> 16,
    (CALI_PLL1CTL_VAL_C & 0xFF000000) >> 24,
    (CALI_PLL1CTL2_VAL_C & 0x000000FF) >>  0,
    (CALI_PLL1CTL2_VAL_C & 0x0000FF00) >>  8,
    (CALI_PLL1CTL2_VAL_C & 0x00FF0000) >> 16,
    (CALI_PLL1CTL2_VAL_C & 0xFF000000) >> 24,
    (CALI_PLL1CTL3_VAL_C & 0x000000FF) >>  0,
    (CALI_PLL1CTL3_VAL_C & 0x0000FF00) >>  8,
    (CALI_PLL1CTL3_VAL_C & 0x00FF0000) >> 16,
    (CALI_PLL1CTL3_VAL_C & 0xFF000000) >> 24,
    (CALI_GPDIV_VAL_C & 0x00FF) >> 0,
    (CALI_GPDIV_VAL_C & 0xFF00) >> 8,
    (CALI_PTGHF_VAL_C & 0x00FF) >> 0,
    (CALI_PTGHF_VAL_C & 0xFF00) >> 8,
    (CALI_PTGHP_VAL_C & 0x00FF) >> 0,
    (CALI_PTGHP_VAL_C & 0xFF00) >> 8,
    (CALI_PTGHB_VAL_C & 0x00FF) >> 0,
    (CALI_PTGHB_VAL_C & 0xFF00) >> 8,
    (CALI_PTGVF_VAL_C & 0x00FF) >> 0,
    (CALI_PTGVF_VAL_C & 0xFF00) >> 8,
    (CALI_PTGVP_VAL_C & 0x00FF) >> 0,
    (CALI_PTGVP_VAL_C & 0xFF00) >> 8,
    (CALI_PTGVB_VAL_C & 0x00FF) >> 0,
    (CALI_PTGVB_VAL_C & 0xFF00) >> 8,
    RTN_C,
    GIP_C,
};
#endif

#ifdef SHDISP_POWER_MODE_CHK
static struct dsi_cmd_desc mipi_sh_andy_cmds_dispon_check[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, mipi_sh_andy_cmd_SwitchCommand[0]}
};
#endif /* SHDISP_POWER_MODE_CHK */

static struct shdisp_panel_operations shdisp_andy_fops = {
    shdisp_andy_API_init_io,
    shdisp_andy_API_exit_io,
    shdisp_andy_API_power_on,
    shdisp_andy_API_power_off,
    shdisp_andy_API_disp_on,
    shdisp_andy_API_disp_off,
    shdisp_andy_API_mipi_lcd_on_after_black_screen,
    shdisp_andy_API_mipi_lcd_off_black_screen_on,
    shdisp_andy_API_start_display,
    shdisp_andy_API_diag_write_reg,
    shdisp_andy_API_diag_read_reg,
    shdisp_andy_API_check_flicker_param,
    shdisp_andy_API_diag_set_flicker_param,
    shdisp_andy_API_diag_get_flicker_param,
    shdisp_andy_API_diag_get_flicker_low_param,
    shdisp_andy_API_check_recovery,
    shdisp_andy_API_diag_set_gamma_info,
    shdisp_andy_API_diag_get_gamma_info,
    shdisp_andy_API_diag_set_gamma,
    shdisp_andy_API_set_drive_freq,
    shdisp_andy_API_get_drive_freq,
    shdisp_andy_API_shutdown,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX_CLMR(x)         (shdisp_panel_API_mipi_dsi_cmds_tx(&shdisp_mipi_andy_tx_buf, x, ARRAY_SIZE(x)))
#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
#define MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(x)  (shdisp_panel_API_mipi_dsi_cmds_mltshortpkt_tx(&shdisp_mipi_andy_tx_buf, x, ARRAY_SIZE(x)))
#endif
#define MIPI_DSI_COMMAND_TX_CLMR_DUMMY(x)   (shdisp_panel_API_mipi_dsi_cmds_tx_dummy(x))
#define IS_FLICKER_ADJUSTED(param) (((param & 0xF000) == 0x9000) ? 1 : 0)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_create                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_andy_API_create(void)
{
    SHDISP_TRACE("\n");
    return &shdisp_andy_fops;
}


static int shdisp_andy_mipi_dsi_buf_alloc(struct dsi_buf *dp, int size)
{

    dp->start = kmalloc(size, GFP_KERNEL);
    if (dp->start == NULL) {
        SHDISP_ERR("%u\n", __LINE__);
        return -ENOMEM;
    }

    dp->end = dp->start + size;
    dp->size = size;

    dp->data = dp->start;
    dp->len = 0;
    return size;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_init_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_init_io(void)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    unsigned short tmp_alpha;
    unsigned short tmp_alpha_low;
    unsigned short tmp;
    unsigned short vcomdcoff;
#endif
    int ret = 0;
    struct shdisp_lcddr_phy_gamma_reg* phy_gamma;

    SHDISP_TRACE("in\n");


#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
    shdisp_andy_mipi_dsi_buf_alloc(&shdisp_mipi_andy_tx_buf, 2048);
#else
    shdisp_andy_mipi_dsi_buf_alloc(&shdisp_mipi_andy_tx_buf, DSI_BUF_SIZE);
#endif
    shdisp_andy_mipi_dsi_buf_alloc(&shdisp_mipi_andy_rx_buf, DSI_BUF_SIZE);

#ifndef SHDISP_NOT_SUPPORT_FLICKER
    tmp_alpha = shdisp_api_get_alpha();
    tmp_alpha_low = shdisp_api_get_alpha_low();
    SHDISP_DEBUG("alpha=0x%04x alpha_low=0x%04x\n", tmp_alpha, tmp_alpha_low);
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM1_L][1] = (char) (tmp_alpha & 0xFF);
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM2_L][1] = mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM1_L][1];
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1] = (char) (mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1] & 0xFC);
    if ((tmp_alpha >> 8) & 0x01){
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1] |= 0x03;
    }
    SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x\n",
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM1_L][1],
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM2_L][1],
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1]);

    if ((tmp_alpha % 2) == 0){
        vcomdcoff = tmp_alpha / 2;
    } else {
        vcomdcoff = (tmp_alpha + 1) / 2;
    }

    if (tmp_alpha_low - tmp_alpha >= 0){
        tmp = tmp_alpha_low - tmp_alpha;
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM1][1] = (tmp & 0x0F);
    } else {
        tmp = tmp_alpha - tmp_alpha_low - 1;
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM1][1] = ((tmp & 0x0F) | 0x10);
    }
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM2][1] = mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM1][1];
    if (vcomdcoff & 0x100){
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM2][1] |= 0x80;
    }
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOMOFF][1] = (unsigned char) (vcomdcoff & 0xFF);
    SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF=0x%02x\n",
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM1][1],
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM2][1],
        mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOMOFF][1]);

#endif
    phy_gamma = shdisp_api_get_lcddr_phy_gamma();
    ret = shdisp_andy_init_phy_gamma(phy_gamma);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_init_phy_gamma.\n");
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_exit_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_exit_io(void)
{
    SHDISP_TRACE("in\n");

    if (shdisp_mipi_andy_tx_buf.start != NULL) {
        SHDISP_DEBUG("memory free: shdisp_mipi_andy_tx_buf.start\n");
        kfree(shdisp_mipi_andy_tx_buf.start);
    }
    if (shdisp_mipi_andy_rx_buf.start != NULL) {
        SHDISP_DEBUG("memory free: shdisp_mipi_andy_rx_buf.start\n");
        kfree(shdisp_mipi_andy_rx_buf.start);
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_power_on(int mode)
{

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
   current_page = 0;
#endif
    SHDISP_TRACE("in mode=%d\n", mode);


    if (mode == SHDISP_PANEL_POWER_NORMAL_ON){
        shdisp_clmr_api_disp_init();
    }


#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif

    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_cmd_delay_us(3*1000);

    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_cmd_delay_us(10*1000);
    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_cmd_delay_us(1*1000);
    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_cmd_delay_us(10*1000);
    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_cmd_delay_us(1*1000);
    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_cmd_delay_us(10*1000);

    shdisp_bdic_API_LCD_vo2_off();
    shdisp_bdic_API_LCD_power_on();
    shdisp_bdic_API_LCD_m_power_on();
    shdisp_bdic_API_LCD_vo2_on();
    shdisp_SYS_cmd_delay_us(1*1000);

#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif


    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_power_off(int mode)
{
    switch (mode){
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
        SHDISP_TRACE("in RECOVERY_OFF: mode=%d\n", mode);
        break;
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_TRACE("in SHUTDOWN_OFF: mode=%d\n", mode);
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        SHDISP_TRACE("in NORMAL_OFF: mode=%d\n", mode);
        break;
    }


    if(mode != SHDISP_PANEL_POWER_RECOVERY_OFF){
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif
        shdisp_clmr_api_display_stop();
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_safe_finishanddoKick();
#endif
    }
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif

    if (mode == SHDISP_PANEL_POWER_RECOVERY_OFF){
        SHDISP_DEBUG("excute andy HW reset");
        shdisp_bdic_API_LCD_set_hw_reset();
        shdisp_SYS_cmd_delay_us(100*1000);
    }

    shdisp_bdic_API_LCD_m_power_off();
    shdisp_bdic_API_LCD_power_off();
    if (mode == SHDISP_PANEL_POWER_SHUTDOWN_OFF){
        SHDISP_DEBUG("excute andy HW reset");
        shdisp_bdic_API_LCD_set_hw_reset();
        shdisp_SYS_cmd_delay_us(5*1000);
    }

#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif


    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}



/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_disp_on                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif

    ret = shdisp_andy_mipi_cmd_lcd_on();

#ifdef SHDISP_FW_STACK_EXCUTE
    ret = shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out dokick err ret=%d\n", ret);
        return ret;
    }
#endif
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_disp_off                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_disp_off(void)
{
    SHDISP_TRACE("in\n");
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif

    shdisp_andy_mipi_cmd_lcd_off();

#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif

    SHDISP_TRACE("out\n");
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_start_display                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_start_display(void)
{
    SHDISP_TRACE("in\n");
    shdisp_andy_API_mipi_start_display();
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_check_flicker_param                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    short tmp_alpha = alpha_in;

    SHDISP_TRACE("in alpha_in=%d\n", alpha_in);
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = VCOM1_L;
        SHDISP_TRACE("out1\n");
        return SHDISP_RESULT_SUCCESS;
    }

    tmp_alpha = tmp_alpha & 0x01FF;
    if ((tmp_alpha < SHDISP_ANDY_VCOM_MIN) || (tmp_alpha > SHDISP_ANDY_VCOM_MAX)) {
        *alpha_out = VCOM1_L;
        SHDISP_TRACE("out2\n");
        return SHDISP_RESULT_SUCCESS;
    }

    *alpha_out = tmp_alpha;

    SHDISP_TRACE("out\n");
#endif
    return SHDISP_RESULT_SUCCESS;
}

static int shdisp_andy_API_start_video(void)
{
    SHDISP_TRACE("in\n");
    {
        char pageChg[4]    = { DTYPE_DCS_WRITE1, 0xff, 00, 00 };
        char displayOn[4] = { DTYPE_DCS_WRITE, 0x29, 00, 00 };
        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_LP_SWRITE, sizeof(pageChg), pageChg );
        shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_LP_SWRITE, sizeof(displayOn), displayOn );
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }
    {
        char clmrPstVideoOn[6] = { 0x00, 0x12, 0x03, 00, 00, 00 };
        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_HOST_1WORD_WRITE, sizeof(clmrPstVideoOn), clmrPstVideoOn);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }

    shdisp_panel_API_request_RateCtrl(1, SHDISP_PANEL_RATE_60_0, SHDISP_PANEL_RATE_1);

    shdisp_bdic_API_IRQ_det_irq_ctrl(1);
    shdisp_clmr_api_fw_detlcdandbdic_ctrl(1);

    shdisp_panel_API_detect_bad_timing_transfer(1);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}
static int shdisp_andy_API_stop_video(void)
{
    SHDISP_TRACE("in\n");

    shdisp_panel_API_detect_bad_timing_transfer(0);

    shdisp_clmr_api_fw_detlcdandbdic_ctrl(0);
    shdisp_bdic_API_IRQ_det_irq_ctrl(0);

    shdisp_panel_API_request_RateCtrl(0,0,0);

    {
        char pageChg[4]    = { DTYPE_DCS_WRITE1, 0xff, 00, 00 };
        char displayOff[4] = { DTYPE_DCS_WRITE, 0x28, 00, 00 };

        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_LP_SWRITE, sizeof(pageChg), pageChg );
        shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_LP_SWRITE, sizeof(displayOff), displayOff );
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
        shdisp_SYS_cmd_delay_us(WAIT_1FRAME_US*1);
    }
    {
        char clmrPstVideoOff[6] = { 0x00, 0x12, 00, 00, 00, 00 };
        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_HOST_1WORD_WRITE, sizeof(clmrPstVideoOff), clmrPstVideoOff);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
        shdisp_SYS_cmd_delay_us(WAIT_1FRAME_US*1);
    }
    {
        char pageRestore[4]    = { DTYPE_DCS_WRITE1, 0xff, 00, 00 };
        pageRestore[2] = (char)current_page;
        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_LP_SWRITE, sizeof(pageRestore), pageRestore );
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_write_reg(int cog, unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    if(addr == 0xff)
    {
        current_page = *write_data;
    }
    SHDISP_DEBUG("current_page = %d\n", current_page);
    shdisp_andy_API_stop_video();
#endif
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
#endif
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf, addr, write_data, size);
    ret = shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
    shdisp_SYS_delay_us(50*1000);

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_andy_API_start_video();
#endif
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out dokick err ret=%d\n", ret);
        return ret;
    }
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_read_reg(int cog, unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_andy_API_stop_video();
#endif

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
#endif

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, addr, read_data, size);
    shdisp_SYS_delay_us(50*1000);
#endif
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, addr, read_data, size);

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_andy_API_start_video();
#endif
    if(ret) {
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int alpha = flicker_param.master_alpha;
    int i;
    int ret = 0;
    char pageChange[4] = { DTYPE_DCS_WRITE1, 0xff, 01, 00 };
    unsigned char andy_rdata_tmp[8];
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    unsigned char back_page = 0;
#endif
    unsigned short vcomdcoff;

    SHDISP_TRACE("in\n");
    if ((alpha < SHDISP_ANDY_VCOM_MIN) || (alpha > SHDISP_ANDY_VCOM_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", alpha);
        return SHDISP_RESULT_FAILURE;
    }

    for (i=1; i<=7; i++) {
        andy_rdata[i] = 0;
        andy_rdata_tmp[i] = 0;
        andy_wdata[i] = 0;
    }

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    back_page = current_page;
    shdisp_andy_API_stop_video();
#endif
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_init(0);
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
    shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_HS_SWRITE, sizeof(pageChange), pageChange );

    andy_rdata_tmp[0] = (unsigned char) (alpha & 0xFF);
    andy_rdata_tmp[1] = (unsigned char) (alpha & 0xFF);
    andy_rdata_tmp[2] = 0x60;
    if ((alpha >> 8) & 0x01){
        andy_rdata_tmp[2] = (unsigned char) (andy_rdata_tmp[2] | 0x03);
    }
    if ((alpha % 2) == 0){
        vcomdcoff = alpha / 2;
    } else {
        vcomdcoff = (alpha + 1) / 2;
    }
    andy_rdata_tmp[3] = 0x00;
    andy_rdata_tmp[4] = 0x00;
    if ((vcomdcoff >> 8) & 0x01){
        andy_rdata_tmp[4] = (unsigned char) (andy_rdata_tmp[4] | 0x80);
    }
    andy_rdata_tmp[5] = (unsigned char) (vcomdcoff & 0xFF);

#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
    for (i=0; i<ANDY_VCOM_REG_NUM; i++) {
        andy_wdata[i] = andy_rdata_tmp[i];
    }
    ret = shdisp_panel_API_mipi_diag_mltshortpkt_write_reg(&shdisp_mipi_andy_tx_buf, Andy_VCOM_Reg, andy_wdata, ANDY_VCOM_REG_NUM);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg!!\n" );
    }
#else
    for (i=0; i<ANDY_VCOM_REG_NUM; i++) {
        andy_wdata[0] = andy_rdata_tmp[i];
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf, Andy_VCOM_Reg[i], andy_wdata, 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg addr=0x%02x data=0x%02x\n", Andy_VCOM_Reg[i], andy_wdata[0]);
        }
    }
#endif

    ret = shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    current_page = back_page;
    shdisp_andy_API_start_video();
#endif
    SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x\n",
        andy_rdata_tmp[0],andy_rdata_tmp[1],andy_rdata_tmp[2]);
    SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF=0x%02x\n",
        andy_rdata_tmp[3],andy_rdata_tmp[4],andy_rdata_tmp[5]);
    SHDISP_DEBUG("alpha=0x%04x\n", alpha);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out dokick err ret=%d\n", ret);
        return ret;
    }
    SHDISP_TRACE("out\n");
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    char pageChange[4] = { DTYPE_DCS_WRITE1, 0xff, 01, 00 };
    unsigned char andy_rdata_tmp[8];
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    unsigned char back_page = 0;
#endif

    SHDISP_TRACE("in\n");
    if (flicker_param == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }

    for (i=1; i<=7; i++) {
        andy_rdata[i] = 0;
        andy_rdata_tmp[i] = 0;
    }

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    back_page = current_page;
    shdisp_andy_API_stop_video();
#endif
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_init(0);
    shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_HS_SWRITE, sizeof(pageChange), pageChange );
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 0, 0);

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, Andy_VCOM_Reg[0], andy_rdata, 1);
    shdisp_SYS_delay_us(50*1000);
#endif
    for (i=0; i<3; i++) {
        if (i != 1){
            ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, Andy_VCOM_Reg[i], andy_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x\n", Andy_VCOM_Reg[i], andy_rdata[0]);
            }
            andy_rdata_tmp[i] = andy_rdata[0];
        }
    }

    flicker_param->master_alpha = ((andy_rdata_tmp[2] & 0x01) << 8) | andy_rdata_tmp[0];

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    current_page = back_page;
    shdisp_andy_API_start_video();
#endif

    SHDISP_TRACE("out alpha=0x%04X\n", flicker_param->master_alpha);
#endif
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    char pageChange[4] = { DTYPE_DCS_WRITE1, 0xff, 01, 00 };
    unsigned char andy_rdata_tmp[8];
#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    unsigned char back_page = 0;
#endif
    unsigned short tmp_alpha;

    SHDISP_TRACE("in\n");
    if (flicker_param == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }

    for (i=1; i<=7; i++) {
        andy_rdata[i] = 0;
        andy_rdata_tmp[i] = 0;
    }

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    back_page = current_page;
    shdisp_andy_API_stop_video();
#endif
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_init(0);
    shdisp_FWCMD_buf_add( SHDISP_CLMR_FWCMD_DSI_HS_SWRITE, sizeof(pageChange), pageChange );
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 0, 0);

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, Andy_VCOM_Reg[0], andy_rdata, 1);
    shdisp_SYS_delay_us(50*1000);
#endif
    for (i=0; i<4; i++) {
        if (i != 1){
            ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf, &shdisp_mipi_andy_rx_buf, Andy_VCOM_Reg[i], andy_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x\n", Andy_VCOM_Reg[i], andy_rdata[0]);
            }
            andy_rdata_tmp[i] = andy_rdata[0];
        }
    }

    tmp_alpha = ((andy_rdata_tmp[2] & 0x01) << 8) | andy_rdata_tmp[0];
    if (andy_rdata_tmp[3] & 0x10){
        flicker_param->master_alpha = tmp_alpha - (andy_rdata_tmp[3] & 0x0f) - 1;
    } else {
        flicker_param->master_alpha = tmp_alpha + (andy_rdata_tmp[3] & 0x0f);
    }

#ifdef SHDISP_ANDY_PROVISIONAL_REG_RW
    current_page = back_page;
    shdisp_andy_API_start_video();
#endif

    SHDISP_TRACE("out low_alpha=0x%04X\n", flicker_param->master_alpha);
#endif
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_check_recovery                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_check_recovery(void)
{
#ifndef SHDISP_NOT_SUPPORT_DET
    int ret;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_POWER_MODE_CHK
    ret = shdisp_panel_andy_power_mode_chk(0x0A);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_andy_power_mode_chk.\n");
        return SHDISP_RESULT_FAILURE;
    }
#endif
    ret = shdisp_bdic_API_RECOVERY_check_restoration();

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_set_gamma_info                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info, int set_applied_voltage)
{
#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
    int i, j = 0;
    int ret = 0;
    unsigned char andy_gamma_wdata[372];
    unsigned char andy_gamma_addr[372] = {
        mipi_sh_andy_cmd_SwitchCommand[1][0],
        mipi_sh_andy_cmd_GAMMAREDposi[0][0],
        mipi_sh_andy_cmd_GAMMAREDposi[1][0],
        mipi_sh_andy_cmd_GAMMAREDposi[2][0],
        mipi_sh_andy_cmd_GAMMAREDposi[3][0],
        mipi_sh_andy_cmd_GAMMAREDposi[4][0],
        mipi_sh_andy_cmd_GAMMAREDposi[5][0],
        mipi_sh_andy_cmd_GAMMAREDposi[6][0],
        mipi_sh_andy_cmd_GAMMAREDposi[7][0],
        mipi_sh_andy_cmd_GAMMAREDposi[8][0],
        mipi_sh_andy_cmd_GAMMAREDposi[9][0],
        mipi_sh_andy_cmd_GAMMAREDposi[10][0],
        mipi_sh_andy_cmd_GAMMAREDposi[11][0],
        mipi_sh_andy_cmd_GAMMAREDposi[12][0],
        mipi_sh_andy_cmd_GAMMAREDposi[13][0],
        mipi_sh_andy_cmd_GAMMAREDposi[14][0],
        mipi_sh_andy_cmd_GAMMAREDposi[15][0],
        mipi_sh_andy_cmd_GAMMAREDposi[16][0],
        mipi_sh_andy_cmd_GAMMAREDposi[17][0],
        mipi_sh_andy_cmd_GAMMAREDposi[18][0],
        mipi_sh_andy_cmd_GAMMAREDposi[19][0],
        mipi_sh_andy_cmd_GAMMAREDposi[20][0],
        mipi_sh_andy_cmd_GAMMAREDposi[21][0],
        mipi_sh_andy_cmd_GAMMAREDposi[22][0],
        mipi_sh_andy_cmd_GAMMAREDposi[23][0],
        mipi_sh_andy_cmd_GAMMAREDposi[24][0],
        mipi_sh_andy_cmd_GAMMAREDposi[25][0],
        mipi_sh_andy_cmd_GAMMAREDposi[26][0],
        mipi_sh_andy_cmd_GAMMAREDposi[27][0],
        mipi_sh_andy_cmd_GAMMAREDposi[28][0],
        mipi_sh_andy_cmd_GAMMAREDposi[29][0],
        mipi_sh_andy_cmd_GAMMAREDposi[30][0],
        mipi_sh_andy_cmd_GAMMAREDposi[31][0],
        mipi_sh_andy_cmd_GAMMAREDposi[32][0],
        mipi_sh_andy_cmd_GAMMAREDposi[33][0],
        mipi_sh_andy_cmd_GAMMAREDposi[34][0],
        mipi_sh_andy_cmd_GAMMAREDposi[35][0],
        mipi_sh_andy_cmd_GAMMAREDposi[36][0],
        mipi_sh_andy_cmd_GAMMAREDposi[37][0],
        mipi_sh_andy_cmd_GAMMAREDposi[38][0],
        mipi_sh_andy_cmd_GAMMAREDposi[39][0],
        mipi_sh_andy_cmd_GAMMAREDposi[40][0],
        mipi_sh_andy_cmd_GAMMAREDposi[41][0],
        mipi_sh_andy_cmd_GAMMAREDposi[42][0],
        mipi_sh_andy_cmd_GAMMAREDposi[43][0],
        mipi_sh_andy_cmd_GAMMAREDposi[44][0],
        mipi_sh_andy_cmd_GAMMAREDposi[45][0],
        mipi_sh_andy_cmd_GAMMAREDposi[46][0],
        mipi_sh_andy_cmd_GAMMAREDposi[47][0],
        mipi_sh_andy_cmd_GAMMAREDposi[48][0],
        mipi_sh_andy_cmd_GAMMAREDposi[49][0],
        mipi_sh_andy_cmd_GAMMAREDposi[50][0],
        mipi_sh_andy_cmd_GAMMAREDposi[51][0],
        mipi_sh_andy_cmd_GAMMAREDposi[52][0],
        mipi_sh_andy_cmd_GAMMAREDposi[53][0],
        mipi_sh_andy_cmd_GAMMAREDposi[54][0],
        mipi_sh_andy_cmd_GAMMAREDposi[55][0],
        mipi_sh_andy_cmd_GAMMAREDposi[56][0],
        mipi_sh_andy_cmd_GAMMAREDposi[57][0],
        mipi_sh_andy_cmd_GAMMAREDposi[58][0],
        mipi_sh_andy_cmd_GAMMAREDposi[59][0],
        mipi_sh_andy_cmd_GAMMAREDnega[0][0],
        mipi_sh_andy_cmd_GAMMAREDnega[1][0],
        mipi_sh_andy_cmd_GAMMAREDnega[2][0],
        mipi_sh_andy_cmd_GAMMAREDnega[3][0],
        mipi_sh_andy_cmd_GAMMAREDnega[4][0],
        mipi_sh_andy_cmd_GAMMAREDnega[5][0],
        mipi_sh_andy_cmd_GAMMAREDnega[6][0],
        mipi_sh_andy_cmd_GAMMAREDnega[7][0],
        mipi_sh_andy_cmd_GAMMAREDnega[8][0],
        mipi_sh_andy_cmd_GAMMAREDnega[9][0],
        mipi_sh_andy_cmd_GAMMAREDnega[10][0],
        mipi_sh_andy_cmd_GAMMAREDnega[11][0],
        mipi_sh_andy_cmd_GAMMAREDnega[12][0],
        mipi_sh_andy_cmd_GAMMAREDnega[13][0],
        mipi_sh_andy_cmd_GAMMAREDnega[14][0],
        mipi_sh_andy_cmd_GAMMAREDnega[15][0],
        mipi_sh_andy_cmd_GAMMAREDnega[16][0],
        mipi_sh_andy_cmd_GAMMAREDnega[17][0],
        mipi_sh_andy_cmd_GAMMAREDnega[18][0],
        mipi_sh_andy_cmd_GAMMAREDnega[19][0],
        mipi_sh_andy_cmd_GAMMAREDnega[20][0],
        mipi_sh_andy_cmd_GAMMAREDnega[21][0],
        mipi_sh_andy_cmd_GAMMAREDnega[22][0],
        mipi_sh_andy_cmd_GAMMAREDnega[23][0],
        mipi_sh_andy_cmd_GAMMAREDnega[24][0],
        mipi_sh_andy_cmd_GAMMAREDnega[25][0],
        mipi_sh_andy_cmd_GAMMAREDnega[26][0],
        mipi_sh_andy_cmd_GAMMAREDnega[27][0],
        mipi_sh_andy_cmd_GAMMAREDnega[28][0],
        mipi_sh_andy_cmd_GAMMAREDnega[29][0],
        mipi_sh_andy_cmd_GAMMAREDnega[30][0],
        mipi_sh_andy_cmd_GAMMAREDnega[31][0],
        mipi_sh_andy_cmd_GAMMAREDnega[32][0],
        mipi_sh_andy_cmd_GAMMAREDnega[33][0],
        mipi_sh_andy_cmd_GAMMAREDnega[34][0],
        mipi_sh_andy_cmd_GAMMAREDnega[35][0],
        mipi_sh_andy_cmd_GAMMAREDnega[36][0],
        mipi_sh_andy_cmd_GAMMAREDnega[37][0],
        mipi_sh_andy_cmd_GAMMAREDnega[38][0],
        mipi_sh_andy_cmd_GAMMAREDnega[39][0],
        mipi_sh_andy_cmd_GAMMAREDnega[40][0],
        mipi_sh_andy_cmd_GAMMAREDnega[41][0],
        mipi_sh_andy_cmd_GAMMAREDnega[42][0],
        mipi_sh_andy_cmd_GAMMAREDnega[43][0],
        mipi_sh_andy_cmd_GAMMAREDnega[44][0],
        mipi_sh_andy_cmd_GAMMAREDnega[45][0],
        mipi_sh_andy_cmd_GAMMAREDnega[46][0],
        mipi_sh_andy_cmd_GAMMAREDnega[47][0],
        mipi_sh_andy_cmd_GAMMAREDnega[48][0],
        mipi_sh_andy_cmd_GAMMAREDnega[49][0],
        mipi_sh_andy_cmd_GAMMAREDnega[50][0],
        mipi_sh_andy_cmd_GAMMAREDnega[51][0],
        mipi_sh_andy_cmd_GAMMAREDnega[52][0],
        mipi_sh_andy_cmd_GAMMAREDnega[53][0],
        mipi_sh_andy_cmd_GAMMAREDnega[54][0],
        mipi_sh_andy_cmd_GAMMAREDnega[55][0],
        mipi_sh_andy_cmd_GAMMAREDnega[56][0],
        mipi_sh_andy_cmd_GAMMAREDnega[57][0],
        mipi_sh_andy_cmd_GAMMAREDnega[58][0],
        mipi_sh_andy_cmd_GAMMAREDnega[59][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[0][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[1][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[2][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[3][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[4][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[5][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[6][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[7][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[8][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[9][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[10][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[11][0],
        mipi_sh_andy_cmd_SwitchCommand[2][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[12][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[13][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[14][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[15][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[16][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[17][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[18][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[19][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[20][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[21][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[22][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[23][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[24][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[25][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[26][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[27][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[28][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[29][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[30][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[31][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[32][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[33][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[34][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[35][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[36][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[37][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[38][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[39][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[40][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[41][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[42][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[43][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[44][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[45][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[46][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[47][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[48][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[49][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[50][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[51][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[52][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[53][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[54][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[55][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[56][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[57][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[58][0],
        mipi_sh_andy_cmd_GAMMAGREENposi[59][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[0][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[1][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[2][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[3][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[4][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[5][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[6][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[7][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[8][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[9][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[10][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[11][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[12][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[13][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[14][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[15][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[16][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[17][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[18][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[19][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[20][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[21][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[22][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[23][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[24][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[25][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[26][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[27][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[28][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[29][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[30][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[31][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[32][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[33][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[34][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[35][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[36][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[37][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[38][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[39][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[40][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[41][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[42][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[43][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[44][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[45][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[46][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[47][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[48][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[49][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[50][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[51][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[52][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[53][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[54][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[55][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[56][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[57][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[58][0],
        mipi_sh_andy_cmd_GAMMAGREENnega[59][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[0][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[1][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[2][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[3][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[4][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[5][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[6][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[7][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[8][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[9][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[10][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[11][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[12][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[13][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[14][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[15][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[16][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[17][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[18][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[19][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[20][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[21][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[22][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[23][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[24][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[25][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[26][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[27][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[28][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[29][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[30][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[31][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[32][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[33][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[34][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[35][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[36][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[37][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[38][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[39][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[40][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[41][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[42][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[43][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[44][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[45][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[46][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[47][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[48][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[49][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[50][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[51][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[52][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[53][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[54][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[55][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[56][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[57][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[58][0],
        mipi_sh_andy_cmd_GAMMABLUEposi[59][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[0][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[1][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[2][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[3][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[4][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[5][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[6][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[7][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[8][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[9][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[10][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[11][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[12][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[13][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[14][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[15][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[16][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[17][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[18][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[19][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[20][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[21][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[22][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[23][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[24][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[25][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[26][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[27][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[28][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[29][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[30][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[31][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[32][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[33][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[34][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[35][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[36][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[37][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[38][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[39][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[40][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[41][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[42][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[43][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[44][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[45][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[46][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[47][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[48][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[49][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[50][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[51][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[52][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[53][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[54][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[55][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[56][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[57][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[58][0],
        mipi_sh_andy_cmd_GAMMABLUEnega[59][0],
        mipi_sh_andy_cmd_SwitchCommand[1][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][0],
    };

    SHDISP_TRACE("in\n");

    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_set_nokick(1);
    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_OTHER);

    for (i = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        if (i == 0) {
            andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
        }
        andy_gamma_wdata[j++] = ((gamma_info->gammaR[i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (gamma_info->gammaR[i]       & 0x00FF);
    }

    for (i = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        if (i == 6) {
            andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        }
        andy_gamma_wdata[j++] = ((gamma_info->gammaG[i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (gamma_info->gammaG[i]       & 0x00FF);
    }

    for (i = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        andy_gamma_wdata[j++] = ((gamma_info->gammaB[i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (gamma_info->gammaB[i]       & 0x00FF);
    }

    if (!set_applied_voltage) {
        ret = shdisp_panel_API_mipi_diag_mltshortpkt_write_reg(&shdisp_mipi_andy_tx_buf, andy_gamma_addr, andy_gamma_wdata, j);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_mltshortpkt_write_reg!!\n" );
            goto shdisp_end;
        }
        ret = shdisp_FWCMD_buf_finish();
        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = shdisp_FWCMD_doKick(1, 0, NULL);
        }
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> doKick shdisp_panel_API_mipi_diag_mltshortpkt_write_reg.\n");
            goto shdisp_end;
        }
        goto shdisp_end;
    }

    andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
    andy_gamma_wdata[j++] = gamma_info->vgh;
    andy_gamma_wdata[j++] = gamma_info->vgl;
    andy_gamma_wdata[j++] = gamma_info->gvddp;
    andy_gamma_wdata[j++] = gamma_info->gvddn;
    andy_gamma_wdata[j++] = gamma_info->gvddp2;
    andy_gamma_wdata[j++] = gamma_info->vgho;
    andy_gamma_wdata[j++] = gamma_info->vglo;
    andy_gamma_wdata[j++] = gamma_info->avddr;
    andy_gamma_wdata[j++] = gamma_info->aveer;

    ret = shdisp_panel_API_mipi_diag_mltshortpkt_write_reg(&shdisp_mipi_andy_tx_buf, andy_gamma_addr, andy_gamma_wdata, j);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_mltshortpkt_write_reg!!\n" );
        goto shdisp_end;
    }
    ret = shdisp_FWCMD_buf_finish();
    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_FWCMD_doKick(1, 0, NULL);
    }
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> doKick shdisp_panel_API_mipi_diag_mltshortpkt_write_reg.\n");
        goto shdisp_end;
    }
    memcpy(&diag_tmp_gamma_info, gamma_info, sizeof(diag_tmp_gamma_info));
    diag_tmp_gamma_info_set = 1;

shdisp_end:

    shdisp_FWCMD_buf_set_nokick(0);
    shdisp_FWCMD_buf_init(0);

    shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_SwitchCommand[0][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[0][1],
                                               1);

    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
#else
    int i, j;
    int ret = 0;
    unsigned char andy_wdata[1];

    SHDISP_TRACE("in\n");

    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_set_nokick(1);
    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_OTHER);

    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        if (i == 0) {
            ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                       mipi_sh_andy_cmd_SwitchCommand[1][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                                       1);
            if(ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
                goto shdisp_end;
            }
        }
        andy_wdata[0] = ((gamma_info->gammaR[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAREDposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaR[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAREDposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        andy_wdata[0] = ((gamma_info->gammaR[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAREDnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaR[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAREDnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        if (i == 6) {
            ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                       mipi_sh_andy_cmd_SwitchCommand[2][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[2][1],
                                                       1);
            if(ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
                goto shdisp_end;
            }
        }
        andy_wdata[0] = ((gamma_info->gammaG[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAGREENposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaG[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAGREENposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        andy_wdata[0] = ((gamma_info->gammaG[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAGREENnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaG[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMAGREENnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        andy_wdata[0] = ((gamma_info->gammaB[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMABLUEposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaB[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMABLUEposi[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        andy_wdata[0] = ((gamma_info->gammaB[i] >> 8) & 0x0003);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMABLUEnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        andy_wdata[0] = (gamma_info->gammaB[i] & 0x00FF);
        ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                   mipi_sh_andy_cmd_GAMMABLUEnega[j++][0],
                                                   andy_wdata,
                                                   1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
    }

    if (!set_applied_voltage) {
        ret = shdisp_FWCMD_buf_finish();
        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = shdisp_FWCMD_doKick(1, 0, NULL);
        }
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
            goto shdisp_end;
        }
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_SwitchCommand[1][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][0],
                                               &gamma_info->vgh,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][0],
                                               &gamma_info->vgl,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][0],
                                               &gamma_info->gvddp,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][0],
                                               &gamma_info->gvddn,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][0],
                                               &gamma_info->gvddp2,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][0],
                                               &gamma_info->vgho,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][0],
                                               &gamma_info->vglo,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][0],
                                               &gamma_info->avddr,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][0],
                                               &gamma_info->aveer,
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    ret = shdisp_FWCMD_buf_finish();
    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_FWCMD_doKick(1, 0, NULL);
    }
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    memcpy(&diag_tmp_gamma_info, gamma_info, sizeof(diag_tmp_gamma_info));
    diag_tmp_gamma_info_set = 1;

shdisp_end:

    shdisp_FWCMD_buf_set_nokick(0);
    shdisp_FWCMD_buf_init(0);

    shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_SwitchCommand[0][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[0][1],
                                               1);

    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
#endif
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_get_gamma_info                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info, int set_applied_voltage)
{
    int i, j;
    int ret = 0;
    unsigned char andy_rdata[1];
    unsigned short andy_temp_data[SHDISP_ANDY_GAMMA_SETTING_SIZE];

    SHDISP_TRACE("in\n");
    if (gamma_info == NULL){
        SHDISP_ERR("<NULL_POINTER> gamma_info.\n");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        if (i == 0) {
            ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                       mipi_sh_andy_cmd_SwitchCommand[1][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                                       1);
            if(ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
                goto shdisp_end;
            }
            memset(andy_rdata, 0, sizeof(andy_rdata));
            ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                      &shdisp_mipi_andy_rx_buf,
                                                      mipi_sh_andy_cmd_GAMMAREDposi[0][0],
                                                      andy_rdata,
                                                      1);
        }
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAREDposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAREDposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAREDnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAREDnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaR, andy_temp_data, sizeof(andy_temp_data));

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        if (i == 6) {
            ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                                       mipi_sh_andy_cmd_SwitchCommand[2][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[2][1],
                                                       1);
            if(ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
                goto shdisp_end;
            }
        }
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAGREENposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAGREENposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAGREENnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMAGREENnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaG, andy_temp_data, sizeof(andy_temp_data));

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE / 2; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMABLUEposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMABLUEposi[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GAMMA_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMABLUEnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                                  &shdisp_mipi_andy_rx_buf,
                                                  mipi_sh_andy_cmd_GAMMABLUEnega[j++][0],
                                                  andy_rdata,
                                                  1);
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.\n");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaB, andy_temp_data, sizeof(andy_temp_data));

    if (!set_applied_voltage) {
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_SwitchCommand[1][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                               1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        goto shdisp_end;
    }

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->vgh = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->vgl = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->gvddp = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->gvddn = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->gvddp2 = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->vgho = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->vglo = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->avddr = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(&shdisp_mipi_andy_tx_buf,
                                              &shdisp_mipi_andy_rx_buf,
                                               mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][0],
                                              andy_rdata,
                                              1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        goto shdisp_end;
    }
    gamma_info->aveer = andy_rdata[0];

shdisp_end:

    shdisp_panel_API_mipi_diag_write_reg(&shdisp_mipi_andy_tx_buf,
                                               mipi_sh_andy_cmd_SwitchCommand[0][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[0][1],
                                               1);

    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_gamma_info                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;

    SHDISP_TRACE("in\n");

    ret = shdisp_andy_diag_set_gamma_info(gamma_info, 1);
    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_gamma_info                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;

    SHDISP_TRACE("in\n");

    shdisp_andy_API_stop_video();
    ret = shdisp_andy_diag_get_gamma_info(gamma_info, 1);
    shdisp_andy_API_start_video();
    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_gamma                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_gamma(struct shdisp_diag_gamma *gamma)
{
    int ret = 0;
    int i = 0, j = 0, k = 0;
    int group_idx, level_idx, addr_idx;
    unsigned char andy_gamma_wdata[26];
    unsigned char andy_gamma_addr[26];

    SHDISP_TRACE("in\n");
    if ((gamma->level < SHDISP_ANDY_GAMMA_LEVEL_MIN) || (gamma->level > SHDISP_ANDY_GAMMA_LEVEL_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> gamma->level(%d).\n", gamma->level);
        return SHDISP_RESULT_FAILURE;
    }


    if (!diag_tmp_gamma_info_set) {

        shdisp_andy_API_stop_video();
        ret = shdisp_andy_diag_get_gamma_info(&diag_tmp_gamma_info, 0);
        shdisp_andy_API_start_video();
        if(ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_diag_get_gamma_info.\n");
            goto shdisp_end;
        }
        diag_tmp_gamma_info_set = 1;
    }

    diag_tmp_gamma_info.gammaR[gamma->level - 1]                                       = gamma->gammaR_p;
    diag_tmp_gamma_info.gammaR[SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaR_n;
    diag_tmp_gamma_info.gammaG[gamma->level - 1]                                       = gamma->gammaG_p;
    diag_tmp_gamma_info.gammaG[SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaG_n;
    diag_tmp_gamma_info.gammaB[gamma->level - 1]                                       = gamma->gammaB_p;
    diag_tmp_gamma_info.gammaB[SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaB_n;

    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);

    shdisp_FWCMD_buf_set_nokick(1);
    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_OTHER);

    group_idx = (gamma->level - 1) / 2;
    level_idx = group_idx * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL;
    addr_idx = group_idx * SHDISP_ANDY_GAMMA_GROUP_BELONG_ADDR;

    andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
    andy_gamma_addr[k++] =  mipi_sh_andy_cmd_SwitchCommand[1][0];

    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaR[level_idx + i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaR[level_idx + i]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAREDposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAREDposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaR[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaR[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAREDnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAREDnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    if (gamma->level > 6) {
        andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        andy_gamma_addr[k++] =  mipi_sh_andy_cmd_SwitchCommand[2][0];
    }
    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaG[level_idx + i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaG[level_idx + i]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAGREENposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAGREENposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }
    
    if (gamma->level <= 6) {
        andy_gamma_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        andy_gamma_addr[k++] =  mipi_sh_andy_cmd_SwitchCommand[2][0];
    }
    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaG[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaG[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAGREENnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMAGREENnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaB[level_idx + i] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaB[level_idx + i]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMABLUEposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMABLUEposi[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }
    
    for (i=0; i<SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL; i++) {
        andy_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaB[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gamma_wdata[j++] =  (diag_tmp_gamma_info.gammaB[level_idx + i + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET]       & 0x00FF);
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMABLUEnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL][0];
        andy_gamma_addr[k++]  =  mipi_sh_andy_cmd_GAMMABLUEnega[addr_idx + i * SHDISP_ANDY_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }
    
    ret = shdisp_panel_API_mipi_diag_mltshortpkt_write_reg(&shdisp_mipi_andy_tx_buf, andy_gamma_addr, andy_gamma_wdata, j);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_mltshortpkt_write_reg!!\n" );
        goto shdisp_end;
    }
    ret = shdisp_FWCMD_buf_finish();
    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_FWCMD_doKick(1, 0, NULL);
    }
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> doKick shdisp_panel_API_mipi_diag_mltshortpkt_write_reg.\n");
        goto shdisp_end;
    }

shdisp_end:


    if(ret) {
        return ret;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_sqe_set_drive_freq                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_sqe_set_drive_freq(int type)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in type=%d.\n", type);

    switch(type) {
    case SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_HOST_DSI_TX_FREQ_SET, sizeof(freq_drive_a), (unsigned char*)freq_drive_a);
        drive_freq_last_type = type;
        break;
    case SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_B:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_HOST_DSI_TX_FREQ_SET, sizeof(freq_drive_b), (unsigned char*)freq_drive_b);
        drive_freq_last_type = type;
        break;
#if defined(CONFIG_MACH_TBS)
    case SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_C:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_HOST_DSI_TX_FREQ_SET, sizeof(freq_drive_c), (unsigned char*)freq_drive_c);
        drive_freq_last_type = type;
        break;
#endif
    default:
        SHDISP_ERR("type error.\n");
    }

    SHDISP_TRACE("out\n");
    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_set_drive_freq                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_set_drive_freq(int type)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in type=%d.\n", type);

    switch(type) {
    case SHDISP_MAIN_DISP_DRIVE_FREQ_DEFAULT:
        type = SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A;
        break;
    default:
        break;
    }

    if (drive_freq_last_type == type) {
        SHDISP_DEBUG("<INVALID_VALUE> Same Parameter.\n");
        return ret;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        drive_freq_last_type = type;
        SHDISP_DEBUG("CLMR IS NOT ACTIVE.\n");
        return ret;
    }

    if (shdisp_api_get_main_disp_status() != SHDISP_MAIN_DISP_ON) {
        drive_freq_last_type = type;
        SHDISP_DEBUG("BEFORE DISPLAY_ON.\n");
        return ret;
    }

    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);

    shdisp_andy_sqe_set_drive_freq(type);

    ret = shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_get_drive_freq                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_get_drive_freq(void)
{
    return drive_freq_last_type;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_shutdown                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_shutdown(void)
{
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif
        shdisp_bdic_API_LCD_set_hw_reset();
        shdisp_SYS_cmd_delay_us(5*1000);
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_safe_finishanddoKick();
        shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif
    
    return SHDISP_RESULT_SUCCESS;
}

static int shdisp_panel_andy_reg_read(unsigned char addr, unsigned char* out_data)
{
    struct dsi_buf *tp = &shdisp_mipi_andy_tx_buf;
    struct dsi_buf *rp = &shdisp_mipi_andy_rx_buf;
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[2+1];

    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;


    cmd[0].dtype    = DTYPE_GEN_READ1;
    cmd[0].last     = 0x01;
    cmd[0].vc       = 0x00;
    cmd[0].ack      = 0x01;
    cmd[0].wait     = 0x00;
    cmd[0].dlen     = 1;
    cmd[0].payload  = cmd_buf;

    memset(rp->data, 0, sizeof(rp->data));
    rp->data[5] = 0xff;
    if (shdisp_panel_API_mipi_dsi_cmds_rx(tp, rp, cmd, 1) != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("mipi_dsi_cmds_rx error\n");
        return SHDISP_RESULT_FAILURE;
    }
    *out_data = rp->data[5];
    return SHDISP_RESULT_SUCCESS;
}

#if 0
/* ------------------------------------------------------------------------- */
/* shdisp_andy_regulator_init                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_regulator_init(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (shdisp_api_get_hw_handset() == 1 && shdisp_api_get_hw_revision() >= SHDISP_HW_REV_PP2) {
        SHDISP_TRACE("in\n");
        regu_lcd_vddi = regulator_get(NULL, "lcd_vddi");
        if (IS_ERR(regu_lcd_vddi)) {
            SHDISP_ERR("lcd_vddi get failed.\n");
            regu_lcd_vddi = NULL;
            return SHDISP_RESULT_FAILURE;
        }
        if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
            ret = regulator_enable(regu_lcd_vddi);
            if (ret < 0) {
                SHDISP_ERR("lcd_vddi enable failed, ret=%d\n", ret);
                return SHDISP_RESULT_FAILURE;
            }
        }
        SHDISP_TRACE("out\n");
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_regulator_exit                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_regulator_exit(void)
{
    if (shdisp_api_get_hw_handset() == 1 && shdisp_api_get_hw_revision() >= SHDISP_HW_REV_PP2) {
        SHDISP_TRACE("in\n");
        shdisp_andy_regulator_off();
        regulator_put(regu_lcd_vddi);
        regu_lcd_vddi = NULL;
        SHDISP_TRACE("out\n");
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_regulator_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_regulator_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    /* IOVCC ON */
    if (shdisp_api_get_hw_handset() == 1 && shdisp_api_get_hw_revision() >= SHDISP_HW_REV_PP2) {
        SHDISP_TRACE("in\n");
        ret = regulator_enable(regu_lcd_vddi);
        if (ret < 0) {
            SHDISP_ERR("lcd_vddi enable failed, ret=%d\n", ret);
            return SHDISP_RESULT_FAILURE;
        }
        shdisp_SYS_delay_us(20000);
        SHDISP_TRACE("out\n");
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_regulator_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_regulator_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    /* IOVCC OFF */
    if (shdisp_api_get_hw_handset() == 1 && shdisp_api_get_hw_revision() >= SHDISP_HW_REV_PP2) {
        SHDISP_TRACE("in\n");
        ret = regulator_disable(regu_lcd_vddi);
        if (ret < 0) {
            SHDISP_ERR("lcd_vddi disable failed, ret=%d\n", ret);
            return SHDISP_RESULT_FAILURE;
        }
        shdisp_SYS_delay_us(20000);
        SHDISP_TRACE("out\n");
    }
    return SHDISP_RESULT_SUCCESS;
}
#endif

static int shdisp_andy_mipi_cmd_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct timespec ts,ts2;
    unsigned long long wtime = 0;

    SHDISP_TRACE("in\n");

    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);

#ifdef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
    shdisp_clmr_api_hsclk_on();
#endif
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_display_on1_1);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out1 ret=%d\n", ret);
        return ret;
    }
#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#endif
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_display_on1_2);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out2 ret=%d\n", ret);
        return ret;
    }

    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);

    getnstimeofday(&ts);
    shdisp_clmr_api_custom_blk_init();

    getnstimeofday(&ts2);
    wtime = (ts2.tv_sec - ts.tv_sec)*1000000;
    wtime += (ts2.tv_nsec - ts.tv_nsec)/1000;
    if(wtime < 10000){
      shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
      shdisp_SYS_cmd_delay_us(10000 - wtime);
      shdisp_FWCMD_safe_finishanddoKick();
      shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
    }

    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
    shdisp_clmr_api_hsclk_on();
#endif

    ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_gamma);

#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#endif
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);

    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);


    getnstimeofday(&ts2);
    wtime = (ts2.tv_sec - ts.tv_sec)*1000000;
    wtime += (ts2.tv_nsec - ts.tv_nsec)/1000;
    if(wtime < 80000){
      shdisp_SYS_cmd_delay_us(80000 - wtime);
    }

    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);

    SHDISP_TRACE("out\n");
    return ret;
}

static int shdisp_andy_mipi_cmd_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");

    shdisp_clmr_api_panel_assist_mode_ctrl(SHDISP_CLMR_PANEL_ASSIST_MODE_ON, 0x00);

    shdisp_andy_mipi_cmd_display_on();
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif

    shdisp_clmr_api_data_transfer_starts();

#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif

    if (drive_freq_last_type != SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A) {
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif
        shdisp_andy_sqe_set_drive_freq(drive_freq_last_type);
#ifdef SHDISP_FW_STACK_EXCUTE
        shdisp_FWCMD_safe_finishanddoKick();
        shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif
    }

    SHDISP_TRACE("out\n");

    return ret;
}

static int shdisp_andy_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned short tmp_vcom1;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
#endif

    tmp_vcom1 = (unsigned short) mipi_sh_andy_cmd_RegulatorPumpSetting[12][1];
    if (mipi_sh_andy_cmd_RegulatorPumpSetting[14][1] & 0x03){
        tmp_vcom1 |= 0x100;
    }
    mipi_sh_andy_cmd_VCOM1_OFF_Setting[1] = (char) ((tmp_vcom1 / 2) & 0xFF);
    mipi_sh_andy_cmd_VCOM2_OFF_Setting[1] = (char) ((tmp_vcom1 / 2) & 0xFF);
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_display_off);

    diag_tmp_gamma_info_set = 0;

    SHDISP_TRACE("out\n");
    return ret;
}

/* Test Image Generator ON */
int shdisp_panel_andy_API_TestImageGen(int onoff)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in\n");
    if (onoff) {
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_TIG_on);
    }
    else {
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_TIG_off);
    }
    SHDISP_TRACE("out\n");
    return ret;
}

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_panel_andy_power_mode_chk(unsigned char addr)
{
        unsigned char read_data = 0x00;
#ifdef SHDISP_RESET_LOG
        struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

        MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_dispon_check);

        if (shdisp_panel_andy_reg_read(addr, &read_data) != SHDISP_RESULT_SUCCESS){
            SHDISP_ERR("mipi_dsi_cmds_rx error\n");
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PANEL;
            err_code.code = SHDISP_DBG_CODE_READ_ERROR;
            err_code.subcode = SHDISP_DBG_SUBCODE_STATUS;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            return SHDISP_RESULT_SUCCESS;
        }
        SHDISP_DEBUG("addr = 0x%02x.read_data = 0x%02x\n", addr, read_data);
        if(read_data != 0x9C)
        {
            SHDISP_ERR("POWER_MODE error.addr = 0x%02x.read_data = 0x%02x\n", addr, read_data);
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PANEL;
            err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
            err_code.subcode = SHDISP_DBG_SUBCODE_DISPON_NG;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            return SHDISP_RESULT_FAILURE;
        }
        return SHDISP_RESULT_SUCCESS;
}
#endif

static int shdisp_andy_mipi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char device_code = 0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");

#ifdef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
    shdisp_clmr_api_hsclk_on();
#endif

#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
    ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1);
#else
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_initial1);
#endif

    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out1 ret=%d\n", ret);
        return ret;
    }
    shdisp_FWCMD_safe_finishanddoKick();

#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#endif

    if (shdisp_api_get_device_code() == 0xff)
    {
        unsigned char addr = mipi_sh_andy_cmd_DeviceCodeRead[0];

        if (shdisp_panel_andy_reg_read(addr, &device_code) != SHDISP_RESULT_SUCCESS){
            SHDISP_ERR("mipi_dsi_cmds_rx error\n");
            SHDISP_ERR("mipi_dsi_cmds_rx error. default TS set.\n");
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PANEL;
            err_code.code = SHDISP_DBG_CODE_READ_ERROR;
            err_code.subcode = SHDISP_DBG_SUBCODE_DEVCODE;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
#if defined(CONFIG_MACH_DECKARD_AS97)
            device_code = 0x02;
#else
            device_code = 0x80;
#endif
        }
        SHDISP_DEBUG("device_code = 0x%02x\n", device_code);

        shdisp_api_set_device_code(device_code);
    }
    else {
        device_code = shdisp_api_get_device_code();
        SHDISP_DEBUG("device_code = 0x%02x\n", device_code);
    }


#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
    shdisp_clmr_api_hsclk_on();
#endif

#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
    {

        if (device_code == 0x00){
            ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts1_0);
            if (ret != SHDISP_RESULT_SUCCESS){
                SHDISP_DEBUG("out1-3 ret=%d\n", ret);
                return ret;
            }
        }
        else if (device_code == 0x01) {
            if (IS_FLICKER_ADJUSTED(shdisp_api_get_alpha_nvram())) {
                ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts1_1);
                if (ret != SHDISP_RESULT_SUCCESS){
                    SHDISP_DEBUG("out1-3 ret=%d\n", ret);
                    return ret;
                }
            }
            else {
                ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts1_1_flicker_unadjusted);
                if (ret != SHDISP_RESULT_SUCCESS){
                    SHDISP_DEBUG("out1-3 ret=%d\n", ret);
                    return ret;
                }
            }
        }
        else {
#if defined(CONFIG_MACH_LYNX_DL40) || defined(CONFIG_MACH_LYNX_DL45) || defined(CONFIG_MACH_TBS) || defined(CONFIG_MACH_DECKARD_AS87)
            if (device_code == 0x02) {
                mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VDD_Reg][1] = VDD_Reg_TS2;
            }
            else {
                mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VDD_Reg][1] = VDD_Reg_TS3;
            }
#endif /* defined(CONFIG_MACH_LYNX_DL40) */
            if (IS_FLICKER_ADJUSTED(shdisp_api_get_alpha_nvram())) {
                ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
                if (ret != SHDISP_RESULT_SUCCESS){
                    SHDISP_DEBUG("out1-3 ret=%d\n", ret);
                    return ret;
                }
            }
            else {
                ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts2_0_flicker_unadjusted);
                if (ret != SHDISP_RESULT_SUCCESS){
                    SHDISP_DEBUG("out1-3 ret=%d\n", ret);
                    return ret;
                }
            }
        }

        ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_terminal);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out3 ret=%d\n", ret);
            return ret;
        }

        ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_timing);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out4 ret=%d\n", ret);
            return ret;
        }

        ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_initial2);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out5 ret=%d\n", ret);
            return ret;
        }

        ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_power);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out6 ret=%d\n", ret);
            return ret;
        }

        if ((device_code == 0x00) || (device_code == 0x01)){
            ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_sync_ts1_0);
            if (ret != SHDISP_RESULT_SUCCESS){
                SHDISP_DEBUG("out7 ret=%d\n", ret);
                return ret;
            }
        }
        else {
            ret = MIPI_DSI_COMMAND_MLTPKT_TX_CLMR(mipi_sh_andy_cmds_sync_ts2_0);
            if (ret != SHDISP_RESULT_SUCCESS){
                SHDISP_DEBUG("out7 ret=%d\n", ret);
                return ret;
            }
        }
    }
#else

    if (device_code == 0x00){
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts1_0);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out1-3 ret=%d\n", ret);
            return ret;
        }
    }
    else if (device_code == 0x01) {
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts1_1);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out1-3 ret=%d\n", ret);
            return ret;
        }
    }
    else {
#if defined(CONFIG_MACH_LYNX_DL40) || defined(CONFIG_MACH_LYNX_DL45) || defined(CONFIG_MACH_TBS) || defined(CONFIG_MACH_DECKARD_AS87)
        if (device_code == 0x02) {
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VDD_Reg][1] = VDD_Reg_TS2;
        }
        else {
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VDD_Reg][1] = VDD_Reg_TS3;
        }
#endif /* defined(CONFIG_MACH_LYNX_DL40) */
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out1-3 ret=%d\n", ret);
            return ret;
        }
    }

    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_terminal);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out3 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_timing);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out4 ret=%d\n", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_initial2);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out5 ret=%d\n", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_power);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out6 ret=%d\n", ret);
        return ret;
    }
    if ((device_code == 0x00) || (device_code == 0x01)){
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_sync_ts1_0);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out7 ret=%d\n", ret);
            return ret;
        }
    }
    else {
        ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_sync_ts2_0);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_DEBUG("out7 ret=%d\n", ret);
            return ret;
        }
    }
#endif


#ifndef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
    shdisp_clmr_api_hsclk_off();
#endif


    SHDISP_TRACE("out\n");
    return ret;
}

static int shdisp_andy_mipi_cmd_lcd_off_black_screen_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
#endif
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_display_on_black_screen);
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif
    SHDISP_TRACE("out\n");
    return ret;
}

static int shdisp_andy_mipi_cmd_lcd_on_after_black_screen(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
#ifdef SHDISP_LOW_POWER_MODE
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_LOW_POWER_MODE);
#else
    shdisp_panel_API_mipi_set_transfer_mode(SHDISP_DSI_HIGH_SPEED_MODE);
#endif
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
#endif
    ret = MIPI_DSI_COMMAND_TX_CLMR(mipi_sh_andy_cmds_display_on_after_black_screen);
#ifdef SHDISP_FW_STACK_EXCUTE
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
#endif
    SHDISP_TRACE("out\n");
    return ret;
}

int shdisp_andy_API_mipi_lcd_on_after_black_screen(void)
{
    int ret = 0;

    SHDISP_TRACE("in\n");
    ret = shdisp_andy_mipi_cmd_lcd_on_after_black_screen();
    shdisp_clmr_api_auto_pat_ctrl(MIPI_SHARP_CLMR_AUTO_PAT_OFF);
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

int shdisp_andy_API_mipi_lcd_off_black_screen_on(void)
{
    SHDISP_TRACE("in\n");
    shdisp_clmr_api_auto_pat_ctrl(MIPI_SHARP_CLMR_1HZ_BLACK_ON);
    shdisp_andy_mipi_cmd_lcd_off_black_screen_on();
    SHDISP_TRACE("out\n");
    return 0;
}

int shdisp_andy_API_mipi_start_display(void)
{
    SHDISP_TRACE("in\n");

    shdisp_andy_mipi_cmd_start_display();

    SHDISP_TRACE("out\n");
    return 0;
}


int shdisp_andy_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma)
{
    int ret = 0;
    int cnt, idx;
    unsigned int checksum;

    SHDISP_TRACE("in\n");

    memcpy(mipi_sh_andy_set_val_GAMMAREDposi,         mipi_sh_andy_cmd_GAMMAREDposi,         sizeof(mipi_sh_andy_set_val_GAMMAREDposi));
    memcpy(mipi_sh_andy_set_val_GAMMAREDnega,         mipi_sh_andy_cmd_GAMMAREDnega,         sizeof(mipi_sh_andy_set_val_GAMMAREDnega));
    memcpy(mipi_sh_andy_set_val_GAMMAGREENposi,       mipi_sh_andy_cmd_GAMMAGREENposi,       sizeof(mipi_sh_andy_set_val_GAMMAGREENposi));
    memcpy(mipi_sh_andy_set_val_GAMMAGREENnega,       mipi_sh_andy_cmd_GAMMAGREENnega,       sizeof(mipi_sh_andy_set_val_GAMMAGREENnega));
    memcpy(mipi_sh_andy_set_val_GAMMABLUEposi,        mipi_sh_andy_cmd_GAMMABLUEposi,        sizeof(mipi_sh_andy_set_val_GAMMABLUEposi));
    memcpy(mipi_sh_andy_set_val_GAMMABLUEnega,        mipi_sh_andy_cmd_GAMMABLUEnega,        sizeof(mipi_sh_andy_set_val_GAMMABLUEnega));
    memcpy(mipi_sh_andy_set_val_RegulatorPumpSetting, mipi_sh_andy_cmd_RegulatorPumpSetting, sizeof(mipi_sh_andy_set_val_RegulatorPumpSetting));

    if(phy_gamma == NULL) {
        SHDISP_ERR("phy_gamma is NULL.\n");
        ret = -1;
    }
    else if(phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK) {
        SHDISP_ERR("gammg status invalid. status=%02x\n", phy_gamma->status);
        ret = -1;
    }
    else {
        checksum = phy_gamma->status;
        for(cnt = 0; cnt < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; cnt++) {
            checksum = checksum + phy_gamma->buf[cnt];
        }
#if 1
        for(cnt = 0; cnt < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; cnt++) {
            checksum = checksum + phy_gamma->applied_voltage[cnt];
        }
#endif
        if((checksum & 0x00FFFFFF) != phy_gamma->chksum) {
            SHDISP_ERR("%s: gammg chksum NG. chksum=%06x calc_chksum=%06x\n", __func__, phy_gamma->chksum, (checksum & 0x00FFFFFF));
            ret = -1;
        }
        else {
            for(cnt = 0; cnt < SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET; cnt++) {
                idx = cnt * 2;
                mipi_sh_andy_set_val_GAMMAREDposi[idx][1]       = ((phy_gamma->buf[cnt] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMAREDposi[idx + 1][1]   = ( phy_gamma->buf[cnt] & 0x00FF);
                mipi_sh_andy_set_val_GAMMAREDnega[idx][1]       = ((phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET    ] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMAREDnega[idx + 1][1]   = ( phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET    ] & 0x00FF);
                mipi_sh_andy_set_val_GAMMAGREENposi[idx][1]     = ((phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 2] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMAGREENposi[idx + 1][1] = ( phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 2] & 0x00FF);
                mipi_sh_andy_set_val_GAMMAGREENnega[idx][1]     = ((phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 3] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMAGREENnega[idx + 1][1] = ( phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 3] & 0x00FF);
                mipi_sh_andy_set_val_GAMMABLUEposi[idx][1]      = ((phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 4] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMABLUEposi[idx + 1][1]  = ( phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 4] & 0x00FF);
                mipi_sh_andy_set_val_GAMMABLUEnega[idx][1]      = ((phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 5] >> 8) & 0x0003);
                mipi_sh_andy_set_val_GAMMABLUEnega[idx + 1][1]  = ( phy_gamma->buf[cnt + SHDISP_ANDY_GAMMA_NEGATIVE_OFFSET * 5] & 0x00FF);
            }
#if 1
            cnt = 0;
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VGH][1]    = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VGL][1]    = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][1]  = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][1]  = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][1] = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VGHO][1]   = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_VGLO][1]   = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][1]  = phy_gamma->applied_voltage[cnt++];
            mipi_sh_andy_set_val_RegulatorPumpSetting[SHDISP_ANDY_AVEER][1]  = phy_gamma->applied_voltage[cnt++];
#endif
        }
    }


    SHDISP_TRACE("out ret=%04x\n", ret);
    return ret;
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_dump_reg                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_andy_API_dump_reg(int cog)
{
    int i, arraysize;
    struct dsi_cmd_desc *dumpptr;
    unsigned char addr, page, read_data;
    unsigned char device_code = 0;

    device_code = shdisp_api_get_device_code();

    SHDISP_TRACE("in PANEL PARAMETER INFO ->>");

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1);
    dumpptr   = mipi_sh_andy_cmds_initial1;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    if (device_code == 0x00){
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts1_0);
        dumpptr   = mipi_sh_andy_cmds_initial1_regulator_ts1_0;
    }
    else if (device_code == 0x01) {
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts1_1);
        dumpptr   = mipi_sh_andy_cmds_initial1_regulator_ts1_1;
    }
    else {
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
        dumpptr   = mipi_sh_andy_cmds_initial1_regulator_ts2_0;
    }

    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_gamma);
    dumpptr   = mipi_sh_andy_cmds_gamma;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_terminal);
    dumpptr   = mipi_sh_andy_cmds_terminal;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_timing);
    dumpptr   = mipi_sh_andy_cmds_timing;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial2);
    dumpptr   = mipi_sh_andy_cmds_initial2;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_power);
    dumpptr   = mipi_sh_andy_cmds_power;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    if ((device_code == 0x00) || (device_code == 0x01)){
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_sync_ts1_0);
        dumpptr   = mipi_sh_andy_cmds_sync_ts1_0;
    }
    else {
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_sync_ts2_0);
        dumpptr   = mipi_sh_andy_cmds_sync_ts2_0;
    }

    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(cog, addr, &page, 1);
            SHDISP_DEBUG("PANEL_PARA_DUMP PAGE: %02X\n", page);
        }
        else {
            if (dumpptr->dtype == DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(cog, addr, &read_data, 1);
                SHDISP_DEBUG("PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }
    SHDISP_TRACE("out PANEL PARAMETER INFO <<-");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* CONFIG_ANDROID_ENGINEERING */




MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
