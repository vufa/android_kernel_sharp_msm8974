/* include/sharp/shdisp_kerl_lcdc.h  (Display Driver)
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

#ifndef SHDISP_KERN_CLMR_H
#define SHDISP_KERN_CLMR_H

/* ------------------------------------------------------------------------- */
/* NOT SUPPORT FUNCTION                                                      */
/* ------------------------------------------------------------------------- */

#define SHDISP_NOT_SUPPORT_PANEL

#define SHDISP_NOT_SUPPORT_CABC

#if defined(SHDISP_FACTORY_MODE_ENABLE)
#define SHDISP_NOT_SUPPORT_DET
#else
#endif


#if defined(CONFIG_SHDISP_PANEL_RYOMA) && defined(CONFIG_USES_SHLCDC)
#define SHDISP_USE_EXT_CLOCK
#endif

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_GEMINI)
#else
#define SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
#endif


#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_RYOMA) || defined(CONFIG_SHDISP_PANEL_GEMINI)
#endif /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_RYOMA) */

#if defined(CONFIG_MACH_LYNX_DL45) || defined(CONFIG_MACH_TBS) || defined(CONFIG_MACH_DECKARD_AS87)
#define SHDISP_LOW_POWER_MODE
#else
#endif

#define SHDISP_NOT_SUPPORT_EWB_2SURFACE

#if defined(CONFIG_MACH_LYNX_DL40) || defined(CONFIG_MACH_DECKARD_AS97) || defined(CONFIG_MACH_ATK) \
||  defined(CONFIG_MACH_LYNX_DL45) || defined(CONFIG_MACH_DECKARD_AS87) || defined(CONFIG_MACH_TBS) \
||  defined(CONFIG_MACH_DECKARD_GP7K)
#else
#define SHDISP_NOT_SUPPORT_LCDC_TEARINT
#endif

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_LCDC_WRITE_REG                     _IOW  (SHDISP_IOC_MAGIC, 100, struct shdisp_diag_lcdc_reg)
#define SHDISP_IOCTL_LCDC_READ_REG                      _IOR  (SHDISP_IOC_MAGIC, 101, struct shdisp_diag_lcdc_reg)
#define SHDISP_IOCTL_LCDC_I2C_WRITE                     _IOW  (SHDISP_IOC_MAGIC, 102, struct shdisp_diag_lcdc_i2c)
#define SHDISP_IOCTL_LCDC_I2C_READ                      _IOR  (SHDISP_IOC_MAGIC, 103, struct shdisp_diag_lcdc_i2c)
#define SHDISP_IOCTL_LCDC_POW_CTL                       _IOW  (SHDISP_IOC_MAGIC, 104, int)
#define SHDISP_IOCTL_BDIC_POW_CTL                       _IOW  (SHDISP_IOC_MAGIC, 105, int)
#define SHDISP_IOCTL_SET_GAMMA_INFO                     _IOW  (SHDISP_IOC_MAGIC, 106, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_GET_GAMMA_INFO                     _IOWR (SHDISP_IOC_MAGIC, 107, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_SET_GAMMATABLE_AND_VOLTAGE         _IOW  (SHDISP_IOC_MAGIC, 106, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_GET_GAMMATABLE_AND_VOLTAGE         _IOWR (SHDISP_IOC_MAGIC, 107, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_SET_GAMMA                          _IOW  (SHDISP_IOC_MAGIC, 108, struct shdisp_diag_gamma)
#define SHDISP_IOCTL_SET_EWB_TBL                        _IOW  (SHDISP_IOC_MAGIC, 109, struct shdisp_diag_ewb_tbl)
#define SHDISP_IOCTL_SET_EWB                            _IOW  (SHDISP_IOC_MAGIC, 110, struct shdisp_diag_set_ewb)
#define SHDISP_IOCTL_READ_EWB                           _IOWR (SHDISP_IOC_MAGIC, 111, struct shdisp_diag_read_ewb)
#define SHDISP_IOCTL_SET_EWB_TBL2                       _IOW  (SHDISP_IOC_MAGIC, 112, struct shdisp_diag_ewb_tbl)
#define SHDISP_IOCTL_LCDC_SET_TRV_PARAM                 _IOW  (SHDISP_IOC_MAGIC, 113, struct shdisp_trv_param)
#define SHDISP_IOCTL_LCDC_SET_DBC_PARAM                 _IOW  (SHDISP_IOC_MAGIC, 114, struct shdisp_main_dbc)
#define SHDISP_IOCTL_LCDC_SET_FLICKER_TRV               _IOW  (SHDISP_IOC_MAGIC, 115, struct shdisp_flicker_trv)
#define SHDISP_IOCTL_LCDC_FW_CMD_WRITE                  _IOW  (SHDISP_IOC_MAGIC, 116, struct shdisp_diag_fw_cmd)
#define SHDISP_IOCTL_LCDC_FW_CMD_READ                   _IOR  (SHDISP_IOC_MAGIC, 117, struct shdisp_diag_fw_cmd)
#define SHDISP_IOCTL_LCDC_SET_PIC_ADJ_PARAM             _IOW  (SHDISP_IOC_MAGIC, 118, struct shdisp_main_pic_adj)
#define SHDISP_IOCTL_LCDC_SET_AE_PARAM                  _IOW  (SHDISP_IOC_MAGIC, 119, struct shdisp_main_ae)
#define SHDISP_IOCTL_LCDC_SET_PIC_ADJ_AP_TYPE           _IOW  (SHDISP_IOC_MAGIC, 120, unsigned short)
#define SHDISP_IOCTL_LCDC_SET_DRIVE_FREQ                _IOW  (SHDISP_IOC_MAGIC, 121, struct shdisp_main_drive_freq)
#define SHDISP_IOCTL_SET_FLICKER_PARAM_MULTI_COG        _IOW  (SHDISP_IOC_MAGIC, 122, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_FLICKER_PARAM_MULTI_COG        _IOWR (SHDISP_IOC_MAGIC, 123, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_FLICKER_LOW_PARAM_MULTI_COG    _IOWR (SHDISP_IOC_MAGIC, 124, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_PANEL_INIT_STATE               _IOR  (SHDISP_IOC_MAGIC, 125, int)

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_LCDDR_GAMMA_STATUS_OK            0x96
#define SHDISP_LCDDR_GAMMA_STATUS_OK_2          0x97

enum {
     SHDISP_PANEL_POWER_NORMAL_ON,
     SHDISP_PANEL_POWER_RECOVERY_ON
};

enum {
     SHDISP_PANEL_POWER_NORMAL_OFF,
     SHDISP_PANEL_POWER_RECOVERY_OFF,
     SHDISP_PANEL_POWER_SHUTDOWN_OFF
};

enum {
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_00,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_01,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_02,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_03,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_04,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_05,
    SHDISP_MAIN_DISP_PIC_ADJ_MODE_06,
    NUM_SHDISP_MAIN_DISP_PIC_ADJ_MODE
};

enum {
    SHDISP_LCDC_TRV_REQ_STOP,
    SHDISP_LCDC_TRV_REQ_START,
    SHDISP_LCDC_TRV_REQ_SET_PARAM,
    NUM_SHDISP_LCDC_TRV_REQ
};
enum {
    SHDISP_LCDC_TRV_STRENGTH_00,
    SHDISP_LCDC_TRV_STRENGTH_01,
    SHDISP_LCDC_TRV_STRENGTH_02,
    NUM_SHDISP_LCDC_TRV_STRENGTH
};
enum {
    SHDISP_LCDC_TRV_ADJUST_00,
    SHDISP_LCDC_TRV_ADJUST_01,
    SHDISP_LCDC_TRV_ADJUST_02,
    SHDISP_LCDC_TRV_ADJUST_03,
    SHDISP_LCDC_TRV_ADJUST_04,
    SHDISP_LCDC_TRV_ADJUST_05,
    SHDISP_LCDC_TRV_ADJUST_06,
    SHDISP_LCDC_TRV_ADJUST_07,
    SHDISP_LCDC_TRV_ADJUST_08,
    SHDISP_LCDC_TRV_ADJUST_09,
    SHDISP_LCDC_TRV_ADJUST_10,
    SHDISP_LCDC_TRV_ADJUST_11,
    SHDISP_LCDC_TRV_ADJUST_12,
    NUM_SHDISP_LCDC_TRV_ADJUST
};
#define SHDISP_LCDC_TRV_DATA_HEADER_SIZE        16
#define SHDISP_LCDC_TRV_DATA_HEADER_LENGTH_SIZE 4
#define SHDISP_LCDC_TRV_DATA_HEADER_H_SIZE      2
#define SHDISP_LCDC_TRV_DATA_HEADER_Y_SIZE      2

enum {
    SHDISP_LCDC_FLICKER_TRV_OFF,
    SHDISP_LCDC_FLICKER_TRV_ON,
    NUM_SHDISP_LCDC_FLICKER_TRV_REQUEST
};
enum {
    SHDISP_LCDC_FLICKER_TRV_COLUMN,
    SHDISP_LCDC_FLICKER_TRV_DOT1H,
    SHDISP_LCDC_FLICKER_TRV_DOT2H,
    NUM_SHDISP_LCDC_FLICKER_TRV_TYPE,
};

enum {
    SHDISP_MAIN_DISP_DBC_MODE_OFF,
    SHDISP_MAIN_DISP_DBC_MODE_DBC,
    NUM_SHDISP_MAIN_DISP_DBC_MODE
};
enum {
    SHDISP_MAIN_DISP_DBC_AUTO_MODE_OFF,
    SHDISP_MAIN_DISP_DBC_AUTO_MODE_ON,
    NUM_SHDISP_MAIN_DISP_DBC_AUTO_MODE
};

enum {
    SHDISP_MAIN_DISP_AE_TIME_DAYTIME,
    SHDISP_MAIN_DISP_AE_TIME_MIDNIGHT,
    SHDISP_MAIN_DISP_AE_TIME_MORNING,
    NUM_SHDISP_MAIN_DISP_AE_TIME
};

enum {
    SHDISP_LCDC_PIC_ADJ_AP_NORMAL,
    SHDISP_LCDC_PIC_ADJ_AP_1SEG,
    SHDISP_LCDC_PIC_ADJ_AP_FULLSEG,
    SHDISP_LCDC_PIC_ADJ_AP_TMM,
    SHDISP_LCDC_PIC_ADJ_AP_CAM,
    NUM_SHDISP_LCDC_PIC_ADJ_AP
};


enum {
    SHDISP_PANEL_INIT_SUCCESS,
    SHDISP_PANEL_INIT_FAILURE,
    SHDISP_PANEL_INIT_NOINIT,
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_CLMR_IS_NOT_EXIST,
    SHDISP_CLMR_IS_EXIST,
    NUM_CLMR_EXIST_STATUS
};


struct shdisp_diag_fw_cmd {
    unsigned short  cmd;
    unsigned short  write_count;
    unsigned char   write_val[512];
    unsigned short  read_count;
    unsigned char   read_val[16];
};

struct shdisp_clmr_ewb_accu {
    unsigned short valR[SHDISP_LCDC_EWB_TBL_SIZE];
    unsigned short valG[SHDISP_LCDC_EWB_TBL_SIZE];
    unsigned short valB[SHDISP_LCDC_EWB_TBL_SIZE];
};

#include "shdisp_boot_context.h"
#include "shdisp_kerl_context.h"

#define SHDISP_TRV_DATA_MAX    (102400)
struct shdisp_clmr_trv_info {
    int            status;
    int            strength;
    int            adjust;
    unsigned int   data_size;
    unsigned short hw;
    unsigned short y_size;
    unsigned char  data[SHDISP_TRV_DATA_MAX];
};
struct shdisp_trv_data_header {
    unsigned int   data_size;
    unsigned short hw;
    unsigned short y_size;
    unsigned int   reserve[2];
};

struct shdisp_diag_set_ewb {
    unsigned short  valR;
    unsigned short  valG;
    unsigned short  valB;
};


struct shdisp_main_pic_adj {
    int mode;
};

struct shdisp_trv_param {
    int            request;
    int            strength;
    int            adjust;
    unsigned char  *data;
};


struct shdisp_main_ae {
    unsigned char time;
};

struct shdisp_main_common_info {
    unsigned short mode;
};

struct shdisp_flicker_trv {
    int request;
    unsigned char level;
    unsigned char type;
};

struct shdisp_main_drive_freq {
    int freq;
};

struct shdisp_panel_operations {
    int (*init_io) (void);
    int (*exit_io) (void);
    int (*power_on) (int mode);
    int (*power_off) (int mode);
    int (*disp_on) (void);
    int (*disp_off)(void);
    int (*after_black_screen)(void);
    int (*black_screen_on)(void);
    int (*start_display)(void);
    int (*write_reg) (int cog, unsigned char addr, unsigned char *write_data, unsigned char size);
    int (*read_reg) (int cog, unsigned char addr, unsigned char *read_data, unsigned char size);
    int (*check_flicker) (unsigned short alpha_in, unsigned short *alpha_out);
    int (*set_flicker) (struct shdisp_diag_flicker_param alpha);
    int (*get_flicker) (struct shdisp_diag_flicker_param *alpha);
    int (*get_flicker_low) (struct shdisp_diag_flicker_param *alpha);
    int (*check_recovery) (void);
    int (*set_gamma_info) (struct shdisp_diag_gamma_info *gamma_info);
    int (*get_gamma_info) (struct shdisp_diag_gamma_info *gamma_info);
    int (*set_gamma) (struct shdisp_diag_gamma *gamma);
    int (*set_drive_freq) (int type);
    int (*get_drive_freq) (void);
    int (*shutdown)(void);
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shdisp_ioctl_lcdc_write_reg(void __user *argp);
int shdisp_ioctl_lcdc_read_reg(void __user *argp);
int shdisp_ioctl_lcdc_devchk(void);
int shdisp_ioctl_lcdc_i2c_write(void __user *argp);
int shdisp_ioctl_lcdc_i2c_read(void __user *argp);

int shdisp_ioctl_lcdc_set_ewb_tbl(void __user *argp);
int shdisp_ioctl_lcdc_set_ewb(void __user *argp);
int shdisp_ioctl_lcdc_read_ewb(void __user *argp);
int shdisp_ioctl_lcdc_set_ewb_tbl2(void __user *argp);
int shdisp_ioctl_lcdc_set_pic_adj_param(void __user *argp);
int shdisp_ioctl_lcdc_set_trv_param(void __user *argp);
int shdisp_ioctl_lcdc_set_dbc_param(void __user *argp);
int shdisp_ioctl_lcdc_set_ae_param(void __user *argp);
int shdisp_ioctl_lcdc_set_pic_adj_ap_type(void __user *argp);
int shdisp_ioctl_lcdc_set_flicker_trv(void __user *argp);
int shdisp_ioctl_lcdc_fw_cmd_write(void __user *argp);
int shdisp_ioctl_lcdc_fw_cmd_read(void __user *argp);
int shdisp_ioctl_lcdc_set_drive_freq(void __user *argp);


struct shdisp_clmr_ewb_accu* shdisp_api_get_clmr_ewb_accu(unsigned char no);
int shdisp_api_lcd_on(void);
int shdisp_api_lcd_off(int para);
int shdisp_api_start_display(void);

int shdisp_api_main_mipi_cmd_lcd_probe(void);
int shdisp_api_main_mipi_cmd_lcd_on(void);
int shdisp_api_main_mipi_cmd_lcd_off(void);
int shdisp_api_main_mipi_cmd_lcd_start_display(void);
int shdisp_api_check_det(void);
void shdisp_api_requestrecovery(void);
int shdisp_api_main_mipi_cmd_lcd_stop_prepare(void);
int shdisp_api_main_mipi_cmd_lcd_off_black_screen_on(void);
int shdisp_api_main_mipi_cmd_lcd_on_after_black_screen(void);
int shdisp_api_main_mipi_cmd_is_fw_timeout(void);
#if defined(CONFIG_SHDISP_PANEL_GEMINI)
int shdisp_api_main_mipi_cmd_is_retry_over_err(void);
#endif

int shdisp_api_do_lcdc_mipi_dsi_det_recovery(void);

unsigned short shdisp_api_get_hw_handset(void);
unsigned char shdisp_api_get_device_code(void);
int shdisp_api_set_device_code(unsigned char device_code);
unsigned short shdisp_api_get_alpha_nvram(void);
unsigned short shdisp_api_get_slave_alpha(void);
unsigned short shdisp_api_get_slave_alpha_low(void);

unsigned char shdisp_api_get_pll_on_ctl_count(void);
int shdisp_api_set_pll_on_ctl_count(unsigned char pll_on_ctl_count);
void shdisp_api_set_pll_on_ctl_reg(int ctrl);

#endif /* SHDISP_KERN_CLMR_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
