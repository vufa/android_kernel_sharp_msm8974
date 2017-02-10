/* drivers/sharp/shdisp/shdisp_panel.h  (Display Driver)
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

#ifndef SHDISP_PANEL_API_H
#define SHDISP_PANEL_API_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/types.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* dcs read/write */
#define DTYPE_DCS_WRITE     0x05    /* short write, 0 parameter */
#define DTYPE_DCS_WRITE1    0x15    /* short write, 1 parameter */
#define DTYPE_DCS_READ      0x06    /* read */
#define DTYPE_DCS_LWRITE    0x39    /* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE     0x03    /* short write, 0 parameter */
#define DTYPE_GEN_WRITE1    0x13    /* short write, 1 parameter */
#define DTYPE_GEN_WRITE2    0x23    /* short write, 2 parameter */
#define DTYPE_GEN_LWRITE    0x29    /* long write */
#define DTYPE_GEN_READ      0x04    /* long read, 0 parameter */
#define DTYPE_GEN_READ1     0x14    /* long read, 1 parameter */
#define DTYPE_GEN_READ2     0x24    /* long read, 2 parameter */

#define DTYPE_TEAR_ON       0x35    /* set tear on */
#define DTYPE_MAX_PKTSIZE   0x37    /* set max packet size */
#define DTYPE_NULL_PKT      0x09    /* null packet, no data */
#define DTYPE_BLANK_PKT     0x19    /* blankiing packet, no data */

#define DTYPE_CM_ON     0x02    /* color mode off */
#define DTYPE_CM_OFF        0x12    /* color mode on */
#define DTYPE_PERIPHERAL_OFF    0x22
#define DTYPE_PERIPHERAL_ON 0x32

/*
 * dcs response
 */
#define DTYPE_ACK_ERR_RESP      0x02
#define DTYPE_EOT_RESP          0x08    /* end of tx */
#define DTYPE_GEN_READ1_RESP    0x11    /* 1 parameter, short */
#define DTYPE_GEN_READ2_RESP    0x12    /* 2 parameter, short */
#define DTYPE_GEN_LREAD_RESP    0x1a
#define DTYPE_DCS_LREAD_RESP    0x1c
#define DTYPE_DCS_READ1_RESP    0x21    /* 1 parameter, short */
#define DTYPE_DCS_READ2_RESP    0x22    /* 2 parameter, short */

#define DSI_HOST_HDR_SIZE   4
#define DSI_HDR_LAST        BIT(31)
#define DSI_HDR_LONG_PKT    BIT(30)
#define DSI_HDR_BTA     BIT(29)
#define DSI_HDR_VC(vc)      (((vc) & 0x03) << 22)
#define DSI_HDR_DTYPE(dtype)    (((dtype) & 0x03f) << 16)
#define DSI_HDR_DATA2(data) (((data) & 0x0ff) << 8)
#define DSI_HDR_DATA1(data) ((data) & 0x0ff)
#define DSI_HDR_WC(wc)      ((wc) & 0x0ffff)

#define DSI_BUF_SIZE    64
#define MIPI_DSI_MRPS   0x04    /* Maximum Return Packet Size */

#define MIPI_DSI_LEN 8 /* 4 x 4 - 6 - 2, bytes dcs header+crc-align  */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DSI_LOW_POWER_MODE = 0,
    SHDISP_DSI_LOW_POWER_MODE_MS,
    SHDISP_DSI_LOW_POWER_MODE_SL,
    SHDISP_DSI_LOW_POWER_MODE_BOTH,

    SHDISP_DSI_HIGH_SPEED_MODE,
    SHDISP_DSI_HIGH_SPEED_MODE_MS,
    SHDISP_DSI_HIGH_SPEED_MODE_SL,
    SHDISP_DSI_HIGH_SPEED_MODE_BOTH,

    SHDISP_DSI_TRANSFER_MODE_MAX
};

enum {
    SHDISP_PANEL_RATE_60_0, /* 60   Hz */
    SHDISP_PANEL_RATE_40_0, /* 40   Hz */
    SHDISP_PANEL_RATE_30_0, /* 30   Hz */
    SHDISP_PANEL_RATE_24_0, /* 24   Hz */
    SHDISP_PANEL_RATE_20_0, /* 20   Hz */
    SHDISP_PANEL_RATE_15_0, /* 15   Hz */
    SHDISP_PANEL_RATE_10_0, /* 10   Hz */
    SHDISP_PANEL_RATE_5_0,  /*  5   Hz */
    SHDISP_PANEL_RATE_4_0,  /*  4   Hz */
    SHDISP_PANEL_RATE_3_0,  /*  3   Hz */
    SHDISP_PANEL_RATE_2_5,  /*  2.5 Hz */
    SHDISP_PANEL_RATE_2_0,  /*  2   Hz */
    SHDISP_PANEL_RATE_1_5,  /*  1.5 Hz */
    SHDISP_PANEL_RATE_1,    /*  1   Hz */
    SHDISP_PANEL_RATE_0_8,  /*  0.8 Hz */
    SHDISP_PANEL_RATE_0_6,  /*  0.6 Hz */
};

typedef u32 uint32;

struct dsi_cmd_desc {
    short dtype;
    char last;
    char vc;
    char ack;    /* ask ACK from peripheral */
    int  wait;
    short dlen;
    char *payload;
};

struct dsi_buf {
    uint32 *hdr;    /* dsi host header */
    char *start;    /* buffer start addr */
    char *end;  /* buffer end addr */
    int size;   /* size of buffer */
    char *data; /* buffer */
    int len;    /* data length */
};

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int shdisp_panel_API_check_panel_info(void);

int shdisp_panel_API_SQE_write_reg(struct shdisp_lcddr_reg *panel_reg);
int shdisp_panel_API_SQE_read_reg(struct shdisp_lcddr_reg *panel_reg);

void shdisp_panel_API_create(void);
int shdisp_panel_API_init_io(void);
int shdisp_panel_API_exit_io(void);
int shdisp_panel_API_power_on(int mode);
int shdisp_panel_API_power_off(int mode);
int shdisp_panel_API_disp_on(void);
int shdisp_panel_API_disp_off(void);
int shdisp_panel_API_diag_write_reg(int cog, unsigned char addr, unsigned char *write_data, unsigned char size);
int shdisp_panel_API_diag_read_reg(int cog, unsigned char addr, unsigned char *read_data, unsigned char size);
#ifndef SHDISP_NOT_SUPPORT_FLICKER
int shdisp_panel_API_diag_set_flicker_param(struct shdisp_diag_flicker_param alpha);
int shdisp_panel_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *alpha);
int shdisp_panel_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
int shdisp_panel_API_check_recovery(void);
int shdisp_panel_API_get_recovery_type(int *type);

int shdisp_panel_API_cabc_init(void);
int shdisp_panel_API_cabc_indoor_on(void);
int shdisp_panel_API_cabc_outdoor_on(int lut_level);
int shdisp_panel_API_cabc_off(int wait_on, int pwm_disable);
int shdisp_panel_API_cabc_outdoor_move(int lut_level);

int shdisp_panel_API_check_upper_unit(void);
int shdisp_panel_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
int shdisp_panel_API_disp_update(struct shdisp_main_update *update);
int shdisp_panel_API_disp_clear_screen(struct shdisp_main_clear *clear);
int shdisp_panel_API_mipi_diag_write_reg(struct dsi_buf *tp, unsigned char addr, char *write_data, unsigned char size);
#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
int shdisp_panel_API_mipi_diag_mltshortpkt_write_reg(struct dsi_buf *tp, unsigned char * addrs, unsigned char * write_data, unsigned int size);
#endif
int shdisp_panel_API_mipi_diag_read_reg(struct dsi_buf *tp, struct dsi_buf *rp, unsigned char addr, unsigned char *read_data, unsigned char size);
#if defined(CONFIG_SHDISP_PANEL_GEMINI)
int shdisp_panel_API_mipi_diag_write_reg_multi_cog(struct dsi_buf *tp, unsigned char addr, unsigned char *write_data, unsigned char size, int mode);
int shdisp_panel_API_mipi_diag_read_reg_multi_cog(struct dsi_buf *tp, struct dsi_buf *rp, unsigned char addr, unsigned char *read_data, unsigned char size, int mode);
#endif

int shdisp_panel_API_mipi_cmd_lcd_on(void);
int shdisp_panel_API_mipi_cmd_lcd_off(void);
int shdisp_panel_API_mipi_cmd_start_display(void);
int shdisp_panel_API_mipi_cmd_lcd_on_after_black_screen(void);
int shdisp_panel_API_mipi_cmd_lcd_off_black_screen_on(void);
int shdisp_panel_API_mipi_cmd_stop_prepare(void);
void shdisp_panel_API_request_RateCtrl( int ctrl, unsigned char maxFR, unsigned char minFR );
void shdisp_panel_API_detect_bad_timing_transfer( int ctrl );
int shdisp_panel_API_mipi_dsi_cmds_tx(struct dsi_buf *tp, struct dsi_cmd_desc *cmds, int cnt);
void shdisp_panel_API_mipi_dsi_cmds_tx_dummy(struct dsi_cmd_desc *cmds);
#ifndef SHDISP_NOT_SUPPORT_COMMAND_MLTPKT_TX_CLMR
int shdisp_panel_API_mipi_dsi_cmds_mltshortpkt_tx(struct dsi_buf *tp, struct dsi_cmd_desc *cmds, int cnt);
int shdisp_panel_API_mipi_dsi_cmds_mltshortpkt_tx_LP(struct dsi_buf *tpa, struct dsi_buf *tpb, struct dsi_buf *tpc, struct dsi_cmd_desc *cmds, int cnt);
#endif
int shdisp_panel_API_mipi_dsi_cmds_rx(struct dsi_buf *tp, struct dsi_buf *rp, struct dsi_cmd_desc *cmds, unsigned char size);
#if defined(CONFIG_SHDISP_PANEL_GEMINI)
int shdisp_panel_API_mipi_dsi_cmds_rx2(struct dsi_buf *tp, struct dsi_buf *rp, struct dsi_cmd_desc *cmds, unsigned char size);
#endif
int shdisp_panel_API_mipi_set_transfer_mode(int mode);

int shdisp_panel_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
int shdisp_panel_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
int shdisp_panel_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);

#if defined (CONFIG_ANDROID_ENGINEERING)
int shdisp_panel_API_dump_reg(int cog);
#endif /* CONFIG_ANDROID_ENGINEERING */

int shdisp_panel_API_set_drive_freq(int type);
int shdisp_panel_API_get_drive_freq(void);
int shdisp_panel_API_shutdown(void);
#if defined(CONFIG_SHDISP_PANEL_GEMINI)
int shdisp_panel_API_mipi_cmd_is_retry_over_err(void);
#endif
#endif /* SHDISP_PANEL_API_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

