/* drivers/sharp/shgrip/shgrip_kerl.h
 *
 * Copyright (C) 2013 SHARP CORPORATION
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
#ifndef __SHGRIP_KERL_H__
#define __SHGRIP_KERL_H__

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
#define SHGRIP_IOC_MAGIC					's'

#define SHGRIP_IOCTL_GRIP_SENSOR_ON			_IO  (SHGRIP_IOC_MAGIC,  1)
#define SHGRIP_IOCTL_GRIP_SENSOR_OFF		_IO  (SHGRIP_IOC_MAGIC,  2)
#define SHGRIP_IOCTL_SET_SENSOR_ADJUST		_IOW (SHGRIP_IOC_MAGIC,  3,  struct shgrip_user_setting)
#define SHGRIP_IOCTL_GET_SENSOR_ADJUST		_IOR (SHGRIP_IOC_MAGIC,  4,  struct shgrip_user_setting)
#define SHGRIP_IOCTL_GET_STATE				_IOR (SHGRIP_IOC_MAGIC,  5,  struct shgrip_sensor_state)
#define SHGRIP_IOCTL_CHANGE_DETECT_MODE		_IOW (SHGRIP_IOC_MAGIC,  6,  int)
#define SHGRIP_IOCTL_GET_FW_VERSION			_IOR (SHGRIP_IOC_MAGIC,  7,  struct shgrip_fw_version)
#define SHGRIP_IOCTL_DOWNLOAD_FW			_IOW (SHGRIP_IOC_MAGIC,  8,  struct shgrip_fw_data) 
#define SHGRIP_IOCTL_DIAG_SET_SENSOR_ADJUST	_IOW (SHGRIP_IOC_MAGIC,  9,  struct shgrip_diag_sens_setting_params)
#define SHGRIP_IOCTL_DIAG_GET_SENSOR_ADJUST	_IOWR(SHGRIP_IOC_MAGIC,  10, struct shgrip_diag_sens_setting_params)
#define SHGRIP_IOCTL_DIAG_GET_SENSOR_LEVEL	_IOR (SHGRIP_IOC_MAGIC,  11, struct shgrip_get_level)
#define SHGRIP_IOCTL_DIAG_CHANGE_RUN_MODE	_IOW (SHGRIP_IOC_MAGIC,  12, int)
#define SHGRIP_IOCTL_DIAG_SET_SAMPLING		_IOR (SHGRIP_IOC_MAGIC,  13, struct shgrip_sampling_val)
#define SHGRIP_IOCTL_DIAG_GET_SAMPLING		_IOR (SHGRIP_IOC_MAGIC,  14, struct shgrip_sampling_val)
#define SHGRIP_IOCTL_DIAG_GET_MODEL			_IOR (SHGRIP_IOC_MAGIC,  15, unsigned char)
#define SHGRIP_IOCTL_DIAG_DOWNLOAD_FW		_IOW (SHGRIP_IOC_MAGIC,  16, int)
#define SHGRIP_IOCTL_DIAG_CHECK_SUM			_IOR (SHGRIP_IOC_MAGIC,  17, unsigned short)
#define SHGRIP_IOCTL_GET_MODE_STATE			_IOR (SHGRIP_IOC_MAGIC,  18, struct shgrip_mode_state)
#define SHGRIP_IOCTL_READ_SFR1_REG			_IOR (SHGRIP_IOC_MAGIC,  19, struct shgrip_sfr1_reg)
#define SHGRIP_IOCTL_READ_SFR2_REG			_IOWR(SHGRIP_IOC_MAGIC,  20, struct shgrip_sfr2_reg)
#define SHGRIP_IOCTL_THRESHOLD3_ON			_IOW (SHGRIP_IOC_MAGIC,  21, int)
#define SHGRIP_IOCTL_THRESHOLD3_OFF			_IOW (SHGRIP_IOC_MAGIC,  22, int)
#define SHGRIP_IOCTL_SET_THRESHOLD3_REG		_IOW (SHGRIP_IOC_MAGIC,  23, struct shgrip_threshold3_reg)
#define SHGRIP_IOCTL_GET_THRESHOLD3_REG		_IOR (SHGRIP_IOC_MAGIC,  24, struct shgrip_threshold3_reg)
#define SHGRIP_IOCTL_SET_DEBUG_PORT			_IOW (SHGRIP_IOC_MAGIC,  25, struct shgrip_debug_port)
#define SHGRIP_IOCTL_GET_DRV_STATUS			_IOR (SHGRIP_IOC_MAGIC,  26, unsigned char)
#define SHGRIP_IOCTL_RANDOM_ON				_IO  (SHGRIP_IOC_MAGIC,  27)
#define SHGRIP_IOCTL_RANDOM_OFF				_IO  (SHGRIP_IOC_MAGIC,  28)
#define SHGRIP_IOCTL_SET_RANDOM_VALUE		_IOW (SHGRIP_IOC_MAGIC,  29, struct shgrip_random_val)
#define SHGRIP_IOCTL_GET_RANDOM_VALUE		_IOR (SHGRIP_IOC_MAGIC,  30, struct shgrip_random_val)
#define SHGRIP_IOCTL_DRIFT_OFF				_IOW (SHGRIP_IOC_MAGIC, 31, unsigned char)
#define SHGRIP_IOCTL_SET_DRIFT_VALUE		_IOW (SHGRIP_IOC_MAGIC, 32, struct shgrip_drift_set)
#define SHGRIP_IOCTL_GET_DRIFT_VALUE		_IOR (SHGRIP_IOC_MAGIC, 33, struct shgrip_drift_val)
#define SHGRIP_IOCTL_GET_CHPRD2_VALUE		_IOR (SHGRIP_IOC_MAGIC, 34, struct shgrip_chprd2)
#define SHGRIP_IOCTL_DEBUG_RW_COMMAND		_IOWR(SHGRIP_IOC_MAGIC, 35, struct shgrip_dbg_command)

/* ------------------------------------------------------------------------- */
/* TYPE                                                                     */
/* ------------------------------------------------------------------------- */
enum {
	GRIP_RESULT_SUCCESS,
	GRIP_RESULT_FAILURE,
	GRIP_RESULT_FAILURE_USER,
	GRIP_RESULT_FAILURE_USER_STATE
};

enum {
	SHGRIP_TYPE_LPTIME_LEVEL00,
	SHGRIP_TYPE_LPTIME_LEVEL01,
	SHGRIP_TYPE_LPTIME_LEVEL02,
	SHGRIP_TYPE_LPTIME_LEVEL03,
	MAX_SHGRIP_TYPE_LPTIME
};

enum {
	SHGRIP_TYPE_SPTIME_LEVEL00,
	SHGRIP_TYPE_SPTIME_LEVEL01,
	SHGRIP_TYPE_SPTIME_LEVEL02,
	SHGRIP_TYPE_SPTIME_LEVEL03,
	MAX_SHGRIP_TYPE_SPTIME
};

enum {
	SHGRIP_TYPE_SENS_LEVEL00,
	SHGRIP_TYPE_SENS_LEVEL01,
	SHGRIP_TYPE_SENS_LEVEL02,
	SHGRIP_TYPE_SENS_LEVEL03,
	MAX_SHGRIP_TYPE_SENS
};

enum {
	SHGRIP_OFF,
	SHGRIP_ON
};

enum {
	SHGRIP_CH_A,
	SHGRIP_CH_B
};

enum {
	SHGRIP_DL_ALL_BLOCK,
	SHGRIP_DL_APP_BLOCK,
	MAX_SHGRIP_DL_BLOCK
};

enum {
	SHGRIP_NORMAL_MODE,
	SHGRIP_RUN_MODE,
	MAX_SHGRIP_ACTION_MODE
};

enum shgrip_state {
	STATE_POWER_OFF,
	STATE_SENSOR_OFF,
	STATE_SENSOR_ON,
	STATE_FW_DL
};

enum shgrip_loader_mode {
    MODE_LOADER0,
    MODE_LOADER1
};

enum shgrip_detect_mode{
	SHGRIP_CHANGE_AUTO_MODE,
	SHGRIP_CHANGE_MANU_MODE
};

struct shgrip_dcount {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_nref {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_dci {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_acd {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_df_scs {
	unsigned char val;
};

struct shgrip_nhys {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_msa {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_nthr {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_sampling_val {
	unsigned char sampling_a;
	unsigned char sampling_b;
	unsigned char sampling_c;
	unsigned char sampling_d;
};

struct shgrip_user_setting {
	unsigned char ch0;
	unsigned char ch2;
	unsigned char lptime;
	unsigned char sptime;
};

struct shgrip_params_val {
	struct shgrip_dcount dcount_val;
	struct shgrip_nref nref_val;
	struct shgrip_prm_dci prm_dci_val;
	struct shgrip_prm_acd prm_acd_val;
	struct shgrip_df_scs df_scs_val;
	struct shgrip_nhys nhys_val;
	struct shgrip_prm_msa prm_msa_val;
	struct shgrip_nthr nthr_val;
};

struct shgrip_drift_thr{
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scancnt{
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_drift_val{
	struct shgrip_drift_thr thr_val_ch0;
	struct shgrip_drift_thr thr_val_ch2;
	struct shgrip_scancnt scancnt_ch0;
	struct shgrip_dcount dcount_ch0;
	struct shgrip_scancnt scancnt_ch2;
	struct shgrip_dcount dcount_ch2;
};

struct shgrip_drift_set{
	unsigned char drift_on_ch;
	struct shgrip_drift_val val;
};

struct shgrip_diag_sens_setting_params {
	unsigned char ch;
	struct shgrip_params_val val;
};

struct shgrip_level_val {
	struct shgrip_dcount dcount_val;
	struct shgrip_nref nref_val;
	struct shgrip_nthr nthr_val;
};

struct shgrip_get_level {
	struct shgrip_level_val ch0;
	struct shgrip_level_val ch2;
};

struct shgrip_fw_version {
	unsigned short pver;
	unsigned short lver0;
	unsigned short lver1;
};

struct shgrip_fw_data {
	unsigned long size;
	unsigned char* data;
};

struct shgrip_sensor_state {
	unsigned char state_grip;
	unsigned char ch0;
	unsigned char ch2;
};

struct shgrip_mode_state {
	unsigned char run_mode;
	unsigned char event_enable;
	unsigned char manual_mode;
	unsigned char debug_mode;
	unsigned char ch0_th3_hys;
	unsigned char ch2_th3_hys;
	unsigned char ch0_drift_mode;
	unsigned char ch2_drift_mode;
	unsigned char ch0_th3_cancelset;
	unsigned char ch2_th3_cancelset;
	unsigned char uphosei_quick;
	unsigned char uphosei_slow;
	unsigned char uphosei_patern;
	unsigned char ch0_drift2_mode;
	unsigned char ch2_drift2_mode;
	unsigned char mode3_bit0;
	unsigned char mode3_bit1;
	unsigned char mode3_bit2;
	unsigned char mode3_bit3;
	unsigned char mode3_bit4;
	unsigned char mode3_bit5;
	unsigned char mode3_bit6;
	unsigned char mode3_bit7;
};

struct shgrip_sfr1_reg {
	unsigned char scucr0;
	unsigned char scufr;
	unsigned char scustc;
	unsigned char scuscc;
	unsigned char fra0;
};

struct shgrip_sfr2_reg {
	unsigned short addr;
	unsigned char val;
};

struct shgrip_threshold3_reg {
	struct shgrip_nhys nhys3_ch0;
	struct shgrip_nhys nhys3_ch2;
	struct shgrip_nthr nthr3_ch0;
	struct shgrip_nthr nthr3_ch2;
};

struct shgrip_debug_port {
	unsigned char p0;
	unsigned char p1;
	unsigned char p2;
	unsigned char p3;
	unsigned char p4;
	unsigned char p6;
};

#define SHGRIP_RNDVAL_DATA_SIZE				(8)
struct shgrip_random_val {
	unsigned char data[SHGRIP_RNDVAL_DATA_SIZE];
};

struct shgrip_ncount {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scudata {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_chprd2 {
	struct shgrip_dcount dcount_ch0;
	struct shgrip_nref nref_ch0;
	struct shgrip_nthr nthr_ch0;
	struct shgrip_dcount dcount_ch2;
	struct shgrip_nref nref_ch2;
	struct shgrip_nthr nthr_ch2;
	struct shgrip_ncount ncount_ch0;
	struct shgrip_scudata scudata_ch0;
	struct shgrip_ncount ncount_ch2;
	struct shgrip_scudata scudata_ch2;
};

struct shgrip_scancntn {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_thr3_cancelval {
	struct shgrip_scancntn scancntn_ch0;
	unsigned char scancntm_ch0;
	struct shgrip_scancntn scancntn_ch2;
	unsigned char scancntm_ch2;
};

struct shgrip_s_dtimes {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_s_delta {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_s_dfilter {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_uphosei {
	unsigned char hosei_onoff;
	unsigned char patern;
	unsigned char q_dtimes_ch0;
	unsigned char q_dtimes_ch2;
	unsigned char q_delta_ch0;
	unsigned char q_delta_ch2;
	unsigned char q_dfilter;
	struct shgrip_s_dtimes s_dtimes_ch0;
	struct shgrip_s_dtimes s_dtimes_ch2;
	struct shgrip_s_delta s_delta_ch0;
	struct shgrip_s_delta s_delta_ch2;
	struct shgrip_s_dfilter s_dfilter;
};

struct shgrip_drift_set2 {
	unsigned char drift_on_ch;
	unsigned char scancnt_ch0;
	unsigned char dcount_ch0;
	unsigned char scancnt_ch2;
	unsigned char dcount_ch2;
};

struct shgrip_msa {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_msa_x {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scmsa_val {
	unsigned char a;
	unsigned char b;
	struct shgrip_msa_x x;
	unsigned char y;
	unsigned char z;
};

struct shgrip_scmsa {
	unsigned char scmsa_on;
	unsigned char triga_on;
	struct shgrip_msa msa;
	struct shgrip_scmsa_val ch0;
	struct shgrip_scmsa_val ch2;
};

struct shgrip_scmsa_count {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scmsa2 {
	struct shgrip_scmsa_count count_a;
	struct shgrip_scmsa_count count_t;
};

struct shgrip_threshold4_reg {
	unsigned char th4_on;
	struct shgrip_nhys nhys4_ch0;
	struct shgrip_nhys nhys4_ch2;
	struct shgrip_nthr nthr4_ch0;
	struct shgrip_nthr nthr4_ch2;
};

struct shgrip_sens_setting_params {
	struct shgrip_params_val ch0_val;
	struct shgrip_params_val ch2_val;
	struct shgrip_user_setting setting_val;
	struct shgrip_sampling_val smp;
	struct shgrip_drift_set drift_val;
	struct shgrip_drift_set2 drift2_val;
	unsigned char th3onset;
	struct shgrip_threshold3_reg th3_val;
	unsigned char thr3cancel_on;
	struct shgrip_thr3_cancelval thr3_cancelval;
	struct shgrip_uphosei uphosei_val;
	struct shgrip_scmsa scmsa_val;
	struct shgrip_scmsa2 scmsa2_val;
	struct shgrip_threshold4_reg th4_val;
};

struct shgrip_dbg_command {
	unsigned char addr;
	unsigned char w_size;
	unsigned char w_buf[32];
	unsigned char r_size;
	unsigned char r_buf[32];
};

#endif /* __SHGRIP_KERL_H__ */
