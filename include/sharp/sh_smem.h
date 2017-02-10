/* include/sharp/sh_smem.h
 *
 * Copyright (C) 2011 Sharp Corporation
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

/* 
 * AMSS/LINUX common SH_SMEM structure definition
 * 
 * [The file path which must synchronize]
 *  adsp_proc/core/sharp/shsys/inc/sh_smem.h
 *  boot_images/core/sharp/shsys/inc/sh_smem.h
 *  LINUX/android/kernel/include/sharp/sh_smem.h
 *  modem_proc/core/sharp/shsys/inc/sh_smem.h
 */
typedef struct 
{
    unsigned long       shdisp_data_buf[38944];      /* Buffer for shdisp */
    unsigned char       shusb_softupdate_mode_flag;  /* softupdate mode flag */
    unsigned long       sh_filesystem_init;          /* file system innitialize flag */
    int                 sh_sleep_test_mode;          /* sleep test mode flag */
    unsigned char       shusb_qxdm_ena_flag;         /* QXDM enable flag */
    unsigned char       shusb_usb_charge_ena_flag;   /* USB charge enable flag */
    unsigned long       fota_boot_mode;              /* FOTA mode information */
    unsigned char       conf_clrvari[4];             /* Color Variations information */
    unsigned long       shdiag_FlagData;             /* shdiag flag information */
    unsigned short      shdiag_BootMode;             /* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;            /* shdiag FirstBoot information */
    unsigned char       shdiag_AdjChashe[16];        /* shdiag Adj chashe information */
    unsigned short      shdiag_TpsBaseLineTbl[1000]; /* Touch adjustment */
    unsigned char       sh_100hflg;                  /* 100 hours test flag */
    unsigned short      shdiag_proxadj[2];           /* Proximity sensor adjust */
    unsigned char       shdiag_fullchgflg;           /* Full charge FLG(F Only) */
    char                shdiag_debugflg;             /* Debug FLG */
    char                shdiag_factoryflg;           /* Factory FLG */
    unsigned long long  shsys_timestamp[32];         /* System Timestamp */
    unsigned long       sh_hw_revision;              /* hardware revision number */
    unsigned char       sh_hw_handset;               /* Handset FLG */
    unsigned long       sh_boot_mode;                /* power up mode information */
    unsigned long       sh_boot_key;                 /* key(s) ditected OSBL */
    unsigned long       sh_pwr_on_status;            /* power on status information from pmic */
    char                sh_sbl_version[8];           /* sbl Version */
    unsigned char       pImeiData[16];               /* W-CDMA Imei data */
    unsigned char       shdiag_tspdrv_acal_data[3];  /* Haptics LRA control IC AutoCalibration result */
    unsigned char       sh_pvs_flg;                  /* PVS flag */
    int                 shpwr_battery_present;       /* PWR:battery present */
    int                 shpwr_battery_voltage;       /* PWR:battery voltage */
    int                 shpwr_battery_temperature;   /* PWR:battery temperature */
    int                 shpwr_cable_status;          /* PWR:cable status */
    int                 shpwr_fuel_data[4];          /* PWR:fuel gauge correction value */
    int                 shpwr_vbat_data[4];          /* PWR:battery A/D converter  correction value */
    unsigned char       shpwr_batauthflg;            /* PWR:battery ID Authentication plug infomation */
    unsigned char       sh_camOtpData[10240];        /* Camera Production adjustment Data */
    unsigned char       shdiag_charge_th_high[8];    /* ChageLimitMax */
    unsigned char       shdiag_charge_th_low[8];     /* ChageLimitMin */
    unsigned char       shdarea_WlanMacAddress[6];   /* WLAN Mac Address */
    unsigned char       shtps_fwup_flag;             /* Touch panel firmware update flag */
    unsigned char       shdiag_rvcflg;               /* Condenser judge flag */
} sharp_smem_common_type;

#define SH_SMEM_COMMON_SIZE 256000


/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void );

/*=============================================================================

FUNCTION sh_smem_get_sleep_power_collapse_disabled_address

=============================================================================*/
unsigned long *sh_smem_get_sleep_power_collapse_disabled_address( void );

/*=============================================================================

FUNCTION sh_smem_get_100hflg

=============================================================================*/
unsigned char sh_smem_get_100hflg( void );

/*=============================================================================

FUNCTION sh_smem_get_softupdate_flg

=============================================================================*/
unsigned char sh_smem_get_softupdate_flg( void );

/*=============================================================================

FUNCTION sh_smem_get_fota_boot_mode

=============================================================================*/
unsigned long sh_smem_get_fota_boot_mode( void );

/*=============================================================================

FUNCTION sh_smem_get_pvs_flg

=============================================================================*/
unsigned char sh_smem_get_pvs_flg( void );

