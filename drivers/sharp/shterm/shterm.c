/* drivers/sharp/shterm/shterm.c
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

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include "sharp/shterm_k.h"
#ifdef CONFIG_SHSYS_CUST
#include "sharp/sh_smem.h"
#endif /* CONFIG_SHSYS_CUST */
#ifdef CONFIG_SHLOG_SYSTEM
#include "asm/cputype.h"
#endif


#define EVENT_NAME_MAX 20

typedef struct {
    unsigned char type;
    unsigned char vbat;
    unsigned char vchg;
    unsigned char ichg;
    unsigned char temp;
    unsigned char off_chg;
    unsigned char dev;
    unsigned char open;
    unsigned char ant;
    unsigned char vave;
    unsigned char iave;
    unsigned char curr;
    unsigned char acur;
    unsigned char vper;
    unsigned char dper;
} shbattlog_disp_type;

static const shbattlog_disp_type event_str[] =
    {{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {1,1,0,0,1,1,0,0,0,0,0,0,0,1,0},
     {3,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {0,1,0,1,1,1,1,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,1,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,1,1,1,1,1,1,1,1,1},
     {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
     {0,1,0,1,1,1,1,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,1,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,1,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,1,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,1,0,0,1,1,0,0,0,0,0,1,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,1,1,1,0,0,0,1,1,0,0,1,0},
     {0,1,0,0,1,1,0,0,0,0,0,0,0,1,0},
     {0,0,1,1,1,1,0,0,0,0,0,0,1,1,1},
     {0,1,0,1,1,1,1,0,0,0,0,1,0,1,0},
     {0,1,0,1,1,1,1,0,0,0,0,1,0,1,0},
     {0,1,0,1,1,1,1,0,0,0,0,1,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {2,1,0,0,1,1,2,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {3,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {1,1,0,0,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,0,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,0,1,1,1,1,1,1,1,1,1,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {1,1,0,1,1,1,0,0,0,0,0,0,0,1,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} shterm_data;

typedef struct {
    unsigned long int id;
    const char *name;
} shterm_id_tbl;

#define CREATE_ATTR_ROOT( idname )                                   \
    static struct kobj_attribute shterm_info_##idname##_attribute =  \
         __ATTR( idname, 0600, NULL, NULL );                         \

#define CREATE_ATTR_READ( idname )                                   \
    static struct kobj_attribute shterm_info_##idname##_attribute =  \
         __ATTR( idname, 0644, NULL, NULL );                         \

static struct kobject shterm_kobj;
static struct kset *shterm_kset;
static struct semaphore shterm_sem;
static struct semaphore shterm_flip_sem;

static int shterm_info[SHTERM_MAX];
static int shterm_info_read[SHTERM_MAX];
static int shterm_flip_status = SHTERM_FLIP_STATE_CLOSE;
static unsigned long int flip_counter = 0;
static int shterm_old_lmk_bit = 0;

static int onex_min = -1;
static int onex_max = -1;
static int onex_old = -1;
static int evdo_min = -1;
static int evdo_max = -1;
static int evdo_old = -1;
static int gsm_min = -1;
static int gsm_max = -1;
static int gsm_old = -1;
static int wcdma_min = -1;
static int wcdma_max = -1;
static int wcdma_old = -1;
static int lte_min = -1;
static int lte_max = -1;
static int lte_old = -1;
static int shterm_init = 0;

static shterm_id_tbl id_k_tbl[SHTERM_MAX] =
    {{ SHTERM_INFO_SPEAKER, "SHTERM_INFO_SPEAKER" },
     { SHTERM_INFO_VIB, "SHTERM_INFO_VIB" },
     { SHTERM_INFO_CAMERA, "SHTERM_INFO_CAMERA" },
     { SHTERM_INFO_LINE, "SHTERM_INFO_LINE" },
     { SHTERM_INFO_QTV, "SHTERM_INFO_QTV" },
     { SHTERM_INFO_DTB, "SHTERM_INFO_DTB" },
     { SHTERM_INFO_LCDPOW, "SHTERM_INFO_LCDPOW" },
     { SHTERM_INFO_BACKLIGHT, "SHTERM_INFO_BACKLIGHT" },
     { SHTERM_INFO_BLUETOOTH, "SHTERM_INFO_BLUETOOTH" },
     { SHTERM_INFO_MOBILE_LIGHT, "SHTERM_INFO_MOBILE_LIGHT" },
     { SHTERM_INFO_MUSIC, "SHTERM_INFO_MUSIC" },
     { SHTERM_INFO_LINE_RINGING, "SHTERM_INFO_LINE_RINGING" },
     { SHTERM_INFO_TMM, "SHTERM_INFO_TMM" },
     { SHTERM_INFO_WLAN_TXRX, "SHTERM_INFO_WLAN_TXRX" },
     { SHTERM_INFO_SPEAKER_LEV, "SHTERM_INFO_SPEAKER_LEV" },
     { SHTERM_INFO_BACKLIGHT_LEV, "SHTERM_INFO_BACKLIGHT_LEV" },
     { SHTERM_INFO_IR, "SHTERM_INFO_IR" },
     { SHTERM_INFO_SD, "SHTERM_INFO_SD" },
     { SHTERM_INFO_GBNAND, "SHTERM_INFO_GBNAND" },
     { SHTERM_INFO_USB, "SHTERM_INFO_USB" },
     { SHTERM_INFO_WTAP, "SHTERM_INFO_WTAP" },
     { SHTERM_INFO_GPS, "SHTERM_INFO_GPS" },
     { SHTERM_INFO_ACCELE, "SHTERM_INFO_ACCELE" },
     { SHTERM_INFO_COMPS, "SHTERM_INFO_COMPS" },
     { SHTERM_INFO_KEYBACKLIGHT, "SHTERM_INFO_KEYBACKLIGHT" },
     { SHTERM_INFO_CTSBACKLIGHT, "SHTERM_INFO_CTSBACKLIGHT" },
     { SHTERM_INFO_SUBCAMERA, "SHTERM_INFO_SUBCAMERA" },
     { SHTERM_INFO_LMK_S1, "SHTERM_INFO_LMK_S1" },
     { SHTERM_INFO_LMK_TH1, "SHTERM_INFO_LMK_TH1" },
     { SHTERM_INFO_LMK_TH2, "SHTERM_INFO_LMK_TH2" },
     { SHTERM_INFO_LMK_TH3, "SHTERM_INFO_LMK_TH3" },
     { SHTERM_INFO_GYRO, "SHTERM_INFO_GYRO" },
     { SHTERM_INFO_PEDOMETER, "SHTERM_INFO_PEDOMETER" },
     { SHTERM_INFO_MUSIC_TNL, "SHTERM_INFO_MUSIC_TNL" },
     { SHTERM_INFO_ALARM, "SHTERM_INFO_ALARM" },
     { SHTERM_INFO_GRIP, "SHTERM_INFO_GRIP" }};

CREATE_ATTR_ROOT( SHTERM_INFO_SPEAKER );
CREATE_ATTR_ROOT( SHTERM_INFO_VIB );
CREATE_ATTR_ROOT( SHTERM_INFO_CAMERA );
CREATE_ATTR_ROOT( SHTERM_INFO_LINE );
CREATE_ATTR_ROOT( SHTERM_INFO_QTV );
CREATE_ATTR_ROOT( SHTERM_INFO_DTB );
CREATE_ATTR_ROOT( SHTERM_INFO_LCDPOW );
CREATE_ATTR_ROOT( SHTERM_INFO_BACKLIGHT );
CREATE_ATTR_ROOT( SHTERM_INFO_BLUETOOTH );
CREATE_ATTR_ROOT( SHTERM_INFO_MOBILE_LIGHT );
CREATE_ATTR_ROOT( SHTERM_INFO_MUSIC );
CREATE_ATTR_ROOT( SHTERM_INFO_LINE_RINGING );
CREATE_ATTR_ROOT( SHTERM_INFO_TMM );
CREATE_ATTR_ROOT( SHTERM_INFO_WLAN_TXRX );
CREATE_ATTR_ROOT( SHTERM_INFO_SPEAKER_LEV );
CREATE_ATTR_ROOT( SHTERM_INFO_BACKLIGHT_LEV );
CREATE_ATTR_ROOT( SHTERM_INFO_IR );
CREATE_ATTR_ROOT( SHTERM_INFO_SD );
CREATE_ATTR_ROOT( SHTERM_INFO_GBNAND );
CREATE_ATTR_ROOT( SHTERM_INFO_USB );
CREATE_ATTR_ROOT( SHTERM_INFO_WTAP );
CREATE_ATTR_ROOT( SHTERM_INFO_GPS );
CREATE_ATTR_ROOT( SHTERM_INFO_ACCELE );
CREATE_ATTR_ROOT( SHTERM_FLIP_STATE );
CREATE_ATTR_ROOT( SHTERM_FLIP_COUNT );
CREATE_ATTR_ROOT( SHTERM_INFO_COMPS );
CREATE_ATTR_ROOT( SHTERM_INFO_KEYBACKLIGHT );
CREATE_ATTR_ROOT( SHTERM_INFO_CTSBACKLIGHT );
CREATE_ATTR_ROOT( SHTERM_INFO_SUBCAMERA );
CREATE_ATTR_ROOT( SHTERM_INFO_GYRO );
CREATE_ATTR_ROOT( SHTERM_INFO_PEDOMETER );
CREATE_ATTR_ROOT( ONEX );
CREATE_ATTR_ROOT( EVDO );
CREATE_ATTR_ROOT( GSM );
CREATE_ATTR_ROOT( WCDMA );
CREATE_ATTR_ROOT( LTE );
CREATE_ATTR_ROOT( MIN_ONEX );
CREATE_ATTR_ROOT( MAX_ONEX );
CREATE_ATTR_ROOT( MIN_EVDO );
CREATE_ATTR_ROOT( MAX_EVDO );
CREATE_ATTR_ROOT( MIN_GSM );
CREATE_ATTR_ROOT( MAX_GSM );
CREATE_ATTR_ROOT( MIN_WCDMA );
CREATE_ATTR_ROOT( MAX_WCDMA );
CREATE_ATTR_ROOT( MIN_LTE );
CREATE_ATTR_ROOT( MAX_LTE );
CREATE_ATTR_READ( SHTERM_INIT );
CREATE_ATTR_ROOT( SHTERM_INFO_LMK_S1 );
CREATE_ATTR_ROOT( SHTERM_INFO_LMK_TH1 );
CREATE_ATTR_ROOT( SHTERM_INFO_LMK_TH2 );
CREATE_ATTR_ROOT( SHTERM_INFO_LMK_TH3 );
CREATE_ATTR_ROOT( DRB );
CREATE_ATTR_ROOT( CPUID );
CREATE_ATTR_ROOT( MUSIC );
CREATE_ATTR_ROOT( SHTERM_INFO_MUSIC_TNL );
CREATE_ATTR_ROOT( SHTERM_INFO_ALARM );
CREATE_ATTR_ROOT( SHTERM_INFO_GRIP );

static struct attribute *shterm_default_attrs[] = {
    &shterm_info_SHTERM_INFO_SPEAKER_attribute.attr,
    &shterm_info_SHTERM_INFO_VIB_attribute.attr,
    &shterm_info_SHTERM_INFO_CAMERA_attribute.attr,
    &shterm_info_SHTERM_INFO_LINE_attribute.attr,
    &shterm_info_SHTERM_INFO_QTV_attribute.attr,
    &shterm_info_SHTERM_INFO_DTB_attribute.attr,
    &shterm_info_SHTERM_INFO_LCDPOW_attribute.attr,
    &shterm_info_SHTERM_INFO_BACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_BLUETOOTH_attribute.attr,
    &shterm_info_SHTERM_INFO_MOBILE_LIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_MUSIC_attribute.attr,
    &shterm_info_SHTERM_INFO_LINE_RINGING_attribute.attr,
    &shterm_info_SHTERM_INFO_TMM_attribute.attr,
    &shterm_info_SHTERM_INFO_WLAN_TXRX_attribute.attr,
    &shterm_info_SHTERM_INFO_SPEAKER_LEV_attribute.attr,
    &shterm_info_SHTERM_INFO_BACKLIGHT_LEV_attribute.attr,
    &shterm_info_SHTERM_INFO_IR_attribute.attr,
    &shterm_info_SHTERM_INFO_SD_attribute.attr,
    &shterm_info_SHTERM_INFO_GBNAND_attribute.attr,
    &shterm_info_SHTERM_INFO_USB_attribute.attr,
    &shterm_info_SHTERM_INFO_WTAP_attribute.attr,
    &shterm_info_SHTERM_INFO_GPS_attribute.attr,
    &shterm_info_SHTERM_INFO_ACCELE_attribute.attr,
    &shterm_info_SHTERM_INFO_COMPS_attribute.attr,
    &shterm_info_SHTERM_FLIP_STATE_attribute.attr,
    &shterm_info_SHTERM_FLIP_COUNT_attribute.attr,
    &shterm_info_SHTERM_INFO_KEYBACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_CTSBACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_SUBCAMERA_attribute.attr,
    &shterm_info_SHTERM_INFO_GYRO_attribute.attr,
    &shterm_info_SHTERM_INFO_PEDOMETER_attribute.attr,
    &shterm_info_ONEX_attribute.attr,
    &shterm_info_EVDO_attribute.attr,
    &shterm_info_GSM_attribute.attr,
    &shterm_info_WCDMA_attribute.attr,
    &shterm_info_LTE_attribute.attr,
    &shterm_info_MIN_ONEX_attribute.attr,
    &shterm_info_MAX_ONEX_attribute.attr,
    &shterm_info_MIN_EVDO_attribute.attr,
    &shterm_info_MAX_EVDO_attribute.attr,
    &shterm_info_MIN_GSM_attribute.attr,
    &shterm_info_MAX_GSM_attribute.attr,
    &shterm_info_MIN_WCDMA_attribute.attr,
    &shterm_info_MAX_WCDMA_attribute.attr,
    &shterm_info_MIN_LTE_attribute.attr,
    &shterm_info_MAX_LTE_attribute.attr,
    &shterm_info_SHTERM_INIT_attribute.attr,
    &shterm_info_SHTERM_INFO_LMK_S1_attribute.attr,
    &shterm_info_SHTERM_INFO_LMK_TH1_attribute.attr,
    &shterm_info_SHTERM_INFO_LMK_TH2_attribute.attr,
    &shterm_info_SHTERM_INFO_LMK_TH3_attribute.attr,
    &shterm_info_DRB_attribute.attr,
    &shterm_info_CPUID_attribute.attr,
    &shterm_info_MUSIC_attribute.attr,
    &shterm_info_SHTERM_INFO_MUSIC_TNL_attribute.attr,
    &shterm_info_SHTERM_INFO_ALARM_attribute.attr,
    &shterm_info_SHTERM_INFO_GRIP_attribute.attr,
    NULL,
};

static void shterm_release( struct kobject *kobj );
static ssize_t shterm_info_show( struct kobject *kobj, struct attribute *attr, char *buff );
static ssize_t shterm_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len );

static struct sysfs_ops shterm_sysfs_ops = {
    .show  = shterm_info_show,
    .store = shterm_info_store,
};

static struct kobj_type shterm_ktype = {
    .release = shterm_release,
    .sysfs_ops = &shterm_sysfs_ops,
    .default_attrs = shterm_default_attrs,
};

static shterm_data data = {
    .name  = "info",
    .kobj  = &shterm_kobj,
    .ktype = &shterm_ktype,
};

int shterm_get_music_info( void )
{
    if( shterm_info[SHTERM_INFO_MUSIC] || shterm_info[SHTERM_INFO_LINE] || shterm_info[SHTERM_INFO_MUSIC_TNL] ){
        return 1;
    }
    return 0;
}

int shterm_k_set_info( unsigned long int shterm_info_id, unsigned long int shterm_info_value )
{
    int lmk_bit = 0;

    if( shterm_info_id < 0 || shterm_info_id >= SHTERM_MAX ){
        return SHTERM_FAILURE;
    }

    shterm_info[shterm_info_id] = shterm_info_value;
    if( shterm_info_value ){
        shterm_info_read[shterm_info_id] = shterm_info_value;
    }

    if( shterm_info_id == SHTERM_INFO_LMK_S1 ){
        shterm_old_lmk_bit = 0;
        shterm_old_lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH1];
        shterm_old_lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH2] << 1;
        shterm_old_lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH3] << 2;
    }
    else if( shterm_info_id == SHTERM_INFO_LMK_TH3 ){
        lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH1];
        lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH2] << 1;
        lmk_bit |= shterm_info[SHTERM_INFO_LMK_TH3] << 2;
        if( shterm_old_lmk_bit == 0 ){
            shterm_info[SHTERM_INFO_LMK_TH1] = (lmk_bit >> 0) & 0x1;
            shterm_info[SHTERM_INFO_LMK_TH2] = (lmk_bit >> 1) & 0x1;
            shterm_info[SHTERM_INFO_LMK_TH3] = (lmk_bit >> 2) & 0x1;
        } else if( lmk_bit > shterm_old_lmk_bit ){
            shterm_info[SHTERM_INFO_LMK_TH1] = (shterm_old_lmk_bit >> 0) & 0x1;
            shterm_info[SHTERM_INFO_LMK_TH2] = (shterm_old_lmk_bit >> 1) & 0x1;
            shterm_info[SHTERM_INFO_LMK_TH3] = (shterm_old_lmk_bit >> 2) & 0x1;
        }
            shterm_info_read[SHTERM_INFO_LMK_TH1] = shterm_info[SHTERM_INFO_LMK_TH1];
            shterm_info_read[SHTERM_INFO_LMK_TH2] = shterm_info[SHTERM_INFO_LMK_TH2];
            shterm_info_read[SHTERM_INFO_LMK_TH3] = shterm_info[SHTERM_INFO_LMK_TH3];
    }

    return SHTERM_SUCCESS;
}

int shterm_k_set_event( shbattlog_info_t *info )
{
    char *envp[20];
    char e_num[EVENT_NAME_MAX];
    char e_chg_vol[EVENT_NAME_MAX];
    char e_chg_cur[EVENT_NAME_MAX];
    char e_latest_cur[EVENT_NAME_MAX];
    char e_bat_vol[EVENT_NAME_MAX];
    char e_bat_tmp[EVENT_NAME_MAX];
    char e_chg_tmp[EVENT_NAME_MAX];
    char e_cam_tmp[EVENT_NAME_MAX];
    char e_pmic_tmp[EVENT_NAME_MAX];
    char e_pa_tmp[EVENT_NAME_MAX];
    char e_avg_cur[EVENT_NAME_MAX];
    char e_avg_vol[EVENT_NAME_MAX];
    char e_acc_cur[EVENT_NAME_MAX];
    char e_vol_per[EVENT_NAME_MAX];
    char e_cur_dep_per[EVENT_NAME_MAX];
    char e_avg_dep_per[EVENT_NAME_MAX];
#ifdef CONFIG_SHTERM_EMERGENCY
    char e_tmp_cut[EVENT_NAME_MAX];
#endif /* CONFIG_SHTERM_EMERGENCY */
    int idx = 0;
    int ret;

    if( info->event_num >= SHBATTLOG_EVENT_QUEUE_FULL ){
        return SHTERM_FAILURE;
    }

    memset( e_num, 0x00, sizeof(e_num) );
    snprintf( e_num, EVENT_NAME_MAX - 1, "EVENT_NUM=%d", info->event_num );
    envp[idx++] = e_num;

    if( event_str[info->event_num].vbat ){
        memset( e_bat_vol, 0x00, sizeof(e_bat_vol) );
        snprintf( e_bat_vol, EVENT_NAME_MAX - 1, "BAT_VOL=%d", info->bat_vol );
        envp[idx++] = e_bat_vol;
    }

    if( event_str[info->event_num].vchg ){
        memset( e_chg_vol, 0x00, sizeof(e_chg_vol) );
        snprintf( e_chg_vol, EVENT_NAME_MAX - 1, "CHG_VOL=%d", info->chg_vol );
        envp[idx++] = e_chg_vol;
    }

    if( event_str[info->event_num].ichg ){
        memset( e_chg_cur, 0x00, sizeof(e_chg_cur) );
        snprintf( e_chg_cur, EVENT_NAME_MAX - 1, "CHG_CUR=%d", info->chg_cur );
        envp[idx++] = e_chg_cur;
    }

    if( event_str[info->event_num].temp ){
        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CPU_TEMP=%d", info->cpu_temp );
        envp[idx++] = e_chg_tmp;

        memset( e_cam_tmp, 0x00, sizeof(e_cam_tmp) );
        snprintf( e_cam_tmp, EVENT_NAME_MAX - 1, "CAM_TEMP=%d", info->cam_temp );
        envp[idx++] = e_cam_tmp;

        memset( e_pmic_tmp, 0x00, sizeof(e_pmic_tmp) );
        snprintf( e_pmic_tmp, EVENT_NAME_MAX - 1, "LCD_TEMP=%d", info->lcd_temp );
        envp[idx++] = e_pmic_tmp;

        memset( e_pa_tmp, 0x00, sizeof(e_pa_tmp) );
        snprintf( e_pa_tmp, EVENT_NAME_MAX - 1, "PA_TEMP=%d", info->pa_temp );
        envp[idx++] = e_pa_tmp;
    }

    if( event_str[info->event_num].vave ){
        memset( e_avg_vol, 0x00, sizeof(e_avg_vol) );
        snprintf( e_avg_vol, EVENT_NAME_MAX - 1, "AVG_VOL=%d", info->avg_vol );
        envp[idx++] = e_avg_vol;
    }

    if( event_str[info->event_num].iave ){
        memset( e_avg_cur, 0x00, sizeof(e_avg_cur) );
        snprintf( e_avg_cur, EVENT_NAME_MAX - 1, "AVG_CUR=%d", info->avg_cur );
        envp[idx++] = e_avg_cur;
    }

    if( event_str[info->event_num].curr ){
        memset( e_latest_cur, 0x00, sizeof(e_latest_cur) );
        snprintf( e_latest_cur, EVENT_NAME_MAX - 1, "LAST_CUR=%d", info->latest_cur );
        envp[idx++] = e_latest_cur;
    }

    if( event_str[info->event_num].acur ){
        memset( e_acc_cur, 0x00, sizeof(e_acc_cur) );
        snprintf( e_acc_cur, EVENT_NAME_MAX - 1, "ACC_CUR=%d", info->acc_cur );
        envp[idx++] = e_acc_cur;
    }

    if( event_str[info->event_num].acur ){
        memset( e_vol_per, 0x00, sizeof(e_vol_per) );
        snprintf( e_vol_per, EVENT_NAME_MAX - 1, "VOL_PER=%d", info->vol_per );
        envp[idx++] = e_vol_per;
    }

    if( event_str[info->event_num].dper ){
        memset( e_cur_dep_per, 0x00, sizeof(e_cur_dep_per) );
        snprintf( e_cur_dep_per, EVENT_NAME_MAX - 1, "C_DEP_PER=%d", info->cur_dep_per );
        envp[idx++] = e_cur_dep_per;

        memset( e_avg_dep_per, 0x00, sizeof(e_avg_dep_per) );
        snprintf( e_avg_dep_per, EVENT_NAME_MAX - 1, "A_DEP_PER=%d", info->avg_dep_per );
        envp[idx++] = e_avg_dep_per;
    }

    envp[idx++] = NULL;
    ret = kobject_uevent_env( data.kobj, KOBJ_CHANGE, envp );

    return ret;
}

int shterm_flip_status_set( int state )
{
    int ret = SHTERM_FAILURE;

    if( down_interruptible(&shterm_flip_sem) ){
        printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
        return -ERESTARTSYS;
    }

    if( SHTERM_FLIP_STATE_OPEN == state ){
        if( SHTERM_FLIP_STATE_CLOSE == shterm_flip_status &&
            flip_counter < 0xffffffff ){
            flip_counter++;
        }
        ret = SHTERM_SUCCESS;
        shterm_flip_status = state;
    }
    else if( SHTERM_FLIP_STATE_CLOSE == state ){
        ret = SHTERM_SUCCESS;
        shterm_flip_status = state;
    }

    up( &shterm_flip_sem );

    return ret;
}

static void shterm_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t shterm_info_show( struct kobject *kobj, struct attribute *attr, char *buff )
{
    int n, ret;

    if( !strncmp(attr->name, "MIN_ONEX", strlen("MIN_ONEX")) ){
        ret = sprintf( buff, "%d", onex_min );
        onex_min = onex_old;
    }
    else if( !strncmp(attr->name, "MAX_ONEX", strlen("MAX_ONEX")) ){
        ret = sprintf( buff, "%d", onex_max );
        onex_max = onex_old;
    }
    else if( !strncmp(attr->name, "MIN_EVDO", strlen("MIN_EVDO")) ){
        ret = sprintf( buff, "%d", evdo_min );
        evdo_min = evdo_old;
    }
    else if( !strncmp(attr->name, "MAX_EVDO", strlen("MAX_EVDO")) ){
        ret = sprintf( buff, "%d", evdo_max );
        evdo_max = evdo_old;
    }
    else if( !strncmp(attr->name, "MIN_GSM", strlen("MIN_GSM")) ){
        ret = sprintf( buff, "%d", gsm_min );
        gsm_min = gsm_old;
    }
    else if( !strncmp(attr->name, "MAX_GSM", strlen("MAX_GSM")) ){
        ret = sprintf( buff, "%d", gsm_max );
        gsm_max = gsm_old;
    }
    else if( !strncmp(attr->name, "MIN_WCDMA", strlen("MIN_WCDMA")) ){
        ret = sprintf( buff, "%d", wcdma_min );
        wcdma_min = wcdma_old;
    }
    else if( !strncmp(attr->name, "MAX_WCDMA", strlen("MAX_WCDMA")) ){
        ret = sprintf( buff, "%d", wcdma_max );
        wcdma_max = wcdma_old;
    }
    else if( !strncmp(attr->name, "MIN_LTE", strlen("MIN_LTE")) ){
        ret = sprintf( buff, "%d", lte_min );
        lte_min = lte_old;
    }
    else if( !strncmp(attr->name, "MAX_LTE", strlen("MAX_LTE")) ){
        ret = sprintf( buff, "%d", lte_max );
        lte_max = lte_old;
    }
    else if( !strncmp(attr->name, "SHTERM_INIT", strlen("SHTERM_INIT")) ){
        ret = sprintf( buff, "%d", shterm_init );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_COUNT", strlen("SHTERM_FLIP_COUNT")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        ret = sprintf( buff, "%lu", flip_counter );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_STATE", strlen("SHTERM_FLIP_STATE")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        ret = sprintf( buff, "%d", shterm_flip_status );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "DRB", strlen("DRB")) ){
        ret = 0;
    }
    else if( !strncmp(attr->name, "CPUID", strlen("CPUID")) ){
#ifdef CONFIG_SHLOG_SYSTEM
        ret = sprintf( buff, "%d", read_cpuid_id() );
#else
        ret = sprintf( buff, "%d", 0 );
#endif /* CONFIG_SHLOG_SYSTEM */
    }
    else if( !strncmp(attr->name, "MUSIC", strlen("MUSIC")) ){
        ret = sprintf( buff, "%d", shterm_info[SHTERM_INFO_MUSIC] || shterm_info[SHTERM_INFO_MUSIC_TNL] );
    }
    else {
        for( n = 0; n < SHTERM_MAX; n++ ){
            if( !strncmp(attr->name, id_k_tbl[n].name, strlen(attr->name)) ){
                break;
            }
            else if( n == SHTERM_MAX - 1 ){
                return -1;
            }
        }

        if( down_interruptible(&shterm_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }

        ret = sprintf( buff, "%d", shterm_info_read[n] );
        if( n == SHTERM_INFO_LMK_S1  ||
            n == SHTERM_INFO_LMK_TH1 ||
            n == SHTERM_INFO_LMK_TH2 ||
            n == SHTERM_INFO_LMK_TH3 ){
            shterm_info[n] = 0;
            shterm_old_lmk_bit = 0;
        }
        shterm_info_read[n] = shterm_info[n];

        up( &shterm_sem );
    }

    return ret;

}

static ssize_t shterm_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len )
{
    int n;
    int val;

    if( !strncmp(attr->name, "ONEX", strlen("ONEX")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < onex_min ){
            onex_min = val;
        }
        if( val > onex_max ){
            onex_max = val;
        }
        onex_old = val;
    }
    else if( !strncmp(attr->name, "EVDO", strlen("EVDO")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < evdo_min ){
            evdo_min = val;
        }
        if( val > evdo_max ){
            evdo_max = val;
        }
        evdo_old = val;
    }
    else if( !strncmp(attr->name, "GSM", strlen("GSM")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < gsm_min ){
            gsm_min = val;
        }
        if( val > gsm_max ){
            gsm_max = val;
        }
        gsm_old = val;
    }
    else if( !strncmp(attr->name, "WCDMA", strlen("WCDMA")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < wcdma_min ){
            wcdma_min = val;
        }
        if( val > wcdma_max ){
            wcdma_max = val;
        }
        wcdma_old = val;
    }
    else if( !strncmp(attr->name, "LTE", strlen("LTE")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < lte_min ){
            lte_min = val;
        }
        if( val > lte_max ){
            lte_max = val;
        }
        lte_old = val;
    }
    else if( !strncmp(attr->name, "SHTERM_INIT", strlen("SHTERM_INIT")) ){
        shterm_init = (int)simple_strtol( buff, (char **)NULL, 10 );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_STATE", strlen("SHTERM_FLIP_STATE")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        shterm_flip_status = simple_strtol( buff, (char **)NULL, 10 );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_COUNT", strlen("SHTERM_FLIP_COUNT")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        flip_counter = simple_strtoul( buff, (char **)NULL, 10 );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "MUSIC", strlen("MUSIC")) ){
    }
    else if( !strncmp(attr->name, "DRB", strlen("DRB")) ){
        shbattlog_info_t info = {0};
        info.event_num = SHBATTLOG_EVENT_DVM_REBOOT;
        shterm_k_set_event( &info );
    }
    else if( !strncmp(attr->name, "CPUID", strlen("CPUID")) ){
    }
    else {
        for( n = 0; n < SHTERM_MAX; n++ ){
            if( !strncmp(attr->name, id_k_tbl[n].name, strlen(attr->name)) ){
                break;
            }
            else if( n == SHTERM_MAX - 1 ){
                return -1;
            }
        }

        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        /* errno check */
        if( down_interruptible(&shterm_sem) ){
            printk( "%s down_interruptible for write failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        if( SHTERM_INFO_MUSIC == n ){
            if( val ){
                if( shterm_info[n] < 100 ){
                    shterm_info[n]++;
                    shterm_info_read[n] = val;
                }
            }
            else {
                if( shterm_info[n] > 0 ){
                    shterm_info[n]--;
                }
            }
        }
        else {
            shterm_info[n] = val;
            if( val ){
                shterm_info_read[n] = val;
            }
        }

        up( &shterm_sem );
    }

    return len;
}

static int __init shterm_kobject_init( void )
{
    int ret;

    /* Create a kset with the name of "shterm" */
    /* located under /sys/kernel/ */
    shterm_kset = kset_create_and_add( "shterm", NULL, kernel_kobj );
    if( !shterm_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = shterm_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }

    /* Allowing only 1 user r/w access to a file */
    sema_init( &shterm_sem, 1 );
    sema_init( &shterm_flip_sem, 1 );
    memset( shterm_info, 0x00, sizeof(shterm_info) );
    memset( shterm_info_read, 0x00, sizeof(shterm_info_read) );

    return ret;
}

static void __exit shterm_kobject_exit( void )
{
    kset_unregister( shterm_kset );
}

module_init(shterm_kobject_init);
module_exit(shterm_kobject_exit);
