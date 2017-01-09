/* drivers/sharp/shdisp/shdisp_dbg.c  (Display Driver)
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

/*---------------------------------------------------------------------------*/
/* INCLUDE FILES                                                             */
/*---------------------------------------------------------------------------*/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/path.h>
#include <linux/namei.h>
#include <linux/time.h>
#include <asm/unwind.h>
#include <asm/stacktrace.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/sh_boot_manager.h>
#include "shdisp_dbg.h"

/*---------------------------------------------------------------------------*/
/* MACROS                                                                    */
/*---------------------------------------------------------------------------*/

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_LOG_ENABLE
    unsigned char shdisp_log_lv = SHDISP_LOG_LV_ERR;
#endif

#define SHDISP_DBG_DISPLAY_ERROR_FILE               ("/durable/display/displaylog.txt")

#define SHDISP_DBG_ERROR_LOG_ADD_NORMAL             (0)
#define SHDISP_DBG_ERROR_LOG_ADD_CYCLIC             (1)
#define SHDISP_DBG_ERROR_LOG_FILE_TITLE             ("Display Driver Log File\n")
#define SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE        (sizeof(SHDISP_DBG_ERROR_LOG_FILE_TITLE)-1)

#define SHDISP_DBG_ERROR_LOG_TITLE                  ("\n<Error Log>\n")
#define SHDISP_DBG_ERROR_LOG_HEADER                 ("No,  Date,            Time,     Timezone,   Date2,           Time2,      Mode,  Type,       Code,              Sub Code,          \n")
#define SHDISP_DBG_ERROR_LOG_BLANK                  ("                                                                                                                                  \n")
#define SHDISP_DBG_ERROR_LOG_NUM                    (100)

#define SHDISP_DBG_ERROR_LOG_TITLE_SIZE             (sizeof(SHDISP_DBG_ERROR_LOG_TITLE)-1)
#define SHDISP_DBG_ERROR_LOG_HEADER_SIZE            (sizeof(SHDISP_DBG_ERROR_LOG_HEADER)-1)
#define SHDISP_DBG_ERROR_LOG_LINE_SIZE              (sizeof(SHDISP_DBG_ERROR_LOG_BLANK)-1)
#define SHDISP_DBG_ERROR_LOG_TOP                    (SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE)

#define SHDISP_DBG_MODE_LINUX_STR                   "Linux, "
#define SHDISP_DBG_MODE_NoOS_STR                    "NoOS,  "

#define SHDISP_DBG_TYPE_CLMR_FW_STR                 "Clmr FW,    "
#define SHDISP_DBG_TYPE_CLMR_HW_STR                 "Clmr HW,    "
#define SHDISP_DBG_TYPE_PANEL_STR                   "Panel,      "
#define SHDISP_DBG_TYPE_BDIC_STR                    "BDIC,       "
#define SHDISP_DBG_TYPE_PSALS_STR                   "PSALS,      "

#define SHDISP_DBG_CODE_TIMEOUT_STR                 "Time Out,          "
#define SHDISP_DBG_CODE_HSERROR_STR                 "Handshake Error,   "
#define SHDISP_DBG_CODE_CMDERR_STR                  "Command Error,     "
#define SHDISP_DBG_CODE_UNEXPHINT_STR               "Unexpect HINT,     "
#define SHDISP_DBG_CODE_ASYNCERR_STR                "ASYNC Error,       "
#define SHDISP_DBG_CODE_RETRYOVER_STR               "Retry Over,        "
#define SHDISP_DBG_CODE_READERR_STR                 "Read Error,        "
#define SHDISP_DBG_CODE_ERRDET_STR                  "Error Detection,   "

#define SHDISP_DBG_SUBCODE_NONE_STR                 "---,               "
#define SHDISP_DBG_SUBCODE_COMMAND_STR              "Command,           "
#define SHDISP_DBG_SUBCODE_BOOT_STR                 "Boot,              "
#define SHDISP_DBG_SUBCODE_MIF_STR                  "MIF,               "
#define SHDISP_DBG_SUBCODE_NOTCOMP_STR              "Not Complete,      "
#define SHDISP_DBG_SUBCODE_MISMATCH_STR             "Number Mismatch,   "
#define SHDISP_DBG_SUBCODE_DEVCODE_STR              "Devcode,           "
#define SHDISP_DBG_SUBCODE_STATUS_STR               "Status,            "
#define SHDISP_DBG_SUBCODE_DISPNG_STR               "Disp ON NG,        "
#define SHDISP_DBG_SUBCODE_DETLOW_STR               "DET LOW,           "
#define SHDISP_DBG_SUBCODE_ESDDETIN_STR             "ESD DETIN,         "
#define SHDISP_DBG_SUBCODE_ESDMIPI_STR              "ESD MIPI,          "
#define SHDISP_DBG_SUBCODE_I2CREAD_STR              "I2C READ,          "
#define SHDISP_DBG_SUBCODE_I2CERR_STR               "I2C_ERR,           "
#define SHDISP_DBG_SUBCODE_PSREQ_STR                "PS_REQ,            "
#define SHDISP_DBG_SUBCODE_RECOVNG_STR              "Recovery NG,       "
#define SHDISP_DBG_SUBCODE_PSALSNG_STR              "PSALS ON NG,       "
#define SHDISP_DBG_SUBCODE_POWERNG_STR              "POWER ON NG,       "

#define SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE          ("\n<Error Log Summary>\n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER         ("Mode,  Type,       Code,              Sub Code,          Count \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_COMMAND_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE02         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_BOOT_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE03         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_MIF_STR      "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE04         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_HSERROR_STR   SHDISP_DBG_SUBCODE_NOTCOMP_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE05         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_HSERROR_STR   SHDISP_DBG_SUBCODE_MISMATCH_STR "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE06         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_CMDERR_STR    SHDISP_DBG_SUBCODE_NONE_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE07         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_UNEXPHINT_STR SHDISP_DBG_SUBCODE_COMMAND_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE08         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_UNEXPHINT_STR SHDISP_DBG_SUBCODE_BOOT_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE09         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_UNEXPHINT_STR SHDISP_DBG_SUBCODE_MIF_STR      "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE10         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_ASYNCERR_STR  SHDISP_DBG_SUBCODE_NONE_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE11         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_CLMR_HW_STR SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_DEVCODE_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE12         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_READERR_STR   SHDISP_DBG_SUBCODE_DEVCODE_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE13         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_READERR_STR   SHDISP_DBG_SUBCODE_STATUS_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE14         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_DISPNG_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE15         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_DETLOW_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE16         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_ESDDETIN_STR "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE17         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_ESDMIPI_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE18         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_DEVCODE_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE19         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_DISPNG_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE20         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_DETLOW_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE21         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_ESDDETIN_STR "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE22         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_ESDMIPI_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE23         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_BDIC_STR    SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_I2CREAD_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE24         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_I2CERR_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE25         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_PSREQ_STR    "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE26         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_RECOVNG_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE27         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_PSALSNG_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE28         (SHDISP_DBG_MODE_LINUX_STR SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_POWERNG_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE29         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_COMMAND_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE30         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_BOOT_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE31         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_TIMEOUT_STR   SHDISP_DBG_SUBCODE_MIF_STR      "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE32         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_HSERROR_STR   SHDISP_DBG_SUBCODE_NOTCOMP_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE33         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_HSERROR_STR   SHDISP_DBG_SUBCODE_MISMATCH_STR "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE34         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_CMDERR_STR    SHDISP_DBG_SUBCODE_NONE_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE35         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_FW_STR SHDISP_DBG_CODE_ASYNCERR_STR  SHDISP_DBG_SUBCODE_NONE_STR     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE36         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_CLMR_HW_STR SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_DEVCODE_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE37         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_READERR_STR   SHDISP_DBG_SUBCODE_DEVCODE_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE38         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_READERR_STR   SHDISP_DBG_SUBCODE_STATUS_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE39         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_PANEL_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_DISPNG_STR   "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE40         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_BDIC_STR    SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_I2CREAD_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE41         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_ERRDET_STR    SHDISP_DBG_SUBCODE_PSALSNG_STR  "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE42         (SHDISP_DBG_MODE_NoOS_STR  SHDISP_DBG_TYPE_PSALS_STR   SHDISP_DBG_CODE_RETRYOVER_STR SHDISP_DBG_SUBCODE_POWERNG_STR  "   0  \n")



#define SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE     (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE)-1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE    (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER)-1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE      (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01)-1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_COUNTER_POS    (SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE-7)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_TOP            (SHDISP_DBG_ERROR_LOG_TOP+SHDISP_DBG_ERROR_LOG_TITLE_SIZE+SHDISP_DBG_ERROR_LOG_HEADER_SIZE+(SHDISP_DBG_ERROR_LOG_LINE_SIZE*SHDISP_DBG_ERROR_LOG_NUM))

struct shdisp_dbg_dump_data_struct {
    char*   data_ptr;
    size_t  length;
};
#ifdef SHDISP_RESET_LOG
struct reset_log_quework {
    struct shdisp_dbg_error_code code;
    int reset;
    struct work_struct wq;
};
#endif /* SHDISP_RESET_LOG */
static struct shdisp_dbg_dump_data_struct shdisp_dbg_dump_dat[NUM_SHDISP_DBG];
static char printk_buf[LINE_BUF_SIZE];
static int log_buf_offset = 0;
static char log_buf[LOG_BUF_SIZE] = {0};
static int ring = 0;
#if defined (CONFIG_ANDROID_ENGINEERING)
static int fw_err_sysetem_reset = SHDISP_DBG_RESET_OFF;
static int shdisp_dbg_fail_retry_flg = SHDISP_DBG_FAIL_RETRY_ON;
#endif /* CONFIG_ANDROID_ENGINEERING */

#ifdef SHDISP_RESET_LOG
static struct workqueue_struct *shdisp_wq_dbg_reset_log = NULL;
#endif /* SHDISP_RESET_LOG */

const static const char* WeekOfDay[] = {
    "Sun"
   ,"Mon"
   ,"Tue"
   ,"Wed"
   ,"Thu"
   ,"Fri"
   ,"Sat"
};

const static const char* ModeIndex[] = {
    SHDISP_DBG_MODE_LINUX_STR
   ,SHDISP_DBG_MODE_NoOS_STR
};

const static const char* TypeIndex[] = {
    SHDISP_DBG_TYPE_CLMR_FW_STR
   ,SHDISP_DBG_TYPE_CLMR_HW_STR
   ,SHDISP_DBG_TYPE_PANEL_STR
   ,SHDISP_DBG_TYPE_BDIC_STR
   ,SHDISP_DBG_TYPE_PSALS_STR
};

const static const char* CodeIndex[] = {
    SHDISP_DBG_CODE_TIMEOUT_STR
   ,SHDISP_DBG_CODE_HSERROR_STR
   ,SHDISP_DBG_CODE_CMDERR_STR
   ,SHDISP_DBG_CODE_UNEXPHINT_STR
   ,SHDISP_DBG_CODE_ASYNCERR_STR
   ,SHDISP_DBG_CODE_RETRYOVER_STR
   ,SHDISP_DBG_CODE_READERR_STR
   ,SHDISP_DBG_CODE_ERRDET_STR
};

const static const char* SubCodeIndex[] = {
    SHDISP_DBG_SUBCODE_NONE_STR
   ,SHDISP_DBG_SUBCODE_COMMAND_STR
   ,SHDISP_DBG_SUBCODE_BOOT_STR
   ,SHDISP_DBG_SUBCODE_MIF_STR
   ,SHDISP_DBG_SUBCODE_NOTCOMP_STR
   ,SHDISP_DBG_SUBCODE_MISMATCH_STR
   ,SHDISP_DBG_SUBCODE_DEVCODE_STR
   ,SHDISP_DBG_SUBCODE_STATUS_STR
   ,SHDISP_DBG_SUBCODE_DISPNG_STR
   ,SHDISP_DBG_SUBCODE_DETLOW_STR
   ,SHDISP_DBG_SUBCODE_ESDDETIN_STR
   ,SHDISP_DBG_SUBCODE_ESDMIPI_STR
   ,SHDISP_DBG_SUBCODE_I2CREAD_STR
   ,SHDISP_DBG_SUBCODE_I2CERR_STR
   ,SHDISP_DBG_SUBCODE_PSREQ_STR
   ,SHDISP_DBG_SUBCODE_RECOVNG_STR
   ,SHDISP_DBG_SUBCODE_PSALSNG_STR
   ,SHDISP_DBG_SUBCODE_POWERNG_STR
};

const static struct shdisp_dbg_error_code ErrorCodeIndex[] = {
    {SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_COMMAND          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_BOOT             }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_MIF              }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_HANDSHAKE_ERROR,    SHDISP_DBG_SUBCODE_NOT_COMPLETE     }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_HANDSHAKE_ERROR,    SHDISP_DBG_SUBCODE_NUMBER_MISMATCH  }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_COMMAND_ERROR,      SHDISP_DBG_SUBCODE_NONE             }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_UNEXPECT_HINT,      SHDISP_DBG_SUBCODE_COMMAND          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_UNEXPECT_HINT,      SHDISP_DBG_SUBCODE_BOOT             }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_UNEXPECT_HINT,      SHDISP_DBG_SUBCODE_MIF              }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_ASYNC_ERROR,        SHDISP_DBG_SUBCODE_NONE             }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_CLMR_HW,    SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_DEVCODE          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,         SHDISP_DBG_SUBCODE_DEVCODE          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,         SHDISP_DBG_SUBCODE_STATUS           }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_DISPON_NG        }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_DET_LOW          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_ESD_DETIN        }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_ESD_MIPI         }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_DEVCODE          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_DISPON_NG        }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_DET_LOW          }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_ESD_DETIN        }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_ESD_MIPI         }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_BDIC,       SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_I2C_READ         }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_I2C_ERROR        }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_PS_REQ           }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_RECOVERY_NG      }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_PSALS_ON_NG      }
   ,{SHDISP_DBG_MODE_LINUX, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_POWER_ON_NG      }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_COMMAND          }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_BOOT             }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_TIMEOUT,            SHDISP_DBG_SUBCODE_MIF              }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_HANDSHAKE_ERROR,    SHDISP_DBG_SUBCODE_NOT_COMPLETE     }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_HANDSHAKE_ERROR,    SHDISP_DBG_SUBCODE_NUMBER_MISMATCH  }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_COMMAND_ERROR,      SHDISP_DBG_SUBCODE_NONE             }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_FW,    SHDISP_DBG_CODE_ASYNC_ERROR,        SHDISP_DBG_SUBCODE_NONE             }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_CLMR_HW,    SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_DEVCODE          }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,         SHDISP_DBG_SUBCODE_DEVCODE          }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,         SHDISP_DBG_SUBCODE_STATUS           }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_DISPON_NG        }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_BDIC,       SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_I2C_READ         }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,       SHDISP_DBG_SUBCODE_PSALS_ON_NG      }
   ,{SHDISP_DBG_MODE_NoOS,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_RETRY_OVER,         SHDISP_DBG_SUBCODE_POWER_ON_NG      }
};

const static const char* SummaryFormat[] = {
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE02
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE03
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE04
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE05
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE06
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE07
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE08
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE09
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE10
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE11
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE12
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE13
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE14
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE15
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE16
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE17
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE18
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE19
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE20
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE21
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE22
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE23
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE24
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE25
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE26
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE27
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE28
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE29
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE30
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE31
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE32
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE33
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE34
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE35
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE36
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE37
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE38
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE39
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE40
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE41
   ,SHDISP_DBG_ERROR_LOG_SUMMARY_LINE42
};

static int subcode_val = SHDISP_DBG_SUBCODE_NONE;


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_ringbuffer_dump(void);
static void shdisp_dbg_stacktrace_dump(void);
static int shdisp_vprintk(const char *fmt, va_list args, int offset);
static spinlock_t shdisp_dbg_spin_lock;
static int shdisp_dbg_lock_init_done = 0;

int shdisp_dbg_dump_create(unsigned short no, size_t len);
int shdisp_dbg_dump_set_length(unsigned short no, size_t len);
size_t shdisp_dbg_dump_get_length(unsigned short no);
char* shdisp_dbg_dump_get_ptr(unsigned short no);
int shdisp_dbg_dump_is_ok(unsigned short no);
size_t shdisp_dbg_get_stacktrace(char * stackdumpbuf, size_t length);
int shdisp_dbg_dump_finalize(unsigned short no);
struct shdisp_dbg_dump_operations shdisp_dbg_dump = {
    shdisp_dbg_dump_create,
    shdisp_dbg_dump_set_length,
    shdisp_dbg_dump_get_length,
    shdisp_dbg_dump_get_ptr,
    shdisp_dbg_dump_is_ok,
    shdisp_dbg_dump_finalize,
};
static ssize_t shdisp_dbg_kernel_write(struct file *fp, const char *buf, size_t size);
static ssize_t shdisp_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset);
static ssize_t shdisp_dbg_kernel_seek(struct file *fp, unsigned int offset);
static int shdisp_dbg_kernel_sync(struct file *fp);

static int shdisp_dbg_add_err_log_one_line(char *buf, struct shdisp_dbg_error_code* code);
static int shdisp_dbg_add_err_log_file(struct shdisp_dbg_error_code* code);
static int shdisp_dbg_summary_init_file(void);
static int shdisp_dbg_summary_countup_file(int idx);
static void shdisp_dbg_summary_countup_line(char *buf);
static int shdisp_dbg_countup_one(char *pCount);

#ifdef SHDISP_RESET_LOG
static void shdisp_workqueue_handler_dbg_reset_log(struct work_struct *w);
static int  shdisp_dbg_reset_log_output(struct shdisp_dbg_error_code* code, int reset);
#endif /* SHDISP_RESET_LOG */


/*---------------------------------------------------------------------------*/
/* FUNCTIONS                                                                 */
/*---------------------------------------------------------------------------*/
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_init                                                           */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_init(void)
{
    spin_lock_init(&shdisp_dbg_spin_lock);
    shdisp_dbg_lock_init_done = 1;

#ifdef SHDISP_RESET_LOG
    shdisp_wq_dbg_reset_log = create_singlethread_workqueue("shdisp_dbg_reset_log");
    if( !shdisp_wq_dbg_reset_log ) {
        SHDISP_DEBUG("shdisp_dbg_workqueue create failed.\n" );
    }
#endif /* SHDISP_RESET_LOG */
}

int shdisp_printk(const char *fmt, ...)
{
    unsigned long irq_flag, spin_flag;

    va_list args;
    int offset;

    if (!shdisp_dbg_lock_init_done) {
        return 0;
    }

    local_irq_save(irq_flag);
    spin_lock_irqsave(&shdisp_dbg_spin_lock, spin_flag);

    va_start(args, fmt);
    offset = log_buf_offset;
    offset = shdisp_vprintk(fmt, args, offset);
    log_buf_offset = offset;
    va_end(args);

    spin_unlock_irqrestore(&shdisp_dbg_spin_lock, spin_flag);
    local_irq_restore(irq_flag);
    return 0;
}

static int shdisp_vprintk(const char *fmt, va_list args, int offset)
{
    int len;
    int this_cpu;
    unsigned long long t;
    unsigned long nanosec_rem;

    this_cpu = smp_processor_id();

    /* Add the current time stamp */
    t = cpu_clock(this_cpu);
    nanosec_rem = do_div(t, 1000000000);
    len = sprintf(printk_buf, "[%5lu.%06lu] ",
            (unsigned long) t,
            nanosec_rem / 1000);

    len += vscnprintf(&printk_buf[len],
                     sizeof(printk_buf) - len, fmt, args);

    if( (offset + len) < LOG_BUF_SIZE) {
        memcpy(&log_buf[offset], printk_buf, len);
        offset = offset + len;
    }
    else {
        ring = 1;
        memcpy(&log_buf[offset], printk_buf, LOG_BUF_SIZE - offset);
        memcpy(&log_buf[0], &printk_buf[LOG_BUF_SIZE - offset], len - (LOG_BUF_SIZE - offset));
        offset = len - (LOG_BUF_SIZE - offset);
    }
    log_buf[offset] = '\0';
    offset++;

    return offset;
}


void shdisp_logdump(void)
{
    shdisp_dbg_ringbuffer_dump();
    shdisp_dbg_stacktrace_dump();
}

static void shdisp_dbg_stacktrace_dump(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    size_t len = 0;
    ret = DMP.create(SHDISP_DBG_STACKTRACE, LOG_BUF_SIZE_ST);
    if(ret != SHDISP_RESULT_SUCCESS)
        return;
    len = shdisp_dbg_get_stacktrace(
      DMP.get_ptr(SHDISP_DBG_STACKTRACE),
      DMP.get_length(SHDISP_DBG_STACKTRACE)
    );
    ret = DMP.set_length(SHDISP_DBG_STACKTRACE, len);
}
static void shdisp_dbg_ringbuffer_dump(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int l;
    int s;
    int len = 0;
    char *p;
    char *file_p = NULL;
    unsigned long irq_flag, spin_flag;
    if (!shdisp_dbg_lock_init_done) {
        return;
    }
    ret = DMP.create(SHDISP_DBG_RINGBUFFER, LOG_BUF_SIZE);
    if(ret == SHDISP_RESULT_SUCCESS)
    {
        file_p = DMP.get_ptr(SHDISP_DBG_RINGBUFFER);
    }

    local_irq_save(irq_flag);
    spin_lock_irqsave(&shdisp_dbg_spin_lock, spin_flag);

    if(ring == 0) {
        s = 0;
    }
    else {
        s = log_buf_offset;
    }

    printk(KERN_ERR "[SHDISP] LOG DUMP -------------------->\n");

    while (len < LOG_BUF_SIZE) {

        p = printk_buf;
        l = 0;

        while (s < LOG_BUF_SIZE && l < LINE_BUF_SIZE - 1) {

            if(log_buf[s] == '\0' ) break;

            p[l] = log_buf[s];
            s++;
            l++;
        }

        if( s >= LOG_BUF_SIZE) {

            s = 0;

            while (s < LOG_BUF_SIZE && l < LINE_BUF_SIZE - 1) {

                if(log_buf[s] == '\0' ) break;

                p[l] = log_buf[s];
                s++;
                l++;
            }
        }

        if(l > 0) {
            if(len > LOG_BUF_SIZE_TO_KERNEL) {
                p[l] = '\0';
                printk(KERN_ERR "%s", p);
            }
            if(file_p != NULL) {
                p[l] = '\n';
                memcpy(file_p+len, p, l+1);
            }
        }
        s++;
        len += (l + 1);

    }

    printk(KERN_ERR "<-------------------- [SHDISP] LOG DUMP\n");

    printk(KERN_ERR "[SHDISP] STACK DUMP -------------------->\n");
    dump_stack();
    printk(KERN_ERR "<-------------------- [SHDISP] STACK DUMP\n");

    spin_unlock_irqrestore(&shdisp_dbg_spin_lock, spin_flag);
    local_irq_restore(irq_flag);

    return;

}

int shdisp_dbg_dump_create(unsigned short no, size_t len)
{
    char* ptr;

    if(shdisp_dbg_dump_dat[no].data_ptr != NULL)
    {
        SHDISP_ERR("warning. dump heap not free. no=%d\n", no);
        kfree(shdisp_dbg_dump_dat[no].data_ptr);
    }

    ptr = kzalloc(len, GFP_KERNEL);
    if (ptr == NULL) {
        SHDISP_ERR("allocate dump heap error. [no memory] no=%d\n", no);
        shdisp_dbg_dump_dat[no].data_ptr = NULL;
        shdisp_dbg_dump_dat[no].length = 0;
        return -ENOMEM;
    }
    memset(ptr, 0x00, len);
    shdisp_dbg_dump_dat[no].data_ptr = ptr;
    shdisp_dbg_dump_dat[no].length = len;

    return SHDISP_RESULT_SUCCESS;
}
int shdisp_dbg_dump_set_length(unsigned short no, size_t len)
{
    shdisp_dbg_dump_dat[no].length = len;
    return SHDISP_RESULT_SUCCESS;
}
size_t shdisp_dbg_dump_get_length(unsigned short no)
{
    return shdisp_dbg_dump_dat[no].length;
}
char* shdisp_dbg_dump_get_ptr(unsigned short no)
{
    return shdisp_dbg_dump_dat[no].data_ptr;
}

int shdisp_dbg_dump_is_ok(unsigned short no)
{
    switch(no)
    {
        case SHDISP_DBG_RINGBUFFER:
            if(shdisp_dbg_dump_dat[no].length > LOG_BUF_SIZE)
            {
                SHDISP_ERR("ring buffer dump heap length is over. no=%d\n", no);
                return SHDISP_DBG_ERR_LENGTH_OVER;
            }
            break;
        case SHDISP_DBG_STACKTRACE:
            if(shdisp_dbg_dump_dat[no].length > LOG_BUF_SIZE_ST)
            {
                SHDISP_ERR("stack trace dump heap length is over. no=%d\n", no);
                return SHDISP_DBG_ERR_LENGTH_OVER;
            }
            break;
        default:
            SHDISP_ERR("unexpet no=%d\n", no);
            return SHDISP_DBG_ERR_UNEXPECT;
    }
    if(shdisp_dbg_dump_dat[no].data_ptr == NULL)
    {
        SHDISP_ERR("dump heap ptr is null. no=%d\n", no);
        return SHDISP_DBG_ERR_HEAP_NULL;
    }
    if(shdisp_dbg_dump_dat[no].length == 0)
    {
        SHDISP_ERR("dump heap length is 0. no=%d\n", no);
        return SHDISP_DBG_ERRL_LENGTH_ZERO;
    }
    return SHDISP_RESULT_SUCCESS;
}
int shdisp_dbg_dump_finalize(unsigned short no)
{
    if (shdisp_dbg_dump_dat[no].data_ptr){
        kfree(shdisp_dbg_dump_dat[no].data_ptr);
    }
    shdisp_dbg_dump_dat[no].data_ptr = NULL;
    shdisp_dbg_dump_dat[no].length = 0;
    return SHDISP_RESULT_SUCCESS;
}

size_t shdisp_dbg_get_stacktrace(char * stackdumpbuf, size_t length)
{
    size_t writelen = 0;
    struct stackframe frame = { 0 };
    register unsigned long current_sp asm ("sp");

    frame.fp = (unsigned long)__builtin_frame_address(0);
    frame.sp = current_sp;
    frame.lr = (unsigned long)__builtin_return_address(0);
    frame.pc = (unsigned long)shdisp_dbg_get_stacktrace;

   SHDISP_DEBUG("shdisp_dbg_get_stacktrace ptr=0x%p len=%d\n", stackdumpbuf, length);
    while (1) {
        int urc, rtn;

        rtn = snprintf( stackdumpbuf, length, "%pS\n", (void*)frame.pc );


        if( rtn < 0 ) {
            SHDISP_ERR("dump heap shortage error. \n");
            break;
        }

        urc = unwind_frame(&frame);
        length -= rtn;
        writelen += rtn;
        stackdumpbuf += rtn;

        if (urc != URC_OK){
             break;
        }
    }
    SHDISP_DEBUG("dump stuck trace size=%d\n", writelen);
    return writelen;
}

#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_dbg_set_fail_retry_flg(int flg)
{
    shdisp_dbg_fail_retry_flg = flg;
}
int  shdisp_dbg_get_fail_retry_flg(void)
{
    return shdisp_dbg_fail_retry_flg;
}

void shdisp_dbg_api_set_reset_flg(int sw)
{
    fw_err_sysetem_reset = sw;
}
int shdisp_dbg_api_get_reset_flg(void)
{
    return fw_err_sysetem_reset;
}
#endif /* defined (CONFIG_ANDROID_ENGINEERING) */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_write                                                   */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_write(struct file *fp, const char *buf, size_t size)
{
    mm_segment_t old_fs;
    ssize_t res = 0;

    if( buf == NULL ){
        SHDISP_ERR("<NULL_POINTER>\n");
        return res;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->write(fp, buf, size, &fp->f_pos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_read                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset)
{
    mm_segment_t old_fs;
    ssize_t res = 0;
    loff_t fpos = offset;

    if( buf == NULL ){
        SHDISP_ERR("<NULL_POINTER>\n");
        return res;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->read(fp, buf, size, &fpos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_seek                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_seek(struct file *fp, unsigned int offset)
{
    ssize_t res;
    loff_t fpos;

    fpos = offset;
    res = fp->f_op->llseek(fp, fpos, SEEK_SET);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_sync                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_kernel_sync(struct file *fp)
{
    int res;

    res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_api_add_err_log                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_api_add_err_log(struct shdisp_dbg_error_code* code)
{
    int ret;

    SHDISP_TRACE("in\n");

    if (code->mode    >= SHDISP_DBG_MODE_MAX ||
        code->type    >= SHDISP_DBG_TYPE_MAX ||
        code->code    >= SHDISP_DBG_CODE_MAX ||
        code->subcode >= SHDISP_DBG_SUBCODE_MAX){
        SHDISP_DEBUG("parameter error.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_dbg_add_err_log_file(code);
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_add_err_log_file                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_add_err_log_file(struct shdisp_dbg_error_code* code)
{
    struct path  path;
    struct file *fp;
    unsigned int offset;
    char *buf;
    int ret = -EINVAL;
    size_t size = (SHDISP_DBG_ERROR_LOG_LINE_SIZE * SHDISP_DBG_ERROR_LOG_NUM);

    SHDISP_TRACE("in\n");
    ret = kern_path(SHDISP_DBG_DISPLAY_ERROR_FILE, LOOKUP_OPEN, &path);
    if (ret != 0) {
        ret = shdisp_dbg_summary_init_file();
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
    }
    offset =  SHDISP_DBG_ERROR_LOG_TOP;
    offset += SHDISP_DBG_ERROR_LOG_TITLE_SIZE;
    offset += SHDISP_DBG_ERROR_LOG_HEADER_SIZE;

    buf = kzalloc((size + SHDISP_DBG_ERROR_LOG_LINE_SIZE), GFP_KERNEL);
    if (!buf) {
        SHDISP_ERR("allocate read buffer error. [no memory]\n");
        return -ENOMEM;
    }
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_RDWR, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        kfree(buf);
        SHDISP_ERR("Cannot open file: %s\n", SHDISP_DBG_DISPLAY_ERROR_FILE);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_dbg_kernel_read(fp, buf, size, offset);

    ret = shdisp_dbg_add_err_log_one_line(buf, code);
    shdisp_dbg_kernel_seek(fp, offset);

    if (ret == SHDISP_DBG_ERROR_LOG_ADD_NORMAL) {
        shdisp_dbg_kernel_write(fp, buf, size);
    }
    else {
        shdisp_dbg_kernel_write(fp, buf + SHDISP_DBG_ERROR_LOG_LINE_SIZE, size);
    }
    kfree(buf);
    shdisp_dbg_kernel_sync(fp);
    filp_close(fp, NULL);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_add_err_log_one_line                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_add_err_log_one_line(char *buf, struct shdisp_dbg_error_code* code)
{
    struct timeval tv;
    struct tm tm1, tm2;
    int i, blank_area = 0;
    char *bufwk;

    bufwk = buf;

    do_gettimeofday(&tv);
    time_to_tm((time_t)tv.tv_sec, 0, &tm1);
    time_to_tm((time_t)tv.tv_sec, (sys_tz.tz_minuteswest*60*(-1)), &tm2);

    for (i = 1; i <= SHDISP_DBG_ERROR_LOG_NUM; i++) {
        if (*bufwk == ' ') {
            blank_area = 1;
            break;
        }
        else {
            bufwk += SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        }
    }

    sprintf(bufwk, "%03d, %04d/%02d/%02d(%s), %02d:%02d:%02d, UTC +00h,   %04d/%02d/%02d(%s), %02d:%02d:%02d,   %s%s%s%s",
        ((blank_area == 1) ? i : SHDISP_DBG_ERROR_LOG_NUM),
        (int)(tm1.tm_year+1900), tm1.tm_mon + 1, tm1.tm_mday, WeekOfDay[tm1.tm_wday],
        tm1.tm_hour, tm1.tm_min, tm1.tm_sec,
        (int)(tm2.tm_year+1900), tm2.tm_mon + 1, tm2.tm_mday, WeekOfDay[tm2.tm_wday],
        tm2.tm_hour, tm2.tm_min, tm2.tm_sec,
        ModeIndex[code->mode],
        TypeIndex[code->type],
        CodeIndex[code->code],
        SubCodeIndex[code->subcode]);
    bufwk += (SHDISP_DBG_ERROR_LOG_LINE_SIZE - 1);
    if (*bufwk == 0x00) {
        *bufwk = 0x0a;
    }

    if (!blank_area) {
        bufwk = buf + SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        for (i = 1; i < SHDISP_DBG_ERROR_LOG_NUM; i++) {
            *(bufwk+0) = '0'+(i/100);
            *(bufwk+1) = '0'+((i%100)/10);
            *(bufwk+2) = '0'+(i%10);
            bufwk += SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        }
        return SHDISP_DBG_ERROR_LOG_ADD_CYCLIC;
    }
    return SHDISP_DBG_ERROR_LOG_ADD_NORMAL;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_api_err_countup                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_api_err_countup(struct shdisp_dbg_error_code* code)
{
    int idx = -1;
    int i, ret;

    SHDISP_DEBUG("in\n");
    for (i = 0; i < ARRAY_SIZE(ErrorCodeIndex); i++) {
        if (code->mode      == ErrorCodeIndex[i].mode &&
            code->type      == ErrorCodeIndex[i].type &&
            code->code      == ErrorCodeIndex[i].code &&
            code->subcode   == ErrorCodeIndex[i].subcode){
            idx = i;
            break;
        }
    }
    if (idx < 0){
        SHDISP_ERR("parameter not matched.mode=%d.type=%d.code=%d.subcode=%d.\n",
            code->mode,
            code->type,
            code->code,
            code->subcode);
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_dbg_summary_countup_file(idx);
    SHDISP_DEBUG("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_init_file                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_summary_init_file(void)
{
    struct file *fp;
    int i;

    SHDISP_TRACE("in\n");

    SHDISP_TRACE("open file: %s pid=%d tgid=%d comm=%s\n", SHDISP_DBG_DISPLAY_ERROR_FILE, current->pid, current->tgid, current->comm);
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        SHDISP_ERR("Cannot create file: %s err=%d pid=%d tgid=%d comm=%s\n", SHDISP_DBG_DISPLAY_ERROR_FILE, (int)fp, current->pid, current->tgid, current->comm);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_FILE_TITLE, SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_TITLE, SHDISP_DBG_ERROR_LOG_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_HEADER, SHDISP_DBG_ERROR_LOG_HEADER_SIZE);
    for (i = 0; i < SHDISP_DBG_ERROR_LOG_NUM; i++) {
        shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_BLANK, SHDISP_DBG_ERROR_LOG_LINE_SIZE);
    }

    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE, SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER, SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE);
    for (i = 0; i < ARRAY_SIZE(SummaryFormat); i++) {
        shdisp_dbg_kernel_write(fp, SummaryFormat[i], SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE);
    }

    shdisp_dbg_kernel_sync(fp);
    filp_close(fp, NULL);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_countup_file                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_summary_countup_file(int idx)
{
    struct path  path;
    struct file *fp;
    unsigned int offset;
    char *buf;
    int ret = -EINVAL;
    size_t size = SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE;

    SHDISP_DEBUG("in\n");
    ret = kern_path(SHDISP_DBG_DISPLAY_ERROR_FILE, LOOKUP_OPEN, &path);
    if (ret != 0) {
        ret = shdisp_dbg_summary_init_file();
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
    }
    offset =  SHDISP_DBG_ERROR_LOG_SUMMARY_TOP;
    offset += SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE;
    offset += SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE;
    offset += (idx * size);

    buf = kzalloc(size, GFP_KERNEL);
    if (!buf) {
        SHDISP_ERR("allocate read buffer error. [no memory]\n");
        return -ENOMEM;
    }
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_RDWR, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        kfree(buf);
        SHDISP_ERR("Cannot open file: %s err=%d pid=%d tgid=%d comm=%s\n", SHDISP_DBG_DISPLAY_ERROR_FILE, (int)fp, current->pid, current->tgid, current->comm);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_dbg_kernel_read(fp, buf, size, offset);
    shdisp_dbg_summary_countup_line(buf);
    shdisp_dbg_kernel_seek(fp, offset);
    shdisp_dbg_kernel_write(fp, buf, size);
    kfree(buf);
    shdisp_dbg_kernel_sync(fp);
    filp_close(fp, NULL);

    SHDISP_DEBUG("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_countup_line                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_summary_countup_line(char *buf)
{
    char *pCount1000, *pCount100, *pCount10, *pCount1;
    int carry = 0;

    pCount1000  = buf + SHDISP_DBG_ERROR_LOG_SUMMARY_COUNTER_POS;
    pCount100   = pCount1000 + 1;
    pCount10    = pCount100  + 1;
    pCount1     = pCount10   + 1;

    if (*pCount1000 == '9' && *pCount100 == '9' && *pCount10 == '9' && *pCount1 == '9') {
        SHDISP_DEBUG("Counter is Max Value %c%c%c%c\n", *pCount1000, *pCount100, *pCount10, *pCount1);
        return;
    }

    carry = shdisp_dbg_countup_one(pCount1);
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount10);
    }
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount100);
    }
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount1000);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_countup_one                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_countup_one(char *pCount)
{
    int carry = 0;

    if (*pCount >= '0' && *pCount <= '8') {
        *pCount += 0x01;
    }
    else if (*pCount == '9') {
        *pCount = '0';
        carry = 1;
    }
    else {
        *pCount = '1';
    }
    return carry;
}

int shdisp_dbg_set_subcode(int sub_code)
{
    subcode_val = sub_code;

    return 0;
}

int shdisp_dbg_get_subcode(void)
{
    return subcode_val;
}

#ifdef SHDISP_RESET_LOG
/*---------------------------------------------------------------------------*/
/*      shdisp_dbg_reset_log_output                                          */
/*---------------------------------------------------------------------------*/
static int shdisp_dbg_reset_log_output(struct shdisp_dbg_error_code* code, int reset) {
    struct reset_log_quework *wk = NULL;
    int error = -EINVAL;

    SHDISP_TRACE("called.\n");

    wk = (struct reset_log_quework*)kzalloc(sizeof(struct reset_log_quework), GFP_KERNEL);
    if (!wk) {
        SHDISP_ERR("allocate workqueue work error. [no memory]\n");
        error = -ENOMEM;
        goto errout;
    }

    INIT_WORK(&wk->wq, shdisp_workqueue_handler_dbg_reset_log);

    wk->code.mode = code->mode;
    wk->code.type = code->type;
    wk->code.code = code->code;
    wk->code.subcode = code->subcode;
    wk->reset     = reset;

    if (shdisp_wq_dbg_reset_log) {
        if (queue_work(shdisp_wq_dbg_reset_log, &wk->wq) == 0) {
            SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_dbg_reset_log_output. giveup.\n");
            goto errout;
        }
    }
    else {
        goto errout;
    }

    SHDISP_TRACE("normaly finished.\n");
    return 0;

errout:
    if (wk)  kfree(wk);

    SHDISP_ERR("abnormaly finished. error=%d\n", error);
    return error;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_workqueue_handler_dbg_reset_log                               */
/*---------------------------------------------------------------------------*/
static void shdisp_workqueue_handler_dbg_reset_log(struct work_struct *w)
{
    int ret;
    struct reset_log_quework *wk = NULL;

    SHDISP_TRACE("called.\n");

    if (!w) {
        SHDISP_ERR("work data is not exist.\n");
        return;
    }

    wk = container_of(w, struct reset_log_quework, wq);

    ret = shdisp_dbg_api_add_err_log(&wk->code);
    ret = shdisp_dbg_api_err_countup(&wk->code);
#if defined (CONFIG_ANDROID_ENGINEERING)
    if (wk->reset == 1) {
        BUG();
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (wk) {
        kfree(wk);
    }

    SHDISP_TRACE("done.\n");
}
#endif /* SHDISP_RESET_LOG */


/*---------------------------------------------------------------------------*/
/*    shdisp_dbg_i2bit                                                       */
/*---------------------------------------------------------------------------*/
int shdisp_dbg_i2bit(int val, char * buf, int len, unsigned int headbitpos, unsigned int tailbitpos)
{

    int ret = 0;
    unsigned int pos = headbitpos;

    if( !buf ){
        return ret;
    }

    if( (headbitpos < tailbitpos )
    ||  (headbitpos > 31 )
    ){
        return ret;
    }

    if( len < 1 + (headbitpos - tailbitpos) ){
        return ret;
    }

    while(1){
        int bmask = 1 << pos;
        *buf = (bmask & val) ? '1': '0';
        ret++;
        buf++;
        if( pos == tailbitpos ){
            *buf = '\0';
            break;
        }
        pos--;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_api_err_output                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_api_err_output(struct shdisp_dbg_error_code* code, int reset)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_DEBUG("in\n");
#ifdef SHDISP_RESET_LOG
    ret = shdisp_dbg_reset_log_output(code, reset);
#endif /* SHDISP_RESET_LOG */
    SHDISP_DEBUG("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

