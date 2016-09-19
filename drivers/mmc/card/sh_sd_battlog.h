#ifndef ___SH_SD_BATTLOG_H___
#define ___SH_SD_BATTLOG_H___

#ifdef CONFIG_MMC_SD_BATTLOG_CUST_SH

#include "sharp/shterm_k.h"

#define SHBATTLOG_EVENT_SD_DETECT_BASE (SHBATTLOG_EVENT_SD_DETECTED)
#define SHBATTLOG_EVENT_SD_ERROR_BASE  (SHBATTLOG_EVENT_SD_ERROR_UNKNOWN_UNKNOWN_RECOVERED)

/* SD Card Detection */
typedef enum {
	SD_DETECTED,
	SD_DETECT_FAILED,
	SD_PHY_REMOVED,
	SD_SOFT_REMOVED,

	SD_DETECT_MAX
} sh_mmc_batlog_detect;

/* note MMC -> SD in battlog */
typedef enum {
	MMC_ERROR_UNKNOWN,
	MMC_ERROR_READ,
	MMC_ERROR_WRITE,
	MMC_ERROR_SECURE,
	MMC_ERROR_MISC,

	MMC_ERROR_SDIO, /* ignore them for now */
	MMC_ERROR_CMD_MAX = MMC_ERROR_SDIO
} sh_mmc_batlog_err_cmd;

typedef enum {
	_UNKNOWN,
	_CMD_TIMEOUT,
	_DATA_TIMEOUT,
	_REQ_TIMEOUT,
	_CMD_CRC_ERROR,
	_DATA_CRC_ERROR,
	_OTHER_ERROR,

	_MMC_ERR_TYPE_MAX
} sh_mmc_batlog_err_type;

typedef enum {
	_RECOVERED,
	_RETRY_OUT,

	_MMC_ERR_RESULT_MAX
} sh_mmc_batlog_err_result;

int mmc_detection_status_check(struct mmc_host *host);
void mmc_post_detection(struct mmc_host *host, sh_mmc_batlog_detect detect);

void mmc_inc_sh_retry_hist(struct mmc_card *card, int count);
void mmc_inc_sh_retry_hist_out(struct mmc_card *card);

/* call from core.c */
void mmc_inc_retry_hist(struct mmc_host *host, struct mmc_command *cmd);
void mmc_inc_retry_hist_out(struct mmc_host *host, struct mmc_command *cmd);

void mmc_set_err_cmd_type(struct mmc_host *host,
                          struct mmc_command *cmd, sh_mmc_batlog_err_type type);
void mmc_set_post_err_result(struct mmc_host *host, sh_mmc_batlog_err_result result);

void mmc_post_dev_info(struct mmc_host *host);

#endif /* CONFIG_MMC_SD_BATTLOG_CUST_SH */

#endif /* ___SH_SD_BATTLOG_H___ */
