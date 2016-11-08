/* drivers/sharp/mfc/mfc.h (MFC Common Header)
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

#ifndef MFC_H
#define MFC_H

/* DEBUG_LOG */
#if 0
#define DEBUG_MFC_DRV
#endif

#ifdef DEBUG_MFC_DRV
#define MFC_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[MFC][%s]" fmt "\n", __func__, ## args)
#else
#define MFC_DRV_DBG_LOG(fmt, args...)
#endif

/* ERROR_LOG */
#define MFC_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[MFC][%s]ERR " fmt "\n", __func__, ## args)

/* prototype */
struct poll_data
{
	wait_queue_head_t read_wait;
	int irq_handler_done;
	struct delayed_work work;
	int device_status;
	int read_error;
	int open_flag;
};

/* GPIO number */
#define D_PON_GPIO_NO			(117)
#define D_RFS_GPIO_NO			(77)
#define D_INT_GPIO_NO			(59)
#define D_INT_POLL_GPIO_NO		D_INT_GPIO_NO
#define D_HSEL_GPIO_NO			g_snfc_hsel_gpio_no
#define D_INTU_GPIO_NO			g_snfc_intu_gpio_no
#define D_VFEL_GPIO_NO			g_snfc_vfel_gpio_no
#define D_MVDD_GPIO_NO 			(144)
#define D_UART_TX_GPIO_NO		(27)
#define D_UART_RX_GPIO_NO		(28)

extern int g_snfc_hsel_gpio_no;
extern int g_snfc_intu_gpio_no;
extern int g_snfc_vfel_gpio_no;

/* request muliple RFS irq */
#define D_RFS_DEV_LOW			(0)
#define D_RFS_DEV_HIGH			(1)

#ifdef CONFIG_SHSNFC
#define D_RFS_IRQ_MAX_NUM		(2)
#else
#define D_RFS_IRQ_MAX_NUM		(1)
#endif /* CONFIG_SHSNFC */

typedef void (*rfs_irq_handler)(int);

int request_notify_rfs_irq(rfs_irq_handler on_irq);
void free_notify_rfs_irq(rfs_irq_handler on_irq);

/* get RFS value */
int get_rfs_value(void);

/* get CEN value */
int get_cen_value(void);

/* set PON value */
#define D_PON_DEV_LOW			(0)
#define D_PON_DEV_HIGH			(1)
int set_pon_value(int value);

/* RWS */
#define D_RWS_RW_ENABLE			(0)
#define D_RWS_RW_DISABLE		(1)

#ifdef CONFIG_SHSNFC
/* set HSEL value */
#define D_HSEL_DEV_LOW			(0)
#define D_HSEL_DEV_HIGH			(1)
int set_hsel_value(int value);

/* UART Collision Control */
enum
{
	SUCC_RETVAL_OK = 0,
	SUCC_RETVAL_BUSY = -1,
	SUCC_RETVAL_ABNORMAL = -2
};

enum
{
	SUCC_STATE_IDLE = 0,
	SUCC_STATE_AUTOPOLL,
	SUCC_STATE_NFC,
	SUCC_STATE_FELICA,
	SUCC_STATE_NUM
};

enum
{
	SUCC_EVENT_AUTOPOLL = 0,
	SUCC_EVENT_START_NFC,
	SUCC_EVENT_START_FELICA,
	SUCC_EVENT_END_NFC,
	SUCC_EVENT_END_FELICA,
	SUCC_EVENT_NUM
};

int succ_handle_event(int event, int flags);

/* NFC Available */
enum
{
	NFC_AVAILABLE_RFS			= 0x01,
	NFC_AVAILABLE_CEN			= 0x02,
	NFC_AVAILABLE_FELICA		= 0x04,
	NFC_AVAILABLE_RWS			= 0x08
};

#define AVAILABLE_TRUE			(1)
#define AVAILABLE_FALSE			(0)
void notify_nfc_avalable_change(int type, int status);

#endif /* CONFIG_SHSNFC */

/* MVDD */
unsigned int snfc_available(void);
unsigned int snfc_available_wake_up(void);

#endif /* MFC_H */

