/* drivers/sharp/shirda/shirda_ldisc.h (sharp IrDA driver)
 *
 * Copyright (C) 2011 - 2013 SHARP CORPORATION All rights reserved.
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
#ifndef __SHIRDA_LDISC__
#define __SHIRDA_LDISC__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <asm/atomic.h>

#include "sharp/irda_common.h"
#include "sharp/irda_kdrv_api.h"

#define SHIRDA_LDISC_DRIVER_NAME	"shirda"


#include <linux/interrupt.h>

struct shirda_ldisc_admin_t;

typedef struct {
	int state;
	int in_frame;

	__u8 *head;
	__u8 *data;

	int len;
	int truesize;
	__u16 fcs;

	shirda_stats stats;
} iobuff_t;

typedef enum {
	IRDA_KDRV_WU_EV_ISR_RECV,
	IRDA_KDRV_WU_EV_ISR_SEND,
	IRDA_KDRV_WU_EV_ISR_TIMEOUT,
	IRDA_KDRV_WU_EV_CLOSE,
	IRDA_KDRV_WU_EV_USR_CANCEL,
	IRDA_KDRV_WU_EV_MEMORY_FULL,
	IRDA_KDRV_WU_EV_RX_OVERFLOW,
	IRDA_KDRV_WU_EV_ENUM_MAX
} irda_kdrv_wakeup_event_enum;

#define IRDA_KDRV_WU_EV_QUE_MAX (5)
typedef struct {
	unsigned short			rp;
	unsigned short			wp;
	irda_kdrv_wakeup_event_enum	event[IRDA_KDRV_WU_EV_QUE_MAX];
}irda_kdrv_wakeup_que_struct;

struct ring_pointer {
	unsigned short			rp;
	unsigned short			wp;
};

enum shirda_ldisc_state{
	SHIRDA_STATE_IDLE,
	SHIRDA_STATE_READY,
	SHIRDA_STATE_RECEIVE,
	SHIRDA_STATE_SEND_WAIT,
	SHIRDA_STATE_SEND,
	SHIRDA_STATE_MEDIABUSY,
	SHIRDA_STATE_LP_READY,
	SHIRDA_STATE_LP_RWAIT,
	SHIRDA_STATE_LP_RECEIVE,
	SHIRDA_STATE_LP_SEND,
	SHIRDA_STATE_LP_WAITW,
	ENUM_STATE_END
};

#define MAX_CHUNK_SIZE (2050)
#define MAX_RX_BUFF_SIZE (MAX_CHUNK_SIZE+2)
struct data_chunk{
	long length;
	__u8 data[MAX_CHUNK_SIZE];
};

struct shirda_ldisc_mtt_keeper {
	ktime_t last_rx;
	int need_mtt_flag;
	struct semaphore sem;
};

struct shirda_ldisc_write_timekeeper {
	struct hrtimer tx_timer;
	atomic_t write_wakeup_flag;
	struct shirda_ldisc_admin_t *admin;
	ktime_t start;
	ktime_t end;
};

struct shirda_ldisc_wakeup_queue {
	spinlock_t lock;
	wait_queue_head_t		wait_queue;
	irda_kdrv_wakeup_que_struct	wakeup_queue;
};

struct shirda_ldisc_rx_queue {
	spinlock_t lock;
	struct ring_pointer rx_pointer;
	struct data_chunk *rx_buffer;
};

struct shirda_ldisc_admin_t {
	enum shirda_ldisc_state		state;
	struct semaphore		sem;

	spinlock_t			close_lock;

	atomic_t			rx_enable;

	struct shirda_ldisc_wakeup_queue wakeup_queue;

	int				recent_err;

	unsigned char			*tx_payload;
	int				nr_tx_payload;
	unsigned char			*tx_frame;
	int				nr_tx_frame;

	iobuff_t			rx_buf;
	struct shirda_ldisc_rx_queue	rx_queue;
	irda_qos_info			qos;

	struct shirda_ldisc_mtt_keeper	mtt_kp;
	struct shirda_ldisc_write_timekeeper tx_kp;

	int				media_busy;

	struct tasklet_struct		tlet;
};

#define SHIRDA_ADMIN(tty) ((struct shirda_ldisc_admin_t *)((tty)->disc_data))

int async_wrap_tty(struct tty_struct *tty, __u8 *tx_buff, int buffsize);
void async_unwrap_char_tty(struct tty_struct *tty,
			   iobuff_t *rx_buff, __u8 byte);

void shirda_async_bump(struct tty_struct *tty,iobuff_t *rx_buff);

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif
