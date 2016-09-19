/* drivers/sharp/shirda/shirda_ldisc.c (sharp IrDA driver)
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/hardirq.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/termbits.h>
#include <asm/atomic.h>
#include <linux/string.h>

#include <linux/serial.h>
#include <linux/serial_core.h>

#include "shirda_ldisc.h"
#include "shirda_kdrv.h"

static void _shirda_ldisc_write_wakeup(unsigned long);

#include "sharp/irda_common.h"
#include "sharp/irda_kdrv_api.h"

#define SHIRDA_LDISC_VERSION "02.09.00"







#define MAX_TX_PAYLOAD 2050
static unsigned char tx_payload[MAX_TX_PAYLOAD];

#define MAXIMUM_LAP_FRAME_SIZE 4096
#define MAX_TX_BUFF_SIZE MAXIMUM_LAP_FRAME_SIZE
static unsigned char tx_buff[MAX_TX_BUFF_SIZE];

static unsigned char rx_buff[MAX_RX_BUFF_SIZE];

#define RX_WAIT_TIMEOUT (HZ/5)

static struct shirda_ldisc_admin_t ldisc_admin;

const irda_qos_info default_qos = {
	.baud_rate		= IRDA_BAUD_9600,
	.connection_address	= IRDA_KDRV_DEF_CA,
	.add_bof		= IRDA_KDRV_DEF_ABOF,
	.mtt			= IRDA_KDRV_DEF_MTT,
	.mpi			= IRDA_DRV_SET_MPI_MAX
};

static long ktime_to_nsec(ktime_t a)
{
	long ret = ((a).tv64 & 0xffffffff);
	return ret;
}















#define SHIRDA_TXREADY_WAIT 5000

#define SHIRDA_TXEMPTY_WAIT 5000


static unsigned long shirda_calculate_tx_time(
	irda_boud_enum baud_rate,
	unsigned long tx_frame_byte
)
{
	unsigned long ret,bp10ms;

	if (tx_frame_byte > MAXIMUM_LAP_FRAME_SIZE) {
		IRDALOG_WARNING(
			"given tx_frame_byte is too large data size: %lu."
			"tx_frame_byte is treated as 4096byte.",
			tx_frame_byte
		);
		tx_frame_byte = MAXIMUM_LAP_FRAME_SIZE;
	}

	switch (baud_rate){
	case IRDA_BAUD_9600:
		bp10ms = 96;
		break;
	case IRDA_BAUD_19200:
		bp10ms = 192;
		break;
	case IRDA_BAUD_38400:
		bp10ms = 384;
		break;
	case IRDA_BAUD_57600:
		bp10ms = 576;
		break;
	case IRDA_BAUD_115200:
		bp10ms = 1152;
		break;
	default:
		IRDALOG_ERROR(
			"given baud_rate is invalid value : %d."
			"baud_rate is treated as %d(9600bps).",
			baud_rate,
			IRDA_BAUD_9600
		);
		bp10ms = 96;
		break;
	}
	#define BIT_IN_A_CHAR 10
	ret = (tx_frame_byte * BIT_IN_A_CHAR * 10000lu / bp10ms) + 1;

	IRDALOG_INFO(
		"baud_rate=%lu,byte=%lu,tx_time=%lu\n",
		bp10ms*100,
		tx_frame_byte,
		ret
	);
	return ret;
}

static unsigned long shirda_calculate_tx_timeout(
	irda_boud_enum baud_rate,
	unsigned long tx_frame_byte
)
{
	#define MAX_TX_WAIT_TIMEOUT (HZ*3)
	unsigned long ret,bps;

	if (tx_frame_byte>4096) {
		IRDALOG_WARNING(
			"too large data size : %lu."
			"tx_frame_byte is treated as 4096byte.",
			tx_frame_byte
		);
		return MAX_TX_WAIT_TIMEOUT;
	}

	switch (baud_rate){
	case IRDA_BAUD_9600:
		bps = 9600;
		break;
	case IRDA_BAUD_19200:
		bps = 19200;
		break;
	case IRDA_BAUD_38400:
		bps = 38400;
		break;
	case IRDA_BAUD_57600:
		bps = 57600;
		break;
	case IRDA_BAUD_115200:
		bps = 115200;
		break;
	default:
		IRDALOG_ERROR(
			"given baud_rate is invalid value : %d."
			"baud_rate is treated as %d(9600bps).",
			baud_rate,
			IRDA_BAUD_9600
		);
		return MAX_TX_WAIT_TIMEOUT;
		break;
	}

	#define BIT_IN_A_CHAR 10
	#define REDUNDANCY_COFFICIENT 2
	#define MINIMUM_TIMEOUT 10
	ret = tx_frame_byte * BIT_IN_A_CHAR * REDUNDANCY_COFFICIENT * HZ / bps
	      + MINIMUM_TIMEOUT;

	IRDALOG_INFO("shirda_calculate_tx_timeout =%lu\n", ret);
	if (ret>MAX_TX_WAIT_TIMEOUT) {
		IRDALOG_WARNING(
			"calculated value is over max timeout =%lu  ==> %d\n",
			ret,
			MAX_TX_WAIT_TIMEOUT
		);
		ret = MAX_TX_WAIT_TIMEOUT;
	}
	return ret;
}


static void shirda_ldisc_init_mtt_keeper(
	struct shirda_ldisc_mtt_keeper *mtt_kp
)
{
	mtt_kp->last_rx = ktime_set(0,0);
	mtt_kp->need_mtt_flag = FALSE;
	sema_init(&mtt_kp->sem,1);
}

static void shirda_ldisc_update_mtt_keeper(
	struct shirda_ldisc_mtt_keeper *mtt_kp
)
{
	down(&mtt_kp->sem);
	mtt_kp->last_rx = ktime_get();
	mtt_kp->need_mtt_flag = TRUE;
	up(&mtt_kp->sem);
}

static void shirda_ldisc_wait_mtt(
	struct shirda_ldisc_mtt_keeper *mtt_kp,
	uint32 mtt
)
{
	ktime_t timeout;
	unsigned long sec, usec;

	if (mtt_kp->need_mtt_flag==FALSE || mtt==IRDA_DRV_SET_MTT_ZERO) {
		return;
	}

	usec = (mtt % 100000lu) * 10lu;
	sec = mtt / 100000lu;
	down(&mtt_kp->sem);
	timeout = ktime_add(mtt_kp->last_rx, ktime_set(sec, usec*1000));
	mtt_kp->need_mtt_flag = FALSE;
	up(&mtt_kp->sem);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&timeout, HRTIMER_MODE_ABS);
}

static void shirda_ldisc_init_wakeup_queue(
	struct shirda_ldisc_wakeup_queue *queue
)
{
	spin_lock_init(&queue->lock);
	init_waitqueue_head(&queue->wait_queue);
	queue->wakeup_queue.rp = queue->wakeup_queue.wp = 0;
}

static void shirda_ldisc_clear_wakeup_queue(
	struct shirda_ldisc_wakeup_queue *queue
)
{
	unsigned long flag;
	spin_lock_irqsave(&queue->lock, flag);
	queue->wakeup_queue.wp = queue->wakeup_queue.rp;
	spin_unlock_irqrestore(&queue->lock, flag);
}

static void shirda_ldisc_wakeup_que_increment_wp(
	irda_kdrv_wakeup_que_struct* p_wque
)
{
	p_wque->wp++;
	if (p_wque->wp >= IRDA_KDRV_WU_EV_QUE_MAX) {
		p_wque->wp = 0;
	}
}

static void shirda_ldisc_wakeup_event_enqueue_locked(
	struct shirda_ldisc_wakeup_queue *queue,
	irda_kdrv_wakeup_event_enum event
)
{
	unsigned short wp;
	wp = queue->wakeup_queue.wp;
	shirda_ldisc_wakeup_que_increment_wp(&queue->wakeup_queue);
	if (queue->wakeup_queue.wp == queue->wakeup_queue.rp) {
		IRDALOG_WARNING("wakeup que memory full\n");
		shirda_ldisc_wakeup_que_increment_wp(&queue->wakeup_queue);
		event = IRDA_KDRV_WU_EV_MEMORY_FULL;
	}
	queue->wakeup_queue.event[wp] = event;

	IRDALOG_INFO(
		"wakeup que rp=%#x wp=%#x event=%#x\n",
		queue->wakeup_queue.rp,queue->wakeup_queue.wp,
		(int)event
	);

	wake_up_interruptible(&queue->wait_queue);
}

static void shirda_ldisc_wakeup_event_enqueue(
	struct shirda_ldisc_wakeup_queue *queue,
	irda_kdrv_wakeup_event_enum event
)
{
	unsigned long flag;
	spin_lock_irqsave(&queue->lock, flag);
	shirda_ldisc_wakeup_event_enqueue_locked(queue, event);
	spin_unlock_irqrestore(&queue->lock, flag);
}

static irda_kdrv_wakeup_event_enum shirda_ldisc_wakeup_event_dequeue_locked(
	struct shirda_ldisc_wakeup_queue *queue
)
{
	irda_kdrv_wakeup_event_enum ret;
	ret = queue->wakeup_queue.event[queue->wakeup_queue.rp];
	if (queue->wakeup_queue.rp != queue->wakeup_queue.wp) {
		queue->wakeup_queue.rp += 1;
		if (queue->wakeup_queue.rp >= IRDA_KDRV_WU_EV_QUE_MAX)
			queue->wakeup_queue.rp = 0;
	}
	return ret;
}

static irda_kdrv_wakeup_event_enum shirda_ldisc_wakeup_event_dequeue(
	struct shirda_ldisc_wakeup_queue *queue
)
{
	irda_kdrv_wakeup_event_enum ret;
	unsigned long flag;
	spin_lock_irqsave(&queue->lock, flag);
	ret = shirda_ldisc_wakeup_event_dequeue_locked(queue);
	spin_unlock_irqrestore(&queue->lock, flag);
	return ret;
}

static int shirda_ldisc_wait_event(
	struct shirda_ldisc_wakeup_queue *queue
)
{
	int iret = 0;
	IRDALOG_INFO(
		"rp=%d,wp=%d",
		queue->wakeup_queue.rp,
		queue->wakeup_queue.wp
	);

	iret = wait_event_interruptible(
		queue->wait_queue,
		(queue->wakeup_queue.rp != queue->wakeup_queue.wp)
	);
	if( iret < 0 ) {
		IRDALOG_ERROR("wait_event err=%d\n",iret);
	}
	return iret;
}

static long shirda_ldisc_wait_event_timeout(
	struct shirda_ldisc_wakeup_queue *queue,
	unsigned long timeout
)
{
	long remained;
	IRDALOG_INFO(
		"rp=%d,wp=%d",
		queue->wakeup_queue.rp,
		queue->wakeup_queue.wp
	);

	remained = wait_event_interruptible_timeout(
		queue->wait_queue,
		(queue->wakeup_queue.rp != queue->wakeup_queue.wp),
		timeout
	);

	if (remained == 0) {
		shirda_ldisc_wakeup_event_enqueue(
			queue,
			IRDA_KDRV_WU_EV_ISR_TIMEOUT
		);
	} else if( remained < 0 ) {
		IRDALOG_ERROR("wait_event err=%ld\n",remained);
	}
	return remained;
}

static enum hrtimer_restart shirda_ldisc_write_timekeeper_handler(
	struct hrtimer *timer
)
{
	struct shirda_ldisc_write_timekeeper *kp = container_of(
		timer,
		struct shirda_ldisc_write_timekeeper,
		tx_timer
	);

	kp->end = ktime_get();
	shirda_ldisc_wakeup_event_enqueue(
		&kp->admin->wakeup_queue,
		IRDA_KDRV_WU_EV_ISR_SEND
	);
	return HRTIMER_NORESTART;
}

static void shirda_ldisc_write_timekeeper_init(
	struct shirda_ldisc_write_timekeeper *kp
)
{
	kp->admin = container_of(kp, struct shirda_ldisc_admin_t, tx_kp);
	hrtimer_init(&kp->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kp->tx_timer.function = shirda_ldisc_write_timekeeper_handler;
	atomic_set(&kp->write_wakeup_flag, FALSE);
}

static void shirda_ldisc_write_timekeeper_write_wakeup(
	struct shirda_ldisc_write_timekeeper *kp
)
{
	atomic_set(&kp->write_wakeup_flag, TRUE);
}

static void shirda_ldisc_write_timekeeper_start(
	struct shirda_ldisc_write_timekeeper *kp,
	unsigned long send_time
)
{
	unsigned long sec,usec;
	usec = (send_time % 1000000lu);
	sec = (send_time - usec) / 1000000lu;
	kp->start = ktime_get();
	hrtimer_start(
		&kp->tx_timer,
		ktime_set(sec, usec*1000lu),
		HRTIMER_MODE_REL
	);
}

static int shirda_ldisc_write_timekeeper_wait(
	struct shirda_ldisc_write_timekeeper *kp,
	ktime_t now
)
{
	int repeat = 100;
	int ret = 0;
	ktime_t sleep;
	sleep = ktime_set(0,100000);

	IRDALOG_INFO(
		"start=%llu, end=%llu, now=%llu",
		kp->start.tv64,
		kp->end.tv64,
		now.tv64
	);

	while (atomic_read(&kp->write_wakeup_flag)==FALSE && repeat>0) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);
		repeat--;
	}

	if (atomic_read(&kp->write_wakeup_flag)==FALSE) {
		IRDALOG_WARNING(
			"Interrupt is not found.\n"
		);
	} else {
		long usec = ktime_to_nsec(ktime_sub(ktime_get(), now)) / 1000;
		if (usec > SHIRDA_TXREADY_WAIT) {
			IRDALOG_WARNING("Interrupt is too late %ld\n", usec);
		} else {
			IRDALOG_INFO("found Interrupt count=%d",repeat);
		}
	}

	return ret;
}

static int shirda_ldisc_write_wakeup_wait(
	struct shirda_ldisc_write_timekeeper *kp
)
{
	int repeat = 1000;
	int ret = 0;
	ktime_t now, sleep;
	now = ktime_get();
	sleep = ktime_set(0,1000000);

	IRDALOG_INFO(
		"start=%llu, end=%llu, now=%llu",
		kp->start.tv64,
		kp->end.tv64,
		now.tv64
	);

	while (atomic_read(&kp->write_wakeup_flag)==FALSE && repeat>0) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);
		repeat--;
	}

	if (atomic_read(&kp->write_wakeup_flag)==FALSE) {
		IRDALOG_WARNING(
			" write-wakeup is not found\n"
		);
	} else {
		IRDALOG_INFO("found write-wakeup count=%d",repeat);
	}

	return ret;
}

static int shirda_ldisc_write_txempty_wait(struct tty_struct *tty,
	ktime_t now)
{
	struct uart_state *state;
	unsigned int tx_empty = 0;
	ktime_t sleep;
	int repeat = 200;
	int ret = 0;

	state = (struct uart_state *)tty->driver_data;
	sleep = ktime_set(0,100000);

	tx_empty = state->uart_port->ops->tx_empty(state->uart_port);

	while ((tx_empty == 0) && (repeat > 0)) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);

		tx_empty = state->uart_port->ops->tx_empty(state->uart_port);

		repeat--;
	}


	if (repeat <= 0) {
		IRDALOG_ERROR("TX EMPTY wait over.\n");
		ret = -EIO;
	} else {
		long usec = ktime_to_nsec(ktime_sub(ktime_get(), now)) / 1000;
		if (usec > SHIRDA_TXEMPTY_WAIT) {
			IRDALOG_WARNING(" tx-empty is too late %ld\n", usec);
		} else {
			IRDALOG_INFO("found! count=%d",repeat);
		}
	}
	return ret;
}

#define MAX_RX_CHUNK_NUMBER 3
static struct data_chunk rx_queue[MAX_RX_CHUNK_NUMBER];

#define RX_QUE_SUCCESS 0
#define RX_QUE_OVERFLOW_ERROR -1
#define RX_QUE_COPY_ERROR -2

static void shirda_ldisc_rx_queue_init(struct shirda_ldisc_rx_queue *que)
{
	spin_lock_init(&que->lock);
	que->rx_pointer.rp = que->rx_pointer.wp = 0;
	que->rx_buffer = rx_queue;
}

static void shirda_ldisc_rx_queue_clear(struct shirda_ldisc_rx_queue *que)
{
	unsigned long flag;
	spin_lock_irqsave(&que->lock, flag);
	que->rx_pointer.rp = que->rx_pointer.wp = 0;
	spin_unlock_irqrestore(&que->lock, flag);
}

static int shirda_ldisc_rx_queue_enqueue(
	iobuff_t *rx_buff,
	struct shirda_ldisc_rx_queue *que
)
{
	int ret = RX_QUE_OVERFLOW_ERROR;
	unsigned long flag;

	spin_lock_irqsave(&que->lock, flag);
		if (rx_buff->len-2 < 0) {
			IRDALOG_ERROR("recieve lenght is invalid.");
		}
		que->rx_buffer[que->rx_pointer.wp].length = rx_buff->len - 2;
		memcpy(
			que->rx_buffer[que->rx_pointer.wp].data,
			rx_buff->data,
			rx_buff->len - 2
		);
		if (que->rx_pointer.wp+1 >= MAX_RX_CHUNK_NUMBER) {
			que->rx_pointer.wp = 0;
		} else {
			que->rx_pointer.wp++;
		}

		if (que->rx_pointer.wp == que->rx_pointer.rp) {
			IRDALOG_WARNING("rx buffer overflow.\n");
			ret = RX_QUE_OVERFLOW_ERROR;
		} else {
			ret = RX_QUE_SUCCESS;
		}
	spin_unlock_irqrestore(&que->lock, flag);

	return ret;
}

static ssize_t shirda_ldisc_rx_queue_dequeue_to_user(
	unsigned char __user *buf,
	struct shirda_ldisc_rx_queue *que
)
{
	int ret, err;
	unsigned long flag;

	spin_lock_irqsave(&que->lock, flag);
		ret = que->rx_buffer[que->rx_pointer.rp].length;

		err = copy_to_user(
			buf,
			que->rx_buffer[que->rx_pointer.rp].data,
			que->rx_buffer[que->rx_pointer.rp].length
		);
		if (err != 0) {
			ret = RX_QUE_COPY_ERROR;
		}

		if (que->rx_pointer.rp+1 >= MAX_RX_CHUNK_NUMBER) {
			que->rx_pointer.rp = 0;
		} else {
			que->rx_pointer.rp++;
		}
	spin_unlock_irqrestore(&que->lock, flag);

	return ret;
}

static void shirda_ldisc_disable_rx(struct tty_struct *tty)
{
	atomic_set(&SHIRDA_ADMIN(tty)->rx_enable,FALSE);
}

static void shirda_ldisc_enable_rx(struct tty_struct *tty)
{
	atomic_set(&SHIRDA_ADMIN(tty)->rx_enable,TRUE);
}

static int shirda_ldisc_check_rx_enabled(struct tty_struct *tty)
{
	return atomic_read(&SHIRDA_ADMIN(tty)->rx_enable);
}

static int shirda_ldisc_check_qos(const irda_qos_info *set_qos_info)
{
	if (set_qos_info->baud_rate < IRDA_BAUD_9600 ||
				set_qos_info->baud_rate > IRDA_BAUD_115200) {
		IRDALOG_ERROR("baud_rate err\n");
		return FALSE;
	}

	switch (set_qos_info->add_bof) {
	case IRDA_DRV_SET_ADD_BOF_48:
	case IRDA_DRV_SET_ADD_BOF_32:
	case IRDA_DRV_SET_ADD_BOF_24:
	case IRDA_DRV_SET_ADD_BOF_20:
	case IRDA_DRV_SET_ADD_BOF_16:
	case IRDA_DRV_SET_ADD_BOF_14:
	case IRDA_DRV_SET_ADD_BOF_12:
	case IRDA_DRV_SET_ADD_BOF_10:
	case IRDA_DRV_SET_ADD_BOF_8:
	case IRDA_DRV_SET_ADD_BOF_6:
	case IRDA_DRV_SET_ADD_BOF_5:
	case IRDA_DRV_SET_ADD_BOF_4:
	case IRDA_DRV_SET_ADD_BOF_3:
	case IRDA_DRV_SET_ADD_BOF_2:
	case IRDA_DRV_SET_ADD_BOF_1:
	case IRDA_DRV_SET_ADD_BOF_0:
		break;
	default:
		IRDALOG_ERROR("add_bof err\n");
		return FALSE;
		break;
	}

	if ((set_qos_info->mtt < IRDA_DRV_SET_MTT_MIN ||
				set_qos_info->mtt > IRDA_DRV_SET_MTT_MAX) &&
				set_qos_info->mtt != IRDA_DRV_SET_MTT_ZERO) {
		IRDALOG_ERROR("mtt err\n");
		return FALSE;
	}

	IRDALOG_INFO("qos OK\n");
	return TRUE;
}

static int shirda_ldisc_set_qos_trunk(
	struct tty_struct *tty,
	const irda_qos_info *next_qos
)
{
	speed_t baud_rate;
	struct ktermios old_termios;

	if (!shirda_ldisc_check_qos(next_qos)) {
		return -EINVAL;
	}

	switch (next_qos->baud_rate) {
	case IRDA_BAUD_9600:
		baud_rate = 9600;
		break;
	case IRDA_BAUD_19200:
		baud_rate = 19200;
		break;
	case IRDA_BAUD_38400:
		baud_rate = 38400;
		break;
	case IRDA_BAUD_57600:
		baud_rate = 57600;
		break;
	case IRDA_BAUD_115200:
		baud_rate = 115200;
		break;
	default:
		IRDALOG_ERROR("invalid baudrate\n");
		return -EINVAL;
		break;
	}

	mutex_lock(&tty->termios_mutex);
	memcpy(&(SHIRDA_ADMIN(tty)->qos), next_qos, sizeof(irda_qos_info));
	old_termios = *(tty->termios);
	if (test_and_clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags)) {
		IRDALOG_WARNING("TTY_DO_WRITE_WAKEUP is cleared.\n");
	}
	tty->ops->flush_buffer(tty);
	tty_encode_baud_rate(tty,baud_rate,baud_rate);
	tty->ops->set_termios(tty,&old_termios);
	mutex_unlock(&tty->termios_mutex);

	return 0;
}

static int shirda_ldisc_open(struct tty_struct *tty)
{
	struct shirda_ldisc_admin_t *admin = &ldisc_admin;

	IRDALOG_INFO("open\n");


	down(&admin->sem);
		if (admin->state!=SHIRDA_STATE_IDLE) {
			IRDALOG_ERROR("ldisc is already opened.\n");
			up(&admin->sem);
			return -EIO;
		}

		tty->disc_data = admin;
		admin->state = SHIRDA_STATE_READY;
	up(&admin->sem);

	shirda_ldisc_init_wakeup_queue(&admin->wakeup_queue);

	admin->recent_err = IRDA_LDISC_NO_ERR;
	admin->tx_payload = tx_payload;
	admin->nr_tx_payload = 0;
	admin->tx_frame = tx_buff;
	admin->nr_tx_frame = 0;

	admin->rx_buf.state = 0;
	admin->rx_buf.in_frame = 0;
	admin->rx_buf.head = rx_buff;
	admin->rx_buf.data = rx_buff;
	admin->rx_buf.fcs = 0;
	admin->rx_buf.len = 0;
	admin->rx_buf.truesize = MAX_RX_BUFF_SIZE;

	admin->media_busy = IRDA_LDISC_MEDIA_FREE;

	shirda_ldisc_rx_queue_init(&admin->rx_queue);
	shirda_ldisc_init_mtt_keeper(&admin->mtt_kp);
	shirda_ldisc_write_timekeeper_init(&admin->tx_kp);
	shirda_ldisc_set_qos_trunk(tty,&default_qos);

	memset(&(admin->rx_buf.stats), 0x00, sizeof(shirda_stats));

	#ifndef TTY_MAX_RECEIVE_ROOM
	#define TTY_MAX_RECEIVE_ROOM 65536
	#endif
	tty->receive_room = TTY_MAX_RECEIVE_ROOM;
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	tty_driver_flush_buffer(tty);
	shirda_ldisc_enable_rx(tty);

	tasklet_init(&admin->tlet,
		_shirda_ldisc_write_wakeup, (unsigned long)tty);



	if (!((struct uart_state *)tty->driver_data)->uart_port->ops->tx_empty) {
		IRDALOG_FATAL("tx_empty is not registered to uart_port_ops.");
		return -EBADFD;
	}
	if (!tty->ops->set_termios) {
		IRDALOG_FATAL("set_termios is not registered to tty_ops.");
		return -EBADFD;
	}
	if (!tty->ops->write) {
		IRDALOG_FATAL("write is not registered to tty_ops.");
		return -EBADFD;
	}
	if (!tty->ops->chars_in_buffer) {
		IRDALOG_FATAL("chars_in_buffer is not registered to tty_ops.");
		return -EBADFD;
	}

	return 0;
}

static void shirda_ldisc_close(struct tty_struct *tty)
{
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	shirda_ldisc_wakeup_event_enqueue(
		&SHIRDA_ADMIN(tty)->wakeup_queue,
		IRDA_KDRV_WU_EV_CLOSE
	);

	tasklet_kill(&admin->tlet);
	down(&admin->sem);
		admin->state = SHIRDA_STATE_IDLE;
	up(&admin->sem);
}

static ssize_t shirda_ldisc_read_READY(
	struct tty_struct *tty,
	struct file *file,
	unsigned char __user *buf,
	size_t nr
)
{
	int ret = 0;
	int iret;
	int event;
	int exit_flag = FALSE;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	down(&admin->sem);
	admin->state = SHIRDA_STATE_RECEIVE;
	up(&admin->sem);

	IRDALOG_INFO("start waiting event\n");
	while (exit_flag == FALSE) {
		iret = shirda_ldisc_wait_event(&admin->wakeup_queue);

		if (iret < 0) {
			IRDALOG_ERROR(" wait_event iret=%d\n",iret);
			ret = iret;
			break;
		}

		down(&admin->sem);
			event = shirda_ldisc_wakeup_event_dequeue(
				&admin->wakeup_queue
			);

			switch (event){
			case IRDA_KDRV_WU_EV_ISR_RECV:
				IRDALOG_INFO("done receive.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_NO_ERR;
				ret = shirda_ldisc_rx_queue_dequeue_to_user(
					buf,
					&admin->rx_queue
				);

				if (ret == RX_QUE_COPY_ERROR) {
					ret = -EFAULT;
				}

				break;

			case IRDA_KDRV_WU_EV_ISR_SEND:
				IRDALOG_ERROR("unexpected event:%d.\n",event);
				exit_flag = FALSE;
				break;

			case IRDA_KDRV_WU_EV_MEMORY_FULL:
				IRDALOG_ERROR("wakeup queue overflow.\n");
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				exit_flag = TRUE;
				break;

			case IRDA_KDRV_WU_EV_RX_OVERFLOW:
				admin->state = SHIRDA_STATE_READY;
				exit_flag = TRUE;
				break;

			case IRDA_KDRV_WU_EV_USR_CANCEL:
				IRDALOG_INFO("user cansel.\n");
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_READ_CANCELED;
				ret = -EAGAIN;
				exit_flag = TRUE;
				break;

			case IRDA_KDRV_WU_EV_CLOSE:
				IRDALOG_INFO("close.\n");
				admin->state = SHIRDA_STATE_IDLE;
				admin->recent_err = IRDA_LDISC_CLOSED;
				ret = -EIO;
				exit_flag = TRUE;
				break;

			default:
				IRDALOG_ERROR("unknown event.\n");
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				exit_flag = TRUE;
				break;
			}
		up(&admin->sem);
	}

	return ret;
}

static ssize_t shirda_ldisc_read_LP_READY(
	struct tty_struct *tty,
	struct file *file,
	unsigned char __user *buf,
	size_t nr
)
{
	int ret = 0;
	int event;
	int exit_flag = FALSE;
	long remained = RX_WAIT_TIMEOUT;

	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	down(&admin->sem);
		admin->state = SHIRDA_STATE_LP_RECEIVE;
	up(&admin->sem);

	while (exit_flag == FALSE) {
		remained = shirda_ldisc_wait_event_timeout(
			&admin->wakeup_queue,
			remained
		);

		down(&admin->sem);
			event = shirda_ldisc_wakeup_event_dequeue(
				&admin->wakeup_queue
			);

			switch (event) {
			case IRDA_KDRV_WU_EV_ISR_RECV:
				IRDALOG_INFO("done receive.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_NO_ERR;

				ret = shirda_ldisc_rx_queue_dequeue_to_user(
					buf,
					&admin->rx_queue
				);

				if (ret == RX_QUE_COPY_ERROR) {
					ret = -EFAULT;
				}

				break;

			case IRDA_KDRV_WU_EV_USR_CANCEL:
			case IRDA_KDRV_WU_EV_ISR_SEND:
				exit_flag = FALSE;
				IRDALOG_ERROR("unexpected event:%d.\n",event);
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				break;

			case IRDA_KDRV_WU_EV_ISR_TIMEOUT:
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_RX_TIMEOUT;
				ret = -ETIME;
				break;

			case IRDA_KDRV_WU_EV_MEMORY_FULL:
				IRDALOG_ERROR("wakeup queue overflow.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;

			case IRDA_KDRV_WU_EV_RX_OVERFLOW:
				admin->state = SHIRDA_STATE_READY;
				exit_flag = TRUE;
				break;

			case IRDA_KDRV_WU_EV_CLOSE:
				IRDALOG_INFO("close.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_IDLE;
				admin->recent_err = IRDA_LDISC_CLOSED;
				ret = -EIO;
				break;

			default:
				IRDALOG_ERROR("unknown event.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;
			}
		up(&admin->sem);
	}

	return ret;
}

static ssize_t shirda_ldisc_read_LP_RWAIT(
	struct tty_struct *tty,
	struct file *file,
	unsigned char __user *buf,
	size_t nr
)
{
	int ret = 0;
	int event;
	long remained = RX_WAIT_TIMEOUT;
	int exit_flag = FALSE;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	down(&admin->sem);
	admin->state = SHIRDA_STATE_LP_RECEIVE;
	up(&admin->sem);

	 while (exit_flag == FALSE) {
		remained = shirda_ldisc_wait_event_timeout(
			&admin->wakeup_queue,
			remained
		);

		down(&admin->sem);
			event = shirda_ldisc_wakeup_event_dequeue(
				&admin->wakeup_queue
			);

			switch (event) {
			case IRDA_KDRV_WU_EV_ISR_RECV:
				IRDALOG_INFO("done receive.\n");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_NO_ERR;

				ret = shirda_ldisc_rx_queue_dequeue_to_user(
					buf,
					&admin->rx_queue
				);

				if (ret == RX_QUE_COPY_ERROR) {
					ret = -EFAULT;
				}

				break;

			case IRDA_KDRV_WU_EV_USR_CANCEL:
			case IRDA_KDRV_WU_EV_ISR_SEND:
				exit_flag = FALSE;
				IRDALOG_ERROR("unexpected event:%d.",event);
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				break;

			case IRDA_KDRV_WU_EV_ISR_TIMEOUT:
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_RX_TIMEOUT;
				ret = -ETIME;
				break;

			case IRDA_KDRV_WU_EV_MEMORY_FULL:
				IRDALOG_ERROR("wakeup queue overflow");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;

			case IRDA_KDRV_WU_EV_RX_OVERFLOW:
				admin->state = SHIRDA_STATE_READY;
				exit_flag = TRUE;
				break;

			case IRDA_KDRV_WU_EV_CLOSE:
				IRDALOG_INFO("close");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_IDLE;
				admin->recent_err = IRDA_LDISC_CLOSED;
				ret = -EIO;
				break;

			default:
				IRDALOG_ERROR("unknown event.");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;
			}
		up(&admin->sem);
	}

	return ret;
}

static ssize_t shirda_ldisc_read(struct tty_struct *tty, struct file *file,
	                            unsigned char __user *buf, size_t nr)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	if (in_interrupt()) {
		return -EIO;
	}

	switch (admin->state) {
	case SHIRDA_STATE_IDLE:
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR("logical error.\n");
		break;

	case SHIRDA_STATE_READY:
		ret = shirda_ldisc_read_READY(tty,file,buf,nr);
		break;

	case SHIRDA_STATE_LP_READY:
		ret = shirda_ldisc_read_LP_READY(tty,file,buf,nr);
		break;

	case SHIRDA_STATE_LP_RWAIT:
		ret = shirda_ldisc_read_LP_RWAIT(tty,file,buf,nr);
		break;

	case SHIRDA_STATE_SEND_WAIT:
	case SHIRDA_STATE_SEND:
	case SHIRDA_STATE_LP_RECEIVE:
	case SHIRDA_STATE_LP_SEND:
	case SHIRDA_STATE_RECEIVE:
	case SHIRDA_STATE_MEDIABUSY:
	case SHIRDA_STATE_LP_WAITW:
		admin->recent_err = IRDA_LDISC_PERMISSION_ERR;
		ret = -EPERM;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t shirda_ldisc_write_READY(
	struct tty_struct *tty,
	struct file *file,
	const unsigned char *buf,
	size_t nr
)
{
	int ret = 0;
	int iret;
	long  lret;
	unsigned long wait_time,sent_byte,send_time;
	int event;
	int exit_flag = FALSE;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);
	ktime_t now;


	down(&admin->sem);
		admin->state = SHIRDA_STATE_SEND;
		memcpy(admin->tx_payload, buf, nr);
		admin->nr_tx_payload = nr;
		admin->nr_tx_frame = async_wrap_tty(
			tty,
			admin->tx_frame,
			MAX_TX_BUFF_SIZE
		);
		#define MINIMUM_LAP_FRAME_SIZE 5
		if (admin->nr_tx_frame < 0) {
			admin->state = SHIRDA_STATE_READY;
			IRDALOG_ERROR("tx buffer overflow");
			up(&admin->sem);
			return -ENOMEM;
		} else if (admin->nr_tx_frame == 0) {
			admin->state = SHIRDA_STATE_READY;
			IRDALOG_WARNING("tx datasize == 0 byte");
			up(&admin->sem);
			return 0;
		} else if (0< admin->nr_tx_frame
			&& admin->nr_tx_frame < MINIMUM_LAP_FRAME_SIZE){
			IRDALOG_WARNING(
				"unexpected datasize %d byte",
				admin->nr_tx_frame
			);
		}
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		shirda_ldisc_disable_rx(tty);
		shirda_ldisc_clear_wakeup_queue(&admin->wakeup_queue);
		shirda_ldisc_rx_queue_clear(&admin->rx_queue);
	up(&admin->sem);

	shirda_ldisc_wait_mtt(&admin->mtt_kp, admin->qos.mtt);

	sent_byte = admin->nr_tx_frame;
	send_time = shirda_calculate_tx_time(admin->qos.baud_rate, sent_byte);
	wait_time = shirda_calculate_tx_timeout(
		admin->qos.baud_rate,
		sent_byte
	);
	atomic_set(&admin->tx_kp.write_wakeup_flag, FALSE);
	ret = tty->ops->write(
		tty,
		admin->tx_frame,
		admin->nr_tx_frame);

	shirda_ldisc_write_timekeeper_start(&admin->tx_kp, send_time);
	if (ret < sent_byte) {
		IRDALOG_ERROR(
			"fatal error: writing is uncomplete."
			"not all bytes are written."
		);
	}
	if (ret > nr) {
		ret = nr;
	}

	IRDALOG_INFO("start waiting event");

	while (exit_flag == FALSE) {
		lret = shirda_ldisc_wait_event_timeout(
			&admin->wakeup_queue,
			wait_time
		);
		if (lret < 0) {
			IRDALOG_ERROR("write wait_event lret=%ld\n",lret);
			ret = lret;
			shirda_ldisc_write_wakeup_wait(&admin->tx_kp);
			break;
		}

		wait_time = (unsigned long) lret;
		down(&admin->sem);
			event = shirda_ldisc_wakeup_event_dequeue(
				&admin->wakeup_queue
			);
			switch (event) {
			case IRDA_KDRV_WU_EV_ISR_RECV:
				IRDALOG_WARNING("unexpected event :%d", event);
				shirda_ldisc_rx_queue_clear(&admin->rx_queue);
				exit_flag = FALSE;
				break;

			case IRDA_KDRV_WU_EV_ISR_SEND:
				shirda_ldisc_enable_rx(tty);
				now = ktime_get();
				iret = shirda_ldisc_write_timekeeper_wait(
					&admin->tx_kp, now
				);
				if (iret == 0) {
					iret =
					shirda_ldisc_write_txempty_wait(tty,
						now);
				}
				if (iret < 0) {
					exit_flag = TRUE;
					IRDALOG_ERROR("send error.");
					admin->state = SHIRDA_STATE_READY;
					admin->recent_err =
						IRDA_LDISC_TX_SEND_ERR;
					ret = -EIO;
				} else {
					exit_flag = TRUE;
					IRDALOG_INFO("done sending.");
					admin->state = SHIRDA_STATE_READY;
					admin->recent_err = IRDA_LDISC_NO_ERR;
				}
				break;

			case IRDA_KDRV_WU_EV_ISR_TIMEOUT:
				IRDALOG_INFO("sending timeout.");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_TX_TIMEOUT;
				ret = -ETIME;
				break;

			case IRDA_KDRV_WU_EV_MEMORY_FULL:
				IRDALOG_ERROR("wakeup queue overflow");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;

			case IRDA_KDRV_WU_EV_USR_CANCEL:
				IRDALOG_ERROR("unexpected event:%d.",event);
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				shirda_ldisc_write_wakeup_wait(&admin->tx_kp);
				break;

			case IRDA_KDRV_WU_EV_CLOSE:
				IRDALOG_INFO("close");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_IDLE;
				admin->recent_err = IRDA_LDISC_CLOSED;
				ret = -EIO;
				shirda_ldisc_write_wakeup_wait(&admin->tx_kp);
				break;

			default:
				IRDALOG_ERROR("unknown event :%d", event);
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				shirda_ldisc_write_wakeup_wait(&admin->tx_kp);
				break;
			}
		up(&admin->sem);
	}

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	shirda_ldisc_enable_rx(tty);

	return ret;
}

static ssize_t shirda_ldisc_write_LP_READY(
	struct tty_struct *tty,
	struct file *file,
	const unsigned char *buf,
	size_t nr
)
{
	int ret = 0;
	unsigned long wait_time, sent_byte, send_time;
	int event;
	int exit_flag = FALSE;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	down(&admin->sem);
		admin->state = SHIRDA_STATE_LP_SEND;
		memcpy(admin->tx_payload, buf, nr);
		admin->nr_tx_payload = nr;
		admin->nr_tx_frame = async_wrap_tty(
			tty,
			admin->tx_frame,
			MAX_TX_BUFF_SIZE
		);

		#define MINIMUM_LAP_FRAME_SIZE 5
		if (admin->nr_tx_frame < 0) {
			admin->state = SHIRDA_STATE_LP_READY;
			IRDALOG_ERROR("tx buffer overflow");
			up(&admin->sem);
			return -ENOMEM;
		} else if (admin->nr_tx_frame == 0) {
			admin->state = SHIRDA_STATE_LP_READY;
			IRDALOG_WARNING("tx datasize == 0 byte");
			up(&admin->sem);
			return 0;
		} else if (0< admin->nr_tx_frame
			&& admin->nr_tx_frame < MINIMUM_LAP_FRAME_SIZE){
			IRDALOG_WARNING(
				"unexpected datasize %d byte",
				admin->nr_tx_frame
			);
		}
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		shirda_ldisc_clear_wakeup_queue(&admin->wakeup_queue);
		shirda_ldisc_rx_queue_clear(&admin->rx_queue);
	up(&admin->sem);

	sent_byte = admin->nr_tx_frame;
	send_time = shirda_calculate_tx_time(admin->qos.baud_rate, sent_byte);
	wait_time = shirda_calculate_tx_timeout(
		admin->qos.baud_rate,
		sent_byte
	);
	atomic_set(&admin->tx_kp.write_wakeup_flag, FALSE);
	ret = tty->ops->write(tty, admin->tx_frame, admin->nr_tx_frame);
	shirda_ldisc_write_timekeeper_start(&admin->tx_kp, send_time);

	if (ret < sent_byte) {
		IRDALOG_ERROR("fatal error: writing all byte is uncomplete");
	}
	if (ret > nr) {
		ret = nr;
	}

	 while (exit_flag == FALSE) {
		shirda_ldisc_wait_event_timeout(
			&admin->wakeup_queue,
			wait_time
		);
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		event = shirda_ldisc_wakeup_event_dequeue(
			&admin->wakeup_queue
		);

		down(&admin->sem);
			switch (event) {
			case IRDA_KDRV_WU_EV_ISR_RECV:
				IRDALOG_WARNING("unexpected event :%d", event);
				shirda_ldisc_rx_queue_clear(&admin->rx_queue);
				exit_flag = FALSE;
				break;

			case IRDA_KDRV_WU_EV_ISR_SEND:
				shirda_ldisc_write_timekeeper_wait(
					&admin->tx_kp, ktime_get()
				);
				exit_flag = TRUE;
				IRDALOG_INFO("done sending.");
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_NO_ERR;
				break;

			case IRDA_KDRV_WU_EV_ISR_TIMEOUT:
				IRDALOG_INFO("sending timeout.");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_TX_TIMEOUT;
				ret = -ETIME;
				break;

			case IRDA_KDRV_WU_EV_MEMORY_FULL:
				IRDALOG_ERROR("wakeup queue overflow");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;

			case IRDA_KDRV_WU_EV_USR_CANCEL:
				IRDALOG_ERROR("unexpected event:%d.",event);
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;

			case IRDA_KDRV_WU_EV_CLOSE:
				IRDALOG_INFO("close");
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_IDLE;
				admin->recent_err = IRDA_LDISC_CLOSED;
				ret = -EIO;
				break;

			default:
				IRDALOG_ERROR("unknown event :%d", event);
				exit_flag = TRUE;
				admin->state = SHIRDA_STATE_LP_READY;
				admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
				ret = -EINVAL;
				break;
			}
		up(&admin->sem);
	}

	return ret;
}

static ssize_t shirda_ldisc_write(
	struct tty_struct *tty,
	struct file *file,
	const unsigned char *buf,
	size_t nr
)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	if (nr<0 || nr>MAX_TX_PAYLOAD) {
		IRDALOG_ERROR("too large size");
		return -EINVAL;
	}

	if (in_interrupt()) {
		return -EIO;
	}

	switch (admin->state) {
	case SHIRDA_STATE_IDLE:
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR("logical error.");
		break;

	case SHIRDA_STATE_READY:
		ret = shirda_ldisc_write_READY(tty,file,buf,nr);
		break;

	case SHIRDA_STATE_LP_READY:
		ret = shirda_ldisc_write_LP_READY(tty,file,buf,nr);
		break;

	case SHIRDA_STATE_LP_RWAIT:
	case SHIRDA_STATE_SEND_WAIT:
	case SHIRDA_STATE_SEND:
	case SHIRDA_STATE_LP_RECEIVE:
	case SHIRDA_STATE_LP_SEND:
	case SHIRDA_STATE_RECEIVE:
	case SHIRDA_STATE_MEDIABUSY:
	case SHIRDA_STATE_LP_WAITW:
		admin->recent_err = IRDA_LDISC_PERMISSION_ERR;
		ret = -EPERM;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int shirda_ldisc_set_qos(
	struct tty_struct *tty,
	const void __user *irda_qos
)
{

	int ret = 0;
	irda_qos_info next_qos;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("state=%d\n",admin->state);

	switch (admin->state)
	{
	case SHIRDA_STATE_IDLE:
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR(
			"logical error: unexpected state(IDLE)."
		);
		break;

	case SHIRDA_STATE_SEND_WAIT:
	case SHIRDA_STATE_SEND:
	case SHIRDA_STATE_LP_RWAIT:
	case SHIRDA_STATE_LP_RECEIVE:
	case SHIRDA_STATE_LP_SEND:
		admin->recent_err = IRDA_LDISC_PERMISSION_ERR;
		ret = -EPERM;
		break;

	case SHIRDA_STATE_READY:
	case SHIRDA_STATE_RECEIVE:
	case SHIRDA_STATE_MEDIABUSY:
	case SHIRDA_STATE_LP_READY:
	case SHIRDA_STATE_LP_WAITW:
		if (copy_from_user(&next_qos,irda_qos,sizeof(irda_qos_info))) {
			IRDALOG_ERROR("failed to copy from user\n");
			ret = -EFAULT;
		} else {
			ret = shirda_ldisc_set_qos_trunk(tty,&next_qos);
		}
		break;

	default:
		ret = -EINVAL;
		IRDALOG_ERROR("unknown ldisc state.\n");
		break;
	}

	return ret;
}

static int shirda_ldisc_get_qos(
	struct tty_struct *tty,
	void __user *irda_qos
)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	if (admin->state==SHIRDA_STATE_IDLE) {
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR("logical error: unexpected state");
	}

	if (copy_to_user(irda_qos, &admin->qos, sizeof(admin->qos))) {
		ret = -EFAULT;
	}

	return ret;
}

static int shirda_ldisc_rwakeup(struct tty_struct *tty)
{
	int ret = -EINVAL;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	down(&admin->sem);
		IRDALOG_INFO("state=%d",admin->state);

		if (
			admin->state!=SHIRDA_STATE_SEND
			&& admin->state!=SHIRDA_STATE_LP_SEND
		)
		{
			shirda_ldisc_wakeup_event_enqueue(
				&SHIRDA_ADMIN(tty)->wakeup_queue,
				IRDA_KDRV_WU_EV_USR_CANCEL
			);
			ret = 0;
		}
	up(&admin->sem);

	return ret;
}

static int shirda_ldisc_get_err(struct tty_struct *tty, void __user *err_info)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("shirda_ldisc_get_err()");

	if (admin->state==SHIRDA_STATE_IDLE) {
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR("logical error");
	}

	admin->rx_buf.stats.irda_err = admin->recent_err;

	if (copy_to_user(err_info, &(admin->rx_buf.stats),
					sizeof(admin->rx_buf.stats))) {
		ret = -EFAULT;
	}

	return ret;
}

static int shirda_ldisc_get_mediabusy(
	struct tty_struct *tty,
	void __user *mbusy
)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("shirda_ldisc_get_mediabusy()=%d\n",admin->media_busy);

	if (copy_to_user(mbusy, &(admin->media_busy),
					sizeof(admin->media_busy))) {
		IRDALOG_ERROR("failed to copy to user\n");
		ret = -EFAULT;
	}

	return ret;
}
static int shirda_ldisc_clr_mediabusy(struct tty_struct *tty)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("shirda_ldisc_clr_mediabusy()");

	admin->media_busy = IRDA_LDISC_MEDIA_FREE;

	return ret;
}

static int shirda_ldisc_loopback_READY(struct tty_struct *tty)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("state=%d",admin->state);
	admin->state = SHIRDA_STATE_LP_READY;

	return ret;
}

static int shirda_ldisc_loopback(struct tty_struct *tty)
{
	int ret = 0;
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	IRDALOG_INFO("state=%d",admin->state);

	if (down_interruptible(&admin->sem)) {
		return -ERESTARTSYS;
	}
	switch (admin->state) {
	case SHIRDA_STATE_READY:
		ret = shirda_ldisc_loopback_READY(tty);
		break;

	case SHIRDA_STATE_IDLE:
		ret = -EINVAL;
		admin->recent_err = IRDA_LDISC_LOGICAL_ERR;
		IRDALOG_ERROR("logical error.");
		break;

	case SHIRDA_STATE_LP_READY:
		ret = 0;
		IRDALOG_WARNING("shirda already loopback mode.");
		break;

	case SHIRDA_STATE_RECEIVE:
	case SHIRDA_STATE_SEND_WAIT:
	case SHIRDA_STATE_SEND:
	case SHIRDA_STATE_MEDIABUSY:
	case SHIRDA_STATE_LP_RECEIVE:
	case SHIRDA_STATE_LP_RWAIT:
	case SHIRDA_STATE_LP_SEND:
	case SHIRDA_STATE_LP_WAITW:
		admin->recent_err = IRDA_LDISC_PERMISSION_ERR;
		ret = -EPERM;
		break;

	default:
		ret = -EINVAL;
		IRDALOG_ERROR("unknown ldisc state.");
		break;
	}
	up(&admin->sem);

	return ret;
}

static int shirda_ldisc_get_capability(void __user *capa_info)
{
	int ret = 0;
	irda_kdrv_capa_notify capa = {
		.qos_init = {
			.baud_rate		= IRDA_BAUD_9600,
			.connection_address	= IRDA_KDRV_DEF_CA,
			.add_bof		= IRDA_KDRV_DEF_ABOF,
			.mtt			= IRDA_KDRV_DEF_MTT,
			.mpi			= IRDA_DRV_SET_MPI_MAX
		},
		.capability = {
			.baud_rate =
				  IRDA_DRV_CAPA_BAUD_9600
				| IRDA_DRV_CAPA_BAUD_19200
				| IRDA_DRV_CAPA_BAUD_38400
				| IRDA_DRV_CAPA_BAUD_57600
				| IRDA_DRV_CAPA_BAUD_115200,
			.add_bof =
				  IRDA_DRV_CAPA_ADD_BOF_48
				| IRDA_DRV_CAPA_ADD_BOF_32
				| IRDA_DRV_CAPA_ADD_BOF_24
				| IRDA_DRV_CAPA_ADD_BOF_20
				| IRDA_DRV_CAPA_ADD_BOF_16
				| IRDA_DRV_CAPA_ADD_BOF_14
				| IRDA_DRV_CAPA_ADD_BOF_12
				| IRDA_DRV_CAPA_ADD_BOF_10
				| IRDA_DRV_CAPA_ADD_BOF_8
				| IRDA_DRV_CAPA_ADD_BOF_6
				| IRDA_DRV_CAPA_ADD_BOF_5
				| IRDA_DRV_CAPA_ADD_BOF_4
				| IRDA_DRV_CAPA_ADD_BOF_3
				| IRDA_DRV_CAPA_ADD_BOF_2
				| IRDA_DRV_CAPA_ADD_BOF_1
				| IRDA_DRV_CAPA_ADD_BOF_0,
			.min_tat =
				  IRDA_DRV_CAPA_MINTAT_10000
				| IRDA_DRV_CAPA_MINTAT_5000
				| IRDA_DRV_CAPA_MINTAT_1000
				| IRDA_DRV_CAPA_MINTAT_500
				| IRDA_DRV_CAPA_MINTAT_100
				| IRDA_DRV_CAPA_MINTAT_50
				| IRDA_DRV_CAPA_MINTAT_10,
			.mpi = {
				.mpi_min = IRDA_DRV_SET_MPI_MIN,
				.mpi_max = IRDA_DRV_SET_MPI_MAX
			},
			.buff = {
				.max_size = MAX_RX_BUFF_SIZE,
				.hw_buff_cnt = 1,
				.sw_tx_buff_cnt = 1,
				.sw_rx_buff_cnt = 3,
			},
			.window_size = IRDA_DRV_CAPA_WINDOW_SIZE_1,
		},
		.request_capability = {
			.baud_rate =
				  IRDA_DRV_CAPA_BAUD_9600
				| IRDA_DRV_CAPA_BAUD_19200
				| IRDA_DRV_CAPA_BAUD_38400
				| IRDA_DRV_CAPA_BAUD_57600
				| IRDA_DRV_CAPA_BAUD_115200,
			.baud_rate_ext = 0x0,
 			.add_bof = 0xff,
			.min_tat = 0x03,
			.window_size = IRDA_DRV_CAPA_WINDOW_SIZE_1,
			.mpi = 0xA,
			.data_size =
				  IRDA_DRV_CAPA_DATA_SIZE_64
				| IRDA_DRV_CAPA_DATA_SIZE_128
				| IRDA_DRV_CAPA_DATA_SIZE_256
				| IRDA_DRV_CAPA_DATA_SIZE_512
				| IRDA_DRV_CAPA_DATA_SIZE_1024
				| IRDA_DRV_CAPA_DATA_SIZE_2048
,
		}
	};

	if (copy_to_user(capa_info, &capa, sizeof(capa))) {
		ret = -EFAULT;
	}
	return ret;
}

static int shirda_ldisc_ioctl(
	struct tty_struct *tty,
	struct file * file,
	unsigned int cmd,
	unsigned long arg
)
{
	int ret = 0;
	IRDALOG_INFO("shirda_ldisc_ioctl()\n");

	switch (cmd) {
	case IRDA_DRV_IOCTL_SET_QOS:
		ret = shirda_ldisc_set_qos(tty, (const void __user *)arg);
		break;
	case IRDA_DRV_IOCTL_GET_QOS:
		ret = shirda_ldisc_get_qos(tty, (void __user *)arg);
		break;
	case IRDA_DRV_IOCTL_READ_WAKEUP:
		shirda_ldisc_rwakeup(tty);
		break;
	case IRDA_DRV_IOCTL_GET_ERR:
		ret = shirda_ldisc_get_err(tty, (void __user *)arg);
		break;
	case IRDA_DRV_IOCTL_LOOPBACK:
		ret = shirda_ldisc_loopback(tty);
		break;
	case IRDA_DRV_IOCTL_GET_CAPABILITY:
		ret = shirda_ldisc_get_capability((void __user *)arg);
		break;
	case IRDA_DRV_IOCTL_GET_MEDIABUSY:
		ret = shirda_ldisc_get_mediabusy(tty,(void __user *)arg);
		break;
	case IRDA_DRV_IOCTL_CLR_MEDIABUSY:
		ret = shirda_ldisc_clr_mediabusy(tty);
		break;
	default:
		ret = -EINVAL;
		IRDALOG_ERROR("unknown ioctl command.\n");
		break;
	}

	return ret;
}

static void shirda_ldisc_receive_buf(struct tty_struct *tty,
			const unsigned char *cp, char *fp, int count)
{
	SHIRDA_ADMIN(tty)->media_busy = IRDA_LDISC_MEDIA_BUSY;

	if (shirda_ldisc_check_rx_enabled(tty)==FALSE) {
		return;
	}


	while (count > 0) {
		async_unwrap_char_tty(tty, &(SHIRDA_ADMIN(tty)->rx_buf), *cp);
		cp++;
		count--;
	}
}

static int shirda_ldisc_address_filter(
	const iobuff_t *rx_buff,
	const irda_qos_info *qos
)
{
#define ADDRESS_CRBIT_MASK 0x01
#define ADDRESS_FIELD_INDEX 0
	__u8 xor;

	xor = rx_buff->data[ADDRESS_FIELD_INDEX] ^ qos->connection_address;
	xor = xor & ~ADDRESS_CRBIT_MASK;

	if ((xor==0) || (qos->connection_address==0xff)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void shirda_async_bump(struct tty_struct *tty, iobuff_t *rx_buff)
{
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);
	irda_kdrv_wakeup_event_enum event;
	int err;

	IRDALOG_INFO("detected IrLAP frame\n");

	if (shirda_ldisc_address_filter(rx_buff, &admin->qos)==FALSE) {
		IRDALOG_INFO("trapped by address filter");
		rx_buff->stats.rx_mask_packets++;
		return;
	}

	err = shirda_ldisc_rx_queue_enqueue(rx_buff, &admin->rx_queue);

	if (err == RX_QUE_OVERFLOW_ERROR) {
		IRDALOG_ERROR("rx buffer overflow");
		admin->recent_err = IRDA_LDISC_RX_BUFFER_OVERFLOW;
		event = IRDA_KDRV_WU_EV_RX_OVERFLOW;
	} else {
		IRDALOG_INFO("save recieved data to rx buffer.");
		shirda_ldisc_update_mtt_keeper(&admin->mtt_kp);
		event = IRDA_KDRV_WU_EV_ISR_RECV;
	}

	rx_buff->state = 0;
	rx_buff->in_frame = 0;
	rx_buff->data = rx_buff->head;
	rx_buff->len = 0;
	rx_buff->truesize = MAX_RX_BUFF_SIZE;
	rx_buff->fcs = 0;

	shirda_ldisc_wakeup_event_enqueue(
		&SHIRDA_ADMIN(tty)->wakeup_queue,
		event
	);
}

static void shirda_ldisc_write_wakeup(struct tty_struct *tty)
{
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	if (tty->ops->chars_in_buffer(tty) == 0) {
		IRDALOG_INFO("TX_READY interrupt");
		shirda_ldisc_enable_rx(tty);
		shirda_ldisc_write_timekeeper_write_wakeup(&admin->tx_kp);
	}
}

static void shirda_ldisc_write_wakeup_tlet(struct tty_struct *tty)
{
	struct shirda_ldisc_admin_t *admin = SHIRDA_ADMIN(tty);

	tasklet_schedule(&admin->tlet);
}

static void _shirda_ldisc_write_wakeup(unsigned long data)
{
	shirda_ldisc_write_wakeup((struct tty_struct *)data);
}

struct tty_ldisc_ops shirda_ldisc_ops = {
	.magic        = TTY_LDISC_MAGIC,
	.name         = SHIRDA_LDISC_DRIVER_NAME,
	.open         = shirda_ldisc_open,
	.close        = shirda_ldisc_close,
	.read         = shirda_ldisc_read,
	.write        = shirda_ldisc_write,
	.ioctl        = shirda_ldisc_ioctl,
	.receive_buf  = shirda_ldisc_receive_buf,
	.write_wakeup = shirda_ldisc_write_wakeup_tlet,
};

static int __init n_shirda_init(void)
{
	int err;
	err = tty_register_ldisc(N_SHIRDA, &shirda_ldisc_ops);
	if (err != 0) {
		IRDALOG_ERROR(
			"failed to register N_SHIRDA (error:%d).\n", err);
	}

	sema_init(&ldisc_admin.sem,1);

	ldisc_admin.state = SHIRDA_STATE_IDLE;

	return err;
}

static void __exit n_shirda_exit(void)
{
	int err;
	err = tty_unregister_ldisc(N_SHIRDA);
	if (err != 0) {
		IRDALOG_ERROR(
			"failed to unregister N_SHIRDA (error:%d).\n",err);
	}
}

module_init(n_shirda_init);
module_exit(n_shirda_exit);

MODULE_VERSION(SHIRDA_LDISC_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_LDISC(N_SHIRDA);
