/*********************************************************************
 *
 * Filename:      wrapper.c
 * Version:       1.2
 * Description:   IrDA SIR async wrapper layer
 * Status:        Stable
 * Author:        Dag Brattli <dagb@cs.uit.no>
 * Created at:    Mon Aug  4 20:40:53 1997
 * Modified at:   Fri Jan 28 13:21:09 2000
 * Modified by:   Dag Brattli <dagb@cs.uit.no>
 * Modified at:   Fri May 28  3:11 CST 1999
 * Modified by:   Horst von Brand <vonbrand@sleipnir.valparaiso.cl>
 *
 *     Copyright (c) 1998-2000 Dag Brattli <dagb@cs.uit.no>,
 *     All Rights Reserved.
 *     Copyright (c) 2000-2002 Jean Tourrilhes <jt@hpl.hp.com>
 *
 *     This program is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of
 *     the License, or (at your option) any later version.
 *
 *     Neither Dag Brattli nor University of Tromso admit liability nor
 *     provide warranty for any of this software. This material is
 *     provided "AS-IS" and at no charge.
 *
 ********************************************************************/
/*
 * Modified by SHARP CORPORATION. 2011-2012
 */
#include <linux/string.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <asm/byteorder.h>

#include <net/irda/crc.h>

#include "shirda_ldisc.h"
#include "shirda_kdrv.h"


#ifdef SHIRDA_DEBUG
#undef SHIRDA_WRAP_DEBUG
#endif

#ifdef SHIRDA_WRAP_DEBUG
#define SHIRDA_DEBUGLOG(format, args...)	IRDALOG_ERROR(format, ##args)
#else	/* SHIRDA_WRAP_DEBUG	*/
#define SHIRDA_DEBUGLOG(format, args...)
#endif	/* SHIRDA_WRAP_DEBUG	*/

#define BOF		0xc0	/* Beginning of frame */
#define XBOF		0xff	/* additional BOF */
#define EOF		0xc1	/* End of frame */
#define CE		0x7d	/* Control escape */
#define IRDA_TRANS	0x20	/* Asyncronous transparency modifier */

/* States for receiving a frame in async mode */
enum {
	OUTSIDE_FRAME,
	BEGIN_FRAME,
	LINK_ESCAPE,
	INSIDE_FRAME
};


/************************** FRAME WRAPPING **************************/
/*
 * Unwrap and unstuff SIR frames
 *
 * Note : at FIR and MIR, HDLC framing is used and usually handled
 * by the controller, so we come here only for SIR... Jean II
 */

/*
 * Function stuff_byte (byte, buf)
 *
 *    Byte stuff one single byte and put the result in buffer pointed to by
 *    buf. The buffer must at all times be able to have two bytes inserted.
 *
 * This is in a tight loop, better inline it, so need to be prior to callers.
 * (2000 bytes on P6 200MHz, non-inlined ~370us, inline ~170us) - Jean II
 */
static inline int stuff_byte(__u8 byte, __u8 *buf)
{
	switch (byte) {
	case BOF: /* FALLTHROUGH */
	case EOF: /* FALLTHROUGH */
	case CE:
		/* Insert transparently coded */
		buf[0] = CE;               /* Send link escape */
		buf[1] = byte^IRDA_TRANS;    /* Complement bit 5 */
		return 2;
		/* break; */
	default:
		 /* Non-special value, no transparency required */
		buf[0] = byte;
		return 1;
		/* break; */
	}
}

/*
 * Function async_wrap (tty, *tx_buff, buffsize)
 */
int async_wrap_tty(struct tty_struct *tty, __u8 *tx_buff, int buffsize)
{
	struct shirda_ldisc_admin_t *sp = tty->disc_data;
	int xbofs;
	int i;
	int n;
	union {
		__u16 value;
		__u8 bytes[2];
	} fcs;

	/* Initialize variables */
	fcs.value = INIT_FCS;
	n = 0;

	/*
	 *  Send  XBOF's for required min. turn time and for the negotiated
	 *  additional XBOFS
	 */

	xbofs = sp->qos.add_bof;

	SHIRDA_DEBUGLOG("abofs=%d\n", xbofs);

	/* Check that we never use more than 115 + 48 xbofs */
	if (xbofs > 163) {
		IRDALOG_WARNING("too many xbofs (%d)\n", xbofs);
		xbofs = 163;
	}

	memset(tx_buff + n, XBOF, xbofs);
	n += xbofs;

	/* Start of packet character BOF */
	tx_buff[n++] = BOF;

	SHIRDA_DEBUGLOG("tx_payload length=%d\n", sp->nr_tx_payload);
	/* Insert frame and calc CRC */
	for (i=0; i < sp->nr_tx_payload; i++) {
		/*
		 *  Check for the possibility of tx buffer overflow. We use
		 *  bufsize-5 since the maximum number of bytes that can be
		 *  transmitted after this point is 5.
		 */
		if(n >= (buffsize-5)) {
			IRDALOG_ERROR("tx buffer overflow (n=%d)\n", n);
			return -1;
		}

		n += stuff_byte(sp->tx_payload[i], tx_buff+n);
		fcs.value = irda_fcs(fcs.value, sp->tx_payload[i]);
		SHIRDA_DEBUGLOG("len=%04d:%04d, data=0x%02x, fcs=0x%04x\n",
					i, n, sp->tx_payload[i], fcs.value);
	}

	/* Insert CRC in little endian format (LSB first) */
	fcs.value = ~fcs.value;
#ifdef __LITTLE_ENDIAN
	n += stuff_byte(fcs.bytes[0], tx_buff+n);
	n += stuff_byte(fcs.bytes[1], tx_buff+n);
#else /* ifdef __BIG_ENDIAN */
	n += stuff_byte(fcs.bytes[1], tx_buff+n);
	n += stuff_byte(fcs.bytes[0], tx_buff+n);
#endif
	tx_buff[n++] = EOF;

	SHIRDA_DEBUGLOG("tx_buff length=%d\n", n);
	return n;
}

/************************* FRAME UNWRAPPING *************************/
/*
 * Unwrap and unstuff SIR frames
 *
 * Complete rewrite by Jean II :
 * More inline, faster, more compact, more logical. Jean II
 * (16 bytes on P6 200MHz, old 5 to 7 us, new 4 to 6 us)
 * (24 bytes on P6 200MHz, old 9 to 10 us, new 7 to 8 us)
 * (for reference, 115200 b/s is 1 byte every 69 us)
 * And reduce wrapper.o by ~900B in the process ;-)
 *
 * Then, we have the addition of ZeroCopy, which is optional
 * (i.e. the driver must initiate it) and improve final processing.
 * (2005 B frame + EOF on P6 200MHz, without 30 to 50 us, with 10 to 25 us)
 *
 * Note : at FIR and MIR, HDLC framing is used and usually handled
 * by the controller, so we come here only for SIR... Jean II
 */

/*
 * We can also choose where we want to do the CRC calculation. We can
 * do it "inline", as we receive the bytes, or "postponed", when
 * receiving the End-Of-Frame.
 * (16 bytes on P6 200MHz, inlined 4 to 6 us, postponed 4 to 5 us)
 * (24 bytes on P6 200MHz, inlined 7 to 8 us, postponed 5 to 7 us)
 * With ZeroCopy :
 * (2005 B frame on P6 200MHz, inlined 10 to 25 us, postponed 140 to 180 us)
 * Without ZeroCopy :
 * (2005 B frame on P6 200MHz, inlined 30 to 50 us, postponed 150 to 180 us)
 * (Note : numbers taken with irq disabled)
 *
 * From those numbers, it's not clear which is the best strategy, because
 * we end up running through a lot of data one way or another (i.e. cache
 * misses). I personally prefer to avoid the huge latency spike of the
 * "postponed" solution, because it come just at the time when we have
 * lot's of protocol processing to do and it will hurt our ability to
 * reach low link turnaround times... Jean II
 */
//#define POSTPONE_RX_CRC

/*
 * Function async_bump (buf, len, stats)
 *
 *    Got a frame, make a copy of it, and pass it up the stack! We can try
 *    to inline it since it's only called from state_inside_frame
 */
#ifndef CONFIG_SHIRDA
static inline void
async_bump(struct net_device *dev,
	   struct net_device_stats *stats,
	   iobuff_t *rx_buff)
{
	struct sk_buff *newskb;
	struct sk_buff *dataskb;
	int		docopy;

	/* Check if we need to copy the data to a new skb or not.
	 * If the driver doesn't use ZeroCopy Rx, we have to do it.
	 * With ZeroCopy Rx, the rx_buff already point to a valid
	 * skb. But, if the frame is small, it is more efficient to
	 * copy it to save memory (copy will be fast anyway - that's
	 * called Rx-copy-break). Jean II */
	docopy = ((rx_buff->skb == NULL) ||
		  (rx_buff->len < IRDA_RX_COPY_THRESHOLD));

	/* Allocate a new skb */
	newskb = dev_alloc_skb(docopy ? rx_buff->len + 1 : rx_buff->truesize);
	if (!newskb)  {
		stats->rx_dropped++;
		/* We could deliver the current skb if doing ZeroCopy Rx,
		 * but this would stall the Rx path. Better drop the
		 * packet... Jean II */
		return;
	}

	/* Align IP header to 20 bytes (i.e. increase skb->data)
	 * Note this is only useful with IrLAN, as PPP has a variable
	 * header size (2 or 1 bytes) - Jean II */
	skb_reserve(newskb, 1);

	if(docopy) {
		/* Copy data without CRC (length already checked) */
		skb_copy_to_linear_data(newskb, rx_buff->data,
					rx_buff->len - 2);
		/* Deliver this skb */
		dataskb = newskb;
	} else {
		/* We are using ZeroCopy. Deliver old skb */
		dataskb = rx_buff->skb;
		/* And hook the new skb to the rx_buff */
		rx_buff->skb = newskb;
		rx_buff->head = newskb->data;	/* NOT newskb->head */
		//printk(KERN_DEBUG "ZeroCopy : len = %d, dataskb = %p, newskb = %p\n", rx_buff->len, dataskb, newskb);
	}

	/* Set proper length on skb (without CRC) */
	skb_put(dataskb, rx_buff->len - 2);

	/* Feed it to IrLAP layer */
	dataskb->dev = dev;
	skb_reset_mac_header(dataskb);
	dataskb->protocol = htons(ETH_P_IRDA);

	netif_rx(dataskb);

	stats->rx_packets++;
	stats->rx_bytes += rx_buff->len;

	/* Clean up rx_buff (redundant with async_unwrap_bof() ???) */
	rx_buff->data = rx_buff->head;
	rx_buff->len = 0;
}
#endif	/* CONFIG_SHIRDA */

/*
 * Function async_unwrap_bof(dev, byte)
 *
 *    Handle Beginning Of Frame character received within a frame
 *
 */
static inline void
async_unwrap_bof(struct tty_struct *tty, iobuff_t *rx_buff, __u8 byte)
{
	SHIRDA_DEBUGLOG("state=%d,byte=0x%x\n",rx_buff->state,byte);
	switch(rx_buff->state) {
	case LINK_ESCAPE:
	case INSIDE_FRAME:
		/* Not supposed to happen, the previous frame is not
		 * finished - Jean II */
		rx_buff->stats.rx_errors++;
		rx_buff->stats.rx_missed_errors++;
		break;

	default:
		break;
	}
	/* Now receiving frame */
	rx_buff->state = BEGIN_FRAME;

	/* Time to initialize receive buffer */
	rx_buff->data = rx_buff->head;
	rx_buff->len = 0;
	rx_buff->fcs = INIT_FCS;
}

/*
 * Function async_unwrap_eof(dev, byte)
 *
 *    Handle End Of Frame character received within a frame
 *
 */
static inline void
async_unwrap_eof(struct tty_struct *tty, iobuff_t *rx_buff, __u8 byte)
{
#ifdef POSTPONE_RX_CRC
	int	i;
#endif

	SHIRDA_DEBUGLOG("state=%d,byte=0x%x\n",rx_buff->state,byte);
	switch(rx_buff->state) {
	case OUTSIDE_FRAME:
		/* Probably missed the BOF */
		rx_buff->stats.rx_errors++;
		rx_buff->stats.rx_missed_errors++;
		break;

	case BEGIN_FRAME:
	case LINK_ESCAPE:
	case INSIDE_FRAME:
	default:
		/* Note : in the case of BEGIN_FRAME and LINK_ESCAPE,
		 * the fcs will most likely not match and generate an
		 * error, as expected - Jean II */
		rx_buff->state = OUTSIDE_FRAME;
		rx_buff->in_frame = FALSE;

#ifdef POSTPONE_RX_CRC
		/* If we haven't done the CRC as we receive bytes, we
		 * must do it now... Jean II */
		for(i = 0; i < rx_buff->len; i++)
			rx_buff->fcs = irda_fcs(rx_buff->fcs,
						rx_buff->data[i]);
#endif

		/* Test FCS and signal success if the frame is good */
		if (rx_buff->fcs == GOOD_FCS) {
			/* Deliver frame */
			rx_buff->stats.rx_packets++;
			rx_buff->stats.rx_bytes += (rx_buff->len - 2);
			shirda_async_bump(tty, rx_buff);
			break;
		} else {
			/* Wrong CRC, discard frame!  */
			SHIRDA_DEBUGLOG("crc error\n");
			rx_buff->stats.rx_errors++;
			rx_buff->stats.rx_crc_errors++;
		}
		break;
	}
}


/*
 * Function async_unwrap_ce(dev, byte)
 *
 *    Handle Character Escape character received within a frame
 *
 */
static inline void
async_unwrap_ce(struct tty_struct *tty, iobuff_t *rx_buff, __u8 byte)
{
	SHIRDA_DEBUGLOG("state=%d,byte=0x%x\n",rx_buff->state,byte);
	switch(rx_buff->state) {
	case OUTSIDE_FRAME:
		/* Activate carrier sense */
		break;

	case LINK_ESCAPE:
		SHIRDA_DEBUGLOG(": state not defined\n");
		break;

	case BEGIN_FRAME:
	case INSIDE_FRAME:
	default:
		/* Stuffed byte coming */
		rx_buff->state = LINK_ESCAPE;
		break;
	}
}

/*
 * Function async_unwrap_other(dev, byte)
 *
 *    Handle other characters received within a frame
 *
 */
static inline void
async_unwrap_other(struct tty_struct *tty,
		   iobuff_t *rx_buff, __u8 byte)
{
	SHIRDA_DEBUGLOG("state=%d,byte=0x%x\n",rx_buff->state,byte);
	switch(rx_buff->state) {
		/* This is on the critical path, case are ordered by
		 * probability (most frequent first) - Jean II */
	case INSIDE_FRAME:
		/* Must be the next byte of the frame */
		if (rx_buff->len < rx_buff->truesize)  {
			rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
			rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
		} else {
			SHIRDA_DEBUGLOG("Rx buffer overflow, aborting\n");
			rx_buff->state = OUTSIDE_FRAME;
		}
		break;

	case LINK_ESCAPE:
		/*
		 *  Stuffed char, complement bit 5 of byte
		 *  following CE, IrLAP p.114
		 */
		byte ^= IRDA_TRANS;
		if (rx_buff->len < rx_buff->truesize)  {
			rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
			rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
			rx_buff->state = INSIDE_FRAME;
		} else {
			SHIRDA_DEBUGLOG("Rx buffer overflow, aborting\n");
			rx_buff->state = OUTSIDE_FRAME;
		}
		break;

	case OUTSIDE_FRAME:
		/* Activate carrier sense */
		break;

	case BEGIN_FRAME:
	default:
		rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
		rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
		rx_buff->state = INSIDE_FRAME;
		break;
	}
}


/*
 * Function async_unwrap_char (tty, rx_buff, byte)
 * arguments:
 *	tty->
 *	*rx_buf		O Receive data buffeer
 *	byte		I Receive byte
 */
void async_unwrap_char_tty(struct tty_struct *tty,
			   iobuff_t *rx_buff, __u8 byte)
{
	switch(byte) {
	case CE:
		async_unwrap_ce(tty, rx_buff, byte);
		break;
	case BOF:
		async_unwrap_bof(tty, rx_buff, byte);
		break;
	case EOF:
		async_unwrap_eof(tty, rx_buff, byte);
		break;
	default:
		async_unwrap_other(tty, rx_buff, byte);
		break;
	}
}
