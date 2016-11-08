/*
* Copyright (C) 2012 Invensense, Inc.
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

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_gyro.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This driver currently works for the ITG3500, MPU6050, MPU9150
 *               MPU3050
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "inv_gyro.h"
/* shmds add 1-4 -> */
#include <linux/gpio.h>
/* shmds add 1-4 <- */
/* shmds add 11-1 -> */
#include <asm/atomic.h>
/* shmds add 11-1 <- */
/* shmds add 7-1 -> */
#ifdef CONFIG_SHTERM
#define MPL_SHTERM_ENABLE
#endif /* CONFIG_SHTERM */
#ifdef MPL_SHTERM_ENABLE
#include <sharp/shterm_k.h>
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */
/* shmds add 8-1 -> */
#include <linux/regulator/consumer.h>
/* shmds add 8-1 <- */
/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
#include <sharp/shmds_driver.h>

static struct shmds_gyro_data current_gyro_data;
static struct shmds_gyro_data last_gyro_data;
static struct shmds_accl_data current_accl_data;
static struct shmds_accl_data last_accl_data;
static unsigned char shmds_write_flag[2] = {0,0};
static unsigned char shmds_last_data_flag[2] = {0,0};

#endif /* SHMDS_CAF */

/* shmds add 12-4 -> */
#include <linux/pm_qos.h>
#include <mach/cpuidle.h>

#define SHMDS_PM_QOS_LATENCY_VALUE 34
static struct pm_qos_request shmds_qos_cpu_dma_latency;

static void shmds_qos_start(void)
{
    pm_qos_update_request(&shmds_qos_cpu_dma_latency, SHMDS_PM_QOS_LATENCY_VALUE);
}
static void shmds_qos_end(void)
{
    pm_qos_update_request(&shmds_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);
}
/* shmds add 12-4 <- */


/* shmds add 9-1 <- */

/* shmds add 17-2 -> */
#include <linux/wakelock.h>
static struct wake_lock shmds_wake_lock;
static int shmds_wake_lock_num = 0;
static spinlock_t shmds_wake_spinlock;
/* shmds add 17-2 <- */

/* shmds add 11-1 -> */
#include <linux/mutex.h>
static struct mutex	shmds_lock;
static struct mutex	shmds_timerlock;
#ifdef SHMDS_DETECT
int shmds_ths_value = 6;
int shmds_duration_value = 50;
static unsigned long shmds_count = 0;
/* shmds mod 11-2 -> */
static int shmds_buf_size = 34;
int shmds_buf_size_temp = 34;
/* shmds mod 11-2 <- */
int shmds_md_thresh_value = 200;
int shmds_mg_to_LSB = 16;
int shmds_md_thresh = 3200;      				/* shmds_md_thresh_value * shmds_mg_to_LSB */
static int shmds_detect_switch = DETECT_OFF;
static signed short *shmds_detect_buf[3];
static signed short shmds_economize_buf[3] = {0, 0, 0};
atomic_t shmds_detect_mode_flg = ATOMIC_INIT(SHMDS_DETECT_NORMAL);
int shmds_full_buffer_flg = 0;
static signed short shmds_tmp_buf[3][SHMDS_TMP_BUFSIZE];
static signed short center_data[3];
#endif /* SHMDS_DETECT */
struct timer_list timer_acc;
atomic_t shmds_timer_kind_flg = ATOMIC_INIT(SHMDS_TIMER_ADD);
/* shmds add 11-1 <- */

/* shmds add 17-2 -> */
/**
 *  shmds_wake_lock_init
 */
static void shmds_wake_lock_init(void)
{
	spin_lock_init(&shmds_wake_spinlock);
	shmds_wake_lock_num = 0;
	wake_lock_init(&shmds_wake_lock, WAKE_LOCK_SUSPEND, "shmds_wake_lock");
	return;
}

/**
 *  shmds_wake_lock_destroy
 */
static void shmds_wake_lock_destroy(void)
{
	wake_unlock(&shmds_wake_lock);
	shmds_wake_lock_num = 0;
	wake_lock_destroy(&shmds_wake_lock);
	return;
}

/**
 *  shmds_wake_lock_start
 */
static void shmds_wake_lock_start(void)
{
/* shmds del 17-4 -> */
//	unsigned long flags;

//	spin_lock_irqsave(&shmds_wake_spinlock, flags);
//	if (!shmds_wake_lock_num) {
//		wake_lock(&shmds_wake_lock);
//	}
//	shmds_wake_lock_num++;
//	spin_unlock_irqrestore(&shmds_wake_spinlock, flags);
/* shmds del 17-4 <- */

	return;
}

/**
 *  shmds_wake_lock_end
 */
static void shmds_wake_lock_end(void)
{
/* shmds del 17-4 -> */
//	unsigned long flags;

//	spin_lock_irqsave(&shmds_wake_spinlock, flags);
//	shmds_wake_lock_num--;
//	if (!shmds_wake_lock_num) {
//		wake_unlock(&shmds_wake_lock);
//	}
//	if (shmds_wake_lock_num < 0) {
//		shmds_wake_lock_num = 0;
//	}
//	spin_unlock_irqrestore(&shmds_wake_spinlock, flags);
/* shmds del 17-4 <- */

	return;
}
/* shmds add 17-2 <- */


#define CHECK_DMP	do \
	{ \
		if ((st->chip_config.is_asleep) || \
		(0 == st->chip_config.firmware_loaded)) \
			return -EPERM; \
		result = kstrtoul(buf, 10, (long unsigned int *)&data); \
		if (result) \
			return result; \
	} while (0);

#define CHECK_ENABLE  do \
	{ \
		if (st->chip_config.is_asleep || \
			st->chip_config.enable) \
			return -EPERM; \
		result = kstrtoul(buf, 10, &data); \
		if (result) \
			return result; \
	} while (0);

/* shmds add 12-4 -> */
#define CHECK_ENABLE_EX  do \
	{ \
		if (st->chip_config.is_asleep || \
	        st->chip_config.enable) { \
	        shmds_qos_end(); \
	        shmds_wake_lock_end(); \
			return -EPERM; \
	        } \
		result = kstrtoul(buf, 10, &data); \
	    if (result) { \
	        shmds_qos_end(); \
	        shmds_wake_lock_end(); \
			return result; \
	    } \
	} while (0);
/* shmds add 12-4 <- */

void inv_setup_reg(struct inv_reg_map_s *reg)
{
	reg->who_am_i			= 0x75;
	reg->sample_rate_div	= 0x19;
	reg->lpf				= 0x1A;
	reg->product_id			= 0x0C;
	reg->bank_sel			= 0x6D;
	reg->user_ctrl			= 0x6A;
	reg->fifo_en			= 0x23;
	reg->gyro_config		= 0x1B;
	reg->accl_config		= 0x1C;
	reg->fifo_count_h		= 0x72;
	reg->fifo_r_w			= 0x74;
	reg->raw_gyro			= 0x43;
	reg->raw_accl			= 0x3B;
	reg->temperature		= 0x41;
	reg->int_enable			= 0x38;
	reg->int_status			= 0x3A;
	reg->pwr_mgmt_1			= 0x6B;
	reg->pwr_mgmt_2			= 0x6C;
	reg->mem_start_addr		= 0x6E;
	reg->mem_r_w			= 0x6F;
	reg->prgm_strt_addrh	= 0x70;
/* shmds add 11-1 -> */
	reg->mot_thr			= 0x1F;
	reg->mot_dur			= 0x20;
/* shmds add 11-1 <- */
	
};

static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	{119, "ITG3500"},
	{ 63, "MPU3050"},
	{118, "MPU6050"},
	{118, "MPU9150"}
};

s64 get_time_ns(void)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

/**
 *  inv_i2c_read_base() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
	unsigned char reg, unsigned short length, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data)
		return -EINVAL;

	msgs[0].addr = i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	pr_debug("%s RD%02X%02X%02X\n", st->hw->name, i2c_addr, reg, length);
	res = i2c_transfer(st->sl_handle, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/**
 *  inv_i2c_single_write_base() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
	unsigned short i2c_addr, unsigned char reg, unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	pr_debug("%s WS%02X%02X%02X\n", st->hw->name, i2c_addr, reg, data);
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/* shmds add 8-1 -> */
/* shmds mod 8-2 -> */
#if 0
static void regulator_ctrl(unsigned int flag)
#else
static void regulator_ctrl(struct inv_gyro_state_s *st, unsigned int flag)
/* shmds mod 8-2 <- */
#endif
{
	int ret;

/* shmds mod 1-6 -> */
	struct regulator *msdrv_vreg_8941_l18 = NULL;
/* shmds mod 8-2 -> */
#if 0
	msdrv_vreg_8941_l18 = regulator_get(NULL, "8941_l18");
#else
	msdrv_vreg_8941_l18 = regulator_get(st->inv_dev, "8941_l18");
#endif
/* shmds mod 8-2 <- */
	if(flag == SHMDS_REG_CHG_LPM) {
		ret = regulator_set_mode(msdrv_vreg_8941_l18, REGULATOR_MODE_STANDBY);
	}
	else if(flag == SHMDS_REG_CHG_NORMAL) {
		ret = regulator_set_mode(msdrv_vreg_8941_l18, REGULATOR_MODE_NORMAL);
	}
	
	regulator_put(msdrv_vreg_8941_l18);
/* shmds mod 1-6 <- */

}
/* shmds add 8-1 <- */

/**
 *  inv_clear_kfifo() - clear time stamp fifo
 *  @st:	Device driver instance.
 */
void inv_clear_kfifo(struct inv_gyro_state_s *st)
{
	unsigned long flags;
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->trigger.timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}
static int set_power_itg(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;

	reg = &st->reg;

/* shmds mod 8-1 -> */
	if (power_on){
/* shmds mod 8-2 -> */
#if 0
		regulator_ctrl(SHMDS_REG_CHG_NORMAL);
#else
		regulator_ctrl(st, SHMDS_REG_CHG_NORMAL);
#endif
/* shmds mod 8-2 <- */
		usleep(200);
		data = 0;
	} else {
		data = BIT_SLEEP;
	}
#if 0
	if (power_on)
		data = 0;
	else
		data = BIT_SLEEP;
#endif
/* shmds mod 8-1 <- */
	if (st->chip_config.lpa_mode)
		data |= BIT_CYCLE;
	if (st->chip_config.gyro_enable) {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_PLL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_INTERNAL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}

	if (power_on) {
/* shmds mod 12-1 -> */
		usleep(SLEEP_WAIT_UTIME);
#if 0
		msleep(POWER_UP_TIME);
#endif
/* shmds mod 12-1 <- */
		data = 0;
		if (0 == st->chip_config.accl_enable)
			data |= BIT_PWR_ACCL_STBY;
		if (0 == st->chip_config.gyro_enable)
			data |= BIT_PWR_GYRO_STBY;
		data |= (st->chip_config.lpa_freq << LPA_FREQ_SHIFT);

		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
/* shmds mod 12-2 -> */
#if 0
/* shmds mod 12-1 -> */
		usleep(STANDBY_WAIT_UTIME);
#if 0
		msleep(POWER_UP_TIME);
#endif
/* shmds mod 12-1 <- */
#else
		if ( st->chip_config.shmds_fwdl_comp ){
			usleep(STANDBY_WAIT_UTIME);
		}
#endif
/* shmds mod 12-2 <- */
		st->chip_config.is_asleep = 0;
/* shmds add 8-1 -> */
	} else {
/* shmds mod 8-2 -> */
#if 0
		regulator_ctrl(SHMDS_REG_CHG_LPM);
#else
		regulator_ctrl(st, SHMDS_REG_CHG_LPM);
#endif
/* shmds mod 8-2 <- */
		st->chip_config.is_asleep = 1;
	}
#if 0
	} else
		st->chip_config.is_asleep = 1;
#endif
/* shmds add 8-1 <- */
	return 0;
}

/**
 *  inv_set_power_state() - Turn device on/off.
 *  @st:	Device driver instance.
 *  @power_on:	1 to turn on, 0 to suspend.
 */
int inv_set_power_state(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	if (INV_MPU3050 == st->chip_type)
		return set_power_mpu3050(st, power_on);
	else
		return set_power_itg(st, power_on);
}

/* shmds add 2-1 -> */
int sysfs_read_value( unsigned char *ic_buf, const char *buf, size_t size, int len)
{
	unsigned long value;
	const char *pbuf = buf;
	char *scan_buf = NULL;
	int i=0, j=0;
	int retval = size;
	
	scan_buf = kzalloc(PIPE_BUF, GFP_KERNEL);
	if (scan_buf == NULL) {
		pr_err("%s Failed to allocate scan buffer for\n", __func__);
		goto err_sysfs_read;
	}
	
	while (pbuf <= (buf + size)) {
		while (((*pbuf == ' ') || (*pbuf == ',')) && (pbuf < (buf + size))) {
			pbuf++;
		}
		if (pbuf < (buf + size)) {
			memset(scan_buf, 0, PIPE_BUF);
			for (j = 0; j < sizeof("0xHH") && *pbuf != ' ' && *pbuf != ','; j++) {
				scan_buf[j] = *pbuf++;
			}
			retval = strict_strtoul(scan_buf, 16, &value);
			if (retval < 0) {
				pr_err("%s Invalid data format. Use \"0xHH,...,0xHH\" instead.\n", __func__);
				retval = -1;
				goto err_sysfs_read;
			} else {
				if( i < len ) {
					ic_buf[i] = value;
//					pr_err("%s: ic_buf[%d] = 0x%02X\n", __func__, i, ic_buf[i]);
					i++;
				}
			}
		} else {
			break;
		}
	}
	
err_sysfs_read:
	if (scan_buf != NULL)
		kfree(scan_buf);
		
	return size;
}
/* shmds add 2-1 <- */

/**
 *  inv_reset_dmp_pedometer() - Clear DMP variables for pedometer
 *  @st:	Device driver instance.
 */
static int inv_reset_dmp_pedometer(struct inv_gyro_state_s *st)
{
	int i;
	int result;
	unsigned char data[16];

	/* Clear DMP variables for pedometer.
	 * Not clear final counter values. */
	for(i = 0; i < 16; i++)
		data[i] = 0;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x30), 16, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x50), 16, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x64), 4, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x70), 8, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x88), 8, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0x90), 8, data);
	if (result)
		return result;

	result = mpu_memory_write(st->sl_handle, st->i2c_addr, 
			(768 + 0xa0), 4, data);
	if (result)
		return result;

	return 0;
}


/**
 *  reset_fifo_itg() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
static int reset_fifo_itg(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val;
	int ms_wait = 0;

	reg = &st->reg;
	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(st, reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	result = inv_i2c_single_write(st, reg->user_ctrl, 0);
	if (result)
		goto reset_fifo_fail;

	if (st->chip_config.dmp_on) {
		st->last_isr_time = get_time_ns();
		if (st->dmp_enabled) {
			val = (BIT_FIFO_RST | BIT_DMP_RST);
			st->dmp_enabled = 0;
			ms_wait = 5;
		} else {
			val = (BIT_FIFO_RST);
			ms_wait = 0;
		}
		val |= (BIT_DMP_EN | BIT_FIFO_EN);
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		if (ms_wait)
			msleep(ms_wait);	/* make sure reset completion of DMP */
		if (st->chip_config.dmp_int_on) {
			result = inv_i2c_single_write(st, reg->int_enable,
							BIT_DMP_INT_EN);
			if (result)
				return result;
		}
	} else {
		val = BIT_FIFO_RST;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		st->last_isr_time = get_time_ns();
		/* enable interrupt */
		if (st->chip_config.accl_fifo_enable ||
			st->chip_config.gyro_fifo_enable ||
			st->chip_config.compass_enable){
			result = inv_i2c_single_write(st, reg->int_enable,
						BIT_DATA_RDY_EN);
			if (result)
				return result;
		}
		/* enable FIFO reading and I2C master interface*/
		val = BIT_FIFO_EN;
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		/* enable sensor output to FIFO */
		val = 0;
		if (st->chip_config.gyro_fifo_enable)
			val |= BITS_GYRO_OUT;
		if (st->chip_config.accl_fifo_enable)
			val |= BIT_ACCEL_OUT;
		result = inv_i2c_single_write(st, reg->fifo_en, val);
		if (result)
			goto reset_fifo_fail;
	}
	return 0;
reset_fifo_fail:
	if (st->chip_config.dmp_on)
		val = BIT_DMP_INT_EN;
	else
		val = BIT_DATA_RDY_EN;
	inv_i2c_single_write(st, reg->int_enable, val);
	pr_err("%s failed\n", __func__);
	return result;
}

/**
 *  inv_reset_fifo() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
int inv_reset_fifo(struct inv_gyro_state_s *st)
{
	if (INV_MPU3050 == st->chip_type)
		return reset_fifo_mpu3050(st);
	else
		return reset_fifo_itg(st);
}

/**
 *  set_inv_enable() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 *  @fifo_enable: enable/disable
 */
int set_inv_enable(struct inv_gyro_state_s *st,
					unsigned long enable) {
	struct inv_reg_map_s *reg;
	int result;
	if (st->chip_config.is_asleep)
		return -EINVAL;
	reg = &st->reg;
	if (enable) {
		result = inv_reset_fifo(st);
		if (result)
			return result;
		inv_clear_kfifo(st);
		st->chip_config.enable = 1;
	} else {
		result = inv_i2c_single_write(st, reg->fifo_en, 0);
		if (result)
			return result;
		result = inv_i2c_single_write(st, reg->int_enable, 0);
		if (result)
			return result;
		/* disable fifo reading */
		if (INV_MPU3050 != st->chip_type) {
			result = inv_i2c_single_write(st, reg->user_ctrl, 0);
			if (result)
				return result;
		}
		st->chip_config.enable = 0;
	}
	return 0;
}

/**
 *  inv_setup_input() - internal setup input device.
 *  @st:	Device driver instance.
 *  @**idev_in  pointer to input device
 *  @*client    i2c client
 *  @*name      name of the input device.
 */
static int inv_setup_input(struct inv_gyro_state_s *st,
	struct input_dev **idev_in, struct i2c_client *client,
	unsigned char *name) {
	int result;
	struct input_dev *idev;
	idev = input_allocate_device();
	if (!idev) {
		result = -ENOMEM;
		return result;
	}
	/* Setup input device. */
	idev->name = name;

	idev->id.bustype = BUS_I2C;
	idev->id.product = 'S';
	idev->id.vendor     = ('I'<<8) | 'S';
	idev->id.version    = 1;
	idev->dev.parent = &client->dev;

	input_set_capability(idev, EV_REL, REL_X);
	input_set_capability(idev, EV_REL, REL_Y);
	input_set_capability(idev, EV_REL, REL_Z);
	input_set_capability(idev, EV_REL, REL_RX);
	input_set_capability(idev, EV_REL, REL_RY);
	input_set_capability(idev, EV_REL, REL_RZ);

	input_set_capability(idev, EV_REL, REL_MISC);
	input_set_capability(idev, EV_REL, REL_WHEEL);

	input_set_drvdata(idev, st);
	result = input_register_device(idev);
	if (result)
		input_free_device(idev);

	*idev_in = idev;
	return result;
}

/**
 *  inv_init_config() - Initialize hardware, disable FIFO.
 *  @st:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_init_config(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	int i, j;
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */

	if (st->chip_config.is_asleep)
		return -EPERM;

/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	for (i = 0; i < 3; i++) {
		shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size * sizeof(signed short), GFP_KERNEL);
		if (NULL == shmds_detect_buf[i]) {
			for (j = 0; j < i; j ++) {
				kfree(shmds_detect_buf[j]);
			}
			return -1;
		}
		memset(shmds_detect_buf[i], 0, shmds_buf_size * sizeof(signed short));
	}
#endif /* SHMDS_DETECT */
	mutex_init(&shmds_lock);
	mutex_init(&shmds_timerlock);
/* shmds add 11-1 <- */	
/* shmds add 2-1 -> */
	st->refreg.addr = 0x00;
	st->refreg.reg = 0x00;
/* shmds add 2-1 <- */
	reg = &st->reg;
	result = set_inv_enable(st, 0);
	if (result)
		return result;

	result = inv_i2c_single_write(st, reg->gyro_config,
		INV_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT);
	if (result)
		return result;
	st->chip_config.fsr = INV_FSR_2000DPS;

	result = inv_i2c_single_write(st, reg->lpf, INV_FILTER_42HZ);
	if (result)
		return result;
	st->chip_config.lpf = INV_FILTER_42HZ;

	result = inv_i2c_single_write(st, reg->sample_rate_div,
					ONE_K_HZ/INIT_FIFO_RATE - 1);
	if (result)
		return result;

	st->chip_config.fifo_rate = INIT_FIFO_RATE;
	st->irq_dur_us            = INIT_DUR_TIME;
	st->irq_dmp_dur_us        = INIT_DUR_TIME;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (INV_ITG3500 != st->chip_type) {
		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
		st->chip_config.accl_fs = INV_FS_02G;
		result = inv_i2c_single_write(st, reg->accl_config,
			(INV_FS_02G << ACCL_CONFIG_FSR_SHIFT));
		if (result)
			return result;
		st->tap.time = INIT_TAP_TIME;
		st->tap.thresh = INIT_TAP_THRESHOLD;
		st->tap.min_count = INIT_TAP_MIN_COUNT;
	}
	return 0;
}

/* shmds add 11-1 -> */
#if defined(SHMDS_DETECT) && defined(SHMDS_ADB_FLAG)
static ssize_t shmds_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%ld\n", shmds_count );
}

static ssize_t shmds_ths_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n", shmds_ths_value );
}

static ssize_t shmds_ths_value_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	
	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_ths_value = data;	
	shmds_md_thresh = shmds_md_thresh_value * shmds_mg_to_LSB;
	
	return count;
}

static ssize_t shmds_duration_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n",shmds_duration_value );
}

static ssize_t shmds_duration_value_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_duration_value = data;	

	return count;
}

static ssize_t shmds_buf_size_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n",shmds_buf_size_temp );
}

static ssize_t shmds_buf_size_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_buf_size_temp = data;	

	return count;
}

static ssize_t shmds_md_thresh_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n",shmds_md_thresh );
}

static ssize_t shmds_md_thresh_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_md_thresh = data;
		
	return count;
}

static ssize_t shmds_mg_to_LSB_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n",shmds_mg_to_LSB );
}

static ssize_t shmds_mg_to_LSB_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_mg_to_LSB = data;	
	shmds_md_thresh = shmds_md_thresh_value * shmds_mg_to_LSB;
	return count;
}

static ssize_t shmds_md_thresh_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf( buf, "%d\n", shmds_md_thresh_value );
}

static ssize_t shmds_md_thresh_value_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	shmds_md_thresh_value = data;	
	shmds_md_thresh = shmds_md_thresh_value * shmds_mg_to_LSB;
	return count;
}

#endif /* SHMDS_DETECT && SHMDS_ADB_FLAG */
/* shmds add 11-1 <- */
/* shmds add 2-1 -> */

/* shmds add 12-2 -> */
static ssize_t shmds_fwdl_comp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	return sprintf( buf, "%d\n", st->chip_config.shmds_fwdl_comp );
}

static ssize_t shmds_fwdl_comp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	if (kstrtoul(buf, 10, &data))
		return -EINVAL;
	st->chip_config.shmds_fwdl_comp = data;	

	return count;
}
/* shmds add 12-2 <- */

static ssize_t reference_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);
	
	return sprintf( buf, "%X %X\n",st->refreg.addr, st->refreg.reg );
}

static ssize_t reference_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	int retval = 0;
	unsigned char scanbuf[2];
	
	st = dev_get_drvdata(dev);
	
	retval = sysfs_read_value( scanbuf, buf, count, sizeof(scanbuf) );
	
	st->refreg.addr = scanbuf[0];
	st->refreg.reg = scanbuf[1];
	

	return retval;
}

static ssize_t rw_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	unsigned char data[1];
	int result = 0;
	
	st = dev_get_drvdata(dev);
	if( st->refreg.addr == st->plat_data.secondary_i2c_addr ) {
		result = inv_secondary_read( st->refreg.reg, 1, data );
	} else if( st->refreg.addr == st->i2c_addr ) {
		result = inv_i2c_read(st, st->refreg.reg, 1, data);
	} else {
		printk(KERN_ERR "unknown slave addrs.\n");
		return result;
	}
	
	if (result) {
		printk(KERN_ERR "Could not read registers.\n");
		return result;
	}
	
	return sprintf(buf, "%X\n",	(signed short)data[0]);
}

static ssize_t rw_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	int result = 0;
	unsigned char scanbuf[1];
	
	
	result = sysfs_read_value( scanbuf, buf, count, sizeof(scanbuf) );
	
	st = dev_get_drvdata(dev);
	
	if( st->refreg.addr == st->plat_data.secondary_i2c_addr ) {
		result = inv_secondary_write( st->refreg.reg, scanbuf[0] );
	} else if( st->refreg.addr == st->i2c_addr ) {
		result = inv_i2c_single_write(st, st->refreg.reg, scanbuf[0] );
	} else {
		printk(KERN_ERR "unknown slave addrs.\n");
		return result;
	}
	
	if(result) {
		return -EINVAL;
	}
	
	return count;
}

static ssize_t port_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	
	unsigned char gpio_state[1];
	gpio_state[0] = gpio_get_value(GPIO_GYRO_INT);
	
	return sprintf( buf, "%d\n", (signed short)gpio_state[0] );
}
/* shmds add 2-1 <- */

/**
 *  inv_raw_gyro_show() - Read gyro data directly from registers.
 */
static ssize_t inv_raw_gyro_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data[BYTES_PER_SENSOR];

	st = dev_get_drvdata(dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (0 == st->chip_config.gyro_enable)
		return -EPERM;

	result = inv_i2c_read(st, reg->raw_gyro, BYTES_PER_SENSOR, data);
	if (result) {
		printk(KERN_ERR "Could not read raw registers.\n");
		return result;
	}

	return sprintf(buf, "%d %d %d %lld\n",
		(signed short)(be16_to_cpup((short *)&data[0])),
		(signed short)(be16_to_cpup((short *)&data[2])),
		(signed short)(be16_to_cpup((short *)&data[4])),
		get_time_ns());
}
/**
 *  inv_raw_accl_show() - Read accel data directly from registers.
 */
static ssize_t inv_raw_accl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	short out[THREE_AXIS];
	unsigned char data[BYTES_PER_SENSOR];

	st = dev_get_drvdata(dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (0 == st->chip_config.accl_enable)
		return -EPERM;
	if (INV_MPU3050 == st->chip_type) {
		if (0 == st->mpu_slave->get_mode(st))
			return -EINVAL;
		result = inv_i2c_read(st, REG_3050_AUX_XOUT_H,
				BYTES_PER_SENSOR, data);
		out[0] = out[1] = out[2] = 0;
		if (0 == result)
			st->mpu_slave->combine_data(data, out);
		return sprintf(buf, "%d %d %d %lld\n", out[0],
			out[1], out[2], get_time_ns());

	}
	result = inv_i2c_read(st, reg->raw_accl, BYTES_PER_SENSOR, data);
	if (result) {
		printk(KERN_ERR "Could not read raw registers.\n");
		return result;
	}
	return sprintf(buf, "%d %d %d %lld\n",
		((signed short)(be16_to_cpup((short *)&data[0]))*
				st->chip_info.multi),
		((signed short)(be16_to_cpup((short *)&data[2]))*
				st->chip_info.multi),
		((signed short)(be16_to_cpup((short *)&data[4]))*
				st->chip_info.multi),
		get_time_ns());
}

/* shmds add 2-1 -> */
static ssize_t raw_compass_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	int result;
	unsigned char data[8];
	unsigned char *stat = &data[0];
	unsigned char *stat2 = &data[7];
	int status = 0;
	
	st = dev_get_drvdata(dev);

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (0 == st->chip_config.compass_enable)
		return -EPERM;
			
	result = inv_secondary_read( REG_AKM_STATUS, 8, data );
	if (result) {
		printk(KERN_ERR "Could not read raw registers.\n");
		return result;
	}
	
	if (*stat & 0x01)
		status = 0;
		
	if (*stat2 & 0x04)
		status = 0x04;
		
	if (*stat2 & 0x08)
		status = 0x08;
	
	if (*stat & 0x02) {
		/* status = INV_ERROR_COMPASS_DATA_UNDERFLOW; */
		status = 0;
	}
	
	if (status)
		pr_err("%s, line=%d, status=%d\n", __func__, __LINE__, status);
	
	if (*stat != 0x00 || *stat2 != 0x00) {
		result = inv_secondary_write(REG_AKM8963_CNTL1, 0x01);
		if (result) {
			pr_err("%s, line=%d\n", __func__, __LINE__);
		}
	}
	
	return sprintf(buf, "%d %d %d %lld\n",
		(signed short)(data[2] << 8) | data[1],
		(signed short)(data[4] << 8) | data[3],
		(signed short)(data[6] << 8) | data[5],
		get_time_ns());
}

static ssize_t compass_self_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	
	int result;
	int bias[3];
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned char *sens;
	
	if (st->chip_config.is_asleep)
		return -EPERM;
	if (0 == st->chip_config.compass_enable)
		return -EPERM;
		
	sens = st->chip_info.compass_sens;
	
	result = compass_hw_self_test(st, bias);
	return sprintf(buf, "%d %d %d %d %d %d %d\n",
		sens[0], sens[1], sens[2], bias[0], bias[1], bias[2], result);
		
}
/* shmds add 2-1 <- */
/**
 *  inv_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t inv_temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	short temp;
	long scale_t;
	unsigned char data[2];

	st = dev_get_drvdata(dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (result) {
		printk(KERN_ERR "Could not read temperature register.\n");
		return result;
	}
	temp = (signed short)(be16_to_cpup((short *)&data[0]));

	if (INV_MPU3050 == st->chip_type)
		scale_t = MPU3050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU3050_TEMP_SCALE);
	else
		scale_t = MPU6050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU6050_TEMP_SCALE);
	return sprintf(buf, "%ld %lld\n", scale_t, get_time_ns());
}

static int inv_set_lpf(struct inv_gyro_state_s *st, int rate)
{
	const short hz[] = {188, 98, 42, 20, 10, 5};
	const int   d[] = {INV_FILTER_188HZ, INV_FILTER_98HZ,
			INV_FILTER_42HZ, INV_FILTER_20HZ,
			INV_FILTER_10HZ, INV_FILTER_5HZ};
	int i, h, data, result;
	struct inv_reg_map_s *reg;
	reg = &st->reg;
	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d)))
		i++;
	if (i == ARRAY_SIZE(d))
		i -= 1;
	data = d[i];
	if (INV_MPU3050 == st->chip_type) {
		if (st->mpu_slave != NULL) {
			result = st->mpu_slave->set_lpf(st, rate);
			if (result)
				return result;
		}
		result = inv_i2c_single_write(st, reg->lpf,
			data | (st->chip_config.fsr << 3));
		if (result)
			return result;
	} else
		result = inv_i2c_single_write(st, reg->lpf, data);
	if (result)
		return result;
	st->chip_config.lpf = data;
	return 0;
}

/**
 *  inv_fifo_rate_store() - Set fifo rate.
 */
static ssize_t inv_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fifo_rate;
	unsigned char data;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	int i;
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */
	st = dev_get_drvdata(dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (kstrtoul(buf, 10, &fifo_rate))
		return -EINVAL;
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	if( fifo_rate == 1 || fifo_rate == 2 ) {
		
		if( fifo_rate == 1 ) {
			fifo_rate = SHMDS_DETECT_RATE1;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE1;
		} else {
			fifo_rate = SHMDS_DETECT_RATE2;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE2;
		}
		shmds_detect_switch		= DETECT_ON;
		shmds_count				= 0;
		shmds_full_buffer_flg	= 0;
		atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_NORMAL);
		if ( shmds_buf_size == shmds_buf_size_temp ) {
			for (i = 0; i < 3; i++) {
				memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
			}
		}
	} else if ( st->chip_config.dmp_on == 0 ) {
		shmds_detect_switch = DETECT_OFF;
		mutex_lock(&shmds_timerlock);
		del_timer_sync(&timer_acc);
		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
		mutex_unlock(&shmds_timerlock);
	}
#else /* SHMDS_DETECT */
	if( fifo_rate == 1 ) {
		fifo_rate = SHMDS_DETECT_RATE1;
	} else if ( fifo_rate == 2 ) {
		fifo_rate = SHMDS_DETECT_RATE2;
	}
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */

	if ((fifo_rate < MIN_FIFO_RATE) || (fifo_rate > MAX_FIFO_RATE))
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;
	if (st->has_compass) {
		data = COMPASS_RATE_SCALE*fifo_rate/ONE_K_HZ;
		if (data > 0)
			data -= 1;
		st->compass_divider = data;
		st->compass_counter = 0;
		/* I2C_MST_DLY is set according to sample rate,
		   AKM cannot be read or set at sample rate higher than 100Hz*/
		result = inv_i2c_single_write(st, REG_I2C_SLV4_CTRL, data);
		if (result)
			return result;
	}
	data = ONE_K_HZ / fifo_rate - 1;
	result = inv_i2c_single_write(st, reg->sample_rate_div, data);
	if (result)
		return result;
	st->chip_config.fifo_rate = fifo_rate;
	result = inv_set_lpf(st, fifo_rate);
	if (result)
		return result;
	st->irq_dur_us = (data + 1) * ONE_K_HZ;
	st->last_isr_time = get_time_ns();
	return count;
}

/**
 *  inv_power_state_store() - Turn device on/off.
 */
static ssize_t inv_power_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result;
	unsigned long power_state;
	struct inv_gyro_state_s *st;

/* shmds add 17-2 -> */
	shmds_wake_lock_start();
/* shmds add 17-2 <- */
/* shmds add 12-4 -> */
    shmds_qos_start();
/* shmds add 12-4 <- */

	st = dev_get_drvdata(dev);
    if (kstrtoul(buf, 10, &power_state)){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return -EINVAL;
    }
    if (!power_state == st->chip_config.is_asleep){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return count;
    }
	if ((power_state == 0) && st->chip_config.enable) {
		result = set_inv_enable(st, power_state);
	    if (result){
/* shmds add 12-4 -> */
            shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
            shmds_wake_lock_end();
/* shmds add 17-2 <- */
			return result;
	    }
	}
	result = inv_set_power_state(st, power_state);
    if (result){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return result;
    }
/* shmds add 12-4 -> */
    shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
    shmds_wake_lock_end();
/* shmds add 17-2 <- */
	return count;
}
/**
 *  inv_enable_store() - Enable/disable chip operation.
 */
static ssize_t inv_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable;
	struct inv_gyro_state_s *st;
	int result;

/* shmds add 17-2 -> */
	shmds_wake_lock_start();
/* shmds add 17-2 <- */
/* shmds add 12-4 -> */
    shmds_qos_start();
/* shmds add 12-4 <- */

	st = dev_get_drvdata(dev);
    if (st->chip_config.is_asleep){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return -EPERM;
    }

    if (kstrtoul(buf, 10, &enable)){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return -EINVAL;
    }
    if (!enable == !st->chip_config.enable){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return count;
    }
	result = set_inv_enable(st, enable);
    if (result){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return result;
    }
/* shmds add 12-4 -> */
    shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
    shmds_wake_lock_end();
/* shmds add 17-2 <- */
	return count;
}

static int inv_switch_gyro_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data, p;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (en) {
			data = INV_CLK_PLL;
			p = (BITS_3050_POWER1 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = (BITS_3050_POWER2 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = data;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		} else {
			p = BITS_3050_GYRO_STANDBY;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		}
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;

/* shmds mod 12-2 -> */
#if 0
		if (en)
			data &= (~BIT_PWR_GYRO_STBY);
		else
			data |= BIT_PWR_GYRO_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
/* shmds add 12-1 -> */
		usleep(SENSOR_UP_UTIME);
/*		msleep(SENSOR_UP_TIME); */
/* shmds add 12-1 <- */
#else
		if ( en ){
			if ( (data & BIT_PWR_GYRO_STBY) == BIT_PWR_GYRO_STBY ){
				data &= (~BIT_PWR_GYRO_STBY);
				result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
				if ( result ){
					return result;
				}
				usleep(SENSOR_UP_UTIME);
			}
		} else {
			if ( (data & BIT_PWR_GYRO_STBY) == 0 ){
				data |= BIT_PWR_GYRO_STBY;
				result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
				if ( result ){
					return result;
				}
			}
		}
#endif
/* shmds mod 12-2 <- */
	}
	if (en)
		st->chip_config.clk_src = INV_CLK_PLL;
	else
		st->chip_config.clk_src = INV_CLK_INTERNAL;

/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
	if (en)
	{
		memset(&current_gyro_data , 0x00 , sizeof(&current_gyro_data));
		memset(&last_gyro_data , 0x00 , sizeof(&last_gyro_data));
		shmds_write_flag[0] = 0;
		shmds_last_data_flag[0] = 0;
	}
	current_gyro_data.enable = en;
#endif /* SHMDS_CAF */
/* shmds add 9-1 <- */
	return 0;
}

static int inv_switch_accl_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (NULL == st->mpu_slave)
			return -EPERM;
		if (en)
			result = st->mpu_slave->resume(st);
		else
			result = st->mpu_slave->suspend(st);
		if (result)
			return result;
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;

/* shmds mod 12-2 -> */
#if 0
		if (en)
			data &= (~BIT_PWR_ACCL_STBY);
		else
			data |= BIT_PWR_ACCL_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
/* shmds add 12-1 -> */
		usleep(SENSOR_UP_UTIME);
/*		msleep(SENSOR_UP_TIME); */
/* shmds add 12-1 <- */
#else
		if ( en ){
			if ( (data & BIT_PWR_ACCL_STBY) == BIT_PWR_ACCL_STBY ){
				data &= (~BIT_PWR_ACCL_STBY);
				result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
				if ( result ){
					return result;
				}
				usleep(SENSOR_UP_UTIME);
			}
		} else {
			if ( (data & BIT_PWR_ACCL_STBY) == 0 ){
				data |= BIT_PWR_ACCL_STBY;
				result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
				if ( result ){
					return result;
				}
			}
		}
#endif
/* shmds mod 12-2 <- */
	}
/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
	if (en)
	{
		memset(&current_accl_data , 0x00 , sizeof(&current_accl_data));
		memset(&last_accl_data , 0x00 , sizeof(&last_accl_data));
		shmds_write_flag[1] = 0;
		shmds_last_data_flag[1] = 0;
	}
	current_accl_data.enable = en;
#endif /* SHMDS_CAF */
/* shmds add 9-1 <- */
	return 0;
}

/**
 *  inv_accl_fifo_enable_store() - Enable/disable accl fifo output.
 */
static ssize_t inv_accl_fifo_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, en;
	int result;
	struct inv_gyro_state_s *st;

/* shmds add 17-2 -> */
	shmds_wake_lock_start();
/* shmds add 17-2 <- */
/* shmds add 12-4 -> */
    shmds_qos_start();
/* shmds add 12-4 <- */
	st = dev_get_drvdata(dev);

/* shmds mod 12-4 -> */
    CHECK_ENABLE_EX
/* shmds mod 12-4 <- */
	en = !!data;
    if (en == st->chip_config.accl_fifo_enable){
/* shmds add 12-4 -> */
        shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
        shmds_wake_lock_end();
/* shmds add 17-2 <- */
		return count;
    }
	if (en && (0 == st->chip_config.accl_enable)) {
		result = inv_switch_accl_engine(st, en);
	    if (result){
/* shmds add 12-4 -> */
            shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
            shmds_wake_lock_end();
/* shmds add 17-2 <- */
			return result;
	    }
		st->chip_config.accl_enable = en;
	}

/* shmds add 7-1 -> */
#ifdef MPL_SHTERM_ENABLE
	shterm_k_set_info(SHTERM_INFO_ACCELE, (en)? 1 : 0 );
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */
	
	st->chip_config.accl_fifo_enable = en;
/* shmds add 12-4 -> */
    shmds_qos_end();
/* shmds add 12-4 <- */
/* shmds add 17-2 -> */
    shmds_wake_lock_end();
/* shmds add 17-2 <- */
	return count;
}
/**
 *  inv_gyro_fifo_enable_store() - Enable/disable gyro fifo output.
 */
static ssize_t inv_gyro_fifo_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, en;
	int result;
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);

	CHECK_ENABLE
	en = !!data;
	if (en == st->chip_config.gyro_fifo_enable)
		return count;
	if (en && (0 == st->chip_config.gyro_enable)) {
		result = inv_switch_gyro_engine(st, en);
		if (result)
			return result;
		st->chip_config.gyro_enable = en;
	}
	
/* shmds add 7-1 -> */
#ifdef MPL_SHTERM_ENABLE
	shterm_k_set_info(SHTERM_INFO_GYRO, (en)? 1 : 0 );
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */

	st->chip_config.gyro_fifo_enable = en;
	return count;
}

/**
 *  inv_gyro_enable_store() - Enable/disable gyro.
 */
static ssize_t inv_gyro_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, en;
	struct inv_gyro_state_s *st;
	int result;
	st = dev_get_drvdata(dev);

	CHECK_ENABLE
	en = !!data;
	if (en == st->chip_config.gyro_enable)
		return count;

/* shmds add 11-1 -> */
	if ( !en ){
		mutex_lock(&shmds_timerlock);
		del_timer_sync(&timer_acc);
		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
		mutex_unlock(&shmds_timerlock);
	}
/* shmds add 11-1 <- */
	result = inv_switch_gyro_engine(st, en);
	if (result)
		return result;
	st->chip_config.gyro_enable = en;
	if (0 == en)
		st->chip_config.gyro_fifo_enable = 0;
	return count;
}

/**
 *  inv_accl_enable_store() - Enable/disable accl.
 */
static ssize_t inv_accl_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, en;
	struct inv_gyro_state_s *st;
	int result;
	st = dev_get_drvdata(dev);

	CHECK_ENABLE
	en = !!data;
	if (en == st->chip_config.accl_enable)
		return count;

/* shmds add 11-1 -> */
	if ( !en ){
		mutex_lock(&shmds_timerlock);
		del_timer_sync(&timer_acc);
		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
		mutex_unlock(&shmds_timerlock);
	}
/* shmds add 11-1 <- */
		
	result = inv_switch_accl_engine(st, en);
	if (result)
		return result;
	st->chip_config.accl_enable = en;
	if (0 == en)
		st->chip_config.accl_fifo_enable = 0;
	return count;
}

/**
 *  inv_gyro_fs_store() - Change the gyro full-scale range (and scale factor).
 */
static ssize_t inv_gyro_fs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fsr;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	st = dev_get_drvdata(dev);

	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &fsr);
	if (result)
		return -EINVAL;
	if (fsr > MAX_GYRO_FS_PARAM)
		return -EINVAL;
	if (fsr == st->chip_config.fsr)
		return count;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type)
		result = inv_i2c_single_write(st, reg->lpf,
			(fsr << GYRO_CONFIG_FSR_SHIFT) | st->chip_config.lpf);
	else
		result = inv_i2c_single_write(st, reg->gyro_config,
			fsr << GYRO_CONFIG_FSR_SHIFT);
	if (result)
		return result;
	st->chip_config.fsr = fsr;
	return count;
}

/**
 *  inv_accl_fs_store() - Configure the accelerometer's scale range.
 */
static ssize_t inv_accl_fs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fs;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	st = dev_get_drvdata(dev);

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (kstrtoul(buf, 10, &fs))
		return -EINVAL;
	if (fs > MAX_ACCL_FS_PARAM)
		return -EINVAL;
	if (fs == st->chip_config.accl_fs)
		return count;

	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type)
		result = st->mpu_slave->set_fs(st, fs);
	else
		result = inv_i2c_single_write(st, reg->accl_config,
				(fs << ACCL_CONFIG_FSR_SHIFT));
	if (result)
		return result;
	/* reset fifo because the data could be mixed with old bad data */
	st->chip_config.accl_fs = fs;
	return count;
}

/**
 * inv_firmware_loaded_store() -  calling this function will change
 *                        firmware load
 */
static ssize_t inv_firmware_loaded_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long data, result;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data != 0)
		return -EINVAL;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.dmp_on = 0;
/* shmds add 12-2 -> */
	st->chip_config.shmds_fwdl_comp = 0;
/* shmds add 12-2 <- */
	return count;
}
/**
 *  inv_lpa_mode_store() - store current low power settings
 */
static ssize_t inv_lpa_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, lpa_mode;
	unsigned char d;
	struct inv_reg_map_s *reg;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_mode);
	if (result)
		return result;

	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_1, 1, &d);
	if (result)
		return result;
	d &= ~BIT_CYCLE;
	if (lpa_mode)
		d |= BIT_CYCLE;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, d);
	if (result)
		return result;
	st->chip_config.lpa_mode = lpa_mode;
	return count;
}
/**
 *  inv_lpa_freq_store() - store current low power frequency setting.
 */
static ssize_t inv_lpa_freq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, lpa_freq;
	unsigned char d;
	struct inv_reg_map_s *reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_freq);
	if (result)
		return result;
	if (lpa_freq > MAX_LPA_FREQ_PARAM)
		return -EINVAL;
	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &d);
	if (result)
		return result;
	d &= ~BIT_LPA_FREQ;
	d |= (unsigned char)(lpa_freq << LPA_FREQ_SHIFT);
	result = inv_i2c_single_write(st, reg->pwr_mgmt_2, d);
	if (result)
		return result;
	st->chip_config.lpa_freq = lpa_freq;
	return count;
}

/**
 * inv_compass_en_store() -  calling this function will store compass
 *                         enable
 */
static ssize_t inv_compass_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	unsigned long data, result, en;
	st = dev_get_drvdata(dev);
	CHECK_ENABLE
	en = !!data;
	if (en == st->chip_config.compass_enable)
		return count;
/* shmds add 11-1 -> */
	if ( !en ){
		mutex_lock(&shmds_timerlock);
		del_timer_sync(&timer_acc);
		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
		mutex_unlock(&shmds_timerlock);
	}
/* shmds add 11-1 <- */

/* shmds add 7-1 -> */
#ifdef MPL_SHTERM_ENABLE
	shterm_k_set_info(SHTERM_INFO_COMPS, (en)? 1 : 0 );
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */
	st->chip_config.compass_enable = en;

	return count;
}

/**
 *  inv_compass_scale_store() - show current compass scale settings
 */
static ssize_t inv_compass_scale_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	unsigned long data, result, en;
	char d;
	st = dev_get_drvdata(dev);
	if (COMPASS_ID_AK8963 != st->plat_data.sec_slave_id)
		return count;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		en = 1;
	else
		en = 0;
	if (st->compass_scale == en)
		return count;
	st->compass_scale = en;
	d = (1 | (st->compass_scale << AKM8963_SCALE_SHIFT));
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, d);
	if (result)
		return result;
	return count;
}

/**
 * inv_flick_lower_store() -  calling this function will store current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtol(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;

	result = mem_w_key(KEY_FLICK_LOWER, 4, p);
	if (result)
		return result;
	st->flick.lower = data;
	return count;
}
/**
 * inv_flick_upper_store() -  calling this function will store current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_UPPER, 4, p);
	if (result)
		return result;
	st->flick.upper = data;
	return count;
}
/**
 * inv_flick_counter_store() -  calling this function will store current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_COUNTER, 4, p);
	if (result)
		return result;
	st->flick.counter = data;

	return count;
}


/**
 * inv_flick_int_on_store() -  calling this function will store current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, data;
	unsigned char d[4];
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		/* Use interrupt to signal when gesture was observed */
		d[0] = DIND40+4;
	else
		d[0] = DINAA0+8;
	result = mem_w_key(KEY_CGNOTICE_INTR, 1, d);
	if (result)
		return result;
	st->flick.int_on = data;
	return count;
}
/**
 * inv_flick_axis_store() -  calling this function will store current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, data;
	unsigned char d[4];
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data == 0)
		d[0] = DINBC2;
	else if (data == 2)
		d[2] = DINBC6;
	else
		d[0] = DINBC4;
	result = mem_w_key(KEY_CFG_FLICK_IN, 1, d);
	if (result)
		return result;
	st->flick.axis = data;

	return count;
}

/**
 * inv_flick_msg_on_store() -  calling this function will store current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	if (data)
		data = DATA_MSG_ON;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_MSG, 4, p);
	if (result)
		return result;
	st->flick.msg_on = data;

	return count;
}

/**
 * inv_pedometer_steps_store() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_STEPCTR, 4, p);
	if (result)
		return result;

/* shmds del 16-1 -> */
#if 0
	/* Initialize DMP internal variables */
	result = inv_reset_dmp_pedometer(st);
	if (result)
		return result;
#endif
/* shmds del 16-1 <- */
	
	return count;
}
/**
 * inv_pedometer_time_store() -  calling this function will store current
 *                        pedometer time into MPU memory
 */
static ssize_t inv_pedometer_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	data /= 20;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_TIMECTR, 4, p);
	if (result)
		return result;

	data = 0;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mpu_memory_write(st->sl_handle, st->i2c_addr, (768 + 0xc8), 4, p);
	if (result)
		return result;

	return count;
}

/**
 * inv_pedometer_params_write() -  calling this function will write current
 *                        pedometer params into MPU memory
 *                        TODO: need to separate this into 7 sysfs entries
 */
static int inv_pedometer_params_write(struct device *dev)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int outi;
	unsigned short outs;
	unsigned char *p;
	int result = 0;

	/* Pedmeter Parameters */	
	unsigned int threshold = 25000000U;
	unsigned int clipThreshold = 0x04000000U;
	unsigned int minEnergy = 0x0c000000U;
	unsigned short minUpTime = 16;
	unsigned short maxUpTime = 60;
	unsigned short minSteps = 5;
	unsigned short maxStepBufferTime = 150;

	/* write clipThreshold */
	outi = cpu_to_be32p(&clipThreshold);
	p = (unsigned char *)&outi;
	result = mem_w_key(KEY_D_PEDSTD_CLIP, 4, p);
	if (result)
		return result;

	/* write minSteps */
	outs = cpu_to_be16p(&minSteps);
	p = (unsigned char *)&outs;
	result = mem_w_key(KEY_D_PEDSTD_SB, 2, p);
	if (result)
		return result;

	/* write maxStepBufferTime */	
	outs = cpu_to_be16p(&maxStepBufferTime);
	p = (unsigned char *)&outs;
	result = mem_w_key(KEY_D_PEDSTD_SB_TIME, 2, p);
	if (result)
		return result;

	/* write threshold */
	outi = cpu_to_be32p(&threshold);
	p = (unsigned char *)&outi;
	result = mem_w_key(KEY_D_PEDSTD_PEAKTHRSH, 4, p);
	if (result)
		return result;

	/* write minUpTime */
	outs = cpu_to_be16p(&minUpTime);
	p = (unsigned char *)&outs;
	result = mem_w_key(KEY_D_PEDSTD_TIML, 2, p);
	if (result)
		return result;

	/* write maxUpTime */
	outs = cpu_to_be16p(&maxUpTime);
	p = (unsigned char *)&outs;
	result = mem_w_key(KEY_D_PEDSTD_TIMH, 2, p);
	if (result)
		return result;

	/* write minEnergy */
	outi = cpu_to_be32p(&minEnergy);
	p = (unsigned char *)&outi;
	result = mem_w_key(KEY_D_PEDSTD_INT_THRSH, 4, p);
	if (result)
		return result;

	return result;
}

/**
 * inv_dmp_on_store() -  calling this function will store current dmp on
 */
static ssize_t inv_dmp_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
/* shmds add 7-1 -> */
#ifdef MPL_SHTERM_ENABLE
	unsigned long en;
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */
	unsigned char orig_dmp_on;

	orig_dmp_on = st->chip_config.dmp_on;

	CHECK_DMP
	
/* shmds add 7-1 -> */
#ifdef MPL_SHTERM_ENABLE
	en = !!data;
	if( en != st->chip_config.dmp_on ) {
		shterm_k_set_info(SHTERM_INFO_PEDOMETER, (en)? 1 : 0 );
	}
#endif /* MPL_SHTERM_ENABLE */
/* shmds add 7-1 <- */
	st->chip_config.dmp_on = !!data;
	
	/* Update Pedometer parameters in DMP */
	if ((st->chip_config.dmp_on != orig_dmp_on) && st->chip_config.dmp_on) {
		result = inv_pedometer_params_write(dev);
		if (result)
			return result;
		st->dmp_enabled = 1;
/* shmds add 16-1 -> */
		/* Initialize DMP internal variables */
		result = inv_reset_dmp_pedometer(st);
		if (result)
			return result;
/* shmds add 16-1 <- */
	}
		

	return count;
}

/**
 * inv_dmp_int_on_store() -  calling this function will store current dmp int on
 */
static ssize_t inv_dmp_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
	CHECK_DMP
	st->chip_config.dmp_int_on = !!data;
	return count;
}

/**
 * inv_dmp_output_rate_store() -  calling this function store dmp_output_rate.
 *                                Valid values are > 0.
 */
static ssize_t inv_dmp_output_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
/* shmds mod 11-1 -> */
#ifdef SHMDS_DETECT
	int i;
#endif /* SHMDS_DETECT */
/* shmds mod 11-1 <- */
	CHECK_DMP
	if (data == 0)
		return -EINVAL;
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	if( data == 1 || data == 2 ) {
		if( data == 1 ) {
			data = SHMDS_DETECT_RATE1;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE1;
		} else {
			data = SHMDS_DETECT_RATE2;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE2;
		}
		shmds_detect_switch		= DETECT_ON;
		shmds_count				= 0;
		shmds_full_buffer_flg	= 0;
		atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_NORMAL);
		if ( shmds_buf_size == shmds_buf_size_temp ) {
			for (i = 0; i < 3; i++) {
				memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
			}
		}
	} else {
		shmds_detect_switch = DETECT_OFF;
		mutex_lock(&shmds_timerlock);
		del_timer_sync(&timer_acc);
		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
		mutex_unlock(&shmds_timerlock);
	}
#else /* SHMDS_DETECT */
	if ( data == 1 ) {
		data = SHMDS_DETECT_RATE1;
	} else if( data == 2 ) {
		data = SHMDS_DETECT_RATE2;
	}
#endif
/* shmds mod 11-1 <- */
	result = inv_set_fifo_rate(st, data);
	if (result)
		return result;
	if (st->has_compass) {
		st->compass_dmp_divider = COMPASS_RATE_SCALE * data /
			ONE_K_HZ;
		if (st->compass_dmp_divider > 0)
			st->compass_dmp_divider -= 1;
	}
	st->chip_config.dmp_output_rate = data;
	st->irq_dmp_dur_us = (ONE_K_HZ / data) * ONE_K_HZ;
	st->last_isr_time = get_time_ns();
	return count;
}

/**
 * inv_orientation_on_store() -  calling this function will store
 *                               current orientation on
 */
static ssize_t inv_orientation_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, en;
	CHECK_DMP
	en = !!data;
	result = inv_enable_orientation_dmp(st, en);
	if (result)
		return result;
	st->chip_config.orientation_on = en;
	return count;
}

/**
 * inv_tap_on_store() -  calling this function will store current tap on
 */
static ssize_t inv_tap_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
	CHECK_DMP
	st->tap.on = !!data;
	result = inv_enable_tap_dmp(st, st->tap.on);
	return count;
}

/**
 * inv_tap_time_store() -  calling this function will store current tap time
 */
static ssize_t inv_tap_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
	CHECK_DMP
	result = inv_set_tap_time_dmp(st, data);
	if (result)
		return result;
	st->tap.time = data;
	return count;
}

/**
 * inv_tap_min_count_store() -  calling this function will store tap count
 */
static ssize_t inv_tap_min_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
	CHECK_DMP
	result = inv_set_min_taps_dmp(st, data);
	if (result)
		return result;
	st->tap.min_count = data;
	return count;
}

/**
 * inv_tap_threshold_store() -  calling this function will store tap threshold
 */
static ssize_t inv_tap_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data;
	CHECK_DMP
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_X, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Y, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Z, data);
	if (result)
		return result;

	st->tap.thresh = data;
	return count;
}

/**
 * inv_key_store() -  calling this function will store authenticate key
 */
static ssize_t inv_key_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;
	CHECK_DMP
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_AUTH_IN, 4, p);
	if (result)
		return result;
	return count;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 *  inv_early_suspend_on_store() - set early_suspend_enable value
 */
static ssize_t inv_early_suspend_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	unsigned long data;
	int result;
	st = dev_get_drvdata(dev);
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	atomic_set(&(st->early_suspend_enable), !!data);
	return count;
}
#endif
/**
 *  inv_gyro_fs_show() - Get the current gyro full-scale range.
 */
static ssize_t inv_gyro_fs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (1 << st->chip_config.fsr)*GYRO_DPS_SCALE);
}
/**
 *  inv_accl_fs_show() - Get the current gyro full-scale range.
 */
static ssize_t inv_accl_fs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", 2 << st->chip_config.accl_fs);
}

/**
 *  inv_clk_src_show() - Show the device's clock source.
 */
static ssize_t inv_clk_src_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	switch (st->chip_config.clk_src) {
	case INV_CLK_INTERNAL:
		return sprintf(buf, "INTERNAL\n");
	case INV_CLK_PLL:
		return sprintf(buf, "Gyro PLL\n");
	default:
		return -EPERM;
	}
}
/**
 *  inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}
/**
 *  inv_enable_show() - Check if the chip are enabled.
 */
static ssize_t inv_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.enable);
}

/**
 *  inv_gyro_fifo_enable_show() - Check if gyro FIFO are enabled.
 */
static ssize_t inv_gyro_fifo_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.gyro_fifo_enable);
}
/**
 *  inv_accl_fifo_enable_show() - Check if accl FIFO are enabled.
 */
static ssize_t inv_accl_fifo_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.accl_fifo_enable);
}
/**
 *  inv_gyro_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_gyro_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
}
/**
 *  inv_accl_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_accl_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.accl_enable);
}

/**
 *  inv_power_state_show() - Check if the device is on or in sleep mode.
 */
static ssize_t inv_power_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "1\n");
}

/**
 *  inv_lpa_mode_show() - show current low power settings
 */
static ssize_t inv_lpa_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.lpa_mode);
}

/**
 *  inv_lpa_freq_show() - show current low power frequency setting
 */
static ssize_t inv_lpa_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	switch (st->chip_config.lpa_freq) {
	case 0:
		return sprintf(buf, "1.25\n");
	case 1:
		return sprintf(buf, "5\n");
	case 2:
		return sprintf(buf, "20\n");
	case 3:
		return sprintf(buf, "40\n");
	default:
		return sprintf(buf, "0\n");
	}
}

/**
 *  inv_compass_scale_show() - show current compass scale settings
 */
static ssize_t inv_compass_scale_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	long scale;
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id)
		scale = DATA_AKM8975_SCALE;
	else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id)
		scale = DATA_AKM8972_SCALE;
	else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id)
		if (st->compass_scale)
			scale = DATA_AKM8963_SCALE1;
		else
			scale = DATA_AKM8963_SCALE0;
	else
		return -EINVAL;
	scale *= (1L << 15);
	return sprintf(buf, "%ld\n", scale);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 *  inv_early_suspend_en_show() - show the current status of early_suspend_enable
 */
static ssize_t inv_early_suspend_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", atomic_read(&st->early_suspend_enable));
}
#endif

/**
 *  inv_reg_dump_show() - Register dump for testing.
 *  TODO: Only for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	ssize_t bytes_printed = 0;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	for (ii = 0; ii < st->hw->num_reg; ii++) {
		/* don't read fifo r/w register */
		if (ii == st->reg.fifo_r_w)
			data = 0;
		else
			inv_i2c_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	return bytes_printed;
}

/**
 * inv_self_test_show() - self test result. 0 for fail; 1 for success.
 *                        calling this function will trigger self test
 *                        and return test result.
 */
static ssize_t inv_self_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result;
	int bias[3];
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	if (INV_MPU3050 == st->chip_type) {
		bias[0] = bias[1] = bias[2] = 0;
		result = 0;
	} else
		result = inv_hw_self_test(st, bias);
	return sprintf(buf, "%d, %d, %d, %d\n",
		bias[0], bias[1], bias[2], result);
}
/**
 * inv_get_accl_bias_show() - show accl bias value
 */
static ssize_t inv_get_accl_bias_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result;
	int bias[3];
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	result = inv_get_accl_bias(st, bias);
	if (result)
		return -EINVAL;
	return sprintf(buf, "%d, %d, %d\n", bias[0], bias[1], bias[2]);
}

/**
 * inv_gyro_matrix_show() - show orientation matrix
 */
static ssize_t inv_gyro_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;
	m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_accl_matrix_show() - show orientation matrix
 */
static ssize_t inv_accl_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_ACCEL)
		m = st->plat_data.secondary_orientation;
	else
		m = st->plat_data.orientation;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_compass_matrix_show() - show orientation matrix
 */
static ssize_t inv_compass_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_COMPASS)
		m = st->plat_data.secondary_orientation;
	else
		return -1;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_key_show() -  calling this function will show the key
 *
 */
#if 0
static ssize_t inv_key_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data;
	unsigned char d[4];
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		D_AUTH_OUT, 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}
#else
static ssize_t inv_key_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned char *key;
	key = st->plat_data.key;

	return sprintf(buf,
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x\n",
		key[0],  key[1],  key[2],  key[3],
		key[4],  key[5],  key[6],  key[7],
		key[8],  key[9],  key[10], key[11],
		key[12], key[13], key[14], key[15]);
}
#endif
/**
 * inv_firmware_loaded_show() -  calling this function will show current
 *                        firmware load status
 */
static ssize_t inv_firmware_loaded_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
}
/**
 * inv_compass_en_show() -  calling this function will show compass
 *                         enable status
 */
static ssize_t inv_compass_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.compass_enable);
}
/**
 * inv_flick_lower_show() -  calling this function will show current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.lower);
}
/**
 * inv_flick_upper_show() -  calling this function will show current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.upper);
}
/**
 * inv_flick_counter_show() -  calling this function will show current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.counter);
}
/**
 * inv_flick_int_on_show() -  calling this function will show current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.int_on);
}
/**
 * inv_flick_axis_show() -  calling this function will show current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.axis);
}

/**
 * inv_flick_msg_on_show() -  calling this function will show current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.msg_on);
}
/**
 * inv_dmp_on_show() -  calling this function will show current dmp_on
 */
static ssize_t inv_dmp_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_on);
}
/**
 * inv_dmp_int_on_show() -  calling this function will show current dmp_int_on
 */
static ssize_t inv_dmp_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_int_on);
}

/**
 * inv_dmp_output_rate_show() -  calling this shows dmp_output_rate
 */
static ssize_t inv_dmp_output_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_output_rate);
}

/**
 * inv_orientation_on_show() -  calling this function will show
 *				current orientation_on
 */
static ssize_t inv_orientation_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", st->chip_config.orientation_on);
}

/**
 * inv_tap_on_show() -  calling this function will show current tap_on
 */
static ssize_t inv_tap_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->tap.on);
}

/**
 * inv_tap_time_show() -  calling this function will show current tap time
 */
static ssize_t inv_tap_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->tap.time);
}

/**
 * inv_tap_thresh_show() -  calling this function show current tap threshold
 */
static ssize_t inv_tap_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->tap.thresh);
}

/**
 * inv_tap_min_count_show() -  calling this function show minimum count
 */
static ssize_t inv_tap_min_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->tap.min_count);
}

/**
 * inv_pedometer_steps_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data;
	unsigned char d[4];
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_STEPCTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}

/**
 * inv_pedometer_time_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data;
	unsigned char d[4];
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_TIMECTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%lld\n", (long long)data * 20);
}

/* shmds add 11-1 -> */
static void shmds_get_center_data( signed short *c_data )
{
	int				i,j,k;
	signed short	CustodyData;
	signed short	buff[3][SHMDS_TMP_BUFSIZE];

	memcpy( buff, shmds_tmp_buf, sizeof(shmds_tmp_buf) );

	for ( i=0; i<3; i++ ){
		for ( j=0; j<SHMDS_TMP_BUFSIZE-1; j++ ){
			for ( k=0; k<( SHMDS_TMP_BUFSIZE-1-j ); k++ ){
				if ( buff[i][k] > buff[i][k+1] ){
					CustodyData = buff[i][k];
					buff[i][k] = buff[i][k+1];
					buff[i][k+1] = CustodyData;
				}
			}
		}
	}

	for ( i=0; i<3; i++ ){
		c_data[i] = buff[i][SHMDS_TMP_BUFSIZE/2];
	}
	return;
}
/* shmds add 11-1 <- */
static void inv_report_gyro_accl(struct inv_gyro_state_s *st, s64 t,
		unsigned char *data)
{
	short x, y, z;
	int ind;
	struct inv_chip_config_s *conf;
	
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	int i,j;
	unsigned char temp1, readbuff, writebuff;
	signed long sub[3] = {0,0,0};
	signed short max, min;
	short acclx = 0, accly = 0, acclz = 0;
	int result;
#endif /* SHMDS_DETECT */
	mutex_lock(&shmds_lock);
/* shmds add 11-1 <- */

	conf = &st->chip_config;
	ind = 0;
	if (conf->accl_fifo_enable | conf->dmp_on) {
		x =	be16_to_cpup((__be16 *)(&data[ind]));
		y =	be16_to_cpup((__be16 *)(&data[ind+2]));
		z =	be16_to_cpup((__be16 *)(&data[ind+4]));

		x *= st->chip_info.multi;
		y *= st->chip_info.multi;
		z *= st->chip_info.multi;

/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
		acclx = x;
		accly = y;
		acclz = z;
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */

/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
		/* accl */
		shmds_write_flag[1] = 1;
		if (shmds_last_data_flag[1] == 0)
		{
			current_accl_data.x = x;
			current_accl_data.y = y;
			current_accl_data.z = z;

			last_accl_data.x = x;
			last_accl_data.y = y;
			last_accl_data.z = z;

			shmds_last_data_flag[1] = 1;
		}
		else
		{
			last_accl_data.x = current_accl_data.x;
			last_accl_data.y = current_accl_data.y;
			last_accl_data.z = current_accl_data.z;
			
			current_accl_data.x = x;
			current_accl_data.y = y;
			current_accl_data.z = z;
		}
		shmds_write_flag[1] = 0;
#endif /* SHMDS_CAF */
/* shmds add 9-1 <- */

		if (conf->accl_fifo_enable) {
			/*it is possible that accl disabled when dmp is on*/
			input_report_rel(st->idev, REL_RX,  x);
			input_report_rel(st->idev, REL_RY,  y);
			input_report_rel(st->idev, REL_RZ,  z);
		}
		ind += 6;
	}
	if (conf->gyro_fifo_enable | conf->dmp_on) {
		x =	be16_to_cpup((__be16 *)(&data[ind]));
		y =	be16_to_cpup((__be16 *)(&data[ind+2]));
		z =	be16_to_cpup((__be16 *)(&data[ind+4]));

/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
		/* gyro */
		shmds_write_flag[0] = 1;
		if (shmds_last_data_flag[0] == 0)
		{
			current_gyro_data.x = x;
			current_gyro_data.y = y;
			current_gyro_data.z = z;

			last_gyro_data.x = x;
			last_gyro_data.y = y;
			last_gyro_data.z = z;

			shmds_last_data_flag[1] = 1;
		}
		else
		{
			last_gyro_data.x = current_gyro_data.x;
			last_gyro_data.y = current_gyro_data.y;
			last_gyro_data.z = current_gyro_data.z;
			
			current_gyro_data.x = x;
			current_gyro_data.y = y;
			current_gyro_data.z = z;
		}
		shmds_write_flag[0] = 0;
#endif /* SHMDS_CAF */
/* shmds add 9-1 <- */

		if (conf->gyro_fifo_enable) {
			/*it is possible that gyro disabled when dmp is on*/
			input_report_rel(st->idev, REL_X,  x);
			input_report_rel(st->idev, REL_Y,  y);
			input_report_rel(st->idev, REL_Z,  z);
		}
		ind += 6;
	}
	if (conf->dmp_on) {
		/* report tap information */
		if (data[ind + 1] & 1) {
			input_report_rel(st->idev_dmp, REL_RX, data[ind+3]);
			input_sync(st->idev_dmp);
		}
		/* report orientation information */
		if (data[ind + 1] & 2) {
			input_report_rel(st->idev_dmp, REL_RY, data[ind+2]);
			input_sync(st->idev_dmp);
		}
	}
	if (conf->accl_fifo_enable | conf->gyro_fifo_enable) {
		input_report_rel(st->idev, REL_MISC, (unsigned int)(t >> 32));
		input_report_rel(st->idev, REL_WHEEL,
			(unsigned int)(t & 0xffffffff));
		input_sync(st->idev);
	}

/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT

	if (shmds_buf_size != shmds_buf_size_temp){
		for (i = 0; i < 3; i++) {
			kfree(shmds_detect_buf[i]);
			shmds_detect_buf[i] = NULL;
		}
		
		for (i = 0; i < 3; i++) {
			shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size_temp * sizeof(signed short), GFP_KERNEL);
			if (NULL == shmds_detect_buf[i]) {
				for (j = 0; j < i; j ++) {
					kfree(shmds_detect_buf[j]);
				}
				pr_err("shmds_detect_buf is NULL");
				goto end_session;
			}
			memset(shmds_detect_buf[i], 0, shmds_buf_size_temp * sizeof(signed short));
		}
		shmds_buf_size = shmds_buf_size_temp;
	}

	if (shmds_detect_switch == DETECT_ON) {
	
		if ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_NORMAL ) {
			if (( !shmds_count ) && ( !shmds_full_buffer_flg )){
				for ( i=0; i<SHMDS_TMP_BUFSIZE; i++ ){
					shmds_tmp_buf[0][i] = acclx;
					shmds_tmp_buf[1][i] = accly;
					shmds_tmp_buf[2][i] = acclz;
				}
			} else {
				for ( i=0; i<( SHMDS_TMP_BUFSIZE-1 ); i++ ){
					shmds_tmp_buf[0][i] = shmds_tmp_buf[0][i+1];
					shmds_tmp_buf[1][i] = shmds_tmp_buf[1][i+1];
					shmds_tmp_buf[2][i] = shmds_tmp_buf[2][i+1];
				}
				shmds_tmp_buf[0][SHMDS_TMP_BUFSIZE-1] = acclx;
				shmds_tmp_buf[1][SHMDS_TMP_BUFSIZE-1] = accly;
				shmds_tmp_buf[2][SHMDS_TMP_BUFSIZE-1] = acclz;
			}

			shmds_get_center_data( center_data );

			shmds_detect_buf[0][shmds_count] = center_data[0];
			shmds_detect_buf[1][shmds_count] = center_data[1];
			shmds_detect_buf[2][shmds_count] = center_data[2];

			if ( (shmds_count+1) == shmds_buf_size ) {
				shmds_full_buffer_flg = 1;
			}
					
			if ( shmds_full_buffer_flg ) {
			
				for (i = 0; i < 3; i++) {
					max = -32768;
					min = 32767;
						
					for (j = 0; j < shmds_buf_size; j++) {
						if (shmds_detect_buf[i][j] > max)
							max = shmds_detect_buf[i][j];
						if (shmds_detect_buf[i][j] < min)
							min = shmds_detect_buf[i][j];
					}
					sub[i] = max - min;
				}
				if ((sub[0] <= shmds_md_thresh) && (sub[1] <= shmds_md_thresh) && (sub[2] <= shmds_md_thresh)) {
					result = inv_i2c_single_write(st, st->reg.mot_thr, shmds_ths_value);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					result = inv_i2c_single_write(st, st->reg.int_enable, BIT_MOT_EN);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					result = inv_i2c_single_write(st, st->reg.mot_dur, shmds_duration_value);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					result = inv_i2c_read(st, st->reg.accl_config, 1, &readbuff);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}
					writebuff = ( readbuff & 0xF8 );
					result = inv_i2c_single_write(st, st->reg.accl_config, writebuff);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}
					writebuff = ( readbuff | 0x07 );
					result = inv_i2c_single_write(st, st->reg.accl_config, writebuff);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_ECONOMIZE);
					shmds_economize_buf[0] = shmds_detect_buf[0][shmds_count];
					shmds_economize_buf[1] = shmds_detect_buf[1][shmds_count];
					shmds_economize_buf[2] = shmds_detect_buf[2][shmds_count];
					shmds_count = 0;
					shmds_full_buffer_flg = 0;
					for (i = 0; i < 3; i++) {
						memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
					}
					mutex_lock(&shmds_timerlock);
					shmds_start_timer();
					mutex_unlock(&shmds_timerlock);
					goto end_session;
				}
			}
			shmds_count = (shmds_count+1) % shmds_buf_size;

		} else if (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) {

			temp1	= 0x00;

			result = inv_i2c_read(st, st->reg.int_status, 1, &temp1);
			if(result) {
				pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
				goto end_session;
			}
			if( (temp1 & 0x40) != 0 ) {

				result = inv_i2c_single_write(st, st->reg.int_enable, BIT_DATA_RDY_EN);
				if (result) {
					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					goto end_session;
				}

				result = inv_i2c_read(st, st->reg.accl_config, 1, &readbuff);
				if (result) {
					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					goto end_session;
				}
				writebuff = ( readbuff & 0xF8 );
				result = inv_i2c_single_write(st, st->reg.accl_config, writebuff);
				if (result) {
					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					goto end_session;
				}

				atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);

				result = inv_reset_fifo(st);
				if (result){
					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					goto end_session;
				}
			} else {

				sub[0] = acclx;
				sub[1] = accly;
				sub[2] = acclz;
				
				for (i = 0; i < 3; i++) {
					sub[i] = abs(sub[i] - shmds_economize_buf[i]);
				}
				if ((sub[0] > shmds_md_thresh) || (sub[1] > shmds_md_thresh) || (sub[2] > shmds_md_thresh)) {

					result = inv_i2c_single_write(st, st->reg.int_enable, BIT_DATA_RDY_EN);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					result = inv_i2c_read(st, st->reg.accl_config, 1, &readbuff);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}
					writebuff = ( readbuff & 0xF8 );
					result = inv_i2c_single_write(st, st->reg.accl_config, writebuff);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}

					atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);

					result = inv_reset_fifo(st);
					if (result){
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
						goto end_session;
					}
				}
			}
		}
	}
end_session:
	mutex_unlock(&shmds_lock);
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */
}
static int inv_report_compass(struct inv_gyro_state_s *st, s64 t)
{
	short x, y, z;
	int result;
	unsigned char data[8], compass_divider;
	/*mpu_memory_read(st->sl_handle,
		st->i2c_addr,
		14,
		10, data);*/
	/*divider and counter is used to decrease the speed of read in
		high frequency sample rate*/
	if (st->chip_config.dmp_on)
		compass_divider = st->compass_dmp_divider;
	else
		compass_divider = st->compass_divider;
	if (compass_divider == st->compass_counter) {
		/*read from external sensor data register */
		result = inv_i2c_read(st, REG_EXT_SENS_DATA_00, 8, data);
		if (result)
			return result;
		/* data[7] is status 2 register */
		/*for AKM8975, bit 2 and 3 should be all be zero*/
		/* for AMK8963, bit 3 should be zero*/
		if ((DATA_AKM_DRDY == data[0]) &&
			(0 == (data[7] & DATA_AKM_STAT_MASK))) {
			unsigned char *sens;
			sens = st->chip_info.compass_sens;
			x = (short)((data[2] << 8) | data[1]);
			y = (short)((data[4] << 8) | data[3]);
			z = (short)((data[6] << 8) | data[5]);
			x = ((x * (sens[0] + 128)) >> 8);
			y = ((y * (sens[1] + 128)) >> 8);
			z = ((z * (sens[2] + 128)) >> 8);
			input_report_rel(st->idev_compass, REL_X, x);
			input_report_rel(st->idev_compass, REL_Y, y);
			input_report_rel(st->idev_compass, REL_Z, z);
			input_report_rel(st->idev_compass, REL_MISC,
					 (unsigned int)(t >> 32));
			input_report_rel(st->idev_compass, REL_WHEEL,
					 (unsigned int)(t & 0xffffffff));
			input_sync(st->idev_compass);
		}
		st->compass_counter = 0;
	} else if (st->compass_divider != 0)
		st->compass_counter++;
	return 0;
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
static irqreturn_t inv_read_fifo(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	unsigned char bytes_per_datum;
	int result;
	unsigned char data[16];
	unsigned short fifo_count;
	unsigned int copied;
	s64 timestamp;
	struct inv_reg_map_s *reg;
/* shmds add 17-4 -> */
	static s64 start_time = 0LL;
	static s64 end_time = 0LL;

	start_time = get_time_ns();
/* shmds add 17-4 <- */
	st = (struct inv_gyro_state_s *)dev_id;
	reg = &st->reg;

/* shmds add 17-2 -> */
	shmds_wake_lock_start();
/* shmds add 17-2 <- */
/* shmds add 17-1 -> */
	shmds_qos_start();
/* shmds add 17-1 <- */

	if (st->chip_config.is_asleep)
		goto end_session;
	if (!(st->chip_config.enable))
		goto end_session;
	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on |
		st->chip_config.compass_enable))
		goto end_session;
	if (st->chip_config.dmp_on && st->flick.int_on) {
		/*dmp interrupt status */
		inv_i2c_read(st, REG_DMP_INT_STATUS, 1, data);
		if (data[0] & 8) {
			input_report_rel(st->idev_dmp, REL_RZ, data[0]);
			input_sync(st->idev_dmp);
		}
	}
	if (st->chip_config.lpa_mode) {
		result = inv_i2c_read(st, reg->raw_accl, 6, data);
		if (result)
			goto end_session;
		inv_report_gyro_accl(st, get_time_ns(), data);
		goto end_session;
	}

	if (st->chip_config.dmp_on)
		bytes_per_datum = BYTES_FOR_DMP;
	else
		bytes_per_datum = (st->chip_config.accl_fifo_enable +
		st->chip_config.gyro_fifo_enable)*BYTES_PER_SENSOR;
	fifo_count = 0;
	if (bytes_per_datum != 0) {
/* shmds add 17-4 -> */
		if(end_time != 0LL && start_time - end_time < 100000LL)
		{
			usleep(1000);
	    }
/* shmds add 17-4 <- */
		result = inv_i2c_read(st, reg->fifo_count_h, 2, data);
		if (result)
			goto end_session;
		fifo_count = (data[0] << 8) + data[1];
/* shmds add 17-3 -> */
		if (fifo_count == 0){
			printk(KERN_DEBUG "[ms] fifo size = 0\n");
			goto flush_fifo;
		}
/* shmds add 17-3 <- */
/* shmds add 17-4 -> */
	    if (fifo_count > bytes_per_datum)
	    {
	        usleep(1000);
	    }
/* shmds add 17-4 -> */
		if (fifo_count < bytes_per_datum)
			goto end_session;
		if (fifo_count%2)
			goto flush_fifo;
		if (fifo_count > FIFO_THRESHOLD)
			goto flush_fifo;
		/* Timestamp mismatch. */
		if (kfifo_len(&st->trigger.timestamps) <
			fifo_count / bytes_per_datum)
			goto flush_fifo;
		if (kfifo_len(&st->trigger.timestamps) >
			fifo_count / bytes_per_datum + TIME_STAMP_TOR) {
			if (st->chip_config.dmp_on) {
				result = kfifo_to_user(&st->trigger.timestamps,
				&timestamp, sizeof(timestamp), &copied);
				if (result)
					goto flush_fifo;
			} else
				goto flush_fifo;
		}
	}

	if (bytes_per_datum == 0) {
		result = kfifo_to_user(&st->trigger.timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
	}
	while ((bytes_per_datum != 0) && (fifo_count >= bytes_per_datum)) {
		result = inv_i2c_read(st, reg->fifo_r_w, bytes_per_datum,
			data);
		if (result)
			goto flush_fifo;
		result = kfifo_to_user(&st->trigger.timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
		inv_report_gyro_accl(st, timestamp, data);
		fifo_count -= bytes_per_datum;
	}
	if (st->chip_config.compass_enable)
		inv_report_compass(st, timestamp);
end_session:
/* shmds add 17-1 -> */
	shmds_qos_end();
/* shmds add 17-1 <- */
/* shmds add 17-2 -> */
	shmds_wake_lock_end();
/* shmds add 17-2 <- */
/* shmds add 17-4 -> */
	end_time = get_time_ns();
/* shmds add 17-4 <- */
	return IRQ_HANDLED;
flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(st);
	inv_clear_kfifo(st);
/* shmds add 17-1 -> */
	shmds_qos_end();
/* shmds add 17-1 <- */
/* shmds add 17-2 -> */
	shmds_wake_lock_end();
/* shmds add 17-2 <- */
/* shmds add 17-4 -> */
	end_time = get_time_ns();
/* shmds add 17-4 <- */
	return IRQ_HANDLED;
}

/**
 *  inv_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
static irqreturn_t inv_irq_handler(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	long long timestamp;
	int result, catch_up;
	unsigned int time_since_last_irq;
	unsigned int irq_dur_us;
	
	st = (struct inv_gyro_state_s *)dev_id;
	
/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	if ( (shmds_detect_switch == DETECT_ON)
		 && ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) ) {
		schedule_work(&(st->work_timerhandler));
		return IRQ_HANDLED;
	}
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */
	
	timestamp = get_time_ns();
	time_since_last_irq = ((unsigned int)(timestamp
		- st->last_isr_time))/ONE_K_HZ;
	if (st->chip_config.dmp_on)
		irq_dur_us = st->irq_dmp_dur_us;
	else 
		irq_dur_us = st->irq_dur_us;
	spin_lock(&st->time_stamp_lock);
	catch_up = 0;
	while ((time_since_last_irq > irq_dur_us*2)
		&& (catch_up < MAX_CATCH_UP)
		&& (0 == st->chip_config.lpa_mode)) {

		st->last_isr_time += irq_dur_us * ONE_K_HZ;
		result = kfifo_in(&st->trigger.timestamps,
			&st->last_isr_time, 1);
		time_since_last_irq = ((unsigned int)(timestamp
			- st->last_isr_time)) / ONE_K_HZ;
		catch_up++;
	}
	result = kfifo_in(&st->trigger.timestamps, &timestamp, 1);
	st->last_isr_time = timestamp;
	spin_unlock(&st->time_stamp_lock);

/* shmds add 10-1 -> */
/* shmds mod 1-6 -> */
	if (st->trigger.irq == gpio_to_irq(GPIO_GYRO_INT)) {
/* shmds mod 1-6 <- */
		schedule_work(&(st->work));
	}
/* shmds add 10-1 <- */

	return IRQ_WAKE_THREAD;
}

/* shmds add 11-1 -> */
static void accl_timerhandler(u_long dev_id)
{
	struct inv_gyro_state_s *st;

	st = (struct inv_gyro_state_s *)dev_id;

	atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );

	schedule_work(&(st->work_timerhandler));
}

static void slaveirq_work_acc(struct work_struct *work)
{
	mutex_lock(&shmds_timerlock);
	shmds_start_timer();
	mutex_unlock(&shmds_timerlock);
}

void timer_work_acc( struct inv_gyro_state_s *st )
{
	unsigned char data[16];
	struct inv_reg_map_s *reg;

	mutex_lock(&shmds_timerlock);
	shmds_start_timer();
	mutex_unlock(&shmds_timerlock);

	if (st->chip_config.is_asleep)
		return;
	if (!(st->chip_config.enable))
		return;
	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on |
		st->chip_config.compass_enable))
		return;
		
	reg = &st->reg;
	inv_i2c_read(st, reg->raw_accl, 6, data);
	inv_report_gyro_accl(st, get_time_ns(), data);
	
}

static void timerhandler_work(struct work_struct *work)
{
	char gpio_data = 0;
	struct inv_gyro_state_s *st = container_of(work, struct inv_gyro_state_s, work_timerhandler); 

#ifdef SHMDS_DETECT
	if ( (shmds_detect_switch == DETECT_ON)
		 && ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) ) {

		timer_work_acc( st );
	}
/* shmds mod 1-6 -> */
	else if (st->trigger.irq == gpio_to_irq(GPIO_GYRO_INT)) {
/* shmds mod 1-6 <- */
#else
/* shmds mod 1-6 -> */
	if (st->trigger.irq == gpio_to_irq(GPIO_GYRO_INT)) {
/* shmds mod 1-6 <- */
#endif /* SHMDS_DETECT */
		gpio_data = gpio_get_value(GPIO_GYRO_INT);
		if (gpio_data == 1) {

			timer_work_acc( st );
		}
	}
}
/* shmds add 11-1 <- */


#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * inv_early_suspend - callback function for early_suspend
 */
static void inv_early_suspend(struct early_suspend *h)
{
	struct inv_reg_map_s *reg;
/* shmds del 12-3 -> */
#if 0
	unsigned char data;
	int result;
#endif
/* shmds del 12-3 <- */

	struct inv_gyro_state_s *st =
		container_of(h, struct inv_gyro_state_s, early_suspend);

/* shmds add 11-1 -> */
	mutex_lock(&shmds_lock);
	mutex_lock(&shmds_timerlock);
	del_timer_sync(&timer_acc);
	atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_ADD );
	mutex_unlock(&shmds_timerlock);
/* shmds add 11-1 <- */

	reg = &st->reg;
	if (atomic_read(&st->early_suspend_enable) != 1)
/* shmds add 11-1 -> */
		goto end_session;
#if 0
		return;
#endif
/* shmds add 11-1 <- */

/* shmds del 12-3 -> */
#if 0
	if (st->chip_config.dmp_on) {
		/* pedometer is running */
		if (st->chip_config.compass_enable) {
			result = inv_i2c_read(st, reg->user_ctrl, 1, &data);
			if (result)
				pr_debug("Cannot read user_ctrl register.\n");
			else {
				data &= (~BIT_I2C_MST_EN);
				result = inv_i2c_single_write(
						st, reg->user_ctrl, data);
			}
		}
		if (st->chip_config.gyro_enable)
			inv_switch_gyro_engine(st, 0);
	} else
		inv_switch_gyro_engine(st, 0);
#endif
/* shmds del 12-3 <- */
/* shmds add 11-1 -> */
end_session:
	mutex_unlock(&shmds_lock);
/* shmds add 11-1 <- */
}

/**
 * inv_late_resume - callback function for late_resume
 */
static void inv_late_resume(struct early_suspend *h)
{
	struct inv_reg_map_s *reg;
/* shmds del 12-3 -> */
#if 0
	unsigned char data;
	int result;
#endif
/* shmds del 12-3 <- */

	struct inv_gyro_state_s *st =
		container_of(h, struct inv_gyro_state_s, early_suspend);

	reg = &st->reg;
	if (atomic_read(&st->early_suspend_enable) != 1)
		return;

/* shmds del 12-3 -> */
#if 0
	if (st->chip_config.dmp_on) {
		/* pedometer is running */
		if (st->chip_config.compass_enable) {
			result = inv_i2c_read(st, reg->user_ctrl, 1, &data);
			if (result)
				pr_debug("Cannot read user_ctrl register.\n");
			else {
				data |= BIT_I2C_MST_EN;
				result = inv_i2c_single_write(
						st, reg->user_ctrl, data);
			}
		}
		if (st->chip_config.gyro_enable)
			inv_switch_gyro_engine(st, 1);
	} else
		inv_switch_gyro_engine(st, 1);
#endif
/* shmds del 12-3 <- */
}
#endif

/* shmds mod 13-1 -> */
static DEVICE_ATTR(raw_gyro, 0444, inv_raw_gyro_show, NULL);
static DEVICE_ATTR(raw_accl, 0444, inv_raw_accl_show, NULL);
static DEVICE_ATTR(temperature, 0444, inv_temperature_show, NULL);
static DEVICE_ATTR(fifo_rate, 0664, inv_fifo_rate_show,
	inv_fifo_rate_store);
static DEVICE_ATTR(enable, 0664, inv_enable_show,
	inv_enable_store);
static DEVICE_ATTR(gyro_fifo_enable, 0664,
	inv_gyro_fifo_enable_show,
	inv_gyro_fifo_enable_store);
static DEVICE_ATTR(gyro_enable, 0664, inv_gyro_enable_show,
	inv_gyro_enable_store);
static DEVICE_ATTR(accl_fifo_enable, 0664,
	inv_accl_fifo_enable_show,
	inv_accl_fifo_enable_store);
static DEVICE_ATTR(accl_enable, 0664, inv_accl_enable_show,
	inv_accl_enable_store);
static DEVICE_ATTR(accl_fs, 0664, inv_accl_fs_show,
			inv_accl_fs_store);
static DEVICE_ATTR(gyro_fs, 0664, inv_gyro_fs_show,
			inv_gyro_fs_store);
static DEVICE_ATTR(clock_source, 0444, inv_clk_src_show, NULL);
static DEVICE_ATTR(power_state, 0664, inv_power_state_show,
	inv_power_state_store);
static DEVICE_ATTR(firmware_loaded, 0664,
	inv_firmware_loaded_show, inv_firmware_loaded_store);
static DEVICE_ATTR(lpa_mode, 0664, inv_lpa_mode_show,
	inv_lpa_mode_store);
static DEVICE_ATTR(lpa_freq, 0664, inv_lpa_freq_show,
	inv_lpa_freq_store);
static DEVICE_ATTR(compass_enable, 0664, inv_compass_en_show,
	inv_compass_en_store);
static DEVICE_ATTR(compass_scale, 0664, inv_compass_scale_show,
	inv_compass_scale_store);
static DEVICE_ATTR(reg_dump, 0444, inv_reg_dump_show, NULL);
static DEVICE_ATTR(self_test, 0444, inv_self_test_show, NULL);
static DEVICE_ATTR(key, 0664, inv_key_show, inv_key_store);
static DEVICE_ATTR(gyro_matrix, 0444, inv_gyro_matrix_show, NULL);
static DEVICE_ATTR(accl_matrix, 0444, inv_accl_matrix_show, NULL);
static DEVICE_ATTR(compass_matrix, 0444, inv_compass_matrix_show, NULL);
static DEVICE_ATTR(accl_bias, 0444, inv_get_accl_bias_show, NULL);
static DEVICE_ATTR(flick_lower, 0664, inv_flick_lower_show,
	inv_flick_lower_store);
static DEVICE_ATTR(flick_upper, 0664, inv_flick_upper_show,
	inv_flick_upper_store);
static DEVICE_ATTR(flick_counter, 0664, inv_flick_counter_show,
	inv_flick_counter_store);
static DEVICE_ATTR(flick_message_on, 0664, inv_flick_msg_on_show,
	inv_flick_msg_on_store);
static DEVICE_ATTR(flick_int_on, 0664, inv_flick_int_on_show,
	inv_flick_int_on_store);
static DEVICE_ATTR(flick_axis, 0664, inv_flick_axis_show,
	inv_flick_axis_store);
static DEVICE_ATTR(dmp_on, 0664, inv_dmp_on_show,
	inv_dmp_on_store);
static DEVICE_ATTR(dmp_int_on, 0664, inv_dmp_int_on_show,
	inv_dmp_int_on_store);
static DEVICE_ATTR(dmp_output_rate, 0664,
	inv_dmp_output_rate_show, inv_dmp_output_rate_store);
static DEVICE_ATTR(orientation_on, 0664,
	inv_orientation_on_show, inv_orientation_on_store);
static DEVICE_ATTR(tap_on, 0664, inv_tap_on_show,
	inv_tap_on_store);
static DEVICE_ATTR(tap_time, 0664, inv_tap_time_show,
	inv_tap_time_store);
static DEVICE_ATTR(tap_min_count, 0664, inv_tap_min_count_show,
	inv_tap_min_count_store);
static DEVICE_ATTR(tap_threshold, 0664, inv_tap_threshold_show,
	inv_tap_threshold_store);
static DEVICE_ATTR(pedometer_time, 0664, inv_pedometer_time_show,
	inv_pedometer_time_store);
static DEVICE_ATTR(pedometer_steps, 0664,
		inv_pedometer_steps_show, inv_pedometer_steps_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static DEVICE_ATTR(early_suspend_enable, 0664,
	inv_early_suspend_en_show,
	inv_early_suspend_en_store);
#endif
/* shmds mod 13-1 <- */
/* shmds add 2-1 11-1 13-1 -> */
static DEVICE_ATTR(reference_register, 0644, reference_register_show, reference_register_store);
static DEVICE_ATTR(rw_register, 0644, rw_register_show, rw_register_store);
static DEVICE_ATTR(port_state, 0444, port_state_show, NULL);
static DEVICE_ATTR(raw_compass, 0444, raw_compass_show, NULL);
static DEVICE_ATTR(compass_self_test, 0444, compass_self_test_show, NULL);
#if defined(SHMDS_DETECT) && defined(SHMDS_ADB_FLAG)
static DEVICE_ATTR(shmds_count, 0444, shmds_count_show, NULL);
static DEVICE_ATTR(shmds_ths_value, 0644, shmds_ths_value_show, shmds_ths_value_store);
static DEVICE_ATTR(shmds_duration_value, 0644, shmds_duration_value_show, shmds_duration_value_store);
static DEVICE_ATTR(shmds_buf_size_temp, 0644, shmds_buf_size_temp_show, shmds_buf_size_temp_store);
static DEVICE_ATTR(shmds_md_thresh, 0644, shmds_md_thresh_show, shmds_md_thresh_store);
static DEVICE_ATTR(shmds_mg_to_LSB, 0644, shmds_mg_to_LSB_show, shmds_mg_to_LSB_store);
static DEVICE_ATTR(shmds_md_thresh_value, 0644, shmds_md_thresh_value_show, shmds_md_thresh_value_store);
#endif /* SHMDS_DETECT, SHMDS_ADB_FLAG */
/* shmds add 2-1 11-1 13-1 <- */
/* shmds add 12-2 -> */
static DEVICE_ATTR(shmds_fwdl_comp, 0644, shmds_fwdl_comp_show, shmds_fwdl_comp_store);
/* shmds add 12-2 <- */

static struct device_attribute *inv_attributes[] = {
	&dev_attr_raw_gyro,
	&dev_attr_temperature,
	&dev_attr_fifo_rate,
	&dev_attr_enable,
	&dev_attr_clock_source,
	&dev_attr_power_state,
	&dev_attr_gyro_fs,
	&dev_attr_reg_dump,
	&dev_attr_self_test,
	&dev_attr_key,
	&dev_attr_gyro_matrix,
#ifdef CONFIG_HAS_EARLYSUSPEND
	&dev_attr_early_suspend_enable,
#endif
	NULL
};
static struct device_attribute *inv_mpu6050_attributes[] = {
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_enable,
	&dev_attr_accl_fs,
	&dev_attr_accl_bias,
	&dev_attr_raw_accl,
	&dev_attr_accl_matrix,
	&dev_attr_firmware_loaded,
	&dev_attr_lpa_mode,
	&dev_attr_lpa_freq,
	&dev_attr_flick_lower,
	&dev_attr_flick_upper,
	&dev_attr_flick_counter,
	&dev_attr_flick_message_on,
	&dev_attr_flick_int_on,
	&dev_attr_flick_axis,
	&dev_attr_dmp_on,
	&dev_attr_dmp_int_on,
	&dev_attr_dmp_output_rate,
	&dev_attr_orientation_on,
	&dev_attr_tap_on,
	&dev_attr_tap_time,
	&dev_attr_tap_min_count,
	&dev_attr_tap_threshold,
	&dev_attr_pedometer_time,
	&dev_attr_pedometer_steps,
/* shmds add 2-1 11-1 -> */
	&dev_attr_reference_register,
	&dev_attr_rw_register,
	&dev_attr_port_state,
#if defined(SHMDS_DETECT) && defined(SHMDS_ADB_FLAG)
	&dev_attr_shmds_count,
	&dev_attr_shmds_ths_value,
	&dev_attr_shmds_duration_value,
	&dev_attr_shmds_buf_size_temp,
	&dev_attr_shmds_md_thresh,
	&dev_attr_shmds_mg_to_LSB,
	&dev_attr_shmds_md_thresh_value,
#endif /* SHMDS_DETECT, SHMDS_ADB_FLAG*/
/* shmds add 2-1 11-1 <- */
/* shmds add 12-2 -> */
	&dev_attr_shmds_fwdl_comp,
/* shmds add 12-2 <- */
	NULL
};
static struct device_attribute *inv_mpu3050_attributes[] = {
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_enable,
	&dev_attr_raw_accl,
	&dev_attr_accl_matrix,
	&dev_attr_accl_fs,
	NULL
};

static struct device_attribute *inv_compass_attributes[] = {
	&dev_attr_compass_enable,
	&dev_attr_compass_scale,
	&dev_attr_compass_matrix,
/* shmds add 2-1 -> */
	&dev_attr_raw_compass,
	&dev_attr_compass_self_test,
/* shmds add 2-1 <- */
	NULL
};

static int inv_setup_compass(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[4];

	result = inv_i2c_read(st, REG_YGOFFS_TC, 1, data);
	if (result)
		return result;
	data[0] &= ~BIT_I2C_MST_VDDIO;
	if (st->plat_data.level_shifter)
		data[0] |= BIT_I2C_MST_VDDIO;
	/*set up VDDIO register */
	result = inv_i2c_single_write(st, REG_YGOFFS_TC, data[0]);
	if (result)
		return result;
	/* set to bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, BIT_BYPASS_EN);
	if (result)
		return result;
	/*read secondary i2c ID register */
	result = inv_secondary_read(REG_AKM_ID, 1, data);
	if (result)
		return result;
	if (data[0] != DATA_AKM_ID)
		return -ENXIO;
	/*set AKM to Fuse ROM access mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_FR);
	if (result)
		return result;
	result = inv_secondary_read(REG_AKM_SENSITIVITY,
			THREE_AXIS, st->chip_info.compass_sens);
	if (result)
		return result;
	/*revert to power down mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_DN);
	if (result)
		return result;
	pr_err("senx=%d, seny=%d,senz=%d\n",
		st->chip_info.compass_sens[0],
		st->chip_info.compass_sens[1],
		st->chip_info.compass_sens[2]);
	/*restore to non-bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, 0);
	if (result)
		return result;

	/*setup master mode and master clock and ES bit*/
	result = inv_i2c_single_write(st, REG_I2C_MST_CTRL, BIT_WAIT_FOR_ES);
	if (result)
		return result;
	/* slave 0 is used to read data from compass */
	/*read mode */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_ADDR, BIT_I2C_READ|
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM status register address is 2 */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_REG, REG_AKM_STATUS);
	if (result)
		return result;
	/* slave 0 is enabled at the beginning, read 8 bytes from here */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_CTRL, BIT_SLV_EN |
					NUM_BYTES_COMPASS_SLAVE);
	if (result)
		return result;
	/*slave 1 is used for AKM mode change only*/
	result = inv_i2c_single_write(st, REG_I2C_SLV1_ADDR,
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM mode register address is 0x0A */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_REG, REG_AKM_MODE);
	if (result)
		return result;
	/* slave 1 is enabled, byte length is 1 */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_CTRL, BIT_SLV_EN | 1);
	if (result)
		return result;
	/* output data for slave 1 is fixed, single measure mode*/
	st->compass_scale = 1;
	data[0] = 1;
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8975_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8975_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8975_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8975_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8975_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8975_ST_Z_LW;
	} else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8972_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8972_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8972_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8972_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8972_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8972_ST_Z_LW;
	} else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8963_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8963_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8963_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8963_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8963_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8963_ST_Z_LW;
		data[0] |= (st->compass_scale << AKM8963_SCALE_SHIFT);
	}
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, data[0]);
	if (result)
		return result;
	/* slave 0 and 1 timer action is enabled every sample*/
	result = inv_i2c_single_write(st, REG_I2C_MST_DELAY_CTRL,
				BIT_SLV0_DLY_EN | BIT_SLV1_DLY_EN);
	return result;
}
static int inv_check_chip_type(struct inv_gyro_state_s *st,
		const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result;
	if (!strcmp(id->name, "itg3500"))
		st->chip_type = INV_ITG3500;
	else if (!strcmp(id->name, "mpu3050"))
		st->chip_type = INV_MPU3050;
	else if (!strcmp(id->name, "mpu6050"))
		st->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu9150"))
		st->chip_type = INV_MPU9150;
	else
		return -EPERM;
	st->hw  = (struct inv_hw_s *)(hw_info  + st->chip_type);
	reg = &st->reg;
	st->mpu_slave = NULL;
	if (INV_MPU3050 == st->chip_type)
		inv_setup_reg_mpu3050(reg);
	else
		inv_setup_reg(reg);

	if (INV_MPU9150 == st->chip_type) {
		st->plat_data.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS;
		st->plat_data.sec_slave_id = COMPASS_ID_AK8975;
		st->has_compass = 1;
	}
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (ACCEL_ID_BMA250 == st->plat_data.sec_slave_id)
			inv_register_bma250_slave(st);
	}
	if (SECONDARY_SLAVE_TYPE_COMPASS == st->plat_data.sec_slave_type)
		st->has_compass = 1;
	else
		st->has_compass = 0;
	st->chip_config.gyro_enable = 1;
	result = inv_set_power_state(st, 1);
	if (result)
		return result;

	if (st->has_compass) {
		result = inv_setup_compass(st);
		if (result)
			return result;
	}
	return 0;
}
static int inv_create_input(struct inv_gyro_state_s *st,
		struct i2c_client *client){
	struct input_dev *idev;
	int result;
	idev = NULL;
	result = inv_setup_input(st, &idev, client, st->hw->name);
	if (result)
		return result;
	st->idev = idev;
	if (INV_ITG3500 == st->chip_type)
		return 0;

	result = inv_setup_input(st, &idev, client, "INV_DMP");
	if (result) {
		input_unregister_device(st->idev);
		return result;
	}
	st->idev_dmp = idev;
	if (!st->has_compass)
		return 0;
	if (st->has_compass) {
		result = inv_setup_input(st, &idev, client, "INV_COMPASS");
		if (result) {
			input_unregister_device(st->idev);
			input_unregister_device(st->idev_dmp);
			return result;
		}
		st->idev_compass = idev;
	}
	return 0;
}
/* shmds add 15-1 -> */
void SHMDS_Pedometer_ReStart(void)
{
}
EXPORT_SYMBOL(SHMDS_Pedometer_ReStart);

void SHMDS_Pedometer_Pause(void)
{
}
EXPORT_SYMBOL(SHMDS_Pedometer_Pause);
/* shmds add 15-1 <- */
/* shmds add 14-1 -> */
void SHMDS_SetFlipInformation(unsigned char position)
{
    (void)position;
}
EXPORT_SYMBOL(SHMDS_SetFlipInformation);
/* shmds add 14-1 <- */

/* shmds add 9-1 -> */
#ifdef SHMDS_CAF
int SHMDS_Sensor_Read(struct shmds_sensor_data *sensor_data)
{
	int result = 0;
	
	if( sensor_data == NULL)
	{
		return result;
	}
	
	/* gyro */
	sensor_data->gyro_data.enable = current_gyro_data.enable;

	if(sensor_data->gyro_data.enable != 0)
	{
		if (shmds_write_flag[0] != 0)
		{
			sensor_data->gyro_data.x = last_gyro_data.x;
			sensor_data->gyro_data.y = last_gyro_data.y;
			sensor_data->gyro_data.z = last_gyro_data.z;
		}
		else
		{
			sensor_data->gyro_data.x = current_gyro_data.x;
			sensor_data->gyro_data.y = current_gyro_data.y;
			sensor_data->gyro_data.z = current_gyro_data.z;
		}
	}

	/* accl */
	sensor_data->accl_data.enable = current_accl_data.enable;
	if(sensor_data->accl_data.enable != 0)
	{
		if (shmds_write_flag[1] != 0)
		{
			sensor_data->accl_data.x = last_accl_data.x;
			sensor_data->accl_data.y = last_accl_data.y;
			sensor_data->accl_data.z = last_accl_data.z;
		}
		else
		{
			sensor_data->accl_data.x = current_accl_data.x;
			sensor_data->accl_data.y = current_accl_data.y;
			sensor_data->accl_data.z = current_accl_data.z;
		}
	}
	
	result = 1;
	
	return result;
	
}
EXPORT_SYMBOL(SHMDS_Sensor_Read);
#endif /* SHMDS_CAF */
/* shmds add 9-1 <- */

/* shmds add 11-1 -> */
void shmds_start_timer( void )
{

	if (atomic_read( &shmds_timer_kind_flg ) == SHMDS_TIMER_ADD ){
#ifdef SHMDS_DETECT
		if ( (shmds_detect_switch == DETECT_ON)
			 && ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) ) {
			timer_acc.expires = jiffies + msecs_to_jiffies(5000);
		} else {
			timer_acc.expires = jiffies + msecs_to_jiffies(1000);
		}
#else /* SHMDS_DETECT */
		timer_acc.expires = jiffies + msecs_to_jiffies(1000);
#endif /* SHMDS_DETECT */
		add_timer(&timer_acc);

		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_MOD );

	} else {
#ifdef SHMDS_DETECT
		if ( (shmds_detect_switch == DETECT_ON)
			 && ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) ) {
			mod_timer(&timer_acc,jiffies + msecs_to_jiffies(5000));
		} else {
			mod_timer(&timer_acc,jiffies + msecs_to_jiffies(1000));
		}
#else /* SHMDS_DETECT */
		mod_timer(&timer_acc,jiffies + msecs_to_jiffies(1000));
#endif /* SHMDS_DETECT */

		atomic_set( &shmds_timer_kind_flg, SHMDS_TIMER_MOD );
	}
}
/* shmds add 11-1 <- */


int create_device_attributes(struct device *dev,
	struct device_attribute **attrs)
{
	int i;
	int err = 0;
	for (i = 0 ; NULL != attrs[i] ; ++i) {
		err = sysfs_create_file(&dev->kobj, &attrs[i]->attr);
		if (0 != err)
			break;
	}
	if (0 != err) {
		for (; i >= 0 ; --i)
			sysfs_remove_file(&dev->kobj, &attrs[i]->attr);
	}
	return err;
}

void remove_device_attributes(struct device *dev,
	struct device_attribute **attrs)
{
	int i;
	for (i = 0 ; NULL != attrs[i] ; ++i)
		device_remove_file(dev, attrs[i]);
}

static char const *const inv_class_name = "invensense";
static char const *const inv_device_name = "mpu";
static dev_t const inv_device_dev_t = MKDEV(MISC_MAJOR, MISC_DYNAMIC_MINOR);
static struct bin_attribute dmp_firmware = {
	.attr = {
		.name = "dmp_firmware",
/* shmds mod 13-1 -> */
/*		.mode = S_IRUGO | S_IWUSR	*/
		.mode = 0664
/* shmds mod 13-1 <- */
	},
	.size = 4096,
	.read = inv_dmp_firmware_read,
	.write = inv_dmp_firmware_write,
};

static int create_sysfs_interfaces(struct inv_gyro_state_s *st)
{
	int result;
	result = 0;
	st->inv_class = class_create(THIS_MODULE, inv_class_name);
	if (IS_ERR(st->inv_class)) {
		result = PTR_ERR(st->inv_class);
		goto exit_nullify_class;
	}
	st->inv_dev = device_create(st->inv_class, &st->i2c->dev,
			inv_device_dev_t, st, inv_device_name);
	if (IS_ERR(st->inv_dev)) {
		result = PTR_ERR(st->inv_dev);
		goto exit_destroy_class;
	}
	result = create_device_attributes(st->inv_dev, inv_attributes);
	if (result < 0)
		goto exit_destroy_device;
	if (INV_ITG3500 == st->chip_type)
		return 0;
	result = sysfs_create_bin_file(&st->inv_dev->kobj, &dmp_firmware);
	if (result < 0)
		goto exit_remove_device_attributes;

	if (INV_MPU3050 == st->chip_type) {
		result = create_device_attributes(st->inv_dev,
					inv_mpu3050_attributes);
		if (result)
			goto exit_remove_bin_file;
		return 0;
	}

	result = create_device_attributes(st->inv_dev, inv_mpu6050_attributes);
	if (result < 0)
		goto exit_remove_bin_file;
	if (!st->has_compass)
		return 0;

	result = create_device_attributes(st->inv_dev, inv_compass_attributes);
	if (result < 0)
		goto exit_remove_6050_attributes;

	return 0;
exit_remove_6050_attributes:
	remove_device_attributes(st->inv_dev, inv_mpu6050_attributes);
exit_remove_bin_file:
	sysfs_remove_bin_file(&st->inv_dev->kobj, &dmp_firmware);
exit_remove_device_attributes:
	remove_device_attributes(st->inv_dev, inv_attributes);
exit_destroy_device:
	device_destroy(st->inv_class, inv_device_dev_t);
exit_destroy_class:
	st->inv_dev = NULL;
	class_destroy(st->inv_class);
exit_nullify_class:
	st->inv_class = NULL;
	return result;
}

static void remove_sysfs_interfaces(struct inv_gyro_state_s *st)
{
	remove_device_attributes(st->inv_dev, inv_attributes);
	if (INV_ITG3500 != st->chip_type)
		sysfs_remove_bin_file(&st->inv_dev->kobj, &dmp_firmware);
	if ((INV_ITG3500 != st->chip_type) && (INV_MPU3050 != st->chip_type))
		remove_device_attributes(st->inv_dev, inv_mpu6050_attributes);
	if (INV_MPU3050 == st->chip_type)
		remove_device_attributes(st->inv_dev, inv_mpu3050_attributes);
	if (st->has_compass)
		remove_device_attributes(st->inv_dev, inv_compass_attributes);
	device_destroy(st->inv_class, inv_device_dev_t);
	class_destroy(st->inv_class);
	st->inv_dev = NULL;
	st->inv_class = NULL;
}

/* shmds add 1-6 -> */
#ifdef CONFIG_OF
static const struct of_device_id inv_i2c_table[] = {
	{ .compatible = "InvenSense,MPU9150",},
	{}
};
#else
#define inv_i2c_table NULL;
#endif /* CONFIG_OF */
/* shmds add 1-6 <- */

static int inv_mod_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct inv_gyro_state_s *st;
	int result;

/* shmds add 5-1 -> */
	int ii =0;
#if defined (CONFIG_SHMDS_POLARITY_1)
	signed char orient[9] = { -1,  0,  0, 
							  0,  1,  0, 
							  0,  0,  -1 };
							  
    signed char second_orient[9] = { 0,  -1,  0,
									 1,  0,  0,
									 0,  0,  1 };
#else
	signed char orient[9] =	{ 1,  0,  0,
							  0,  1,  0,
							  0,  0,  1 };
							  
	signed char second_orient[9] = { 0,  1,  0,
									 1,  0,  0,
									 0,  0,  -1 };
#endif /* defined (CONFIG_SHMDS_POLARITY_1) */
/* shmds add 5-1 <- */
/* shmds add 17-2 -> */
	shmds_wake_lock_init();
/* shmds add 17-2 <- */
/* shmds add 1-4 -> */
	gpio_request(GPIO_GYRO_INT,"MPUIRQ");	
/* shmds add 1-4 <- */

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}
	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		result = -ENOMEM;
		goto out_no_free;
	}
	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, st);
	st->i2c = client;
	st->sl_handle = client->adapter;
	st->i2c_addr = client->addr;
/* shmds mod 1-6 -> */
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		int i;
		u32  int_config_data;
		u32  level_shifter_data;
		u32  sec_slave_type_data;
		u32  sec_slave_id_data;
		u32  secondary_i2c_addr_data;
		u32  key_data[16];
		u32  orientation_data[9];
		u32  secondary_orientation_data[9];

		result = of_property_read_u32( client->dev.of_node, "int_config", &int_config_data);
		if ( result == 0 ) {
			st->plat_data.int_config = (__u8)int_config_data;
			//dev_err( &client->dev, "st->plat_data.int_config[%d]\n", st->plat_data.int_config );
		}
		else {
			dev_err( &client->dev, "of_property_read_u32 error! named [int_config] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32( client->dev.of_node, "level_shifter", &level_shifter_data);
		if ( result == 0 ) {
			st->plat_data.level_shifter = (__u8)level_shifter_data;
			//dev_err( &client->dev, "st->plat_data.level_shifter[%d]\n", st->plat_data.level_shifter );
		}
		else {
			dev_err( &client->dev, "of_property_read_u32 error! named [level_shifter] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32_array( client->dev.of_node, "orientation", orientation_data, 9);	
		if ( result == 0 ) {
			for( i = 0; i < 9; i ++ ){
				st->plat_data.orientation[i] = (__s8)orientation_data[i];
				//dev_err( &client->dev, "st->plat_data.orientation[%d]=%d\n",i, st->plat_data.orientation[i]);
			}
		}
		else {
			dev_err( &client->dev, "of_property_read_u32_array error! named [orientation] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32( client->dev.of_node, "sec_slave_type", &sec_slave_type_data);
		if ( result == 0 ) {
			st->plat_data.sec_slave_type = (enum secondary_slave_type)sec_slave_type_data;
			//dev_err( &client->dev, "st->plat_data.sec_slave_type[%d]\n", st->plat_data.sec_slave_type );
		}
		else {
			dev_err( &client->dev, "of_property_read_u32 error! named [sec_slave_type] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32( client->dev.of_node, "sec_slave_id", &sec_slave_id_data);
		if ( result == 0 ) {
			st->plat_data.sec_slave_id = (enum ext_slave_id)sec_slave_id_data;
			//dev_err( &client->dev, "st->plat_data.sec_slave_id[%d]\n", st->plat_data.sec_slave_id );
		}
		else {
			dev_err( &client->dev, "of_property_read_u32 error! named [sec_slave_id] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32( client->dev.of_node, "secondary_i2c_addr", &secondary_i2c_addr_data);
		if ( result == 0 ) {
			st->plat_data.secondary_i2c_addr = (__u16)secondary_i2c_addr_data;
			//dev_err( &client->dev, "st->plat_data.secondary_i2c_addr[%d]\n", st->plat_data.secondary_i2c_addr );
		}
		else {
			dev_err( &client->dev, "of_property_read_u32 error! named [secondary_i2c_addr] =%d \n", result );
			goto out_free;
		}

		result = of_property_read_u32_array( client->dev.of_node, "secondary_orientation", secondary_orientation_data, 9);
		if ( result == 0 ) {
			for( i = 0; i < 9; i ++ ){
				st->plat_data.secondary_orientation[i] = (__s8)secondary_orientation_data[i];
				//dev_err( &client->dev, "st->plat_data.secondary_orientation[%d]=%d\n",i, st->plat_data.secondary_orientation[i]);
			}
		}
		else {
			dev_err( &client->dev, "of_property_read_u32_array error! named [secondary_orientation] =%d \n", result );	
			goto out_free;
		}

		result = of_property_read_u32_array( client->dev.of_node, "key", key_data, 16);
		if ( result == 0 ) {
			for( i = 0; i < 16; i ++ ){
				st->plat_data.key[i] = (__u8)key_data[i];
				//dev_err( &client->dev, "st->plat_data.key[%d]=%d\n",i, st->plat_data.key[i]);
			}
		}
		else {
			dev_err( &client->dev, "of_property_read_u32_array error! named [key] =%d \n", result );
			goto out_free;
		}
	}
#else
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
#endif /* CONFIG_OF */
/* shmds mod 1-6 <- */
		
/* shmds add 5-1 -> */
	for (ii = 0; ii < ARRAY_SIZE(st->plat_data.orientation); ii++){
		st->plat_data.orientation[ii] = orient[ii];
	}
	for (ii = 0; ii < ARRAY_SIZE(st->plat_data.secondary_orientation); ii++){
		st->plat_data.secondary_orientation[ii] = second_orient[ii];
	}
/* shmds add 5-1 <- */

	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(st, id);
	if (result)
		goto out_free;
	if (INV_MPU3050 == st->chip_type)
		result = inv_init_config_mpu3050(st);
	else
		result = inv_init_config(st);
	if (result) {
		dev_err(&client->adapter->dev,
			"Could not initialize device.\n");
		goto out_free;
	}
	if (INV_ITG3500 != st->chip_type && INV_MPU3050 != st->chip_type) {
		result = inv_get_silicon_rev_mpu6050(st);
		if (result) {
			dev_err(&client->adapter->dev,
				"%s get silicon error.\n", st->hw->name);
			goto out_free;
		}
	}
	result = inv_set_power_state(st, 0);
	if (result) {
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw->name);
		goto out_free;
	}

	INIT_KFIFO(st->trigger.timestamps);
	result = create_sysfs_interfaces(st);
	if (result)
		goto out_free_kfifo;
	if (!client->irq) {
		dev_err(&client->adapter->dev, "IRQ not assigned.\n");
		result = -EPERM;
		goto out_close_sysfs;
	}
	st->trigger.irq = client->irq;
	if (INV_MPU3050 == st->chip_type)
		result = request_threaded_irq(client->irq, inv_irq_handler,
			inv_read_fifo_mpu3050,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "inv_irq", st);
	else
		result = request_threaded_irq(client->irq, inv_irq_handler,
			inv_read_fifo,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "inv_irq", st);
	if (result)
		goto out_close_sysfs;
	spin_lock_init(&st->time_stamp_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
	atomic_set(&st->early_suspend_enable, 1);
	st->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->early_suspend.suspend = inv_early_suspend;
	st->early_suspend.resume = inv_late_resume;
	register_early_suspend(&st->early_suspend);
#endif
	result = inv_create_input(st, client);
	if (result) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->early_suspend);
#endif
		free_irq(client->irq, st);
		goto out_close_sysfs;
	}
	
/* shmds add 11-1 -> */
/* shmds mod 1-6 -> */
	if (client->irq == gpio_to_irq(GPIO_GYRO_INT)) {
/* shmds mod 1-6 <- */
		timer_acc.function = accl_timerhandler;
		timer_acc.data = (u_long)st;
		init_timer(&timer_acc);
		INIT_WORK(&(st->work), slaveirq_work_acc);
		INIT_WORK(&(st->work_timerhandler), timerhandler_work);
	}
/* shmds add 11-1 <- */

/* shmds add 12-4 -> */
    pm_qos_add_request(&shmds_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
/* shmds add 12-4 <- */

	pr_info("%s: Probe name %s\n", __func__, id->name);
	dev_info(&client->adapter->dev, "%s is ready to go!\n", st->hw->name);
	return 0;

out_close_sysfs:
	remove_sysfs_interfaces(st);
out_free_kfifo:
	kfifo_free(&st->trigger.timestamps);
out_free:
	kfree(st);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

static int inv_mod_remove(struct i2c_client *client)
{
	int result;
	struct inv_gyro_state_s *st = i2c_get_clientdata(client);

/* shmds add 11-1 -> */
#ifdef SHMDS_DETECT
	int i;
	
	for (i = 0; i < 3; i++) {
		kfree(shmds_detect_buf[i]);
		shmds_detect_buf[i] = NULL;
	}
#endif /* SHMDS_DETECT */
/* shmds add 11-1 <- */
	result = inv_set_power_state(st, 0);
	if (result)
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw->name);

	remove_sysfs_interfaces(st);
	kfifo_free(&st->trigger.timestamps);
	free_irq(client->irq, st);
	input_unregister_device(st->idev);
	if (INV_ITG3500 != st->chip_type)
		input_unregister_device(st->idev_dmp);
	if (st->has_compass)
		input_unregister_device(st->idev_compass);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->early_suspend);
#endif
/* shmds add 17-2 -> */
	shmds_wake_lock_destroy();
/* shmds add 17-2 <- */
	kfree(st);
	dev_info(&client->adapter->dev, "Gyro module removed.\n");
	return 0;
}
static unsigned short normal_i2c[] = { I2C_CLIENT_END };
/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_mod_id[] = {
	{"itg3500", 0},
	{"mpu3050", 0},
	{"mpu6050", 0},
	{"mpu9150", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mod_id);

static struct i2c_driver inv_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_mod_probe,
	.remove		=	inv_mod_remove,
	.id_table	=	inv_mod_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv_dev",
/* shmds add 1-6 -> */
#ifdef CONFIG_OF
		.of_match_table = inv_i2c_table,
#endif /* CONFIG_OF */
/* shmds add 1-6 <- */
	},
	.address_list = normal_i2c,
};

static int __init inv_mod_init(void)
{
	int result = i2c_add_driver(&inv_mod_driver);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	return 0;
}

static void __exit inv_mod_exit(void)
{
	i2c_del_driver(&inv_mod_driver);
}

module_init(inv_mod_init);
module_exit(inv_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv_dev");
/**
 *  @}
 */

