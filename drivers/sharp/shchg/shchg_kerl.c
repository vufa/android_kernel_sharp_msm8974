/*
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :													|*/
/*+-----------------------------------------------------------------------------+*/

/* MHL wake-lock control */
#define SHCHG_ENABLE_MHL_WAKE_LOCK

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :															|*/
/*+-----------------------------------------------------------------------------+*/

#include "linux/init.h"
#include "linux/module.h"
#include "linux/kernel.h"
#include "linux/err.h"
#include "linux/io.h"
#include "linux/cdev.h"
#include "linux/platform_device.h"
#include "linux/interrupt.h"
#include "linux/android_alarm.h"
#include "linux/slab.h"
#include "linux/power_supply.h"
#include "linux/wakelock.h"
#include "linux/mfd/pm8xxx/pm8921-bms.h"
#include "linux/mfd/pm8xxx/pm8921-charger.h"
#include "linux/delay.h"
#include "linux/gpio.h"
#include "linux/poll.h"
#include "asm/uaccess.h"
#include "sharp/shbatt_kerl.h"
#include "sharp/shchg_kerl.h"
#include "shchg_type.h"
#include <linux/sched.h>
#include <linux/qpnp/qpnp-api.h>

#ifdef CONFIG_CRADLE_SWIC
/*| ######## ZANTEI ######## */
//#define __SUPPORT_CRADLE__
#endif /* CONFIG_CRADLE_SWIC */

#ifdef __SUPPORT_CRADLE__
#include "sharp/shswic_kerl.h"
#endif /* __SUPPORT_CRADLE__ */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE : 													|*/
/*+-----------------------------------------------------------------------------+*/
//#define SHCHG_ENABLE_DEBUG

#ifdef SHCHG_ENABLE_DEBUG

#ifdef SHCHG_ENABLE_LOGGING

#define SHCHG_INFO(x...)   shlogger_api_post_log(x)
#define SHCHG_TRACE(x...)  shlogger_api_post_log(x)
#define SHCHG_ERROR(x...)  shlogger_api_post_log(x)

#else

#define SHCHG_INFO(x...)   printk(KERN_INFO "[SHCHG] " x)
#define SHCHG_TRACE(x...)  printk(KERN_DEBUG "[SHCHG] " x)
#define SHCHG_ERROR(x...)  printk(KERN_ERR "[SHCHG] " x)

#endif /* SHCHG_ENABLE_LOGGING */

#else

#define SHCHG_INFO(x...)   do {} while(0)
#define SHCHG_TRACE(x...)  do {} while(0)
#define SHCHG_ERROR(x...)  do {} while(0)

#endif /* SHCHG_ENABLE_DEBUG */

#define SHCHG_DEV_NAME "shchg"

#define SHCHG_WAKE_CTL(x)										\
{																\
	do															\
	{															\
		if(x == 0)												\
		{														\
			SHCHG_TRACE("[P] %s SHCHG_WAKE_CTL(0) call shchg_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shchg_wake_lock_num));	\
			if(atomic_dec_return(&shchg_wake_lock_num) == 0)	\
			{													\
				SHCHG_TRACE("[P] %s SHCHG_WAKE_CTL(0) done shchg_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shchg_wake_lock_num));	\
				wake_unlock(&shchg_wake_lock);					\
			}													\
		}														\
		else													\
		{														\
			SHCHG_TRACE("[P] %s SHCHG_WAKE_CTL(1) call shchg_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shchg_wake_lock_num));	\
			if(atomic_inc_return(&shchg_wake_lock_num) == 1)	\
			{													\
				SHCHG_TRACE("[P] %s SHCHG_WAKE_CTL(1) done shchg_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shchg_wake_lock_num));	\
				wake_lock(&shchg_wake_lock);					\
			}													\
		}														\
	} while(0); 												\
}

#define SHCHG_IOC_MAGIC 's'
#define SHCHG_DRV_IOCTL_CMD_PULL_USSE_PACKET							_IOR( SHCHG_IOC_MAGIC, 1, shchg_usse_packet_t)
#define SHCHG_DRV_IOCTL_CMD_DONE_USSE_PACKET							_IOW( SHCHG_IOC_MAGIC, 2, shchg_usse_packet_t)
#define SHCHG_DRV_IOCTL_CMD_INVALID										_IO(  SHCHG_IOC_MAGIC, 3)
#define SHCHG_DRV_IOCTL_CMD_GET_PARAMETER_LEVEL							_IOWR(SHCHG_IOC_MAGIC, 4, shchg_prm_level_info_t)
#define SHCHG_DRV_IOCTL_CMD_SET_TIMER									_IOW( SHCHG_IOC_MAGIC, 5, shchg_poll_timer_info_t)
#define SHCHG_DRV_IOCTL_CMD_CLR_TIMER									_IOW( SHCHG_IOC_MAGIC, 6, shchg_poll_timer_info_t)
#define SHCHG_DRV_IOCTL_CMD_SET_CHARGER_VMAXSEL							_IOW( SHCHG_IOC_MAGIC, 7, int)
#define SHCHG_DRV_IOCTL_CMD_GET_CHARGER_VMAXSEL							_IOR( SHCHG_IOC_MAGIC, 8, int)
#define SHCHG_DRV_IOCTL_CMD_SET_CHARGER_IMAXSEL							_IOW( SHCHG_IOC_MAGIC, 9, int)
#define SHCHG_DRV_IOCTL_CMD_GET_CHARGER_IMAXSEL							_IOR( SHCHG_IOC_MAGIC, 10, int)

#define SHCHG_DRV_IOCTL_CMD_SET_CHARGER_CIN_LIMIT						_IOW( SHCHG_IOC_MAGIC, 11, int)
#define SHCHG_DRV_IOCTL_CMD_GET_CHARGER_CIN_LIMIT						_IOR( SHCHG_IOC_MAGIC, 12, int)
#define SHCHG_DRV_IOCTL_CMD_SET_CHARGER_TRANSISTOR_SWITCH				_IOW( SHCHG_IOC_MAGIC, 13, shchg_pm_sw_t)
#define SHCHG_DRV_IOCTL_CMD_SET_CRADLE_CB_REGIST						_IO(  SHCHG_IOC_MAGIC, 14)
#define SHCHG_DRV_IOCTL_CMD_GET_CRADLE_STATUS							_IOR( SHCHG_IOC_MAGIC, 15, shchg_cradle_status_t)
#define SHCHG_DRV_IOCTL_CMD_SET_VBSW_REG								_IOW( SHCHG_IOC_MAGIC, 16, int)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_CONNECTED				_IOW( SHCHG_IOC_MAGIC, 17, shchg_device_t)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_DISCONNECTED				_IO(  SHCHG_IOC_MAGIC, 18)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_CRADLE_CONNECTED						_IO(  SHCHG_IOC_MAGIC, 19)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_CRADLE_DISCONNECTED					_IO(  SHCHG_IOC_MAGIC, 20)

#define SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_I_IS_AVAILABLE			_IOW( SHCHG_IOC_MAGIC, 21, int)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE		_IO(  SHCHG_IOC_MAGIC, 22)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_BATTERY_DISCONNECTED					_IO(  SHCHG_IOC_MAGIC, 23)
#define SHCHG_DRV_IOCTL_CMD_NOTIFY_CHARGING_STATE_MACHINE_EXPIRE		_IO(  SHCHG_IOC_MAGIC, 24)
#define SHCHG_DRV_IOCTL_CMD_CHECK_CHARGER_DISCONNECTED					_IO(  SHCHG_IOC_MAGIC, 25)
#define SHCHG_DRV_IOCTL_CMD_INITIALIZE									_IO(  SHCHG_IOC_MAGIC, 26)
#define SHCHG_DRV_IOCTL_CMD_GET_KERNEL_TIME								_IOR( SHCHG_IOC_MAGIC, 27, struct timeval)
#define SHCHG_DRV_IOCTL_CMD_SET_CHARGER_VINMIN							_IOW( SHCHG_IOC_MAGIC, 28, int)
#define SHCHG_DRV_IOCTL_CMD_GET_CHARGER_VINMIN							_IOR( SHCHG_IOC_MAGIC, 29, int)
#define SHCHG_DRV_IOCTL_CMD_GET_MHL_CABLE_STATUS						_IOR( SHCHG_IOC_MAGIC, 30, int)

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN VARIABLE : 														|*/
/*+-----------------------------------------------------------------------------+*/

	/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE : 														|*/
/*+-----------------------------------------------------------------------------+*/

	/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE : 														|*/
/*+-----------------------------------------------------------------------------+*/

static bool shchg_task_is_initialized = false;
static struct workqueue_struct* shchg_task_workqueue_p;
static shchg_timer_t shchg_tim_state_machine;
static shchg_timer_t shchg_tim_battery_deterioration_diagnosis;
static shchg_timer_t shchg_tim_charger_ovp_detect;
static struct mutex shchg_api_lock;
static struct mutex shchg_task_lock;
static spinlock_t shchg_pkt_lock;
static struct completion shchg_api_cmp;
static struct wake_lock shchg_wake_lock;
static atomic_t shchg_wake_lock_num;
static dev_t shchg_dev;
static int shchg_major;
static int shchg_minor;
static struct cdev shchg_cdev;
static struct class* shchg_dev_class;
static atomic_t shchg_usse_op_cnt;
static wait_queue_head_t shchg_usse_wait;
static shchg_usse_packet_t shchg_usse_pkt;
static struct completion shchg_usse_cmp;
static unsigned int shchg_cur_vmaxsel = 0;
static unsigned int shchg_cur_imaxsel = 0;
static unsigned int shchg_cur_cin_limit = 0;
static unsigned int shchg_cur_vinmin = 0;
static shchg_device_t shchg_cur_boot_time_cable_status = SHCHG_DEVICE_NONE;
static int shchg_is_cradle_present_in_boot_time = 0;
static int shchg_cur_boot_time_ima = 0;
static shchg_packet_t shchg_pkt[16];
static shchg_pm_sw_t shchg_cur_switch_state = SHCHG_PM_SW_OFF;
static unsigned int shchg_charger_connect = 0x00;
#ifdef SHCHG_ENABLE_MHL_WAKE_LOCK
static struct wake_lock shchg_mhl_wake_lock;
#endif /* SHCHG_ENABLE_MHL_WAKE_LOCK */
static int shchg_charge_ignore_flg = 0;

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :										|*/
/*+-----------------------------------------------------------------------------+*/

	/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INTERNAL API FUNCTION PROTO TYPE DECLARE :								|*/
/*+-----------------------------------------------------------------------------+*/

static shchg_result_t shchg_api_initialize( void );
static shchg_result_t shchg_api_exec_state_machine_sequence( void );
static shchg_result_t shchg_api_exec_battery_deterioration_diagnosis_sequence( void );
static shchg_result_t shchg_api_exec_charger_ovp_detect_sequence( void );
static shchg_result_t shchg_api_get_mhl_cable_status( int* mhl_cable_status );

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :										|*/
/*+-----------------------------------------------------------------------------+*/

static void shchg_task( struct work_struct* work_p );
static shchg_packet_t* shchg_task_get_packet( void );
static void shchg_task_free_packet( shchg_packet_t* pkt );
static void shchg_task_cmd_invalid( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_usb_charger_connected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_usb_charger_disconnected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_cradle_connected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_cradle_disconnected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_usb_charger_i_is_available( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_usb_charger_i_is_not_available( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_battery_disconnected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_notify_charging_state_machine_expire( shchg_packet_t* pkt_p );
static void shchg_task_cmd_check_charger_disconnected( shchg_packet_t* pkt_p );
static void shchg_task_cmd_initialize( shchg_packet_t* pkt_p );
static void shchg_task_cmd_exec_state_machine_sequence( shchg_packet_t* pkt_p );
static void shchg_task_cmd_exec_battery_deterioration_diagnosis_sequence( shchg_packet_t* pkt_p );
static void shchg_task_cmd_exec_charger_ovp_detect_sequence( shchg_packet_t* pkt_p );
static void shchg_task_cmd_set_log_enable( shchg_packet_t* pkt_p );
static void shchg_task_cmd_get_mhl_cable_status( shchg_packet_t* pkt_p );

static shchg_result_t shchg_seq_call_user_space_sequence_executor( void );
static shchg_result_t shchg_seq_notify_usb_charger_connected( shchg_device_t dev );
static shchg_result_t shchg_seq_notify_usb_charger_disconnected( void );
static shchg_result_t shchg_seq_notify_cradle_connected( void );
static shchg_result_t shchg_seq_notify_cradle_disconnected( void );
static shchg_result_t shchg_seq_notify_usb_charger_i_is_available( int ima );
static shchg_result_t shchg_seq_notify_usb_charger_i_is_not_available( void );
static shchg_result_t shchg_seq_notify_battery_disconnected( void );
static shchg_result_t shchg_seq_notify_charging_state_machine_expire( void );
static shchg_result_t shchg_seq_check_charger_disconnected( void );
static shchg_result_t shchg_seq_initialize( void );
static shchg_result_t shchg_seq_exec_state_machine_sequence( void );
static shchg_result_t shchg_seq_exec_battery_deterioration_diagnosis_sequence( void );
static shchg_result_t shchg_seq_exec_charger_ovp_detect_sequence( void );
static shchg_result_t shchg_seq_set_log_enable( int val );
static shchg_result_t shchg_seq_get_mhl_cable_status( int* mhl_cable_status );
static void shchg_seq_state_machine_timer_expire_cb( struct alarm* alm_p );
static void shchg_seq_battery_deterioration_diagnosis_timer_expire_cb( struct alarm* alm_p );
static void shchg_seq_charger_ovp_detect_timer_expire_cb( struct alarm* alm_p );

#ifdef __SUPPORT_CRADLE__
static void shchg_seq_cradle_detect_cb( unsigned char dev, void* usr_p );
#endif /* __SUPPORT_CRADLE__ */

static int shchg_seq_get_parameter_level( shchg_get_prm_t prm );
static shchg_result_t shchg_pm_set_charger_vmaxsel( int mv );
static shchg_result_t shchg_pm_get_charger_vmaxsel( int* mv_p );
static shchg_result_t shchg_pm_set_charger_imaxsel( int ma );
static shchg_result_t shchg_pm_get_charger_imaxsel( int* ma_p );
static shchg_result_t shchg_pm_set_charger_cin_limit( int cl );
static shchg_result_t shchg_pm_get_charger_cin_limit( int* cl_p );
static shchg_result_t shchg_pm_set_charger_vinmin( int vol );
static shchg_result_t shchg_pm_get_charger_vinmin( int* vol );
static shchg_result_t shchg_pm_set_charger_transistor_switch( shchg_pm_sw_t sw );

static int shchg_drv_ioctl_cmd_invalid( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_pull_usse_packet( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_done_usse_packet( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_parameter_level( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_timer( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_clr_timer( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_charger_vmaxsel( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_charger_vmaxsel( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_charger_imaxsel( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_charger_imaxsel( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_charger_cin_limit( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_charger_cin_limit( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_charger_vinmin( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_charger_vinmin( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_charger_transistor_switch( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_cradle_cb_resist( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_cradle_status( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_set_vbsw_reg( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_usb_charger_connected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_usb_charger_disconnected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_cradle_connected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_cradle_disconnected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_usb_charger_i_is_available( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_usb_charger_i_is_not_available( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_battery_disconnected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_notify_charging_state_machine_expire( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_check_charger_disconnected( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_initialize( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_kernel_time( struct file* fi_p, unsigned long arg );
static int shchg_drv_ioctl_cmd_get_mhl_cable_status( struct file* fi_p, unsigned long arg );

static int shchg_drv_open( struct inode* in_p, struct file* fi_p );
static int shchg_drv_release( struct inode* in_p, struct file* fi_p );
static unsigned int shchg_drv_poll( struct file* fi_p, poll_table* wait_p );
static long shchg_drv_ioctl( struct file* fi_p, unsigned int cmd, unsigned long arg );
static int shchg_drv_create_device( void );
static int shchg_drv_register_irq( struct platform_device* dev_p );
static int shchg_drv_probe( struct platform_device* dev_p );
static int __devexit shchg_drv_remove( struct platform_device* dev_p );
static void shchg_drv_shutdown( struct platform_device* dev_p );
static int __init shchg_drv_module_init( void );
static void __exit shchg_drv_module_exit( void );

/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :										|*/
/*+-----------------------------------------------------------------------------+*/
#define SHCHG_OF_DEV_NAME			"sharp,shchg"

#ifdef CONFIG_OF
static struct of_device_id shchg_match_table[] = {
	{ .compatible = SHCHG_OF_DEV_NAME },
	{}
};
#else  /* CONFIG_OF */
#define shchg_match_table NULL
#endif /* CONFIG_OF */

static struct platform_driver shchg_platform_driver = 
{
	.probe		= shchg_drv_probe,
	.remove		= __devexit_p(shchg_drv_remove),
	.shutdown	= shchg_drv_shutdown,
	.driver		= {
		.name	= "shchg",
		.owner	= THIS_MODULE,
		.of_match_table = shchg_match_table,
	},
};

static struct file_operations shchg_fops = 
{
	.owner	 = THIS_MODULE,
	.open	 = shchg_drv_open,
	.release = shchg_drv_release,
	.poll	 = shchg_drv_poll,
	.unlocked_ioctl   = shchg_drv_ioctl,
};

static void (*const shchg_task_cmd_func[])( shchg_packet_t* pkt_p ) = 
{
	shchg_task_cmd_invalid,
	shchg_task_cmd_notify_usb_charger_connected,
	shchg_task_cmd_notify_usb_charger_disconnected,
	shchg_task_cmd_notify_cradle_connected,
	shchg_task_cmd_notify_cradle_disconnected,
	shchg_task_cmd_notify_usb_charger_i_is_available,
	shchg_task_cmd_notify_usb_charger_i_is_not_available,
	shchg_task_cmd_notify_battery_disconnected,
	shchg_task_cmd_notify_charging_state_machine_expire,
	shchg_task_cmd_check_charger_disconnected,
	shchg_task_cmd_initialize,
	shchg_task_cmd_exec_state_machine_sequence,
	shchg_task_cmd_exec_battery_deterioration_diagnosis_sequence,
	shchg_task_cmd_exec_charger_ovp_detect_sequence,
	shchg_task_cmd_set_log_enable,
	shchg_task_cmd_get_mhl_cable_status
};

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

shchg_result_t shchg_api_notify_usb_charger_connected( shchg_device_t dev )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		shchg_cur_boot_time_cable_status = dev;

		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}
	
	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_CONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.dev = dev;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_usb_charger_disconnected( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		shchg_cur_boot_time_cable_status = SHCHG_DEVICE_NONE;

		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}
	
	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_DISCONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_cradle_connected( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		shchg_is_cradle_present_in_boot_time = 1;
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_CRADLE_CONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_cradle_disconnected( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		shchg_is_cradle_present_in_boot_time = 0;
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_CRADLE_DISCONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_usb_charger_i_is_available( int ima )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	
	if(shchg_charge_ignore_flg == 1)
	{
		return SHCHG_RESULT_SUCCESS;
	}
	
	if(shchg_task_is_initialized == false)
	{
		shchg_cur_boot_time_ima = ima;

		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_AVAILABLE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.ima = ima;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_usb_charger_i_is_not_available( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		shchg_cur_boot_time_ima = 0;

		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_battery_disconnected( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_BATTERY_DISCONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_notify_charging_state_machine_expire( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_NOTIFY_CHARGING_STATE_MACHINE_EXPIRE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

shchg_result_t shchg_api_check_charger_disconnected( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_CHECK_CHARGER_DISCONNECTED;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ INTERNAL API FUNCTION'S CODE AREA :                                       |*/
/*+-----------------------------------------------------------------------------+*/

static shchg_result_t shchg_api_initialize( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == true)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_INITIALIZE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

static shchg_result_t shchg_api_exec_state_machine_sequence( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_EXEC_STATE_MACHINE_SEQUENCE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

static shchg_result_t shchg_api_exec_battery_deterioration_diagnosis_sequence( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_EXEC_BATTERY_DETERIORATION_DIAGNOSIS_SEQUENCE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

static shchg_result_t shchg_api_exec_charger_ovp_detect_sequence( void )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_EXEC_CHARGER_OVP_DETECT_SEQUENCE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = 0;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

static void shchg_task( struct work_struct* work_p )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	mutex_lock(&shchg_task_lock);

	pkt_p = (shchg_packet_t*)work_p;

	if(pkt_p->hdr.cmd < NUM_SHCHG_CMD)
	{
		shchg_task_cmd_func[pkt_p->hdr.cmd](pkt_p);
	}
	else
	{
		shchg_task_cmd_invalid(pkt_p);
	}

	SHCHG_WAKE_CTL(0);

	shchg_task_free_packet(pkt_p);

	mutex_unlock(&shchg_task_lock);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static shchg_packet_t* shchg_task_get_packet( void )
{
	int idx;

	unsigned long flags;

	shchg_packet_t* ret = NULL;

	spin_lock_irqsave(&shchg_pkt_lock,flags);

	for(idx = 0; idx < 16; idx++)
	{
		if(shchg_pkt[idx].is_used == false)
		{
			shchg_pkt[idx].is_used = true;

			ret = &shchg_pkt[idx];

			break;
		}
	}

	spin_unlock_irqrestore(&shchg_pkt_lock,flags);

	return ret;
}

static void shchg_task_free_packet( shchg_packet_t* pkt )
{
	unsigned long flags;

	spin_lock_irqsave(&shchg_pkt_lock,flags);

	pkt->is_used = false;

	spin_unlock_irqrestore(&shchg_pkt_lock,flags);

	return;
}

static void shchg_task_cmd_invalid( shchg_packet_t* pkt_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_usb_charger_connected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_usb_charger_connected(pkt_p->prm.dev);

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_usb_charger_disconnected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_usb_charger_disconnected();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_cradle_connected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_cradle_connected();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_cradle_disconnected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_cradle_disconnected();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_usb_charger_i_is_available( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_usb_charger_i_is_available(pkt_p->prm.ima);

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_usb_charger_i_is_not_available( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_usb_charger_i_is_not_available();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_battery_disconnected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_battery_disconnected();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_notify_charging_state_machine_expire( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_notify_charging_state_machine_expire();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_check_charger_disconnected( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_check_charger_disconnected();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_initialize( shchg_packet_t* pkt_p )
{
	shchg_cb_func_t1 cb_func;

	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_initialize();

	if(pkt_p->hdr.cb_p != NULL)
	{
		cb_func = (shchg_cb_func_t1)pkt_p->hdr.cb_p;

		cb_func(result);
	}
	else
	{
		if(pkt_p->hdr.ret_p != NULL)
		{
			*(pkt_p->hdr.ret_p) = result;
		}

		if(pkt_p->hdr.cmp_p != NULL)
		{
			complete(pkt_p->hdr.cmp_p);
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_exec_state_machine_sequence( shchg_packet_t* pkt_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_seq_exec_state_machine_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_exec_battery_deterioration_diagnosis_sequence( shchg_packet_t* pkt_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_seq_exec_battery_deterioration_diagnosis_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_exec_charger_ovp_detect_sequence( shchg_packet_t* pkt_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_seq_exec_charger_ovp_detect_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_task_cmd_set_log_enable( shchg_packet_t* pkt_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_seq_set_log_enable(pkt_p->prm.val);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static shchg_result_t shchg_seq_call_user_space_sequence_executor( void )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	INIT_COMPLETION(shchg_usse_cmp);

	atomic_inc(&shchg_usse_op_cnt);

	wake_up_interruptible(&shchg_usse_wait);

	wait_for_completion_killable(&shchg_usse_cmp);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return shchg_usse_pkt.hdr.ret;
}

static shchg_result_t shchg_seq_notify_usb_charger_connected( shchg_device_t dev )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_charger_connect |= SHCHG_CHARGER_CONNECT_USB_AC;

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_CONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.dev = dev;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_usb_charger_disconnected( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	
	shchg_charger_connect &= ~SHCHG_CHARGER_CONNECT_USB_AC;

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_DISCONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_cradle_connected( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_charger_connect |= SHCHG_CHARGER_CONNECT_CRADLE;

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_CRADLE_CONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_cradle_disconnected( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_charger_connect &= ~SHCHG_CHARGER_CONNECT_CRADLE;

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_CRADLE_DISCONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_usb_charger_i_is_available( int ima )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_AVAILABLE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.ima = ima;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

#ifdef SHCHG_ENABLE_MHL_WAKE_LOCK
#define SHCHG_MHL_WAKELOCK_TIMEOUT	1*HZ
#endif /* SHCHG_ENABLE_MHL_WAKE_LOCK */

static shchg_result_t shchg_seq_notify_usb_charger_i_is_not_available( void )
{
	shchg_result_t result;
	int tpin_status = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

#ifdef SHCHG_ENABLE_MHL_WAKE_LOCK
	shbatt_api_get_tpin_status(&tpin_status);
	if(tpin_status == 0)
	{
		if (!wake_lock_active(&shchg_mhl_wake_lock))
		{
			wake_lock_timeout(&shchg_mhl_wake_lock, SHCHG_MHL_WAKELOCK_TIMEOUT);
		}
		else
		{
			wake_unlock(&shchg_mhl_wake_lock);
			wake_lock_timeout(&shchg_mhl_wake_lock, SHCHG_MHL_WAKELOCK_TIMEOUT);
		}
	}
#endif /* SHCHG_ENABLE_MHL_WAKE_LOCK */

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_battery_disconnected( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_BATTERY_DISCONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_notify_charging_state_machine_expire( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_NOTIFY_CHARGING_STATE_MACHINE_EXPIRE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_check_charger_disconnected( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_CHECK_CHARGER_DISCONNECTED;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_initialize( void )
{
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_task_is_initialized = true;

	if(shchg_is_cradle_present_in_boot_time)
	{
		shchg_seq_notify_cradle_connected();
	}
#ifdef __SUPPORT_CRADLE__
	else
	{
		uint8_t status = SHSWIC_ID_NONE;

		shswic_get_cradle_status(&status);
		if (status == SHSWIC_ID_CRADLE)
		{
			shchg_seq_notify_cradle_connected();
		}
	}
#endif //__SUPPORT_CRADLE__

	if(shchg_cur_boot_time_cable_status != SHCHG_DEVICE_NONE)
	{
		shchg_seq_notify_usb_charger_connected(shchg_cur_boot_time_cable_status);
	}

	if(shchg_cur_boot_time_ima != 0)
	{
		shchg_seq_notify_usb_charger_i_is_available(shchg_cur_boot_time_ima);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_exec_state_machine_sequence( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_EXEC_STATE_MACHINE_SEQUENCE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_exec_battery_deterioration_diagnosis_sequence( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_EXEC_BATTERY_DETERIORATION_DIAGNOSIS_SEQUENCE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_exec_charger_ovp_detect_sequence( void )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_EXEC_CHARGER_OVP_DETECT_SEQUENCE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_seq_set_log_enable( int val )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_SET_LOG_ENABLE;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = val;

	result = shchg_seq_call_user_space_sequence_executor();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static void shchg_seq_state_machine_timer_expire_cb( struct alarm* alm_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_api_exec_state_machine_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_seq_battery_deterioration_diagnosis_timer_expire_cb( struct alarm* alm_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_api_exec_battery_deterioration_diagnosis_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static void shchg_seq_charger_ovp_detect_timer_expire_cb( struct alarm* alm_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_api_exec_charger_ovp_detect_sequence();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

#ifdef __SUPPORT_CRADLE__
static void shchg_seq_cradle_detect_cb( unsigned char dev, void* usr_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(dev == SHSWIC_ID_CRADLE)
	{
		shchg_api_notify_cradle_connected();
	}
	else
	{
		shchg_api_notify_cradle_disconnected();
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}
#endif /* __SUPPORT_CRADLE__ */

static int shchg_seq_get_parameter_level( shchg_get_prm_t prm )
{
	shbatt_adc_t adc;
	int ibatt, level_val = 0;
	shbatt_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	switch(prm)
	{
		case SHCHG_GET_PRM_BATTERY_VOLT:
			adc.channel = SHBATT_ADC_CHANNEL_VBAT;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_WALL_CHARGER_VOLT:
			adc.channel = SHBATT_ADC_CHANNEL_DCIN;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_USB_CHARGER_VOLT:
			adc.channel = SHBATT_ADC_CHANNEL_USBIN;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATTERY_I:
			if(shbatt_api_get_battery_current(&ibatt) == SHBATT_RESULT_SUCCESS)
			{
				level_val = ibatt;
			}
			else
			{
				level_val = 0;
			}
			break;

		case SHCHG_GET_PRM_CHARGER_I:
			adc.channel = SHBATT_ADC_CHANNEL_ICHG;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATTERY_ID:
			adc.channel = SHBATT_ADC_CHANNEL_BAT_ID;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.adc_code;
			break;

		case SHCHG_GET_PRM_BATT_THERM_DEGC_NO_MOVING_AVERAGE:
			adc.channel = SHBATT_ADC_CHANNEL_BAT_THERM_NO_MOVING_AVERAGE;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATT_THERM_DEGC:
			adc.channel = SHBATT_ADC_CHANNEL_BAT_THERM;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_CHG_THERM_DEGC:
			adc.channel = SHBATT_ADC_CHANNEL_CHG_TEMP;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_CAMERA_THERM_DEGC:
			adc.channel = SHBATT_ADC_CHANNEL_CAM_THERM;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_PMIC_DIE_DEGC:
			adc.channel = SHBATT_ADC_CHANNEL_PMIC_TEMP;
			result = shbatt_api_read_adc_channel_no_conversion(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATTERY_VOLT_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_VBAT;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;
		
		case SHCHG_GET_PRM_WALL_CHARGER_VOLT_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_DCIN;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_USB_CHARGER_VOLT_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_USBIN;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATTERY_I_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_IBAT;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_CHARGER_I_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_ICHG;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATTERY_ID_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_BAT_ID;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_BATT_THERM_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_BAT_THERM;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_CHG_THERM_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_CHG_TEMP;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_CAMERA_THERM_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_CAM_THERM;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_PMIC_DIE_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_PMIC_TEMP;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_PA_THERM0_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_PA_THERM;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_LCD_THERM_DEGC_BUF:
			adc.channel = SHBATT_ADC_CHANNEL_LCD_THERM;
			result = shbatt_api_read_adc_channel_buffered(&adc);
			level_val = adc.physical;
			break;

		case SHCHG_GET_PRM_DUMMY_READ:
			/*don't need.*/
		default:
			break;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return level_val;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ PMIC DRIVER MODULE CODE AREA :											|*/
/*+-----------------------------------------------------------------------------+*/

static shchg_result_t shchg_pm_set_charger_vmaxsel( int mv )
{
	int val = mv;
	int rc;
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	SHCHG_TRACE("[vmax] %dmV \n", val);
	
	rc = qpnp_chg_charger_vddmax_set(val);
	if (rc) {
		result = SHBATT_RESULT_FAIL;
		SHCHG_ERROR("error qpnp_chg_charger_vddmax_set = %d, rc = %d\n", val, rc);
	}
	shchg_cur_vmaxsel = val;

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_pm_get_charger_vmaxsel( int* mv_p )
{
	shchg_result_t result = SHCHG_RESULT_SUCCESS;
	int rc;
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(mv_p != NULL)
	{
		rc = qpnp_chg_charger_vddmax_get(mv_p);
		if (rc)
		{
			result = SHBATT_RESULT_FAIL;
			SHCHG_ERROR("error qpnp_chg_charger_vddmax_get rc = %d\n", rc);
		}
	}
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
	return result;
}

static shchg_result_t shchg_pm_set_charger_imaxsel( int ma )
{
	int rc;
	int val = ma;
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	do
	{
		SHCHG_TRACE("[imax] %dmA \n", val);
		rc = qpnp_chg_charger_ibatmax_set(val);
		if (rc) {
			result = SHBATT_RESULT_FAIL;
			SHCHG_ERROR("error qpnp_chg_charger_ibatmax_set = %d, rc = %d\n", val, rc);
			break;
		}
		shchg_cur_imaxsel = val;
	}
	while(0);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_pm_get_charger_imaxsel( int* ma_p )
{
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(ma_p != NULL)
	{
		*ma_p = shchg_cur_imaxsel;
	}
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}
static shchg_result_t shchg_pm_set_charger_cin_limit( int cl )
{
	int rc;
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	do
	{
		SHCHG_TRACE("[cur] %dmA\n", cl);
		rc = qpnp_chg_charger_iusbmax_set( cl );
		if (rc)
		{
			result = SHBATT_RESULT_FAIL;
			SHCHG_ERROR("error qpnp_chg_charger_iusbmax_set = %d, rc = %d\n", cl, rc);
			break;
		}
		shchg_cur_cin_limit = cl;
	}
	while(0);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
	return result;
}


static shchg_result_t shchg_pm_get_charger_cin_limit( int* cl_p )
{
	shchg_result_t result = SHCHG_RESULT_FAIL;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(cl_p != NULL)
	{
		*cl_p = shchg_cur_cin_limit;
		result = SHCHG_RESULT_SUCCESS;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static shchg_result_t shchg_pm_set_charger_vinmin( int vol )
{
	int rc;
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s vol=%d\n",__FUNCTION__,vol);
	
	do
	{
		rc = qpnp_chg_charger_vinmin_set(vol);
		if(rc != 0)
		{
			result = SHBATT_RESULT_FAIL;
			SHCHG_ERROR("error qpnp_chg_charger_vinmin_set vol=%d, rc=%d\n", vol, rc);
			break;
		}
		shchg_cur_vinmin = vol;
	}
	while(0);
	
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
	
	return result;
}

static shchg_result_t shchg_pm_get_charger_vinmin( int* vol )
{
	int rc;
	shchg_result_t result = SHCHG_RESULT_FAIL;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	
	do
	{
		rc = qpnp_chg_charger_vinmin_get(vol);
		if(rc != 0)
		{
			result = SHBATT_RESULT_FAIL;
			SHCHG_ERROR("error shchg_pm_get_charger_vinmin vol=%d, rc=%d\n", *vol, rc);
			break;
		}
		shchg_cur_vinmin = *vol;
	}
	while(0);

	SHCHG_TRACE("[E] %s vol=%d \n",__FUNCTION__, *vol);

	return result;
}

static shchg_result_t shchg_pm_set_charger_transistor_switch( shchg_pm_sw_t sw )
{
	SHCHG_TRACE("[S] %s sw=%d shchg_cur_switch_state=%d \n",__FUNCTION__, sw, shchg_cur_switch_state);
	do
	{
		switch(sw)
		{
			case SHCHG_PM_SW_OFF:
			//	shbatt_api_set_pmic_battery_transistor_switch(1);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_ON);
				shbatt_api_set_pmic_charger_transistor_switch(0);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_OFF);
				if((shchg_cur_switch_state == SHCHG_PM_SW_ON) || (shchg_cur_switch_state == SHCHG_PM_SW_POWER_FEED))
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_OFF SHCHG_WAKE_CTL(0) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(0);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_OFF;
				break;
			case SHCHG_PM_SW_ON:
			//	shbatt_api_set_pmic_battery_transistor_switch(1);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_ON);
				shbatt_api_set_pmic_charger_transistor_switch(1);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_ON);
				if(shchg_cur_switch_state == SHCHG_PM_SW_OFF)
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_ON SHCHG_WAKE_CTL(1) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(1);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_ON;
				break;
			case SHCHG_PM_SW_POWER_FEED:
			//	shbatt_api_set_pmic_battery_transistor_switch(0);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_OFF);
				shbatt_api_set_pmic_charger_transistor_switch(1);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_WAIT);
				if(shchg_cur_switch_state == SHCHG_PM_SW_OFF)
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_POWER_FEED SHCHG_WAKE_CTL(1) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(1);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_POWER_FEED;
				break;
			case SHCHG_PM_SW_DC_OFF:
			//	shbatt_api_set_pmic_battery_transistor_switch(1);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_ON);
				qpnp_chg_charger_force_run_on_batt(1);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_OFF);
				if((shchg_cur_switch_state == SHCHG_PM_SW_ON) || (shchg_cur_switch_state == SHCHG_PM_SW_POWER_FEED))
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_DC_OFF SHCHG_WAKE_CTL(0) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(0);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_OFF;
				break;
			case SHCHG_PM_SW_DC_ON:
			//	shbatt_api_set_pmic_battery_transistor_switch(1);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_ON);
				qpnp_chg_charger_force_run_on_batt(0);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_ON);
				if(shchg_cur_switch_state == SHCHG_PM_SW_OFF)
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_DC_ON SHCHG_WAKE_CTL(1) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(1);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_ON;
				break;
			case SHCHG_PM_SW_DC_POWER_FEED:
			//	shbatt_api_set_pmic_battery_transistor_switch(0);
				shbatt_api_set_pmic_battery_transistor_switch(SHBATT_TRANSISTOR_SWITCH_OFF);
				qpnp_chg_charger_force_run_on_batt(0);
				shbatt_api_notify_charge_switch_status(SHBATT_CHG_SWITCH_WAIT);
				if(shchg_cur_switch_state == SHCHG_PM_SW_OFF)
				{
					SHCHG_TRACE("[P] %s SHCHG_PM_SW_DC_POWER_FEED SHCHG_WAKE_CTL(1) \n",__FUNCTION__);
					SHCHG_WAKE_CTL(1);
				}
				shchg_cur_switch_state = SHCHG_PM_SW_POWER_FEED;
				break;
			default:
				break;
		}
	}
	while(0);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}
/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :										|*/
/*+-----------------------------------------------------------------------------+*/

static int shchg_drv_open( struct inode* in_p, struct file* fi_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shchg_drv_release( struct inode* in_p, struct file* fi_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static unsigned int shchg_drv_poll( struct file* fi_p, poll_table* wait_p )
{
	unsigned int mask = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(atomic_read(&shchg_usse_op_cnt) > 0)
	{
		atomic_dec(&shchg_usse_op_cnt);

		mask = POLLIN;
	}
	else
	{
		poll_wait(fi_p,&shchg_usse_wait,wait_p);

		complete(&shchg_usse_cmp);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return mask;
}

static long shchg_drv_ioctl( struct file* fi_p, unsigned int cmd, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(cmd == SHCHG_DRV_IOCTL_CMD_PULL_USSE_PACKET)
	{
		ret = shchg_drv_ioctl_cmd_pull_usse_packet(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_DONE_USSE_PACKET)
	{
		ret = shchg_drv_ioctl_cmd_done_usse_packet(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_INVALID)
	{
		ret = shchg_drv_ioctl_cmd_invalid(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_PARAMETER_LEVEL)
	{
		ret = shchg_drv_ioctl_cmd_get_parameter_level(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_TIMER)
	{
		ret = shchg_drv_ioctl_cmd_set_timer(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_CLR_TIMER)
	{
		ret = shchg_drv_ioctl_cmd_clr_timer(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CHARGER_VMAXSEL)
	{
		ret = shchg_drv_ioctl_cmd_set_charger_vmaxsel(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_CHARGER_VMAXSEL)
	{
		ret = shchg_drv_ioctl_cmd_get_charger_vmaxsel(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CHARGER_IMAXSEL)
	{
		ret = shchg_drv_ioctl_cmd_set_charger_imaxsel(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_CHARGER_IMAXSEL)
	{
		ret = shchg_drv_ioctl_cmd_get_charger_imaxsel(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CHARGER_CIN_LIMIT)
	{
		ret = shchg_drv_ioctl_cmd_set_charger_cin_limit(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_CHARGER_CIN_LIMIT)
	{
		ret = shchg_drv_ioctl_cmd_get_charger_cin_limit(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CHARGER_TRANSISTOR_SWITCH)
	{
		ret = shchg_drv_ioctl_cmd_set_charger_transistor_switch(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CRADLE_CB_REGIST)
	{
		ret = shchg_drv_ioctl_cmd_set_cradle_cb_resist(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_CRADLE_STATUS)
	{
		ret = shchg_drv_ioctl_cmd_get_cradle_status(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_VBSW_REG)
	{
		ret = shchg_drv_ioctl_cmd_set_vbsw_reg(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_CONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_notify_usb_charger_connected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_DISCONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_notify_usb_charger_disconnected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_CRADLE_CONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_notify_cradle_connected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_CRADLE_DISCONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_notify_cradle_disconnected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_I_IS_AVAILABLE)
	{
		ret = shchg_drv_ioctl_cmd_notify_usb_charger_i_is_available(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE)
	{
		ret = shchg_drv_ioctl_cmd_notify_usb_charger_i_is_not_available(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_BATTERY_DISCONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_notify_battery_disconnected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_NOTIFY_CHARGING_STATE_MACHINE_EXPIRE)
	{
		ret = shchg_drv_ioctl_cmd_notify_charging_state_machine_expire(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_CHECK_CHARGER_DISCONNECTED)
	{
		ret = shchg_drv_ioctl_cmd_check_charger_disconnected(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_INITIALIZE)
	{
		ret = shchg_drv_ioctl_cmd_initialize(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_KERNEL_TIME)
	{
		ret = shchg_drv_ioctl_cmd_get_kernel_time(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_SET_CHARGER_VINMIN)
	{
		ret = shchg_drv_ioctl_cmd_set_charger_vinmin(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_CHARGER_VINMIN)
	{
		ret = shchg_drv_ioctl_cmd_get_charger_vinmin(fi_p,arg);
	}
	else if(cmd == SHCHG_DRV_IOCTL_CMD_GET_MHL_CABLE_STATUS)
	{
		ret = shchg_drv_ioctl_cmd_get_mhl_cable_status(fi_p,arg);
	}
	else
	{
		ret = shchg_drv_ioctl_cmd_invalid(fi_p,arg);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_invalid( struct file* fi_p, unsigned long arg )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return -EINVAL;
}

static int shchg_drv_ioctl_cmd_pull_usse_packet( struct file* fi_p, unsigned long arg )
{
	shchg_usse_packet_t* pkt_p;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	pkt_p = &shchg_usse_pkt;

	if(copy_to_user((shchg_usse_packet_t*)arg,pkt_p,sizeof(shchg_usse_packet_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_done_usse_packet( struct file* fi_p, unsigned long arg )
{
	shchg_usse_packet_t* pkt_p;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	pkt_p = &shchg_usse_pkt;

	if(copy_from_user(pkt_p,(shchg_usse_packet_t*)arg,sizeof(shchg_usse_packet_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_parameter_level( struct file* fi_p, unsigned long arg )
{
	shchg_prm_level_info_t pli;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&pli,(shchg_prm_level_info_t*)arg,sizeof(shchg_prm_level_info_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		pli.lvl = shchg_seq_get_parameter_level(pli.prm);

		if(copy_to_user((shchg_prm_level_info_t*)arg,&pli,sizeof(shchg_prm_level_info_t)) != 0)
		{
			SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
			ret = -EPERM;
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_timer( struct file* fi_p, unsigned long arg )
{
	shchg_poll_timer_info_t pti;

	struct timespec ts;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&pti,(shchg_poll_timer_info_t*)arg,sizeof(shchg_poll_timer_info_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		ts = ktime_to_timespec(alarm_get_elapsed_realtime());

		ts.tv_sec += pti.ms / 1000;
		ts.tv_nsec += (pti.ms % 1000) * 1000000;

		if(ts.tv_nsec >= 1000000000)
		{
			ts.tv_sec += 1;
			ts.tv_nsec -= 1000000000;
		}

		if(pti.ptt == SHCHG_POLL_TIMER_TYPE_STATE_MACHINE)
		{
			shchg_tim_state_machine.prm = pti.prm;

			alarm_cancel(&shchg_tim_state_machine.alm);

			alarm_start_range(&shchg_tim_state_machine.alm,
												timespec_to_ktime(ts),
												timespec_to_ktime(ts));
		}
		else if(pti.ptt == SHCHG_POLL_TIMER_TYPE_BATTERY_DETERIORATION_DIAGNOSIS)
		{
			shchg_tim_battery_deterioration_diagnosis.prm = pti.prm;

			alarm_cancel(&shchg_tim_battery_deterioration_diagnosis.alm);

			alarm_start_range(&shchg_tim_battery_deterioration_diagnosis.alm,
												timespec_to_ktime(ts),
												timespec_to_ktime(ts));
		}
		else if(pti.ptt == SHCHG_POLL_TIMER_TYPE_CHARGER_OVP_DETECT)
		{
			shchg_tim_charger_ovp_detect.prm = pti.prm;

			alarm_cancel(&shchg_tim_charger_ovp_detect.alm);

			alarm_start_range(&shchg_tim_charger_ovp_detect.alm,
												timespec_to_ktime(ts),
												timespec_to_ktime(ts));
		}
		else
		{
			SHCHG_ERROR("%s : timer type invalid.\n",__FUNCTION__);
			ret = -EPERM;
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_clr_timer( struct file* fi_p, unsigned long arg )
{
	shchg_poll_timer_info_t pti;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&pti,(shchg_poll_timer_info_t*)arg,sizeof(shchg_poll_timer_info_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		if(pti.ptt == SHCHG_POLL_TIMER_TYPE_STATE_MACHINE)
		{
			alarm_cancel(&shchg_tim_state_machine.alm);
		}
		else if(pti.ptt == SHCHG_POLL_TIMER_TYPE_BATTERY_DETERIORATION_DIAGNOSIS)
		{
			alarm_cancel(&shchg_tim_battery_deterioration_diagnosis.alm);
		}
		else if(pti.ptt == SHCHG_POLL_TIMER_TYPE_CHARGER_OVP_DETECT)
		{
			alarm_cancel(&shchg_tim_charger_ovp_detect.alm);
		}
		else
		{
			SHCHG_ERROR("%s : timer type invalid.\n",__FUNCTION__);
			ret = -EPERM;
		}
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_charger_vmaxsel( struct file* fi_p, unsigned long arg )
{
	int mv;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&mv,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		shchg_pm_set_charger_vmaxsel(mv);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_charger_vmaxsel( struct file* fi_p, unsigned long arg )
{
	int mv = 0;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_pm_get_charger_vmaxsel(&mv);

	if(copy_to_user((int*)arg,&mv,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_charger_imaxsel( struct file* fi_p, unsigned long arg )
{
	int ma;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&ma,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		shchg_pm_set_charger_imaxsel(ma);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_charger_imaxsel( struct file* fi_p, unsigned long arg )
{
	int ma = 0;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_pm_get_charger_imaxsel(&ma);

	if(copy_to_user((int*)arg,&ma,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_charger_cin_limit( struct file* fi_p, unsigned long arg )
{
	int cl;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&cl,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		shchg_pm_set_charger_cin_limit(cl);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_charger_cin_limit( struct file* fi_p, unsigned long arg )
{
	int cl = 0;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_pm_get_charger_cin_limit(&cl);

	if(copy_to_user((int*)arg,&cl,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_charger_vinmin( struct file* fi_p, unsigned long arg )
{
	int vol;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&vol,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		shchg_pm_set_charger_vinmin(vol);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_charger_vinmin( struct file* fi_p, unsigned long arg )
{
	int vol = 0;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_pm_get_charger_vinmin(&vol);

	if(copy_to_user((int*)arg,&vol,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}


static int shchg_drv_ioctl_cmd_set_charger_transistor_switch( struct file* fi_p, unsigned long arg )
{
	shchg_pm_sw_t sw;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&sw,(shchg_pm_sw_t*)arg,sizeof(shchg_pm_sw_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		shchg_pm_set_charger_transistor_switch(sw);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_cradle_cb_resist( struct file* fi_p, unsigned long arg )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

#ifdef __SUPPORT_CRADLE__
	shswic_detect_cb_regist(SHSWIC_CHG_DEVICE,SHSWIC_ID_CRADLE,(void*)shchg_seq_cradle_detect_cb,NULL);
#endif /* __SUPPORT_CRADLE__ */

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shchg_drv_ioctl_cmd_get_cradle_status( struct file* fi_p, unsigned long arg )
{
	shchg_cradle_status_t cds;

#ifdef __SUPPORT_CRADLE__
	unsigned char status = 0;
#endif /* __SUPPORT_CRADLE__ */

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

#ifdef __SUPPORT_CRADLE__
	shswic_get_cradle_status(&status);

	cds = (status != SHSWIC_ID_CRADLE) ? SHCHG_CRADLE_STATUS_NONE : SHCHG_CRADLE_STATUS_ACTIVE;
#else  /* __SUPPORT_CRADLE__ */
	cds = SHCHG_CRADLE_STATUS_NONE;
#endif /* __SUPPORT_CRADLE__ */

	if(copy_to_user((shchg_cradle_status_t*)arg,&cds,sizeof(shchg_cradle_status_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_set_vbsw_reg( struct file* fi_p, unsigned long arg )
{
	int vbsw;

	int ret = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&vbsw,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
#ifdef __SUPPORT_CRADLE__
		ret = (shswic_write_vbsw_reg((u8)vbsw) == SHSWIC_SUCCESS) ? 0 : -EPERM;
#else  /* __SUPPORT_CRADLE__ */
		ret = 0;
#endif /* __SUPPORT_CRADLE__ */
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_usb_charger_connected( struct file* fi_p, unsigned long arg )
{
	shchg_device_t dev;

	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&dev,(shchg_device_t*)arg,sizeof(shchg_device_t)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		ret = shchg_api_notify_usb_charger_connected(dev);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

shchg_result_t shchg_api_set_log_enable( int val )
{
	shchg_packet_t* pkt_p;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_task_is_initialized == false)
	{
		return SHCHG_RESULT_REJECTED;
	}

	pkt_p = shchg_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHCHG_RESULT_REJECTED;
	}

	SHCHG_WAKE_CTL(1);

	pkt_p->hdr.cmd = SHCHG_CMD_SET_LOG_ENABLE;
	pkt_p->hdr.cb_p = NULL;
	pkt_p->hdr.cmp_p = NULL;
	pkt_p->hdr.ret_p = NULL;
	pkt_p->prm.val = val;

	INIT_WORK((struct work_struct*)pkt_p,shchg_task);

	queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return SHCHG_RESULT_SUCCESS;
}

static int shchg_drv_ioctl_cmd_notify_usb_charger_disconnected( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_usb_charger_disconnected();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_cradle_connected( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_cradle_connected();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_cradle_disconnected( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_cradle_disconnected();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_usb_charger_i_is_available( struct file* fi_p, unsigned long arg )
{
	int ima;

	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(copy_from_user(&ima,(int*)arg,sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		ret = shchg_api_notify_usb_charger_i_is_available(ima);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_usb_charger_i_is_not_available( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_usb_charger_i_is_not_available();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_battery_disconnected( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_battery_disconnected();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_notify_charging_state_machine_expire( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_notify_charging_state_machine_expire();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_check_charger_disconnected( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_check_charger_disconnected();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_initialize( struct file* fi_p, unsigned long arg )
{
	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = shchg_api_initialize();

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_kernel_time( struct file* fi_p, unsigned long arg )
{
	int ret = 0;

	struct timeval tv;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	tv = ktime_to_timeval(alarm_get_elapsed_realtime());

	if(copy_to_user((struct timeval*)arg,&tv,sizeof(struct timeval)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shchg_drv_ioctl_cmd_get_mhl_cable_status( struct file* fi_p, unsigned long arg )
{
	int ret = 0;
	int mhl_cable_status;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_api_get_mhl_cable_status( &mhl_cable_status ) != SHCHG_RESULT_SUCCESS)
	{
		SHCHG_ERROR("%s : shchg_api_get_mhl_cable_status failed.\n",__FUNCTION__);
		ret = -EPERM;
		goto error;
	}

	if(copy_to_user((int __user *)arg, &mhl_cable_status, sizeof(int)) != 0)
	{
		SHCHG_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
		goto error;
	}

error:
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static shchg_result_t shchg_api_get_mhl_cable_status( int* mhl_cable_status )
{
	shchg_packet_t* pkt_p;
	shchg_result_t result = SHCHG_RESULT_SUCCESS;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);
	do
	{
		if(shchg_task_is_initialized == false)
		{
			result = SHCHG_RESULT_REJECTED;
			break;
		}

		pkt_p = shchg_task_get_packet();

		if(pkt_p == NULL)
		{
			result = SHCHG_RESULT_REJECTED;
			break;
		}

		mutex_lock(&shchg_api_lock);

		INIT_COMPLETION(shchg_api_cmp);

		SHCHG_WAKE_CTL(1);

		pkt_p->hdr.cmd	 = SHCHG_CMD_GET_MHL_CABLE_STATUS;
		pkt_p->hdr.cb_p  = NULL;
		pkt_p->hdr.cmp_p = &shchg_api_cmp;
		pkt_p->hdr.ret_p = &result;
		pkt_p->prm.mhl_cable_status_p = mhl_cable_status;

		INIT_WORK((struct work_struct*)pkt_p,shchg_task);

		queue_work(shchg_task_workqueue_p,(struct work_struct*)pkt_p);

		wait_for_completion_killable(&shchg_api_cmp);

		mutex_unlock(&shchg_api_lock);
	}
	while(0);
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

static void shchg_task_cmd_get_mhl_cable_status( shchg_packet_t* pkt_p )
{
	shchg_result_t result;
	int mhl_cable_status = 0;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	result = shchg_seq_get_mhl_cable_status( &mhl_cable_status );

	if(pkt_p->prm.mhl_cable_status_p != NULL)
	{
		*(pkt_p->prm.mhl_cable_status_p) = mhl_cable_status;
	}

	if(pkt_p->hdr.ret_p != NULL)
	{
		*(pkt_p->hdr.ret_p) = result;
	}

	if(pkt_p->hdr.cmp_p != NULL)
	{
		complete(pkt_p->hdr.cmp_p);
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static shchg_result_t shchg_seq_get_mhl_cable_status( int* mhl_cable_status )
{
	shchg_result_t result;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_GET_MHL_CABLE_STATUS;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	result = shchg_seq_call_user_space_sequence_executor();

	if(result == SHCHG_RESULT_SUCCESS)
	{
		*mhl_cable_status = shchg_usse_pkt.prm.mhl_cable_status;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

shchg_result_t shchg_api_set_charge_ignore_flg(int val)
{
	SHCHG_TRACE("[S] %s val=%d shchg_charge_ignore_flg=%d \n",__FUNCTION__, val, shchg_charge_ignore_flg);
	
	shchg_charge_ignore_flg = val;
	
	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
	
	return SHCHG_RESULT_SUCCESS;
}

static int shchg_drv_create_device( void )
{
	struct device* dev_p;

	int ret;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	ret = alloc_chrdev_region(&shchg_dev,0,1,SHCHG_DEV_NAME);

	if(ret < 0)
	{
		SHCHG_ERROR("%s : alloc_chrdev_region failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_0;
	}

	shchg_major = MAJOR(shchg_dev);
	shchg_minor = MINOR(shchg_dev);

	cdev_init(&shchg_cdev,&shchg_fops);

	shchg_cdev.owner = THIS_MODULE;

	ret = cdev_add(&shchg_cdev,shchg_dev,1);

	if(ret < 0)
	{
		SHCHG_ERROR("%s : cdev_add failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_1;
	}

	shchg_dev_class = class_create(THIS_MODULE,SHCHG_DEV_NAME);

	if(IS_ERR(shchg_dev_class))
	{
		ret = PTR_ERR(shchg_dev_class);
		SHCHG_ERROR("%s : class_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_2;
	}

	dev_p = device_create(shchg_dev_class,NULL,shchg_dev,&shchg_cdev,SHCHG_DEV_NAME);

	if(IS_ERR(dev_p))
	{
		ret = PTR_ERR(dev_p);
		SHCHG_ERROR("%s : device_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_3;
	}

	shchg_usse_pkt.hdr.cmd = SHCHG_CMD_INVALID;
	shchg_usse_pkt.hdr.ret = SHCHG_RESULT_FAIL;
	shchg_usse_pkt.prm.val = 0;

	atomic_set(&shchg_usse_op_cnt,0);

	init_waitqueue_head(&shchg_usse_wait);

	init_completion(&shchg_usse_cmp);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return ret;

create_device_exit_3:
	class_destroy(shchg_dev_class);

create_device_exit_2:
	cdev_del(&shchg_cdev);

create_device_exit_1:
	unregister_chrdev_region(shchg_dev,1);

create_device_exit_0:

	return ret;
}

static int shchg_drv_register_irq( struct platform_device* dev_p )
{
	return 0;
}

static int shchg_drv_probe( struct platform_device* dev_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	if(shchg_drv_create_device() < 0)
	{
		SHCHG_ERROR("%s : create device failed.\n",__FUNCTION__);
		return -EPERM;
	}

	if(shchg_drv_register_irq(dev_p) < 0)
	{
		SHCHG_ERROR("%s : register irq failed.\n",__FUNCTION__);
		return -EPERM;
	}

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int __devexit shchg_drv_remove( struct platform_device* dev_p )
{
	shchg_pm_device_info_t* di_p;

	int irq;

	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	device_destroy(shchg_dev_class,shchg_dev);

	class_destroy(shchg_dev_class);

	cdev_del(&shchg_cdev);

	unregister_chrdev_region(shchg_dev,1);

	di_p = platform_get_drvdata(dev_p);

	irq = platform_get_irq(dev_p,0);

	free_irq(irq,di_p);

	platform_set_drvdata(dev_p,NULL);

	kfree(di_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void shchg_drv_shutdown( struct platform_device* dev_p )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_task_is_initialized = false;

	alarm_cancel(&shchg_tim_state_machine.alm);

	alarm_cancel(&shchg_tim_battery_deterioration_diagnosis.alm);

	alarm_cancel(&shchg_tim_charger_ovp_detect.alm);

	flush_workqueue(shchg_task_workqueue_p);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

static int __init shchg_drv_module_init( void )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	shchg_task_workqueue_p = create_singlethread_workqueue("shchg_task");

	mutex_init(&shchg_api_lock);

	mutex_init(&shchg_task_lock);

	spin_lock_init(&shchg_pkt_lock);

	memset(&shchg_pkt[0],0,sizeof(shchg_pkt));

	init_completion(&shchg_api_cmp);

	wake_lock_init(&shchg_wake_lock,WAKE_LOCK_SUSPEND,"shchg_wake");

	atomic_set(&shchg_wake_lock_num,0);

#ifdef SHCHG_ENABLE_MHL_WAKE_LOCK
	wake_lock_init(&shchg_mhl_wake_lock,WAKE_LOCK_SUSPEND,"shchg_mhl_wake");
#endif /* SHCHG_ENABLE_MHL_WAKE_LOCK */

	alarm_init(&shchg_tim_state_machine.alm,
						 ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
						 shchg_seq_state_machine_timer_expire_cb);

	alarm_init(&shchg_tim_battery_deterioration_diagnosis.alm,
						 ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
						 shchg_seq_battery_deterioration_diagnosis_timer_expire_cb);

	alarm_init(&shchg_tim_charger_ovp_detect.alm,
						 ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
						 shchg_seq_charger_ovp_detect_timer_expire_cb);

	platform_driver_register(&shchg_platform_driver);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void __exit shchg_drv_module_exit( void )
{
	SHCHG_TRACE("[S] %s \n",__FUNCTION__);

	platform_driver_unregister(&shchg_platform_driver);

	wake_lock_destroy(&shchg_wake_lock);

	SHCHG_TRACE("[E] %s \n",__FUNCTION__);
}

module_init(shchg_drv_module_init);
module_exit(shchg_drv_module_exit);

MODULE_DESCRIPTION("SH Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :															|*/
/*+-----------------------------------------------------------------------------+*/
