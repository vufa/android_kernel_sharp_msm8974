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

#ifndef SHBATT_TYPE_H
#define SHBATT_TYPE_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <sharp/shbatt_kerl.h>
#include <linux/android_alarm.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

#define SHBIF_IOCTL_MAGIC 'o'

#define SHBATT_IOCTL_MAGIC 'b'

#define SHBATT_DRV_IOCTL_CMD_GET_CHARGER_CABLE_STATUS				_IOR( SHBATT_IOCTL_MAGIC,   1, shbatt_cable_status_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CHARGING_STATUS			_IOR( SHBATT_IOCTL_MAGIC,   2, shbatt_chg_status_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CAPACITY					_IOR( SHBATT_IOCTL_MAGIC,   3, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_VOLTAGE					_IOR( SHBATT_IOCTL_MAGIC,   4, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_PRESENT					_IOR( SHBATT_IOCTL_MAGIC,   5, shbatt_present_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_HEALTH 					_IOR( SHBATT_IOCTL_MAGIC,   6, shbatt_health_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_TECHNOLOGY 				_IOR( SHBATT_IOCTL_MAGIC,   7, shbatt_technology_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CURRENT					_IOR( SHBATT_IOCTL_MAGIC,   8, int*)
#define SHBATT_DRV_IOCTL_CMD_READ_ADC_CHANNEL						_IOWR(SHBATT_IOCTL_MAGIC,   9, shbatt_adc_t*)
#define SHBATT_DRV_IOCTL_CMD_SET_VBATT_CALIBRATION_DATA 			_IOW( SHBATT_IOCTL_MAGIC,  10, shbatt_vbatt_cal_t)

#define SHBATT_DRV_IOCTL_CMD_REFRESH_VBATT_CALIBRATION_DATA 		_IO(  SHBATT_IOCTL_MAGIC,  11)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_CHARGER_TRANSISTOR_SWITCH		_IOW( SHBATT_IOCTL_MAGIC,  12, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_BATTERY_TRANSISTOR_SWITCH		_IOW( SHBATT_IOCTL_MAGIC,  13, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_VMAXSEL						_IOW( SHBATT_IOCTL_MAGIC,  14, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_IMAXSEL						_IOW( SHBATT_IOCTL_MAGIC,  15, int)
#define SHBATT_DRV_IOCTL_CMD_CHECK_STARTUP_VOLTAGE					_IOR( SHBATT_IOCTL_MAGIC,  16, int*)
#define SHBATT_DRV_IOCTL_CMD_CHECK_STARTUP_BATTERY_PRESENT_CHECK	_IO(  SHBATT_IOCTL_MAGIC,  17)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_TEMPERATURE				_IOR( SHBATT_IOCTL_MAGIC,  18, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_CHARGER_TEMPERATURE				_IOR( SHBATT_IOCTL_MAGIC,  19, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_CAMERA_TEMPERATURE 				_IOR( SHBATT_IOCTL_MAGIC,  20, int*)

#define SHBATT_DRV_IOCTL_CMD_GET_PA_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  21, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_XO_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  22, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_PMIC_TEMPERATURE					_IOR( SHBATT_IOCTL_MAGIC,  23, int*)
#define SHBATT_DRV_IOCTL_CMD_SET_SMBC_CHARGER						_IOW( SHBATT_IOCTL_MAGIC,  24, shbatt_smbc_chg_t)
#define SHBATT_DRV_IOCTL_CMD_READ_CC								_IOR( SHBATT_IOCTL_MAGIC,  25, shbatt_adc_conv_offset_t*)
#define SHBATT_DRV_IOCTL_CMD_READ_VSENSE_AVG						_IOR( SHBATT_IOCTL_MAGIC,  26, shbatt_adc_conv_int_t*)
#define SHBATT_DRV_IOCTL_CMD_READ_VBATT_AVG							_IOR( SHBATT_IOCTL_MAGIC,  27, shbatt_adc_conv_int_t*)
#define SHBATT_DRV_IOCTL_CMD_RECALIB_ADC_DEVICE						_IO(  SHBATT_IOCTL_MAGIC,  28)
#define SHBATT_DRV_IOCTL_CMD_GET_CALIBRATE_BATTERY_VOLTAGE			_IOR( SHBATT_IOCTL_MAGIC,  29, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_WIRELESS_STATUS					_IOR( SHBATT_IOCTL_MAGIC,  30, int*)

#define SHBATT_DRV_IOCTL_CMD_SET_WIRELESS_SWITCH					_IOW( SHBATT_IOCTL_MAGIC,  31, int)
#define SHBATT_DRV_IOCTL_CMD_SET_VSENSE_AVG_CALIBRATION_DATA 		_IOW( SHBATT_IOCTL_MAGIC,  32, shbatt_vsense_avg_cal_t)
#define SHBATT_DRV_IOCTL_CMD_REFRESH_VSENSE_AVG_CALIBRATION_DATA	_IO(  SHBATT_IOCTL_MAGIC,  33)
#define SHBATT_DRV_IOCTL_CMD_GET_HW_REVISION						_IOR( SHBATT_IOCTL_MAGIC,  34, uint*)
#define SHBATT_DRV_IOCTL_CMD_GET_SMEM_INFO							_IOR( SHBATT_IOCTL_MAGIC,  35, shbatt_smem_info_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_LCD_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  36, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_MSM_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  37, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_APQ_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  38, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_PA1_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC,  39, int*)
#define SHBATT_DRV_IOCTL_CMD_CONV_GPIO_PORT							_IOR( SHBATT_IOCTL_MAGIC,  40,  shbatt_port_conv_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_TPIN_STATUS	 					_IOR( SHBATT_IOCTL_MAGIC,  41, int*)

/*| TODO: New API add point */

#define SHBATT_IOC_MAGIC 's'

#define SHBATT_DRV_IOCTL_CMD_INITIALIZE											_IO(  SHBATT_IOC_MAGIC,  1)
#define SHBATT_DRV_IOCTL_CMD_PULL_USSE_PACKET									_IOR( SHBATT_IOC_MAGIC,  2, shbatt_usse_packet_t)
#define SHBATT_DRV_IOCTL_CMD_DONE_USSE_PACKET									_IOW( SHBATT_IOC_MAGIC,  3, shbatt_usse_packet_t)
#define SHBATT_DRV_IOCTL_CMD_SET_TIMER											_IOW( SHBATT_IOC_MAGIC,  4, shbatt_poll_timer_info_t)
#define SHBATT_DRV_IOCTL_CMD_CLR_TIMER											_IOW( SHBATT_IOC_MAGIC,  5, shbatt_poll_timer_info_t)
#define SHBATT_DRV_IOCTL_CMD_SET_VOLTAGE_ALARM									_IOW( SHBATT_IOC_MAGIC,  6, shbatt_voltage_alarm_info_t)
#define SHBATT_DRV_IOCTL_CMD_CLR_VOLTAGE_ALARM									_IOW( SHBATT_IOC_MAGIC,  7, shbatt_voltage_alarm_info_t)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_SAFETY									_IOR( SHBATT_IOC_MAGIC,  8, shbatt_safety_t)
#define SHBATT_DRV_IOCTL_CMD_GET_TERMINAL_TEMPERATURE							_IOR( SHBATT_IOC_MAGIC,  9, int)
#define SHBATT_DRV_IOCTL_CMD_GET_MODEM_TEMPERATURE								_IOR( SHBATT_IOC_MAGIC, 10, int)

#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_CURRENT								_IOR( SHBATT_IOC_MAGIC, 11, int)
#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_VOLTAGE								_IOR( SHBATT_IOC_MAGIC, 12, int)
#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_DEVICE_ID							_IOR( SHBATT_IOC_MAGIC, 13, unsigned int)
#define SHBATT_DRV_IOCTL_CMD_SET_FUELGAUGE_MODE									_IOW( SHBATT_IOC_MAGIC, 14, shbatt_fuelgauge_mode_t)
#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_ACCUMULATE_CURRENT					_IOR( SHBATT_IOC_MAGIC, 15, shbatt_accumulate_current_t)
#define SHBATT_DRV_IOCTL_CMD_CLR_FUELGAUGE_ACCUMULATE_CURRENT					_IO(  SHBATT_IOC_MAGIC, 16)
#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_TEMPERATURE							_IOR( SHBATT_IOC_MAGIC, 17, int)
#define SHBATT_DRV_IOCTL_CMD_GET_FUELGAUGE_CURRENT_AD							_IOR( SHBATT_IOC_MAGIC, 18, int)
#define SHBATT_DRV_IOCTL_CMD_READ_ADC_CHANNEL_BUFFERED							_IOWR(SHBATT_IOC_MAGIC, 19, shbatt_adc_t)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_LOG_INFO								_IOR( SHBATT_IOC_MAGIC, 20, shbatt_batt_log_info_t)

#define SHBATT_DRV_IOCTL_CMD_POST_BATTERY_LOG_INFO								_IOW( SHBATT_IOC_MAGIC, 21, shbatt_batt_log_info_t)
#define SHBATT_DRV_IOCTL_CMD_NOTIFY_CHARGER_CABLE_STATUS						_IOW( SHBATT_IOC_MAGIC, 22, shbatt_cable_status_t)
#define SHBATT_DRV_IOCTL_CMD_NOTIFY_BATTERY_CHARGING_STATUS						_IOW( SHBATT_IOC_MAGIC, 23, shbatt_chg_status_t)
#define SHBATT_DRV_IOCTL_CMD_NOTIFY_BATTERY_CAPACITY							_IOW( SHBATT_IOC_MAGIC, 24, int)
#define SHBATT_DRV_IOCTL_CMD_NOTIFY_CHARGING_STATE_MACHINE_ENABLE				_IOW( SHBATT_IOC_MAGIC, 25, shbatt_boolean_t)
#define SHBATT_DRV_IOCTL_CMD_NOTIFY_POWER_SUPPLY_CHANGED						_IOW( SHBATT_IOC_MAGIC, 26, shbatt_ps_category_t)
#define SHBATT_DRV_IOCTL_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE					_IOW( SHBATT_IOC_MAGIC, 27, int)
#define SHBATT_DRV_IOCTL_CMD_GET_KERNEL_TIME									_IOR( SHBATT_IOC_MAGIC, 28, struct timeval)
#define SHBATT_DRV_IOCTL_CMD_READ_ADC_CHANNEL_NO_CONVERSION						_IOWR(SHBATT_IOC_MAGIC, 29, shbatt_adc_t*)
#if defined(SHBATT_ENABLE_WIRELESS)	//wirelss
#define SHBATT_DRV_IOCTL_CMD_ENABLE_WIRELESS_CHARGER							_IO(  SHBATT_IOC_MAGIC, 30)

#define SHBATT_DRV_IOCTL_CMD_DISABLE_WIRELESS_CHARGER							_IO(  SHBATT_IOC_MAGIC, 31)
#define SHBATT_DRV_IOCTL_CMD_GET_WIRELESS_DISABLE_STATE							_IOR( SHBATT_IOC_MAGIC, 32, shbatt_wireless_disable_state_t)
#define SHBATT_DRV_IOCTL_CMD_GET_WIRELESS_CHARGING_STATUS						_IOR( SHBATT_IOC_MAGIC, 33, shbatt_wireless_charging_state_t)
#endif /* SHBATT_ENABLE_WIRELESS */
#define SHBATT_DRV_IOCTL_CMD_SET_LOG_ENABLE										_IOR( SHBATT_IOC_MAGIC, 34, int)
#define SHBATT_DRV_IOCTL_CMD_GET_DEPLETED_CAPACITY								_IOR( SHBATT_IOC_MAGIC, 35, int)
#define SHBATT_DRV_IOCTL_CMD_SET_DEPLETED_BATTERY_FLG							_IOW( SHBATT_IOC_MAGIC, 36, int)
#define SHBATT_DRV_IOCTL_CMD_GET_DEPLETED_BATTERY_FLG							_IOR( SHBATT_IOC_MAGIC, 37, int)
#define SHBATT_DRV_IOCTL_CMD_SET_BATTERY_HEALTH									_IOW( SHBATT_IOC_MAGIC, 38, shbatt_health_t)
#define SHBATT_DRV_IOCTL_CMD_SET_ANDROID_SHUTDOWN_TIMER							_IOR( SHBATT_IOC_MAGIC, 39, int)
#define SHBATT_DRV_IOCTL_CMD_GET_AVE_CURRENT_AND_AVE_VOLTAGE					_IOR( SHBATT_IOC_MAGIC, 40, shbatt_current_voltage_t)

#define SHBATT_DRV_IOCTL_CMD_READ_ADC_CHANNELS									_IOR( SHBATT_IOC_MAGIC, 41, shbatt_adc_read_request_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_UPDATE					_IO(  SHBATT_IOC_MAGIC, 42)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_GET_FVER					_IOR( SHBATT_IOC_MAGIC, 43, int)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_GET_HWVER				_IOR( SHBATT_IOC_MAGIC, 44, int)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_GO_TO_CALIBRATION_MODE				_IO(  SHBATT_IOC_MAGIC, 45)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_VOLTAGE_CALIBRATION				_IOWR(SHBATT_IOC_MAGIC, 46, shbatt_wlchg_voltage_calibration_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_VOLTAGE_AND_CURRENT_CALIBRATION	_IOWR(SHBATT_IOC_MAGIC, 47, shbatt_wlchg_voltage_and_current_calibration_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_CURRENT_CALIBRATION				_IOWR(SHBATT_IOC_MAGIC, 48, shbatt_wlchg_current_calibration_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_WRITE_WPC_SERIAL_NO				_IOW( SHBATT_IOC_MAGIC, 49, uint16_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGE_GET_CALIBRATED_VALUE				_IOR( SHBATT_IOC_MAGIC, 50, shbatt_wlchg_calibrated_value_t)

#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_SET_BATT_TEMP			_IOW( SHBATT_IOC_MAGIC, 51, uint8_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_GET_BATT_TEMP			_IOR( SHBATT_IOC_MAGIC, 52, uint8_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_SET_CPU_TEMP				_IOR( SHBATT_IOC_MAGIC, 53, uint8_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_GET_CPU_TEMP				_IOW( SHBATT_IOC_MAGIC, 54, uint8_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_GET_RECTIFIER_IC_TEMP	_IOR( SHBATT_IOC_MAGIC, 55, uint8_t)
#define SHBATT_DRV_IOCTL_CMD_WIRELESS_CHARGER_FIRMWARE_UPDATE_FROM_BUFFER		_IOW( SHBATT_IOC_MAGIC, 56, shbatt_wlchg_firmware_buffer_t)
#define SHBATT_DRV_IOCTL_CMD_WLCHG_READ_I2C_DATA								_IOR( SHBATT_IOC_MAGIC, 57, shbatt_wlchg_i2c_data_t)
#define SHBATT_DRV_IOCTL_CMD_WLCHG_WRITE_I2C_DATA								_IOW( SHBATT_IOC_MAGIC, 58, shbatt_wlchg_i2c_data_t)
#define SHBATT_DRV_IOCTL_CMD_GET_AVERAGE_CURRENT								_IOR( SHBATT_IOC_MAGIC, 59, int)
#if 0
#define SHBATT_DRV_IOCTL_CMD_CALIB_CCADC										_IO(  SHBATT_IOC_MAGIC, 60)
#else
#define SHBATT_DRV_IOCTL_CMD_CALIB_CCADC										_IOR( SHBATT_IOC_MAGIC, 60, shbatt_calib_ccadc_info_t)
#endif

#define SHBATT_DRV_IOCTL_CMD_GET_SYSTEM_TIME									_IOR( SHBATT_IOC_MAGIC, 61, struct timeval)
#define SHBATT_DRV_IOCTL_CMD_RESET_CC											_IO(  SHBATT_IOC_MAGIC, 62)
#define SHBATT_DRV_IOCTL_CMD_SET_DEPLETED_CALC_ENABLE							_IOR( SHBATT_IOC_MAGIC, 63, int)
#define SHBATT_DRV_IOCTL_CMD_SET_CHARGE_IGNORE_FLG								_IOR( SHBATT_IOC_MAGIC, 64, int)
#define SHBATT_DRV_IOCTL_CMD_SET_FORCE_RUN_ON_BATT								_IOR( SHBATT_IOC_MAGIC, 65, int)
#define SHBATT_DRV_IOCTL_CMD_SET_IDC_MAX										_IOR( SHBATT_IOC_MAGIC, 66, int)
#define SHBATT_DRV_IOCTL_CMD_GET_IDC_MAX										_IOR( SHBATT_IOC_MAGIC, 67, int)
#define SHBATT_DRV_IOCTL_CMD_GET_IUSB_MAX										_IOR( SHBATT_IOC_MAGIC, 68, int)
#define SHBATT_DRV_IOCTL_CMD_CPU_CLOCK_LIMIT_LOCK								_IOR( SHBATT_IOC_MAGIC, 69, shbatt_limit_lock_level_t)
#define SHBATT_DRV_IOCTL_CMD_CPU_CLOCK_LIMIT_UNLOCK								_IO(  SHBATT_IOC_MAGIC, 70)

/*| TODO: New API add point */

#define SHBATT_PS_VALUE_FULL_VOLTAGE			4310
#define SHBATT_PS_VALUE_EMPTYE_VOLTAGE			3400
#define SHBATT_PS_VALUE_FULL_CAPACITY			1800

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE : 													|*/
/*+-----------------------------------------------------------------------------+*/
/*attributes.*/
/*cable(ac,usb)status attributes.*/
enum
{
	SHBATT_PS_PROPERTY_ONLINE,
};
/*battery attributes.*/
enum
{
	SHBATT_PS_PROPERTY_STATUS,
	SHBATT_PS_PROPERTY_HEALTH,
	SHBATT_PS_PROPERTY_PRESENT,
	SHBATT_PS_PROPERTY_CAPACITY,
	SHBATT_PS_PROPERTY_BATT_VOL,
	SHBATT_PS_PROPERTY_BATT_TEMP,
	SHBATT_PS_PROPERTY_TECHNOLOGY,
	SHBATT_PS_PROPERTY_SAFETY,
	SHBATT_PS_PROPERTY_CAMERA_TEMP,
	SHBATT_PS_PROPERTY_TERMINAL_TEMP,
	SHBATT_PS_PROPERTY_MODEM_TEMP,
	SHBATT_PS_PROPERTY_CURRENT_NOW,
	SHBATT_PS_PROPERTY_VOLTAGE_NOW,
	SHBATT_PS_PROPERTY_VOLTAGE_MAX_DESIGN,
	SHBATT_PS_PROPERTY_VOLTAGE_MIN_DESIGN,
	SHBATT_PS_PROPERTY_ENERGY_FULL,
};

/*fuelgauge attributes.*/
enum
{
	SHBATT_PS_PROPERTY_CURRENT,
	SHBATT_PS_PROPERTY_VOLTAGE,
	SHBATT_PS_PROPERTY_DEVICE_ID,
	SHBATT_PS_PROPERTY_MODE,
	SHBATT_PS_PROPERTY_ACCUMULATE_CURRENT,
	SHBATT_PS_PROPERTY_FGIC_TEMP,
	SHBATT_PS_PROPERTY_CURRENT_AD,
};
/*pmic attributes.*/
enum
{
	SHBATT_PS_PROPERTY_CHARGER_TRANSISTOR_SWITCH,
	SHBATT_PS_PROPERTY_VMAXSEL,
	SHBATT_PS_PROPERTY_IMAXSEL,
};
/*adc attributes.*/
enum
{
	SHBATT_PS_PROPERTY_GPADC_IN0,
	SHBATT_PS_PROPERTY_GPADC_IN1,
	SHBATT_PS_PROPERTY_GPADC_IN2,
	SHBATT_PS_PROPERTY_GPADC_IN3,
	SHBATT_PS_PROPERTY_GPADC_IN4,
	SHBATT_PS_PROPERTY_GPADC_IN5,
	SHBATT_PS_PROPERTY_GPADC_IN6,
	SHBATT_PS_PROPERTY_VBAT,
	SHBATT_PS_PROPERTY_VBKP,
	SHBATT_PS_PROPERTY_VAC,
	SHBATT_PS_PROPERTY_VBUS,
	SHBATT_PS_PROPERTY_ICHG,
	SHBATT_PS_PROPERTY_HOTDIE1,
	SHBATT_PS_PROPERTY_HOTDIE2,
	SHBATT_PS_PROPERTY_ID,
	SHBATT_PS_PROPERTY_TESTV,
	SHBATT_PS_PROPERTY_CHRG_LED_TEST,
	SHBATT_PS_PROPERTY_GPADC_IN0_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN1_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN2_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN3_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN4_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN5_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN6_BUF,
	SHBATT_PS_PROPERTY_VBAT_BUF,
	SHBATT_PS_PROPERTY_VBKP_BUF,
	SHBATT_PS_PROPERTY_VAC_BUF,
	SHBATT_PS_PROPERTY_VBUS_BUF,
	SHBATT_PS_PROPERTY_ICHG_BUF,
	SHBATT_PS_PROPERTY_HOTDIE1_BUF,
	SHBATT_PS_PROPERTY_HOTDIE2_BUF,
	SHBATT_PS_PROPERTY_ID_BUF,
	SHBATT_PS_PROPERTY_TESTV_BUF,
	SHBATT_PS_PROPERTY_CHRG_LED_TEST_BUF,
	SHBATT_PS_PROPERTY_VBATT_CALIBRATION,
	SHBATT_PS_PROPERTY_CPU_TEMP,
	SHBATT_PS_PROPERTY_LCD_TEMP,
	SHBATT_PS_PROPERTY_PA0_TEMP,
};
typedef enum shbatt_api_to_tsk_command_tag
{
	SHBATT_TASK_CMD_INVALID,
	SHBATT_TASK_CMD_GET_CHARGER_CABLE_STATUS,
	SHBATT_TASK_CMD_GET_BATTERY_CHARGING_STATUS,
	SHBATT_TASK_CMD_GET_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_GET_BATTERY_VOLTAGE,
	SHBATT_TASK_CMD_GET_BATTERY_HEALTH,
	SHBATT_TASK_CMD_GET_BATTERY_TECHNOLOGY,
	SHBATT_TASK_CMD_GET_BATTERY_CURRENT,
	SHBATT_TASK_CMD_READ_ADC_CHANNEL,
	SHBATT_TASK_CMD_SET_VBATT_CALIBRATION_DATA,
	SHBATT_TASK_CMD_REFRESH_VBATT_CALIBRATION_DATA,
	SHBATT_TASK_CMD_SET_PMIC_CHARGER_TRANSISTOR_SWITCH,
	SHBATT_TASK_CMD_SET_PMIC_BATTERY_TRANSISTOR_SWITCH,
	SHBATT_TASK_CMD_SET_PMIC_VMAXSEL,
	SHBATT_TASK_CMD_SET_PMIC_IMAXSEL,
	SHBATT_TASK_CMD_CHECK_STARTUP_VOLTAGE,
	SHBATT_TASK_CMD_CHECK_STARTUP_BATTERY_PRESENT_CHECK,
	SHBATT_TASK_CMD_GET_BATTERY_TEMPERATURE,
	SHBATT_TASK_CMD_GET_CHARGER_TEMPERATURE,
	SHBATT_TASK_CMD_GET_CAMERA_TEMPERATURE,
	SHBATT_TASK_CMD_GET_PA_TEMPERATURE,
	SHBATT_TASK_CMD_GET_XO_TEMPERATURE,
	SHBATT_TASK_CMD_GET_PMIC_TEMPERATURE,
	SHBATT_TASK_CMD_SET_SMBC_CHARGER,
	SHBATT_TASK_CMD_READ_CC,
	SHBATT_TASK_CMD_READ_VSENSE_AVG,
	SHBATT_TASK_CMD_READ_VBATT_AVG,
	SHBATT_TASK_CMD_RECALIB_ADC_DEVICE,
	SHBATT_TASK_CMD_GET_TERMINAL_TEMPERATURE,
	SHBATT_TASK_CMD_GET_MODEM_TEMPERATURE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_VOLTAGE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_DEVICE_ID,
	SHBATT_TASK_CMD_SET_FUELGAUGE_MODE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_TASK_CMD_CLR_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_TASK_CMD_GET_FUELGAUGE_TEMPERATURE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_CURRENT_AD,
	SHBATT_TASK_CMD_READ_ADC_CHANNEL_BUFFERED,
	SHBATT_TASK_CMD_GET_BATTERY_LOG_INFO,
	SHBATT_TASK_CMD_NOTIFY_CHARGER_CABLE_STATUS,
	SHBATT_TASK_CMD_NOTIFY_BATTERY_CHARGING_STATUS,
	SHBATT_TASK_CMD_NOTIFY_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_NOTIFY_CHARGING_STATE_MACHINE_ENABLE,
	SHBATT_TASK_CMD_GET_BATTERY_SAFETY,
	SHBATT_TASK_CMD_INITIALIZE,
	SHBATT_TASK_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_POST_BATTERY_LOG_INFO,
	SHBATT_TASK_CMD_EXEC_BATTERY_PRESENT_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_EXEC_OVERCURR_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	SHBATT_TASK_CMD_GET_CALIBRATE_BATTERY_VOLTAGE,
	SHBATT_TASK_CMD_GET_WIRELESS_STATUS,
	SHBATT_TASK_CMD_SET_WIRELESS_SWITCH,
	SHBATT_TASK_CMD_SET_VSENSE_AVG_CALIBRATION_DATA,
	SHBATT_TASK_CMD_REFRESH_VSENSE_AVG_CALIBRATION_DATA,
	SHBATT_TASK_CMD_GET_BATTERY_PRESENT,
	SHBATT_TASK_CMD_NOTIFY_WIRELESS_CHARGER_STATE_CHANGED,
	SHBATT_TASK_CMD_SET_LOG_ENABLE,
	SHBATT_TASK_CMD_GET_REAL_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_NOTIFY_CHARGE_SWITCH_STATUS,
	SHBATT_TASK_CMD_GET_DEPLETED_CAPACITY,
	SHBATT_TASK_CMD_SET_DEPLETED_BATTERY_FLG,
	SHBATT_TASK_CMD_GET_DEPLETED_BATTERY_FLG,
	SHBATT_TASK_CMD_SET_BATTERY_HEALTH,
	SHBATT_TASK_CMD_NOTIFY_WIRELESS_CHARGE_TIMER_EXPIRE,
	SHBATT_TASK_CMD_GET_AVERAGE_CURRENT,
	SHBATT_TASK_CMD_SET_DEPLETED_CALC_ENABLE,
/*| TODO: New API add point */
	NUM_SHBATT_TASK_CMD,
} shbatt_api_to_tsk_command_t;

typedef enum
{
	SHBATT_CMD_INVALID,
	SHBATT_CMD_GET_CHARGER_CABLE_STATUS,
	SHBATT_CMD_GET_BATTERY_CHARGING_STATUS,
	SHBATT_CMD_GET_BATTERY_HEALTH,
	SHBATT_CMD_GET_BATTERY_PRESENT,
	SHBATT_CMD_GET_BATTERY_CAPACITY,
	SHBATT_CMD_GET_BATTERY_VOLTAGE,
	SHBATT_CMD_GET_BATTERY_TEMPERATURE,
	SHBATT_CMD_GET_BATTERY_TECHNOLOGY,
	SHBATT_CMD_GET_BATTERY_SAFETY,
	SHBATT_CMD_GET_CAMERA_TEMPERATURE,
	SHBATT_CMD_GET_TERMINAL_TEMPERATURE,
	SHBATT_CMD_GET_MODEM_TEMPERATURE,
	SHBATT_CMD_GET_FUELGAUGE_VOLTAGE,
	SHBATT_CMD_GET_FUELGAUGE_DEVICE_ID,
	SHBATT_CMD_SET_FUELGAUGE_MODE,
	SHBATT_CMD_GET_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_CMD_CLR_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_CMD_GET_FUELGAUGE_TEMPERATURE,
	SHBATT_CMD_GET_FUELGAUGE_CURRENT_AD,
	SHBATT_CMD_SET_PMIC_VMAXSEL,
	SHBATT_CMD_SET_PMIC_IMAXSEL,
	SHBATT_CMD_READ_ADC_CHANNEL,
	SHBATT_CMD_READ_ADC_CHANNEL_BUFFERED,
	SHBATT_CMD_SET_VBATT_CALIBRATION_DATA,
	SHBATT_CMD_REFRESH_VBATT_CALIBRATION_DATA,
	SHBATT_CMD_CHECK_STARTUP_VOLTAGE,
	SHBATT_CMD_GET_BATTERY_LOG_INFO,
	SHBATT_CMD_POST_BATTERY_LOG_INFO,
	SHBATT_CMD_NOTIFY_CHARGER_CABLE_STATUS,
	SHBATT_CMD_NOTIFY_BATTERY_CHARGING_STATUS,
	SHBATT_CMD_NOTIFY_BATTERY_CAPACITY,
	SHBATT_CMD_NOTIFY_CHARGING_STATE_MACHINE_ENABLE,
	SHBATT_CMD_GET_CALIBRATE_BATTERY_VOLTAGE,
	SHBATT_CMD_INITIALIZE,
	SHBATT_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	SHBATT_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE,
	SHBATT_CMD_EXEC_OVERCURR_CHECK_SEQUENCE,
	SHBATT_CMD_EXEC_BATTERY_PRESENT_CHECK_SEQUENCE,
	SHBATT_CMD_NOTIFY_WIRELESS_CHARGER_STATE_CHANGED,
	SHBATT_CMD_SET_LOG_ENABLE,
	SHBATT_CMD_GET_REAL_BATTERY_CAPACITY,
	SHBATT_CMD_NOTIFY_CHARGE_SWITCH_STATUS,
	SHBATT_CMD_GET_DEPLETED_CAPACITY,
	SHBATT_CMD_SET_DEPLETED_BATTERY_FLG,
	SHBATT_CMD_GET_DEPLETED_BATTERY_FLG,
	SHBATT_CMD_SET_BATTERY_HEALTH,
	SHBATT_CMD_NOTIFY_WIRELESS_CHARGE_TIMER_EXPIRE,
	SHBATT_CMD_GET_AVERAGE_CURRENT,
	SHBATT_CMD_SET_DEPLETED_CALC_ENABLE,
	NUM_SHBATT_CMD,
} shbatt_kernel_to_user_command_t;

typedef enum shbatt_low_battery_event_tag
{
  SHBATT_LOW_BATTERY_EVENT_LOW_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_FATAL_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_LOW_TIMER,
  SHBATT_LOW_BATTERY_EVENT_FATAL_TIMER,
  SHBATT_LOW_BATTERY_EVENT_CHARGING_FATAL_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_CHARGING_FATAL_TIMER,
  SHBATT_LOW_BATTERY_EVENT_LOW_SHUTDOWN,
  SHBATT_LOW_BATTERY_EVENT_CHARGER_CONNECT,
  SHBATT_LOW_BATTERY_EVENT_CHARGER_DISCONNECT,
  NUM_SHBATT_LOW_BATTERY_EVENT

} shbatt_low_battery_event_t;

typedef enum shbatt_timer_type_tag
{
  SHBATT_TIMER_TYPE_0,
  SHBATT_TIMER_TYPE_1,
  NUM_SHBATT_TIMER_TYPE

} shbatt_timer_type_t;

typedef enum shbatt_timer_sleep_type_tag
{
  SHBATT_TIMER_TYPE_WAKEUP,
  SHBATT_TIMER_TYPE_SLEEP,
  NUM_SHBATT_TIMER_SLEEP_TYPE

} shbatt_timer_sleep_type_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shbatt_drv_adc_type_tag
{
	SHBATT_DRV_ADC_TYPE_UNKNOWN,
	SHBATT_DRV_ADC_TYPE_VOLTAGE,
	SHBATT_DRV_ADC_TYPE_CURRENT,

} shbatt_drv_adc_type_t;

typedef enum shbatt_drv_adc_channel_tag
{
	/* adc voltage */
	SHBATT_DRV_ADC_CHANNEL_USBIN		= 0x00,	// [  0] USBIN                       = usb_in
	SHBATT_DRV_ADC_CHANNEL_DCIN			= 0x01,	// [  1] DCIN                        = dc_in
	SHBATT_DRV_ADC_CHANNEL_VCHG_SNS		= 0x02,	// [  2] VCHG_SNS                    = vchg_sns
	SHBATT_DRV_ADC_CHANNEL_VCOIN		= 0x05,	// [  5] VCOIN                       = vcoin
	SHBATT_DRV_ADC_CHANNEL_VBAT_SNS		= 0x06,	// [  6] VBAT_SNS                    = vbat_sns
	SHBATT_DRV_ADC_CHANNEL_VPH_PWR		= 0x07,	// [  7] VSYS                        = vph_pwr
	SHBATT_DRV_ADC_CHANNEL_PMIC_TEMP	= 0x08,	// [  8] DIE_TEMP                    = die_temp
	SHBATT_DRV_ADC_CHANNEL_625MV		= 0x09,	// [  9] REF_625MV                   = ref_625mv
	SHBATT_DRV_ADC_CHANNEL_125V			= 0x0A,	// [ 10] REF_125V                    = ref_1250v
	SHBATT_DRV_ADC_CHANNEL_CHG_TEMP		= 0x0B,	// [ 11] CHG_TEMP                    = chg_temp
	SHBATT_DRV_ADC_CHANNEL_GND_REF		= 0x0E,	// [ 14] GND_REF                     = gnd_ref
	SHBATT_DRV_ADC_CHANNEL_VDD_VADC		= 0x0F,	// [ 15] VDD_VADC                    = vdd_vadc
//	SHBATT_DRV_ADC_CHANNEL_MUX2_1_1		= 0x11,	// [ 17] P_MUX2_1_1(MPP2)            = mux2_1_1
//	SHBATT_DRV_ADC_CHANNEL_MUX5_1_1		= 0x14,	// [ 20] P_MUX5_1_1(MPP5)            = mux5_1_1
	SHBATT_DRV_ADC_CHANNEL_ICHG			= 0x21,	// [ 33] P_MUX2_1_3(MPP2)            = chg_cur_out
	SHBATT_DRV_ADC_CHANNEL_BAT_THERM	= 0x30,	// [ 48] LR_MUX1_BATT_THERM          = batt_therm
	SHBATT_DRV_ADC_CHANNEL_BAT_ID		= 0x31,	// [ 49] LR_MUX2_BAT_ID              = batt_id
#if 0
	SHBATT_DRV_ADC_CHANNEL_MSM_THERM	= 0x73,	// [115] LR_MUX4_PU1_AMUX_THM1       = msm_therm_pu1
	SHBATT_DRV_ADC_CHANNEL_CAM_THERM	= 0x74,	// [116] LR_MUX5_PU1_AMUX_THM2       = cam_therm_pu1
	SHBATT_DRV_ADC_CHANNEL_PA_THERM0	= 0x75,	// [117] LR_MUX6_PU1_AMUX_THM3       = pa_therm0_pu1
	SHBATT_DRV_ADC_CHANNEL_PA_THERM1	= 0x77,	// [119] LR_MUX8_PU1_AMUX_THM4       = pa_therm1_pu1
	SHBATT_DRV_ADC_CHANNEL_LCD_THERM	= 0x78,	// [120] LR_MUX9_PU1_AMUX_THM5       = lcd_therm_pu1
#endif
	SHBATT_DRV_ADC_CHANNEL_USB_ID		= 0x79,	// [121] LR_MUX10_PU1_AMUX_USB_ID_LV = usb_id_pu1
	SHBATT_DRV_ADC_CHANNEL_XO_THERM		= 0xB2,	// [178] LR_MUX3_PU2_XO_THERM        = xo_therm_pu2
#if 1
	SHBATT_DRV_ADC_CHANNEL_MSM_THERM	= 0xB3,	// [179] LR_MUX4_PU2_AMUX_THM1       = msm_therm_pu2
	SHBATT_DRV_ADC_CHANNEL_CAM_THERM	= 0xB4,	// [180] LR_MUX5_PU2_AMUX_THM2       = cam_therm_pu2
	SHBATT_DRV_ADC_CHANNEL_PA_THERM0	= 0xB5,	// [181] LR_MUX6_PU2_AMUX_THM3       = pa_therm0_pu2
	SHBATT_DRV_ADC_CHANNEL_PA_THERM1	= 0xB7,	// [183] LR_MUX8_PU2_AMUX_THM4       = pa_therm1_pu2
	SHBATT_DRV_ADC_CHANNEL_LCD_THERM	= 0xB8,	// [184] LR_MUX9_PU2_AMUX_THM5       = lcd_therm_pu2
#endif
//	SHBATT_DRV_ADC_CHANNEL_USB_ID		= 0xB9,	// [185] LR_MUX10_PU2_AMUX_USB_ID_LV = usb_id

	SHBATT_DRV_ADC_CHANNEL_VBAT = SHBATT_DRV_ADC_CHANNEL_VBAT_SNS,
	SHBATT_DRV_ADC_CHANNEL_BAT_THERM_NO_MOVING_AVE = SHBATT_DRV_ADC_CHANNEL_BAT_THERM,
	SHBATT_DRV_ADC_CHANNEL_PA_THERM = SHBATT_DRV_ADC_CHANNEL_PA_THERM0,

	/* adc current */
//	SHBATT_DRV_ADC_CHANNEL_IBAT			= 0x00,	// [  0] INTERNAL_RSENSE = internal_rsense
	SHBATT_DRV_ADC_CHANNEL_IBAT			= 0x01,	// [  1] EXTERNAL_RSENSE = external_rsense

} shbatt_drv_adc_channel_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shbatt_drv_adc_read_tag
{
	shbatt_drv_adc_type_t		type;
	shbatt_drv_adc_channel_t	channel;

} shbatt_drv_adc_read_t;

typedef struct shbatt_soc_tag
{
  shbatt_timer_type_t type;
  shbatt_timer_sleep_type_t sleep;

} shbatt_soc_t;

typedef struct shbatt_packet_hdr_tag
{
	shbatt_api_to_tsk_command_t	cmd;
	void* 					cb_p;
	struct completion*		cmp_p;
	shbatt_result_t* 		ret_p;

} shbatt_packet_hdr_t;

typedef union shbatt_packet_prm_tag
{
	void						*param_p;
	shbatt_cable_status_t*		cbs_p;
	shbatt_chg_status_t*		cgs_p;
	shbatt_health_t*			hea_p;
	shbatt_present_t*			pre_p;
	int*						cap_p;
	int*						vol_p;
	int*						tmp_p;
	shbatt_technology_t*		tec_p;
	int*						cur_p;
	shbatt_adc_t*				adc_p;
	int*						chk_p;
	shbatt_adc_conv_uint_t*		ocv_p;
	shbatt_adc_conv_uint_t*		vsense_p;
	shbatt_adc_conv_uint_t*		vbatt_p;
	int*						cc_p;
	shbatt_adc_conv_int_t*		vsense_avg_p;
	shbatt_adc_conv_int_t*		vbatt_avg_p;
	shbatt_adc_conv_offset_t*	cc_mah_p;
	shbatt_vbatt_cal_t			cal;
	shbatt_cable_status_t		cbs;
	shbatt_chg_status_t			cgs;
	int							cap;
	int							val;
	shbatt_smbc_chg_t			chg;
	shbatt_vsense_avg_cal_t		vac;
	int*						val_p;

	unsigned int* dev_p;
	shbatt_fuelgauge_mode_t mode;
	shbatt_accumulate_current_t* acc_p;
	int* raw_p;
	shbatt_batt_log_info_t* bli_p;
	shbatt_boolean_t ena;
	shbatt_safety_t* saf_p;
	int evt;
	shbatt_batt_log_info_t bli;

	int seq;
	shbatt_soc_t soc;
	shbatt_chg_switch_status_t switch_status;
	int*						depleted_capacity_per;
	int depleted_battery_flg;
	int* depleted_battery_flg_p;
	shbatt_health_t				hea;
} shbatt_packet_prm_t;

typedef struct shbatt_packet_tag
{
	struct work_struct		work;
	shbatt_packet_hdr_t 	hdr;
	shbatt_packet_prm_t 	prm;
	bool is_used;	
} shbatt_packet_t;


typedef struct shbatt_pm_device_info_tag
{
  struct device* dev_p;

} shbatt_pm_device_info_t;
typedef enum shbatt_ps_category_tag
{
  SHBATT_PS_CATEGORY_BATTERY,
  SHBATT_PS_CATEGORY_USB,
  SHBATT_PS_CATEGORY_AC,
  SHBATT_PS_CATEGORY_FUELGAUGE,
  SHBATT_PS_CATEGORY_PMIC,
  SHBATT_PS_CATEGORY_ADC,
  SHBATT_PS_CATEGORY_WIRELESS,
  NUM_SHBATT_POWER_SUPPLY_CAT

} shbatt_ps_category_t;
typedef enum shbatt_poll_timer_type_tag
{
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_MULTI,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP_MULTI,
  SHBATT_POLL_TIMER_TYPE_LOW_BATTERY,
  SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY,
  SHBATT_POLL_TIMER_TYPE_CHARGING_FATAL_BATTERY,
  SHBATT_POLL_TIMER_TYPE_LOW_BATTERY_SHUTDOWN,
  SHBATT_POLL_TIMER_TYPE_OVERCURR,
  SHBATT_POLL_TIMER_TYPE_BATTERY_PRESENT,
  SHBATT_POLL_TIMER_TYPE_WIRELESS_CHARGE,
  NUM_SHBATT_POLL_TIMER_TYPE

} shbatt_poll_timer_type_t;

typedef struct shbatt_usse_packet_hdr_tag
{
  shbatt_kernel_to_user_command_t cmd;
  shbatt_result_t ret;

} shbatt_usse_packet_hdr_t;

typedef union shbatt_usse_packet_prm_tag
{
  shbatt_cable_status_t cbs;
  shbatt_chg_status_t cgs;
  shbatt_health_t hea;
  shbatt_present_t pre;
  int cap;
  int vol;
  int tmp;
  shbatt_technology_t tec;
  shbatt_safety_t saf;
  int cur;
  unsigned int dev;
  shbatt_fuelgauge_mode_t mode;
  shbatt_accumulate_current_t acc;
  int raw;
  shbatt_adc_t adc;
  shbatt_vbatt_cal_t cal;
  shbatt_batt_log_info_t bli;
  shbatt_boolean_t ena;
  int evt;
  int seq;
  int val;
  shbatt_boolean_t chk;
  shbatt_vsense_avg_cal_t vac;
  shbatt_soc_t soc;
  shbatt_chg_switch_status_t switch_status;
  int depleted_capacity_per;
  int depleted_battery_flg;
} shbatt_usse_packet_prm_t;

typedef struct shbatt_usse_packet_tag
{
  shbatt_usse_packet_hdr_t hdr;
  shbatt_usse_packet_prm_t prm;

} shbatt_usse_packet_t;

typedef struct shbatt_timer_tag
{
  struct alarm alm;
  int prm;

} shbatt_timer_t;

typedef struct shbatt_pm_reg_info_tag
{
  int mod;
  int reg;
  unsigned char buf[8];
  int len;

} shbatt_pm_reg_info_t;

typedef struct shbatt_fg_reg_info_tag
{
  int slv;
  int reg;
  unsigned char buf[8];
  int len;

} shbatt_fg_reg_info_t;

typedef struct shbatt_adc_conv_info_tag
{
	shbatt_adc_t	adc;
	long			read_adc_time_sec;
	long			read_adc_time_usec;

} shbatt_adc_conv_info_t;

typedef struct shbatt_poll_timer_info_tag
{
  shbatt_poll_timer_type_t ptt;
  int ms;
  int prm;

} shbatt_poll_timer_info_t;

typedef struct shbatt_voltage_alarm_info_tag
{
  shbatt_voltage_alarm_type_t vat;
  int max;
  int min;

} shbatt_voltage_alarm_info_t;


typedef struct
{
	int cur;
	int vol;
} shbatt_current_voltage_t;

typedef struct shbatt_adc_read_request_tag
{
	int request_num;
	shbatt_adc_t channels[NUM_SHBATT_ADC_CHANNEL];
} shbatt_adc_read_request_t;




typedef struct
{
	uint16_t setting_voltage_mv;
	
	uint16_t result_voltage_mv;
	uint16_t result_set_voltage_mv;
} shbatt_wlchg_voltage_calibration_t;

typedef struct
{
	uint16_t setting_voltage_mv;
	uint16_t setting_current_ma;
	
	uint16_t result_voltage_mv;
	uint16_t result_current_ma;
	uint16_t result_set_voltage_mv;
	uint16_t result_set_current_ma;
} shbatt_wlchg_voltage_and_current_calibration_t;

typedef struct
{
	uint16_t setting_current_ma;
	
	uint16_t result_current_ma;
	uint16_t result_set_current_ma;
} shbatt_wlchg_current_calibration_t;

typedef struct
{
	int8_t		battery_temp;
	int8_t		ic_temp;
	uint16_t	voltage_mv;
	uint16_t	current_ma;
	uint16_t	wpc_serial_no;
	uint16_t	v_gain;
	int16_t		v_offset;
	uint16_t	i_gain;
	int16_t		i_offset;
} shbatt_wlchg_calibrated_value_t;

typedef struct
{
	unsigned char* data;
	long size;
} shbatt_wlchg_firmware_buffer_t;

typedef struct
{
	uint8_t address;
	uint8_t *data;
	uint8_t size;
} shbatt_wlchg_i2c_data_t;

typedef struct shbatt_calib_ccadc_info_tag
{
	int			skip_calib;
	uint16_t	calc_gain;
	uint16_t	calc_offset;
} shbatt_calib_ccadc_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ CALLBACK FUNCTION TYPE DECLARE :                                          |*/
/*+-----------------------------------------------------------------------------+*/

typedef void (*shbatt_cb_func_t1)( shbatt_result_t result );

typedef void (*shbatt_cb_func_t2)( int val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t3)( shbatt_present_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t4)( shbatt_health_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t5)( shbatt_technology_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t6)( shbatt_safety_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t7)( shbatt_cable_status_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t8)( shbatt_chg_status_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t9)( shbatt_adc_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t10)( unsigned int val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t11)( shbatt_accumulate_current_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t12)( shbatt_batt_log_info_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t13)( void* arg_p );

typedef void (*shbatt_cb_func_t14)( shbatt_boolean_t chk, shbatt_result_t result );


/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_TYPE_H */
