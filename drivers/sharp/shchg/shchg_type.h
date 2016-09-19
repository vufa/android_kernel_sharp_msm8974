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

#ifndef SHCHG_TYPE_H
#define SHCHG_TYPE_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include "shchg_def.h"

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

  /* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shchg_get_prm_tag
{
  SHCHG_GET_PRM_BATTERY_VOLT,
  SHCHG_GET_PRM_WALL_CHARGER_VOLT,
  SHCHG_GET_PRM_USB_CHARGER_VOLT,
  SHCHG_GET_PRM_BATTERY_I,
  SHCHG_GET_PRM_CHARGER_I,
  SHCHG_GET_PRM_BATTERY_ID,
  SHCHG_GET_PRM_BATT_THERM_DEGC,
  SHCHG_GET_PRM_CHG_THERM_DEGC,
  SHCHG_GET_PRM_CAMERA_THERM_DEGC,
  SHCHG_GET_PRM_PMIC_DIE_DEGC,
  SHCHG_GET_PRM_BATTERY_VOLT_BUF,
  SHCHG_GET_PRM_WALL_CHARGER_VOLT_BUF,
  SHCHG_GET_PRM_USB_CHARGER_VOLT_BUF,
  SHCHG_GET_PRM_BATTERY_I_BUF,
  SHCHG_GET_PRM_CHARGER_I_BUF,
  SHCHG_GET_PRM_BATTERY_ID_BUF,
  SHCHG_GET_PRM_BATT_THERM_DEGC_BUF,
  SHCHG_GET_PRM_CHG_THERM_DEGC_BUF,
  SHCHG_GET_PRM_CAMERA_THERM_DEGC_BUF,
  SHCHG_GET_PRM_PMIC_DIE_DEGC_BUF,
  SHCHG_GET_PRM_DUMMY_READ,
  SHCHG_GET_PRM_BATT_THERM_DEGC_NO_MOVING_AVERAGE,
  SHCHG_GET_PRM_PA_THERM0_DEGC_BUF,
  SHCHG_GET_PRM_LCD_THERM_DEGC_BUF,
  NUM_SHCHG_GET_PRM

} shchg_get_prm_t;

typedef enum shchg_pm_sw_tag
{
  SHCHG_PM_SW_OFF,
  SHCHG_PM_SW_ON,
  SHCHG_PM_SW_POWER_FEED,
  SHCHG_PM_SW_DC_OFF,
  SHCHG_PM_SW_DC_ON,
  SHCHG_PM_SW_DC_POWER_FEED,
  NUM_SHCHG_PM_SW

} shchg_pm_sw_t;

typedef enum shchg_cradle_status_tag
{
  SHCHG_CRADLE_STATUS_NONE,
  SHCHG_CRADLE_STATUS_ACTIVE,
  NUM_SHCHG_CRADLE_STATUS

} shchg_cradle_status_t;

typedef enum shchg_poll_timer_type_tag
{
  SHCHG_POLL_TIMER_TYPE_STATE_MACHINE,
  SHCHG_POLL_TIMER_TYPE_BATTERY_DETERIORATION_DIAGNOSIS,
  SHCHG_POLL_TIMER_TYPE_CHARGER_OVP_DETECT,
  NUM_SHCHG_POLL_TIMER_TYPE

} shchg_poll_timer_type_t;

typedef enum shchg_cmd_tag
{
  SHCHG_CMD_INVALID,
  SHCHG_CMD_NOTIFY_USB_CHARGER_CONNECTED,
  SHCHG_CMD_NOTIFY_USB_CHARGER_DISCONNECTED,
  SHCHG_CMD_NOTIFY_CRADLE_CONNECTED,
  SHCHG_CMD_NOTIFY_CRADLE_DISCONNECTED,
  SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_AVAILABLE,
  SHCHG_CMD_NOTIFY_USB_CHARGER_I_IS_NOT_AVAILABLE,
  SHCHG_CMD_NOTIFY_BATTERY_DISCONNECTED,
  SHCHG_CMD_NOTIFY_CHARGING_STATE_MACHINE_EXPIRE,
  SHCHG_CMD_CHECK_CHARGER_DISCONNECTED,
  SHCHG_CMD_INITIALIZE,
  SHCHG_CMD_EXEC_STATE_MACHINE_SEQUENCE,
  SHCHG_CMD_EXEC_BATTERY_DETERIORATION_DIAGNOSIS_SEQUENCE,
  SHCHG_CMD_EXEC_CHARGER_OVP_DETECT_SEQUENCE,
  SHCHG_CMD_SET_LOG_ENABLE,
  SHCHG_CMD_GET_MHL_CABLE_STATUS,
  NUM_SHCHG_CMD

} shchg_cmd_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shchg_packet_hdr_tag
{
  shchg_cmd_t cmd;
  void* cb_p;
  struct completion* cmp_p;
  shchg_result_t* ret_p;

} shchg_packet_hdr_t;

typedef union shchg_packet_prm_tag
{
  shchg_device_t dev;
  int ima;
  int val;
  int* mhl_cable_status_p;

} shchg_packet_prm_t;

typedef struct shchg_packet_tag
{
  struct work_struct work;
  bool is_used;
  shchg_packet_hdr_t hdr;
  shchg_packet_prm_t prm;

} shchg_packet_t;

typedef struct shchg_timer_tag
{
  struct alarm alm;
  int prm;

} shchg_timer_t;

typedef struct shchg_pm_device_info_tag
{
  struct device* dev_p;

} shchg_pm_device_info_t;

typedef struct shchg_usse_packet_hdr_tag
{
  shchg_cmd_t cmd;
  shchg_result_t ret;

} shchg_usse_packet_hdr_t;

typedef union shchg_usse_packet_prm_tag
{
  shchg_device_t dev;
  int ima;
  int val;
  int mhl_cable_status;

} shchg_usse_packet_prm_t;

typedef struct shchg_usse_packet_tag
{
  shchg_usse_packet_hdr_t hdr;
  shchg_usse_packet_prm_t prm;

} shchg_usse_packet_t;

typedef struct shchg_prm_level_info_tag
{
  shchg_get_prm_t prm;
  int lvl;

} shchg_prm_level_info_t;

typedef struct shchg_pm_reg_info_tag
{
  int mod;
  int reg;
  unsigned char buf[8];
  int len;

} shchg_pm_reg_info_t;

typedef struct shchg_poll_timer_info_tag
{
  shchg_poll_timer_type_t ptt;
  int ms;
  int prm;

} shchg_poll_timer_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ CALLBACK FUNCTION TYPE DECLARE :                                          |*/
/*+-----------------------------------------------------------------------------+*/

typedef void (*shchg_cb_func_t1)( shchg_result_t result );

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif  /* SHCHG_TYPE_H */
