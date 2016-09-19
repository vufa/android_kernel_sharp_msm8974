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

#ifndef SHCHG_KERL_H
#define SHCHG_KERL_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shchg_result_tag
{
	SHCHG_RESULT_SUCCESS,
	SHCHG_RESULT_FAIL,
	SHCHG_RESULT_REJECTED,
	NUM_SHCHG_RESULT

} shchg_result_t;

typedef enum shchg_device_tag
{
	SHCHG_DEVICE_NONE,
	SHCHG_DEVICE_USB_HOST,
	SHCHG_DEVICE_USB_CHARGER,
	SHCHG_DEVICE_CRADLE_CHARGER,
	SHCHG_DEVICE_MHL,
	SHCHG_DEVICE_IRREGULAR_CHARGER,
	NUM_SHCHG_DEVICE
} shchg_device_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

shchg_result_t shchg_api_notify_usb_charger_connected( shchg_device_t dev );
shchg_result_t shchg_api_notify_usb_charger_disconnected( void );
shchg_result_t shchg_api_notify_cradle_connected( void );
shchg_result_t shchg_api_notify_cradle_disconnected( void );
shchg_result_t shchg_api_notify_usb_charger_i_is_available( int ima );
shchg_result_t shchg_api_notify_usb_charger_i_is_not_available( void );
shchg_result_t shchg_api_notify_battery_disconnected( void );
shchg_result_t shchg_api_notify_charging_state_machine_expire( void );
shchg_result_t shchg_api_check_charger_disconnected( void );
shchg_result_t shchg_api_set_log_enable( int val );
shchg_result_t shchg_api_set_charge_ignore_flg(int val);

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHCHG_KERL_H */
