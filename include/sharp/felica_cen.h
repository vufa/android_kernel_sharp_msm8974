/* include/sharp/felica_cen.h  (FeliCa driver)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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

#ifndef FELICA_CEN_H
#define FELICA_CEN_H

/* I2C					*/
#define DRIVER_NAME 						("felica_cen")
#define	SH_MFD_CEN  						DRIVER_NAME

/* FeliCa Lock */
#define FELICA_CONTROL_LOCK_MASK					0x01			/* rock status mask					*/

#define FELICA_WRITE_REGISTER_ACCESS_MODE			0
#define FELICA_WRITE_E2PROM_ACCESS_MODE				1

/* write lock status	*/
#define SHMFD_CEN_WRITE_LOCK_ON						0				/* lock 							*/
#define SHMFD_CEN_WRITE_LOCK_OFF					1				/* Unlock   						*/
#define SHMFD_CEN_WRITE_LOCK_TEMPOFF				2				/* TempUnlok						*/

/* read lock status		*/
#define SHMFD_CEN_READ_LOCK_ON						0				/* lock								*/
#define SHMFD_CEN_READ_LOCK_OFF						1				/* Unlock							*/

#define CEN_LOCK_ON									0				/* lock								*/
#define CEN_LOCK_OFF								1				/* Unlock							*/

#define FELICA_GET_STATUS_UNLOCK					0x01			/* Unlock							*/

/* item_id				*/
#define FELICA_PPC_RELOAD_COMMAND					0x00			/* reload command					*/
#define FELICA_PPC_REVERSE_OUTPUT_COMMAND			0x01			/* reverse output command			*/
#define FELICA_PPC_E2PROM_WRITE_ENABLE_COMMAND		0x02			/* write E2PROM enable command		*/
#define FELICA_PPC_E2PROM_WRITE_DISABLE_COMMAND		0x03			/* write E2PROM disable command		*/
#define FELICA_PPC_CHANGE_ACCESSMODE_COMMAND		0x04			/* change access mode command		*/
#define FELICA_PPC_READ_STATUS_REGISTER_COMMAND		0x05			/* read status register				*/
#define FELICA_PPC_SET_PORT_DATA_COMMAND			0x06			/* set port command					*/
#define FELICA_PPC_SET_SLAVEADDRESS_COMMAND			0x07			/* set slave address command		*/

/* status_register mask */
#define FELICA_PPC_STATUS_OUTPUT_MASK				0x01			/* port status						*/
#define FELICA_PPC_STATUS_ACCESSMODE_MASK			0x02			/* access mode status				*/
#define FELICA_PPC_STATUS_WRITE_PROTECT_MASK		0x04			/* write protect status				*/
#define FELICA_PPC_STATUS_POR_MASK					0x08			/* POR status						*/

/* access mode			*/
#define FELICA_PPC_STATUS_REGISTER_ACCESSMODE		0x00			/* register access mode				*/
#define FELICA_PPC_STATUS_E2PROM_ACCESSMODE			0x02			/* E2PROM access mode				*/

#define FELICA_PPC_STATUS_POR						0x08			/* POR								*/

/* set_data				*/
#define FELICA_PPC_CHANGE_REGISTER_ACCESSMODE		0
#define FELICA_PPC_CHANGE_E2PROM_ACCESSMODE			1

#define FELICA_PPC_SET_REGISTER_ACCESSMODE			0x00			/* register access mode				*/
#define FELICA_PPC_SET_E2PROM_ACCESSMODE			0x01			/* E2PROM access mode				*/

#define FELICA_PPC_WRITE_DISABLE					0				/* write E2PROM disable				*/
#define FELICA_PPC_WRITE_ENABLE						1				/* write E2PROM enable				*/

#define FELICA_PPC_NO_WAIT							0				/*  no wait flag(i2c)				*/
#define FELICA_PPC_NEED_WAIT						1				/* 	wait flag						*/

#define I2C_WRITE_WAIT_TIME							10				/* WAIT TIME						*/

#define FELICA_PPC_E2PROM_ITEM_ID_MIN				0x04
#define FELICA_PPC_E2PROM_ITEM_ID_MAX				0x07

#define FELICA_PPC_REGISTER_ITEM_ID_MIN				0x04
#define FELICA_PPC_REGISTER_ITEM_ID_MAX				0x07

#define FELICA_PPC_I2C_TRY_COUNT					3				/* I2C retry count					*/

#define FALSE										-1
#define TRUE										0

#define I2C_FAILURE									-1
#define I2C_SUCCESS									0

#endif /* FELICA_CEN_H */
