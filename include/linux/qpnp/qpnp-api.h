/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Qualcomm PMIC QPNP BMS/CHARGER driver header file
 *
 */

#ifndef __QPNP_API_H
#define __QPNP_API_H

#include <linux/kernel.h>
#include <linux/list.h>

#ifdef CONFIG_BATTERY_SH

/* Public API */
#if defined(CONFIG_QPNP_BMS)

int qpnp_bms_get_vbatt_avg(int *result_mV, int *adc_code);
int qpnp_bms_get_vsense_avg_read(int *result_mV, int *result_mA);
int qpnp_bms_get_battery_current(int *result_mA);
int qpnp_bms_reset_cc(void);
int qpnp_bms_get_cc(int64_t offset, int64_t *result_cc, int64_t *result_mAs);
int qpnp_bms_enable(bool enable);
int qpnp_bms_set_vbatt_alarm(int min_mv, int max_mv, int alarm_type);
int qpnp_bms_sync_read_batt_v_and_i(int *vbat_uv, int *ibat_ua);

#else  /* CONFIG_QPNP_BMS */

static inline int qpnp_bms_get_vbatt_avg(int *result_mV, int *adc_code)
{ return -ENXIO; }
static inline int qpnp_bms_get_vsense_avg_read(int *result_mV, int *result_mA)
{ return -ENXIO; }
static inline int qpnp_bms_get_battery_current(int *result_mA)
{ return -ENXIO; }
static inline int qpnp_bms_reset_cc(void)
{ return -ENXIO; }
static inline int qpnp_bms_get_cc(int64_t offset, int64_t *result_cc, int64_t *result_mAs)
{ return -ENXIO; }
static inline int qpnp_bms_enable(bool enable)
{ return -ENXIO; }
static inline int qpnp_bms_set_vbatt_alarm(int min_mv, int max_mv, int alarm_type)
{ return -ENXIO; }
static inline int qpnp_bms_sync_read_batt_v_and_i(int *vbat_uv, int *ibat_ua)
{ return -ENXIO; }

#endif /* CONFIG_QPNP_BMS */

/* Public API */
#if defined(CONFIG_QPNP_CHARGER)

int qpnp_chg_charger_transistor_switch(int enable);
int qpnp_chg_battery_transistor_switch(int enable);
int qpnp_chg_charger_vinmin_set(int voltage);
int qpnp_chg_charger_vinmin_get(int* voltage);
int qpnp_chg_charger_ibatmax_set(int chg_current);
int qpnp_chg_charger_vddmax_set(int voltage);
int qpnp_chg_charger_vddmax_get(int* voltage);
int qpnp_chg_charger_iusbmax_set(int chg_current);
int qpnp_chg_charger_iusbmax_get(int* mA);
int qpnp_chg_charger_is_otg_en_set(void);
int qpnp_chg_charger_switch_usb_to_host_mode(void);
int qpnp_chg_charger_switch_usb_to_charge_mode(void);
int qpnp_chg_charger_force_run_on_batt(int disable);
int qpnp_chg_charger_idcmax_set(int mA);
int qpnp_chg_charger_idcmax_get(int* mA);
int qpnp_chg_batfet_status(bool* batfet_closed);

#else  /* CONFIG_QPNP_CHARGER */

static inline int qpnp_chg_charger_transistor_switch(int enable)
{ return -ENXIO; }
static inline int qpnp_chg_battery_transistor_switch(int enable)
{ return -ENXIO; }
static inline int qpnp_chg_charger_vinmin_set(int voltage)
{ return -ENXIO; }
static inline int qpnp_chg_charger_vinmin_get(int* voltage)
{ return -ENXIO; }
static inline int qpnp_chg_charger_ibatmax_set(int chg_current)
{ return -ENXIO; }
static inline int qpnp_chg_charger_vddmax_set(int voltage)
{ return -ENXIO; }
static inline int qpnp_chg_charger_vddmax_get(int* voltage)
{ return -ENXIO; }
static inline int qpnp_chg_charger_iusbmax_set(int chg_current)
{ return -ENXIO; }
static inline int qpnp_chg_charger_iusbmax_set(int* mA)
{ return -ENXIO; }
static inline int qpnp_chg_charger_is_otg_en_set(void)
{ return -ENXIO; }
static inline int qpnp_chg_charger_switch_usb_to_host_mode(void)
{ return -ENXIO; }
static inline int qpnp_chg_charger_switch_usb_to_charge_mode(void)
{ return -ENXIO; }
static inline int qpnp_chg_charger_force_run_on_batt(int disable)
{ return -ENXIO; }
static inline int qpnp_chg_charger_idcmax_set(int mA)
{ return -ENXIO; }
static inline int qpnp_chg_charger_idcmax_get(int* mA)
{ return -ENXIO; }
static inline int qpnp_chg_batfet_status(bool* batfet_closed)
{ return -ENXIO; }
#endif /* CONFIG_REGULATOR_QPNP */

#if defined(CONFIG_REGULATOR_QPNP)
int qpnp_regulator_boost_bypass_enable(int enable);
#else /* CONFIG_REGULATOR_QPNP */
static inline int qpnp_regulator_boost_bypass_enable(int enable)
{ return -ENXIO; }
#endif /* CONFIG_REGULATOR_QPNP */

#if defined(CONFIG_LEDS_QPNP)
int qpnp_torch_is_enable(void);
#else /* CONFIG_LEDS_QPNP */
static int qpnp_torch_is_enable(void)
{ return -ENXIO; }
#endif /* CONFIG_LEDS_QPNP */


#endif /* CONFIG_BATTERY_SH */

#endif /* __QPNP_API_H */
