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
 * ADC Voltage-Temperature convert table
 *
 */

#ifndef __QPNP_ADC_TABLE_H
#define __QPNP_ADC_TABLE_H

#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_BATTERY_SH

/* BAT_THERM Type-F */
static const struct qpnp_vadc_map_pt adcmap_bat_therm_type_f[] =
{
	{1800,	-40},
	{1571,	-20},
	{1458,	-10},
	{1317,	  0},
	{1158,	 10},
	{ 988,	 20},
	{ 819,	 30},
	{ 664,	 40},
	{ 528,	 50},
	{ 415,	 60},
	{ 325,	 70},
	{   0,	122},
};

/* BAT_THERM Type-C */
static const struct qpnp_vadc_map_pt adcmap_bat_therm_type_c[] =
{
	{1800,	-40},
	{1615,	-20},
	{1514,	-10},
	{1389,	  0},
	{1244,	 10},
	{1081,	 20},
	{ 917,	 30},
	{ 759,	 40},
	{ 616,	 50},
	{ 495,	 60},
	{ 397,	 70},
	{   0,	122},
};

/* XO_THERM Type-E */
static const struct qpnp_vadc_map_pt adcmap_xo_therm_type_e[] =
{
	{1800,	-40},
	{1657,	-20},
	{1554,	-10},
	{1407,	  0},
	{1222,	 10},
	{1009,	 20},
	{ 797,	 30},
	{ 604,	 40},
	{ 446,	 50},
	{ 324,	 60},
	{ 234,	 70},
	{   0,	122},
};

/* XO_THERM Type-F */
static const struct qpnp_vadc_map_pt adcmap_xo_therm_type_f[] =
{
	{1800,	-40},
	{1655,	-20},
	{1552,	-10},
	{1406,	  0},
	{1221,	 10},
	{1009,	 20},
	{ 797,	 30},
	{ 604,	 40},
	{ 446,	 50},
	{ 324,	 60},
	{ 234,	 70},
	{   0,	122},
};

/* PA_THERM with CAM_THERM, LCD_THERM, MSM_THERM Type-A */
static const struct qpnp_vadc_map_pt adcmap_pa_therm_type_a[] =
{
	{1800,	-45},
	{1592,	-20},
	{1453,	-10},
	{1267,	  0},
	{1049,	 10},
	{ 826,	 20},
	{ 622,	 30},
	{ 455,	 40},
	{ 326,	 50},
	{ 232,	 60},
	{ 165,	 70},
	{   0,	106},
};
/* PA_THERM with CAM_THERM, LCD_THERM, MSM_THERM Type-B */
static const struct qpnp_vadc_map_pt adcmap_pa_therm_type_b[] =
{
	{1800,	-40},
	{1655,	-20},
	{1552,	-10},
	{1406,	  0},
	{1221,	 10},
	{1009,	 20},
	{ 797,	 30},
	{ 604,	 40},
	{ 447,	 50},
	{ 324,	 60},
	{ 234,	 70},
	{   0,	122},
};

#endif /* CONFIG_BATTERY_SH */

#endif /* __QPNP_ADC_TABLE_H */
