/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>

#ifdef CONFIG_BATTERY_SH
#include <sharp/sh_smem.h>
#include "qpnp-adc-table.h"
#endif /* CONFIG_BATTERY_SH */

/* Min ADC code represets 0V */
#define QPNP_VADC_MIN_ADC_CODE			0x6000
/* Max ADC code represents full-scale range of 1.8V */
#define QPNP_VADC_MAX_ADC_CODE			0xA800
#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0,	1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};

static const struct qpnp_vadc_map_pt adcmap_qrd_btm_threshold[] = {
	{-200,	1540},
	{-180,	1517},
	{-160,	1492},
	{-140,	1467},
	{-120,	1440},
	{-100,	1412},
	{-80,	1383},
	{-60,	1353},
	{-40,	1323},
	{-20,	1292},
	{0,	1260},
	{20,	1228},
	{40,	1196},
	{60,	1163},
	{80,	1131},
	{100,	1098},
	{120,	1066},
	{140,	1034},
	{160,	1002},
	{180,	971},
	{200,	941},
	{220,	911},
	{240,	882},
	{260,	854},
	{280,	826},
	{300,	800},
	{320,	774},
	{340,	749},
	{360,	726},
	{380,	703},
	{400,	681},
	{420,	660},
	{440,	640},
	{460,	621},
	{480,	602},
	{500,	585},
	{520,	568},
	{540,	552},
	{560,	537},
	{580,	523},
	{600,	510},
	{620,	497},
	{640,	485},
	{660,	473},
	{680,	462},
	{700,	452},
	{720,	442},
	{740,	433},
	{760,	424},
	{780,	416},
	{800,	408},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuaa_btm_threshold[] = {
	{-200,	1476},
	{-180,	1450},
	{-160,	1422},
	{-140,	1394},
	{-120,	1365},
	{-100,	1336},
	{-80,	1306},
	{-60,	1276},
	{-40,	1246},
	{-20,	1216},
	{0,	1185},
	{20,	1155},
	{40,	1126},
	{60,	1096},
	{80,	1068},
	{100,	1040},
	{120,	1012},
	{140,	986},
	{160,	960},
	{180,	935},
	{200,	911},
	{220,	888},
	{240,	866},
	{260,	844},
	{280,	824},
	{300,	805},
	{320,	786},
	{340,	769},
	{360,	752},
	{380,	737},
	{400,	722},
	{420,	707},
	{440,	694},
	{460,	681},
	{480,	669},
	{500,	658},
	{520,	648},
	{540,	637},
	{560,	628},
	{580,	619},
	{600,	611},
	{620,	603},
	{640,	595},
	{660,	588},
	{680,	582},
	{700,	575},
	{720,	569},
	{740,	564},
	{760,	559},
	{780,	554},
	{800,	549},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skug_btm_threshold[] = {
	{-200,	1338},
	{-180,	1307},
	{-160,	1276},
	{-140,	1244},
	{-120,	1213},
	{-100,	1182},
	{-80,	1151},
	{-60,	1121},
	{-40,	1092},
	{-20,	1063},
	{0,	1035},
	{20,	1008},
	{40,	982},
	{60,	957},
	{80,	933},
	{100,	910},
	{120,	889},
	{140,	868},
	{160,	848},
	{180,	830},
	{200,	812},
	{220,	795},
	{240,	780},
	{260,	765},
	{280,	751},
	{300,	738},
	{320,	726},
	{340,	714},
	{360,	704},
	{380,	694},
	{400,	684},
	{420,	675},
	{440,	667},
	{460,	659},
	{480,	652},
	{500,	645},
	{520,	639},
	{540,	633},
	{560,	627},
	{580,	622},
	{600,	617},
	{620,	613},
	{640,	608},
	{660,	604},
	{680,	600},
	{700,	597},
	{720,	593},
	{740,	590},
	{760,	587},
	{780,	585},
	{800,	582},
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_100k_104ef_104fb[] = {
	{1758,	-40},
	{1742,	-35},
	{1719,	-30},
	{1691,	-25},
	{1654,	-20},
	{1608,	-15},
	{1551,	-10},
	{1483,	-5},
	{1404,	0},
	{1315,	5},
	{1218,	10},
	{1114,	15},
	{1007,	20},
	{900,	25},
	{795,	30},
	{696,	35},
	{605,	40},
	{522,	45},
	{448,	50},
	{383,	55},
	{327,	60},
	{278,	65},
	{237,	70},
	{202,	75},
	{172,	80},
	{146,	85},
	{125,	90},
	{107,	95},
	{92,	100},
	{79,	105},
	{68,	110},
	{59,	115},
	{51,	120},
	{44,	125}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_150k_104ef_104fb[] = {
	{1738,	-40},
	{1714,	-35},
	{1682,	-30},
	{1641,	-25},
	{1589,	-20},
	{1526,	-15},
	{1451,	-10},
	{1363,	-5},
	{1266,	0},
	{1159,	5},
	{1048,	10},
	{936,	15},
	{825,	20},
	{720,	25},
	{622,	30},
	{533,	35},
	{454,	40},
	{385,	45},
	{326,	50},
	{275,	55},
	{232,	60},
	{195,	65},
	{165,	70},
	{139,	75},
	{118,	80},
	{100,	85},
	{85,	90},
	{73,	95},
	{62,	100},
	{53,	105},
	{46,	110},
	{40,	115},
	{34,	120},
	{30,	125}
};

static int32_t qpnp_adc_map_voltage_temp(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
#ifndef CONFIG_BATTERY_SH
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
#else  /* CONFIG_BATTERY_SH */
		*output = (((int32_t) ((int64_t)(pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
#endif /* CONFIG_BATTERY_SH */
	}

	return 0;
}

static int32_t qpnp_adc_map_temp_voltage(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
#ifndef CONFIG_BATTERY_SH
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
#else  /* CONFIG_BATTERY_SH */
		*output = (((int32_t) ((int64_t)(pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
#endif /* CONFIG_BATTERY_SH */
	}

	return 0;
}

static int64_t qpnp_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

#ifdef CONFIG_BATTERY_SH
static void qpnp_adc_map_select_bat_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	int select = 0;

#if (CONFIG_PM_BAT_THERM_TYPE == 1)
	select = 1;
#endif /* CONFIG_PM_BAT_THERM_TYPE */

	if (select == 1)
	{
		*ptr = adcmap_bat_therm_type_c;
		*siz = ARRAY_SIZE(adcmap_bat_therm_type_c);
	}
	else
	{
		*ptr = adcmap_bat_therm_type_f;
		*siz = ARRAY_SIZE(adcmap_bat_therm_type_f);
	}
}

static void qpnp_adc_map_select_xo_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	int select = 0;

#if (CONFIG_PM_XO_THERM_TYPE == 1)
	select = 1;
#endif /* CONFIG_PM_XO_THERM_TYPE */

	if (select == 1)
	{
		*ptr = adcmap_xo_therm_type_f;
		*siz = ARRAY_SIZE(adcmap_xo_therm_type_f);
	}
	else
	{
		*ptr = adcmap_xo_therm_type_e;
		*siz = ARRAY_SIZE(adcmap_xo_therm_type_e);
	}
}

static void qpnp_adc_map_select_pa_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	int select = 0;

#if (CONFIG_PM_PA_THERM_TYPE == 1)
	select = 1;
#endif /* CONFIG_PM_PA_THERM_TYPE */

	if (select == 1)
	{
		*ptr = adcmap_pa_therm_type_a;
		*siz = ARRAY_SIZE(adcmap_pa_therm_type_a);
	}
	else
	{
		*ptr = adcmap_pa_therm_type_b;
		*siz = ARRAY_SIZE(adcmap_pa_therm_type_b);
	}
}

static void qpnp_adc_map_select_cam_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	/* use pa_therm table */
	qpnp_adc_map_select_pa_therm(ptr, siz);
}

#ifdef CONFIG_PM_LCD_THERM
static void qpnp_adc_map_select_lcd_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	/* use pa_therm table */
	qpnp_adc_map_select_pa_therm(ptr, siz);
}
#endif /* CONFIG_PM_LCD_THERM */

#ifdef CONFIG_PM_MSM_THERM
static void qpnp_adc_map_select_msm_therm( const struct qpnp_vadc_map_pt **ptr, int *siz )
{
	/* use pa_therm table */
	qpnp_adc_map_select_pa_therm(ptr, siz);
}
#endif /* CONFIG_PM_MSM_THERM */

typedef void (*qpnp_adc_map_select_fn)( const struct qpnp_vadc_map_pt **ptr, int *siz );

static int32_t qpnp_adc_scale_voltage_therm_by_table(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result,
		enum qpnp_adc_calib_type calib_type,
		const qpnp_adc_map_select_fn qpnp_adc_map_select)
{
	int result = 0;
	const struct qpnp_vadc_map_pt* table_ptr;
	int table_siz;
	int32_t voltage;

	if (!qpnp_adc_map_select)
	{
		result = -EBADF;
		pr_err("qpnp_adc_map_select error %d\n", result);
		goto error;
	}

	if (calib_type == CALIB_RATIOMETRIC)
	{
		voltage = qpnp_adc_scale_ratiometric_calib(adc_code, adc_properties, chan_properties);
		if (voltage < 0)
		{
			result = voltage;
			pr_err("qpnp_adc_scale_ratiometric_calib error %d, adc_code %d\n", result, adc_code);
			goto error;
		}

		adc_chan_result->measurement = voltage;
		adc_chan_result->microvolts = (int32_t)adc_chan_result->measurement * 1000;
	}
	else
	{
		result = -EINVAL;
		pr_err("calib_type error %d\n", result);
		goto error;
	}

	(*qpnp_adc_map_select)(&table_ptr, &table_siz);

	result = qpnp_adc_map_voltage_temp(
			table_ptr,
			table_siz,
			voltage,
			&adc_chan_result->physical);

error:
	return result;
}
#endif /* CONFIG_BATTERY_SH */

int32_t qpnp_adc_scale_pmic_therm(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result
		|| !chan_properties->adc_graph[CALIB_ABSOLUTE].dy)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
#ifndef CONFIG_BATTERY_SH
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;
#else  /* CONFIG_BATTERY_SH */
	/* Change to .001 deg C */
	adc_chan_result->physical = adc_chan_result->measurement;
	adc_chan_result->physical -= KELVINMIL_DEGMIL;
#endif /* CONFIG_BATTERY_SH */

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_pmic_therm);

int32_t qpnp_adc_scale_millidegc_pmic_voltage_thr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0, sign = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_ABSOLUTE);
	if (rc < 0) {
		pr_err("Could not acquire gain and offset\n");
		return rc;
	}

	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	low_output = (param->low_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	low_output = (low_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (low_output < 0) {
		sign = 1;
		low_output = -low_output;
	}
	do_div(low_output, QPNP_ADC_625_UV);
	if (sign)
		low_output = -low_output;
	low_output += btm_param.adc_gnd;

	sign = 0;
	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	high_output = (param->high_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	high_output = (high_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (high_output < 0) {
		sign = 1;
		high_output = -high_output;
	}
	do_div(high_output, QPNP_ADC_625_UV);
	if (sign)
		high_output = -high_output;
	high_output += btm_param.adc_gnd;

	*low_threshold = (uint32_t) low_output;
	*high_threshold = (uint32_t) high_output;
	pr_debug("high_temp:%d, low_temp:%d\n", param->high_temp,
				param->low_temp);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_millidegc_pmic_voltage_thr);

/* Scales the ADC code to degC using the mapping
 * table for the XO thermistor.
 */
int32_t qpnp_adc_tdkntcg_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

#ifdef CONFIG_BATTERY_SH
	adc_chan_result->measurement = xo_thm;
	adc_chan_result->microvolts = (int32_t)adc_chan_result->measurement * 1000;
#endif /* CONFIG_BATTERY_SH */

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tdkntcg_therm);

int32_t qpnp_adc_scale_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
#ifndef CONFIG_BATTERY_SH
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
#else  /* CONFIG_BATTERY_SH */
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_RATIOMETRIC,
			&qpnp_adc_map_select_bat_therm);
#endif /* CONFIG_BATTERY_SH */
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_therm);

int32_t qpnp_adc_scale_qrd_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_batt_therm);

int32_t qpnp_adc_scale_qrd_skuaa_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuaa_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuaa_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuaa_batt_therm);

int32_t qpnp_adc_scale_qrd_skug_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skug_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skug_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skug_batt_therm);
int32_t qpnp_adc_scale_therm_pu1(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

#ifdef CONFIG_BATTERY_SH
	adc_chan_result->measurement = therm_voltage;
	adc_chan_result->microvolts = (int32_t)adc_chan_result->measurement * 1000;
#endif /* CONFIG_BATTERY_SH */

	qpnp_adc_map_voltage_temp(adcmap_150k_104ef_104fb,
		ARRAY_SIZE(adcmap_150k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu1);

int32_t qpnp_adc_scale_therm_pu2(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

#ifdef CONFIG_BATTERY_SH
	adc_chan_result->measurement = therm_voltage;
	adc_chan_result->microvolts = (int32_t)adc_chan_result->measurement * 1000;
#endif /* CONFIG_BATTERY_SH */

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu2);

int32_t qpnp_adc_tm_scale_voltage_therm_pu2(struct qpnp_vadc_chip *chip,
					uint32_t reg, int64_t *result)
{
	int64_t adc_voltage = 0;
	struct qpnp_vadc_linear_graph param1;
	int negative_offset;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	adc_voltage = (reg - param1.adc_gnd) * param1.adc_vref;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}

	do_div(adc_voltage, param1.dy);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		adc_voltage, result);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_voltage_therm_pu2);

int32_t qpnp_adc_tm_scale_therm_voltage_pu2(struct qpnp_vadc_chip *chip,
				struct qpnp_adc_tm_config *param)
{
	struct qpnp_vadc_linear_graph param1;
	int rc;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->low_thr_temp, &param->low_thr_voltage);
	if (rc)
		return rc;

	param->low_thr_voltage *= param1.dy;
	do_div(param->low_thr_voltage, param1.adc_vref);
	param->low_thr_voltage += param1.adc_gnd;

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->high_thr_temp, &param->high_thr_voltage);
	if (rc)
		return rc;

	param->high_thr_voltage *= param1.dy;
	do_div(param->high_thr_voltage, param1.adc_vref);
	param->high_thr_voltage += param1.adc_gnd;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_therm_voltage_pu2);

int32_t qpnp_adc_scale_batt_id(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_id);

int32_t qpnp_adc_scale_default(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[chan_properties->calib_type].adc_gnd)
		* chan_properties->adc_graph[chan_properties->calib_type].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[chan_properties->calib_type].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;

	if (chan_properties->calib_type == CALIB_ABSOLUTE)
		scale_voltage +=
		chan_properties->adc_graph[chan_properties->calib_type].dx;
	else
		scale_voltage *= 1000;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/*
	 * Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input
	 */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_default);

#ifdef CONFIG_BATTERY_SH
int32_t qpnp_adc_scale_xo_therm(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_RATIOMETRIC,
			&qpnp_adc_map_select_xo_therm);
}
EXPORT_SYMBOL(qpnp_adc_scale_xo_therm);

int32_t qpnp_adc_scale_pa_therm(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_ABSOLUTE,
			&qpnp_adc_map_select_pa_therm);
}
EXPORT_SYMBOL(qpnp_adc_scale_pa_therm);

int32_t qpnp_adc_scale_cam_therm(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_ABSOLUTE,
			&qpnp_adc_map_select_cam_therm);
}
EXPORT_SYMBOL(qpnp_adc_scale_cam_therm);

int32_t qpnp_adc_scale_lcd_therm(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
#ifdef CONFIG_PM_LCD_THERM
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_ABSOLUTE,
			&qpnp_adc_map_select_lcd_therm);
#else  /* CONFIG_PM_LCD_THERM */
	return -EPERM;
#endif /* CONFIG_PM_LCD_THERM */
}
EXPORT_SYMBOL(qpnp_adc_scale_lcd_therm);

int32_t qpnp_adc_scale_msm_therm(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
#ifdef CONFIG_PM_MSM_THERM
	return qpnp_adc_scale_voltage_therm_by_table(
			adc_code,
			adc_properties,
			chan_properties,
			adc_chan_result,
			CALIB_ABSOLUTE,
			&qpnp_adc_map_select_msm_therm);
#else  /* CONFIG_PM_MSM_THERM */
	return -EPERM;
#endif /* CONFIG_PM_MSM_THERM */
}
EXPORT_SYMBOL(qpnp_adc_scale_msm_therm);

/* for calibration */
static int debug_calib_adc = 0;
module_param_named(debug_calib_adc, debug_calib_adc, int, S_IRUSR | S_IWUSR );

#define SH_CALIB_VBATT_ADJUST (-1)

static int sh_calib_vbatt_amin = 0;
static int sh_calib_vbatt_amax = 0;
static int sh_calib_vbatt_vmin = 0;
static int sh_calib_vbatt_vmax = 0;
static bool sh_calib_vbatt_calflg = false;
static bool sh_calib_vbatt_adcflg = false;

static void qpnp_adc_scale_vbatt_calib_type_0(int32_t adc_code,
		struct qpnp_vadc_result *adc_chan_result)
{
	static int scale_x_10000 = 0;
	static int64_t offset = 0;
	bool negative_flag = false;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("calib: type(0) calflg = %d adcflg = %d\n", sh_calib_vbatt_calflg, sh_calib_vbatt_adcflg);
	}

	if (sh_calib_vbatt_calflg == false)
	{
		if (sh_calib_vbatt_amin != sh_calib_vbatt_amax)
		{
			scale_x_10000 = (sh_calib_vbatt_vmax - sh_calib_vbatt_vmin) * 10000;
			scale_x_10000 /= (sh_calib_vbatt_amax - sh_calib_vbatt_amin);
			offset = ((int64_t)sh_calib_vbatt_vmin * (int64_t)sh_calib_vbatt_amax) - ((int64_t)sh_calib_vbatt_vmax * (int64_t)sh_calib_vbatt_amin);
			offset *= 1000;
			if (offset < 0)
			{
				negative_flag = true;
				offset = -offset;
			}
			do_div(offset, sh_calib_vbatt_amax - sh_calib_vbatt_amin);
			if (negative_flag == true)
			{
				offset = -offset;
			}

			if ((debug_calib_adc & 0x02) != 0)
			{
				pr_info("calib: scale = %d offset = %lld\n", scale_x_10000, offset);
			}

			sh_calib_vbatt_calflg = true;
		}
	}

	if (sh_calib_vbatt_calflg == true)
	{
		adc_chan_result->physical = ((uint32_t)(adc_code * scale_x_10000)) / 10 + offset;
	}

	if ((debug_calib_adc & 0x08) != 0)
	{
		pr_info("calib: voltage = %lld\n", adc_chan_result->physical);
	}
}

static void qpnp_adc_scale_vbatt_calib_type_1(int32_t adc_code,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	static int calc_gain = 0;
	static int calc_offs = 0;
	int calc_data;
	int64_t calc_temp;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("calib: type(1) calflg = %d adcflg = %d\n", sh_calib_vbatt_calflg, sh_calib_vbatt_adcflg);
	}

	if (sh_calib_vbatt_calflg == false)
	{
		calc_data = chan_properties->adc_graph[CALIB_ABSOLUTE].dy * (1 << 14);
		if ((debug_calib_adc & 0x04) != 0)
		{
			pr_info("calib: calc_data = %d\n", calc_data);
		}
		calc_gain = calc_data / 625;
		calc_offs = chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd * (1 << 14);
		calc_offs -= calc_data;

		if ((debug_calib_adc & 0x02) != 0)
		{
			pr_info("calib: gain = %d offset = %d\n", calc_gain, calc_offs);
		}

		if ((calc_gain != 0) &&
			(sh_calib_vbatt_amin != sh_calib_vbatt_amax) &&
			(sh_calib_vbatt_vmin != sh_calib_vbatt_vmax))
		{
			sh_calib_vbatt_calflg = true;
		}
		if (calc_gain == 0)
		{
			sh_calib_vbatt_adcflg = false;
		}
	}

	if ((debug_calib_adc & 0x04) != 0)
	{
		pr_info("calib: read_uv = %lld\n", adc_chan_result->physical);
	}

	if (sh_calib_vbatt_calflg == false)
	{
		if (sh_calib_vbatt_adcflg == true)
		{
			calc_data = ((adc_code * (1 << 14) - calc_offs) * (1 << 3)) / calc_gain;
			adc_chan_result->adc_code = calc_data;
		}
	}
	else
	{
		if ((calc_gain != 0) &&
			(sh_calib_vbatt_amin != sh_calib_vbatt_amax))
		{
			calc_data = ((adc_code * (1 << 14) - calc_offs) * (1 << 3)) / calc_gain;
			calc_temp = (sh_calib_vbatt_vmax - sh_calib_vbatt_vmin) * (calc_data - sh_calib_vbatt_amax) * 1000LL;
			if ((debug_calib_adc & 0x04) != 0)
			{
				pr_info("calib: calc_uv = %d\n", calc_data);
				pr_info("calib: calc_uv = %lld\n", calc_temp);
			}
			calc_data = (int)div_s64(calc_temp, sh_calib_vbatt_amax - sh_calib_vbatt_amin);
			if ((debug_calib_adc & 0x04) != 0)
			{
				pr_info("calib: calc_uv = %d\n", calc_data);
			}
			calc_data += sh_calib_vbatt_vmax * 1000;
			adc_chan_result->physical = calc_data;
		}
	}

	if ((debug_calib_adc & 0x08) != 0)
	{
		pr_info("calib: voltage = %lld\n", adc_chan_result->physical);
	}
}

int32_t qpnp_adc_scale_vbatt(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{

	if ((debug_calib_adc & 0x02) != 0)
	{
		pr_info("adc scale: code[%5d] meas[%9lld] phys[%9lld] volt[%9d]\n",
			adc_code, adc_chan_result->measurement, adc_chan_result->physical,
			adc_chan_result->microvolts);
	}

	if (((sh_calib_vbatt_vmax == SH_CALIB_VBATT_ADJUST) &&
		 (sh_calib_vbatt_vmin == SH_CALIB_VBATT_ADJUST) &&
		 (sh_calib_vbatt_amax == SH_CALIB_VBATT_ADJUST) &&
		 (sh_calib_vbatt_amin == SH_CALIB_VBATT_ADJUST)) ||
		((sh_calib_vbatt_vmax >= 3700) && (sh_calib_vbatt_vmax <= 4300) &&
		 (sh_calib_vbatt_vmin >= 3000) && (sh_calib_vbatt_vmin <= 3600) &&
		 (sh_calib_vbatt_amax < 20000) && (sh_calib_vbatt_amin < 20000)))
	{
		qpnp_adc_scale_vbatt_calib_type_1(adc_code, chan_properties, adc_chan_result);
	}
	else
	{
		qpnp_adc_scale_vbatt_calib_type_0(adc_code, adc_chan_result);
	}

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_vbatt);

void qpnp_adc_set_vbatt_calibration_data(int amin, int amax, int vmin, int vmax)
{
	sh_calib_vbatt_amin = amin;
	sh_calib_vbatt_amax = amax;
	sh_calib_vbatt_vmin = vmin;
	sh_calib_vbatt_vmax = vmax;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("set calib: vmin[%5d] vmax[%5d] amin[%5d] amax[%5d]\n",
			sh_calib_vbatt_vmin, sh_calib_vbatt_vmax,
			sh_calib_vbatt_amin, sh_calib_vbatt_amax);
	}
}
EXPORT_SYMBOL(qpnp_adc_set_vbatt_calibration_data);

void qpnp_adc_refresh_vbatt_calibration_data(void)
{
	sh_calib_vbatt_calflg = false;
	sh_calib_vbatt_adcflg = false;
	if ((sh_calib_vbatt_vmax == SH_CALIB_VBATT_ADJUST) &&
		(sh_calib_vbatt_vmin == SH_CALIB_VBATT_ADJUST) &&
		(sh_calib_vbatt_amax == SH_CALIB_VBATT_ADJUST) &&
		(sh_calib_vbatt_amin == SH_CALIB_VBATT_ADJUST))
	{
		sh_calib_vbatt_adcflg = true;
	}

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("reflesh calib: calflg = %d adcflg = %d\n", sh_calib_vbatt_calflg, sh_calib_vbatt_adcflg);
	}
}
EXPORT_SYMBOL(qpnp_adc_refresh_vbatt_calibration_data);

static int sh_calib_vsense_avg_imin = 0;
static int sh_calib_vsense_avg_imax = 0;
static int sh_calib_vsense_avg_amin = 0;
static int sh_calib_vsense_avg_amax = 0;
static bool sh_calib_vsense_avg_calflg = false;

s64 qpnp_adc_scale_uv_to_ma(s64 uv, int r_sense_uohm)
{
	static int calc_nume = 0;
	static int calc_deno = 0;
	static int calc_base = 0;
	static int calc_offs = 0;
	s64 result_mA;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("calib type(i) calflg = %d\n", sh_calib_vsense_avg_calflg);
	}

	if (sh_calib_vsense_avg_calflg == false)
	{
		if ((sh_calib_vsense_avg_imin != sh_calib_vsense_avg_imax) &&
			(sh_calib_vsense_avg_amin != sh_calib_vsense_avg_amax))
		{
			calc_nume = sh_calib_vsense_avg_imax - sh_calib_vsense_avg_imin;
			calc_deno = sh_calib_vsense_avg_amax - sh_calib_vsense_avg_amin;
			calc_base = sh_calib_vsense_avg_amin;
			calc_offs = sh_calib_vsense_avg_imin;

			if ((debug_calib_adc & 0x02) != 0)
			{
				pr_info("calib: nume = %d deno = %d base = %d offset = %d\n",
					calc_nume, calc_deno, calc_base, calc_offs);
			}

			sh_calib_vsense_avg_calflg = true;
		}
	}

	if ((debug_calib_adc & 0x04) != 0)
	{
		pr_info("calib: read_uv = %lld\n", uv);
	}

	if (sh_calib_vsense_avg_calflg == true)
	{
		result_mA = uv - calc_base;
		result_mA *= calc_nume;
		result_mA = div_s64(result_mA, calc_deno);
		result_mA += calc_offs;
	}
	else
	{
		if (r_sense_uohm == 0)
		{
			pr_err("calib: r_sense_uohm is zero\n");
			return -EINVAL;
		}
		result_mA = div_s64(uv * 1000LL, r_sense_uohm);
	}

	if ((debug_calib_adc & 0x08) != 0)
	{
		pr_info("calib: current = %lld\n", result_mA);
	}

	return result_mA;
}
EXPORT_SYMBOL(qpnp_adc_scale_uv_to_ma);

void qpnp_adc_set_vsense_avg_calibration_data(int amin, int amax, int imin, int imax)
{
	sh_calib_vsense_avg_imin = imin;
	sh_calib_vsense_avg_imax = imax;
	sh_calib_vsense_avg_amin = amin;
	sh_calib_vsense_avg_amax = amax;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("set calib: imin[%5d] imax[%5d] amin[%5d] amax[%5d]\n",
			sh_calib_vsense_avg_imin, sh_calib_vsense_avg_imax,
			sh_calib_vsense_avg_amin, sh_calib_vsense_avg_amax);
	}
}
EXPORT_SYMBOL(qpnp_adc_set_vsense_avg_calibration_data);

void qpnp_adc_refresh_vsense_avg_calibration_data(void)
{
	sh_calib_vsense_avg_calflg = false;

	if ((debug_calib_adc & 0x01) != 0)
	{
		pr_info("reflesh calib: calflg = %d\n", sh_calib_vsense_avg_calflg);
	}
}
EXPORT_SYMBOL(qpnp_adc_refresh_vsense_avg_calibration_data);
#endif /* CONFIG_BATTERY_SH */

int32_t qpnp_adc_usb_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph usb_param;

	qpnp_get_vadc_gain_and_offset(chip, &usb_param, CALIB_RATIOMETRIC);

	*low_threshold = param->low_thr * usb_param.dy;
	do_div(*low_threshold, usb_param.adc_vref);
	*low_threshold += usb_param.adc_gnd;

	*high_threshold = param->high_thr * usb_param.dy;
	do_div(*high_threshold, usb_param.adc_vref);
	*high_threshold += usb_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_usb_scaler);

int32_t qpnp_adc_vbatt_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_vbatt_rscaler);

int32_t qpnp_adc_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_btm_scaler);

int32_t qpnp_vadc_check_result(int32_t *data)
{
	if (*data < QPNP_VADC_MIN_ADC_CODE)
		*data = QPNP_VADC_MIN_ADC_CODE;
	else if (*data > QPNP_VADC_MAX_ADC_CODE)
		*data = QPNP_VADC_MAX_ADC_CODE;

	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_check_result);

int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_debug("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
	else if ((revid_data->rev1 == PM8941_V3P0_REV1) &&
		(revid_data->rev2 == PM8941_V3P0_REV2) &&
		(revid_data->rev3 == PM8941_V3P0_REV3) &&
		(revid_data->rev4 == PM8941_V3P0_REV4) &&
		(revid_data->pmic_type == PM8941_V3P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P0_SUBTYPE))
			return QPNP_REV_ID_8941_3_0;
	else if ((revid_data->rev1 == PM8941_V2P0_REV1) &&
		(revid_data->rev2 == PM8941_V2P0_REV2) &&
		(revid_data->rev3 == PM8941_V2P0_REV3) &&
		(revid_data->rev4 == PM8941_V2P0_REV4) &&
		(revid_data->pmic_type == PM8941_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V2P0_SUBTYPE))
			return QPNP_REV_ID_8941_2_0;
	else if ((revid_data->rev1 == PM8226_V2P2_REV1) &&
		(revid_data->rev2 == PM8226_V2P2_REV2) &&
		(revid_data->rev3 == PM8226_V2P2_REV3) &&
		(revid_data->rev4 == PM8226_V2P2_REV4) &&
		(revid_data->pmic_type == PM8226_V2P2_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P2_SUBTYPE))
			return QPNP_REV_ID_8026_2_2;
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
	else if ((revid_data->rev1 == PM8110_V2P0_REV1) &&
		(revid_data->rev2 == PM8110_V2P0_REV2) &&
		(revid_data->rev3 == PM8110_V2P0_REV3) &&
		(revid_data->rev4 == PM8110_V2P0_REV4) &&
		(revid_data->pmic_type == PM8110_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V2P0_SUBTYPE))
			return QPNP_REV_ID_8110_2_0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);

int32_t qpnp_adc_get_devicetree_data(struct spmi_device *spmi,
			struct qpnp_adc_drv *adc_qpnp)
{
	struct device_node *node = spmi->dev.of_node;
	struct resource *res;
	struct device_node *child;
	struct qpnp_adc_amux *adc_channel_list;
	struct qpnp_adc_properties *adc_prop;
	struct qpnp_adc_amux_properties *amux_prop;
	int count_adc_channel_list = 0, decimation, rc = 0, i = 0;

	if (!node)
		return -EINVAL;

	for_each_child_of_node(node, child)
		count_adc_channel_list++;

	if (!count_adc_channel_list) {
		pr_err("No channel listing\n");
		return -EINVAL;
	}

	adc_qpnp->spmi = spmi;

	adc_prop = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_adc_properties),
					GFP_KERNEL);
	if (!adc_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	adc_channel_list = devm_kzalloc(&spmi->dev,
		((sizeof(struct qpnp_adc_amux)) * count_adc_channel_list),
				GFP_KERNEL);
	if (!adc_channel_list) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	amux_prop = devm_kzalloc(&spmi->dev,
		sizeof(struct qpnp_adc_amux_properties) +
		sizeof(struct qpnp_vadc_chan_properties), GFP_KERNEL);
	if (!amux_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_qpnp->adc_channels = adc_channel_list;
	adc_qpnp->amux_prop = amux_prop;

	for_each_child_of_node(node, child) {
		int channel_num, scaling, post_scaling, hw_settle_time;
		int fast_avg_setup, calib_type = 0, rc;
		const char *calibration_param, *channel_name;
#ifdef CONFIG_BATTERY_SH
		int sh_post_scaling;
#endif /* CONFIG_BATTERY_SH */

		channel_name = of_get_property(child,
				"label", NULL) ? : child->name;
		if (!channel_name) {
			pr_err("Invalid channel name\n");
			return -EINVAL;
		}

		rc = of_property_read_u32(child, "reg", &channel_num);
		if (rc) {
			pr_err("Invalid channel num\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child, "qcom,decimation",
								&decimation);
		if (rc) {
			pr_err("Invalid channel decimation property\n");
			return -EINVAL;
		}
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			rc = of_property_read_u32(child,
				"qcom,hw-settle-time", &hw_settle_time);
			if (rc) {
				pr_err("Invalid channel hw settle time property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,pre-div-channel-scaling", &scaling);
			if (rc) {
				pr_err("Invalid channel scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,scale-function", &post_scaling);
			if (rc) {
				pr_err("Invalid channel post scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_string(child,
				"qcom,calibration-type", &calibration_param);
			if (rc) {
				pr_err("Invalid calibration type\n");
				return -EINVAL;
			}
			if (!strcmp(calibration_param, "absolute"))
				calib_type = CALIB_ABSOLUTE;
			else if (!strcmp(calibration_param, "ratiometric"))
				calib_type = CALIB_RATIOMETRIC;
			else {
				pr_err("%s: Invalid calibration property\n",
						__func__);
				return -EINVAL;
			}
		}
		rc = of_property_read_u32(child,
				"qcom,fast-avg-setup", &fast_avg_setup);
		if (rc) {
			pr_err("Invalid channel fast average setup\n");
			return -EINVAL;
		}
		rc = of_property_read_string(child, "qcom,calibration-type",
							&calibration_param);
		if (rc) {
			pr_err("Invalid calibration type\n");
			return -EINVAL;
		}
		if (!strncmp(calibration_param, "absolute", 8))
			calib_type = CALIB_ABSOLUTE;
		else if (!strncmp(calibration_param, "ratiometric", 11))
			calib_type = CALIB_RATIOMETRIC;
		else {
			pr_err("%s: Invalid calibration property\n", __func__);
			return -EINVAL;
		}
		/* Individual channel properties */
		adc_channel_list[i].name = (char *)channel_name;
		adc_channel_list[i].channel_num = channel_num;
		adc_channel_list[i].chan_path_prescaling = scaling;
		adc_channel_list[i].adc_decimation = decimation;
		adc_channel_list[i].adc_scale_fn = post_scaling;
		adc_channel_list[i].hw_settle_time = hw_settle_time;
		adc_channel_list[i].fast_avg_setup = fast_avg_setup;
#ifdef CONFIG_BATTERY_SH
		rc = of_property_read_u32(child,
				"sharp,scale-function", &sh_post_scaling);
		if (rc) {
			sh_post_scaling = SH_SCALE_DEFAULT;
		} else {
			pr_debug("channel@%02x sh-post-scaling = %d\n",
				channel_num, sh_post_scaling);
		}
		adc_channel_list[i].sh_adc_scale_fn = sh_post_scaling;
#endif /* CONFIG_BATTERY_SH */
		i++;
	}

	/* Get the ADC VDD reference voltage and ADC bit resolution */
	rc = of_property_read_u32(node, "qcom,adc-vdd-reference",
			&adc_prop->adc_vdd_reference);
	if (rc) {
		pr_err("Invalid adc vdd reference property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,adc-bit-resolution",
			&adc_prop->bitresolution);
	if (rc) {
		pr_err("Invalid adc bit resolution property\n");
		return -EINVAL;
	}
	adc_qpnp->adc_prop = adc_prop;

	/* Get the peripheral address */
	res = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("No base address definition\n");
		return -EINVAL;
	}

	adc_qpnp->slave = spmi->sid;
	adc_qpnp->offset = res->start;

	/* Register the ADC peripheral interrupt */
	adc_qpnp->adc_irq_eoc = spmi_get_irq_byname(spmi, NULL,
						"eoc-int-en-set");
	if (adc_qpnp->adc_irq_eoc < 0) {
		pr_err("Invalid irq\n");
		return -ENXIO;
	}

	init_completion(&adc_qpnp->adc_rslt_completion);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_get_devicetree_data);
