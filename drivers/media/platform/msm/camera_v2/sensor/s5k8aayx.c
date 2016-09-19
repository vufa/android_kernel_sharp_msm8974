/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>

#define S5K8AAYX_SENSOR_NAME "s5k8aayx"
DEFINE_MSM_MUTEX(s5k8aayx_mut);

/* #define CONFIG_MSMB_CAMERA_DEBUG */
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_sensor_ctrl_t s5k8aayx_s_ctrl;

static struct perf_lock s5k8aayx_perf_lock;

static struct msm_sensor_power_setting s5k8aayx_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 17000000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_cam_clk_info s5k8aayx_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_gp1_src_clk", 17000000},
	[SENSOR_CAM_CLK] = {"cam_gp1_clk", 0},
};

static struct msm_camera_i2c_reg_conf init_reg_array[] = {
//==================================================================================
// 01.Start Setting
//==================================================================================
	{0xFCFC, 0xD000},
	{0x0010, 0x0001},	//S/W Reset
	{0xFCFC, 0x0000},
	{0x0000, 0x0000},	//Simmian bug workaround
	{0xFCFC, 0xD000},
	{0x1030, 0x0000},	//contint_host_int
	{0x0014, 0x0001},	//sw_load_complete - Release CORE (Arm) from reset state

//==================================================================================
// 02.ETC Setting
//==================================================================================

//==================================================================================
// 03.Trap and Patch
//==================================================================================
//Start of Patch data
	{0x0028, 0x7000},
	{0x002A, 0x2470},
	{0x0F12, 0xB510},
	{0x0F12, 0x490E},
	{0x0F12, 0x480E},
	{0x0F12, 0xF000},
	{0x0F12, 0xF9ED},
	{0x0F12, 0x490E},
	{0x0F12, 0x480E},
	{0x0F12, 0xF000},
	{0x0F12, 0xF9E9},
	{0x0F12, 0x490E},
	{0x0F12, 0x480E},
	{0x0F12, 0x6341},
	{0x0F12, 0x490E},
	{0x0F12, 0x480F},
	{0x0F12, 0xF000},
	{0x0F12, 0xF9E2},
	{0x0F12, 0x490E},
	{0x0F12, 0x480F},
	{0x0F12, 0xF000},
	{0x0F12, 0xF9DE},
	{0x0F12, 0x490E},
	{0x0F12, 0x480F},
	{0x0F12, 0xF000},
	{0x0F12, 0xF9DA},
	{0x0F12, 0x480E},
	{0x0F12, 0x490F},
	{0x0F12, 0x6448},
	{0x0F12, 0xBC10},
	{0x0F12, 0xBC08},
	{0x0F12, 0x4718},
	{0x0F12, 0x27CC},
	{0x0F12, 0x7000},
	{0x0F12, 0x8EDD},
	{0x0F12, 0x0000},
	{0x0F12, 0x2744},
	{0x0F12, 0x7000},
	{0x0F12, 0x8725},
	{0x0F12, 0x0000},
	{0x0F12, 0x26E4},
	{0x0F12, 0x7000},
	{0x0F12, 0x0080},
	{0x0F12, 0x7000},
	{0x0F12, 0x2638},
	{0x0F12, 0x7000},
	{0x0F12, 0xA6EF},
	{0x0F12, 0x0000},
	{0x0F12, 0x2604},
	{0x0F12, 0x7000},
	{0x0F12, 0xA0F1},
	{0x0F12, 0x0000},
	{0x0F12, 0x25D0},
	{0x0F12, 0x7000},
	{0x0F12, 0x058F},
	{0x0F12, 0x0000},
	{0x0F12, 0x24E4},
	{0x0F12, 0x7000},
	{0x0F12, 0x0000},
	{0x0F12, 0x7000},
	{0x0F12, 0x403E},
	{0x0F12, 0xE92D},
	{0x0F12, 0x00DD},
	{0x0F12, 0xEB00},
	{0x0F12, 0x2000},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x1002},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0F86},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x00DC},
	{0x0F12, 0xEB00},
	{0x0F12, 0x200A},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x1000},
	{0x0F12, 0xE28D},
	{0x0F12, 0x0E3F},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x00DB},
	{0x0F12, 0xEB00},
	{0x0F12, 0x2001},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x1002},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0F86},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x00D4},
	{0x0F12, 0xEB00},
	{0x0F12, 0x0000},
	{0x0F12, 0xE5DD},
	{0x0F12, 0x00C3},
	{0x0F12, 0xE350},
	{0x0F12, 0x0027},
	{0x0F12, 0x1A00},
	{0x0F12, 0x0001},
	{0x0F12, 0xE5DD},
	{0x0F12, 0x003C},
	{0x0F12, 0xE350},
	{0x0F12, 0x0024},
	{0x0F12, 0x1A00},
	{0x0F12, 0x02E0},
	{0x0F12, 0xE59F},
	{0x0F12, 0x1000},
	{0x0F12, 0xE5D0},
	{0x0F12, 0x0000},
	{0x0F12, 0xE351},
	{0x0F12, 0x0003},
	{0x0F12, 0x1A00},
	{0x0F12, 0x12D4},
	{0x0F12, 0xE59F},
	{0x0F12, 0x10B8},
	{0x0F12, 0xE1D1},
	{0x0F12, 0x0000},
	{0x0F12, 0xE351},
	{0x0F12, 0x001C},
	{0x0F12, 0x0A00},
	{0x0F12, 0x1000},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x1000},
	{0x0F12, 0xE5C0},
	{0x0F12, 0x0000},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x1002},
	{0x0F12, 0xE28D},
	{0x0F12, 0x0015},
	{0x0F12, 0xEA00},
	{0x0F12, 0x2000},
	{0x0F12, 0xE5D1},
	{0x0F12, 0x3001},
	{0x0F12, 0xE5D1},
	{0x0F12, 0x3403},
	{0x0F12, 0xE182},
	{0x0F12, 0xC2A8},
	{0x0F12, 0xE59F},
	{0x0F12, 0x2080},
	{0x0F12, 0xE08C},
	{0x0F12, 0xE7B4},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x039E},
	{0x0F12, 0xE004},
	{0x0F12, 0xE80F},
	{0x0F12, 0xE3E0},
	{0x0F12, 0x4624},
	{0x0F12, 0xE00E},
	{0x0F12, 0x47B4},
	{0x0F12, 0xE1C2},
	{0x0F12, 0x4004},
	{0x0F12, 0xE280},
	{0x0F12, 0xC084},
	{0x0F12, 0xE08C},
	{0x0F12, 0x47B4},
	{0x0F12, 0xE1DC},
	{0x0F12, 0x0493},
	{0x0F12, 0xE004},
	{0x0F12, 0x4624},
	{0x0F12, 0xE00E},
	{0x0F12, 0x47B4},
	{0x0F12, 0xE1CC},
	{0x0F12, 0xC8B4},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x039C},
	{0x0F12, 0xE003},
	{0x0F12, 0x3623},
	{0x0F12, 0xE00E},
	{0x0F12, 0x38B4},
	{0x0F12, 0xE1C2},
	{0x0F12, 0x0001},
	{0x0F12, 0xE280},
	{0x0F12, 0x1002},
	{0x0F12, 0xE281},
	{0x0F12, 0x0004},
	{0x0F12, 0xE350},
	{0x0F12, 0xFFE7},
	{0x0F12, 0xBAFF},
	{0x0F12, 0x403E},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x4010},
	{0x0F12, 0xE92D},
	{0x0F12, 0x00AB},
	{0x0F12, 0xEB00},
	{0x0F12, 0x0248},
	{0x0F12, 0xE59F},
	{0x0F12, 0x00B2},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x0000},
	{0x0F12, 0xE350},
	{0x0F12, 0x0004},
	{0x0F12, 0x0A00},
	{0x0F12, 0x0080},
	{0x0F12, 0xE310},
	{0x0F12, 0x0002},
	{0x0F12, 0x1A00},
	{0x0F12, 0x1234},
	{0x0F12, 0xE59F},
	{0x0F12, 0x0001},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0DB2},
	{0x0F12, 0xE1C1},
	{0x0F12, 0x4010},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x4010},
	{0x0F12, 0xE92D},
	{0x0F12, 0x4000},
	{0x0F12, 0xE590},
	{0x0F12, 0x0004},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x009F},
	{0x0F12, 0xEB00},
	{0x0F12, 0x0214},
	{0x0F12, 0xE59F},
	{0x0F12, 0x0000},
	{0x0F12, 0xE5D0},
	{0x0F12, 0x0000},
	{0x0F12, 0xE350},
	{0x0F12, 0x0002},
	{0x0F12, 0x0A00},
	{0x0F12, 0x0004},
	{0x0F12, 0xE594},
	{0x0F12, 0x00A0},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x0004},
	{0x0F12, 0xE584},
	{0x0F12, 0x4010},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x4070},
	{0x0F12, 0xE92D},
	{0x0F12, 0x0000},
	{0x0F12, 0xE590},
	{0x0F12, 0x0800},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x0820},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x4041},
	{0x0F12, 0xE280},
	{0x0F12, 0x01E0},
	{0x0F12, 0xE59F},
	{0x0F12, 0x11B8},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x51B6},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x0005},
	{0x0F12, 0xE041},
	{0x0F12, 0x0094},
	{0x0F12, 0xE000},
	{0x0F12, 0x1D11},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x008D},
	{0x0F12, 0xEB00},
	{0x0F12, 0x11C0},
	{0x0F12, 0xE59F},
	{0x0F12, 0x1000},
	{0x0F12, 0xE5D1},
	{0x0F12, 0x0000},
	{0x0F12, 0xE351},
	{0x0F12, 0x0000},
	{0x0F12, 0x0A00},
	{0x0F12, 0x00A0},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x21A8},
	{0x0F12, 0xE59F},
	{0x0F12, 0x3FB0},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x0000},
	{0x0F12, 0xE353},
	{0x0F12, 0x0003},
	{0x0F12, 0x0A00},
	{0x0F12, 0x31A4},
	{0x0F12, 0xE59F},
	{0x0F12, 0x5BB2},
	{0x0F12, 0xE1C3},
	{0x0F12, 0xC000},
	{0x0F12, 0xE085},
	{0x0F12, 0xCBB4},
	{0x0F12, 0xE1C3},
	{0x0F12, 0x0000},
	{0x0F12, 0xE351},
	{0x0F12, 0x0000},
	{0x0F12, 0x0A00},
	{0x0F12, 0x0080},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x1DBC},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x3EB4},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x2EB2},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x0193},
	{0x0F12, 0xE001},
	{0x0F12, 0x0092},
	{0x0F12, 0xE000},
	{0x0F12, 0x2811},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0194},
	{0x0F12, 0xE001},
	{0x0F12, 0x0092},
	{0x0F12, 0xE000},
	{0x0F12, 0x11A1},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x01A0},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x0072},
	{0x0F12, 0xEB00},
	{0x0F12, 0x1160},
	{0x0F12, 0xE59F},
	{0x0F12, 0x02B4},
	{0x0F12, 0xE1C1},
	{0x0F12, 0x4070},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x4010},
	{0x0F12, 0xE92D},
	{0x0F12, 0x006E},
	{0x0F12, 0xEB00},
	{0x0F12, 0x2148},
	{0x0F12, 0xE59F},
	{0x0F12, 0x14B0},
	{0x0F12, 0xE1D2},
	{0x0F12, 0x0080},
	{0x0F12, 0xE311},
	{0x0F12, 0x0005},
	{0x0F12, 0x0A00},
	{0x0F12, 0x013C},
	{0x0F12, 0xE59F},
	{0x0F12, 0x00B0},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x0001},
	{0x0F12, 0xE350},
	{0x0F12, 0x0001},
	{0x0F12, 0x9A00},
	{0x0F12, 0x0001},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0000},
	{0x0F12, 0xEA00},
	{0x0F12, 0x0000},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x3110},
	{0x0F12, 0xE59F},
	{0x0F12, 0x0000},
	{0x0F12, 0xE5C3},
	{0x0F12, 0x0000},
	{0x0F12, 0xE5D3},
	{0x0F12, 0x0000},
	{0x0F12, 0xE350},
	{0x0F12, 0x0003},
	{0x0F12, 0x0A00},
	{0x0F12, 0x0080},
	{0x0F12, 0xE3C1},
	{0x0F12, 0x110C},
	{0x0F12, 0xE59F},
	{0x0F12, 0x04B0},
	{0x0F12, 0xE1C2},
	{0x0F12, 0x00B2},
	{0x0F12, 0xE1C1},
	{0x0F12, 0x4010},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x41F0},
	{0x0F12, 0xE92D},
	{0x0F12, 0x1000},
	{0x0F12, 0xE590},
	{0x0F12, 0xC801},
	{0x0F12, 0xE1A0},
	{0x0F12, 0xC82C},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x1004},
	{0x0F12, 0xE590},
	{0x0F12, 0x1801},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x1821},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x4008},
	{0x0F12, 0xE590},
	{0x0F12, 0x500C},
	{0x0F12, 0xE590},
	{0x0F12, 0x2004},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x3005},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x000C},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x004E},
	{0x0F12, 0xEB00},
	{0x0F12, 0x60A0},
	{0x0F12, 0xE59F},
	{0x0F12, 0x00B2},
	{0x0F12, 0xE1D6},
	{0x0F12, 0x0000},
	{0x0F12, 0xE350},
	{0x0F12, 0x000E},
	{0x0F12, 0x0A00},
	{0x0F12, 0x00B8},
	{0x0F12, 0xE59F},
	{0x0F12, 0x05B4},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x0002},
	{0x0F12, 0xE350},
	{0x0F12, 0x000A},
	{0x0F12, 0x1A00},
	{0x0F12, 0x70AC},
	{0x0F12, 0xE59F},
	{0x0F12, 0x10F4},
	{0x0F12, 0xE1D6},
	{0x0F12, 0x26B0},
	{0x0F12, 0xE1D7},
	{0x0F12, 0x00F0},
	{0x0F12, 0xE1D4},
	{0x0F12, 0x0044},
	{0x0F12, 0xEB00},
	{0x0F12, 0x00B0},
	{0x0F12, 0xE1C4},
	{0x0F12, 0x26B0},
	{0x0F12, 0xE1D7},
	{0x0F12, 0x10F6},
	{0x0F12, 0xE1D6},
	{0x0F12, 0x00F0},
	{0x0F12, 0xE1D5},
	{0x0F12, 0x003F},
	{0x0F12, 0xEB00},
	{0x0F12, 0x00B0},
	{0x0F12, 0xE1C5},
	{0x0F12, 0x41F0},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x4010},
	{0x0F12, 0xE92D},
	{0x0F12, 0x4000},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x1004},
	{0x0F12, 0xE594},
	{0x0F12, 0x0040},
	{0x0F12, 0xE59F},
	{0x0F12, 0x00B0},
	{0x0F12, 0xE1D0},
	{0x0F12, 0x0000},
	{0x0F12, 0xE350},
	{0x0F12, 0x0008},
	{0x0F12, 0x0A00},
	{0x0F12, 0x005C},
	{0x0F12, 0xE59F},
	{0x0F12, 0x3001},
	{0x0F12, 0xE1A0},
	{0x0F12, 0x2068},
	{0x0F12, 0xE590},
	{0x0F12, 0x0054},
	{0x0F12, 0xE59F},
	{0x0F12, 0x1005},
	{0x0F12, 0xE3A0},
	{0x0F12, 0x0032},
	{0x0F12, 0xEB00},
	{0x0F12, 0x0000},
	{0x0F12, 0xE584},
	{0x0F12, 0x4010},
	{0x0F12, 0xE8BD},
	{0x0F12, 0xFF1E},
	{0x0F12, 0xE12F},
	{0x0F12, 0x0000},
	{0x0F12, 0xE594},
	{0x0F12, 0x0030},
	{0x0F12, 0xEB00},
	{0x0F12, 0x0000},
	{0x0F12, 0xE584},
	{0x0F12, 0xFFF9},
	{0x0F12, 0xEAFF},
	{0x0F12, 0x28E8},
	{0x0F12, 0x7000},
	{0x0F12, 0x3370},
	{0x0F12, 0x7000},
	{0x0F12, 0x1272},
	{0x0F12, 0x7000},
	{0x0F12, 0x1728},
	{0x0F12, 0x7000},
	{0x0F12, 0x112C},
	{0x0F12, 0x7000},
	{0x0F12, 0x28EC},
	{0x0F12, 0x7000},
	{0x0F12, 0x122C},
	{0x0F12, 0x7000},
	{0x0F12, 0xF200},
	{0x0F12, 0xD000},
	{0x0F12, 0x2340},
	{0x0F12, 0x7000},
	{0x0F12, 0x0E2C},
	{0x0F12, 0x7000},
	{0x0F12, 0xF400},
	{0x0F12, 0xD000},
	{0x0F12, 0x0CDC},
	{0x0F12, 0x7000},
	{0x0F12, 0x20D4},
	{0x0F12, 0x7000},
	{0x0F12, 0x06D4},
	{0x0F12, 0x7000},
	{0x0F12, 0x4778},
	{0x0F12, 0x46C0},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0xC091},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x0467},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x2FA7},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0xCB1F},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x058F},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0xA0F1},
	{0x0F12, 0x0000},
	{0x0F12, 0xF004},
	{0x0F12, 0xE51F},
	{0x0F12, 0xD14C},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x2B43},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x8725},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x6777},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x8E49},
	{0x0F12, 0x0000},
	{0x0F12, 0xC000},
	{0x0F12, 0xE59F},
	{0x0F12, 0xFF1C},
	{0x0F12, 0xE12F},
	{0x0F12, 0x8EDD},
	{0x0F12, 0x0000},
	{0x0F12, 0x96FF},
	{0x0F12, 0x0000},
	{0x0F12, 0x0001},
	{0x0F12, 0x0000},
//Parameters Defined in T&P:
//                          14580 70000000 STRUCT
//TnP_SvnVersion                2 700028E4 SHORT
//Tune_TP                      10 70003370 STRUCT
//afit_bUseNormBrForAfit        2 70003370 SHORT
//awbb_bUseOutdoorGrid          2 70003372 SHORT
//awbb_OutdoorGridCorr_R        2 70003374 SHORT
//awbb_OutdoorGridCorr_B        2 70003376 SHORT
//Tune_TP_bReMultGainsByNvm     2 70003378 SHORT
//End of Patch data

//==================================================================================
// 04.Analog Setting & APS Control
//==================================================================================
//This register is for FACTORY ONLY.
//If you change it without prior notification
//YOU are RESPONSIBLE for the FAILURE that will happen in the future
	{0x0028, 0x7000},
	{0x002A, 0x0E38},
	{0x0F12, 0x0476},	//senHal_RegCompBiasNormSf //CDS bias
	{0x0F12, 0x0476},	//senHal_RegCompBiasYAv //CDS bias
	{0x002A, 0x0AA0},
	{0x0F12, 0x0001},	//setot_bUseDigitalHbin //1-Digital, 0-Analog
	{0x002A, 0x0E2C},
	{0x0F12, 0x0001},	//senHal_bUseAnalogVerAv //2-Adding/averaging, 1-Y-Avg, 0-PLA
	{0x002A, 0x0E66},
	{0x0F12, 0x0001},	//senHal_RegBlstEnNorm
	{0x002A, 0x1250},
	{0x0F12, 0xFFFF},	//senHal_Bls_nSpExpLines
	{0x002A, 0x1202},
	{0x0F12, 0x0010},	//senHal_Dblr_VcoFreqMHZ
//ADLC Filter
	{0x002A, 0x1288},
	{0x0F12, 0x020F},	//gisp_dadlc_ResetFilterValue
	{0x0F12, 0x1C02},	//gisp_dadlc_SteadyFilterValue
	{0x0F12, 0x0006},	//gisp_dadlc_NResetIIrFrames

//==================================================================================
// 05.OTP Control
//==================================================================================
	{0x002A, 0x3378},
	{0x0F12, 0x0000},	//Tune_TP_bReMultGainsByNvm

//==================================================================================
// 06.GAS (Grid Anti-Shading)
//==================================================================================
	{0x002A, 0x1326},
	{0x0F12, 0x0000},	//gisp_gos_Enable
	{0x002A, 0x063A},
	{0x0F12, 0x0110},	//0110	//TVAR_ash_GASAlpha_0__0_ //Horizon
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_0__1_
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_0__2_
	{0x0F12, 0x00F0},	//00F0	//TVAR_ash_GASAlpha_0__3_
	{0x0F12, 0x0120},	//0120	//TVAR_ash_GASAlpha_1__0_ //IncandA
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_1__1_
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_1__2_
	{0x0F12, 0x00F0},	//00F0	//TVAR_ash_GASAlpha_1__3_
	{0x0F12, 0x00DC},	//0100	//TVAR_ash_GASAlpha_2__0_ //WW
	{0x0F12, 0x00F0},	//0100	//TVAR_ash_GASAlpha_2__1_
	{0x0F12, 0x00F0},	//0100	//TVAR_ash_GASAlpha_2__2_
	{0x0F12, 0x00E0},	//0100	//TVAR_ash_GASAlpha_2__3_
	{0x0F12, 0x00DC},	//00E8	//TVAR_ash_GASAlpha_3__0_ //CW
	{0x0F12, 0x00F0},	//0100	//TVAR_ash_GASAlpha_3__1_
	{0x0F12, 0x00F0},	//0100	//TVAR_ash_GASAlpha_3__2_
	{0x0F12, 0x00E0},	//0100	//TVAR_ash_GASAlpha_3__3_
	{0x0F12, 0x00D8},	//00D8	//TVAR_ash_GASAlpha_4__0_ //D50
	{0x0F12, 0x00F8},	//00F8	//TVAR_ash_GASAlpha_4__1_
	{0x0F12, 0x00F8},	//00F8	//TVAR_ash_GASAlpha_4__2_
	{0x0F12, 0x00F0},	//00F0	//TVAR_ash_GASAlpha_4__3_
	{0x0F12, 0x00C0},	//00C0	//TVAR_ash_GASAlpha_5__0_ //D65
	{0x0F12, 0x00D8},	//00D8	//TVAR_ash_GASAlpha_5__1_
	{0x0F12, 0x00D8},	//00D8	//TVAR_ash_GASAlpha_5__2_
	{0x0F12, 0x00D0},	//00D0	//TVAR_ash_GASAlpha_5__3_
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_6__0_ //D75
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_6__1_
	{0x0F12, 0x0100},	//0100	//TVAR_ash_GASAlpha_6__2_
	{0x0F12, 0x00F0},	//00F0	//TVAR_ash_GASAlpha_6__3_
	{0x002A, 0x067A},
	{0x0F12, 0x0000},	//ash_GASBeta_0__0_ //Horizon
	{0x0F12, 0x0000},	//ash_GASBeta_0__1_
	{0x0F12, 0x0000},	//ash_GASBeta_0__2_
	{0x0F12, 0x0000},	//ash_GASBeta_0__3_
	{0x0F12, 0x0000},	//ash_GASBeta_1__0_ //IncandA
	{0x0F12, 0x0000},	//ash_GASBeta_1__1_
	{0x0F12, 0x0000},	//ash_GASBeta_1__2_
	{0x0F12, 0x0000},	//ash_GASBeta_1__3_
	{0x0F12, 0x0000},	//ash_GASBeta_2__0_ //WW
	{0x0F12, 0x0000},	//ash_GASBeta_2__1_
	{0x0F12, 0x0000},	//ash_GASBeta_2__2_
	{0x0F12, 0x0000},	//ash_GASBeta_2__3_
	{0x0F12, 0x0000},	//ash_GASBeta_3__0_ //CW
	{0x0F12, 0x0000},	//ash_GASBeta_3__1_
	{0x0F12, 0x0000},	//ash_GASBeta_3__2_
	{0x0F12, 0x0000},	//ash_GASBeta_3__3_
	{0x0F12, 0x0000},	//ash_GASBeta_4__0_ //D50
	{0x0F12, 0x0000},	//ash_GASBeta_4__1_
	{0x0F12, 0x0000},	//ash_GASBeta_4__2_
	{0x0F12, 0x0000},	//ash_GASBeta_4__3_
	{0x0F12, 0x0000},	//ash_GASBeta_5__0_ //D65
	{0x0F12, 0x0000},	//ash_GASBeta_5__1_
	{0x0F12, 0x0000},	//ash_GASBeta_5__2_
	{0x0F12, 0x0000},	//ash_GASBeta_5__3_
	{0x0F12, 0x0000},	//ash_GASBeta_6__0_ //D75
	{0x0F12, 0x0000},	//ash_GASBeta_6__1_
	{0x0F12, 0x0000},	//ash_GASBeta_6__2_
	{0x0F12, 0x0000},	//ash_GASBeta_6__3_
	{0x002A, 0x06BA},
	{0x0F12, 0x0001},	//ash_bLumaMode
	{0x002A, 0x0632},
	{0x0F12, 0x0100},	//ash_CGrasAlphas_0_
	{0x0F12, 0x0100},	//ash_CGrasAlphas_1_
	{0x0F12, 0x0100},	//ash_CGrasAlphas_2_
	{0x0F12, 0x0100},	//ash_CGrasAlphas_3_
	{0x002A, 0x0672},
	{0x0F12, 0x0108},	//TVAR_ash_GASOutdoorAlpha_0_
	{0x0F12, 0x00E0},	//TVAR_ash_GASOutdoorAlpha_1_
	{0x0F12, 0x00E0},	//TVAR_ash_GASOutdoorAlpha_2_
	{0x0F12, 0x00E0},	//TVAR_ash_GASOutdoorAlpha_3_
	{0x002A, 0x06B2},
	{0x0F12, 0x0000},	//ash_GASOutdoorBeta_0_
	{0x0F12, 0x0000},	//ash_GASOutdoorBeta_1_
	{0x0F12, 0x0000},	//ash_GASOutdoorBeta_2_
	{0x0F12, 0x0000},	//ash_GASOutdoorBeta_3_
	{0x002A, 0x06D0},
	{0x0F12, 0x000D},	//ash_uParabolicScalingA
	{0x0F12, 0x000F},	//ash_uParabolicScalingB
	{0x002A, 0x06CC},
	{0x0F12, 0x0280},	//ash_uParabolicCenterX
	{0x0F12, 0x01E0},	//ash_uParabolicCenterY
	{0x002A, 0x06C6},
	{0x0F12, 0x0001},	//ash_bParabolicEstimation
	{0x002A, 0x0624},
	{0x0F12, 0x009D},	//TVAR_ash_AwbAshCord_0_ //Horizon
	{0x0F12, 0x00D5},	//TVAR_ash_AwbAshCord_1_ //IncandA
	{0x0F12, 0x0103},	//TVAR_ash_AwbAshCord_2_ //WW
	{0x0F12, 0x0128},	//TVAR_ash_AwbAshCord_3_ //CW
	{0x0F12, 0x0166},	//TVAR_ash_AwbAshCord_4_ //D50
	{0x0F12, 0x0193},	//TVAR_ash_AwbAshCord_5_ //D65
	{0x0F12, 0x01A0},	//TVAR_ash_AwbAshCord_6_ //D75
	{0x002A, 0x347C},
	{0x0F12, 0x0169},	//Tune_wbt_GAS_0_
	{0x0F12, 0x012F},	//Tune_wbt_GAS_1_
	{0x0F12, 0x00F7},	//Tune_wbt_GAS_2_
	{0x0F12, 0x00C5},	//Tune_wbt_GAS_3_
	{0x0F12, 0x00A1},	//Tune_wbt_GAS_4_
	{0x0F12, 0x0088},	//Tune_wbt_GAS_5_
	{0x0F12, 0x008B},	//Tune_wbt_GAS_6_
	{0x0F12, 0x0097},	//Tune_wbt_GAS_7_
	{0x0F12, 0x00B1},	//Tune_wbt_GAS_8_
	{0x0F12, 0x00DF},	//Tune_wbt_GAS_9_
	{0x0F12, 0x0111},	//Tune_wbt_GAS_10_
	{0x0F12, 0x0150},	//Tune_wbt_GAS_11_
	{0x0F12, 0x0196},	//Tune_wbt_GAS_12_
	{0x0F12, 0x0147},	//Tune_wbt_GAS_13_
	{0x0F12, 0x010E},	//Tune_wbt_GAS_14_
	{0x0F12, 0x00CD},	//Tune_wbt_GAS_15_
	{0x0F12, 0x0097},	//Tune_wbt_GAS_16_
	{0x0F12, 0x0073},	//Tune_wbt_GAS_17_
	{0x0F12, 0x005B},	//Tune_wbt_GAS_18_
	{0x0F12, 0x0053},	//Tune_wbt_GAS_19_
	{0x0F12, 0x005F},	//Tune_wbt_GAS_20_
	{0x0F12, 0x007A},	//Tune_wbt_GAS_21_
	{0x0F12, 0x00A6},	//Tune_wbt_GAS_22_
	{0x0F12, 0x00E6},	//Tune_wbt_GAS_23_
	{0x0F12, 0x0128},	//Tune_wbt_GAS_24_
	{0x0F12, 0x0165},	//Tune_wbt_GAS_25_
	{0x0F12, 0x011F},	//Tune_wbt_GAS_26_
	{0x0F12, 0x00E5},	//Tune_wbt_GAS_27_
	{0x0F12, 0x00A0},	//Tune_wbt_GAS_28_
	{0x0F12, 0x006B},	//Tune_wbt_GAS_29_
	{0x0F12, 0x0043},	//Tune_wbt_GAS_30_
	{0x0F12, 0x002F},	//Tune_wbt_GAS_31_
	{0x0F12, 0x0026},	//Tune_wbt_GAS_32_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_33_
	{0x0F12, 0x004F},	//Tune_wbt_GAS_34_
	{0x0F12, 0x007F},	//Tune_wbt_GAS_35_
	{0x0F12, 0x00B9},	//Tune_wbt_GAS_36_
	{0x0F12, 0x00FF},	//Tune_wbt_GAS_37_
	{0x0F12, 0x0136},	//Tune_wbt_GAS_38_
	{0x0F12, 0x0112},	//Tune_wbt_GAS_39_
	{0x0F12, 0x00D4},	//Tune_wbt_GAS_40_
	{0x0F12, 0x0089},	//Tune_wbt_GAS_41_
	{0x0F12, 0x0050},	//Tune_wbt_GAS_42_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_43_
	{0x0F12, 0x0019},	//Tune_wbt_GAS_44_
	{0x0F12, 0x0010},	//Tune_wbt_GAS_45_
	{0x0F12, 0x001F},	//Tune_wbt_GAS_46_
	{0x0F12, 0x0038},	//Tune_wbt_GAS_47_
	{0x0F12, 0x0065},	//Tune_wbt_GAS_48_
	{0x0F12, 0x00A2},	//Tune_wbt_GAS_49_
	{0x0F12, 0x00EE},	//Tune_wbt_GAS_50_
	{0x0F12, 0x012A},	//Tune_wbt_GAS_51_
	{0x0F12, 0x0104},	//Tune_wbt_GAS_52_
	{0x0F12, 0x00C3},	//Tune_wbt_GAS_53_
	{0x0F12, 0x0078},	//Tune_wbt_GAS_54_
	{0x0F12, 0x003F},	//Tune_wbt_GAS_55_
	{0x0F12, 0x001C},	//Tune_wbt_GAS_56_
	{0x0F12, 0x000A},	//Tune_wbt_GAS_57_
	{0x0F12, 0x0003},	//Tune_wbt_GAS_58_
	{0x0F12, 0x0007},	//Tune_wbt_GAS_59_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_60_
	{0x0F12, 0x0049},	//Tune_wbt_GAS_61_
	{0x0F12, 0x0091},	//Tune_wbt_GAS_62_
	{0x0F12, 0x00DE},	//Tune_wbt_GAS_63_
	{0x0F12, 0x011C},	//Tune_wbt_GAS_64_
	{0x0F12, 0x00FF},	//Tune_wbt_GAS_65_
	{0x0F12, 0x00BD},	//Tune_wbt_GAS_66_
	{0x0F12, 0x0074},	//Tune_wbt_GAS_67_
	{0x0F12, 0x003C},	//Tune_wbt_GAS_68_
	{0x0F12, 0x0019},	//Tune_wbt_GAS_69_
	{0x0F12, 0x0006},	//Tune_wbt_GAS_70_
	{0x0F12, 0x0000},	//Tune_wbt_GAS_71_
	{0x0F12, 0x0004},	//Tune_wbt_GAS_72_
	{0x0F12, 0x0021},	//Tune_wbt_GAS_73_
	{0x0F12, 0x0046},	//Tune_wbt_GAS_74_
	{0x0F12, 0x008D},	//Tune_wbt_GAS_75_
	{0x0F12, 0x00DC},	//Tune_wbt_GAS_76_
	{0x0F12, 0x0116},	//Tune_wbt_GAS_77_
	{0x0F12, 0x010F},	//Tune_wbt_GAS_78_
	{0x0F12, 0x00CC},	//Tune_wbt_GAS_79_
	{0x0F12, 0x0081},	//Tune_wbt_GAS_80_
	{0x0F12, 0x0049},	//Tune_wbt_GAS_81_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_82_
	{0x0F12, 0x0011},	//Tune_wbt_GAS_83_
	{0x0F12, 0x000A},	//Tune_wbt_GAS_84_
	{0x0F12, 0x0016},	//Tune_wbt_GAS_85_
	{0x0F12, 0x0030},	//Tune_wbt_GAS_86_
	{0x0F12, 0x005B},	//Tune_wbt_GAS_87_
	{0x0F12, 0x009B},	//Tune_wbt_GAS_88_
	{0x0F12, 0x00EB},	//Tune_wbt_GAS_89_
	{0x0F12, 0x0126},	//Tune_wbt_GAS_90_
	{0x0F12, 0x0116},	//Tune_wbt_GAS_91_
	{0x0F12, 0x00D8},	//Tune_wbt_GAS_92_
	{0x0F12, 0x0092},	//Tune_wbt_GAS_93_
	{0x0F12, 0x0059},	//Tune_wbt_GAS_94_
	{0x0F12, 0x0036},	//Tune_wbt_GAS_95_
	{0x0F12, 0x001F},	//Tune_wbt_GAS_96_
	{0x0F12, 0x001C},	//Tune_wbt_GAS_97_
	{0x0F12, 0x0024},	//Tune_wbt_GAS_98_
	{0x0F12, 0x003E},	//Tune_wbt_GAS_99_
	{0x0F12, 0x006E},	//Tune_wbt_GAS_100_
	{0x0F12, 0x00AC},	//Tune_wbt_GAS_101_
	{0x0F12, 0x00F8},	//Tune_wbt_GAS_102_
	{0x0F12, 0x0133},	//Tune_wbt_GAS_103_
	{0x0F12, 0x0133},	//Tune_wbt_GAS_104_
	{0x0F12, 0x00FB},	//Tune_wbt_GAS_105_
	{0x0F12, 0x00B8},	//Tune_wbt_GAS_106_
	{0x0F12, 0x0080},	//Tune_wbt_GAS_107_
	{0x0F12, 0x0059},	//Tune_wbt_GAS_108_
	{0x0F12, 0x0046},	//Tune_wbt_GAS_109_
	{0x0F12, 0x0041},	//Tune_wbt_GAS_110_
	{0x0F12, 0x004C},	//Tune_wbt_GAS_111_
	{0x0F12, 0x0067},	//Tune_wbt_GAS_112_
	{0x0F12, 0x0090},	//Tune_wbt_GAS_113_
	{0x0F12, 0x00C7},	//Tune_wbt_GAS_114_
	{0x0F12, 0x0117},	//Tune_wbt_GAS_115_
	{0x0F12, 0x0154},	//Tune_wbt_GAS_116_
	{0x0F12, 0x0160},	//Tune_wbt_GAS_117_
	{0x0F12, 0x0129},	//Tune_wbt_GAS_118_
	{0x0F12, 0x00ED},	//Tune_wbt_GAS_119_
	{0x0F12, 0x00B7},	//Tune_wbt_GAS_120_
	{0x0F12, 0x0091},	//Tune_wbt_GAS_121_
	{0x0F12, 0x007B},	//Tune_wbt_GAS_122_
	{0x0F12, 0x0076},	//Tune_wbt_GAS_123_
	{0x0F12, 0x0082},	//Tune_wbt_GAS_124_
	{0x0F12, 0x009E},	//Tune_wbt_GAS_125_
	{0x0F12, 0x00CA},	//Tune_wbt_GAS_126_
	{0x0F12, 0x0102},	//Tune_wbt_GAS_127_
	{0x0F12, 0x014A},	//Tune_wbt_GAS_128_
	{0x0F12, 0x01A0},	//Tune_wbt_GAS_129_
	{0x0F12, 0x0189},	//Tune_wbt_GAS_130_
	{0x0F12, 0x0146},	//Tune_wbt_GAS_131_
	{0x0F12, 0x010B},	//Tune_wbt_GAS_132_
	{0x0F12, 0x00DF},	//Tune_wbt_GAS_133_
	{0x0F12, 0x00BB},	//Tune_wbt_GAS_134_
	{0x0F12, 0x00A7},	//Tune_wbt_GAS_135_
	{0x0F12, 0x00A2},	//Tune_wbt_GAS_136_
	{0x0F12, 0x00AB},	//Tune_wbt_GAS_137_
	{0x0F12, 0x00C5},	//Tune_wbt_GAS_138_
	{0x0F12, 0x00ED},	//Tune_wbt_GAS_139_
	{0x0F12, 0x0121},	//Tune_wbt_GAS_140_
	{0x0F12, 0x0177},	//Tune_wbt_GAS_141_
	{0x0F12, 0x01F2},	//Tune_wbt_GAS_142_
	{0x0F12, 0x00C8},	//Tune_wbt_GAS_143_
	{0x0F12, 0x00A2},	//Tune_wbt_GAS_144_
	{0x0F12, 0x0086},	//Tune_wbt_GAS_145_
	{0x0F12, 0x006E},	//Tune_wbt_GAS_146_
	{0x0F12, 0x005D},	//Tune_wbt_GAS_147_
	{0x0F12, 0x004C},	//Tune_wbt_GAS_148_
	{0x0F12, 0x0050},	//Tune_wbt_GAS_149_
	{0x0F12, 0x0058},	//Tune_wbt_GAS_150_
	{0x0F12, 0x0069},	//Tune_wbt_GAS_151_
	{0x0F12, 0x0081},	//Tune_wbt_GAS_152_
	{0x0F12, 0x009B},	//Tune_wbt_GAS_153_
	{0x0F12, 0x00C2},	//Tune_wbt_GAS_154_
	{0x0F12, 0x00F7},	//Tune_wbt_GAS_155_
	{0x0F12, 0x00B8},	//Tune_wbt_GAS_156_
	{0x0F12, 0x0092},	//Tune_wbt_GAS_157_
	{0x0F12, 0x0071},	//Tune_wbt_GAS_158_
	{0x0F12, 0x0055},	//Tune_wbt_GAS_159_
	{0x0F12, 0x0042},	//Tune_wbt_GAS_160_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_161_
	{0x0F12, 0x002E},	//Tune_wbt_GAS_162_
	{0x0F12, 0x0037},	//Tune_wbt_GAS_163_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_164_
	{0x0F12, 0x0060},	//Tune_wbt_GAS_165_
	{0x0F12, 0x0084},	//Tune_wbt_GAS_166_
	{0x0F12, 0x00A7},	//Tune_wbt_GAS_167_
	{0x0F12, 0x00D4},	//Tune_wbt_GAS_168_
	{0x0F12, 0x00A1},	//Tune_wbt_GAS_169_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_170_
	{0x0F12, 0x005B},	//Tune_wbt_GAS_171_
	{0x0F12, 0x003F},	//Tune_wbt_GAS_172_
	{0x0F12, 0x0028},	//Tune_wbt_GAS_173_
	{0x0F12, 0x001B},	//Tune_wbt_GAS_174_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_175_
	{0x0F12, 0x001E},	//Tune_wbt_GAS_176_
	{0x0F12, 0x002F},	//Tune_wbt_GAS_177_
	{0x0F12, 0x004A},	//Tune_wbt_GAS_178_
	{0x0F12, 0x006A},	//Tune_wbt_GAS_179_
	{0x0F12, 0x008F},	//Tune_wbt_GAS_180_
	{0x0F12, 0x00B3},	//Tune_wbt_GAS_181_
	{0x0F12, 0x009E},	//Tune_wbt_GAS_182_
	{0x0F12, 0x007B},	//Tune_wbt_GAS_183_
	{0x0F12, 0x0053},	//Tune_wbt_GAS_184_
	{0x0F12, 0x0033},	//Tune_wbt_GAS_185_
	{0x0F12, 0x001E},	//Tune_wbt_GAS_186_
	{0x0F12, 0x0010},	//Tune_wbt_GAS_187_
	{0x0F12, 0x000A},	//Tune_wbt_GAS_188_
	{0x0F12, 0x0015},	//Tune_wbt_GAS_189_
	{0x0F12, 0x0026},	//Tune_wbt_GAS_190_
	{0x0F12, 0x003F},	//Tune_wbt_GAS_191_
	{0x0F12, 0x0062},	//Tune_wbt_GAS_192_
	{0x0F12, 0x008B},	//Tune_wbt_GAS_193_
	{0x0F12, 0x00AF},	//Tune_wbt_GAS_194_
	{0x0F12, 0x0094},	//Tune_wbt_GAS_195_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_196_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_197_
	{0x0F12, 0x0028},	//Tune_wbt_GAS_198_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_199_
	{0x0F12, 0x0007},	//Tune_wbt_GAS_200_
	{0x0F12, 0x0002},	//Tune_wbt_GAS_201_
	{0x0F12, 0x0005},	//Tune_wbt_GAS_202_
	{0x0F12, 0x001B},	//Tune_wbt_GAS_203_
	{0x0F12, 0x002E},	//Tune_wbt_GAS_204_
	{0x0F12, 0x0059},	//Tune_wbt_GAS_205_
	{0x0F12, 0x0082},	//Tune_wbt_GAS_206_
	{0x0F12, 0x00A7},	//Tune_wbt_GAS_207_
	{0x0F12, 0x0093},	//Tune_wbt_GAS_208_
	{0x0F12, 0x006C},	//Tune_wbt_GAS_209_
	{0x0F12, 0x0045},	//Tune_wbt_GAS_210_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_211_
	{0x0F12, 0x0010},	//Tune_wbt_GAS_212_
	{0x0F12, 0x0005},	//Tune_wbt_GAS_213_
	{0x0F12, 0x0000},	//Tune_wbt_GAS_214_
	{0x0F12, 0x0003},	//Tune_wbt_GAS_215_
	{0x0F12, 0x0018},	//Tune_wbt_GAS_216_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_217_
	{0x0F12, 0x0057},	//Tune_wbt_GAS_218_
	{0x0F12, 0x0083},	//Tune_wbt_GAS_219_
	{0x0F12, 0x00A5},	//Tune_wbt_GAS_220_
	{0x0F12, 0x009D},	//Tune_wbt_GAS_221_
	{0x0F12, 0x0078},	//Tune_wbt_GAS_222_
	{0x0F12, 0x0050},	//Tune_wbt_GAS_223_
	{0x0F12, 0x0031},	//Tune_wbt_GAS_224_
	{0x0F12, 0x001B},	//Tune_wbt_GAS_225_
	{0x0F12, 0x000E},	//Tune_wbt_GAS_226_
	{0x0F12, 0x0009},	//Tune_wbt_GAS_227_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_228_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_229_
	{0x0F12, 0x003D},	//Tune_wbt_GAS_230_
	{0x0F12, 0x0063},	//Tune_wbt_GAS_231_
	{0x0F12, 0x008E},	//Tune_wbt_GAS_232_
	{0x0F12, 0x00B2},	//Tune_wbt_GAS_233_
	{0x0F12, 0x00A2},	//Tune_wbt_GAS_234_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_235_
	{0x0F12, 0x005A},	//Tune_wbt_GAS_236_
	{0x0F12, 0x003A},	//Tune_wbt_GAS_237_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_238_
	{0x0F12, 0x0015},	//Tune_wbt_GAS_239_
	{0x0F12, 0x0015},	//Tune_wbt_GAS_240_
	{0x0F12, 0x001A},	//Tune_wbt_GAS_241_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_242_
	{0x0F12, 0x0049},	//Tune_wbt_GAS_243_
	{0x0F12, 0x006C},	//Tune_wbt_GAS_244_
	{0x0F12, 0x0095},	//Tune_wbt_GAS_245_
	{0x0F12, 0x00BA},	//Tune_wbt_GAS_246_
	{0x0F12, 0x00B5},	//Tune_wbt_GAS_247_
	{0x0F12, 0x0092},	//Tune_wbt_GAS_248_
	{0x0F12, 0x0070},	//Tune_wbt_GAS_249_
	{0x0F12, 0x0050},	//Tune_wbt_GAS_250_
	{0x0F12, 0x003B},	//Tune_wbt_GAS_251_
	{0x0F12, 0x002F},	//Tune_wbt_GAS_252_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_253_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_254_
	{0x0F12, 0x0045},	//Tune_wbt_GAS_255_
	{0x0F12, 0x005B},	//Tune_wbt_GAS_256_
	{0x0F12, 0x0079},	//Tune_wbt_GAS_257_
	{0x0F12, 0x00A6},	//Tune_wbt_GAS_258_
	{0x0F12, 0x00CF},	//Tune_wbt_GAS_259_
	{0x0F12, 0x00D4},	//Tune_wbt_GAS_260_
	{0x0F12, 0x00AE},	//Tune_wbt_GAS_261_
	{0x0F12, 0x008F},	//Tune_wbt_GAS_262_
	{0x0F12, 0x0074},	//Tune_wbt_GAS_263_
	{0x0F12, 0x005E},	//Tune_wbt_GAS_264_
	{0x0F12, 0x0051},	//Tune_wbt_GAS_265_
	{0x0F12, 0x004E},	//Tune_wbt_GAS_266_
	{0x0F12, 0x0056},	//Tune_wbt_GAS_267_
	{0x0F12, 0x0066},	//Tune_wbt_GAS_268_
	{0x0F12, 0x0080},	//Tune_wbt_GAS_269_
	{0x0F12, 0x009D},	//Tune_wbt_GAS_270_
	{0x0F12, 0x00C7},	//Tune_wbt_GAS_271_
	{0x0F12, 0x0106},	//Tune_wbt_GAS_272_
	{0x0F12, 0x00EF},	//Tune_wbt_GAS_273_
	{0x0F12, 0x00C0},	//Tune_wbt_GAS_274_
	{0x0F12, 0x00A1},	//Tune_wbt_GAS_275_
	{0x0F12, 0x0088},	//Tune_wbt_GAS_276_
	{0x0F12, 0x0077},	//Tune_wbt_GAS_277_
	{0x0F12, 0x006B},	//Tune_wbt_GAS_278_
	{0x0F12, 0x0068},	//Tune_wbt_GAS_279_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_280_
	{0x0F12, 0x007F},	//Tune_wbt_GAS_281_
	{0x0F12, 0x0095},	//Tune_wbt_GAS_282_
	{0x0F12, 0x00B2},	//Tune_wbt_GAS_283_
	{0x0F12, 0x00EB},	//Tune_wbt_GAS_284_
	{0x0F12, 0x014D},	//Tune_wbt_GAS_285_
	{0x0F12, 0x00D0},	//Tune_wbt_GAS_286_
	{0x0F12, 0x00A7},	//Tune_wbt_GAS_287_
	{0x0F12, 0x008A},	//Tune_wbt_GAS_288_
	{0x0F12, 0x0070},	//Tune_wbt_GAS_289_
	{0x0F12, 0x005D},	//Tune_wbt_GAS_290_
	{0x0F12, 0x004B},	//Tune_wbt_GAS_291_
	{0x0F12, 0x004E},	//Tune_wbt_GAS_292_
	{0x0F12, 0x0057},	//Tune_wbt_GAS_293_
	{0x0F12, 0x0066},	//Tune_wbt_GAS_294_
	{0x0F12, 0x007D},	//Tune_wbt_GAS_295_
	{0x0F12, 0x0097},	//Tune_wbt_GAS_296_
	{0x0F12, 0x00BC},	//Tune_wbt_GAS_297_
	{0x0F12, 0x00F1},	//Tune_wbt_GAS_298_
	{0x0F12, 0x00C1},	//Tune_wbt_GAS_299_
	{0x0F12, 0x009B},	//Tune_wbt_GAS_300_
	{0x0F12, 0x0078},	//Tune_wbt_GAS_301_
	{0x0F12, 0x0058},	//Tune_wbt_GAS_302_
	{0x0F12, 0x0044},	//Tune_wbt_GAS_303_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_304_
	{0x0F12, 0x002D},	//Tune_wbt_GAS_305_
	{0x0F12, 0x0036},	//Tune_wbt_GAS_306_
	{0x0F12, 0x0046},	//Tune_wbt_GAS_307_
	{0x0F12, 0x005D},	//Tune_wbt_GAS_308_
	{0x0F12, 0x0080},	//Tune_wbt_GAS_309_
	{0x0F12, 0x00A4},	//Tune_wbt_GAS_310_
	{0x0F12, 0x00CF},	//Tune_wbt_GAS_311_
	{0x0F12, 0x00AF},	//Tune_wbt_GAS_312_
	{0x0F12, 0x008A},	//Tune_wbt_GAS_313_
	{0x0F12, 0x0064},	//Tune_wbt_GAS_314_
	{0x0F12, 0x0045},	//Tune_wbt_GAS_315_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_316_
	{0x0F12, 0x001D},	//Tune_wbt_GAS_317_
	{0x0F12, 0x0014},	//Tune_wbt_GAS_318_
	{0x0F12, 0x001E},	//Tune_wbt_GAS_319_
	{0x0F12, 0x002E},	//Tune_wbt_GAS_320_
	{0x0F12, 0x0049},	//Tune_wbt_GAS_321_
	{0x0F12, 0x0068},	//Tune_wbt_GAS_322_
	{0x0F12, 0x008D},	//Tune_wbt_GAS_323_
	{0x0F12, 0x00B1},	//Tune_wbt_GAS_324_
	{0x0F12, 0x00AD},	//Tune_wbt_GAS_325_
	{0x0F12, 0x0088},	//Tune_wbt_GAS_326_
	{0x0F12, 0x005D},	//Tune_wbt_GAS_327_
	{0x0F12, 0x003A},	//Tune_wbt_GAS_328_
	{0x0F12, 0x0021},	//Tune_wbt_GAS_329_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_330_
	{0x0F12, 0x000C},	//Tune_wbt_GAS_331_
	{0x0F12, 0x0016},	//Tune_wbt_GAS_332_
	{0x0F12, 0x0025},	//Tune_wbt_GAS_333_
	{0x0F12, 0x003E},	//Tune_wbt_GAS_334_
	{0x0F12, 0x0061},	//Tune_wbt_GAS_335_
	{0x0F12, 0x0089},	//Tune_wbt_GAS_336_
	{0x0F12, 0x00AC},	//Tune_wbt_GAS_337_
	{0x0F12, 0x00A4},	//Tune_wbt_GAS_338_
	{0x0F12, 0x007D},	//Tune_wbt_GAS_339_
	{0x0F12, 0x0052},	//Tune_wbt_GAS_340_
	{0x0F12, 0x002F},	//Tune_wbt_GAS_341_
	{0x0F12, 0x0018},	//Tune_wbt_GAS_342_
	{0x0F12, 0x000A},	//Tune_wbt_GAS_343_
	{0x0F12, 0x0004},	//Tune_wbt_GAS_344_
	{0x0F12, 0x0005},	//Tune_wbt_GAS_345_
	{0x0F12, 0x001A},	//Tune_wbt_GAS_346_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_347_
	{0x0F12, 0x0057},	//Tune_wbt_GAS_348_
	{0x0F12, 0x0081},	//Tune_wbt_GAS_349_
	{0x0F12, 0x00A4},	//Tune_wbt_GAS_350_
	{0x0F12, 0x00A3},	//Tune_wbt_GAS_351_
	{0x0F12, 0x007A},	//Tune_wbt_GAS_352_
	{0x0F12, 0x0051},	//Tune_wbt_GAS_353_
	{0x0F12, 0x002D},	//Tune_wbt_GAS_354_
	{0x0F12, 0x0016},	//Tune_wbt_GAS_355_
	{0x0F12, 0x0008},	//Tune_wbt_GAS_356_
	{0x0F12, 0x0002},	//Tune_wbt_GAS_357_
	{0x0F12, 0x0004},	//Tune_wbt_GAS_358_
	{0x0F12, 0x0017},	//Tune_wbt_GAS_359_
	{0x0F12, 0x002B},	//Tune_wbt_GAS_360_
	{0x0F12, 0x0057},	//Tune_wbt_GAS_361_
	{0x0F12, 0x0082},	//Tune_wbt_GAS_362_
	{0x0F12, 0x00A4},	//Tune_wbt_GAS_363_
	{0x0F12, 0x00AE},	//Tune_wbt_GAS_364_
	{0x0F12, 0x0085},	//Tune_wbt_GAS_365_
	{0x0F12, 0x005C},	//Tune_wbt_GAS_366_
	{0x0F12, 0x0039},	//Tune_wbt_GAS_367_
	{0x0F12, 0x0021},	//Tune_wbt_GAS_368_
	{0x0F12, 0x0011},	//Tune_wbt_GAS_369_
	{0x0F12, 0x000B},	//Tune_wbt_GAS_370_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_371_
	{0x0F12, 0x0024},	//Tune_wbt_GAS_372_
	{0x0F12, 0x003D},	//Tune_wbt_GAS_373_
	{0x0F12, 0x0061},	//Tune_wbt_GAS_374_
	{0x0F12, 0x008D},	//Tune_wbt_GAS_375_
	{0x0F12, 0x00B1},	//Tune_wbt_GAS_376_
	{0x0F12, 0x00B1},	//Tune_wbt_GAS_377_
	{0x0F12, 0x008D},	//Tune_wbt_GAS_378_
	{0x0F12, 0x0065},	//Tune_wbt_GAS_379_
	{0x0F12, 0x0041},	//Tune_wbt_GAS_380_
	{0x0F12, 0x002A},	//Tune_wbt_GAS_381_
	{0x0F12, 0x0019},	//Tune_wbt_GAS_382_
	{0x0F12, 0x0016},	//Tune_wbt_GAS_383_
	{0x0F12, 0x001B},	//Tune_wbt_GAS_384_
	{0x0F12, 0x002B},	//Tune_wbt_GAS_385_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_386_
	{0x0F12, 0x006A},	//Tune_wbt_GAS_387_
	{0x0F12, 0x0093},	//Tune_wbt_GAS_388_
	{0x0F12, 0x00B9},	//Tune_wbt_GAS_389_
	{0x0F12, 0x00C5},	//Tune_wbt_GAS_390_
	{0x0F12, 0x009F},	//Tune_wbt_GAS_391_
	{0x0F12, 0x007C},	//Tune_wbt_GAS_392_
	{0x0F12, 0x0059},	//Tune_wbt_GAS_393_
	{0x0F12, 0x0040},	//Tune_wbt_GAS_394_
	{0x0F12, 0x0033},	//Tune_wbt_GAS_395_
	{0x0F12, 0x002D},	//Tune_wbt_GAS_396_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_397_
	{0x0F12, 0x0044},	//Tune_wbt_GAS_398_
	{0x0F12, 0x005B},	//Tune_wbt_GAS_399_
	{0x0F12, 0x0078},	//Tune_wbt_GAS_400_
	{0x0F12, 0x00A4},	//Tune_wbt_GAS_401_
	{0x0F12, 0x00CE},	//Tune_wbt_GAS_402_
	{0x0F12, 0x00E2},	//Tune_wbt_GAS_403_
	{0x0F12, 0x00BB},	//Tune_wbt_GAS_404_
	{0x0F12, 0x009A},	//Tune_wbt_GAS_405_
	{0x0F12, 0x007B},	//Tune_wbt_GAS_406_
	{0x0F12, 0x0063},	//Tune_wbt_GAS_407_
	{0x0F12, 0x0054},	//Tune_wbt_GAS_408_
	{0x0F12, 0x0050},	//Tune_wbt_GAS_409_
	{0x0F12, 0x0056},	//Tune_wbt_GAS_410_
	{0x0F12, 0x0066},	//Tune_wbt_GAS_411_
	{0x0F12, 0x007F},	//Tune_wbt_GAS_412_
	{0x0F12, 0x009B},	//Tune_wbt_GAS_413_
	{0x0F12, 0x00C5},	//Tune_wbt_GAS_414_
	{0x0F12, 0x0104},	//Tune_wbt_GAS_415_
	{0x0F12, 0x00FB},	//Tune_wbt_GAS_416_
	{0x0F12, 0x00CD},	//Tune_wbt_GAS_417_
	{0x0F12, 0x00AB},	//Tune_wbt_GAS_418_
	{0x0F12, 0x0091},	//Tune_wbt_GAS_419_
	{0x0F12, 0x007C},	//Tune_wbt_GAS_420_
	{0x0F12, 0x006E},	//Tune_wbt_GAS_421_
	{0x0F12, 0x006A},	//Tune_wbt_GAS_422_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_423_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_424_
	{0x0F12, 0x0093},	//Tune_wbt_GAS_425_
	{0x0F12, 0x00B0},	//Tune_wbt_GAS_426_
	{0x0F12, 0x00E9},	//Tune_wbt_GAS_427_
	{0x0F12, 0x014D},	//Tune_wbt_GAS_428_
	{0x0F12, 0x00A2},	//Tune_wbt_GAS_429_
	{0x0F12, 0x0081},	//Tune_wbt_GAS_430_
	{0x0F12, 0x006A},	//Tune_wbt_GAS_431_
	{0x0F12, 0x0056},	//Tune_wbt_GAS_432_
	{0x0F12, 0x0049},	//Tune_wbt_GAS_433_
	{0x0F12, 0x003C},	//Tune_wbt_GAS_434_
	{0x0F12, 0x0040},	//Tune_wbt_GAS_435_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_436_
	{0x0F12, 0x0056},	//Tune_wbt_GAS_437_
	{0x0F12, 0x006D},	//Tune_wbt_GAS_438_
	{0x0F12, 0x0085},	//Tune_wbt_GAS_439_
	{0x0F12, 0x00A5},	//Tune_wbt_GAS_440_
	{0x0F12, 0x00D0},	//Tune_wbt_GAS_441_
	{0x0F12, 0x009C},	//Tune_wbt_GAS_442_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_443_
	{0x0F12, 0x005F},	//Tune_wbt_GAS_444_
	{0x0F12, 0x0044},	//Tune_wbt_GAS_445_
	{0x0F12, 0x0033},	//Tune_wbt_GAS_446_
	{0x0F12, 0x0027},	//Tune_wbt_GAS_447_
	{0x0F12, 0x0021},	//Tune_wbt_GAS_448_
	{0x0F12, 0x002B},	//Tune_wbt_GAS_449_
	{0x0F12, 0x003B},	//Tune_wbt_GAS_450_
	{0x0F12, 0x0052},	//Tune_wbt_GAS_451_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_452_
	{0x0F12, 0x008E},	//Tune_wbt_GAS_453_
	{0x0F12, 0x00B4},	//Tune_wbt_GAS_454_
	{0x0F12, 0x008A},	//Tune_wbt_GAS_455_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_456_
	{0x0F12, 0x004F},	//Tune_wbt_GAS_457_
	{0x0F12, 0x0034},	//Tune_wbt_GAS_458_
	{0x0F12, 0x0020},	//Tune_wbt_GAS_459_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_460_
	{0x0F12, 0x000C},	//Tune_wbt_GAS_461_
	{0x0F12, 0x0014},	//Tune_wbt_GAS_462_
	{0x0F12, 0x0022},	//Tune_wbt_GAS_463_
	{0x0F12, 0x0039},	//Tune_wbt_GAS_464_
	{0x0F12, 0x0055},	//Tune_wbt_GAS_465_
	{0x0F12, 0x0076},	//Tune_wbt_GAS_466_
	{0x0F12, 0x0093},	//Tune_wbt_GAS_467_
	{0x0F12, 0x0088},	//Tune_wbt_GAS_468_
	{0x0F12, 0x006C},	//Tune_wbt_GAS_469_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_470_
	{0x0F12, 0x002C},	//Tune_wbt_GAS_471_
	{0x0F12, 0x0019},	//Tune_wbt_GAS_472_
	{0x0F12, 0x000D},	//Tune_wbt_GAS_473_
	{0x0F12, 0x0006},	//Tune_wbt_GAS_474_
	{0x0F12, 0x000F},	//Tune_wbt_GAS_475_
	{0x0F12, 0x001C},	//Tune_wbt_GAS_476_
	{0x0F12, 0x0031},	//Tune_wbt_GAS_477_
	{0x0F12, 0x004F},	//Tune_wbt_GAS_478_
	{0x0F12, 0x0073},	//Tune_wbt_GAS_479_
	{0x0F12, 0x0090},	//Tune_wbt_GAS_480_
	{0x0F12, 0x0080},	//Tune_wbt_GAS_481_
	{0x0F12, 0x0062},	//Tune_wbt_GAS_482_
	{0x0F12, 0x003E},	//Tune_wbt_GAS_483_
	{0x0F12, 0x0022},	//Tune_wbt_GAS_484_
	{0x0F12, 0x0010},	//Tune_wbt_GAS_485_
	{0x0F12, 0x0006},	//Tune_wbt_GAS_486_
	{0x0F12, 0x0002},	//Tune_wbt_GAS_487_
	{0x0F12, 0x0002},	//Tune_wbt_GAS_488_
	{0x0F12, 0x0014},	//Tune_wbt_GAS_489_
	{0x0F12, 0x0024},	//Tune_wbt_GAS_490_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_491_
	{0x0F12, 0x006C},	//Tune_wbt_GAS_492_
	{0x0F12, 0x0089},	//Tune_wbt_GAS_493_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_494_
	{0x0F12, 0x005F},	//Tune_wbt_GAS_495_
	{0x0F12, 0x003D},	//Tune_wbt_GAS_496_
	{0x0F12, 0x0020},	//Tune_wbt_GAS_497_
	{0x0F12, 0x000D},	//Tune_wbt_GAS_498_
	{0x0F12, 0x0003},	//Tune_wbt_GAS_499_
	{0x0F12, 0x0000},	//Tune_wbt_GAS_500_
	{0x0F12, 0x0001},	//Tune_wbt_GAS_501_
	{0x0F12, 0x0012},	//Tune_wbt_GAS_502_
	{0x0F12, 0x0023},	//Tune_wbt_GAS_503_
	{0x0F12, 0x0048},	//Tune_wbt_GAS_504_
	{0x0F12, 0x006E},	//Tune_wbt_GAS_505_
	{0x0F12, 0x008B},	//Tune_wbt_GAS_506_
	{0x0F12, 0x0087},	//Tune_wbt_GAS_507_
	{0x0F12, 0x0068},	//Tune_wbt_GAS_508_
	{0x0F12, 0x0046},	//Tune_wbt_GAS_509_
	{0x0F12, 0x0029},	//Tune_wbt_GAS_510_
	{0x0F12, 0x0016},	//Tune_wbt_GAS_511_
	{0x0F12, 0x000A},	//Tune_wbt_GAS_512_
	{0x0F12, 0x0006},	//Tune_wbt_GAS_513_
	{0x0F12, 0x000D},	//Tune_wbt_GAS_514_
	{0x0F12, 0x001C},	//Tune_wbt_GAS_515_
	{0x0F12, 0x0031},	//Tune_wbt_GAS_516_
	{0x0F12, 0x0052},	//Tune_wbt_GAS_517_
	{0x0F12, 0x0078},	//Tune_wbt_GAS_518_
	{0x0F12, 0x0097},	//Tune_wbt_GAS_519_
	{0x0F12, 0x008B},	//Tune_wbt_GAS_520_
	{0x0F12, 0x006D},	//Tune_wbt_GAS_521_
	{0x0F12, 0x004C},	//Tune_wbt_GAS_522_
	{0x0F12, 0x0030},	//Tune_wbt_GAS_523_
	{0x0F12, 0x001D},	//Tune_wbt_GAS_524_
	{0x0F12, 0x000F},	//Tune_wbt_GAS_525_
	{0x0F12, 0x000E},	//Tune_wbt_GAS_526_
	{0x0F12, 0x0013},	//Tune_wbt_GAS_527_
	{0x0F12, 0x0022},	//Tune_wbt_GAS_528_
	{0x0F12, 0x003A},	//Tune_wbt_GAS_529_
	{0x0F12, 0x0058},	//Tune_wbt_GAS_530_
	{0x0F12, 0x007E},	//Tune_wbt_GAS_531_
	{0x0F12, 0x009E},	//Tune_wbt_GAS_532_
	{0x0F12, 0x009A},	//Tune_wbt_GAS_533_
	{0x0F12, 0x007F},	//Tune_wbt_GAS_534_
	{0x0F12, 0x0060},	//Tune_wbt_GAS_535_
	{0x0F12, 0x0042},	//Tune_wbt_GAS_536_
	{0x0F12, 0x002F},	//Tune_wbt_GAS_537_
	{0x0F12, 0x0024},	//Tune_wbt_GAS_538_
	{0x0F12, 0x0021},	//Tune_wbt_GAS_539_
	{0x0F12, 0x0027},	//Tune_wbt_GAS_540_
	{0x0F12, 0x0036},	//Tune_wbt_GAS_541_
	{0x0F12, 0x004B},	//Tune_wbt_GAS_542_
	{0x0F12, 0x0066},	//Tune_wbt_GAS_543_
	{0x0F12, 0x008D},	//Tune_wbt_GAS_544_
	{0x0F12, 0x00B0},	//Tune_wbt_GAS_545_
	{0x0F12, 0x00B6},	//Tune_wbt_GAS_546_
	{0x0F12, 0x0098},	//Tune_wbt_GAS_547_
	{0x0F12, 0x007D},	//Tune_wbt_GAS_548_
	{0x0F12, 0x0062},	//Tune_wbt_GAS_549_
	{0x0F12, 0x004E},	//Tune_wbt_GAS_550_
	{0x0F12, 0x0041},	//Tune_wbt_GAS_551_
	{0x0F12, 0x003F},	//Tune_wbt_GAS_552_
	{0x0F12, 0x0045},	//Tune_wbt_GAS_553_
	{0x0F12, 0x0053},	//Tune_wbt_GAS_554_
	{0x0F12, 0x006A},	//Tune_wbt_GAS_555_
	{0x0F12, 0x0084},	//Tune_wbt_GAS_556_
	{0x0F12, 0x00AA},	//Tune_wbt_GAS_557_
	{0x0F12, 0x00DE},	//Tune_wbt_GAS_558_
	{0x0F12, 0x00C5},	//Tune_wbt_GAS_559_
	{0x0F12, 0x00A0},	//Tune_wbt_GAS_560_
	{0x0F12, 0x0086},	//Tune_wbt_GAS_561_
	{0x0F12, 0x006F},	//Tune_wbt_GAS_562_
	{0x0F12, 0x005D},	//Tune_wbt_GAS_563_
	{0x0F12, 0x0052},	//Tune_wbt_GAS_564_
	{0x0F12, 0x004F},	//Tune_wbt_GAS_565_
	{0x0F12, 0x0054},	//Tune_wbt_GAS_566_
	{0x0F12, 0x0061},	//Tune_wbt_GAS_567_
	{0x0F12, 0x0075},	//Tune_wbt_GAS_568_
	{0x0F12, 0x008F},	//Tune_wbt_GAS_569_
	{0x0F12, 0x00BF},	//Tune_wbt_GAS_570_
	{0x0F12, 0x0113},	//Tune_wbt_GAS_571_
	{0x002A, 0x1348},
	{0x0F12, 0x0001},	//gisp_gras_Enable

//==================================================================================
// 07. Analog Setting 2
//==================================================================================
//This register is for FACTORY ONLY.
//If you change it without prior notification
//YOU are RESPONSIBLE for the FAILURE that will happen in the future
	{0x002A, 0x1278},
	{0x0F12, 0xAAF0},	//gisp_dadlc_config //Ladlc mode average
	{0x002A, 0x3370},
	{0x0F12, 0x0000},	//afit_bUseNormBrForAfit //0:Noise Index, 1:Normal Brightness

//==================================================================================
// 08.AF Setting
//==================================================================================

//==================================================================================
// 09.AWB-BASIC setting
//==================================================================================
//For WB Calibration
	{0x002A, 0x0D2A},
	{0x0F12, 0x05D0},	//r
	{0x0F12, 0x0400},	//g
	{0x0F12, 0x07C9},	//b

	{0x002A, 0x0B36},
	{0x0F12, 0x0005},	//awbb_IndoorGrZones_ZInfo_m_GridStep
	{0x002A, 0x0B3A},
	{0x0F12, 0x00EB},	//awbb_IndoorGrZones_ZInfo_m_BMin
	{0x0F12, 0x030B},	//awbb_IndoorGrZones_ZInfo_m_BMax
	{0x002A, 0x0B38},
	{0x0F12, 0x0012},	//awbb_IndoorGrZones_ZInfo_m_GridSz
	{0x002A, 0x0AE6},
	{0x0F12, 0x03BC},	//03BC	//awbb_IndoorGrZones_m_BGrid_0__m_left
	{0x0F12, 0x03E8},	//03E8	//awbb_IndoorGrZones_m_BGrid_0__m_right
	{0x0F12, 0x038B},	//037A	//awbb_IndoorGrZones_m_BGrid_1__m_left
	{0x0F12, 0x0403},	//0403	//awbb_IndoorGrZones_m_BGrid_1__m_right
	{0x0F12, 0x0359},	//034B	//awbb_IndoorGrZones_m_BGrid_2__m_left
	{0x0F12, 0x03FC},	//03FC	//awbb_IndoorGrZones_m_BGrid_2__m_right
	{0x0F12, 0x0309},	//02F4	//
	{0x0F12, 0x03D4},	//03D4	//awbb_IndoorGrZones_m_BGrid_3__m_right
	{0x0F12, 0x0294},	//0294	//awbb_IndoorGrZones_m_BGrid_4__m_left
	{0x0F12, 0x0361},	//0391	//awbb_IndoorGrZones_m_BGrid_4__m_right
	{0x0F12, 0x026D},	//026D	//awbb_IndoorGrZones_m_BGrid_5__m_left
	{0x0F12, 0x032F},	//0359	//awbb_IndoorGrZones_m_BGrid_5__m_right
	{0x0F12, 0x025A},	//025A	//awbb_IndoorGrZones_m_BGrid_6__m_left
	{0x0F12, 0x0309},	//032C	//awbb_IndoorGrZones_m_BGrid_6__m_right
	{0x0F12, 0x0243},	//0243	//awbb_IndoorGrZones_m_BGrid_7__m_left
	{0x0F12, 0x02ED},	//030C	//awbb_IndoorGrZones_m_BGrid_7__m_right
	{0x0F12, 0x0232},	//0232	//awbb_IndoorGrZones_m_BGrid_8__m_left
	{0x0F12, 0x02EA},	//02FC	//awbb_IndoorGrZones_m_BGrid_8__m_right
	{0x0F12, 0x021A},	//021A	//awbb_IndoorGrZones_m_BGrid_9__m_left
	{0x0F12, 0x02E1},	//02E8	//awbb_IndoorGrZones_m_BGrid_9__m_right
	{0x0F12, 0x01FF},	//01FF	//awbb_IndoorGrZones_m_BGrid_10__m_left
	{0x0F12, 0x02D7},	//02D7	//awbb_IndoorGrZones_m_BGrid_10__m_right
	{0x0F12, 0x01EF},	//01EF	//awbb_IndoorGrZones_m_BGrid_11__m_left
	{0x0F12, 0x02BD},	//02BD	//awbb_IndoorGrZones_m_BGrid_11__m_right
	{0x0F12, 0x01DD},	//01DD	//awbb_IndoorGrZones_m_BGrid_12__m_left
	{0x0F12, 0x02A7},	//02A7	//awbb_IndoorGrZones_m_BGrid_12__m_right
	{0x0F12, 0x01C7},	//01C7	//awbb_IndoorGrZones_m_BGrid_13__m_left
	{0x0F12, 0x028D},	//028D	//awbb_IndoorGrZones_m_BGrid_13__m_right
	{0x0F12, 0x01B9},	//01B9	//awbb_IndoorGrZones_m_BGrid_14__m_left
	{0x0F12, 0x0278},	//0278	//awbb_IndoorGrZones_m_BGrid_14__m_right
	{0x0F12, 0x01A7},	//01A7	//awbb_IndoorGrZones_m_BGrid_15__m_left
	{0x0F12, 0x025B},	//025B	//awbb_IndoorGrZones_m_BGrid_15__m_right
	{0x0F12, 0x01A9},	//01A9	//awbb_IndoorGrZones_m_BGrid_16__m_left
	{0x0F12, 0x023F},	//023F	//awbb_IndoorGrZones_m_BGrid_16__m_right
	{0x0F12, 0x01C9},	//01C9	//awbb_IndoorGrZones_m_BGrid_17__m_left
	{0x0F12, 0x01F3},	//01F3	//awbb_IndoorGrZones_m_BGrid_17__m_right
	{0x0F12, 0x0000},	//0000	//awbb_IndoorGrZones_m_BGrid_18__m_left
	{0x0F12, 0x0000},	//0000	//awbb_IndoorGrZones_m_BGrid_18__m_right
	{0x0F12, 0x0000},	//0000	//awbb_IndoorGrZones_m_BGrid_19__m_left
	{0x0F12, 0x0000},	//0000	//awbb_IndoorGrZones_m_BGrid_19__m_right
	{0x002A, 0x0BAA},
	{0x0F12, 0x0006},	//awbb_LowBrGrZones_ZInfo_m_GridStep
	{0x002A, 0x0BAE},
	{0x0F12, 0x009B},	//awbb_LowBrGrZones_ZInfo_m_BMin
	{0x0F12, 0x0336},	//awbb_LowBrGrZones_ZInfo_m_BMax
	{0x002A, 0x0BAC},
	{0x0F12, 0x000C},	//awbb_LowBrGrZones_ZInfo_m_GridSz
	{0x002A, 0x0B7A},
	{0x0F12, 0x040D},	//awbb_LowBrGrZones_m_BGrid_0__m_left
	{0x0F12, 0x0439},	//awbb_LowBrGrZones_m_BGrid_0__m_right
	{0x0F12, 0x0316},	//awbb_LowBrGrZones_m_BGrid_1__m_left
	{0x0F12, 0x0462},	//awbb_LowBrGrZones_m_BGrid_1__m_right
	{0x0F12, 0x0294},	//awbb_LowBrGrZones_m_BGrid_2__m_left
	{0x0F12, 0x0451},	//awbb_LowBrGrZones_m_BGrid_2__m_right
	{0x0F12, 0x025C},	//awbb_LowBrGrZones_m_BGrid_3__m_left
	{0x0F12, 0x0412},	//awbb_LowBrGrZones_m_BGrid_3__m_right
	{0x0F12, 0x022B},	//awbb_LowBrGrZones_m_BGrid_4__m_left
	{0x0F12, 0x03C1},	//awbb_LowBrGrZones_m_BGrid_4__m_right
	{0x0F12, 0x01FA},	//awbb_LowBrGrZones_m_BGrid_5__m_left
	{0x0F12, 0x0376},	//awbb_LowBrGrZones_m_BGrid_5__m_right
	{0x0F12, 0x01D3},	//awbb_LowBrGrZones_m_BGrid_6__m_left
	{0x0F12, 0x0329},	//awbb_LowBrGrZones_m_BGrid_6__m_right
	{0x0F12, 0x01B6},	//awbb_LowBrGrZones_m_BGrid_7__m_left
	{0x0F12, 0x02F4},	//awbb_LowBrGrZones_m_BGrid_7__m_right
	{0x0F12, 0x0198},	//awbb_LowBrGrZones_m_BGrid_8__m_left
	{0x0F12, 0x02CA},	//awbb_LowBrGrZones_m_BGrid_8__m_right
	{0x0F12, 0x0182},	//awbb_LowBrGrZones_m_BGrid_9__m_left
	{0x0F12, 0x028F},	//awbb_LowBrGrZones_m_BGrid_9__m_right
	{0x0F12, 0x017F},	//awbb_LowBrGrZones_m_BGrid_10__m_left
	{0x0F12, 0x0232},	//awbb_LowBrGrZones_m_BGrid_10__m_right
	{0x0F12, 0x01DF},	//awbb_LowBrGrZones_m_BGrid_11__m_left
	{0x0F12, 0x014C},	//awbb_LowBrGrZones_m_BGrid_11__m_right
	{0x002A, 0x0B70},
	{0x0F12, 0x0005},	//awbb_OutdoorGrZones_ZInfo_m_GridStep
	{0x002A, 0x0B74},
	{0x0F12, 0x01F8},	//awbb_OutdoorGrZones_ZInfo_m_BMin
	{0x0F12, 0x02AD},	//awbb_OutdoorGrZones_ZInfo_m_BMax
	{0x002A, 0x0B72},
	{0x0F12, 0x0007},	//awbb_OutdoorGrZones_ZInfo_m_GridSz
	{0x002A, 0x0B40},
	{0x0F12, 0x0287},	//awbb_OutdoorGrZones_m_BGrid_0__m_left
	{0x0F12, 0x02BB},	//awbb_OutdoorGrZones_m_BGrid_0__m_right
	{0x0F12, 0x025D},	//awbb_OutdoorGrZones_m_BGrid_1__m_left
	{0x0F12, 0x02BF},	//awbb_OutdoorGrZones_m_BGrid_1__m_right
	{0x0F12, 0x024B},	//awbb_OutdoorGrZones_m_BGrid_2__m_left
	{0x0F12, 0x02AC},	//awbb_OutdoorGrZones_m_BGrid_2__m_right
	{0x0F12, 0x0231},	//awbb_OutdoorGrZones_m_BGrid_3__m_left
	{0x0F12, 0x028E},	//awbb_OutdoorGrZones_m_BGrid_3__m_right
	{0x0F12, 0x0220},	//awbb_OutdoorGrZones_m_BGrid_4__m_left
	{0x0F12, 0x0275},	//awbb_OutdoorGrZones_m_BGrid_4__m_right
	{0x0F12, 0x021E},	//awbb_OutdoorGrZones_m_BGrid_5__m_left
	{0x0F12, 0x025D},	//awbb_OutdoorGrZones_m_BGrid_5__m_right
	{0x0F12, 0x023E},	//awbb_OutdoorGrZones_m_BGrid_6__m_left
	{0x0F12, 0x0249},	//awbb_OutdoorGrZones_m_BGrid_6__m_right
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_7__m_left
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_7__m_right
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_8__m_left
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_8__m_right
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_9__m_left
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_9__m_right
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_10__m_left
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_10__m_right
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_11__m_left
	{0x0F12, 0x0000},	//awbb_OutdoorGrZones_m_BGrid_11__m_right
	{0x002A, 0x0BC8},
	{0x0F12, 0x0005},	//awbb_CWSkinZone_ZInfo_m_GridStep
	{0x002A, 0x0BCC},
	{0x0F12, 0x010F},	//awbb_CWSkinZone_ZInfo_m_BMin
	{0x0F12, 0x018F},	//awbb_CWSkinZone_ZInfo_m_BMax
	{0x002A, 0x0BCA},
	{0x0F12, 0x0005},	//awbb_CWSkinZone_ZInfo_m_GridSz
	{0x002A, 0x0BB4},
	{0x0F12, 0x03E7},	//awbb_CWSkinZone_m_BGrid_0__m_left
	{0x0F12, 0x03F8},	//awbb_CWSkinZone_m_BGrid_0__m_right
	{0x0F12, 0x03A7},	//awbb_CWSkinZone_m_BGrid_1__m_left
	{0x0F12, 0x03FC},	//awbb_CWSkinZone_m_BGrid_1__m_right
	{0x0F12, 0x0352},	//awbb_CWSkinZone_m_BGrid_2__m_left
	{0x0F12, 0x03D0},	//awbb_CWSkinZone_m_BGrid_2__m_right
	{0x0F12, 0x0322},	//awbb_CWSkinZone_m_BGrid_3__m_left
	{0x0F12, 0x039E},	//awbb_CWSkinZone_m_BGrid_3__m_right
	{0x0F12, 0x032B},	//awbb_CWSkinZone_m_BGrid_4__m_left
	{0x0F12, 0x034D},	//awbb_CWSkinZone_m_BGrid_4__m_right
	{0x002A, 0x0BE6},
	{0x0F12, 0x0006},	//awbb_DLSkinZone_ZInfo_m_GridStep
	{0x002A, 0x0BEA},
	{0x0F12, 0x019E},	//awbb_DLSkinZone_ZInfo_m_BMin
	{0x0F12, 0x0257},	//awbb_DLSkinZone_ZInfo_m_BMax
	{0x002A, 0x0BE8},
	{0x0F12, 0x0004},	//awbb_DLSkinZone_ZInfo_m_GridSz
	{0x002A, 0x0BD2},
	{0x0F12, 0x030B},	//awbb_DLSkinZone_m_BGrid_0__m_left
	{0x0F12, 0x0323},	//awbb_DLSkinZone_m_BGrid_0__m_right
	{0x0F12, 0x02C3},	//awbb_DLSkinZone_m_BGrid_1__m_left
	{0x0F12, 0x030F},	//awbb_DLSkinZone_m_BGrid_1__m_right
	{0x0F12, 0x0288},	//awbb_DLSkinZone_m_BGrid_2__m_left
	{0x0F12, 0x02E5},	//awbb_DLSkinZone_m_BGrid_2__m_right
	{0x0F12, 0x026A},	//awbb_DLSkinZone_m_BGrid_3__m_left
	{0x0F12, 0x02A2},	//awbb_DLSkinZone_m_BGrid_3__m_right
	{0x0F12, 0x0000},	//awbb_DLSkinZone_m_BGrid_4__m_left
	{0x0F12, 0x0000},	//awbb_DLSkinZone_m_BGrid_4__m_right
	{0x002A, 0x0C2C},
	{0x0F12, 0x0130},	//0139	//awbb_IntcR
	{0x0F12, 0x011A},	//0122	//awbb_IntcB
	{0x002A, 0x0BFC},
	{0x0F12, 0x03AD},	//awbb_IndoorWP_0__r
	{0x0F12, 0x013F},	//awbb_IndoorWP_0__b
	{0x0F12, 0x0341},	//awbb_IndoorWP_1__r
	{0x0F12, 0x017B},	//awbb_IndoorWP_1__b
	{0x0F12, 0x038D},	//awbb_IndoorWP_2__r
	{0x0F12, 0x014B},	//awbb_IndoorWP_2__b
	{0x0F12, 0x02C3},	//awbb_IndoorWP_3__r
	{0x0F12, 0x01CC},	//awbb_IndoorWP_3__b
	{0x0F12, 0x0241},	//awbb_IndoorWP_4__r
	{0x0F12, 0x027F},	//awbb_IndoorWP_4__b
	{0x0F12, 0x0241},	//awbb_IndoorWP_5__r
	{0x0F12, 0x027F},	//awbb_IndoorWP_5__b
	{0x0F12, 0x0214},	//awbb_IndoorWP_6__r
	{0x0F12, 0x02A8},	//awbb_IndoorWP_6__b
	{0x0F12, 0x0255},	//awbb_OutdoorWP_r
	{0x0F12, 0x025B},	//awbb_OutdoorWP_b
	{0x002A, 0x0C4C},
	{0x0F12, 0x0452},	//awbb_MvEq_RBthresh
	{0x002A, 0x0C58},
	{0x0F12, 0x0BB0},	//awbb_MvEq_RBthresh
	{0x002A, 0x0BF8},
	{0x0F12, 0x07C8},	//awbb_LowTSep_m_RminusB
	{0x002A, 0x0C28},
	{0x0F12, 0x0000},	//awbb_SkinPreference
	{0x002A, 0x0CAC},
	{0x0F12, 0x0050},	//awbb_OutDMaxIncr
	{0x002A, 0x0C28},
	{0x0F12, 0x0000},	//awbb_SkinPreference
	{0x002A, 0x0D0E},
	{0x0F12, 0x00B8},	//awbb_GridCoeff_R_2
	{0x0F12, 0x00B2},	//awbb_GridCoeff_B_2
	{0x002A, 0x0CFE},
	{0x0F12, 0x0FAB},	//awbb_GridConst_2_0_
	{0x0F12, 0x0FF5},	//awbb_GridConst_2_1_
	{0x0F12, 0x10BB},	//awbb_GridConst_2_2_
	{0x0F12, 0x1153},	//awbb_GridConst_2_3_
	{0x0F12, 0x11C5},	//awbb_GridConst_2_4_
	{0x0F12, 0x122A},	//awbb_GridConst_2_5_
	{0x0F12, 0x00A9},	//awbb_GridCoeff_R_1
	{0x0F12, 0x00C0},	//awbb_GridCoeff_B_1
	{0x002A, 0x0CF8},
	{0x0F12, 0x0312},	//awbb_GridConst_1_0_
	{0x0F12, 0x0350},	//awbb_GridConst_1_1_
	{0x0F12, 0x038C},	//awbb_GridConst_1_2_
	{0x002A, 0x0CB0},
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_0__0_
	{0x0F12, 0x0050},	//0050	//awbb_GridCorr_R_0__1_
	{0x0F12, 0x0064},	//0028	//awbb_GridCorr_R_0__2_
	{0x0F12, 0x0032},	//0014	//awbb_GridCorr_R_0__3_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_0__4_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_0__5_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_1__0_
	{0x0F12, 0x0050},	//0050	//awbb_GridCorr_R_1__1_
	{0x0F12, 0x0064},	//0028	//awbb_GridCorr_R_1__2_
	{0x0F12, 0x0032},	//0014	//awbb_GridCorr_R_1__3_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_1__4_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_1__5_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_2__0_
	{0x0F12, 0x0050},	//0050	//awbb_GridCorr_R_2__1_
	{0x0F12, 0x0064},	//0028	//awbb_GridCorr_R_2__2_
	{0x0F12, 0x0032},	//0014	//awbb_GridCorr_R_2__3_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_2__4_
	{0x0F12, 0x0032},	//0000	//awbb_GridCorr_R_2__5_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_0__0_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_0__1_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_0__2_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_0__3_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_0__4_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_0__5_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_1__0_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_1__1_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_1__2_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_1__3_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_1__4_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_1__5_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_2__0_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_2__1_
	{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_2__2_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_2__3_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_2__4_
	{0x0F12, 0xFF9C},	//FFC0	//awbb_GridCorr_B_2__5_
	{0x002A, 0x0D30},
	{0x0F12, 0x0002},	//awbb_GridEnable
	{0x002A, 0x3372},
	{0x0F12, 0x0001},	//awbb_bUseOutdoorGrid
	{0x0F12, 0x0000},	//awbb_OutdoorGridCorr_R
	{0x0F12, 0x0000},	//awbb_OutdoorGridCorr_B
//For Outdoor Detector
	{0x002A, 0x0C86},
	{0x0F12, 0x0005},	//awbb_OutdoorDetectionZone_ZInfo_m_GridSz
	{0x002A, 0x0C70},
	{0x0F12, 0xFF7B},	//awbb_OutdoorDetectionZone_m_BGrid_0__m_left
	{0x0F12, 0x012C},	//00CE	//awbb_OutdoorDetectionZone_m_BGrid_0__m_right
	{0x0F12, 0xFF23},	//awbb_OutdoorDetectionZone_m_BGrid_1__m_left
	{0x0F12, 0x016B},	//010D	//awbb_OutdoorDetectionZone_m_BGrid_1__m_right
	{0x0F12, 0xFEF3},	//awbb_OutdoorDetectionZone_m_BGrid_2__m_left
	{0x0F12, 0x018A},	//012C	//awbb_OutdoorDetectionZone_m_BGrid_2__m_right
	{0x0F12, 0xFED7},	//awbb_OutdoorDetectionZone_m_BGrid_3__m_left
	{0x0F12, 0x01AC},	//014E	//awbb_OutdoorDetectionZone_m_BGrid_3__m_right
	{0x0F12, 0xFEBB},	//awbb_OutdoorDetectionZone_m_BGrid_4__m_left
	{0x0F12, 0x01C0},	//0162	//awbb_OutdoorDetectionZone_m_BGrid_4__m_right
	{0x0F12, 0x1388},	//awbb_OutdoorDetectionZone_ZInfo_m_AbsGridStep
	{0x002A, 0x0C8A},
	{0x0F12, 0x1F40},	//awbb_OutdoorDetectionZone_ZInfo_m_MaxNB
	{0x002A, 0x0C88},
	{0x0F12, 0x0A7C},	//awbb_OutdoorDetectionZone_ZInfo_m_NBoffs

	{0x002A, 0x0CA6},
	{0x0F12, 0x0004},	//awbb_GainsMaxMove
	{0x002A, 0x0D26},
	{0x0F12, 0x0061},	//awbb_Use_Filters
	{0x002A, 0x0CA0},
	{0x0F12, 0x001E},	//awbb_GnCurPntImmunity
	{0x002A, 0x0BF4},
	{0x0F12, 0x0005},	//awbb_LowBrYThresh_y_low

//==================================================================================
// 10.Clock Setting
//==================================================================================
//Input Clock (Mclk)
	{0x002A, 0x012E},
	{0x0F12, 0x427D},	//REG_TC_IPRM_InClockLSBs		// 17.021277MHz
	{0x0F12, 0x0000},	//REG_TC_IPRM_InClockMSBs
	{0x002A, 0x0146},
	{0x0F12, 0x0000},	//REG_TC_IPRM_UseNPviClocks
	{0x0F12, 0x0001},	//REG_TC_IPRM_UseNMipiClocks
//System Clock & Output clock (Pclk)
	{0x002A, 0x014C},
#if defined(CONFIG_MACH_LYNX_DL45)
	{0x0F12, 0x2C0D},	//REG_TC_IPRM_OpClk4KHz_0		// 45.10638405MHz
#elif defined(CONFIG_MACH_DECKARD_AS87)
	{0x0F12, 0x2FCB},	//REG_TC_IPRM_OpClk4KHz_0		// 48.936171375MHz
#else
	{0x0F12, 0x319C},	//REG_TC_IPRM_OpClk4KHz_0		// 50.797873546875MHz
#endif
	{0x002A, 0x0152},
#if defined(CONFIG_MACH_LYNX_DL45)
	{0x0F12, 0x5818},	//REG_TC_IPRM_MinOutRate4KHz_0	// 90.2127681MHz
#elif defined(CONFIG_MACH_DECKARD_AS87)
	{0x0F12, 0x5F93},	//REG_TC_IPRM_MinOutRate4KHz_0	// 97.87234275MHz
#else
	{0x0F12, 0x6336},	//REG_TC_IPRM_MinOutRate4KHz_0	// 101.59574709375MHz
#endif
	{0x002A, 0x014E},
#if defined(CONFIG_MACH_LYNX_DL45)
	{0x0F12, 0x5819},	//REG_TC_IPRM_MaxOutRate4KHz_0
#elif defined(CONFIG_MACH_DECKARD_AS87)
	{0x0F12, 0x5F94},	//REG_TC_IPRM_MaxOutRate4KHz_0
#else
	{0x0F12, 0x6337},	//REG_TC_IPRM_MaxOutRate4KHz_0
#endif
	{0x002A, 0x0164},
	{0x0F12, 0x0001},	//REG_TC_IPRM_InitParamsUpdated

//==================================================================================
// 11.Auto Flicker Detection
//==================================================================================
//s002A03F4
//s0F120002	//REG_SF_USER_FlickerQuant
//s0F120001	//REG_SF_USER_FlickerQuantChanged

	{0x002A, 0x0ABC},
	{0x0F12, 0x0000},       //AFC_Default60Hz         0:50Hz start    1:60Hz start(default)
	{0x002A, 0x0408},
	{0x0F12, 0x067F},	//REG_TC_DBG_AutoAlgEnBits //all AA are on

//==================================================================================
// 12.AE Setting
//==================================================================================
	{0x002A, 0x0D40},
	{0x0F12, 0x003E}, // 3E TVAR_ae_BrAve

// For LT Calibration
	{0x002A, 0x0D46},
	{0x0F12, 0x000F},	// ae_StatMode

	{0x002A, 0x0440},
	{0x0F12, 0x3415},	// lt_uMaxExp_0_
	{0x002A, 0x0444},
	{0x0F12, 0x682A},	// lt_uMaxExp_1_
	{0x002A, 0x0448},
	{0x0F12, 0x9C40},	// lt_uMaxExp_2_
	{0x002A, 0x044C},
	{0x0F12, 0xD055},	// lt_uMaxExp_3_
	{0x002A, 0x0450},
	{0x0F12, 0x3410},	// lt_uCapMaxExp_0_
	{0x002A, 0x0454},
	{0x0F12, 0x6820},	// lt_uCapMaxExp_1_
	{0x002A, 0x0458},
	{0x0F12, 0x8227},	// lt_uCapMaxExp_2_
	{0x002A, 0x045C},
	{0x0F12, 0xC350},	// lt_uCapMaxExp_3_
	{0x002A, 0x0460},
	{0x0F12, 0x0400},	// lt_uMaxAnGain_0_
	{0x0F12, 0x0400},	// lt_uMaxAnGain_1_
	{0x0F12, 0x0400},	// lt_uMaxAnGain_2_
	{0x0F12, 0x0800},	// lt_uMaxAnGain_3_
	{0x0F12, 0x0100},	// lt_uMaxDigGain
	{0x0F12, 0x0800},	// lt_uMaxTotGain
	{0x002A, 0x042E},
	{0x0F12, 0x010E},	// lt_uLimitHigh
	{0x0F12, 0x00F5},	// lt_uLimitLow
	{0x002A, 0x0DE0},
	{0x0F12, 0x0002},	// ae_Fade2BlackEnable F2B off, F2W on
//For Illum Type Calibration
//WRITE #SARR_IllumType_0_				 	  0078
//WRITE #SARR_IllumType_1_					  00C3
//WRITE #SARR_IllumType_2_					  00E9
//WRITE #SARR_IllumType_3_					  0128
//WRITE #SARR_IllumType_4_					  016F
//WRITE #SARR_IllumType_5_					  0195
//WRITE #SARR_IllumType_6_					  01A4
//WRITE #SARR_IllumTypeF_0_  					  0100
//WRITE #SARR_IllumTypeF_1_  					  0100
//WRITE #SARR_IllumTypeF_2_  					  0110
//WRITE #SARR_IllumTypeF_3_  					  00E5
//WRITE #SARR_IllumTypeF_4_  					  0100
//WRITE #SARR_IllumTypeF_5_  					  00ED
//WRITE #SARR_IllumTypeF_6_  					  00ED

//==================================================================================
// 13.AE Weight (Normal)
//==================================================================================
	{0x002A, 0x0D4E},
	{0x0F12, 0x0000},  //0000	//ae_WeightTbl_16_0_
	{0x0F12, 0x0101},  //0100	//ae_WeightTbl_16_1_
	{0x0F12, 0x0101},  //0001	//ae_WeightTbl_16_2_
	{0x0F12, 0x0000},  //0000	//ae_WeightTbl_16_3_
	{0x0F12, 0x0100},  //0000	//ae_WeightTbl_16_4_
	{0x0F12, 0x0101},  //0101	//ae_WeightTbl_16_5_
	{0x0F12, 0x0101},  //0101	//ae_WeightTbl_16_6_
	{0x0F12, 0x0001},  //0000	//ae_WeightTbl_16_7_
	{0x0F12, 0x0100},  //0100	//ae_WeightTbl_16_8_
	{0x0F12, 0x0201},  //0101	//ae_WeightTbl_16_9_
	{0x0F12, 0x0102},  //0101	//ae_WeightTbl_16_10_
	{0x0F12, 0x0001},  //0001	//ae_WeightTbl_16_11_
	{0x0F12, 0x0100},  //0100	//ae_WeightTbl_16_12_
	{0x0F12, 0x0302},  //0201	//ae_WeightTbl_16_13_
	{0x0F12, 0x0203},  //0102	//ae_WeightTbl_16_14_
	{0x0F12, 0x0001},  //0001	//ae_WeightTbl_16_15_
	{0x0F12, 0x0100},  //0100	//ae_WeightTbl_16_16_
	{0x0F12, 0x0302},  //0201	//ae_WeightTbl_16_17_
	{0x0F12, 0x0203},  //0102	//ae_WeightTbl_16_18_
	{0x0F12, 0x0001},  //0001	//ae_WeightTbl_16_19_
	{0x0F12, 0x0100},  //0100	//ae_WeightTbl_16_20_
	{0x0F12, 0x0201},  //0101	//ae_WeightTbl_16_21_
	{0x0F12, 0x0102},  //0101	//ae_WeightTbl_16_22_
	{0x0F12, 0x0001},  //0001	//ae_WeightTbl_16_23_
	{0x0F12, 0x0100},  //0000	//ae_WeightTbl_16_24_
	{0x0F12, 0x0101},  //0101	//ae_WeightTbl_16_25_
	{0x0F12, 0x0101},  //0101	//ae_WeightTbl_16_26_
	{0x0F12, 0x0001},  //0000	//ae_WeightTbl_16_27_
	{0x0F12, 0x0000},  //0000	//ae_WeightTbl_16_28_
	{0x0F12, 0x0101},  //0100	//ae_WeightTbl_16_29_
	{0x0F12, 0x0101},  //0001	//ae_WeightTbl_16_30_
	{0x0F12, 0x0000},  //0000	//ae_WeightTbl_16_31_

//==================================================================================
// 14.Flash Setting
//==================================================================================

//==================================================================================
// 15.CCM Setting
//==================================================================================
	{0x002A, 0x33A4},
	{0x0F12, 0x01D0},	// Tune_wbt_BaseCcms_0__0_
	{0x0F12, 0xFFA1},	// Tune_wbt_BaseCcms_0__1_
	{0x0F12, 0xFFFA},	// Tune_wbt_BaseCcms_0__2_
	{0x0F12, 0xFF6F},	// Tune_wbt_BaseCcms_0__3_
	{0x0F12, 0x0140},	// Tune_wbt_BaseCcms_0__4_
	{0x0F12, 0xFF49},	// Tune_wbt_BaseCcms_0__5_
	{0x0F12, 0xFFC1},	// Tune_wbt_BaseCcms_0__6_
	{0x0F12, 0x001F},	// Tune_wbt_BaseCcms_0__7_
	{0x0F12, 0x01BD},	// Tune_wbt_BaseCcms_0__8_
	{0x0F12, 0x013F},	// Tune_wbt_BaseCcms_0__9_
	{0x0F12, 0x00E1},	// Tune_wbt_BaseCcms_0__10_
	{0x0F12, 0xFF43},	// Tune_wbt_BaseCcms_0__11_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_0__12_
	{0x0F12, 0xFFC0},	// Tune_wbt_BaseCcms_0__13_
	{0x0F12, 0x01B7},	// Tune_wbt_BaseCcms_0__14_
	{0x0F12, 0xFF30},	// Tune_wbt_BaseCcms_0__15_
	{0x0F12, 0x015F},	// Tune_wbt_BaseCcms_0__16_
	{0x0F12, 0x0106},	// Tune_wbt_BaseCcms_0__17_
	{0x0F12, 0x01D0},	// Tune_wbt_BaseCcms_1__0_
	{0x0F12, 0xFFA1},	// Tune_wbt_BaseCcms_1__1_
	{0x0F12, 0xFFFA},	// Tune_wbt_BaseCcms_1__2_
	{0x0F12, 0xFF6F},	// Tune_wbt_BaseCcms_1__3_
	{0x0F12, 0x0140},	// Tune_wbt_BaseCcms_1__4_
	{0x0F12, 0xFF49},	// Tune_wbt_BaseCcms_1__5_
	{0x0F12, 0xFFC1},	// Tune_wbt_BaseCcms_1__6_
	{0x0F12, 0x001F},	// Tune_wbt_BaseCcms_1__7_
	{0x0F12, 0x01BD},	// Tune_wbt_BaseCcms_1__8_
	{0x0F12, 0x013F},	// Tune_wbt_BaseCcms_1__9_
	{0x0F12, 0x00E1},	// Tune_wbt_BaseCcms_1__10_
	{0x0F12, 0xFF43},	// Tune_wbt_BaseCcms_1__11_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_1__12_
	{0x0F12, 0xFFC0},	// Tune_wbt_BaseCcms_1__13_
	{0x0F12, 0x01B7},	// Tune_wbt_BaseCcms_1__14_
	{0x0F12, 0xFF30},	// Tune_wbt_BaseCcms_1__15_
	{0x0F12, 0x015F},	// Tune_wbt_BaseCcms_1__16_
	{0x0F12, 0x0106},	// Tune_wbt_BaseCcms_1__17_
	{0x0F12, 0x01D0},	// Tune_wbt_BaseCcms_2__0_
	{0x0F12, 0xFFA1},	// Tune_wbt_BaseCcms_2__1_
	{0x0F12, 0xFFFA},	// Tune_wbt_BaseCcms_2__2_
	{0x0F12, 0xFF6F},	// Tune_wbt_BaseCcms_2__3_
	{0x0F12, 0x0140},	// Tune_wbt_BaseCcms_2__4_
	{0x0F12, 0xFF49},	// Tune_wbt_BaseCcms_2__5_
	{0x0F12, 0xFFE3},	// FFC1 Tune_wbt_BaseCcms_2__6_
	{0x0F12, 0xFFF9},	// 001F Tune_wbt_BaseCcms_2__7_
	{0x0F12, 0x01C1},	// 01BD Tune_wbt_BaseCcms_2__8_
	{0x0F12, 0x013F},	// Tune_wbt_BaseCcms_2__9_
	{0x0F12, 0x00E1},	// Tune_wbt_BaseCcms_2__10_
	{0x0F12, 0xFF43},	// Tune_wbt_BaseCcms_2__11_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_2__12_
	{0x0F12, 0xFFC0},	// Tune_wbt_BaseCcms_2__13_
	{0x0F12, 0x01B7},	// Tune_wbt_BaseCcms_2__14_
	{0x0F12, 0xFF30},	// Tune_wbt_BaseCcms_2__15_
	{0x0F12, 0x015F},	// Tune_wbt_BaseCcms_2__16_
	{0x0F12, 0x0106},	// Tune_wbt_BaseCcms_2__17_
	{0x0F12, 0x01D0},	// Tune_wbt_BaseCcms_3__0_
	{0x0F12, 0xFFA1},	// Tune_wbt_BaseCcms_3__1_
	{0x0F12, 0xFFFA},	// Tune_wbt_BaseCcms_3__2_
	{0x0F12, 0xFF6F},	// Tune_wbt_BaseCcms_3__3_
	{0x0F12, 0x0140},	// Tune_wbt_BaseCcms_3__4_
	{0x0F12, 0xFF49},	// Tune_wbt_BaseCcms_3__5_
	{0x0F12, 0xFFE3},	// FFC1 Tune_wbt_BaseCcms_3__6_
	{0x0F12, 0xFFF9},	// 001F Tune_wbt_BaseCcms_3__7_
	{0x0F12, 0x01C1},	// 01BD Tune_wbt_BaseCcms_3__8_
	{0x0F12, 0x013F},	// Tune_wbt_BaseCcms_3__9_
	{0x0F12, 0x00E1},	// Tune_wbt_BaseCcms_3__10_
	{0x0F12, 0xFF43},	// Tune_wbt_BaseCcms_3__11_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_3__12_
	{0x0F12, 0xFFC0},	// Tune_wbt_BaseCcms_3__13_
	{0x0F12, 0x01B7},	// Tune_wbt_BaseCcms_3__14_
	{0x0F12, 0xFF30},	// Tune_wbt_BaseCcms_3__15_
	{0x0F12, 0x015F},	// Tune_wbt_BaseCcms_3__16_
	{0x0F12, 0x0106},	// Tune_wbt_BaseCcms_3__17_
	{0x0F12, 0x01BF},	// Tune_wbt_BaseCcms_4__0_
	{0x0F12, 0xFFBF},	// Tune_wbt_BaseCcms_4__1_
	{0x0F12, 0xFFFE},	// Tune_wbt_BaseCcms_4__2_
	{0x0F12, 0xFF6D},	// Tune_wbt_BaseCcms_4__3_
	{0x0F12, 0x01B4},	// Tune_wbt_BaseCcms_4__4_
	{0x0F12, 0xFF66},	// Tune_wbt_BaseCcms_4__5_
	{0x0F12, 0xFFCA},	// Tune_wbt_BaseCcms_4__6_
	{0x0F12, 0xFFCE},	// Tune_wbt_BaseCcms_4__7_
	{0x0F12, 0x017B},	// Tune_wbt_BaseCcms_4__8_
	{0x0F12, 0x0136},	// Tune_wbt_BaseCcms_4__9_
	{0x0F12, 0x0132},	// Tune_wbt_BaseCcms_4__10_
	{0x0F12, 0xFF85},	// Tune_wbt_BaseCcms_4__11_
	{0x0F12, 0x018B},	// Tune_wbt_BaseCcms_4__12_
	{0x0F12, 0xFF73},	// Tune_wbt_BaseCcms_4__13_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_4__14_
	{0x0F12, 0xFF3F},	// Tune_wbt_BaseCcms_4__15_
	{0x0F12, 0x015B},	// Tune_wbt_BaseCcms_4__16_
	{0x0F12, 0x00D0},	// Tune_wbt_BaseCcms_4__17_
	{0x0F12, 0x01BF},	// Tune_wbt_BaseCcms_5__0_
	{0x0F12, 0xFFBF},	// Tune_wbt_BaseCcms_5__1_
	{0x0F12, 0xFFFE},	// Tune_wbt_BaseCcms_5__2_
	{0x0F12, 0xFF6D},	// Tune_wbt_BaseCcms_5__3_
	{0x0F12, 0x01B4},	// Tune_wbt_BaseCcms_5__4_
	{0x0F12, 0xFF66},	// Tune_wbt_BaseCcms_5__5_
	{0x0F12, 0xFFCA},	// Tune_wbt_BaseCcms_5__6_
	{0x0F12, 0xFFCE},	// Tune_wbt_BaseCcms_5__7_
	{0x0F12, 0x017B},	// Tune_wbt_BaseCcms_5__8_
	{0x0F12, 0x0136},	// Tune_wbt_BaseCcms_5__9_
	{0x0F12, 0x0132},	// Tune_wbt_BaseCcms_5__10_
	{0x0F12, 0xFF85},	// Tune_wbt_BaseCcms_5__11_
	{0x0F12, 0x018B},	// Tune_wbt_BaseCcms_5__12_
	{0x0F12, 0xFF73},	// Tune_wbt_BaseCcms_5__13_
	{0x0F12, 0x0191},	// Tune_wbt_BaseCcms_5__14_
	{0x0F12, 0xFF3F},	// Tune_wbt_BaseCcms_5__15_
	{0x0F12, 0x015B},	// Tune_wbt_BaseCcms_5__16_
	{0x0F12, 0x00D0},	// Tune_wbt_BaseCcms_5__17_
	{0x002A, 0x3380},
	{0x0F12, 0x01AC},	// Tune_wbt_OutdoorCcm_0_
	{0x0F12, 0xFFD7},	// Tune_wbt_OutdoorCcm_1_
	{0x0F12, 0x0019},	// Tune_wbt_OutdoorCcm_2_
	{0x0F12, 0xFF49},	// Tune_wbt_OutdoorCcm_3_
	{0x0F12, 0x01D9},	// Tune_wbt_OutdoorCcm_4_
	{0x0F12, 0xFF63},	// Tune_wbt_OutdoorCcm_5_
	{0x0F12, 0xFFCA},	// Tune_wbt_OutdoorCcm_6_
	{0x0F12, 0xFFCE},	// Tune_wbt_OutdoorCcm_7_
	{0x0F12, 0x017B},	// Tune_wbt_OutdoorCcm_8_
	{0x0F12, 0x0132},	// Tune_wbt_OutdoorCcm_9_
	{0x0F12, 0x012E},	// Tune_wbt_OutdoorCcm_10_
	{0x0F12, 0xFF8D},	// Tune_wbt_OutdoorCcm_11_
	{0x0F12, 0x018B},	// Tune_wbt_OutdoorCcm_12_
	{0x0F12, 0xFF73},	// Tune_wbt_OutdoorCcm_13_
	{0x0F12, 0x0191},	// Tune_wbt_OutdoorCcm_14_
	{0x0F12, 0xFF3F},	// Tune_wbt_OutdoorCcm_15_
	{0x0F12, 0x015B},	// Tune_wbt_OutdoorCcm_16_
	{0x0F12, 0x00D0},	// Tune_wbt_OutdoorCcm_17_
	{0x002A, 0x0612},
	{0x0F12, 0x009D},	// SARR_AwbCcmCord_0_
	{0x0F12, 0x00D5},	// SARR_AwbCcmCord_1_
	{0x0F12, 0x0103},	// SARR_AwbCcmCord_2_
	{0x0F12, 0x0128},	// SARR_AwbCcmCord_3_
	{0x0F12, 0x0166},	// SARR_AwbCcmCord_4_
	{0x0F12, 0x0193},	// SARR_AwbCcmCord_5_

//==================================================================================
// 16.GAMMA
//==================================================================================
//For Pre Post Gamma Calibration
	{0x002A, 0x0538},
	{0x0F12, 0x0000},	//seti_uGammaLutPreDemNoBin_0_
	{0x0F12, 0x001F},	//seti_uGammaLutPreDemNoBin_1_
	{0x0F12, 0x0035},	//seti_uGammaLutPreDemNoBin_2_
	{0x0F12, 0x005A},	//seti_uGammaLutPreDemNoBin_3_
	{0x0F12, 0x0095},	//seti_uGammaLutPreDemNoBin_4_
	{0x0F12, 0x00E6},	//seti_uGammaLutPreDemNoBin_5_
	{0x0F12, 0x0121},	//seti_uGammaLutPreDemNoBin_6_
	{0x0F12, 0x0139},	//seti_uGammaLutPreDemNoBin_7_
	{0x0F12, 0x0150},	//seti_uGammaLutPreDemNoBin_8_
	{0x0F12, 0x0177},	//seti_uGammaLutPreDemNoBin_9_
	{0x0F12, 0x019A},	//seti_uGammaLutPreDemNoBin_10_
	{0x0F12, 0x01BB},	//seti_uGammaLutPreDemNoBin_11_
	{0x0F12, 0x01DC},	//seti_uGammaLutPreDemNoBin_12_
	{0x0F12, 0x0219},	//seti_uGammaLutPreDemNoBin_13_
	{0x0F12, 0x0251},	//seti_uGammaLutPreDemNoBin_14_
	{0x0F12, 0x02B3},	//seti_uGammaLutPreDemNoBin_15_
	{0x0F12, 0x030A},	//seti_uGammaLutPreDemNoBin_16_
	{0x0F12, 0x035F},	//seti_uGammaLutPreDemNoBin_17_
	{0x0F12, 0x03B1},	//seti_uGammaLutPreDemNoBin_18_
	{0x0F12, 0x03FF},	//seti_uGammaLutPreDemNoBin_19_
	{0x0F12, 0x0000},	//seti_uGammaLutPostDemNoBin_0_
	{0x0F12, 0x0001},	//seti_uGammaLutPostDemNoBin_1_
	{0x0F12, 0x0001},	//seti_uGammaLutPostDemNoBin_2_
	{0x0F12, 0x0002},	//seti_uGammaLutPostDemNoBin_3_
	{0x0F12, 0x0004},	//seti_uGammaLutPostDemNoBin_4_
	{0x0F12, 0x000A},	//seti_uGammaLutPostDemNoBin_5_
	{0x0F12, 0x0012},	//seti_uGammaLutPostDemNoBin_6_
	{0x0F12, 0x0016},	//seti_uGammaLutPostDemNoBin_7_
	{0x0F12, 0x001A},	//seti_uGammaLutPostDemNoBin_8_
	{0x0F12, 0x0024},	//seti_uGammaLutPostDemNoBin_9_
	{0x0F12, 0x0031},	//seti_uGammaLutPostDemNoBin_10_
	{0x0F12, 0x003E},	//seti_uGammaLutPostDemNoBin_11_
	{0x0F12, 0x004E},	//seti_uGammaLutPostDemNoBin_12_
	{0x0F12, 0x0075},	//seti_uGammaLutPostDemNoBin_13_
	{0x0F12, 0x00A8},	//seti_uGammaLutPostDemNoBin_14_
	{0x0F12, 0x0126},	//seti_uGammaLutPostDemNoBin_15_
	{0x0F12, 0x01BE},	//seti_uGammaLutPostDemNoBin_16_
	{0x0F12, 0x0272},	//seti_uGammaLutPostDemNoBin_17_
	{0x0F12, 0x0334},	//seti_uGammaLutPostDemNoBin_18_
	{0x0F12, 0x03FF},	//seti_uGammaLutPostDemNoBin_19_
//For Gamma Calibration
	{0x002A, 0x0498},
	{0x0F12, 0x0000},	//0000	//SARR_usDualGammaLutRGBIndoor_0__0_
	{0x0F12, 0x0003},	//0004	//SARR_usDualGammaLutRGBIndoor_0__1_
	{0x0F12, 0x000A},	//000C	//SARR_usDualGammaLutRGBIndoor_0__2_
	{0x0F12, 0x001C},	//0024	//SARR_usDualGammaLutRGBIndoor_0__3_
	{0x0F12, 0x0046},	//007E	//SARR_usDualGammaLutRGBIndoor_0__4_
	{0x0F12, 0x00A7},	//00F5	//SARR_usDualGammaLutRGBIndoor_0__5_
	{0x0F12, 0x010B},	//0142	//SARR_usDualGammaLutRGBIndoor_0__6_
	{0x0F12, 0x013A},	//0166	//SARR_usDualGammaLutRGBIndoor_0__7_
	{0x0F12, 0x0163},	//0186	//SARR_usDualGammaLutRGBIndoor_0__8_
	{0x0F12, 0x019D},	//01BE	//SARR_usDualGammaLutRGBIndoor_0__9_
	{0x0F12, 0x01CD},	//01EF	//SARR_usDualGammaLutRGBIndoor_0__10_
	{0x0F12, 0x01F8},	//021B	//SARR_usDualGammaLutRGBIndoor_0__11_
	{0x0F12, 0x021E},	//0244	//SARR_usDualGammaLutRGBIndoor_0__12_
	{0x0F12, 0x0261},	//028A	//SARR_usDualGammaLutRGBIndoor_0__13_
	{0x0F12, 0x02A1},	//02C2	//SARR_usDualGammaLutRGBIndoor_0__14_
	{0x0F12, 0x0304},	//0318	//SARR_usDualGammaLutRGBIndoor_0__15_
	{0x0F12, 0x0360},	//0360	//SARR_usDualGammaLutRGBIndoor_0__16_
	{0x0F12, 0x039D},	//039D	//SARR_usDualGammaLutRGBIndoor_0__17_
	{0x0F12, 0x03D3},	//03D3	//SARR_usDualGammaLutRGBIndoor_0__18_
	{0x0F12, 0x03FF},	//03FF	//SARR_usDualGammaLutRGBIndoor_0__19_
	{0x0F12, 0x0000},	//0000	//SARR_usDualGammaLutRGBOutdoor_0__0_
	{0x0F12, 0x0003},	//0004	//SARR_usDualGammaLutRGBOutdoor_0__1_
	{0x0F12, 0x000A},	//000C	//SARR_usDualGammaLutRGBOutdoor_0__2_
	{0x0F12, 0x001C},	//0024	//SARR_usDualGammaLutRGBOutdoor_0__3_
	{0x0F12, 0x0046},	//007E	//SARR_usDualGammaLutRGBOutdoor_0__4_
	{0x0F12, 0x00A7},	//00F5	//SARR_usDualGammaLutRGBOutdoor_0__5_
	{0x0F12, 0x010B},	//0142	//SARR_usDualGammaLutRGBOutdoor_0__6_
	{0x0F12, 0x013A},	//0166	//SARR_usDualGammaLutRGBOutdoor_0__7_
	{0x0F12, 0x0163},	//0186	//SARR_usDualGammaLutRGBOutdoor_0__8_
	{0x0F12, 0x019D},	//01BE	//SARR_usDualGammaLutRGBOutdoor_0__9_
	{0x0F12, 0x01CD},	//01EF	//SARR_usDualGammaLutRGBOutdoor_0__10_
	{0x0F12, 0x01F8},	//021B	//SARR_usDualGammaLutRGBOutdoor_0__11_
	{0x0F12, 0x021E},	//0244	//SARR_usDualGammaLutRGBOutdoor_0__12_
	{0x0F12, 0x0261},	//028A	//SARR_usDualGammaLutRGBOutdoor_0__13_
	{0x0F12, 0x02A1},	//02C2	//SARR_usDualGammaLutRGBOutdoor_0__14_
	{0x0F12, 0x0304},	//0318	//SARR_usDualGammaLutRGBOutdoor_0__15_
	{0x0F12, 0x0360},	//0360	//SARR_usDualGammaLutRGBOutdoor_0__16_
	{0x0F12, 0x039D},	//039D	//SARR_usDualGammaLutRGBOutdoor_0__17_
	{0x0F12, 0x03D3},	//03D3	//SARR_usDualGammaLutRGBOutdoor_0__18_
	{0x0F12, 0x03FF},	//03FF	//SARR_usDualGammaLutRGBOutdoor_0__19_

	{0x002A, 0x12CE},
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_0_
	{0x0F12, 0xFFFF}, //gisp_msm_OffsetNoBin_1_
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_2_
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_3_
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_4_
	{0x0F12, 0xFFFF}, //gisp_msm_OffsetNoBin_5_
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_6_
	{0x0F12, 0x0000}, //gisp_msm_OffsetNoBin_7_

//==================================================================================
// 17.AFIT
//==================================================================================
	{0x002A, 0x06D4},
	{0x0F12, 0x005A},	//afit_uNoiseIndInDoor_0_
	{0x0F12, 0x0078},	//afit_uNoiseIndInDoor_1_
	{0x0F12, 0x00C8}, 	//afit_uNoiseIndInDoor_2_
	{0x0F12, 0x0190},	//afit_uNoiseIndInDoor_3_
	{0x0F12, 0x028C},	//afit_uNoiseIndInDoor_4_
	{0x002A, 0x0734},
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__0_  //Brightness[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__1_  //Contrast[15:0]
	{0x0F12, 0xFFF6},  //0000  //0005 //FFB0	//AfitBaseVals_0__2_  //Saturation[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__3_  //Sharp_Blur[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__4_  //Glamour[15:0]
	{0x0F12, 0x0078},  //0078	//AfitBaseVals_0__5_  //sddd8a_edge_high[8:0]
	{0x0F12, 0x012C},  //012C	//AfitBaseVals_0__6_  //demsharpmix1_iLowBright[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_0__7_  //demsharpmix1_iHighBright[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_0__8_  //demsharpmix1_iLowSat[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_0__9_  //demsharpmix1_iHighSat[9:0]
	{0x0F12, 0x000C},  //000C	//AfitBaseVals_0__10_ //demsharpmix1_iLowThreshold[9:0]
	{0x0F12, 0x0010},  //0010	//AfitBaseVals_0__11_ //demsharpmix1_iHighThreshold[9:0]
	{0x0F12, 0x01E6},  //01E6	//AfitBaseVals_0__12_ //demsharpmix1_iRGBOffset[9:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__13_ //demsharpmix1_iDemClamp[9:0]
	{0x0F12, 0x0070},  //0070	//AfitBaseVals_0__14_ //demsharpmix1_iTune[8:0]
	{0x0F12, 0x01FF},  //01FF	//AfitBaseVals_0__15_ //YUV422_DENOISE_iUVLowThresh[9:0]
	{0x0F12, 0x0144},  //0144	//AfitBaseVals_0__16_ //YUV422_DENOISE_iUVHighThresh[9:0]
	{0x0F12, 0x000F},  //000F	//AfitBaseVals_0__17_ //sddd8a_iClustThresh_H[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_0__18_ //sddd8a_iClustThresh_C[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_0__19_ //Sharpening_iLowSharpClamp[9:0]
	{0x0F12, 0x0087},  //0087	//AfitBaseVals_0__20_ //Sharpening_iHighSharpClamp[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_0__21_ //sddd8a_iClustThresh_H_Bin_1_mode[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_0__22_ //sddd8a_iClustThresh_C_Bin_1_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_0__23_ //Sharpening_iLowSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_0__24_ //Sharpening_iHighSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_0__25_ //sddd8a_iClustThresh_H_Bin_2_mode[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_0__26_ //sddd8a_iClustThresh_C_Bin_2_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_0__27_ //Sharpening_iLowSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x0046},  //0046	//AfitBaseVals_0__28_ //Sharpening_iHighSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x2B32},  //2B32	//AfitBaseVals_0__29_ //sddd8a_edge_low[7:0],sddd8a_repl_thresh[15:8]
	{0x0F12, 0x0601},  //0601	//AfitBaseVals_0__30_ //sddd8a_repl_force[7:0],sddd8a_iMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__31_ //sddd8a_iHotThreshHigh[7:0],sddd8a_iHotThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__32_ //sddd8a_iColdThreshHigh[7:0],sddd8a_iColdThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__33_ //sddd8a_AddNoisePower1[7:0],sddd8a_AddNoisePower2[15:8]
	{0x0F12, 0x00FF},  //00FF	//AfitBaseVals_0__34_ //sddd8a_iSatSat[7:0],sddd8a_iRadialTune[15:8]
	{0x0F12, 0x07FF},  //07FF	//AfitBaseVals_0__35_ //sddd8a_iRadialLimit[7:0],sddd8a_iRadialPower[15:8]
	{0x0F12, 0xFFFF},  //FFFF	//AfitBaseVals_0__36_ //sddd8a_iLowMaxSlopeAllowed[7:0],sddd8a_iHighMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__37_ //sddd8a_iLowSlopeThresh[7:0],sddd8a_iHighSlopeThresh[15:8]
	{0x0F12, 0x050D},  //050D	//AfitBaseVals_0__38_ //Demosaicing_iDFD_ReduceCoeff[7:0],Demosaicing_iDecisionThresh[15:8]
	{0x0F12, 0x1E80},  //1E80	//AfitBaseVals_0__39_ //Demosaicing_iCentGrad[7:0],Demosaicing_iMonochrom[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__40_ //Demosaicing_iGRDenoiseVal[7:0],Demosaicing_iGBDenoiseVal[15:8]
	{0x0F12, 0x1406},  //140A  //140A	//AfitBaseVals_0__41_ //Demosaicing_iNearGrayDesat[7:0],Sharpening_iWSharpen[15:8]
	{0x0F12, 0x0214},  //0214	//AfitBaseVals_0__42_ //Sharpening_iWShThresh[7:0],Sharpening_nSharpWidth[15:8]
	{0x0F12, 0xFF01},  //FF01	//AfitBaseVals_0__43_ //Sharpening_iReduceNegative[7:0],Sharpening_iShDespeckle[15:8]
	{0x0F12, 0x180F},  //180F	//AfitBaseVals_0__44_ //demsharpmix1_iBCoeff[7:0],demsharpmix1_iGCoeff[15:8]
	{0x0F12, 0x0001},  //0001	//AfitBaseVals_0__45_ //demsharpmix1_iFilterPower[7:0],demsharpmix1_iWideMult[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__46_ //demsharpmix1_iNarrMult[7:0],demsharpmix1_iRGBMultiplier[15:8]
	{0x0F12, 0x3C03},  //3C03	//AfitBaseVals_0__47_ //YUV422_DENOISE_iUVSupport[7:0],byr_cgras_sim_iShadingPower[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_0__48_ //RGBGamma2_1LUT_sim_iLinearity[7:0],RGBGamma2_1LUT_sim_iDarkReduce[15:8]
	{0x0F12, 0x0080},//0580  //0828	//AfitBaseVals_0__49_ //ccm_oscar_sim_iSaturation[7:0],YUV422_CONTROL_Y_off[15:8]
	{0x0F12, 0x0280},  //0280	//AfitBaseVals_0__50_ //YUV422_CONTROL_Y_mul[7:0],sddd8a_nClustLevel_H[15:8]
	{0x0F12, 0x0308},  //0308	//AfitBaseVals_0__51_ //sddd8a_iClustMulT_H[7:0],sddd8a_iClustMulT_C[15:8]
	{0x0F12, 0x3186},  //3186	//AfitBaseVals_0__52_ //sddd8a_DispTH_Low[7:0],sddd8a_DispTH_High[15:8]
	{0x0F12, 0xC8C8},  //C8C8	//AfitBaseVals_0__53_ //sddd8a_iDenThreshLow[7:0],sddd8a_iDenThreshHigh[15:8]
	{0x0F12, 0x0A02},  //0A02	//AfitBaseVals_0__54_ //Demosaicing_iDemSharpenLow[7:0],Demosaicing_iDemSharpenHigh[15:8]
	{0x0F12, 0x080A},  //080A	//AfitBaseVals_0__55_ //Demosaicing_iDemSharpThresh[7:0],Demosaicing_iDespeckleForDemsharp[15:8]
	{0x0F12, 0x0500},  //0500	//AfitBaseVals_0__56_ //Demosaicing_iEdgeDesatThrLow[7:0],Demosaicing_iEdgeDesatThrHigh[15:8]
	{0x0F12, 0x032D},  //032D	//AfitBaseVals_0__57_ //Demosaicing_iEdgeDesat[7:0],Demosaicing_iEdgeDesatLimit[15:8]
	{0x0F12, 0x324E},  //324E	//AfitBaseVals_0__58_ //Demosaicing_iDemShLowLimit[7:0],Sharpening_iLowSharpPower[15:8]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_0__59_ //Sharpening_iHighSharpPower[7:0],Sharpening_iLowShDenoise[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_0__60_ //Sharpening_iHighShDenoise[7:0],demsharpmix1_iWideFiltReduce[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_0__61_ //demsharpmix1_iNarrFiltReduce[7:0],sddd8a_nClustLevel_H_Bin_1_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_0__62_ //sddd8a_iClustMulT_H_Bin_1_mode[7:0],sddd8a_iClustMulT_C_Bin_1_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_0__63_ //sddd8a_DispTH_Low_Bin_1_mode[7:0],sddd8a_DispTH_High_Bin_1_mode[15:8]
	{0x0F12, 0x4646},  //4646	//AfitBaseVals_0__64_ //sddd8a_iDenThreshLow_Bin_1_mode[7:0],sddd8a_iDenThreshHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0802},  //0802	//AfitBaseVals_0__65_ //Demosaicing_iDemSharpenLow_Bin_1_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0802},  //0802	//AfitBaseVals_0__66_ //Demosaicing_iDemSharpThresh_Bin_1_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_1_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__67_ //Demosaicing_iEdgeDesatThrLow_Bin_1_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_1_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_0__68_ //Demosaicing_iEdgeDesat_Bin_1_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_1_mode[15:8]
	{0x0F12, 0x3202},  //3202	//AfitBaseVals_0__69_ //Demosaicing_iDemShLowLimit_Bin_1_mode[7:0],Sharpening_iLowSharpPower_Bin_1_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_0__70_ //Sharpening_iHighSharpPower_Bin_1_mode[7:0],Sharpening_iLowShDenoise_Bin_1_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_0__71_ //Sharpening_iHighShDenoise_Bin_1_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_1_mode[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_0__72_ //demsharpmix1_iNarrFiltReduce_Bin_1_mode[7:0],sddd8a_nClustLevel_H_Bin_2_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_0__73_ //sddd8a_iClustMulT_H_Bin_2_mode[7:0],sddd8a_iClustMulT_C_Bin_2_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_0__74_ //sddd8a_DispTH_Low_Bin_2_mode[7:0],sddd8a_DispTH_High_Bin_2_mode[15:8]
	{0x0F12, 0x4646},  //4646	//AfitBaseVals_0__75_ //sddd8a_iDenThreshLow_Bin_2_mode[7:0],sddd8a_iDenThreshHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0802},  //0802	//AfitBaseVals_0__76_ //Demosaicing_iDemSharpenLow_Bin_2_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0802},  //0802	//AfitBaseVals_0__77_ //Demosaicing_iDemSharpThresh_Bin_2_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_2_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_0__78_ //Demosaicing_iEdgeDesatThrLow_Bin_2_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_2_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_0__79_ //Demosaicing_iEdgeDesat_Bin_2_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_2_mode[15:8]
	{0x0F12, 0x3202},  //3202	//AfitBaseVals_0__80_ //Demosaicing_iDemShLowLimit_Bin_2_mode[7:0],Sharpening_iLowSharpPower_Bin_2_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_0__81_ //Sharpening_iHighSharpPower_Bin_2_mode[7:0],Sharpening_iLowShDenoise_Bin_2_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_0__82_ //Sharpening_iHighShDenoise_Bin_2_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_2_mode[15:8]
	{0x0F12, 0x0003},  //0003	//AfitBaseVals_0__83_ //demsharpmix1_iNarrFiltReduce_Bin_2_mode[7:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__0_  //Brightness[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__1_  //Contrast[15:0]
	{0x0F12, 0xFFF6},  //0000  //0014	//AfitBaseVals_1__2_  //Saturation[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__3_  //Sharp_Blur[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__4_  //Glamour[15:0]
	{0x0F12, 0x006A},  //006A	//AfitBaseVals_1__5_  //sddd8a_edge_high[8:0]
	{0x0F12, 0x012C},  //012C	//AfitBaseVals_1__6_  //demsharpmix1_iLowBright[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_1__7_  //demsharpmix1_iHighBright[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_1__8_  //demsharpmix1_iLowSat[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_1__9_  //demsharpmix1_iHighSat[9:0]
	{0x0F12, 0x000C},  //000C	//AfitBaseVals_1__10_ //demsharpmix1_iLowThreshold[9:0]
	{0x0F12, 0x0010},  //0010	//AfitBaseVals_1__11_ //demsharpmix1_iHighThreshold[9:0]
	{0x0F12, 0x01E6},  //01E6	//AfitBaseVals_1__12_ //demsharpmix1_iRGBOffset[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_1__13_ //demsharpmix1_iDemClamp[9:0]
	{0x0F12, 0x0070},  //0070	//AfitBaseVals_1__14_ //demsharpmix1_iTune[8:0]
	{0x0F12, 0x007D},  //007D	//AfitBaseVals_1__15_ //YUV422_DENOISE_iUVLowThresh[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_1__16_ //YUV422_DENOISE_iUVHighThresh[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_1__17_ //sddd8a_iClustThresh_H[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_1__18_ //sddd8a_iClustThresh_C[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_1__19_ //Sharpening_iLowSharpClamp[9:0]
	{0x0F12, 0x0087},  //0087	//AfitBaseVals_1__20_ //Sharpening_iHighSharpClamp[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_1__21_ //sddd8a_iClustThresh_H_Bin_1_mode[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_1__22_ //sddd8a_iClustThresh_C_Bin_1_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_1__23_ //Sharpening_iLowSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_1__24_ //Sharpening_iHighSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_1__25_ //sddd8a_iClustThresh_H_Bin_2_mode[9:0]
	{0x0F12, 0x000A},  //000A	//AfitBaseVals_1__26_ //sddd8a_iClustThresh_C_Bin_2_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_1__27_ //Sharpening_iLowSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_1__28_ //Sharpening_iHighSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x2B32},  //2B32	//AfitBaseVals_1__29_ //sddd8a_edge_low[7:0],sddd8a_repl_thresh[15:8]
	{0x0F12, 0x0601},  //0601	//AfitBaseVals_1__30_ //sddd8a_repl_force[7:0],sddd8a_iMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__31_ //sddd8a_iHotThreshHigh[7:0],sddd8a_iHotThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__32_ //sddd8a_iColdThreshHigh[7:0],sddd8a_iColdThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__33_ //sddd8a_AddNoisePower1[7:0],sddd8a_AddNoisePower2[15:8]
	{0x0F12, 0x00FF},  //00FF	//AfitBaseVals_1__34_ //sddd8a_iSatSat[7:0],sddd8a_iRadialTune[15:8]
	{0x0F12, 0x07FF},  //07FF	//AfitBaseVals_1__35_ //sddd8a_iRadialLimit[7:0],sddd8a_iRadialPower[15:8]
	{0x0F12, 0xFFFF},  //FFFF	//AfitBaseVals_1__36_ //sddd8a_iLowMaxSlopeAllowed[7:0],sddd8a_iHighMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__37_ //sddd8a_iLowSlopeThresh[7:0],sddd8a_iHighSlopeThresh[15:8]
	{0x0F12, 0x050D},  //050D	//AfitBaseVals_1__38_ //Demosaicing_iDFD_ReduceCoeff[7:0],Demosaicing_iDecisionThresh[15:8]
	{0x0F12, 0x1E80},  //1E80	//AfitBaseVals_1__39_ //Demosaicing_iCentGrad[7:0],Demosaicing_iMonochrom[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__40_ //Demosaicing_iGRDenoiseVal[7:0],Demosaicing_iGBDenoiseVal[15:8]
	{0x0F12, 0x1405},  //1408  //1408	//AfitBaseVals_1__41_ //Demosaicing_iNearGrayDesat[7:0],Sharpening_iWSharpen[15:8]
	{0x0F12, 0x0214},  //0214	//AfitBaseVals_1__42_ //Sharpening_iWShThresh[7:0],Sharpening_nSharpWidth[15:8]
	{0x0F12, 0xFF01},  //FF01	//AfitBaseVals_1__43_ //Sharpening_iReduceNegative[7:0],Sharpening_iShDespeckle[15:8]
	{0x0F12, 0x180F},  //180F	//AfitBaseVals_1__44_ //demsharpmix1_iBCoeff[7:0],demsharpmix1_iGCoeff[15:8]
	{0x0F12, 0x0002},  //0002	//AfitBaseVals_1__45_ //demsharpmix1_iFilterPower[7:0],demsharpmix1_iWideMult[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__46_ //demsharpmix1_iNarrMult[7:0],demsharpmix1_iRGBMultiplier[15:8]
	{0x0F12, 0x4603},  //4603	//AfitBaseVals_1__47_ //YUV422_DENOISE_iUVSupport[7:0],byr_cgras_sim_iShadingPower[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_1__48_ //RGBGamma2_1LUT_sim_iLinearity[7:0],RGBGamma2_1LUT_sim_iDarkReduce[15:8]
	{0x0F12, 0x0080},  //083C	//AfitBaseVals_1__49_ //ccm_oscar_sim_iSaturation[7:0],YUV422_CONTROL_Y_off[15:8]
	{0x0F12, 0x0280},  //0280	//AfitBaseVals_1__50_ //YUV422_CONTROL_Y_mul[7:0],sddd8a_nClustLevel_H[15:8]
	{0x0F12, 0x0308},  //0308	//AfitBaseVals_1__51_ //sddd8a_iClustMulT_H[7:0],sddd8a_iClustMulT_C[15:8]
	{0x0F12, 0x3165},  //1E65  //1E65	//AfitBaseVals_1__52_ //sddd8a_DispTH_Low[7:0],sddd8a_DispTH_High[15:8]
	{0x0F12, 0x7A78},  //7A78	//AfitBaseVals_1__53_ //sddd8a_iDenThreshLow[7:0],sddd8a_iDenThreshHigh[15:8]
	{0x0F12, 0x0A03},  //0A03	//AfitBaseVals_1__54_ //Demosaicing_iDemSharpenLow[7:0],Demosaicing_iDemSharpenHigh[15:8]
	{0x0F12, 0x080A},  //080A	//AfitBaseVals_1__55_ //Demosaicing_iDemSharpThresh[7:0],Demosaicing_iDespeckleForDemsharp[15:8]
	{0x0F12, 0x0500},  //0500	//AfitBaseVals_1__56_ //Demosaicing_iEdgeDesatThrLow[7:0],Demosaicing_iEdgeDesatThrHigh[15:8]
	{0x0F12, 0x032D},  //032D	//AfitBaseVals_1__57_ //Demosaicing_iEdgeDesat[7:0],Demosaicing_iEdgeDesatLimit[15:8]
	{0x0F12, 0x324D},  //324D	//AfitBaseVals_1__58_ //Demosaicing_iDemShLowLimit[7:0],Sharpening_iLowSharpPower[15:8]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_1__59_ //Sharpening_iHighSharpPower[7:0],Sharpening_iLowShDenoise[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_1__60_ //Sharpening_iHighShDenoise[7:0],demsharpmix1_iWideFiltReduce[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_1__61_ //demsharpmix1_iNarrFiltReduce[7:0],sddd8a_nClustLevel_H_Bin_1_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_1__62_ //sddd8a_iClustMulT_H_Bin_1_mode[7:0],sddd8a_iClustMulT_C_Bin_1_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_1__63_ //sddd8a_DispTH_Low_Bin_1_mode[7:0],sddd8a_DispTH_High_Bin_1_mode[15:8]
	{0x0F12, 0x2F34},  //2F34	//AfitBaseVals_1__64_ //sddd8a_iDenThreshLow_Bin_1_mode[7:0],sddd8a_iDenThreshHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0504},  //0504	//AfitBaseVals_1__65_ //Demosaicing_iDemSharpenLow_Bin_1_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_1_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_1__66_ //Demosaicing_iDemSharpThresh_Bin_1_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_1_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__67_ //Demosaicing_iEdgeDesatThrLow_Bin_1_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_1_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_1__68_ //Demosaicing_iEdgeDesat_Bin_1_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_1_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_1__69_ //Demosaicing_iDemShLowLimit_Bin_1_mode[7:0],Sharpening_iLowSharpPower_Bin_1_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_1__70_ //Sharpening_iHighSharpPower_Bin_1_mode[7:0],Sharpening_iLowShDenoise_Bin_1_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_1__71_ //Sharpening_iHighShDenoise_Bin_1_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_1_mode[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_1__72_ //demsharpmix1_iNarrFiltReduce_Bin_1_mode[7:0],sddd8a_nClustLevel_H_Bin_2_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_1__73_ //sddd8a_iClustMulT_H_Bin_2_mode[7:0],sddd8a_iClustMulT_C_Bin_2_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_1__74_ //sddd8a_DispTH_Low_Bin_2_mode[7:0],sddd8a_DispTH_High_Bin_2_mode[15:8]
	{0x0F12, 0x1414},  //1414	//AfitBaseVals_1__75_ //sddd8a_iDenThreshLow_Bin_2_mode[7:0],sddd8a_iDenThreshHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0504},  //0504	//AfitBaseVals_1__76_ //Demosaicing_iDemSharpenLow_Bin_2_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_2_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_1__77_ //Demosaicing_iDemSharpThresh_Bin_2_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_2_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_1__78_ //Demosaicing_iEdgeDesatThrLow_Bin_2_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_2_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_1__79_ //Demosaicing_iEdgeDesat_Bin_2_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_2_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_1__80_ //Demosaicing_iDemShLowLimit_Bin_2_mode[7:0],Sharpening_iLowSharpPower_Bin_2_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_1__81_ //Sharpening_iHighSharpPower_Bin_2_mode[7:0],Sharpening_iLowShDenoise_Bin_2_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_1__82_ //Sharpening_iHighShDenoise_Bin_2_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_2_mode[15:8]
	{0x0F12, 0x0003},  //0003	//AfitBaseVals_1__83_ //demsharpmix1_iNarrFiltReduce_Bin_2_mode[7:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__0_  //Brightness[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__1_  //Contrast[15:0]
	{0x0F12, 0xFFF6},  //0000  //0019	//AfitBaseVals_2__2_  //Saturation[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__3_  //Sharp_Blur[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__4_  //Glamour[15:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_2__5_  //sddd8a_edge_high[8:0]
	{0x0F12, 0x012C},  //012C	//AfitBaseVals_2__6_  //demsharpmix1_iLowBright[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_2__7_  //demsharpmix1_iHighBright[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_2__8_  //demsharpmix1_iLowSat[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_2__9_  //demsharpmix1_iHighSat[9:0]
	{0x0F12, 0x000C},  //000C	//AfitBaseVals_2__10_ //demsharpmix1_iLowThreshold[9:0]
	{0x0F12, 0x0010},  //0010	//AfitBaseVals_2__11_ //demsharpmix1_iHighThreshold[9:0]
	{0x0F12, 0x01E6},  //01E6	//AfitBaseVals_2__12_ //demsharpmix1_iRGBOffset[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_2__13_ //demsharpmix1_iDemClamp[9:0]
	{0x0F12, 0x0070},  //0070	//AfitBaseVals_2__14_ //demsharpmix1_iTune[8:0]
	{0x0F12, 0x007D},  //007D	//AfitBaseVals_2__15_ //YUV422_DENOISE_iUVLowThresh[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_2__16_ //YUV422_DENOISE_iUVHighThresh[9:0]
	{0x0F12, 0x0032},  //0032	//AfitBaseVals_2__17_ //sddd8a_iClustThresh_H[9:0]
	{0x0F12, 0x003C},  //003C	//AfitBaseVals_2__18_ //sddd8a_iClustThresh_C[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_2__19_ //Sharpening_iLowSharpClamp[9:0]
	{0x0F12, 0x0087},  //0087	//AfitBaseVals_2__20_ //Sharpening_iHighSharpClamp[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_2__21_ //sddd8a_iClustThresh_H_Bin_1_mode[9:0]
	{0x0F12, 0x0019},  //0019	//AfitBaseVals_2__22_ //sddd8a_iClustThresh_C_Bin_1_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_2__23_ //Sharpening_iLowSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_2__24_ //Sharpening_iHighSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_2__25_ //sddd8a_iClustThresh_H_Bin_2_mode[9:0]
	{0x0F12, 0x0019},  //0019	//AfitBaseVals_2__26_ //sddd8a_iClustThresh_C_Bin_2_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_2__27_ //Sharpening_iLowSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_2__28_ //Sharpening_iHighSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x2B32},  //2B32	//AfitBaseVals_2__29_ //sddd8a_edge_low[7:0],sddd8a_repl_thresh[15:8]
	{0x0F12, 0x0601},  //0601	//AfitBaseVals_2__30_ //sddd8a_repl_force[7:0],sddd8a_iMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__31_ //sddd8a_iHotThreshHigh[7:0],sddd8a_iHotThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__32_ //sddd8a_iColdThreshHigh[7:0],sddd8a_iColdThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__33_ //sddd8a_AddNoisePower1[7:0],sddd8a_AddNoisePower2[15:8]
	{0x0F12, 0x00FF},  //00FF	//AfitBaseVals_2__34_ //sddd8a_iSatSat[7:0],sddd8a_iRadialTune[15:8]
	{0x0F12, 0x07FF},  //07FF	//AfitBaseVals_2__35_ //sddd8a_iRadialLimit[7:0],sddd8a_iRadialPower[15:8]
	{0x0F12, 0xFFFF},  //FFFF	//AfitBaseVals_2__36_ //sddd8a_iLowMaxSlopeAllowed[7:0],sddd8a_iHighMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__37_ //sddd8a_iLowSlopeThresh[7:0],sddd8a_iHighSlopeThresh[15:8]
	{0x0F12, 0x050D},  //050D	//AfitBaseVals_2__38_ //Demosaicing_iDFD_ReduceCoeff[7:0],Demosaicing_iDecisionThresh[15:8]
	{0x0F12, 0x1E80},  //1E80	//AfitBaseVals_2__39_ //Demosaicing_iCentGrad[7:0],Demosaicing_iMonochrom[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__40_ //Demosaicing_iGRDenoiseVal[7:0],Demosaicing_iGBDenoiseVal[15:8]
	{0x0F12, 0x0A04},  //0A04	//AfitBaseVals_2__41_ //Demosaicing_iNearGrayDesat[7:0],Sharpening_iWSharpen[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_2__42_ //Sharpening_iWShThresh[7:0],Sharpening_nSharpWidth[15:8]
	{0x0F12, 0xFF01},  //FF01	//AfitBaseVals_2__43_ //Sharpening_iReduceNegative[7:0],Sharpening_iShDespeckle[15:8]
	{0x0F12, 0x180F},  //180F	//AfitBaseVals_2__44_ //demsharpmix1_iBCoeff[7:0],demsharpmix1_iGCoeff[15:8]
	{0x0F12, 0x0002},  //0002	//AfitBaseVals_2__45_ //demsharpmix1_iFilterPower[7:0],demsharpmix1_iWideMult[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__46_ //demsharpmix1_iNarrMult[7:0],demsharpmix1_iRGBMultiplier[15:8]
	{0x0F12, 0x5A03},  //5A03	//AfitBaseVals_2__47_ //YUV422_DENOISE_iUVSupport[7:0],byr_cgras_sim_iShadingPower[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_2__48_ //RGBGamma2_1LUT_sim_iLinearity[7:0],RGBGamma2_1LUT_sim_iDarkReduce[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_2__49_ //ccm_oscar_sim_iSaturation[7:0],YUV422_CONTROL_Y_off[15:8]
	{0x0F12, 0x0280},  //0280	//AfitBaseVals_2__50_ //YUV422_CONTROL_Y_mul[7:0],sddd8a_nClustLevel_H[15:8]
	{0x0F12, 0x0208},  //0208	//AfitBaseVals_2__51_ //sddd8a_iClustMulT_H[7:0],sddd8a_iClustMulT_C[15:8]
	{0x0F12, 0x6464},  //1E50  //1E50	//AfitBaseVals_2__52_ //sddd8a_DispTH_Low[7:0],sddd8a_DispTH_High[15:8]
	{0x0F12, 0x3C4B},  //3C4B	//AfitBaseVals_2__53_ //sddd8a_iDenThreshLow[7:0],sddd8a_iDenThreshHigh[15:8]
	{0x0F12, 0x0A05},  //0A05	//AfitBaseVals_2__54_ //Demosaicing_iDemSharpenLow[7:0],Demosaicing_iDemSharpenHigh[15:8]
	{0x0F12, 0x080A},  //080A	//AfitBaseVals_2__55_ //Demosaicing_iDemSharpThresh[7:0],Demosaicing_iDespeckleForDemsharp[15:8]
	{0x0F12, 0x0500},  //0500	//AfitBaseVals_2__56_ //Demosaicing_iEdgeDesatThrLow[7:0],Demosaicing_iEdgeDesatThrHigh[15:8]
	{0x0F12, 0x032D},  //032D	//AfitBaseVals_2__57_ //Demosaicing_iEdgeDesat[7:0],Demosaicing_iEdgeDesatLimit[15:8]
	{0x0F12, 0x324D},  //324D	//AfitBaseVals_2__58_ //Demosaicing_iDemShLowLimit[7:0],Sharpening_iLowSharpPower[15:8]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_2__59_ //Sharpening_iHighSharpPower[7:0],Sharpening_iLowShDenoise[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_2__60_ //Sharpening_iHighShDenoise[7:0],demsharpmix1_iWideFiltReduce[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_2__61_ //demsharpmix1_iNarrFiltReduce[7:0],sddd8a_nClustLevel_H_Bin_1_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_2__62_ //sddd8a_iClustMulT_H_Bin_1_mode[7:0],sddd8a_iClustMulT_C_Bin_1_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_2__63_ //sddd8a_DispTH_Low_Bin_1_mode[7:0],sddd8a_DispTH_High_Bin_1_mode[15:8]
	{0x0F12, 0x1E23},  //1E23	//AfitBaseVals_2__64_ //sddd8a_iDenThreshLow_Bin_1_mode[7:0],sddd8a_iDenThreshHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0505},  //0505	//AfitBaseVals_2__65_ //Demosaicing_iDemSharpenLow_Bin_1_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_1_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_2__66_ //Demosaicing_iDemSharpThresh_Bin_1_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_1_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__67_ //Demosaicing_iEdgeDesatThrLow_Bin_1_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_1_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_2__68_ //Demosaicing_iEdgeDesat_Bin_1_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_1_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_2__69_ //Demosaicing_iDemShLowLimit_Bin_1_mode[7:0],Sharpening_iLowSharpPower_Bin_1_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_2__70_ //Sharpening_iHighSharpPower_Bin_1_mode[7:0],Sharpening_iLowShDenoise_Bin_1_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_2__71_ //Sharpening_iHighShDenoise_Bin_1_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_1_mode[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_2__72_ //demsharpmix1_iNarrFiltReduce_Bin_1_mode[7:0],sddd8a_nClustLevel_H_Bin_2_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_2__73_ //sddd8a_iClustMulT_H_Bin_2_mode[7:0],sddd8a_iClustMulT_C_Bin_2_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_2__74_ //sddd8a_DispTH_Low_Bin_2_mode[7:0],sddd8a_DispTH_High_Bin_2_mode[15:8]
	{0x0F12, 0x1E23},  //1E23	//AfitBaseVals_2__75_ //sddd8a_iDenThreshLow_Bin_2_mode[7:0],sddd8a_iDenThreshHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0505},  //0505	//AfitBaseVals_2__76_ //Demosaicing_iDemSharpenLow_Bin_2_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_2_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_2__77_ //Demosaicing_iDemSharpThresh_Bin_2_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_2_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_2__78_ //Demosaicing_iEdgeDesatThrLow_Bin_2_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_2_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_2__79_ //Demosaicing_iEdgeDesat_Bin_2_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_2_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_2__80_ //Demosaicing_iDemShLowLimit_Bin_2_mode[7:0],Sharpening_iLowSharpPower_Bin_2_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_2__81_ //Sharpening_iHighSharpPower_Bin_2_mode[7:0],Sharpening_iLowShDenoise_Bin_2_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_2__82_ //Sharpening_iHighShDenoise_Bin_2_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_2_mode[15:8]
	{0x0F12, 0x0003},  //0003	//AfitBaseVals_2__83_ //demsharpmix1_iNarrFiltReduce_Bin_2_mode[7:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__0_  //Brightness[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__1_  //Contrast[15:0]
	{0x0F12, 0xFFF6},  //0000  //0019	//AfitBaseVals_3__2_  //Saturation[15:0]
	{0x0F12, 0x0007},  //0007	//AfitBaseVals_3__3_  //Sharp_Blur[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__4_  //Glamour[15:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_3__5_  //sddd8a_edge_high[8:0]
	{0x0F12, 0x012C},  //012C	//AfitBaseVals_3__6_  //demsharpmix1_iLowBright[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_3__7_  //demsharpmix1_iHighBright[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_3__8_  //demsharpmix1_iLowSat[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_3__9_  //demsharpmix1_iHighSat[9:0]
	{0x0F12, 0x000C},  //000C	//AfitBaseVals_3__10_ //demsharpmix1_iLowThreshold[9:0]
	{0x0F12, 0x0010},  //0010	//AfitBaseVals_3__11_ //demsharpmix1_iHighThreshold[9:0]
	{0x0F12, 0x01E6},  //01E6	//AfitBaseVals_3__12_ //demsharpmix1_iRGBOffset[9:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__13_ //demsharpmix1_iDemClamp[9:0]
	{0x0F12, 0x0070},  //0070	//AfitBaseVals_3__14_ //demsharpmix1_iTune[8:0]
	{0x0F12, 0x007D},  //007D	//AfitBaseVals_3__15_ //YUV422_DENOISE_iUVLowThresh[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_3__16_ //YUV422_DENOISE_iUVHighThresh[9:0]
	{0x0F12, 0x0032},  //0032	//AfitBaseVals_3__17_ //sddd8a_iClustThresh_H[9:0]
	{0x0F12, 0x003C},  //003C	//AfitBaseVals_3__18_ //sddd8a_iClustThresh_C[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_3__19_ //Sharpening_iLowSharpClamp[9:0]
	{0x0F12, 0x009F},  //009F	//AfitBaseVals_3__20_ //Sharpening_iHighSharpClamp[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_3__21_ //sddd8a_iClustThresh_H_Bin_1_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_3__22_ //sddd8a_iClustThresh_C_Bin_1_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_3__23_ //Sharpening_iLowSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0037},  //0037	//AfitBaseVals_3__24_ //Sharpening_iHighSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_3__25_ //sddd8a_iClustThresh_H_Bin_2_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_3__26_ //sddd8a_iClustThresh_C_Bin_2_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_3__27_ //Sharpening_iLowSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x0037},  //0037	//AfitBaseVals_3__28_ //Sharpening_iHighSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x2B32},  //2B32	//AfitBaseVals_3__29_ //sddd8a_edge_low[7:0],sddd8a_repl_thresh[15:8]
	{0x0F12, 0x0601},  //0601	//AfitBaseVals_3__30_ //sddd8a_repl_force[7:0],sddd8a_iMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__31_ //sddd8a_iHotThreshHigh[7:0],sddd8a_iHotThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__32_ //sddd8a_iColdThreshHigh[7:0],sddd8a_iColdThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__33_ //sddd8a_AddNoisePower1[7:0],sddd8a_AddNoisePower2[15:8]
	{0x0F12, 0x00FF},  //00FF	//AfitBaseVals_3__34_ //sddd8a_iSatSat[7:0],sddd8a_iRadialTune[15:8]
	{0x0F12, 0x07A0},  //07A0	//AfitBaseVals_3__35_ //sddd8a_iRadialLimit[7:0],sddd8a_iRadialPower[15:8]
	{0x0F12, 0xFFFF},  //FFFF	//AfitBaseVals_3__36_ //sddd8a_iLowMaxSlopeAllowed[7:0],sddd8a_iHighMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__37_ //sddd8a_iLowSlopeThresh[7:0],sddd8a_iHighSlopeThresh[15:8]
	{0x0F12, 0x050D},  //050D	//AfitBaseVals_3__38_ //Demosaicing_iDFD_ReduceCoeff[7:0],Demosaicing_iDecisionThresh[15:8]
	{0x0F12, 0x1E80},  //1E80	//AfitBaseVals_3__39_ //Demosaicing_iCentGrad[7:0],Demosaicing_iMonochrom[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__40_ //Demosaicing_iGRDenoiseVal[7:0],Demosaicing_iGBDenoiseVal[15:8]
	{0x0F12, 0x0A02},  //0A02	//AfitBaseVals_3__41_ //Demosaicing_iNearGrayDesat[7:0],Sharpening_iWSharpen[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_3__42_ //Sharpening_iWShThresh[7:0],Sharpening_nSharpWidth[15:8]
	{0x0F12, 0xFF01},  //FF01	//AfitBaseVals_3__43_ //Sharpening_iReduceNegative[7:0],Sharpening_iShDespeckle[15:8]
	{0x0F12, 0x180F},  //180F	//AfitBaseVals_3__44_ //demsharpmix1_iBCoeff[7:0],demsharpmix1_iGCoeff[15:8]
	{0x0F12, 0x0001},  //0001	//AfitBaseVals_3__45_ //demsharpmix1_iFilterPower[7:0],demsharpmix1_iWideMult[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__46_ //demsharpmix1_iNarrMult[7:0],demsharpmix1_iRGBMultiplier[15:8]
	{0x0F12, 0x5A03},  //5A03	//AfitBaseVals_3__47_ //YUV422_DENOISE_iUVSupport[7:0],byr_cgras_sim_iShadingPower[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_3__48_ //RGBGamma2_1LUT_sim_iLinearity[7:0],RGBGamma2_1LUT_sim_iDarkReduce[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_3__49_ //ccm_oscar_sim_iSaturation[7:0],YUV422_CONTROL_Y_off[15:8]
	{0x0F12, 0x0280},  //0280	//AfitBaseVals_3__50_ //YUV422_CONTROL_Y_mul[7:0],sddd8a_nClustLevel_H[15:8]
	{0x0F12, 0x0108},  //0108	//AfitBaseVals_3__51_ //sddd8a_iClustMulT_H[7:0],sddd8a_iClustMulT_C[15:8]
	{0x0F12, 0x6464},  //6464	//AfitBaseVals_3__52_ //sddd8a_DispTH_Low[7:0],sddd8a_DispTH_High[15:8]
	{0x0F12, 0x1A24},  //1A24	//AfitBaseVals_3__53_ //sddd8a_iDenThreshLow[7:0],sddd8a_iDenThreshHigh[15:8]
	{0x0F12, 0x0A05},  //0A05	//AfitBaseVals_3__54_ //Demosaicing_iDemSharpenLow[7:0],Demosaicing_iDemSharpenHigh[15:8]
	{0x0F12, 0x080A},  //080A	//AfitBaseVals_3__55_ //Demosaicing_iDemSharpThresh[7:0],Demosaicing_iDespeckleForDemsharp[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__56_ //Demosaicing_iEdgeDesatThrLow[7:0],Demosaicing_iEdgeDesatThrHigh[15:8]
	{0x0F12, 0x0328},  //0328	//AfitBaseVals_3__57_ //Demosaicing_iEdgeDesat[7:0],Demosaicing_iEdgeDesatLimit[15:8]
	{0x0F12, 0x324C},  //324C	//AfitBaseVals_3__58_ //Demosaicing_iDemShLowLimit[7:0],Sharpening_iLowSharpPower[15:8]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_3__59_ //Sharpening_iHighSharpPower[7:0],Sharpening_iLowShDenoise[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_3__60_ //Sharpening_iHighShDenoise[7:0],demsharpmix1_iWideFiltReduce[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_3__61_ //demsharpmix1_iNarrFiltReduce[7:0],sddd8a_nClustLevel_H_Bin_1_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_3__62_ //sddd8a_iClustMulT_H_Bin_1_mode[7:0],sddd8a_iClustMulT_C_Bin_1_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_3__63_ //sddd8a_DispTH_Low_Bin_1_mode[7:0],sddd8a_DispTH_High_Bin_1_mode[15:8]
	{0x0F12, 0x0F0F},  //0F0F	//AfitBaseVals_3__64_ //sddd8a_iDenThreshLow_Bin_1_mode[7:0],sddd8a_iDenThreshHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0307},  //0307	//AfitBaseVals_3__65_ //Demosaicing_iDemSharpenLow_Bin_1_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_1_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_3__66_ //Demosaicing_iDemSharpThresh_Bin_1_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_1_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__67_ //Demosaicing_iEdgeDesatThrLow_Bin_1_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_1_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_3__68_ //Demosaicing_iEdgeDesat_Bin_1_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_1_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_3__69_ //Demosaicing_iDemShLowLimit_Bin_1_mode[7:0],Sharpening_iLowSharpPower_Bin_1_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_3__70_ //Sharpening_iHighSharpPower_Bin_1_mode[7:0],Sharpening_iLowShDenoise_Bin_1_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_3__71_ //Sharpening_iHighShDenoise_Bin_1_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_1_mode[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_3__72_ //demsharpmix1_iNarrFiltReduce_Bin_1_mode[7:0],sddd8a_nClustLevel_H_Bin_2_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_3__73_ //sddd8a_iClustMulT_H_Bin_2_mode[7:0],sddd8a_iClustMulT_C_Bin_2_mode[15:8]
	{0x0F12, 0x9696},  //9696	//AfitBaseVals_3__74_ //sddd8a_DispTH_Low_Bin_2_mode[7:0],sddd8a_DispTH_High_Bin_2_mode[15:8]
	{0x0F12, 0x0F0F},  //0F0F	//AfitBaseVals_3__75_ //sddd8a_iDenThreshLow_Bin_2_mode[7:0],sddd8a_iDenThreshHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0307},  //0307	//AfitBaseVals_3__76_ //Demosaicing_iDemSharpenLow_Bin_2_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_2_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_3__77_ //Demosaicing_iDemSharpThresh_Bin_2_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_2_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_3__78_ //Demosaicing_iEdgeDesatThrLow_Bin_2_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_2_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_3__79_ //Demosaicing_iEdgeDesat_Bin_2_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_2_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_3__80_ //Demosaicing_iDemShLowLimit_Bin_2_mode[7:0],Sharpening_iLowSharpPower_Bin_2_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_3__81_ //Sharpening_iHighSharpPower_Bin_2_mode[7:0],Sharpening_iLowShDenoise_Bin_2_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_3__82_ //Sharpening_iHighShDenoise_Bin_2_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_2_mode[15:8]
	{0x0F12, 0x0003},  //0003	//AfitBaseVals_3__83_ //demsharpmix1_iNarrFiltReduce_Bin_2_mode[7:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__0_  //Brightness[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__1_  //Contrast[15:0]
	{0x0F12, 0xFFF6},  //0000  //000A	//AfitBaseVals_4__2_  //Saturation[15:0]
	{0x0F12, 0x0007},  //0007	//AfitBaseVals_4__3_  //Sharp_Blur[15:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__4_  //Glamour[15:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_4__5_  //sddd8a_edge_high[8:0]
	{0x0F12, 0x012C},  //012C	//AfitBaseVals_4__6_  //demsharpmix1_iLowBright[9:0]
	{0x0F12, 0x03FF},  //03FF	//AfitBaseVals_4__7_  //demsharpmix1_iHighBright[9:0]
	{0x0F12, 0x0014},  //0014	//AfitBaseVals_4__8_  //demsharpmix1_iLowSat[9:0]
	{0x0F12, 0x0064},  //0064	//AfitBaseVals_4__9_  //demsharpmix1_iHighSat[9:0]
	{0x0F12, 0x000C},  //000C	//AfitBaseVals_4__10_ //demsharpmix1_iLowThreshold[9:0]
	{0x0F12, 0x0010},  //0010	//AfitBaseVals_4__11_ //demsharpmix1_iHighThreshold[9:0]
	{0x0F12, 0x01E6},  //01E6	//AfitBaseVals_4__12_ //demsharpmix1_iRGBOffset[9:0]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__13_ //demsharpmix1_iDemClamp[9:0]
	{0x0F12, 0x0070},  //0070	//AfitBaseVals_4__14_ //demsharpmix1_iTune[8:0]
	{0x0F12, 0x0087},  //0087	//AfitBaseVals_4__15_ //YUV422_DENOISE_iUVLowThresh[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_4__16_ //YUV422_DENOISE_iUVHighThresh[9:0]
	{0x0F12, 0x0032},  //0032	//AfitBaseVals_4__17_ //sddd8a_iClustThresh_H[9:0]
	{0x0F12, 0x003C},  //003C	//AfitBaseVals_4__18_ //sddd8a_iClustThresh_C[9:0]
	{0x0F12, 0x0073},  //0073	//AfitBaseVals_4__19_ //Sharpening_iLowSharpClamp[9:0]
	{0x0F12, 0x00B4},  //00B4	//AfitBaseVals_4__20_ //Sharpening_iHighSharpClamp[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_4__21_ //sddd8a_iClustThresh_H_Bin_1_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_4__22_ //sddd8a_iClustThresh_C_Bin_1_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_4__23_ //Sharpening_iLowSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0046},  //0046	//AfitBaseVals_4__24_ //Sharpening_iHighSharpClamp_Bin_1_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_4__25_ //sddd8a_iClustThresh_H_Bin_2_mode[9:0]
	{0x0F12, 0x0028},  //0028	//AfitBaseVals_4__26_ //sddd8a_iClustThresh_C_Bin_2_mode[9:0]
	{0x0F12, 0x0023},  //0023	//AfitBaseVals_4__27_ //Sharpening_iLowSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x0046},  //0046	//AfitBaseVals_4__28_ //Sharpening_iHighSharpClamp_Bin_2_mode[9:0]
	{0x0F12, 0x2B23},  //2B23	//AfitBaseVals_4__29_ //sddd8a_edge_low[7:0],sddd8a_repl_thresh[15:8]
	{0x0F12, 0x0601},  //0601	//AfitBaseVals_4__30_ //sddd8a_repl_force[7:0],sddd8a_iMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__31_ //sddd8a_iHotThreshHigh[7:0],sddd8a_iHotThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__32_ //sddd8a_iColdThreshHigh[7:0],sddd8a_iColdThreshLow[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__33_ //sddd8a_AddNoisePower1[7:0],sddd8a_AddNoisePower2[15:8]
	{0x0F12, 0x00FF},  //00FF	//AfitBaseVals_4__34_ //sddd8a_iSatSat[7:0],sddd8a_iRadialTune[15:8]
	{0x0F12, 0x0B84},  //0B84	//AfitBaseVals_4__35_ //sddd8a_iRadialLimit[7:0],sddd8a_iRadialPower[15:8]
	{0x0F12, 0xFFFF},  //FFFF	//AfitBaseVals_4__36_ //sddd8a_iLowMaxSlopeAllowed[7:0],sddd8a_iHighMaxSlopeAllowed[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__37_ //sddd8a_iLowSlopeThresh[7:0],sddd8a_iHighSlopeThresh[15:8]
	{0x0F12, 0x050D},  //050D	//AfitBaseVals_4__38_ //Demosaicing_iDFD_ReduceCoeff[7:0],Demosaicing_iDecisionThresh[15:8]
	{0x0F12, 0x1E80},  //1E80	//AfitBaseVals_4__39_ //Demosaicing_iCentGrad[7:0],Demosaicing_iMonochrom[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__40_ //Demosaicing_iGRDenoiseVal[7:0],Demosaicing_iGBDenoiseVal[15:8]
	{0x0F12, 0x0A00},  //0A00	//AfitBaseVals_4__41_ //Demosaicing_iNearGrayDesat[7:0],Sharpening_iWSharpen[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_4__42_ //Sharpening_iWShThresh[7:0],Sharpening_nSharpWidth[15:8]
	{0x0F12, 0xFF01},  //FF01	//AfitBaseVals_4__43_ //Sharpening_iReduceNegative[7:0],Sharpening_iShDespeckle[15:8]
	{0x0F12, 0x180F},  //180F	//AfitBaseVals_4__44_ //demsharpmix1_iBCoeff[7:0],demsharpmix1_iGCoeff[15:8]
	{0x0F12, 0x0001},  //0001	//AfitBaseVals_4__45_ //demsharpmix1_iFilterPower[7:0],demsharpmix1_iWideMult[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__46_ //demsharpmix1_iNarrMult[7:0],demsharpmix1_iRGBMultiplier[15:8]
	{0x0F12, 0x5A03},  //5A03	//AfitBaseVals_4__47_ //YUV422_DENOISE_iUVSupport[7:0],byr_cgras_sim_iShadingPower[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_4__48_ //RGBGamma2_1LUT_sim_iLinearity[7:0],RGBGamma2_1LUT_sim_iDarkReduce[15:8]
	{0x0F12, 0x0080},  //0080	//AfitBaseVals_4__49_ //ccm_oscar_sim_iSaturation[7:0],YUV422_CONTROL_Y_off[15:8]
	{0x0F12, 0x0280},  //0280	//AfitBaseVals_4__50_ //YUV422_CONTROL_Y_mul[7:0],sddd8a_nClustLevel_H[15:8]
	{0x0F12, 0x0108},  //0108	//AfitBaseVals_4__51_ //sddd8a_iClustMulT_H[7:0],sddd8a_iClustMulT_C[15:8]
	{0x0F12, 0x6464},  //6464	//AfitBaseVals_4__52_ //sddd8a_DispTH_Low[7:0],sddd8a_DispTH_High[15:8]
	{0x0F12, 0x1419},  //1419	//AfitBaseVals_4__53_ //sddd8a_iDenThreshLow[7:0],sddd8a_iDenThreshHigh[15:8]
	{0x0F12, 0x0A0A},  //0A0A	//AfitBaseVals_4__54_ //Demosaicing_iDemSharpenLow[7:0],Demosaicing_iDemSharpenHigh[15:8]
	{0x0F12, 0x0800},  //0800	//AfitBaseVals_4__55_ //Demosaicing_iDemSharpThresh[7:0],Demosaicing_iDespeckleForDemsharp[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__56_ //Demosaicing_iEdgeDesatThrLow[7:0],Demosaicing_iEdgeDesatThrHigh[15:8]
	{0x0F12, 0x0328},  //0328	//AfitBaseVals_4__57_ //Demosaicing_iEdgeDesat[7:0],Demosaicing_iEdgeDesatLimit[15:8]
	{0x0F12, 0x324C},  //324C	//AfitBaseVals_4__58_ //Demosaicing_iDemShLowLimit[7:0],Sharpening_iLowSharpPower[15:8]
	{0x0F12, 0x001E},  //001E	//AfitBaseVals_4__59_ //Sharpening_iHighSharpPower[7:0],Sharpening_iLowShDenoise[15:8]
	{0x0F12, 0x0200},  //0200	//AfitBaseVals_4__60_ //Sharpening_iHighShDenoise[7:0],demsharpmix1_iWideFiltReduce[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_4__61_ //demsharpmix1_iNarrFiltReduce[7:0],sddd8a_nClustLevel_H_Bin_1_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_4__62_ //sddd8a_iClustMulT_H_Bin_1_mode[7:0],sddd8a_iClustMulT_C_Bin_1_mode[15:8]
	{0x0F12, 0x6464},  //6464	//AfitBaseVals_4__63_ //sddd8a_DispTH_Low_Bin_1_mode[7:0],sddd8a_DispTH_High_Bin_1_mode[15:8]
	{0x0F12, 0x0F0F},  //0F0F	//AfitBaseVals_4__64_ //sddd8a_iDenThreshLow_Bin_1_mode[7:0],sddd8a_iDenThreshHigh_Bin_1_mode[15:8]
	{0x0F12, 0x0307},  //0307	//AfitBaseVals_4__65_ //Demosaicing_iDemSharpenLow_Bin_1_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_1_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_4__66_ //Demosaicing_iDemSharpThresh_Bin_1_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_1_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__67_ //Demosaicing_iEdgeDesatThrLow_Bin_1_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_1_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_4__68_ //Demosaicing_iEdgeDesat_Bin_1_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_1_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_4__69_ //Demosaicing_iDemShLowLimit_Bin_1_mode[7:0],Sharpening_iLowSharpPower_Bin_1_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_4__70_ //Sharpening_iHighSharpPower_Bin_1_mode[7:0],Sharpening_iLowShDenoise_Bin_1_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_4__71_ //Sharpening_iHighShDenoise_Bin_1_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_1_mode[15:8]
	{0x0F12, 0x0103},  //0103	//AfitBaseVals_4__72_ //demsharpmix1_iNarrFiltReduce_Bin_1_mode[7:0],sddd8a_nClustLevel_H_Bin_2_mode[15:8]
	{0x0F12, 0x010C},  //010C	//AfitBaseVals_4__73_ //sddd8a_iClustMulT_H_Bin_2_mode[7:0],sddd8a_iClustMulT_C_Bin_2_mode[15:8]
	{0x0F12, 0x6464},  //6464	//AfitBaseVals_4__74_ //sddd8a_DispTH_Low_Bin_2_mode[7:0],sddd8a_DispTH_High_Bin_2_mode[15:8]
	{0x0F12, 0x0F0F},  //0F0F	//AfitBaseVals_4__75_ //sddd8a_iDenThreshLow_Bin_2_mode[7:0],sddd8a_iDenThreshHigh_Bin_2_mode[15:8]
	{0x0F12, 0x0307},  //0307	//AfitBaseVals_4__76_ //Demosaicing_iDemSharpenLow_Bin_2_mode[7:0],Demosaicing_iDemSharpenHigh_Bin_2_mode[15:8]
	{0x0F12, 0x080F},  //080F	//AfitBaseVals_4__77_ //Demosaicing_iDemSharpThresh_Bin_2_mode[7:0],Demosaicing_iDespeckleForDemsharp_Bin_2_mode[15:8]
	{0x0F12, 0x0000},  //0000	//AfitBaseVals_4__78_ //Demosaicing_iEdgeDesatThrLow_Bin_2_mode[7:0],Demosaicing_iEdgeDesatThrHigh_Bin_2_mode[15:8]
	{0x0F12, 0x030F},  //030F	//AfitBaseVals_4__79_ //Demosaicing_iEdgeDesat_Bin_2_mode[7:0],Demosaicing_iEdgeDesatLimit_Bin_2_mode[15:8]
	{0x0F12, 0x3208},  //3208	//AfitBaseVals_4__80_ //Demosaicing_iDemShLowLimit_Bin_2_mode[7:0],Sharpening_iLowSharpPower_Bin_2_mode[15:8]
	{0x0F12, 0x0F1E},  //0F1E	//AfitBaseVals_4__81_ //Sharpening_iHighSharpPower_Bin_2_mode[7:0],Sharpening_iLowShDenoise_Bin_2_mode[15:8]
	{0x0F12, 0x020F},  //020F	//AfitBaseVals_4__82_ //Sharpening_iHighShDenoise_Bin_2_mode[7:0],demsharpmix1_iWideFiltReduce_Bin_2_mode[15:8]
	{0x0F12, 0x0003},  //0003	//AfitBaseVals_4__83_ //demsharpmix1_iNarrFiltReduce_Bin_2_mode[7:0]
	{0x0F12, 0x7F5E},  //7F5E	//ConstAfitBaseVals_0_
	{0x0F12, 0xFEEE},  //FEEE	//ConstAfitBaseVals_1_
	{0x0F12, 0xD9B7},  //D9B7	//ConstAfitBaseVals_2_
	{0x0F12, 0x0472},  //0472	//ConstAfitBaseVals_3_
	{0x0F12, 0x0001},  //0001	//ConstAfitBaseVals_4_

//==================================================================================
// 18.JPEG Thumnail Setting
//==================================================================================

//==================================================================================
// 19.Input Size Setting
//==================================================================================

//==================================================================================
// 20.Preview & Capture Configration Setting
//==================================================================================
//Preview config[0] 1280x960  30fps~7.5fps
	{0x002A, 0x01BE},
	{0x0F12, 0x0500},	//REG_0TC_PCFG_usWidth //500:1280; 280:640
	{0x0F12, 0x03C0},	//REG_0TC_PCFG_usHeight //3C0:960; 2D0:720; 1E0:480
	{0x0F12, 0x0005},	//REG_0TC_PCFG_Format //5:YUV422; 7:RAW10
	{0x002A, 0x01C8},
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uClockInd
	{0x002A, 0x01C4},
	{0x0F12, 0x0012},	//REG_0TC_PCFG_PVIMask //52:YUV422; 42:RAW10
	{0x002A, 0x01D4},
	{0x0F12, 0x0002},	//REG_0TC_PCFG_FrRateQualityType //1b:FR(bin) 2b:Quality(no-bin)
	{0x002A, 0x01D2},
	{0x0F12, 0x0000},	//REG_0TC_PCFG_usFrTimeType //0:dynamic; 1:fixed not accurate; 2:fixed accurate
	{0x002A, 0x01D8},
	{0x0F12, 0x0535},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A;
	{0x002A, 0x01D6},
	{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10
	{0x002A, 0x01E8},
#if defined(CONFIG_MACH_LYNX_DL45)
	{0x0F12, 0x0003},	//REG_0TC_PCFG_uPrevMirror
#else
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
#endif

//Capture config[0] 1280x960  7.5fps
	{0x002A, 0x02B0},
	{0x0F12, 0x0500},	//REG_0TC_CCFG_usWidth //500:1280; 280:640
	{0x0F12, 0x03C0},	//REG_0TC_CCFG_usHeight //3C0:960; 1E0:480
	{0x0F12, 0x0005},	//REG_0TC_CCFG_Format //5:YUV422; 7:RAW10
	{0x002A, 0x02BA},
	{0x0F12, 0x0000},	//REG_0TC_CCFG_uClockInd
	{0x002A, 0x02B6},
	{0x0F12, 0x0012},	//REG_0TC_CCFG_PVIMask //52:YUV422; 42:RAW10
	{0x002A, 0x02C6},
	{0x0F12, 0x0002},	//REG_0TC_CCFG_FrRateQualityType //1b:FR(bin) 2b:Quality(no-bin)
	{0x002A, 0x02C4},
	{0x0F12, 0x0002},	//REG_0TC_CCFG_usFrTimeType //0:dynamic; 1:fixed not accurate;  2:fixed accurate
	{0x002A, 0x02CA},
	{0x0F12, 0x0535},	//REG_0TC_CCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A;
	{0x002A, 0x02C8},
	{0x0F12, 0x0000},	//REG_0TC_CCFG_usMinFrTimeMsecMult10

#if !defined(CONFIG_MACH_ATK)
//Vod Control
	{0x002A, 0x0E20},
	{0x0F12, 0x0200},	//oif_Regphy8aa_ctll
#endif

//==================================================================================
// 21.Select Cofigration Display
//==================================================================================
//Preview
	{0x002A, 0x01A8},
	{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
	{0x002A, 0x01B0},
	{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
	{0x002A, 0x019E},
	{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
	{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
	{0x0028, 0xD000},
	{0x002A, 0x1000},
	{0x0F12, 0x0001},	//Set host interrupt

//===================================================================================
// 22. ESD Check
//===================================================================================

//===================================================================================
// 23.ISSUE
//===================================================================================

};

static struct msm_camera_i2c_reg_conf res0_reg_array[] = {
	{0x0028, 0x7000},
	{0x002A, 0x01CA},
	{0x0F12, 0x0000},	//REG_0TC_PCFG_Cfg_Input_Sizes_usWidth 
	{0x0F12, 0x0000},	//REG_0TC_PCFG_Cfg_Input_Sizes_usHeight
	{0x0F12, 0x0000},	//REG_0TC_PCFG_Cfg_Input_Ofs_usWidth
	{0x0F12, 0x0000},	//REG_0TC_PCFG_Cfg_Input_Ofs_usHeight

	{0x002A, 0x01BE},
	{0x0F12, 0x0500},	//REG_0TC_PCFG_usWidth //500:1280; 280:640
	{0x0F12, 0x03C0},	//REG_0TC_PCFG_usHeight //3C0:960; 2D0:720; 1E0:480
	{0x0F12, 0x0005},	//REG_0TC_PCFG_Format //5:YUV422; 7:RAW10
	{0x002A, 0x01D4},
	{0x0F12, 0x0002},	//REG_0TC_PCFG_FrRateQualityType //1b:FR(bin) 2b:Quality(no-bin)
	{0x002A, 0x01D2},
	{0x0F12, 0x0000},	//REG_0TC_PCFG_usFrTimeType //0:dynamic; 1:fixed not accurate; 2:fixed accurate
	{0x002A, 0x01D8},
	{0x0F12, 0x0535},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 12fps-0348; 7.5-0535; 6.0-0682; 3.75-0A6A;
	{0x002A, 0x01D6},
	{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10

	{0x002A, 0x01A8},
	{0x0F12, 0x0000},	//Preview Table0   #REG_TC_GP_ActivePrevConfig
	{0x002A, 0x01AC},
	{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
	{0x002A, 0x01A6},
	{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
	{0x002A, 0x01AA},
	{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged 
	{0x002A, 0x019E},
	{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
	{0x002A, 0x01A0},
	{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

static struct msm_camera_i2c_reg_conf s5k8aayx_start_settings[] = {
	{0x0028, 0x7000},
	{0x002A, 0x019E},
	{0x0F12, 0x0001},
	{0x0F12, 0x0001},
};

static struct msm_camera_i2c_reg_conf s5k8aayx_stop_settings[] = {
	{0x0028, 0x7000},
	{0x002A, 0x019E},
	{0x0F12, 0x0000},
	{0x0F12, 0x0001},
};


/* - EXPOSURE COMPENSATION - */
static struct msm_camera_i2c_reg_conf s5k8aayx_exposure[] = {
/* Brightness -12 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0012},

/* Brightness -11 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0014},

/* Brightness -10 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0016},

/* Brightness -9 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0018},

/* Brightness -8 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x001B},

/* Brightness -7 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x001E},

/* Brightness -6 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0023},

/* Brightness -5 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0026},

/* Brightness -4 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0029},

/* Brightness -3 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x002E},

/* Brightness -2 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0033},

/* Brightness -1 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0038},

/* Brightness 0 (Default) */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x003E},

/* Brightness +1 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0045},

/* Brightness +2 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x004C},

/* Brightness +3 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0054},

/* Brightness +4 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x005E},

/* Brightness +5 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0068},

/* Brightness +6 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0074},

/* Brightness +7 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0082},

/* Brightness +8 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x0090},

/* Brightness +9 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x009E},

/* Brightness +10 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x00AF},

/* Brightness +11 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x00C0},

/* Brightness +12 */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x0D40},  //TVAR_ae_BrAve
{0x0F12,0x00D2},

};

static int32_t current_fps_value = MSM_V4L2_FPS_30_7P5AUTO;

static struct msm_camera_i2c_reg_conf s5k8aayx_frame_rate_15fix[] = {
/* 15FIX */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x01D2},
{0x0F12,0x0002},    //REG_0TC_PCFG_usFrTimeType //0:dynamic; 1:fixed not accurate; 2:fixed accurate
{0x002A,0x01D6},
{0x0F12,0x0000},    //REG_0TC_PCFG_usMinFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x0F12,0x029B},    //REG_0TC_PCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x002A,0x01AA},    //REG_TC_GP_PrevConfigChanged
{0x0F12,0x0001},

};

static struct msm_camera_i2c_reg_conf s5k8aayx_frame_rate_auto[] = {
/* 30-7.5AUTO */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x01D2},
{0x0F12,0x0000},    //REG_0TC_PCFG_usFrTimeType //0:dynamic; 1:fixed not accurate; 2:fixed accurate
{0x002A,0x01D6},
{0x0F12,0x014E},    //REG_0TC_PCFG_usMinFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x0F12,0x0535},    //REG_0TC_PCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x002A,0x01AA},    //REG_TC_GP_PrevConfigChanged
{0x0F12,0x0001},

};

static struct msm_camera_i2c_reg_conf s5k8aayx_frame_rate_30fix[] = {
/* 30FIX */
{0xFCFC,0xD000},
{0x0028,0x7000},
{0x002A,0x01D2},
{0x0F12,0x0002},    //REG_0TC_PCFG_usFrTimeType //0:dynamic; 1:fixed not accurate; 2:fixed accurate
{0x002A,0x01D6},
{0x0F12,0x0000},    //REG_0TC_PCFG_usMinFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x0F12,0x014E},    //REG_0TC_PCFG_usMaxFrTimeMsecMult10 //30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
{0x002A,0x01AA},    //REG_TC_GP_PrevConfigChanged
{0x0F12,0x0001},

};

static struct v4l2_subdev_info s5k8aayx_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

int32_t s5k8aayx_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s", __func__);
	
	rc = msm_sensor_i2c_probe(client, id,&s5k8aayx_s_ctrl);
	if (rc < 0) {
		CDBG("%s msm_sensor_i2c_probe failed\n", __func__);
		return rc;
	}
	
	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (!s_ctrl) {
		pr_err("%s:%d sensor ctrl structure NULL\n", __func__,
			__LINE__);
		return -EINVAL;
	}
	
	s_ctrl->clk_info = s5k8aayx_clk_info;
	s_ctrl->clk_info_size = ARRAY_SIZE(s5k8aayx_clk_info);
	
	return rc;
}

static const struct i2c_device_id s5k8aayx_i2c_id[] = {
	{S5K8AAYX_SENSOR_NAME, (kernel_ulong_t)&s5k8aayx_s_ctrl},
	{ }
};

static struct i2c_driver s5k8aayx_i2c_driver = {
	.id_table = s5k8aayx_i2c_id,
	.probe  = s5k8aayx_sensor_i2c_probe,
	.driver = {
		.name = S5K8AAYX_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k8aayx_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id s5k8aayx_dt_match[] = {
	{.compatible = "qcom,s5k8aayx", .data = &s5k8aayx_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, s5k8aayx_dt_match);

static struct platform_driver s5k8aayx_platform_driver = {
	.driver = {
		.name = "qcom,s5k8aayx",
		.owner = THIS_MODULE,
		.of_match_table = s5k8aayx_dt_match,
	},
};

static int32_t s5k8aayx_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(s5k8aayx_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init s5k8aayx_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	perf_lock_init(&s5k8aayx_perf_lock, PERF_LOCK_729600KHz, "camera_s5k8aayx");
	rc = platform_driver_probe(&s5k8aayx_platform_driver,
		s5k8aayx_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&s5k8aayx_i2c_driver);
}

static void __exit s5k8aayx_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (s5k8aayx_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&s5k8aayx_s_ctrl);
		platform_driver_unregister(&s5k8aayx_platform_driver);
	} else
		i2c_del_driver(&s5k8aayx_i2c_driver);
	return;
}

int32_t s5k8aayx_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	void *data;
	
	CDBG("%s start\n", __func__);
	
	data = kmalloc(cdata->cfg.i2c_info.length, GFP_KERNEL);
	if(data == NULL){
		pr_err("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}
	
	CDBG("%s i2c_info.addr = 0x%0x\n", __func__, cdata->cfg.i2c_info.addr);
	CDBG("%s i2c_info.length = 0x%0x\n", __func__, cdata->cfg.i2c_info.length);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(s_ctrl->sensor_i2c_client, cdata->cfg.i2c_info.addr, data, cdata->cfg.i2c_info.length);
	if(rc < 0){
		pr_err("%s i2c_read_seq failed\n", __func__);
		kfree(data);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.i2c_info.data,
		data,
		cdata->cfg.i2c_info.length)){
		kfree(data);
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	kfree(data);
	
	return rc;
}

int32_t s5k8aayx_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_conf *reg_conf_tbl, uint16_t size,
	enum msm_camera_i2c_data_type data_type)
{
	long rc = 0;
	struct msm_camera_i2c_seq_reg_setting conf_sec_array;
	struct msm_camera_i2c_seq_reg_array *reg_sec_setting = NULL;
	struct msm_camera_i2c_seq_reg_array *pos_reg_sec_setting = NULL;
	int i;
	int cur_size, array_size;

	reg_sec_setting = kzalloc(size *
		(sizeof(struct msm_camera_i2c_seq_reg_array)),
		GFP_KERNEL);
	if (!reg_sec_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}

	if(data_type == MSM_CAMERA_I2C_WORD_DATA){
	
		pos_reg_sec_setting = reg_sec_setting;
		
		array_size = 1;
		conf_sec_array.addr_type = client->addr_type;
		conf_sec_array.delay = 0;
		conf_sec_array.reg_setting = reg_sec_setting;
		
		pos_reg_sec_setting->reg_addr = reg_conf_tbl->reg_addr;
		pos_reg_sec_setting->reg_data[0] = ((reg_conf_tbl->reg_data) & 0xFF00) >> 8;
		pos_reg_sec_setting->reg_data[1] = reg_conf_tbl->reg_data & 0xFF;
		cur_size = 2;
		reg_conf_tbl++;
		
		for (i = 1; i < size; i++) {
			if(reg_conf_tbl->reg_addr == 0x0F12){
				if(pos_reg_sec_setting->reg_addr != 0x0F12){
					pos_reg_sec_setting->reg_data_size = cur_size;
				
					pos_reg_sec_setting++;
					array_size++;
					
					cur_size = 0;
					pos_reg_sec_setting->reg_addr = reg_conf_tbl->reg_addr;
				}
				if(cur_size >= 6){
					pos_reg_sec_setting->reg_data_size = cur_size;
					
					pos_reg_sec_setting++;
					array_size++;
					
					cur_size = 0;
					pos_reg_sec_setting->reg_addr = reg_conf_tbl->reg_addr;
				}
				pos_reg_sec_setting->reg_data[cur_size + 0] = ((reg_conf_tbl->reg_data) & 0xFF00) >> 8;
				pos_reg_sec_setting->reg_data[cur_size + 1] = reg_conf_tbl->reg_data & 0xFF;
				cur_size+=2;
			} else {
				pos_reg_sec_setting->reg_data_size = cur_size;
				
				pos_reg_sec_setting++;
				array_size++;
				
				pos_reg_sec_setting->reg_addr = reg_conf_tbl->reg_addr;
				pos_reg_sec_setting->reg_data[0] = ((reg_conf_tbl->reg_data) & 0xFF00) >> 8;
				pos_reg_sec_setting->reg_data[1] = reg_conf_tbl->reg_data & 0xFF;
				cur_size = 2;
			}
			reg_conf_tbl++;
		}

		CDBG("%s array_size = %d", __func__, array_size);
		pos_reg_sec_setting->reg_data_size = cur_size;

		conf_sec_array.size = array_size;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_sec_array);
		
		CDBG("%s rc = %d", __func__, (int)rc);

	} else {

		CDBG("%s data_type = %d", __func__, data_type);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(client, reg_conf_tbl, size, data_type);
			
		CDBG("%s rc = %d", __func__, (int)rc);
	}
	
	kfree(reg_sec_setting);

	CDBG("%s end", __func__);
	
	return rc;
}

int32_t s5k8aayx_sensor_setfps_sub(struct msm_sensor_ctrl_t *s_ctrl, int32_t fps_mode)
{
	int32_t rc = 0;
	struct msm_camera_i2c_reg_conf *conf;
	uint16_t size;
	int delay;
	CDBG("%s:  fps_mode = %d\n", __func__, fps_mode);
	switch(fps_mode){
		case MSM_V4L2_FPS_15FIX:
			size = ARRAY_SIZE(s5k8aayx_frame_rate_15fix);
			conf = &s5k8aayx_frame_rate_15fix[0];
			break;
		case MSM_V4L2_FPS_30_7P5AUTO:
			size = ARRAY_SIZE(s5k8aayx_frame_rate_auto);
			conf = &s5k8aayx_frame_rate_auto[0];
			break;
		case MSM_V4L2_FPS_30FIX:
			size = ARRAY_SIZE(s5k8aayx_frame_rate_30fix);
			conf = &s5k8aayx_frame_rate_30fix[0];
			break;
		default:
			rc = -EFAULT;
			break;
	}
	delay = 0;
	rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, conf, size, MSM_CAMERA_I2C_WORD_DATA);
	current_fps_value = fps_mode;
	return rc;
}

int32_t s5k8aayx_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
		case CFG_GET_SENSOR_INFO:
			memcpy(cdata->cfg.sensor_info.sensor_name,
				s_ctrl->sensordata->sensor_name,
				sizeof(cdata->cfg.sensor_info.sensor_name));
			cdata->cfg.sensor_info.session_id =
				s_ctrl->sensordata->sensor_info->session_id;
			for (i = 0; i < SUB_MODULE_MAX; i++)
				cdata->cfg.sensor_info.subdev_id[i] =
					s_ctrl->sensordata->sensor_info->subdev_id[i];
			CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
				cdata->cfg.sensor_info.sensor_name);
			CDBG("%s:%d session id %d\n", __func__, __LINE__,
				cdata->cfg.sensor_info.session_id);
			for (i = 0; i < SUB_MODULE_MAX; i++)
				CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
					cdata->cfg.sensor_info.subdev_id[i]);

			break;
		case CFG_SET_INIT_SETTING:
			/* 1. Write Recommend settings */
			/* 2. Write change settings */
			rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, init_reg_array, ARRAY_SIZE(init_reg_array), MSM_CAMERA_I2C_WORD_DATA);
			current_fps_value = MSM_V4L2_FPS_30_7P5AUTO;
			break;
		case CFG_SET_RESOLUTION:
			rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, res0_reg_array, ARRAY_SIZE(res0_reg_array), MSM_CAMERA_I2C_WORD_DATA);
			if(current_fps_value != MSM_V4L2_FPS_30_7P5AUTO) {
				rc = s5k8aayx_sensor_setfps_sub(s_ctrl, current_fps_value);
			}
			break;
		case CFG_SET_STOP_STREAM:
			rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, s5k8aayx_stop_settings, ARRAY_SIZE(s5k8aayx_stop_settings), MSM_CAMERA_I2C_WORD_DATA);
			break;
		case CFG_SET_START_STREAM:
			rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, s5k8aayx_start_settings, ARRAY_SIZE(s5k8aayx_start_settings), MSM_CAMERA_I2C_WORD_DATA);
			break;
		case CFG_GET_SENSOR_INIT_PARAMS:
			cdata->cfg.sensor_init_params =
				*s_ctrl->sensordata->sensor_init_params;
			CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
				__LINE__,
				cdata->cfg.sensor_init_params.modes_supported,
				cdata->cfg.sensor_init_params.position,
				cdata->cfg.sensor_init_params.sensor_mount_angle);
			break;
		case CFG_SET_SLAVE_INFO: {
			struct msm_camera_sensor_slave_info sensor_slave_info;
			struct msm_sensor_power_setting_array *power_setting_array;
			int slave_index = 0;
			if (copy_from_user(&sensor_slave_info,
			    (void *)cdata->cfg.setting,
			    sizeof(struct msm_camera_sensor_slave_info))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			/* Update sensor slave address */
			if (sensor_slave_info.slave_addr) {
				s_ctrl->sensor_i2c_client->cci_client->sid =
					sensor_slave_info.slave_addr >> 1;
			}

			/* Update sensor address type */
			s_ctrl->sensor_i2c_client->addr_type =
				sensor_slave_info.addr_type;

			/* Update power up / down sequence */
			s_ctrl->power_setting_array =
				sensor_slave_info.power_setting_array;
			power_setting_array = &s_ctrl->power_setting_array;
			power_setting_array->power_setting = kzalloc(
				power_setting_array->size *
				sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
			if (!power_setting_array->power_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(power_setting_array->power_setting,
			    (void *)sensor_slave_info.power_setting_array.power_setting,
			    power_setting_array->size *
			    sizeof(struct msm_sensor_power_setting))) {
				kfree(power_setting_array->power_setting);
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->free_power_setting = true;
			CDBG("%s sensor id %x\n", __func__,
				sensor_slave_info.slave_addr);
			CDBG("%s sensor addr type %d\n", __func__,
				sensor_slave_info.addr_type);
			CDBG("%s sensor reg %x\n", __func__,
				sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
			CDBG("%s sensor id %x\n", __func__,
				sensor_slave_info.sensor_id_info.sensor_id);
			for (slave_index = 0; slave_index <
				power_setting_array->size; slave_index++) {
				CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
					slave_index,
					power_setting_array->power_setting[slave_index].
					seq_type,
					power_setting_array->power_setting[slave_index].
					seq_val,
					power_setting_array->power_setting[slave_index].
					config_val,
					power_setting_array->power_setting[slave_index].
					delay);
			}
			kfree(power_setting_array->power_setting);
			break;
		}
		case CFG_WRITE_I2C_ARRAY: {
			struct msm_camera_i2c_reg_setting conf_array;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;

			if (copy_from_user(&conf_array,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = kzalloc(conf_array.size *
				(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
				conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}

			conf_array.reg_setting = reg_setting;
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
			kfree(reg_setting);
			break;
		}
		case CFG_WRITE_I2C_SEQ_ARRAY: {
			struct msm_camera_i2c_seq_reg_setting conf_array;
			struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

			if (copy_from_user(&conf_array,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_seq_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = kzalloc(conf_array.size *
				(sizeof(struct msm_camera_i2c_seq_reg_array)),
				GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
				conf_array.size *
				sizeof(struct msm_camera_i2c_seq_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}

			conf_array.reg_setting = reg_setting;
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_seq_table(s_ctrl->sensor_i2c_client,
				&conf_array);
			kfree(reg_setting);
			break;
		}

		case CFG_POWER_UP:
			if (s_ctrl->func_tbl->sensor_power_up)
				rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			else
				rc = -EFAULT;
			break;

		case CFG_POWER_DOWN:
			if (s_ctrl->func_tbl->sensor_power_down)
				rc = s_ctrl->func_tbl->sensor_power_down(
					s_ctrl);
			else
				rc = -EFAULT;
			break;

		case CFG_SET_STOP_STREAM_SETTING: {
			struct msm_camera_i2c_reg_setting *stop_setting =
				&s_ctrl->stop_setting;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;
			if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
			    sizeof(struct msm_camera_i2c_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = stop_setting->reg_setting;
			stop_setting->reg_setting = kzalloc(stop_setting->size *
				(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!stop_setting->reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(stop_setting->reg_setting,
			    (void *)reg_setting, stop_setting->size *
			    sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(stop_setting->reg_setting);
				stop_setting->reg_setting = NULL;
				stop_setting->size = 0;
				rc = -EFAULT;
				break;
			}
			break;
		}
		case SHCFG_GET_I2C_DATA:
			rc = s5k8aayx_i2c_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k8aayx_i2c_read failed", __func__);
			}
			break;

		case CFG_SET_EXPOSURE_COMPENSATION: {
			int32_t exposure_comp_level;
			struct msm_camera_i2c_reg_conf *conf;
			uint16_t size;
			int delay;
			if (copy_from_user(&exposure_comp_level, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if(exposure_comp_level < -12) {
				exposure_comp_level = -12;
			} else if(exposure_comp_level > 12) {
				exposure_comp_level = 12;
			}
			exposure_comp_level += 12;
			size = ARRAY_SIZE(s5k8aayx_exposure) / 25;
			conf = &s5k8aayx_exposure[exposure_comp_level * size];
			delay = 0;
			rc = s5k8aayx_i2c_write(s_ctrl, s_ctrl->sensor_i2c_client, conf, size, MSM_CAMERA_I2C_WORD_DATA);
			usleep_range(delay*1000, (delay+1)*1000);
			break;
		}

		case SHCFG_SET_FPS: {
			int32_t fps_mode;
			if (copy_from_user(&fps_mode, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			rc = s5k8aayx_sensor_setfps_sub(s_ctrl, fps_mode);
			break;
		}

		default:
			rc = -EFAULT;
			break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t s5k8aayx_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s\n", __func__);
	
	CDBG("%s MSM_TLMM_BASE + 0x2024 = 0x%0x\n", __func__, (unsigned int)MSM_TLMM_BASE + 0x2024);
	CDBG("%s __raw_readl = 0x%0x\n", __func__, __raw_readl((MSM_TLMM_BASE + 0x2024)));
	
	__raw_writel(__raw_readl((MSM_TLMM_BASE + 0x2024)) | 0x02, (MSM_TLMM_BASE + 0x2024));
	
	s_ctrl->clk_info = s5k8aayx_clk_info;
	s_ctrl->clk_info_size = ARRAY_SIZE(s5k8aayx_clk_info);
	
	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n",
			__func__);
		return 0;
	}

	perf_lock(&s5k8aayx_perf_lock);
	return rc;
}

int32_t s5k8aayx_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("%s\n", __func__);
	msm_sensor_power_down(s_ctrl);
	
	CDBG("%s MSM_TLMM_BASE + 0x2024 = 0x%0x\n", __func__, (unsigned int)MSM_TLMM_BASE + 0x2024);
	CDBG("%s __raw_readl = 0x%0x\n", __func__, __raw_readl(MSM_TLMM_BASE + 0x2024));
	
	__raw_writel(__raw_readl((MSM_TLMM_BASE + 0x2024)) & ~(0x02), (MSM_TLMM_BASE + 0x2024));

	perf_unlock(&s5k8aayx_perf_lock);

	return 0;
}

int32_t s5k8aayx_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0xFCFC,
			0x0000, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: i2c_write failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static struct msm_sensor_fn_t s5k8aayx_sensor_func_tbl = {
	.sensor_config = s5k8aayx_sensor_config,
	.sensor_power_up = s5k8aayx_sensor_power_up,
	.sensor_power_down = s5k8aayx_sensor_power_down,
	.sensor_match_id = s5k8aayx_sensor_match_id,
};

static struct msm_sensor_ctrl_t s5k8aayx_s_ctrl = {
	.sensor_i2c_client = &s5k8aayx_sensor_i2c_client,
	.power_setting_array.power_setting = s5k8aayx_power_setting,
	.power_setting_array.size = ARRAY_SIZE(s5k8aayx_power_setting),
	.msm_sensor_mutex = &s5k8aayx_mut,
	.sensor_v4l2_subdev_info = s5k8aayx_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k8aayx_subdev_info),
	.func_tbl = &s5k8aayx_sensor_func_tbl,
};

module_init(s5k8aayx_init_module);
module_exit(s5k8aayx_exit_module);
MODULE_DESCRIPTION("s5k8aayx");
MODULE_LICENSE("GPL v2");
