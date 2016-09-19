/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>


static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};
#if 0
static struct gpiomux_setting gpio_suspend_config[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,  /* IN-NP */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,  /* O-LOW */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
};
#endif
static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting taiko_reset = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting taiko_int = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
#if 0
static struct gpiomux_setting mhl_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mhl_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config msm_mhl_configs[] __initdata = {
	{
		/* mhl-sii8334 intr */
		.gpio = 82,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_suspend_config,
			[GPIOMUX_ACTIVE]    = &mhl_active_1_cfg,
		},
	},
};
#endif
static struct gpiomux_setting gpio_uart7_active_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_uart7_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_blsp2_uart7_configs[] __initdata = {
	{
		.gpio	= 43,	/* BLSP2 UART7 CTS */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 44,	/* BLSP2 UART7 RFR */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8974_slimbus_config[] __initdata = {
	{
		.gpio	= 70,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 71,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};
#if 0
static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 15, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 16, /* CAM_MCLK2 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 21, /* CCI_I2C_SDA1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 22, /* CCI_I2C_SCL1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 24, /* FLASH_LED_NOW */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};
#endif
static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_taiko_config[] __initdata = {
	{
		.gpio	= 63,		/* SYS_RST_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_reset,
		},
	},
	{
		.gpio	= 72,		/* CDC_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_int,
		},
	},
};

/* suspended */
static struct gpiomux_setting sh_sus_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_4ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_8ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_sus_func2_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func2_pu_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting sh_sus_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func3_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func4_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func4_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func5_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_np_6ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/* active */
static struct gpiomux_setting sh_act_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_4ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_8ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_act_func2_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func2_pu_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting sh_act_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func3_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func4_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func4_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func5_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_act_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sh_msm8974_gpio_configs[] __initdata = {
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_4ma_cfg,
		},
	},
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_8ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_8ma_cfg,
		},
	},
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_8ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_8ma_cfg,
		},
	},
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func4_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func4_np_6ma_cfg,
		},
	},
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func5_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func5_np_6ma_cfg,
		},
	},
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func4_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func4_np_2ma_cfg,
		},
	},
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_2ma_cfg,
		},
	},
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func2_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func2_np_2ma_cfg,
		},
	},
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func2_pu_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func2_pu_2ma_cfg,
		},
	},
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 50,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 57,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 59,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 61,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 62,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 74,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 75,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 76,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 79,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 82,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 83,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_6ma_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_6ma_cfg,
		},
	},
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 103,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 114,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 115,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 118,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 120,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 121,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 122,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 124,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 125,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 126,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 127,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 129,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 130,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 131,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 132,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 135,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 136,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 137,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 144,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 145,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
};

void __init msm_8974_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	msm_gpiomux_install(msm_blsp2_uart7_configs,
			 ARRAY_SIZE(msm_blsp2_uart7_configs));
	msm_gpiomux_install(wcnss_5wire_interface,
				ARRAY_SIZE(wcnss_5wire_interface));

	msm_gpiomux_install(msm8974_slimbus_config,
			ARRAY_SIZE(msm8974_slimbus_config));
#if 0
	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
#endif
	msm_gpiomux_install(msm_taiko_config, ARRAY_SIZE(msm_taiko_config));
#if 0
	if (of_board_is_fluid())
		msm_gpiomux_install(msm_mhl_configs,
				    ARRAY_SIZE(msm_mhl_configs));
#endif
	sh_msm_gpiomux_install(sh_msm8974_gpio_configs,
			ARRAY_SIZE(sh_msm8974_gpio_configs));
}
