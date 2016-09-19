/* include/sharp/sh_gpio.h
 *
 * Copyright (C) 2012 Sharp Corporation
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
 
#ifndef __SH_GPIO_H__
#define __SH_GPIO_H__

#include <linux/types.h>
#include <linux/ioctl.h>


#define SH_GPIO_DEVFILE "/dev/sh_gpio"

typedef struct {
  uint32_t 	gpio;
  uint8_t 	func;
  uint8_t 	dir;
  uint8_t 	pull;
  uint8_t 	drvstr;
  uint32_t 	outval;
}sh_gpio_gpio_config;

typedef struct {
  uint32_t 	enable;
  uint32_t 	trigger;
}sh_gpio_gpioint_info;

struct sh_gpio_read_write {
/*
 * LINUX/android/hardware/sharp/shsys/sysprobe/sysprobe_v01.h
 */
 	uint32_t flag;										/* Update Flag */
#define SH_GPIO_CONFIG_BIT_FLAG_FUNC_V01 	0x00000001	/* Function */
#define SH_GPIO_CONFIG_BIT_FLAG_DIR_V01 	0x00000002	/* Direction */
#define SH_GPIO_CONFIG_BIT_FLAG_PULL_V01 	0x00000004	/* Pull */
#define SH_GPIO_CONFIG_BIT_FLAG_DRIVE_V01 	0x00000008	/* DriverStrength */
#define SH_GPIO_CONFIG_BIT_FLAG_RMT_V01 	0x00000010	/* OutValue */

	sh_gpio_gpio_config  	gpio_config	;	/* Config */
	sh_gpio_gpioint_info	gpioint_info;  	/* GPIO Int */
};

/*
 * PMIC-GPIO/MPP
 */
struct sh_pm_gpio {
  	uint32_t 	gpio;
	int			direction;
	int			output_buffer;
	int			output_value;
	int			pull;
	int			vin_sel;
	int			out_strength;
	int			function;
	int			inv_int_pol;
	int			disable_pin;
};
struct sh_pm8xxx_mpp_config_data {
  	uint32_t 	gpio;
	int			output_value;
	unsigned	type;
	unsigned	level;
	unsigned	control;
};

struct sh_qpnp_gpio {
  	uint32_t 	gpio;
	int			mode		;
	int			output_type	;
	int			invert		;
	int			pull		;
	int			vin_sel		;
	int			out_strength;
	int			src_sel		;
	int			master_en	;
	int			aout_ref	;
	int			ain_route	;
	int			cs_out		;
};
#define SH_GPIO_IOCTL (0x91) /* Magic number for SH_GPIO Iocts */

#define SH_GPIO_READ  				_IOR(SH_GPIO_IOCTL, 0x1, struct sh_gpio_read_write)
#define SH_GPIO_WRITE 				_IOW(SH_GPIO_IOCTL, 0x2, struct sh_gpio_read_write)
#define SH_GPIO_READ_PMIC_GPIO  	_IOR(SH_GPIO_IOCTL, 0x3, struct sh_qpnp_gpio)
#define SH_GPIO_WRITE_PMIC_GPIO 	_IOW(SH_GPIO_IOCTL, 0x4, struct sh_qpnp_gpio)
#define SH_GPIO_READ_PMIC_MPP  		_IOR(SH_GPIO_IOCTL, 0x5, struct sh_pm8xxx_mpp_config_data)
#define SH_GPIO_WRITE_PMIC_MPP 		_IOW(SH_GPIO_IOCTL, 0x6, struct sh_pm8xxx_mpp_config_data)


#endif // __SH_GPIO_H__
