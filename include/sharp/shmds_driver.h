/* include/sharp/shmds_driver.h  (MotionSensor Driver)
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

#ifndef SHMDS_H
#define SHMDS_H

#if defined(CONFIG_SENSORS_AMI603)

#include "shmds_driver_ami.h"

#elif defined(CONFIG_MPU_SENSORS_MPU9150) || defined(CONFIG_MPU_SENSORS_MPU3050)

#include "shmds_driver_mpu.h"

#endif /* defined(CONFIG_SENSORS_AMI603) */

#endif
