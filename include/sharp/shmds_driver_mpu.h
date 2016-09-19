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

#ifndef SHMDS_MPU_H
#define SHMDS_MPU_H

#if defined(CONFIG_MPU_SENSORS_MPU3050)
void SHMDS_Pedometer_ReStart(void)
{
}
void SHMDS_Pedometer_Pause(void)
{
}
void SHMDS_SetFlipInformation(unsigned char position)
{
    (void)position;
}
#elif defined(CONFIG_MPU_SENSORS_MPU9150)

enum {
	SHMDS_RESULT_SUCCESS = 0,
	SHMDS_RESULT_FAILURE = -1,
};

struct shmds_gyro_data {
	unsigned long enable;
	signed short x;
	signed short y;
	signed short z;
};

struct shmds_accl_data {
	unsigned long enable;
	signed short x;
	signed short y;
	signed short z;
};

struct shmds_sensor_data {
	struct shmds_gyro_data gyro_data;
	struct shmds_accl_data accl_data;
};

extern void SHMDS_Pedometer_ReStart(void);
extern void SHMDS_Pedometer_Pause(void);
extern void SHMDS_SetFlipInformation(unsigned char position);
extern int SHMDS_Sensor_Read(struct shmds_sensor_data *sensor_data);
#endif /* defined(CONFIG_MPU_SENSORS_MPU9150) */

#endif
