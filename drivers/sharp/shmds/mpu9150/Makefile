#
# Makefile for Invensense inv-mpu device.
#
# shmds mod 1-2 S
obj-$(CONFIG_MPU_SENSORS_MPU9150) += inv-mpu.o
# shmds mod 1-2 E

inv-mpu-objs := inv_gyro.o 
inv-mpu-objs += inv_gyro_misc.o
inv-mpu-objs += inv_mpu3050.o
inv-mpu-objs += dmpDefaultMPU6050.o

# the Bosch BMA250 driver is added to the inv-mpu device driver because it
# must be connected to an MPU3050 device on the secondary slave bus.
inv-mpu-objs += inv_slave_bma250.o

# shmds add 9-1 11-1 S
EXTRA_CFLAGS += -D SHMDS_DETECT -D SHMDS_CAF
# shmds add 9-1 11-1 E