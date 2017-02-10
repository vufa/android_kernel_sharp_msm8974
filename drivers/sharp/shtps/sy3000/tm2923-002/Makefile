ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_SHTPS_SY3000_TM2923_002)	+= shtps_rmi_spi.o shtps_rmi_devctl.o
