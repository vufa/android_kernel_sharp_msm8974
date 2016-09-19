# miyabi
obj-$(CONFIG_SECURITY_MIYABI)		+= miyabi.o miyabi_common.o miyabi_sandbox.o miyabi_sysfs.o mnhelper.o

ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DLOCAL_MIYABI_FBUILD
endif

