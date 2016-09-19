#Android makefile to build kernel as a part of Android Build
PERL		= perl

ifeq ($(TARGET_PREBUILT_RECOVERY_KERNEL),)

RECOVERY_KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/RECOVERY_KERNEL_OBJ
RECOVERY_KERNEL_CONFIG := $(RECOVERY_KERNEL_OUT)/.config
TARGET_PREBUILT_INT_RECOVERY_KERNEL := $(RECOVERY_KERNEL_OUT)/arch/arm/boot/zImage
RECOVERY_KERNEL_HEADERS_INSTALL := $(RECOVERY_KERNEL_OUT)/usr
RECOVERY_KERNEL_MODULES_INSTALL := system
# RECOVERY_KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
RECOVERY_KERNEL_IMG=$(RECOVERY_KERNEL_OUT)/arch/arm/boot/Image

RECOVERY_DTS_NAMES ?= $(shell $(PERL) -e 'while (<>) {$$a = $$1 if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/; $$r = $$1 if /CONFIG_MSM_SOC_REV_(?!NONE)(\w+)=y/; $$arch = $$arch.lc("$$a$$r ") if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/} print $$arch;' $(RECOVERY_KERNEL_CONFIG))
RECOVERY_KERNEL_USE_OF ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_USE_OF=y/) { $$of = "y"; break; } } print $$of;' kernel/arch/arm/configs/$(RECOVERY_KERNEL_DEFCONFIG))

ifeq "$(RECOVERY_KERNEL_USE_OF)" "y"
RECOVERY_DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/$(DTS_NAME)*.dts)
RECOVERY_DTS_FILE = $(lastword $(subst /, ,$(1)))
RECOVERY_DTB_FILE = $(addprefix $(RECOVERY_KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%.dtb,$(call DTS_FILE,$(1))))
RECOVERY_ZIMG_FILE = $(addprefix $(RECOVERY_KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%-zImage,$(call DTS_FILE,$(1))))
RECOVERY_KERNEL_ZIMG = $(RECOVERY_KERNEL_OUT)/arch/arm/boot/zImage
DTC = $(RECOVERY_KERNEL_OUT)/scripts/dtc/dtc

define append-dtb-recovery
mkdir -p $(RECOVERY_KERNEL_OUT)/arch/arm/boot;\
$(foreach DTS_NAME, $(DTS_NAMES), \
   $(foreach d, $(DTS_FILES), \
      $(DTC) -p 1024 -O dtb -o $(call DTB_FILE,$(d)) $(d); \
      cat $(RECOVERY_KERNEL_ZIMG) $(call DTB_FILE,$(d)) > $(call ZIMG_FILE,$(d));))
endef
else

define append-dtb-recovery
endef
endif

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_RECOVERY_KERNEL := $(RECOVERY_KERNEL_OUT)/piggy
else
TARGET_PREBUILT_RECOVERY_KERNEL := $(TARGET_PREBUILT_INT_RECOVERY_KERNEL)
endif

$(RECOVERY_KERNEL_OUT):
	mkdir -p $(RECOVERY_KERNEL_OUT)

define RECOVERY_CONFIGY
"CONFIG_ANDROID_RECOVERY_BUILD"
endef
define RECOVERY_CONFIGN
""
endef
define RECOVERY_CONFIGYOP
""
endef
define format_kernel_config_engineering_recovery
	perl -le 'if ($$ENV{'SH_BUILD_DEBUG'} eq "y") {@noappear = split(/ /, ($(RECOVERY_CONFIGY) . " " . $(RECOVERY_CONFIGYOP))); } else {@noappear = split(/ /, $(RECOVERY_CONFIGY));} @config_y = @noappear;\
	@config_n = split(/ /, $(RECOVERY_CONFIGN));\
	while (<>) {chomp($$_); $$line = $$_ ; s/^# // ; s/[ =].+$$// ; if (/^CONFIG/) { $$config = $$_ ; \
	if (grep {$$_ eq $$config} @config_y) { $$line = $$_ . "=y" ; @noappear = grep(!/^$$config$$/, @noappear); } \
	elsif (grep {$$_ eq $$config} @config_n) {$$line = "# " . $$_ . " is not set" ; } } \
	print $$line }\
	foreach (@noappear) { print $$_ . "=y"}' $(1) > $(RECOVERY_KERNEL_OUT)/tmp
	rm $(1)
	cp $(RECOVERY_KERNEL_OUT)/tmp $(1)
	$(MAKE) -C kernel O=../$(RECOVERY_KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- oldnoconfig
endef

$(RECOVERY_KERNEL_CONFIG): $(RECOVERY_KERNEL_OUT)
	$(MAKE) -C kernel O=../$(RECOVERY_KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(RECOVERY_KERNEL_DEFCONFIG)
	$(call format_kernel_config_engineering_recovery, $(RECOVERY_KERNEL_CONFIG))

$(RECOVERY_KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_RECOVERY_KERNEL)
	$(hide) gunzip -c $(RECOVERY_KERNEL_OUT)/arch/arm/boot/compressed/piggy.gzip > $(RECOVERY_KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_RECOVERY_KERNEL): $(RECOVERY_KERNEL_OUT) $(RECOVERY_KERNEL_CONFIG) $(RECOVERY_KERNEL_HEADERS_INSTALL)
	$(MAKE) -C kernel O=../$(RECOVERY_KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
	$(append-dtb-recovery)

$(RECOVERY_KERNEL_HEADERS_INSTALL): $(RECOVERY_KERNEL_OUT) $(RECOVERY_KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(RECOVERY_KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install

endif
