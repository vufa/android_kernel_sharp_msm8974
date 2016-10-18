ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHDISP_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_mipi.o

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_kerl.o

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_system.o

ifeq ($(CONFIG_SHDISP_PANEL_GEMINI),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_lm3533.o
else
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_bl69y6.o
endif

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_pm.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_panel.o

ifeq ($(CONFIG_SHDISP_PANEL_ANDY),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_andy.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_MARCO),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_marco.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_CARIN),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_carin.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_GEMINI),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_gemini.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_RYOMA),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_ryoma.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_NULL),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_null_panel.o
endif

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_clmr.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_clmr_usr.o

ifeq ($(CONFIG_SHLCDC_LED_BD2802GU),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_bd2802gu.o
endif

obj-$(CONFIG_SHLCDC_BOARD) += shdisp_dbg.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
