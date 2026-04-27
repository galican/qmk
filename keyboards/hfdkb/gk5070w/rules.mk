
ifeq ($(strip $(CONSOLE_ENABLE)), yes)
    KEYBOARD_SHARED_EP = no
endif

MULTIMODE_ENABLE ?= yes

MODULE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
include $(MODULE_DIR)/common/common.mk
include $(MODULE_DIR)/led/led.mk
include $(MODULE_DIR)/lcd_drv/lcd_drv.mk
