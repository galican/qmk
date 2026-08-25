KEYBOARD_SHARED_EP = yes

RULES_MK_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

OPT_DEFS += -DMULTIMODE_ENABLE
OPT_DEFS += -DENTRY_STOP_MODE
OPT_DEFS += -DNO_USB_STARTUP_CHECK
# OPT_DEFS += -DWWDG_ENABLE

UART_DRIVER_REQUIRED = yes

SRC += common/bt_task.c
SRC += common/retarget_suspend.c
SRC += common/lp_sleep.c
# SRC += common/wb32_wwdg.c
# SRC += common/wwdg.c
SRC += bled/bled.c

LDFLAGS += -L $(RULES_MK_DIR)/common -l_bts

VPATH += $(RULES_MK_DIR)/common
VPATH += $(RULES_MK_DIR)/bled
