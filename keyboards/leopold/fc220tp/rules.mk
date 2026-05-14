ifeq ($(strip $(CONSOLE_ENABLE)), yes)
    KEYBOARD_SHARED_EP = no
endif

OPT_DEFS += -DMULTIMODE_ENABLE
OPT_DEFS += -DENTRY_STOP_MODE
OPT_DEFS += -DNO_USB_STARTUP_CHECK

UART_DRIVER_REQUIRED = yes

SRC += common/bt_task.c
SRC += common/retarget_suspend.c
SRC += common/lp_sleep.c

VPATH += $(TOP_DIR)/keyboards/hfdkb/m89u/async/common

LDFLAGS += -L $(TOP_DIR)/keyboards/hfdkb/m89u/async/common -l_bts
