ifeq ($(strip $(CONSOLE_ENABLE)), yes)
    KEYBOARD_SHARED_EP = no
endif

HFDKB_WIRELESS_DIR := keyboards/hfdkb_wireless

OPT_DEFS += -DBT_MODE_ENABLE
OPT_DEFS += -DENTRY_STOP_MODE
OPT_DEFS += -DNO_USB_STARTUP_CHECK

UART_DRIVER_REQUIRED = yes

SRC += $(HFDKB_WIRELESS_DIR)/common/bt_task.c
SRC += $(HFDKB_WIRELESS_DIR)/common/lp_sleep.c
SRC += $(HFDKB_WIRELESS_DIR)/common/retarget_suspend.c

LDFLAGS += -L $(HFDKB_WIRELESS_DIR)/common -l_bts
