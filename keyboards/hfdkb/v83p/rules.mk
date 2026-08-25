ifeq ($(strip $(CONSOLE_ENABLE)), yes)
    KEYBOARD_SHARED_EP = yes
endif

UART_DRIVER_REQUIRED = yes

SRC += common/bt_task.c
SRC += common/lp_sleep.c
SRC += common/retarget_suspend.c

RULES_MK_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LDFLAGS += -L $(RULES_MK_DIR)/common -l_bts
