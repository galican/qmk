SRC += common/bt_task.c
SRC += common/retarget_suspend.c
SRC += common/lp_sleep.c

VPATH += $(TOP_DIR)/keyboards/hfdkb/m89u/async/common

LDFLAGS += -L $(TOP_DIR)/keyboards/hfdkb/m89u/async/common -l_bts
