SRC += common/bt_task.c
SRC += common/retarget_suspend.c
SRC += common/lp_sleep.c

VPATH += $(TOP_DIR)/keyboards/hfdkb/m89u/sync_a1/common

LDFLAGS += -L $(TOP_DIR)/keyboards/hfdkb/m89u/sync_a1/common -l_bts
