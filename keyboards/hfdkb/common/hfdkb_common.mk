COMMON_DIR = $(TOP_DIR)/keyboards/hfdkb/common

SRC += \
    $(COMMON_DIR)/hfdkb_common.c \
    $(COMMON_DIR)/hfdkb_indicator.c

VPATH += $(COMMON_DIR)
