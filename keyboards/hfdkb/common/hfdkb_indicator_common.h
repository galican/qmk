#pragma once

#include "hfdkb_indicator.h"
#include "hfdkb_indicator_config.h"

#ifdef RGB_MATRIX_ENABLE

static const indicator_config_t ind_caps_config = {
    .first_led = LED_CAPS_LOCK_INDEX,
    .led_count = 1,
    .effect    = IND_SOLID,
    .color     = {0x77, 0x77, 0x77},
    .priority  = 40,
};

static const indicator_config_t ind_gui_config = {
    .first_led = LED_GUI_LOCK_INDEX,
    .led_count = 1,
    .effect    = IND_SOLID,
    .color     = {0x77, 0x77, 0x77},
    .priority  = 40,
};

static const indicator_config_t ind_low_bat_config = {
    .first_led = HFDKB_IND_LOW_BAT_LED,
    .led_count = 1,
    .effect    = IND_BLINK,
    .color     = {0x77, 0, 0},
    .on_ms     = HFDKB_IND_LOW_BAT_ON_MS,
    .off_ms    = HFDKB_IND_LOW_BAT_OFF_MS,
    .priority  = 80,
    .off_mode  = IND_OFF_BLACK,
};

static const indicator_config_t ind_reset_config = {
    .first_led   = HFDKB_IND_KEY_FIRST,
    .led_count   = HFDKB_IND_KEY_COUNT,
    .effect      = IND_BLINK,
    .color       = {0x77, 0, 0},
    .on_ms       = HFDKB_IND_RESET_ON_MS,
    .off_ms      = HFDKB_IND_RESET_OFF_MS,
    .duration_ms = HFDKB_IND_RESET_DURATION_MS,
    .priority    = 100,
    .off_mode    = IND_OFF_BLACK,
};

// 恢复出厂期间，Logo 区域保持黑色。
static const indicator_config_t ind_reset_logo_config = {
    .first_led   = HFDKB_IND_LOGO_FIRST,
    .led_count   = HFDKB_IND_LOGO_COUNT,
    .effect      = IND_SOLID,
    .color       = {0, 0, 0},
    .duration_ms = HFDKB_IND_RESET_DURATION_MS,
    .priority    = 100,
};

#endif
