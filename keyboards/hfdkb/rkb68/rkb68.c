// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#include "hfdkb_common.h"
#include "wireless.h"

void keyboard_pre_init_kb(void) {
#ifdef LED_POWER_EN_PIN
    gpio_write_pin_low(LED_POWER_EN_PIN);
    gpio_set_pin_output(LED_POWER_EN_PIN);
#endif

#ifdef BT_CABLE_PIN
    gpio_set_pin_input_high(BT_CABLE_PIN);
#endif

#ifdef BT_CHARGE_PIN
    gpio_set_pin_input(BT_CHARGE_PIN);
#endif

    keyboard_pre_init_user();
}

void keyboard_post_init_kb(void) {
    wireless_post_init();
    if (keymap_config.no_gui) {
        keymap_config.no_gui = false;
        eeconfig_update_keymap(&keymap_config);
    }

    keyboard_post_init_user();
}

void suspend_power_down_kb(void) {
#ifdef LED_POWER_EN_PIN
    gpio_write_pin_high(LED_POWER_EN_PIN);
#endif
    suspend_power_down_user();
}

void suspend_wakeup_init_kb(void) {
#ifdef LED_POWER_EN_PIN
    gpio_write_pin_low(LED_POWER_EN_PIN);
#endif
    suspend_wakeup_init_user();
}

void housekeeping_task_kb(void) {
    wireless_task();
    housekeeping_task_usb_suspend();
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (wireless_process_record(keycode, record) != true) {
        return false;
    }

    if (process_record_user(keycode, record) != true) {
        return false;
    }

    switch (keycode) {
        case RM_TOGG:
            if (record->event.pressed) {
                switch (rgb_matrix_get_flags()) {
                    case LED_FLAG_ALL: {
                        rgb_matrix_set_flags(LED_FLAG_NONE);
                        rgb_matrix_set_color_all(0, 0, 0);
                    } break;
                    default: {
                        rgb_matrix_set_flags(LED_FLAG_ALL);
                    } break;
                }
            }
            if (!rgb_matrix_is_enabled()) {
                rgb_matrix_set_flags(LED_FLAG_ALL);
                rgb_matrix_enable();
            }
            return false;
    }

    return true;
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (wireless_indicators_advanced(led_min, led_max) != true) {
        return false;
    }

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

    return true;
}
