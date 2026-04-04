// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"
#include <stdlib.h>
#include "usb_main.h"

bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
        // Set our LED pins as output
        gpio_set_pin_output(A14);
        if (dev_info.devs == DEVS_USB) {
            gpio_write_pin_low(A14);
        } else {
            gpio_write_pin_high(A14);
        }

        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        // Set our LED pins as input
        led_inited = false;
    }
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
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

        case LOGO_MOD:
            if (record->event.pressed) {
                dev_info.rgb_logo_mode++;
                dev_info.rgb_logo_mode %= 9;
                eeconfig_update_user(dev_info.raw);
            }
            return false;

        default:
            break;
    }
#ifdef BT_MODE_ENABLE
    if (process_record_bt(keycode, record) != true) {
        return false;
    }
#endif
    return true;
}

void keyboard_post_init_kb(void) {
    if (keymap_config.no_gui) {
        keymap_config.no_gui = 0;
        eeconfig_update_keymap(&keymap_config);
    }
}

void matrix_init_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_init();
    led_config_all();
#endif
    matrix_init_user();
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
#endif
    matrix_scan_user();
}

static uint32_t usb_suspend_timer = 0;
static uint32_t usb_suspend       = false;
static uint32_t rgb_status_saved  = 0;

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif

#ifdef USB_CHECK_SUSPEND_STATE
    if (dev_info.devs == DEVS_USB) {
        if (usb_suspend) {
            bool wakeup = false;
            for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
                if (matrix_get_row(r)) {
                    wakeup = true;
                    break;
                }
            }
            if (wakeup) {
                // usbWakeupHost(&USB_DRIVER);
                // restart_usb_driver(&USB_DRIVER);
                usb_suspend       = false;
                usb_suspend_timer = 0;
                if (rgb_status_saved) {
                    rgb_matrix_enable_noeeprom();
                }
            }
        }

        if (USB_DRIVER.state != USB_ACTIVE || USB_DRIVER.state == USB_SUSPENDED) {
            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                if (!usb_suspend) {
                    usb_suspend      = true;
                    rgb_status_saved = rgb_matrix_is_enabled();
                    if (rgb_status_saved) {
                        rgb_matrix_disable_noeeprom();
                    }
                }
                usb_suspend_timer = 0;
            }
        } else {
            if (usb_suspend) {
                usb_suspend_timer = 0;
                usb_suspend       = false;
                if (rgb_status_saved) {
                    rgb_matrix_enable_noeeprom();
                }
            }
        }
    } else {
        if (usb_suspend) {
            usb_suspend_timer = 0;
            usb_suspend       = false;
            if (rgb_status_saved) {
                rgb_matrix_enable_noeeprom();
            }
        }
    }
#endif
}

uint8_t  rgb_test_en    = false;
uint8_t  rgb_test_index = 0;
uint32_t rgb_test_time  = 0;

static const uint8_t rgb_test_color_table[][3] = {
    {200, 200, 200},
    {200, 0, 0},
    {0, 200, 0},
    {0, 0, 200},
};

static const uint8_t rgb_logo_color_table[][3] = {
    {92, 0, 0}, {92, 92, 0}, {46, 92, 0}, {0, 92, 0}, {0, 92, 92}, {0, 0, 92}, {92, 0, 92}, {0, 0, 0},
};

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (!rgb_matrix_get_flags() || bts_info.bt_info.low_vol) rgb_matrix_set_color_all(0, 0, 0);

    if (dev_info.rgb_logo_mode) {
        rgb_matrix_set_color(0, rgb_logo_color_table[dev_info.rgb_logo_mode - 1][0], rgb_logo_color_table[dev_info.rgb_logo_mode - 1][1], rgb_logo_color_table[dev_info.rgb_logo_mode - 1][2]);
    }

    if (rgb_test_en) {
        if (timer_elapsed32(rgb_test_time) > 3000 * (rgb_test_index + 1)) {
            rgb_test_index++;
        }
        // clang-format off
        rgb_matrix_set_color_all(rgb_test_color_table[rgb_test_index % 4 - 1][0],
        rgb_test_color_table[rgb_test_index % 4 - 1][1],
        rgb_test_color_table[rgb_test_index % 4 - 1][2]);
        // clang-format on
        if (timer_elapsed32(rgb_test_time) > 30 * 1000) {
            rgb_test_en = false;
        }
        return false;
    }

#ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#endif

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

    return true;
}
