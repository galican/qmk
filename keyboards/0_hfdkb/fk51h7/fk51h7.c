// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#ifdef USB_SUSPEND_CHECK_ENABLE
#    include "usb_main.h"
#endif
#include "lib/lib8tion/lib8tion.h"

bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        led_inited = false;
    }
}

bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }
    if (index == 0) {
        default_layer_set(1UL << (active ? 0 : 2));
        if (!active) {
            keymap_config.no_gui = 0;
            eeconfig_update_keymap(&keymap_config);
        }
    }
    return true;
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    extern bool Low_power_off;
    if (Low_power_off) {
        bts_process_keys(keycode, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {
            wait_ms(1);
        }
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
                rgb_matrix_enable();
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

void keyboard_post_init_kb(void) {
    if (keymap_config.no_gui) {
        keymap_config.no_gui = 0;
        eeconfig_update_keymap(&keymap_config);
    }
}

void suspend_power_down_kb() {
    gpio_write_pin_low(SNLED27351_SDB_PIN);

    suspend_power_down_user();
}

void suspend_wakeup_init_kb() {
    gpio_write_pin_high(SNLED27351_SDB_PIN);

    suspend_wakeup_init_user();
}

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef USB_SUSPEND_CHECK_ENABLE
    static uint32_t usb_suspend_timer = 0;
    static uint32_t usb_suspend       = false;

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
#    ifdef RGB_DRIVER_SDB_PIN
                writePinHigh(RGB_DRIVER_SDB_PIN);
#    endif
                gpio_write_pin_high(SNLED27351_SDB_PIN);
            }
        }

        // if ((USB_DRIVER.state != USB_ACTIVE) || (USB_DRIVER.state == USB_SUSPENDED)) {
        if (USB_DRIVER.state != USB_ACTIVE) {
            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                if (!usb_suspend) {
                    usb_suspend = true;
#    ifdef RGB_DRIVER_SDB_PIN
                    writePinLow(RGB_DRIVER_SDB_PIN);
#    endif
                }
                gpio_write_pin_low(SNLED27351_SDB_PIN);
                usb_suspend_timer = 0;
            }
        } else {
            if (usb_suspend) {
                usb_suspend_timer = 0;
                usb_suspend       = false;

#    ifdef RGB_DRIVER_SDB_PIN
                writePinHigh(RGB_DRIVER_SDB_PIN);
#    endif
                gpio_write_pin_high(SNLED27351_SDB_PIN);
            }
        }
    } else {
        if (usb_suspend) {
            usb_suspend_timer = 0;
            usb_suspend       = false;
#    ifdef RGB_DRIVER_SDB_PIN
            writePinHigh(RGB_DRIVER_SDB_PIN);
#    endif
            gpio_write_pin_high(SNLED27351_SDB_PIN);
        }
    }
#endif

#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif
}

static const uint8_t rgb_logo_color_table[][3] = {
    {100, 0, 0}, {0, 100, 0}, {0, 0, 100}, {100, 100, 0}, {100, 100, 100}, {100, 0, 100}, {0, 100, 100},
};

#ifdef RGB_MATRIX_ENABLE
bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    extern bool EE_CLR_flag;
    extern bool Low_power;

    if ((rgb_matrix_get_flags() == LED_FLAG_NONE) || Low_power) {
        // if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
        rgb_matrix_set_color_all(0, 0, 0);
    }

    if ((rgb_matrix_get_flags() != LED_FLAG_NONE) && !EE_CLR_flag && !Low_power) {
        if (dev_info.logo_effect == LOGO_EFFECT_DEFAULT) {
            uint8_t time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_get_speed() / 4, 1));
            for (uint8_t i = 83; i < 87; i++) {
                HSV hsv = {g_led_config.point[i].y - time, 255, rgb_matrix_get_val()};
                RGB rgb = hsv_to_rgb(hsv);
                rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
            }
        } else {
            for (uint8_t i = 83; i < 87; i++) {
                rgb_matrix_set_color(i, rgb_logo_color_table[dev_info.logo_effect - 1][0], rgb_logo_color_table[dev_info.logo_effect - 1][1], rgb_logo_color_table[dev_info.logo_effect - 1][2]);
            }
        }
    }

#    ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#    endif

    // caps lock red
    if (host_keyboard_led_state().caps_lock && (dev_info.devs == DEVS_USB || bts_info.bt_info.paired)) {
        rgb_matrix_set_color(LED_CAPS_LOCK_IND_INDEX, 100, 100, 100);
    }
    // gui lock red
    if (keymap_config.no_gui && (dev_info.devs == DEVS_USB || bts_info.bt_info.paired) && (get_highest_layer(default_layer_state) == 0)) {
        rgb_matrix_set_color(LED_GUI_LOCK_IND_INDEX, 100, 100, 100);
    }

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

    return true;
}
#endif
