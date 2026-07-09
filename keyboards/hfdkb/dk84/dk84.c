/* Copyright (C) 2025 @ HFD (https://www.hfdic.com/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#ifdef BT_MODE_ENABLE
#    include "common/bt_task.h"
#    include "usb_main.h"
#endif
// clang-format off
#ifdef RGB_MATRIX_ENABLE
// const snled27351_led_t PROGMEM g_snled27351_leds[SNLED27351_LED_COUNT] = {
const is31fl3733_led_t PROGMEM g_is31fl3733_leds[IS31FL3733_LED_COUNT] = {
/* Refer to IS31 manual for these locations
 *   driver
 *   |  R location
 *   |  |      G location
 *   |  |      |      B location
 *   |  |      |      | */
    {1, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {1, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {1, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {1, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {1, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {1, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {1, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {1, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {1, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {1, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {1, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {1, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {1, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {1, SW1_CS14,  SW2_CS14,  SW3_CS14},
    {0, SW4_CS15,  SW5_CS15,  SW6_CS15},
    {1, SW4_CS1,  SW5_CS1,  SW6_CS1},

    {0, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {0, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {0, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {0, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {0, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {0, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {0, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {0, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {0, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {0, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {0, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {0, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {0, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {0, SW1_CS14,  SW2_CS14,  SW3_CS14},
    {1, SW4_CS2,  SW5_CS2,  SW6_CS2},

    {0, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {0, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {0, SW4_CS3,   SW5_CS3,   SW6_CS3},
    {0, SW4_CS4,   SW5_CS4,   SW6_CS4},
    {0, SW4_CS5,   SW5_CS5,   SW6_CS5},
    {0, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {0, SW4_CS7,   SW5_CS7,   SW6_CS7},
    {0, SW4_CS8,   SW5_CS8,   SW6_CS8},
    {0, SW4_CS9,   SW5_CS9,   SW6_CS9},
    {0, SW4_CS10,  SW5_CS10,  SW6_CS10},
    {0, SW4_CS11,  SW5_CS11,  SW6_CS11},
    {0, SW4_CS12,  SW5_CS12,  SW6_CS12},
    {0, SW4_CS13,  SW5_CS13,  SW6_CS13},
    {0, SW4_CS14,  SW5_CS14,  SW6_CS14},
    {1, SW4_CS3,  SW5_CS3,  SW6_CS3},

    {0, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {0, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {0, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {0, SW7_CS4,   SW8_CS4,   SW9_CS4},
    {0, SW7_CS5,   SW8_CS5,   SW9_CS5},
    {0, SW7_CS6,   SW8_CS6,   SW9_CS6},
    {0, SW7_CS7,   SW8_CS7,   SW9_CS7},
    {0, SW7_CS8,   SW8_CS8,   SW9_CS8},
    {0, SW7_CS9,   SW8_CS9,   SW9_CS9},
    {0, SW7_CS10,  SW8_CS10,  SW9_CS10},
    {0, SW7_CS11,   SW8_CS11,   SW9_CS11},
    {0, SW7_CS12,   SW8_CS12,   SW9_CS12},
    {0, SW7_CS13,   SW8_CS13,   SW9_CS13},
    {0, SW7_CS14,   SW8_CS14,   SW9_CS14},
    {1, SW4_CS4,  SW5_CS4,  SW6_CS4},

    {0, SW10_CS1,   SW11_CS1,   SW12_CS1},
    {0, SW10_CS2,   SW11_CS2,   SW12_CS2},
    {0, SW10_CS3,   SW11_CS3,   SW12_CS3},
    {0, SW10_CS4,   SW11_CS4,   SW12_CS4},
    {0, SW10_CS5,   SW11_CS5,   SW12_CS5},
    {0, SW10_CS6,   SW11_CS6,   SW12_CS6},
    {0, SW10_CS7,   SW11_CS7,   SW12_CS7},
    {0, SW10_CS8,   SW11_CS8,   SW12_CS8},
    {0, SW10_CS9,   SW11_CS9,   SW12_CS9},
    {0, SW10_CS10,   SW11_CS10,   SW12_CS10},
    {0, SW10_CS11,   SW11_CS11,   SW12_CS11},
    {0, SW10_CS12,   SW11_CS12,   SW12_CS12},
    {0, SW10_CS13,   SW11_CS13,   SW12_CS13},
    {0, SW10_CS14,   SW11_CS14,   SW12_CS14},
    {1, SW4_CS5,  SW5_CS5,  SW6_CS5},

    {0, SW10_CS15,   SW11_CS15,   SW12_CS15},
    {0, SW10_CS16,   SW11_CS16,   SW12_CS16},
    {0, SW7_CS15,   SW8_CS15,   SW9_CS15},
    {0, SW7_CS16,   SW8_CS16,   SW9_CS16},
    {0, SW4_CS16,   SW5_CS16,   SW6_CS16},
    {0, SW1_CS15,   SW2_CS15,   SW3_CS15},
    {0, SW1_CS16,   SW2_CS16,   SW3_CS16},
    {1, SW4_CS8,  SW5_CS8,  SW6_CS8},
    {1, SW4_CS7,  SW5_CS7,  SW6_CS7},
    {1, SW4_CS6,  SW5_CS6,  SW6_CS6},
    // {1, SW1_CA1,   SW2_CA1,   SW3_CA1},
    // {1, SW1_CA2,   SW2_CA2,   SW3_CA2},
    // {1, SW1_CA3,   SW2_CA3,   SW3_CA3},
    // {1, SW1_CA4,   SW2_CA4,   SW3_CA4},
    // {1, SW1_CA5,   SW2_CA5,   SW3_CA5},
    // {1, SW1_CA6,   SW2_CA6,   SW3_CA6},
    // {1, SW1_CA7,   SW2_CA7,   SW3_CA7},
    // {1, SW1_CA8,   SW2_CA8,   SW3_CA8},
    // {1, SW1_CA9,   SW2_CA9,   SW3_CA9},
    // {1, SW1_CA10,  SW2_CA10,  SW3_CA10},
    // {1, SW1_CA11,  SW2_CA11,  SW3_CA11},
    // {1, SW1_CA12,  SW2_CA12,  SW3_CA12},
    // {1, SW1_CA13,  SW2_CA13,  SW3_CA13},
    // {1, SW1_CA14,  SW2_CA14,  SW3_CA14},
    // {0, SW4_CA15,  SW5_CA15,  SW6_CA15},
    // {1, SW4_CA1,  SW5_CA1,  SW6_CA1},

    // {0, SW1_CA1,   SW2_CA1,   SW3_CA1},
    // {0, SW1_CA2,   SW2_CA2,   SW3_CA2},
    // {0, SW1_CA3,   SW2_CA3,   SW3_CA3},
    // {0, SW1_CA4,   SW2_CA4,   SW3_CA4},
    // {0, SW1_CA5,   SW2_CA5,   SW3_CA5},
    // {0, SW1_CA6,   SW2_CA6,   SW3_CA6},
    // {0, SW1_CA7,   SW2_CA7,   SW3_CA7},
    // {0, SW1_CA8,   SW2_CA8,   SW3_CA8},
    // {0, SW1_CA9,   SW2_CA9,   SW3_CA9},
    // {0, SW1_CA10,  SW2_CA10,  SW3_CA10},
    // {0, SW1_CA11,  SW2_CA11,  SW3_CA11},
    // {0, SW1_CA12,  SW2_CA12,  SW3_CA12},
    // {0, SW1_CA13,  SW2_CA13,  SW3_CA13},
    // {0, SW1_CA14,  SW2_CA14,  SW3_CA14},
    // {1, SW4_CA2,  SW5_CA2,  SW6_CA2},

    // {0, SW4_CA1,   SW5_CA1,   SW6_CA1},
    // {0, SW4_CA2,   SW5_CA2,   SW6_CA2},
    // {0, SW4_CA3,   SW5_CA3,   SW6_CA3},
    // {0, SW4_CA4,   SW5_CA4,   SW6_CA4},
    // {0, SW4_CA5,   SW5_CA5,   SW6_CA5},
    // {0, SW4_CA6,   SW5_CA6,   SW6_CA6},
    // {0, SW4_CA7,   SW5_CA7,   SW6_CA7},
    // {0, SW4_CA8,   SW5_CA8,   SW6_CA8},
    // {0, SW4_CA9,   SW5_CA9,   SW6_CA9},
    // {0, SW4_CA10,  SW5_CA10,  SW6_CA10},
    // {0, SW4_CA11,  SW5_CA11,  SW6_CA11},
    // {0, SW4_CA12,  SW5_CA12,  SW6_CA12},
    // {0, SW4_CA13,  SW5_CA13,  SW6_CA13},
    // {0, SW4_CA14,  SW5_CA14,  SW6_CA14},
    // {1, SW4_CA3,  SW5_CA3,  SW6_CA3},

    // {0, SW7_CA1,   SW8_CA1,   SW9_CA1},
    // {0, SW7_CA2,   SW8_CA2,   SW9_CA2},
    // {0, SW7_CA3,   SW8_CA3,   SW9_CA3},
    // {0, SW7_CA4,   SW8_CA4,   SW9_CA4},
    // {0, SW7_CA5,   SW8_CA5,   SW9_CA5},
    // {0, SW7_CA6,   SW8_CA6,   SW9_CA6},
    // {0, SW7_CA7,   SW8_CA7,   SW9_CA7},
    // {0, SW7_CA8,   SW8_CA8,   SW9_CA8},
    // {0, SW7_CA9,   SW8_CA9,   SW9_CA9},
    // {0, SW7_CA10,  SW8_CA10,  SW9_CA10},
    // {0, SW7_CA11,   SW8_CA11,   SW9_CA11},
    // {0, SW7_CA12,   SW8_CA12,   SW9_CA12},
    // {0, SW7_CA13,   SW8_CA13,   SW9_CA13},
    // {0, SW7_CA14,   SW8_CA14,   SW9_CA14},
    // {1, SW4_CA4,  SW5_CA4,  SW6_CA4},

    // {0, SW10_CA1,   SW11_CA1,   SW12_CA1},
    // {0, SW10_CA2,   SW11_CA2,   SW12_CA2},
    // {0, SW10_CA3,   SW11_CA3,   SW12_CA3},
    // {0, SW10_CA4,   SW11_CA4,   SW12_CA4},
    // {0, SW10_CA5,   SW11_CA5,   SW12_CA5},
    // {0, SW10_CA6,   SW11_CA6,   SW12_CA6},
    // {0, SW10_CA7,   SW11_CA7,   SW12_CA7},
    // {0, SW10_CA8,   SW11_CA8,   SW12_CA8},
    // {0, SW10_CA9,   SW11_CA9,   SW12_CA9},
    // {0, SW10_CA10,   SW11_CA10,   SW12_CA10},
    // {0, SW10_CA11,   SW11_CA11,   SW12_CA11},
    // {0, SW10_CA12,   SW11_CA12,   SW12_CA12},
    // {0, SW10_CA13,   SW11_CA13,   SW12_CA13},
    // {0, SW10_CA14,   SW11_CA14,   SW12_CA14},
    // {1, SW4_CA5,  SW5_CA5,  SW6_CA5},

    // {0, SW10_CA15,   SW11_CA15,   SW12_CA15},
    // {0, SW10_CA16,   SW11_CA16,   SW12_CA16},
    // {0, SW7_CA15,   SW8_CA15,   SW9_CA15},
    // {0, SW7_CA16,   SW8_CA16,   SW9_CA16},
    // {0, SW4_CA16,   SW5_CA16,   SW6_CA16},
    // {0, SW1_CA15,   SW2_CA15,   SW3_CA15},
    // {0, SW1_CA16,   SW2_CA16,   SW3_CA16},
    // {1, SW4_CA8,  SW5_CA8,  SW6_CA8},
    // {1, SW4_CA7,  SW5_CA7,  SW6_CA7},
    // {1, SW4_CA6,  SW5_CA6,  SW6_CA6},
};
#endif

// clang-format on
#ifdef DIP_SWITCH_ENABLE
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }
    if (index == 0) {
        default_layer_set(1UL << (active ? 3 : 0));
    }
    if (active) {
        keymap_config.no_gui = false;
        eeconfig_update_keymap(&keymap_config);
    }
    return true;
}
#endif

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    extern bool low_vol_offed_sleep;
    if (low_vol_offed_sleep) {
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
                rgb_matrix_set_flags(LED_FLAG_ALL);
                rgb_matrix_enable();
            }
            return false;

        case G(KC_TAB):
            if (keymap_config.no_gui) {
                if (record->event.pressed) {
                    if (dev_info.devs == DEVS_USB) {
                        register_code(KC_LGUI);
                        register_code(KC_TAB);
                    } else {
                        bts_process_keys(KC_LGUI, 1, dev_info.devs, false, KEY_NUM);
                        bts_process_keys(KC_TAB, 1, dev_info.devs, false, KEY_NUM);
                    }
                } else {
                    if (dev_info.devs == DEVS_USB) {
                        unregister_code(KC_TAB);
                        unregister_code(KC_LGUI);
                    } else {
                        bts_process_keys(KC_TAB, 0, dev_info.devs, false, KEY_NUM);
                        bts_process_keys(KC_LGUI, 0, dev_info.devs, false, KEY_NUM);
                    }
                }
                return false;
            }
            break;
        case G(KC_E):
            if (keymap_config.no_gui) {
                if (record->event.pressed) {
                    if (dev_info.devs == DEVS_USB) {
                        register_code(KC_LGUI);
                        register_code(KC_E);
                    } else {
                        bts_process_keys(KC_LGUI, 1, dev_info.devs, false, KEY_NUM);
                        bts_process_keys(KC_E, 1, dev_info.devs, false, KEY_NUM);
                    }
                } else {
                    if (dev_info.devs == DEVS_USB) {
                        unregister_code(KC_E);
                        unregister_code(KC_LGUI);
                    } else {
                        bts_process_keys(KC_E, 0, dev_info.devs, false, KEY_NUM);
                        bts_process_keys(KC_LGUI, 0, dev_info.devs, false, KEY_NUM);
                    }
                }
                return false;
            }
            break;
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
#endif
    matrix_init_user();
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
#endif
    matrix_scan_user();
}

#ifdef USB_SUSPEND_STATE_CHECK
static bool usb_suspend_rgb_is_off = false;

static void usb_suspend_rgb_off(void) {
#    ifdef RGB_MATRIX_ENABLE
    rgb_matrix_set_color_all(0, 0, 0);
#    endif
#    ifdef RGB_DRIVER_SDB_PIN
    gpio_write_pin_low(RGB_DRIVER_SDB_PIN);
#    endif
    usb_suspend_rgb_is_off = true;
}

static void usb_suspend_rgb_on(void) {
#    ifdef RGB_DRIVER_SDB_PIN
    gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#    endif
#    ifdef RGB_DRIVER_RESET_PIN
    // gpio_write_pin_low(RGB_DRIVER_RESET_PIN);
    // wait_ms(2);
    // gpio_write_pin_high(RGB_DRIVER_RESET_PIN);
#    endif
    wait_ms(10);
#    ifdef RGB_MATRIX_ENABLE
    // rgb_matrix_init();
#    endif
    usb_suspend_rgb_is_off = false;
}

static bool usb_suspend_matrix_pressed(void) {
    for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
        if (matrix_get_row(r)) {
            return true;
        }
    }
    return false;
}
#endif

void suspend_power_down_kb(void) {
#ifdef USB_SUSPEND_STATE_CHECK
    usb_suspend_rgb_off();
#else
#    ifdef RGB_DRIVER_SDB_PIN
    gpio_write_pin_low(RGB_DRIVER_SDB_PIN);
#    endif
#endif
    suspend_power_down_user();
}

extern uint8_t USB_blink_cnt;

void suspend_wakeup_init_kb(void) {
    USB_blink_cnt = 0;
    suspend_wakeup_init_user();
}

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef USB_SUSPEND_STATE_CHECK
    static uint32_t usb_suspend_timer = 0;
    static bool     usb_suspend       = false;
    static bool     wake_key_released = true;

    if (dev_info.devs == DEVS_USB) {
        if (USB_DRIVER.state != USB_ACTIVE || USB_DRIVER.state == USB_SUSPENDED) {
            const bool wake_key_pressed = usb_suspend_matrix_pressed();

            if (!wake_key_pressed) {
                wake_key_released = true;
            } else if (usb_suspend_rgb_is_off && wake_key_released) {
                wake_key_released = false;
                usb_suspend       = false;
                usb_suspend_timer = timer_read32();
                usb_suspend_rgb_on();
                USB_blink_cnt = 0;
            }

            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                if (!usb_suspend) {
                    usb_suspend = true;
                }
                usb_suspend_rgb_off();
                usb_suspend_timer = 0;
            }
        } else {
            if (usb_suspend_rgb_is_off) {
                usb_suspend_rgb_on();
                USB_blink_cnt = 0;
            }
            usb_suspend_timer = 0;
            usb_suspend       = false;
        }
    } else {
        if (usb_suspend_rgb_is_off) {
            usb_suspend_rgb_on();
        }
        usb_suspend_timer = 0;
        usb_suspend       = false;
    }
#endif
}

#ifdef RGB_MATRIX_ENABLE
extern bool battery_low_warning_flag;

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (!rgb_matrix_get_flags() || battery_low_warning_flag) {
        rgb_matrix_set_color_all(0, 0, 0);
    }

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

#    ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#    endif

    return true;
}
#endif

void keyboard_post_init_kb(void) {
    if (keymap_config.no_gui) {
        keymap_config.no_gui = false;
        eeconfig_update_keymap(&keymap_config);
    }
    keyboard_post_init_user();
}
