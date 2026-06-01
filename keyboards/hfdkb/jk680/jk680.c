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
#include "lib/lib8tion/lib8tion.h"

void RGB_effect_user(uint8_t effect, uint8_t color, uint8_t led_start_index, uint8_t led_end_index);

#ifdef DIP_SWITCH_ENABLE
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }
    if (index == 0) {
        default_layer_set(1UL << (active ? 3 : 0));
    }
    if (active) {
        keymap_config.no_gui = 0;
        eeconfig_update_keymap(&keymap_config);
    }
    return true;
}
#endif

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (process_record_user(keycode, record) != true) {
        return false;
    }

    extern bool low_vol_offed_sleep;
    if (low_vol_offed_sleep) {
        bts_process_keys(keycode, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {
            wait_ms(1);
        }
        return false;
    }

#ifdef BT_MODE_ENABLE
    if (process_record_bt(keycode, record) != true) {
        return false;
    }
#endif

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

        case SLED_MODE_NEXT:
            if (record->event.pressed) {
                dev_info.sled_mode++;
                if (dev_info.sled_mode > 4) {
                    dev_info.sled_mode = 0;
                }
                eeconfig_update_user(dev_info.raw);
            }
            return false;

        case ALED_MODE_NEXT:
            if (record->event.pressed) {
                dev_info.aled_mode++;
                if (dev_info.aled_mode > 4) {
                    dev_info.aled_mode = 0;
                }
                eeconfig_update_user(dev_info.raw);
            }
            return false;

        default:
            break;
    }

    return true;
}

void matrix_init_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_init();
#endif
    matrix_init_user();
}

void set_led_state(void) {
    // caps lock rd
    if (host_keyboard_led_state().caps_lock && (((dev_info.devs != DEVS_USB) && bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        gpio_write_pin_high(CAPS_LOCK_LED_PIN);
    } else {
        gpio_write_pin_low(CAPS_LOCK_LED_PIN);
    }
    // GUI lock
    if (keymap_config.no_gui) {
        gpio_write_pin_high(GUI_LOCK_LED_PIN);
    } else {
        gpio_write_pin_low(GUI_LOCK_LED_PIN);
    }
    // all key lock
    // extern bool KEY_LOCK_flag;
    // if (KEY_LOCK_flag) {
    //     gpio_write_pin_high(ALL_KEY_LOCK_PIN);
    // } else {
    //     gpio_write_pin_low(ALL_KEY_LOCK_PIN);
    // }
    // num 2 fn
    extern bool NUM_2_FN_flag;
    if (NUM_2_FN_flag) {
        gpio_write_pin_high(NUM_2_FN_PIN);
    } else {
        gpio_write_pin_low(NUM_2_FN_PIN);
    }
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
#endif

    extern bool kb_sleep_flag;
    if (((dev_info.devs != DEVS_USB) && !kb_sleep_flag) || ((dev_info.devs == DEVS_USB) && (USB_DRIVER.state != USB_SUSPENDED))) {
        set_led_state();
    }

    matrix_scan_user();
}

void keyboard_post_init_kb(void) {
    if (keymap_config.no_gui) {
        keymap_config.no_gui = 0;
        eeconfig_update_keymap(&keymap_config);
    }

    keyboard_post_init_user();
}

void suspend_power_down_kb(void) {
    extern void led_deconfig_all(void);
    led_deconfig_all();
    suspend_power_down_user();
}

void suspend_wakeup_init_kb(void) {
    extern void led_config_all(void);
    led_config_all();
    suspend_wakeup_init_user();
}

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef USB_SUSPEND_STATE_CHECK
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
                gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#    endif
                // extern void led_config_all(void);
                // led_config_all();
            }
        }

        if ((USB_DRIVER.state != USB_ACTIVE)) {
            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                if (!usb_suspend) {
                    usb_suspend = true;
#    ifdef RGB_DRIVER_SDB_PIN
                    gpio_write_pin_low(RGB_DRIVER_SDB_PIN);
#    endif
                    // extern void led_deconfig_all(void);
                    // led_deconfig_all();
                }
                usb_suspend_timer = 0;
            }
        } else {
            if (usb_suspend) {
                usb_suspend_timer = 0;
                usb_suspend       = false;

#    ifdef RGB_DRIVER_SDB_PIN
                gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#    endif
            }
        }
    } else {
        if (usb_suspend) {
            usb_suspend_timer = 0;
            usb_suspend       = false;
#    ifdef RGB_DRIVER_SDB_PIN
            gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#    endif
            // extern void led_config_all(void);
            // led_config_all();
        }
    }
#endif
}

#ifdef RGB_MATRIX_ENABLE
extern bool low_battery_warning_flag;

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (!rgb_matrix_get_flags() || low_battery_warning_flag) {
        rgb_matrix_set_color_all(0, 0, 0);
    }

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

    if (!low_battery_warning_flag) {
        RGB_effect_user(dev_info.sled_mode, dev_info.sled_color, SLED_START_INDEX, SLED_END_INDEX);
        RGB_effect_user(dev_info.aled_mode, dev_info.aled_color, ALED_START_INDEX, ALED_END_INDEX);
    }

#    ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#    endif

    return true;
}
#endif

uint8_t color_table[] = {
    0, 32, 64, 96, 128, 160, 192,
};

#define NEON_EFFCT_CONSTANT(TIME)         \
    (HSV) {                               \
        TIME, 255, RGB_MATRIX_DEFAULT_VAL \
    }

void RGB_neon_effect_user(uint8_t led_start_index, uint8_t led_end_index) {
    uint8_t time = scale16by8(g_rgb_timer, qadd8((UINT8_MAX / 2) / 8, 1));
    for (uint8_t i = led_start_index; i <= led_end_index; i++) {
        rgb_matrix_set_color(i, hsv_to_rgb(NEON_EFFCT_CONSTANT(time)).r, hsv_to_rgb(NEON_EFFCT_CONSTANT(time)).g, hsv_to_rgb(NEON_EFFCT_CONSTANT(time)).b);
    }
}

void RGB_off_effect_user(uint8_t led_start_index, uint8_t led_end_index) {
    for (uint8_t i = led_start_index; i <= led_end_index; i++) {
        rgb_matrix_set_color(i, 0, 0, 0); /* code */
    }
}

#define FLOWING_EFFCT_CONSTANT(TIME, i)                                        \
    (HSV) {                                                                    \
        (uint8_t)(g_led_config.point[i].y - TIME), 255, RGB_MATRIX_DEFAULT_VAL \
    }

void RGB_flowing_effect_user(uint8_t led_start_index, uint8_t led_end_index) {
    uint8_t time = scale16by8(g_rgb_timer, qadd8((UINT8_MAX / 2) / 4, 1));
    for (uint8_t i = led_start_index; i <= led_end_index; i++) {
        rgb_matrix_set_color(i, hsv_to_rgb(FLOWING_EFFCT_CONSTANT(time, i)).r, hsv_to_rgb(FLOWING_EFFCT_CONSTANT(time, i)).g, hsv_to_rgb(FLOWING_EFFCT_CONSTANT(time, i)).b);
    }
}

#define BREATH_EFFCT_CONSTANT(TIME)                                                                                       \
    (HSV) {                                                                                                               \
        color_table[RGB_MATRIX_DEFAULT_HUE], 255, (uint8_t)scale8(abs8(sin8(TIME / 2) - 128) * 2, RGB_MATRIX_DEFAULT_VAL) \
    }

void RGB_breath_effect_user(uint8_t color_mode, uint8_t led_start_index, uint8_t led_end_index) {
    uint16_t time = scale16by8(g_rgb_timer, qadd8((UINT8_MAX / 2) / 4, 1));
    for (uint8_t i = led_start_index; i <= led_end_index; i++) {
        rgb_matrix_set_color(i, hsv_to_rgb(BREATH_EFFCT_CONSTANT(time)).r, hsv_to_rgb(BREATH_EFFCT_CONSTANT(time)).g, hsv_to_rgb(BREATH_EFFCT_CONSTANT(time)).b); /* code */
    }
}

#define SOLID_EFFCT_CONSTANT                                             \
    (HSV) {                                                              \
        color_table[RGB_MATRIX_DEFAULT_HUE], 255, RGB_MATRIX_DEFAULT_VAL \
    }

void RGB_solid_effect_user(uint8_t color_mode, uint8_t led_start_index, uint8_t led_end_index) {
    for (uint8_t i = led_start_index; i <= led_end_index; i++) {
        rgb_matrix_set_color(i, hsv_to_rgb(SOLID_EFFCT_CONSTANT).r, hsv_to_rgb(SOLID_EFFCT_CONSTANT).g, hsv_to_rgb(SOLID_EFFCT_CONSTANT).b); /* code */
    }
}

void RGB_effect_user(uint8_t effect_mode, uint8_t color_mode, uint8_t led_start_index, uint8_t led_end_index) {
    switch (effect_mode) {
        case 0:
            RGB_flowing_effect_user(led_start_index, led_end_index);
            break;
        case 1:
            RGB_neon_effect_user(led_start_index, led_end_index);
            break;
        case 2:
            RGB_solid_effect_user(color_mode, led_start_index, led_end_index);
            break;
        case 3:
            RGB_breath_effect_user(color_mode, led_start_index, led_end_index);
            break;
        case 4:
            RGB_off_effect_user(led_start_index, led_end_index);
            break;
        default:
            break;
    }
}
