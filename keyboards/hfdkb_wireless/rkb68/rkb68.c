// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#ifdef BT_MODE_ENABLE
#    include "common/bt_task.h"
#endif
#include "usb_main.h"
#include "lib/lib8tion/lib8tion.h"

void keyboard_post_init_kb(void) {
#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif

#ifdef LED_POWER_EN_PIN
    setPinOutput(LED_POWER_EN_PIN);
    writePinLow(LED_POWER_EN_PIN);
#endif

#ifdef WL_PWR_SW_PIN
    setPinInputHigh(WL_PWR_SW_PIN);
#endif

#ifdef BT_CABLE_PIN
    setPinInputHigh(BT_CABLE_PIN);
#endif

#ifdef BT_CHARGE_PIN
    setPinInput(BT_CHARGE_PIN);
#endif

    if (keymap_config.no_gui) {
        keymap_config.no_gui = false;
        eeconfig_update_keymap(&keymap_config);
    }

    keyboard_post_init_user();
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    // if ((dev_info.devs != DEVS_USB) && bts_info.bt_info.low_vol_offed) {
    //     WL_PROCESS_KEYS(keycode, 0);
    //     bts_task(dev_info.devs);
    //     while (bts_is_busy()) {
    //         wait_ms(1);
    //     }
    //     return false;
    // }

    if (process_record_user(keycode, record) != true) {
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

        default:
            return true;
    }

    return false;
}

void matrix_scan_init(void) {
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

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
        rgb_matrix_set_color_all(0, 0, 0);
    } else {
        // Logo led effect
        uint8_t time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_get_speed() / 4, 1));
        for (uint8_t i = 68; i <= 74; i++) {
            HSV hsv = {g_led_config.point[i].x - time, 255, rgb_matrix_get_val() / 3};
            RGB rgb = hsv_to_rgb(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
        }
    }

    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }

    if (host_keyboard_led_state().caps_lock) {
        rgb_matrix_set_color(LED_CAPS_LOCK_INDEX, 0x77, 0x77, 0x77);
    }

    if (keymap_config.no_gui) {
        rgb_matrix_set_color(LED_GUI_LOCK_INDEX, 0x77, 0x77, 0x77);
    }

    return true;
}

void housekeeping_task_kb(void) {
#ifdef USB_SUSPEND_CHECK_ENABLE
    static uint32_t usb_suspend_timer = 0;
    static uint32_t usb_suspend       = false;

    if (dev_info.devs == DEVS_USB) {
        if (usb_suspend) {
            if (suspend_wakeup_condition()) {
                // usbWakeupHost(&USB_DRIVER);
                // restart_usb_driver(&USB_DRIVER);
                usb_suspend       = false;
                usb_suspend_timer = 0;
#    ifdef LED_POWER_EN_PIN
                writePinLow(LED_POWER_EN_PIN);
#    endif
            }
        }

        if ((USB_DRIVER.state != USB_ACTIVE)) {
            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                usb_suspend_timer = 0;
                if (!usb_suspend) {
                    usb_suspend = true;
#    ifdef LED_POWER_EN_PIN
                    writePinHigh(LED_POWER_EN_PIN);
#    endif
                    // lpwr_set_state(LPWR_PRESLEEP);
                }
            }
        } else {
            if (usb_suspend) {
                usb_suspend_timer = 0;
                usb_suspend       = false;

#    ifdef LED_POWER_EN_PIN
                writePinLow(LED_POWER_EN_PIN);
#    endif
            }
        }
    }
#endif

#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif
}
