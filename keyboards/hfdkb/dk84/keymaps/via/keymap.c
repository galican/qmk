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
#include "os_detection.h"
#include "common/bt_task.h"

enum __layers {
    WIN_B,
    WIN_FN,
    MAC_B,
    MAC_FN,
};

#define BT1 BT_HOST1
#define BT2 BT_HOST2
#define BT3 BT_HOST3
#define BT4 BT_2_4G
#define SW_OS OS_SWITCH

#define KC_TASK G(KC_TAB)
#define KC_FILE G(KC_E)

#define KEY_FLASK_PRESSED 0x01
#define KEY_M_PRESSED 0x02
#define KEY_W_PRESSED 0x04
#define KEY_B_PRESSED 0x08
#define KEY_L_PRESSED 0x10
#define KEY_1_PRESSED 0x20
#define KEY_2_PRESSED 0x40
#define KEY_3_PRESSED 0x80
#define KEY_ESC_PRESSED 0x100

#define FUN_MAC_OS_MODE (KEY_FLASK_PRESSED | KEY_M_PRESSED)
#define FUN_WIN_OS_MODE (KEY_FLASK_PRESSED | KEY_W_PRESSED)
#define FUN_BAT_LEVEL_DETECT (KEY_FLASK_PRESSED | KEY_B_PRESSED)
#define FUN_LOCK_SCREEN (KEY_FLASK_PRESSED | KEY_L_PRESSED)
#define FUN_BT1_MODE (KEY_FLASK_PRESSED | KEY_1_PRESSED)
#define FUN_BT2_MODE (KEY_FLASK_PRESSED | KEY_2_PRESSED)
#define FUN_BT3_MODE (KEY_FLASK_PRESSED | KEY_3_PRESSED)
#define FUN_FACTORY_RESET (KEY_FLASK_PRESSED | KEY_ESC_PRESSED)

static uint32_t key_press_status = 0;

// static uint8_t  all_blink_cnt      = 0;
// static uint32_t all_blink_time     = 0;
// static RGB      all_blink_color    = {0};
extern uint8_t             single_blink_cnt;
extern uint8_t             single_blink_index;
extern RGB                 single_blink_color;
extern uint32_t            single_blink_time;
extern bool                query_vol_flag;
extern long_pressed_keys_t long_pressed_keys[];

// clang-format off

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [WIN_B] = LAYOUT_ansi_84( /* Base */
        KC_ESC,  KC_F1,    KC_F2,    KC_F3,    KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,     KC_F12,  KC_PSCR, KC_DEL,    KC_FLASK,
        KC_GRV,  KC_1,     KC_2,     KC_3,     KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,           KC_BSPC,   KC_HOME,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC,          KC_BSLS,   KC_END,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,    KC_NUHS,          KC_ENT,    KC_PGUP,
        KC_LSFT, KC_NUBS,  KC_Z,     KC_X,     KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,             KC_RSFT, KC_UP,     KC_PGDN,
        KC_LCTL, KC_LWIN,  KC_LALT,                              KC_SPC,                             KC_RALT, MO(WIN_FN), KC_RCTL, KC_LEFT, KC_DOWN,   KC_RGHT ),

    [WIN_FN] = LAYOUT_ansi_84( /* FN */
        _______, KC_BRID,  KC_BRIU,  KC_TASK,  KC_FILE, RM_VALD, RM_VALU, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, _______, _______,  _______,
        _______, BT1,      BT2,      BT3,      _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, _______,  _______,
        _______, GU_TOGG,  _______,                              _______,                            _______, _______,    _______, _______, _______,  _______),

    [MAC_B] = LAYOUT_ansi_84( /* Base */
        KC_ESC,  KC_BRID,  KC_BRIU,  KC_MCTL,  KC_LPAD, RM_VALD, RM_VALU, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, KC_PSCR, KC_DEL,    KC_FLASK,
        KC_GRV,  KC_1,     KC_2,     KC_3,     KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,           KC_BSPC,   KC_HOME,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC,          KC_BSLS,   KC_END,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,    KC_NUHS,          KC_ENT,    KC_PGUP,
        KC_LSFT, KC_NUBS,  KC_Z,     KC_X,     KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,             KC_RSFT, KC_UP,     KC_PGDN,
        KC_LCTL, KC_LOPT,  KC_LCMD,                              KC_SPC,                             KC_RCMD, MO(MAC_FN), KC_RCTL, KC_LEFT, KC_DOWN,   KC_RGHT ),

    [MAC_FN] = LAYOUT_ansi_84( /* FN */
        _______, KC_F1,    KC_F2,    KC_F3,    KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,     KC_F12,  _______, _______,  _______,
        _______, BT1,      BT2,      BT3,      _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______,          _______,  _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, _______,  _______,
        _______, _______,  _______,                              _______,                            _______, _______,    _______, _______, _______,  _______),
};

// clang-format on
bool process_detected_host_os_user(os_variant_t detected_os) {
    if (detected_os == OS_MACOS || detected_os == OS_IOS) {
        set_single_default_layer(MAC_B);
    } else {
        set_single_default_layer(WIN_B);
    }

    return true;
}

// static bool flask_held = false;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    // if (keycode == KC_FLASK) {
    //     flask_held = record->event.pressed;
    //     return false; // Flask alone sends nothing
    // }

    // if (flask_held && record->event.pressed) {
    //     switch (keycode) {
    //         case KC_0:
    //             layer_move(WIN_B);
    //             return false;
    //         case KC_1:
    //             layer_move(WIN_FN);
    //             return false;
    //         case KC_2:
    //             layer_move(MAC_B);
    //             return false;
    //         case KC_3:
    //             layer_move(MAC_FN);
    //             return false;
    //     }
    // }
    switch (keycode) {
        case KC_FLASK:
            if (record->event.pressed) {
                key_press_status |= KEY_FLASK_PRESSED;

            } else {
                key_press_status &= ~KEY_FLASK_PRESSED;
            }
            return false;

        case KC_M:
            if (record->event.pressed) {
                key_press_status |= KEY_M_PRESSED;
                if (key_press_status == FUN_MAC_OS_MODE) {
                    if (get_highest_layer(default_layer_state) == 0) { // WIN_BASE
                        set_single_persistent_default_layer(2);
                        if (keymap_config.no_gui) {
                            keymap_config.no_gui = false;
                        }
                        eeconfig_update_keymap(&keymap_config);
                        single_blink_cnt   = 6;
                        single_blink_color = (RGB){100 / 3, 100, 100};
                        if (timer_elapsed32(single_blink_time) >= 300) {
                            single_blink_time = timer_read32();
                        }
                        single_blink_index = GUI_LOCK_LED_INDEX;
                        return false;
                    }
                }
            } else {
                key_press_status &= ~KEY_M_PRESSED;
            }
            return true;

        case KC_W:
            if (record->event.pressed) {
                key_press_status |= KEY_W_PRESSED;
                if (key_press_status == FUN_WIN_OS_MODE) {
                    set_single_persistent_default_layer(0);
                    return false;
                }
            } else {
                key_press_status &= ~KEY_W_PRESSED;
            }
            return true;

        case KC_B:
            if (record->event.pressed) {
                key_press_status |= KEY_B_PRESSED;
                if (key_press_status == FUN_BAT_LEVEL_DETECT) {
                    bts_send_vendor(v_query_vol);
                    query_vol_flag = true;
                    return false;
                }
            } else {
                key_press_status &= ~KEY_B_PRESSED;
                query_vol_flag = false;
            }
            return true;

        case KC_L:
            if (record->event.pressed) {
                key_press_status |= KEY_L_PRESSED;
                if (key_press_status == FUN_LOCK_SCREEN) {
                    if (get_highest_layer(default_layer_state) == 0) {
                        tap_code16_delay(G(KC_L), 10);
                    } else {
                        tap_code16_delay(G(C(KC_Q)), 10);
                    }
                    return false;
                }
            } else {
                key_press_status &= ~KEY_L_PRESSED;
            }
            return true;

        case KC_1:
            if (record->event.pressed) {
                key_press_status |= KEY_1_PRESSED;
                if (key_press_status == FUN_BT1_MODE) {
                    if ((dev_info.devs != DEVS_HOST1) && (!gpio_read_pin(BT_MODE_SW_PIN))) {
                        bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
                        single_blink_cnt   = 6;
                        single_blink_index = BT1_LED_INDEX;
                        single_blink_color = (RGB)BT1_LED_COLOR;
                        single_blink_time  = timer_read32();
                    }
                    // long_pressed_keys[0].press_time = timer_read32();
                    return false;
                }
            } else {
                key_press_status &= ~KEY_1_PRESSED;
                long_pressed_keys[0].press_time = 0;
            }
            return true;

        case KC_2:
            if (record->event.pressed) {
                key_press_status |= KEY_2_PRESSED;
                if (key_press_status == FUN_BT2_MODE) {
                    if ((dev_info.devs != DEVS_HOST2) && (!gpio_read_pin(BT_MODE_SW_PIN))) {
                        bt_switch_mode(dev_info.devs, DEVS_HOST2, false);
                        single_blink_cnt   = 6;
                        single_blink_index = BT2_LED_INDEX;
                        single_blink_color = (RGB)BT2_LED_COLOR;
                        single_blink_time  = timer_read32();
                    }
                    // long_pressed_keys[1].press_time = timer_read32();
                    return false;
                }
            } else {
                key_press_status &= ~KEY_2_PRESSED;
                long_pressed_keys[1].press_time = 0;
            }
            return true;

        case KC_3:
            if (record->event.pressed) {
                key_press_status |= KEY_3_PRESSED;
                if (key_press_status == FUN_BT3_MODE) {
                    if ((dev_info.devs != DEVS_HOST3) && (!gpio_read_pin(BT_MODE_SW_PIN))) {
                        bt_switch_mode(dev_info.devs, DEVS_HOST3, false);
                        single_blink_cnt   = 6;
                        single_blink_index = BT3_LED_INDEX;
                        single_blink_color = (RGB)BT3_LED_COLOR;
                        single_blink_time  = timer_read32();
                    }
                    // long_pressed_keys[2].press_time = timer_read32();
                    return false;
                }
            } else {
                key_press_status &= ~KEY_3_PRESSED;
                long_pressed_keys[2].press_time = 0;
            }
            return true;

        case KC_ESC:
            if (record->event.pressed) {
                key_press_status |= KEY_ESC_PRESSED;
                if (key_press_status == FUN_FACTORY_RESET) {
                    long_pressed_keys[3].press_time = timer_read32();
                    return false;
                }
            } else {
                key_press_status &= ~KEY_ESC_PRESSED;
                long_pressed_keys[3].press_time = 0;
            }
            return true;

        default:
            break;
    }

    return true;
}

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    static uint8_t leds[] = {
        17, 18, 19, 33, 55, 67, 69,
    };
    if ((key_press_status & KEY_FLASK_PRESSED) != 0) {
        for (uint8_t i = 0; i < (sizeof(leds) / sizeof(leds[0])); i++) {
            rgb_matrix_set_color(leds[i], 0xFF, 0xF4, 0xE5);
        }
    }
    return true;
}
