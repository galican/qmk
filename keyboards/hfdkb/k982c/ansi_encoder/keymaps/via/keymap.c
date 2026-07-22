/* Copyright 2023 @ Keychron (https://www.keychron.com)
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

#ifdef MM_MODE_ENABLE
#    include "bt_task.h"
#    include "lcd.h"
#endif

// clang-format off

enum layers{
    WIN_BASE,
    WIN_FN,
    MAC_BASE,
    MAC_FN,
};

#define KC_SPOT KC_SPOTLIGHT
#define BT_1 BT_HOST1
#define BT_2 BT_HOST2
#define BT_3 BT_HOST3
#define WL_2G4 BT_2_4G
#define IND_TOGG IND_LIGHT_OFF

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [WIN_BASE] = LAYOUT_ansi(
        KC_ESC,             KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,      KC_F10,   KC_F11,   KC_F12,   KC_MUTE,
        KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,       KC_MINS,  KC_EQL,   KC_BSPC,  KC_DEL,
        KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,       KC_LBRC,  KC_RBRC,  KC_BSLS,  KC_END,
        KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,    KC_QUOT,            KC_ENT,   KC_PGUP,
        KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,     KC_SLSH,  KC_RSFT,  KC_UP,    KC_PGDN,
        KC_LCTL,  KC_LCMD,  KC_LALT,                                KC_SPC,                       KC_RALT,  MO(WIN_FN), KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RGHT),

    [WIN_FN] = LAYOUT_ansi(
        EE_CLR,             KC_MYCM,  KC_WHOM,  KC_MAIL,  KC_CALC,  KC_MSEL,  KC_MSTP,  KC_MPRV,  KC_MPLY,  KC_MNXT,    KC_MUTE,  KC_VOLD,  KC_VOLU,  _______,
        _______,  BT_1,     BT_2,     BT_3,     WL_2G4,   _______,  _______,  _______,  LCD_HOME,  _______,  LCD_PAGE,    _______,  _______,  RM_TOGG,  LCD_TOGGLE,
        LED_WHITE, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,    _______,  _______,  RM_NEXT,  LCD_TIME,
        _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,    _______,            RM_HUEU,  _______,
        _______,            _______,  _______,  VIA_WEB,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  RM_VALU,  _______,
        _______,  GU_TOGG,  _______,                                _______,                      IND_TOGG, _______,    BT_VOL,   RM_SPDD,  RM_VALD,  RM_SPDU),

    [MAC_BASE] = LAYOUT_ansi(
        KC_ESC,             KC_BRID,  KC_BRIU,  KC_MCTL,  KC_LPAD,  KC_SIRI,  KC_SPOT,  KC_MPRV,  KC_MPLY,  KC_MNXT,    KC_MUTE,  KC_VOLD,  KC_VOLU,  KC_MUTE,
        KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,       KC_MINS,  KC_EQL,   KC_BSPC,  KC_DEL,
        KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,       KC_LBRC,  KC_RBRC,  KC_BSLS,  KC_END,
        KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,    KC_QUOT,            KC_ENT,   KC_PGUP,
        KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,     KC_SLSH,  KC_RSFT,  KC_UP,    KC_PGDN,
        KC_LCTL,  KC_LOPT,  KC_LCMD,                                KC_SPC,                       KC_RCMD,  MO(MAC_FN), KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RGHT),

    [MAC_FN] = LAYOUT_ansi(
        EE_CLR,             KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,      KC_F10,   KC_F11,   KC_F12,   _______,
        _______,  BT_1,     BT_2,     BT_3,     WL_2G4,   _______,  _______,  _______,  LCD_HOME,  _______,  LCD_PAGE,    _______,  _______,  RM_TOGG,  LCD_TOGGLE,
        LED_WHITE, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,    _______,  _______,  RM_NEXT,  LCD_TIME,
        _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,    _______,            RM_HUEU,  _______,
        _______,            _______,  _______,  VIA_WEB,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  RM_VALU,  _______,
        _______,  _______,  _______,                                _______,                      IND_TOGG, _______,    BT_VOL,   RM_SPDD,  RM_VALD,  RM_SPDU),

};

#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [WIN_BASE] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU)},
    [WIN_FN]   = { ENCODER_CCW_CW(_______, _______)},
    [MAC_BASE] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU)},
    [MAC_FN]   = { ENCODER_CCW_CW(_______, _______)},
};
#endif // ENCODER_MAP_ENABLE

// clang-format on

bool no_indicator_under_srgb = false;

static uint8_t  all_blink_cnt      = 0;
static RGB      all_blink_color    = {0};
static uint32_t all_blink_time     = 0;
static uint8_t  single_blink_cnt   = 0;
static uint8_t  single_blink_index = 0;
static RGB      single_blink_color = {0};
static uint32_t single_blink_time  = 0;

static bool     is_siri_active = false;
static uint32_t siri_timer     = 0;

enum via_web_states {
    VIA_WEB_IDLE,
    VIA_WEB_LAUNCH_RELEASE,
    VIA_WEB_LAUNCH_WAIT,
    VIA_WEB_CAPS_RELEASE,
    VIA_WEB_CAPS_WAIT,
    VIA_WEB_CHAR_PRESS,
    VIA_WEB_CHAR_RELEASE,
    VIA_WEB_SUBMIT_WAIT,
    VIA_WEB_ENTER_RELEASE,
    VIA_WEB_CAPS_RESTORE_WAIT,
    VIA_WEB_CAPS_RESTORE_RELEASE,
};

static const uint16_t PROGMEM via_web_keycodes[] = {
    KC_H, KC_T, KC_T, KC_P, KC_S, S(KC_SCLN), KC_SLSH, KC_SLSH, KC_Y, KC_U, KC_N, KC_Z, KC_I, KC_I, KC_DOT, KC_D, KC_R, KC_I, KC_V, KC_E, KC_A, KC_L, KC_L, KC_DOT, KC_C, KC_N, KC_SLSH,
};

static uint8_t  via_web_state = VIA_WEB_IDLE;
static uint8_t  via_web_index = 0;
static uint16_t via_web_keycode;
static uint32_t via_web_timer;
static bool     via_web_gui_was_locked;
static bool     via_web_caps_was_on;
static bool     via_web_is_mac;

static void start_via_web(void) {
    if (via_web_state != VIA_WEB_IDLE) {
        return;
    }

    via_web_gui_was_locked = keymap_config.no_gui;
    via_web_caps_was_on    = host_keyboard_led_state().caps_lock;
    via_web_is_mac         = get_highest_layer(default_layer_state) == MAC_BASE;
    via_web_index          = 0;

    keymap_config.no_gui = false;
    via_web_keycode      = via_web_is_mac ? G(KC_SPACE) : G(KC_R);
    register_code16(via_web_keycode);
    via_web_timer = timer_read32();
    via_web_state = VIA_WEB_LAUNCH_RELEASE;
}

static void via_web_task(void) {
    switch (via_web_state) {
        case VIA_WEB_LAUNCH_RELEASE:
            if (timer_elapsed32(via_web_timer) >= 30) {
                unregister_code16(via_web_keycode);
                keymap_config.no_gui = via_web_gui_was_locked;
                via_web_timer        = timer_read32();
                via_web_state        = VIA_WEB_LAUNCH_WAIT;
            }
            break;

        case VIA_WEB_LAUNCH_WAIT:
            if (timer_elapsed32(via_web_timer) >= 600) {
                via_web_timer = timer_read32();
                if (via_web_caps_was_on) {
                    via_web_state = VIA_WEB_CHAR_PRESS;
                } else {
                    register_code(KC_CAPS);
                    via_web_state = VIA_WEB_CAPS_RELEASE;
                }
            }
            break;

        case VIA_WEB_CAPS_RELEASE:
            if (timer_elapsed32(via_web_timer) >= TAP_HOLD_CAPS_DELAY) {
                unregister_code(KC_CAPS);
                via_web_timer = timer_read32();
                via_web_state = VIA_WEB_CAPS_WAIT;
            }
            break;

        case VIA_WEB_CAPS_WAIT:
            if (timer_elapsed32(via_web_timer) >= 100) {
                via_web_timer = timer_read32();
                via_web_state = VIA_WEB_CHAR_PRESS;
            }
            break;

        case VIA_WEB_CHAR_PRESS:
            if (timer_elapsed32(via_web_timer) >= 10) {
                via_web_keycode = pgm_read_word(&via_web_keycodes[via_web_index]);
                register_code16(via_web_keycode);
                via_web_timer = timer_read32();
                via_web_state = VIA_WEB_CHAR_RELEASE;
            }
            break;

        case VIA_WEB_CHAR_RELEASE:
            if (timer_elapsed32(via_web_timer) >= 20) {
                unregister_code16(via_web_keycode);
                via_web_timer = timer_read32();
                via_web_index++;
                via_web_state = via_web_index < ARRAY_SIZE(via_web_keycodes) ? VIA_WEB_CHAR_PRESS : VIA_WEB_SUBMIT_WAIT;
            }
            break;

        case VIA_WEB_SUBMIT_WAIT:
            if (timer_elapsed32(via_web_timer) >= (via_web_is_mac ? 500 : 100)) {
                register_code(KC_ENTER);
                via_web_timer = timer_read32();
                via_web_state = VIA_WEB_ENTER_RELEASE;
            }
            break;

        case VIA_WEB_ENTER_RELEASE:
            if (timer_elapsed32(via_web_timer) >= (via_web_is_mac ? 100 : 50)) {
                unregister_code(KC_ENTER);
                via_web_timer = timer_read32();
                via_web_state = via_web_caps_was_on ? VIA_WEB_IDLE : VIA_WEB_CAPS_RESTORE_WAIT;
            }
            break;

        case VIA_WEB_CAPS_RESTORE_WAIT:
            if (timer_elapsed32(via_web_timer) >= 100) {
                register_code(KC_CAPS);
                via_web_timer = timer_read32();
                via_web_state = VIA_WEB_CAPS_RESTORE_RELEASE;
            }
            break;

        case VIA_WEB_CAPS_RESTORE_RELEASE:
            if (timer_elapsed32(via_web_timer) >= TAP_HOLD_CAPS_DELAY) {
                unregister_code(KC_CAPS);
                via_web_state = VIA_WEB_IDLE;
            }
            break;

        default:
            break;
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case IND_TOGG:
            if (record->event.pressed) {
                if (dev_info.ind_toggle) {
                    dev_info.ind_toggle = 0;
                } else {
                    dev_info.ind_toggle = 1;
                }
                eeconfig_update_user(dev_info.raw);
            }
            return false;

        case RM_VALU:
            if (!no_indicator_under_srgb) {
                if (record->event.pressed) {
                    if (rgb_matrix_get_val() >= (RGB_MATRIX_MAXIMUM_BRIGHTNESS - RGB_MATRIX_VAL_STEP)) {
                        single_blink_cnt   = 6;
                        single_blink_index = 70;
                        single_blink_color = (RGB){100, 100, 100};
                        single_blink_time  = timer_read32();
                    }
                }
                break;
            } else {
                return false;
            }
        case RM_VALD:
            if (!no_indicator_under_srgb) {
                if (record->event.pressed) {
                    if (rgb_matrix_get_val() <= RGB_MATRIX_VAL_STEP) {
                        single_blink_cnt   = 6;
                        single_blink_index = 82;
                        single_blink_color = (RGB){100, 100, 100};
                        single_blink_time  = timer_read32();
                    }
                }
                break;
            } else {
                return false;
            }
        case RM_SPDU:
            if (!no_indicator_under_srgb) {
                if (record->event.pressed) {
                    if (rgb_matrix_get_speed() >= (UINT8_MAX - RGB_MATRIX_SPD_STEP)) {
                        single_blink_cnt   = 6;
                        single_blink_index = 83;
                        single_blink_color = (RGB){100, 100, 100};
                        single_blink_time  = timer_read32();
                    }
                }
                break;
            } else {
                return false;
            }
        case RM_SPDD:
            if (!no_indicator_under_srgb) {
                if (record->event.pressed) {
                    if (rgb_matrix_get_speed() <= RGB_MATRIX_SPD_STEP) {
                        single_blink_cnt   = 6;
                        single_blink_index = 81;
                        single_blink_color = (RGB){100, 100, 100};
                        single_blink_time  = timer_read32();
                    }
                }
                break;
            } else {
                return false;
            }

        case LCD_HOME: {
            if (record->event.pressed) {
                dev_info.LCD_Page = 0;
                LCD_Page_update(dev_info.LCD_Page);
                eeconfig_update_kb(dev_info.raw);
            }
            return false;
        }
        case LCD_PAGE: {
            if (record->event.pressed) {
                dev_info.LCD_Page = 1;
                LCD_Page_update(dev_info.LCD_Page);
                eeconfig_update_kb(dev_info.raw);
            }
            return false;
        }
        case LCD_TOGGLE: {
            if (record->event.pressed) {
                LCD_command_update(LCD_ON_OFF);
            }
            return false;
        }
        case LCD_TIME: {
            if (record->event.pressed) {
                LCD_command_update(LCD_TIME_12H_24H);
            }
            return false;
        }

        case KC_SIRI:
            if (record->event.pressed) {
                if (!is_siri_active) {
                    is_siri_active = true;
                    register_code(KC_LCMD);
                    register_code(KC_SPACE);
                }
                siri_timer = timer_read32();
            } else {
                // Do something else when release
            }
            return false; // Skip all further processing of this key

        case KC_SPOTLIGHT:
            if (dev_info.devs == DEVS_USB) {
                if (record->event.pressed) {
                    host_consumer_send(0x0221);
                } else {
                    host_consumer_send(0);
                }
            } else {
                if (record->event.pressed) {
                    bts_send_consumer(0x221);
                } else {
                    bts_send_consumer(0);
                }
            }
            return false;

        case VIA_WEB:
            if (record->event.pressed) {
                start_via_web();
            }
            return false;

        default:
            break;
    }

    return true;
}

void housekeeping_task_user(void) {
    via_web_task();

    if (is_siri_active) {
        if (timer_elapsed32(siri_timer) >= 500) {
            unregister_code(KC_LCMD);
            unregister_code(KC_SPACE);
            is_siri_active = false;
        }
    }
}

#ifdef RGB_MATRIX_ENABLE
bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    if (single_blink_cnt) {
        if (timer_elapsed32(single_blink_time) > 500) {
            single_blink_time = timer_read32();
            single_blink_cnt--;
        }
        if (single_blink_cnt % 2) {
            rgb_matrix_set_color(single_blink_index, single_blink_color.r, single_blink_color.g, single_blink_color.b);
        } else {
            rgb_matrix_set_color(single_blink_index, 0, 0, 0);
        }
    }
    if (all_blink_cnt) {
        if (timer_elapsed32(all_blink_time) > 500) {
            all_blink_time = timer_read32();
            all_blink_cnt--;
        }
        if (all_blink_cnt % 2) {
            rgb_matrix_set_color_all(all_blink_color.r, all_blink_color.g, all_blink_color.b);
        } else {
            rgb_matrix_set_color_all(0, 0, 0);
        }
    }

    return true;
}
#endif
