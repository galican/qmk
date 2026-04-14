/* Copyright (C) 2023 jonylee@hfd
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
#include "common/bt_task.h"

// clang-format off

enum layers {
    WIN_B,
    WIN_F,
    WIN_F2,
    MAC_B,
    MAC_F,
    MAC_F2,
    LINUX_B,
    LINUX_F,
    LINUX_F2,
};

#define BT_1 BT_HOST1
#define BT_2 BT_HOST2
#define BT_3 BT_HOST3
#define WL_2_4 BT_2_4G

typedef struct PACKED {
    uint8_t len;
    uint8_t keycode[3];
} key_combination_t;

key_combination_t key_comb_list[4] = {
    {3, {KC_LCTL, KC_LSFT, KC_ESC}},
    {2, {KC_LWIN, KC_K}},
    {2, {KC_LCMD, KC_SPC}},
    {3, {KC_LCMD, KC_LSFT, KC_4}},
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [WIN_B] = LAYOUT_87_ansi( /* Base */
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           KC_UP,
        KC_LCTL, KC_LWIN, KC_LALT,                            KC_SPC,                             KC_RALT, MO(1),   KC_APP,  KC_RCTL, KC_LEFT,  KC_DOWN, KC_RGHT),

    [WIN_F] = LAYOUT_87_ansi( /* FN */
        EE_CLR,           KC_BRID, KC_BRIU, KC_CALC, KC_MYCM, KC_TASK, KC_PRJT, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD, KC_VOLU, _______, _______, _______,
        _______, BT_1,    BT_2,    BT_3,    WL_2_4,  _______, SW_OS2,  SW_OS3,  _______, _______, _______, RM_NEXT, RM_HUEU, KC_DEL,  _______, _______, SLED_TOG,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_PSCR, KC_SCRL, KC_PAUS, _______, _______, _______, SLED_MOD,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_INS,           _______, _______,
        _______,          _______, RM_TOGG, _______, _______, _______, _______, _______, LCD_LEFT,LCD_RIGHT,_______,         _______,          RM_VALU,
        _______, GU_TOGG, _______,                            _______,                            MO(2),   _______, _______, _______, RM_SPDD, RM_VALD, RM_SPDU),

    [WIN_F2] = LAYOUT_87_ansi( /* FN */
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,   SLED_VAI,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,  _______, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, SLED_SPD, SLED_VAD, SLED_SPI,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    SLED_HUI,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           _______,
        KC_LCTL, KC_LWIN, KC_LALT,                            KC_SPC,                             KC_RALT, MO(1),   KC_APP,  KC_RCTL, _______,  _______, _______),

    [MAC_B] = LAYOUT_87_ansi( /* Base */
        KC_ESC,           KC_BRID, KC_BRIU, KC_MCTL, KC_SEAR, KC_SIRI, KC_SNAP, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD, KC_VOLU, KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           KC_UP,
        KC_LCTL, KC_LOPT, KC_LCMD,                            KC_SPC,                             KC_RCMD, MO(4),   KC_APP,  KC_RCTL, KC_LEFT,  KC_DOWN, KC_RGHT),

    [MAC_F] = LAYOUT_87_ansi( /* FN */
        EE_CLR,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______, _______, _______,
        _______, BT_1,    BT_2,    BT_3,    WL_2_4,  SW_OS1, _______,  SW_OS3,  _______, _______, _______, RM_NEXT, RM_HUEU, KC_DEL,  _______, _______, SLED_TOG,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, RM_NEXT, _______, _______, SLED_MOD,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______,
        _______,          _______, RM_TOGG, _______, _______, _______, _______, _______, LCD_LEFT,LCD_RIGHT,_______,         _______,          RM_VALU,
        _______, _______, _______,                            _______,                            MO(5),   _______, _______, _______, RM_SPDD, RM_VALD, RM_SPDU),

    [MAC_F2] = LAYOUT_87_ansi( /* FN */
        KC_ESC,           KC_BRID, KC_BRIU, KC_MCTL, KC_SEAR, KC_SIRI, KC_SNAP, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD, KC_VOLU, KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,   SLED_VAI,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,  _______, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, SLED_SPD, SLED_VAD, SLED_SPI,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    SLED_HUI,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           _______,
        KC_LCTL, KC_LOPT, KC_LCMD,                            KC_SPC,                             KC_RCMD, MO(4),   KC_APP,  KC_RCTL, _______,  _______, _______),

    [LINUX_B] = LAYOUT_87_ansi( /* FN */
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           KC_UP,
        KC_LCTL, KC_LWIN, KC_LALT,                            KC_SPC,                             KC_RALT, MO(7),   KC_APP,  KC_RCTL, KC_LEFT,  KC_DOWN, KC_RGHT),

    [LINUX_F] = LAYOUT_87_ansi( /* FN */
        EE_CLR,           KC_BRID, KC_BRIU, KC_CALC, KC_MYCM, KC_TASK, KC_PRJT, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD, KC_VOLU, _______, _______, _______,
        _______, BT_1,    BT_2,    BT_3,    WL_2_4,  SW_OS1,  SW_OS2,  _______, _______, _______, _______, RM_NEXT, RM_HUEU, KC_DEL,  _______, _______, SLED_TOG,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_PSCR, KC_SCRL, KC_PAUS, RM_NEXT, _______, _______, SLED_MOD,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_INS,           _______, _______,
        _______,          _______, RM_TOGG, _______, _______, _______, _______, _______, LCD_LEFT,LCD_RIGHT,_______,         _______,          RM_VALU,
        _______, GU_TOGG, _______,                            _______,                            MO(8),   _______, _______, _______, RM_SPDD, RM_VALD, RM_SPDU),

    [LINUX_F2] = LAYOUT_87_ansi( /* FN */
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_PSCR,  KC_SCRL, KC_PAUS,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_INS,   KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    SLED_VAI,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, _______, KC_DEL,   KC_END,  KC_PGDN,
        KC_CAPS, SLED_SPD, SLED_VAD, SLED_SPI,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_MUTE,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    SLED_HUI,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,           _______,
        KC_LCTL, KC_LWIN, KC_LALT,                            KC_SPC,                             KC_RALT, MO(7),   KC_APP,  KC_RCTL, _______,  _______, _______),

};

#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [WIN_B]  = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU)},
    [WIN_F]  = {ENCODER_CCW_CW(_______, _______)},
    [WIN_F2] = {ENCODER_CCW_CW(_______, _______)},
    [MAC_B]  = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU)},
    [MAC_F]  = {ENCODER_CCW_CW(_______, _______)},
    [MAC_F2] = {ENCODER_CCW_CW(_______, _______)},
    [LINUX_B]  = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU)},
    [LINUX_F]  = {ENCODER_CCW_CW(_______, _______)},
    [LINUX_F2] = {ENCODER_CCW_CW(_______, _______)},
};
#endif // ENCODER_MAP_ENABLE
// clang-format on
#if defined(DIP_SWITCH_MAP_ENABLE)
const uint16_t PROGMEM dip_switch_map[NUM_DIP_SWITCHES][NUM_DIP_STATES] = {DIP_SWITCH_OFF_ON(KC_MUTE, _______)};
#endif

bool     is_siri_active = false;
uint32_t siri_timer     = 0;

// enum {
//     KEY_PRESS_LCTL         = 0x01 << 1,
//     KEY_PRESS_LALT         = 0x01 << 2,
//     KEY_PRESS_UP           = 0x01 << 3,
//     KEY_PRESS_DOWN         = 0x01 << 4,
//     KEY_PRESS_LEFT         = 0x01 << 5,
//     KEY_PRESS_RIGHT        = 0x01 << 6,
//     KEY_PRESS_SLED_VAL_INC = KEY_PRESS_LCTL | KEY_PRESS_LALT | KEY_PRESS_UP,
//     KEY_PRESS_SLED_VAL_DEC = KEY_PRESS_LCTL | KEY_PRESS_LALT | KEY_PRESS_DOWN,
//     KEY_PRESS_SLED_SPD_DEC = KEY_PRESS_LCTL | KEY_PRESS_LALT | KEY_PRESS_LEFT,
//     KEY_PRESS_SLED_SPD_INC = KEY_PRESS_LCTL | KEY_PRESS_LALT | KEY_PRESS_RIGHT,
// };
// uint8_t key_pressed = 0;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case KC_SIRI:
            if (record->event.pressed) {
                if (!is_siri_active) {
                    is_siri_active = true;
                    register_code(KC_LCMD);
                    register_code(KC_SPACE);
                }
                siri_timer = timer_read32();
            }
            return false; // Skip all further processing of this key

        case KC_TASK:
        case KC_PRJT:
        case KC_SEAR:
        case KC_SNAP:
            if (record->event.pressed) {
                for (uint8_t i = 0; i < key_comb_list[keycode - KC_TASK].len; i++) {
                    register_code(key_comb_list[keycode - KC_TASK].keycode[i]);
                }
            } else {
                for (uint8_t i = 0; i < key_comb_list[keycode - KC_TASK].len; i++) {
                    unregister_code(key_comb_list[keycode - KC_TASK].keycode[i]);
                }
            }
            return false; // Skip all further processing of this key

            // case KC_LCTL:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_LCTL;
            //     } else {
            //         key_pressed &= ~KEY_PRESS_LCTL;
            //     }
            //     break;
            // case KC_LALT:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_LALT;
            //     } else {
            //         key_pressed &= ~KEY_PRESS_LALT;
            //     }
            //     break;
            // case KC_UP:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_UP;
            //         if (key_pressed == KEY_PRESS_SLED_VAL_INC) {
            //             SLED_brightness_increase();
            //         }
            //     } else {
            //         key_pressed &= ~KEY_PRESS_UP;
            //     }
            //     break;
            // case KC_DOWN:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_DOWN;
            //         if (key_pressed == KEY_PRESS_SLED_VAL_DEC) {
            //             SLED_brightness_decrease();
            //         }
            //     } else {
            //         key_pressed &= ~KEY_PRESS_DOWN;
            //     }
            //     break;
            // case KC_LEFT:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_LEFT;
            //         if (key_pressed == KEY_PRESS_SLED_SPD_DEC) {
            //             SLED_speed_decrease();
            //         }
            //     } else {
            //         key_pressed &= ~KEY_PRESS_LEFT;
            //     }
            //     break;
            // case KC_RGHT:
            //     if (record->event.pressed) {
            //         key_pressed |= KEY_PRESS_RIGHT;
            //         if (key_pressed == KEY_PRESS_SLED_SPD_INC) {
            //             SLED_speed_increase();
            //         }
            //     } else {
            //         key_pressed &= ~KEY_PRESS_RIGHT;
            //     }
            //     break;

        default:
            break;
    }

    return true;
}

void housekeeping_task_user(void) {
    if (is_siri_active) {
        if (timer_elapsed32(siri_timer) >= 500) {
            unregister_code(KC_LCMD);
            unregister_code(KC_SPACE);
            is_siri_active = false;
        }
    }
}

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    uint8_t layer = get_highest_layer(layer_state);

    if (layer == WIN_F || layer == MAC_F || layer == LINUX_F) {
        uint8_t base_layer = (layer == WIN_F) ? WIN_B : (layer == MAC_F) ? MAC_B : LINUX_B;
        for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
            for (uint8_t col = 0; col < MATRIX_COLS; col++) {
                uint8_t index = g_led_config.matrix_co[row][col];
                if (index >= led_min && index < 87 && index != NO_LED) {
                    uint16_t keycode = keymap_key_to_keycode(layer, (keypos_t){col, row});
                    if (keycode > KC_TRNS) {
                        rgb_matrix_set_color(index, 80, 80, 80);
                    } else {
                        uint16_t base_keycode = keymap_key_to_keycode(base_layer, (keypos_t){col, row});
                        if (base_keycode == MO(layer)) {
                            rgb_matrix_set_color(index, 80, 80, 80);
                        }
                    }
                }
            }
        }
    }

    return true;
}

// enum combos { SLED_Brightness_Inc, SLED_Brightness_Dec, SLED_Speed_Up, SLED_Speed_Down };

// const uint16_t PROGMEM svu_combo[] = {KC_LCTL, KC_LALT, KC_UP, COMBO_END};
// const uint16_t PROGMEM svd_combo[] = {KC_LCTL, KC_LALT, KC_DOWN, COMBO_END};
// const uint16_t PROGMEM ssu_combo[] = {KC_LCTL, KC_LALT, KC_LEFT, COMBO_END};
// const uint16_t PROGMEM ssd_combo[] = {KC_LCTL, KC_LALT, KC_RIGHT, COMBO_END};

// combo_t key_combos[] = {
//     [SLED_Brightness_Inc] = COMBO(svu_combo, SLED_VAI),
//     [SLED_Brightness_Dec] = COMBO(svd_combo, SLED_VAD),
//     [SLED_Speed_Up]       = COMBO(ssu_combo, SLED_SPI),
//     [SLED_Speed_Down]     = COMBO(ssd_combo, SLED_SPD),
// };

// In config.h:
// #define COMBO_TERM 200  // 200ms window

#ifdef COMBO_ENABLE

extern void SLED_brightness_increase(void);
extern void SLED_brightness_decrease(void);
extern void SLED_speed_increase(void);
extern void SLED_speed_decrease(void);

enum combos { COMBO_SLED_VAL_INC, COMBO_SLED_VAL_DEC, COMBO_SLED_SPD_INC, COMBO_SLED_SPD_DEC, COMBO_LENGTH };
uint16_t COMBO_LEN = COMBO_LENGTH;

const uint16_t PROGMEM sled_val_inc_combo[] = {KC_LCTL, KC_LALT, KC_UP, COMBO_END};
const uint16_t PROGMEM sled_val_dec_combo[] = {KC_LCTL, KC_LALT, KC_DOWN, COMBO_END};
const uint16_t PROGMEM sled_spd_inc_combo[] = {KC_LCTL, KC_LALT, KC_RGHT, COMBO_END};
const uint16_t PROGMEM sled_spd_dec_combo[] = {KC_LCTL, KC_LALT, KC_LEFT, COMBO_END};

combo_t key_combos[] = {
    [COMBO_SLED_VAL_INC] = COMBO_ACTION(sled_val_inc_combo),
    [COMBO_SLED_VAL_DEC] = COMBO_ACTION(sled_val_dec_combo),
    [COMBO_SLED_SPD_INC] = COMBO_ACTION(sled_spd_inc_combo),
    [COMBO_SLED_SPD_DEC] = COMBO_ACTION(sled_spd_dec_combo),
};

void process_combo_event(uint16_t combo_index, bool pressed) {
    if (pressed) {
        switch (combo_index) {
            case COMBO_SLED_VAL_INC:
                SLED_brightness_increase();
                break;
            case COMBO_SLED_VAL_DEC:
                SLED_brightness_decrease();
                break;
            case COMBO_SLED_SPD_INC:
                SLED_speed_increase();
                break;
            case COMBO_SLED_SPD_DEC:
                SLED_speed_decrease();
                break;
        }
    }
}
#endif

// const key_override_t sled_val_inc   = ko_make_with_layers_negmods_and_options(MOD_MASK_CA, // Ctrl + Alt must be held
//                                                                               KC_UP,       // trigger key
//                                                                               SLED_VAI,    // output nothing (we handle it ourselves)
//                                                                               ~0,          // all layers
//                                                                               0,           // no negative mods
//                                                                               ko_option_no_reregister_trigger);
// const key_override_t sled_val_dev   = ko_make_with_layers_negmods_and_options(MOD_MASK_CA, // Ctrl + Alt must be held
//                                                                               KC_UP,       // trigger key
//                                                                               SLED_VAD,    // output nothing (we handle it ourselves)
//                                                                               ~0,          // all layers
//                                                                               0,           // no negative mods
//                                                                               ko_option_no_reregister_trigger);
// const key_override_t sled_speed_inc = ko_make_with_layers_negmods_and_options(MOD_MASK_CA, // Ctrl + Alt must be held
//                                                                               KC_UP,       // trigger key
//                                                                               SLED_SPI,    // output nothing (we handle it ourselves)
//                                                                               ~0,          // all layers
//                                                                               0,           // no negative mods
//                                                                               ko_option_no_reregister_trigger);
// const key_override_t sled_speed_dec = ko_make_with_layers_negmods_and_options(MOD_MASK_CA, // Ctrl + Alt must be held
//                                                                               KC_UP,       // trigger key
//                                                                               SLED_SPD,    // output nothing (we handle it ourselves)
//                                                                               ~0,          // all layers
//                                                                               0,           // no negative mods
//                                                                               ko_option_no_reregister_trigger);
// const key_override_t win_func   = ko_make_basic(KC_LCTL, KC_LALT, MO(2));
// const key_override_t mac_func   = ko_make_basic(KC_LCTL, KC_LALT, MO(5));
// const key_override_t linux_func = ko_make_basic(KC_LCTL, KC_LALT, MO(8));

// This globally defines all key overrides to be used
// const key_override_t *key_overrides[] = {&win_func};
