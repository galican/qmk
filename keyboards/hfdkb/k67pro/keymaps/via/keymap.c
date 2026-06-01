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

enum __layers {
    WIN_B,
    WIN_FN,
    WIN_EX,
    MAC_B,
    MAC_FN,
    MAC_EX,
};

#define BT1 BT_HOST1
#define BT2 BT_HOST2
#define BT3 BT_HOST3
#define BT4 BT_2_4G
#define KC_TASK G(KC_TAB)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [WIN_B] = LAYOUT_ansi_67( /* Base */
        KC_ESC,  KC_1,     KC_2,     KC_3,     KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,  KC_BSPC, RM_NEXT,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC, KC_BSLS, KC_DEL,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,             KC_ENT,  KC_PGUP,
        KC_LSFT,           KC_Z,     KC_X,     KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,    KC_RSFT, KC_UP,   KC_PGDN,
        KC_LCTL, KC_LWIN,  KC_LALT,                              KC_SPC,                             KC_RALT, MO(WIN_FN), KC_LEFT, KC_DOWN, KC_RGHT ),

    [WIN_FN] = LAYOUT_ansi_67( /* FN */
        EE_CLR,  KC_BRID,  KC_BRIU,  KC_WSCH,  KC_TASK, RM_VALD, RM_VALU, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, _______, _______,
        RM_HUEU, BT1,      BT2,      BT3,      BT4,     _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, MO(WIN_EX),
        _______,           _______,  _______,  _______, _______, _______,  _______, _______, _______, _______, _______,    _______, RM_VALU, _______,
        _______, GU_TOGG,  _______,                              _______,                             _______, _______,    RM_SPDD, RM_VALD, RM_SPDU),

    [WIN_EX] = LAYOUT_ansi_67( /* FN */
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, _______,
        _______,           _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, RGB_TEST,
        _______, _______,  _______,                              _______,                            _______, _______,    _______, _______, _______),

    [MAC_B] = LAYOUT_ansi_67( /* Base */
        KC_ESC,  KC_1,     KC_2,     KC_3,     KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,  KC_BSPC, RM_NEXT,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC, KC_BSLS, KC_DEL,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,             KC_ENT,  KC_PGUP,
        KC_LSFT,           KC_Z,     KC_X,     KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,    KC_RSFT, KC_UP,   KC_PGDN,
        KC_LCTL, KC_LOPT,  KC_LCMD,                              KC_SPC,                             KC_RCMD, MO(MAC_FN), KC_LEFT, KC_DOWN, KC_RGHT ),

    [MAC_FN] = LAYOUT_ansi_67( /* FN */
        EE_CLR,  KC_BRID,  KC_BRIU,  KC_MCTL,  KC_LPAD, RM_VALD, RM_VALU, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, _______, _______,
        RM_HUEU, BT1,      BT2,      BT3,      BT4,     _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, MO(MAC_EX),
        _______,           _______,  _______,  _______, _______, _______,  _______, _______, _______, _______, _______,    _______, RM_VALU, _______,
        _______, _______,  _______,                              _______,                             _______, _______,    RM_SPDD, RM_VALD, RM_SPDU),

    [MAC_EX] = LAYOUT_ansi_67( /* FN */
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______,
        _______, _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,             _______, _______,
        _______,           _______,  _______,  _______, _______, _______, _______, _______, _______, _______, _______,    _______, _______, RGB_TEST,
        _______, _______,  _______,                              _______,                            _______, _______,    _______, _______, _______),
};
