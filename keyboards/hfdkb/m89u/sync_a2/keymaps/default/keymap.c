/* Copyright 2025 @ HFD (https://www.hfdic.com/)
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
    MAC_B,
    MAC_FN,
};

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [WIN_B] = LAYOUT_numpad_6x4(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(WIN_FN),
        KC_NUM,  KC_EQL,  KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
        KC_P0,   KC_PDOT
    ),

    [WIN_FN] = LAYOUT_numpad_6x4(
        NK_TOGG, _______, _______, _______,
        RM_TOGG, _______, _______, EE_CLR,
        RM_HUEU, RM_VALU, RM_NEXT, RM_SATU,
        _______, _______, RM_SPDU, _______,
        _______, _______, _______, _______,
        _______, _______
    ),

    [MAC_B] = LAYOUT_numpad_6x4(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(MAC_FN),
        KC_NUM,  KC_PEQL, KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
        KC_P0,   KC_PDOT
    ),

    [MAC_FN] = LAYOUT_numpad_6x4(
        NK_TOGG, _______, _______, _______,
        RM_TOGG, _______, _______, EE_CLR,
        RM_HUEU, RM_VALU, RM_NEXT, RM_SATU,
        _______, _______, RM_SPDU, _______,
        _______, _______, _______, _______,
        _______, _______
    )
};
