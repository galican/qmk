/* Copyright (C) 2023 Westberry Technology (ChangZhou) Corp., Ltd
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

#pragma once

#include "bts_lib.h"
#include "quantum.h"

// #define BT_DEBUG_MODE
#define ENTRY_STOP_TIMEOUT 100 // ms
// #define ENTRY_STOP_TIMEOUT (30 * 60000) // ms

enum bt_keycodes {
    BT_HOST1 = QK_KB_0,
    BT_HOST2,
    BT_HOST3,
    BT_2_4G,
    BT_USB,
    BT_VOL,
    SW_OS,
    INDICATOR_HUE,
    INDICATOR_BRIGHTNESS,
    ECO,
    FACTORY_RESET,
    KEYBOARD_RESET,
    BLE_RESET,
    // KEY_EQL,

    RGB_TOG,
    RGB_MOD,
    RGB_HUI,
    RGB_SAI,
    RGB_VAI,
    RGB_SPI,
};

// enum custom_keycodes {
// RGB_TEST = SAFE_RANGE,
// RGB_HUD,
// RGB_SAD,
// RGB_VAD,
// RGB_SPD,
// };

typedef union {
    uint32_t raw;
    struct PACKED {
        // 3+3+1+1+4+4+1+1+6
        uint8_t devs : 3;
        uint8_t last_devs : 3;
        bool    eco_tog_flag : 1;
        bool    manual_usb_mode : 1;
        uint8_t smd_color_index : 4;
        uint8_t ind_color_index : 4;
        uint8_t ind_brightness;
        bool    unsync : 1;
        bool    num_unsync : 1;
        uint8_t revered : 6;
    };
} dev_info_t;

extern dev_info_t dev_info;
extern bts_info_t bts_info;

bool get_low_vol_status(void);

/**
 * @brief bluetooth 初始化函数
 * @param None
 * @return None
 */
void bt_init(void);

/**
 * @brief bluetooth交互任务
 * @param None
 * @return None
 */
void bt_task(void);

/**
 * @brief 处理和BT相关的按键
 * @param keycode: 键值
 * @param record: 记录值
 * @return None
 */
bool bt_process_record(uint16_t keycode, keyrecord_t *record);

/**
 * @brief rgb指示灯任务
 * @param None
 * @return None
 */
bool bt_indicators_advanced(uint8_t led_min, uint8_t led_max);

/**
 * @brief 切换工作模式
 * @param None
 * @return None
 */
void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset);
