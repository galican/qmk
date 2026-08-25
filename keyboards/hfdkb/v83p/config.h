// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#define BT_MODE_ENABLE

#ifdef BT_MODE_ENABLE
#    define NO_USB_STARTUP_CHECK
#    define ENTRY_STOP_MODE  // 超时进入STOP Mode
#    define BT_CABLE_PIN B8  // 充电接入时为高
#    define BT_CHARGE_PIN B9 // 充电时为低，充满时为高
#    define RGB_DRIVER_SDB_PIN A15
#    define BT_MODE_SW_PIN C10 // 低电平时
#    define RF_MODE_SW_PIN C13 // 低电平时
// #  define INDLED_USB_PIN C11
// #  define INDLED_BT_PIN D2
// #  define INDLED_2_4G_PIN C10
#    define KEY_NUM 6
#endif

#define ENCODER_DEFAULT_POS 0x3

/* I2C Config for LED Driver */
#define IS31FL3733_DRIVER_COUNT 2
#define IS31FL3733_I2C_ADDRESS_1 0b1110100
#define IS31FL3733_I2C_ADDRESS_2 0b1110111

#define I2C1_OPMODE OPMODE_I2C
#define I2C1_CLOCK_SPEED 400000

/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5

#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12

#define DYNAMIC_KEYMAP_MACRO_DELAY 5
// #define TAP_CODE_DELAY 5
