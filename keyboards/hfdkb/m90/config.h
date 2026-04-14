// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#define BT_MODE_ENABLE
#ifdef BT_MODE_ENABLE
#    define BT_CABLE_PIN B9
#    define BT_CHARGE_PIN B8
#    define RGB_DRIVER_SDB_PIN A15
#    define BT_HOST1_INDEX 14
#    define BT_HOST2_INDEX 15
#    define BT_HOST3_INDEX 16
#    define BT_2_4G_INDEX 17
#    define BT_USB_INDEX 18
#    define BT_MODE_SW_PIN B10

// Indicate color of wl device
#    define BT_HOST1_COLOR 200, 200, 200 // 蓝牙1指示灯颜色
#    define BT_HOST2_COLOR 200, 200, 200 // 蓝牙2指示灯颜色
#    define BT_HOST3_COLOR 200, 200, 200 // 蓝牙3指示灯颜色
#    define BT_2_4G_COLOR 200, 200, 200  // 2.4G指示灯颜色
#    define BT_USB_COLOR 200, 200, 200   // USB指示灯颜色

#    define USB_SUSPEND_CHECK_ENABLE
#    define USBLINK_Status gpio_read_pin(BT_CABLE_PIN)
#    define CHARGE_Status gpio_read_pin(BT_CHARGE_PIN)

#    define USB_SUSPEND_CHECK_ENABLE
#endif

/* RGB Matrix */

/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5
#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12

/* I2C Config for LED Driver */
#define SNLED27351_I2C_ADDRESS_1 SNLED27351_I2C_ADDRESS_GND
#define SNLED27351_I2C_ADDRESS_2 SNLED27351_I2C_ADDRESS_VDDIO
#define I2C1_OPMODE OPMODE_I2C
#define I2C1_CLOCK_SPEED 400000

/* Set LED driver current */
#define SNLED27351_CURRENT_TUNE {0x50, 0xFF, 0xFF, 0x50, 0xFF, 0xFF, 0x50, 0xFF, 0xFF, 0x50, 0xFF, 0xFF}

#define NUM_LOCK_IND_INDEX 84
#define CAPS_LOCK_IND_INDEX 85
#define SCROLL_LOCK_IND_INDEX 86
