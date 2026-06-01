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

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#ifdef BT_MODE_ENABLE
#    define BT_CABLE_PIN B8
#    define BT_CHARGE_PIN B9
#    define BT_MODE_SW_PIN C10
#    define RF_MODE_SW_PIN D2
#    define RGB_DRIVER_SDB_PIN A15

#    define USB_SUSPEND_STATE_CHECK

#    define BT1_LED_INDEX 16
#    define BT2_LED_INDEX 17
#    define BT3_LED_INDEX 18
#    define BT4_LED_INDEX 19
#    define USB_LED_INDEX 20

#    define BT1_LED_COLOR {0, 0, 100}
#    define BT2_LED_COLOR {0, 0, 100}
#    define BT3_LED_COLOR {0, 0, 100}
#    define BT4_LED_COLOR {0, 100, 0}
#    define USB_LED_COLOR {100, 100, 100}
#endif

/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5
#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12

#define KEY_NUM 6

/* Enable CapsLock LED */
#define CAPS_LOCK_LED_INDEX 30
/* Enable GUI Lock LED */
#define GUI_LOCK_LED_INDEX 59

/* I2C Config for LED Driver */
#define SNLED27351_DRIVER_COUNT 2
#define SNLED27351_I2C_ADDRESS_1 SNLED27351_I2C_ADDRESS_GND
#define SNLED27351_I2C_ADDRESS_2 SNLED27351_I2C_ADDRESS_VDDIO

#define I2C1_OPMODE OPMODE_I2C
#define I2C1_CLOCK_SPEED 400000
#define SNLED27351_PHASE_CHANNEL SNLED27351_SCAN_PHASE_9_CHANNEL

/* Enable GUI Lock LED */
// #define GUI_LOCK_LED_INDEX 65

// #define BAT_VOL_LED_INDEX {67, 68, 69, 70, 71, 72}
