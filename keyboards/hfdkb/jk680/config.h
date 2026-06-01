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
#    define BT_MODE_SW_PIN C13
#    define RF_MODE_SW_PIN C14
#    define RGB_DRIVER_SDB_PIN B7

#    define USB_SUSPEND_STATE_CHECK

#    define BT1_LED_INDEX 56
#    define BT2_LED_INDEX 55
#    define BT3_LED_INDEX 54
#    define BT4_LED_INDEX 53
#    define USB_LED_INDEX 52
#    define PWR_LED_INDEX 63

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
// #define CAPS_LOCK_LED_INDEX 30

/* Enable GUI Lock LED */
// #define GUI_LOCK_LED_INDEX 59

#define CAPS_LOCK_LED_PIN B12
#define GUI_LOCK_LED_PIN B13
#define ALL_KEY_LOCK_PIN A8
#define NUM_2_FN_PIN A8

// #define BAT_VOL_LED_INDEX {67, 68, 69, 70, 71, 72}

#define SLED_START_INDEX 67
#define SLED_END_INDEX 78
#define ALED_START_INDEX 79
#define ALED_END_INDEX 84
