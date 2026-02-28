// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#ifdef BT_MODE_ENABLE
/* POWER */
#    define BT_CABLE_PIN B8
#    define BT_CHARGE_PIN B9
#    define USB_POWER_EN_PIN A14
// #    define WL_PWR_SW_PIN C13

#    define WLS_KEYBOARD_REPORT_KEYS 6

#    define MD_BT_NAME "DOS 68 BT$"

#    define PAIRING_TIMER_INTERVAL 200                 // ms
#    define RECONNECT_TIMER_INTERVAL 500               // ms
#    define PAIRING_TIMER_TIMEOUT 60000                // ms
#    define RECONNECT_TIMER_TIMEOUT 30000              // ms
#    define ENTRY_SLEEP_TIMEOUT ((5 * 60 - 40) * 1000) // ms

#    define WL_PROCESS_KEYS(keycode, pressed) bts_process_keys(keycode, pressed, dev_info.devs, keymap_config.no_gui, WLS_KEYBOARD_REPORT_KEYS)

#    define USB_SUSPEND_CHECK_ENABLE

#    define LED_HOST_BT1_INDEX 1
#    define LED_HOST_BT2_INDEX 2
#    define LED_HOST_BT3_INDEX 3
#    define LED_HOST_BT4_INDEX 255
#    define LED_HOST_BT5_INDEX 255
#    define LED_HOST_2G4_INDEX 0
#    define LED_HOST_USB_INDEX 29
#    define LED_HOST_BT1_COLOR {0, 0, 0x77}
#    define LED_HOST_BT2_COLOR {0, 0, 0x77}
#    define LED_HOST_BT3_COLOR {0, 0, 0x77}
#    define LED_HOST_BT4_COLOR {0, 0, 0}
#    define LED_HOST_BT5_COLOR {0, 0, 0}
#    define LED_HOST_2G4_COLOR {0, 0x77, 0}
#    define LED_HOST_USB_COLOR {0x77, 0x77, 0x77}
#endif

/* RGB Matrix */
#define LED_POWER_EN_PIN B7

/* RGB Indicator index */
#define LED_CAPS_LOCK_INDEX 30
#define LED_GUI_LOCK_INDEX 59
#define LED_LEFT_CTRL_INDEX 58

/* FLASH */
/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5
#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12
