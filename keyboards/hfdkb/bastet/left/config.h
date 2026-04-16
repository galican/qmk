// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

/* RGB Matrix */

/* SPI Config for spi flash*/

/* Bootmagic Lite key configuration */

#ifdef BT_MODE_ENABLE
#    define BT_HOST1_NAME "Bastet L BT$"
#    define BT_HOST2_NAME "Bastet L BT$"
#    define BT_HOST3_NAME "Bastet L BT$"

#    define BT_BLINK_HOST1_INDEX 25
#    define BT_BLINK_HOST2_INDEX 26
#    define BT_BLINK_HOST3_INDEX 27
#    define BT_BLINK_2G4_INDEX 28
#    define BT_BLINK_USB_INDEX 255
#endif

#define LED_PWR_INDEX 0
#define LED_KEY_G_INDEX 14
#define LED_KEY_CAPS_INDEX 19
#define LED_KEY_LGUI_INDEX 2
#define LED_BAT_INDEX_TAB {35, 34, 33, 32, 31}
