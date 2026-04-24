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
#    define BT_HOST1_NAME "Bastet R BT$"
#    define BT_HOST2_NAME "Bastet R BT$"
#    define BT_HOST3_NAME "Bastet R BT$"

#    define BT_BLINK_HOST1_INDEX 26
#    define BT_BLINK_HOST2_INDEX 27
#    define BT_BLINK_HOST3_INDEX 28
#    define BT_BLINK_2G4_INDEX 29
#    define BT_BLINK_USB_INDEX 255

#endif

#define LED_PWR_INDEX 6
#define LED_KEY_H_INDEX 25
#define LED_BAT_INDEX_TAB {43, 42, 41, 40, 39}
