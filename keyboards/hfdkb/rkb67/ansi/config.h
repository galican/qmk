// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#define BT1_LED_INDEX 13
#define BT2_LED_INDEX 12
#define BT3_LED_INDEX 11
#define WL_LED_INDEX 14
#define USB_LED_INDEX 15

/* Enable CapsLock LED */
#define CAPS_LOCK_LED_INDEX 43

/* Enable GUI Lock LED */
#define GUI_LOCK_LED_INDEX 65

/* SLED define */
#define SLED_START_INDEX 67
#define SLED_END_INDEX 72

#define BAT_VOL_LED_INDEX {67, 68, 69, 70, 71, 72}

#define KEY_A_LED_INDEX 41
#define KEY_S_LED_INDEX 42
#define KEY_LCTRL_LED_INDEX 66
