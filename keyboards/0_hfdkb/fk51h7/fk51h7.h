// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"
#include "common/bt_task.h"

enum bt_keycodes {
    BT_HOST1 = QK_KB_0,
    BT_HOST2,
    BT_HOST3,
    BT_2_4G,
    BT_USB,
    BT_VOL,
    // KC_LOCK,
    KC_W2UP,
    LOGO_LED_MODE,
};

enum my_keycodes {
    RGB_TEST = SAFE_RANGE,
};

enum lled_effects {
    LOGO_EFFECT_DEFAULT = 0,
    LOGO_EFFECT_RED,
    LOGO_EFFECT_GREEN,
    LOGO_EFFECT_BLUE,
    LOGO_EFFECT_YELLOW,
    LOGO_EFFECT_WHITE,
    LOGO_EFFECT_PURPLE,
    LOGO_EFFECT_CYAN,
    LOGO_EFFECT_MAX,
};
