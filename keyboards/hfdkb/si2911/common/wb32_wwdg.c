// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#include <stdlib.h>
#include "wwdg.h"

static volatile bool is_initialised = false;

volatile uint8_t wb32_wwdg_started(void) {
    return is_initialised;
}

void wb32_wwdg_start(void) {
    if (!is_initialised) {
        is_initialised = true;
        rccEnableWWDG();
        rccResetWWDG();

        WWDG_SetPrescaler(WWDG_Prescaler_8);
        WWDG_SetWindowValue(0x7F); // maximum
        WWDG_Enable(127);
    }
}

void wb32_wwdg_stop(void) {
    if (is_initialised) {
        is_initialised = false;
        rccResetWWDG();
        rccDisableWWDG();
    }
}
