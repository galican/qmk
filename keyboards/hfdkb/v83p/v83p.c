// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"
#include <stdlib.h>
// clang-format off

#ifdef RGB_MATRIX_ENABLE
const is31fl3733_led_t PROGMEM g_is31fl3733_leds[RGB_MATRIX_LED_COUNT] = {
/* Refer to IS31 manual for these locations
 *   driver
 *   |   R location
 *   |   |     G location
 *   |   |     |     B location
 *   |   |     |     | */
    {1, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {1, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {1, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {1, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {1, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {1, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {1, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {1, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {1, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {1, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {1, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {1, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {1, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {1, SW1_CS14,  SW2_CS14,  SW3_CS14},

    {0, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {0, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {0, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {0, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {0, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {0, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {0, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {0, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {0, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {0, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {0, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {0, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {0, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {0, SW1_CS14,  SW2_CS14,  SW3_CS14},
    {1, SW4_CS1,   SW5_CS1,   SW6_CS1},

    {0, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {0, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {0, SW4_CS3,   SW5_CS3,   SW6_CS3},
    {0, SW4_CS4,   SW5_CS4,   SW6_CS4},
    {0, SW4_CS5,   SW5_CS5,   SW6_CS5},
    {0, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {0, SW4_CS7,   SW5_CS7,   SW6_CS7},
    {0, SW4_CS8,   SW5_CS8,   SW6_CS8},
    {0, SW4_CS9,   SW5_CS9,   SW6_CS9},
    {0, SW4_CS10,  SW5_CS10,  SW6_CS10},
    {0, SW4_CS11,  SW5_CS11,  SW6_CS11},
    {0, SW4_CS12,  SW5_CS12,  SW6_CS12},
    {0, SW4_CS13,  SW5_CS13,  SW6_CS13},
    {0, SW4_CS14,  SW5_CS14,  SW6_CS14},
    {1, SW4_CS2,   SW5_CS2,   SW6_CS2},

    {0, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {0, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {0, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {0, SW7_CS4,   SW8_CS4,   SW9_CS4},
    {0, SW7_CS5,   SW8_CS5,   SW9_CS5},
    {0, SW7_CS6,   SW8_CS6,   SW9_CS6},
    {0, SW7_CS7,   SW8_CS7,   SW9_CS7},
    {0, SW7_CS8,   SW8_CS8,   SW9_CS8},
    {0, SW7_CS9,   SW8_CS9,   SW9_CS9},
    {0, SW7_CS10,  SW8_CS10,  SW9_CS10},
    {0, SW7_CS11,  SW8_CS11,  SW9_CS11},
    {0, SW7_CS12,  SW8_CS12,  SW9_CS12},
    {0, SW7_CS13,  SW8_CS13,  SW9_CS13},
    {1, SW4_CS3,   SW5_CS3,   SW6_CS3},

    {0, SW10_CS1,   SW11_CS1,   SW12_CS1},
    {0, SW10_CS2,   SW11_CS2,   SW12_CS2},
    {0, SW10_CS3,   SW11_CS3,   SW12_CS3},
    {0, SW10_CS4,   SW11_CS4,   SW12_CS4},
    {0, SW10_CS5,   SW11_CS5,   SW12_CS5},
    {0, SW10_CS6,   SW11_CS6,   SW12_CS6},
    {0, SW10_CS7,   SW11_CS7,   SW12_CS7},
    {0, SW10_CS8,   SW11_CS8,   SW12_CS8},
    {0, SW10_CS9,   SW11_CS9,   SW12_CS9},
    {0, SW10_CS10,  SW11_CS10,  SW12_CS10},
    {0, SW10_CS11,  SW11_CS11,  SW12_CS11},
    {0, SW10_CS12,  SW11_CS12,  SW12_CS12},
    {1, SW4_CS7,   SW5_CS7,   SW6_CS7},
    {1, SW4_CS4,   SW5_CS4,   SW6_CS4},

    {0, SW10_CS13,  SW11_CS13,  SW12_CS13},
    {0, SW10_CS14,  SW11_CS14,  SW12_CS14},
    {0, SW10_CS15,  SW11_CS15,  SW12_CS15},
    {0, SW10_CS16,  SW11_CS16,  SW12_CS16},

    {0, SW7_CS14,  SW8_CS14,  SW9_CS14},
    {0, SW7_CS15,  SW8_CS15,  SW9_CS15},
    {0, SW7_CS16,  SW8_CS16,  SW9_CS16},

    {0, SW4_CS15,  SW5_CS15,  SW6_CS15},
    {1, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {1, SW4_CS5,   SW5_CS5,   SW6_CS5},

    {1, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {1, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {1, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {1, SW7_CS4,   SW8_CS4,   SW9_CS4},
    {1, SW7_CS5,   SW8_CS5,   SW9_CS5},

    {1, SW10_CS1,   SW11_CS1,   SW12_CS1},
    {1, SW10_CS2,   SW11_CS2,   SW12_CS2},
    {1, SW10_CS3,   SW11_CS3,   SW12_CS3},
    {1, SW10_CS4,   SW11_CS4,   SW12_CS4},
    {1, SW10_CS5,   SW11_CS5,   SW12_CS5},
};
#endif
// clang-format on

// void led_init_ports(void) {
//     // Set our LED pins as output
//     // setPinOutput(D2); // Num Lock
//     // writePinLow(D2);
//     // setPinOutput(C11); // Scroll Lock
//     // writePinLow(C11);
//     setPinOutput(C10); // Caps Lock
//     writePinLow(C10);
// }

// bool led_update_kb(led_t led_state) {
//     bool res = led_update_user(led_state);
//     if(res) {
//         // writePin(D2, led_state.num_lock);
//         writePin(C10, led_state.caps_lock);
//         // writePin(C11, led_state.scroll_lock);
//     }
//     return res;
// }

bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
        // Set our LED pins as output
        gpio_set_pin_output(A14);
        if (dev_info.devs == DEVS_USB) {
            gpio_write_pin_low(A14);
        } else {
            gpio_write_pin_high(A14);
        }
        gpio_set_pin_output(RGB_DRIVER_SDB_PIN);
        gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        // Set our LED pins as input
        gpio_write_pin_low(RGB_DRIVER_SDB_PIN);
        // setPinInput(A14);
        // writePinLow(A14);
        led_inited = false;
    }
}
// 拨动开关选择系统模式
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }
    if (index == 0) {
        default_layer_set(1UL << (active ? 2 : 0));
    }
    if (active) {
        keymap_config.no_gui = 0;
        eeconfig_update_keymap(&keymap_config);
    }
    return true;
}

void blink_led_set(uint8_t index, uint8_t next_index);
typedef struct {
    uint32_t interval;
    uint32_t remain_time;
    uint32_t total_time;
    uint32_t time;
    uint8_t  index;
    uint8_t  flip;
    uint8_t  next_index;
} blink_led_t;
#define NUM_BLINK_LEDS (sizeof(blink_leds) / sizeof(blink_led_t))
blink_led_t blink_leds[] = {
    {.index = 0xFF, .interval = 200, .total_time = 1200, .time = 0}, // ALL
};

void blink_led_set(uint8_t index, uint8_t next_index) {
    if (index < NUM_BLINK_LEDS) {
        blink_leds[index].remain_time = blink_leds[index].total_time;
        blink_leds[index].next_index  = next_index;
    }
}

bool blink_led_advanced(void) {
    for (uint8_t led_index = 0; led_index < NUM_BLINK_LEDS; led_index++) {
        if (blink_leds[led_index].remain_time != 0) {
            if ((blink_leds[led_index].time == 0) || (timer_elapsed32(blink_leds[led_index].time) >= blink_leds[led_index].interval)) {
                if (!blink_leds[led_index].flip) {
                    switch (blink_leds[led_index].index) {
                        case 1: {
                            // writePin(D2, false);
                        } break;
                        case 2: {
                            // writePin(C10, false);
                        } break;
                        case 3: {
                            // writePin(C11, false);
                        } break;
                        case 0xFF: {
                            // writePin(D2, false);
                            // writePin(C10, false);
                            // writePin(C11, false);
                        } break;
                        default:
                            break;
                    }
                } else {
                    switch (blink_leds[led_index].index) {
                        case 1: {
                            // writePin(D2, true);
                        } break;
                        case 2: {
                            // writePin(C10, true);
                        } break;
                        case 3: {
                            // writePin(C11, true);
                        } break;
                        case 0xFF: {
                            // writePin(D2, true);
                            // writePin(C10, true);
                            // writePin(C11, true);
                        } break;
                        default:
                            break;
                    }
                }
                blink_leds[led_index].flip = !blink_leds[led_index].flip;
                blink_leds[led_index].time = timer_read32();

                if (blink_leds[led_index].remain_time >= blink_leds[led_index].interval)
                    blink_leds[led_index].remain_time -= blink_leds[led_index].interval;
                else
                    blink_leds[led_index].remain_time = 0;

                if (blink_leds[led_index].remain_time == 0) {
                    if (blink_leds[led_index].next_index > 0) {
                        blink_led_set(blink_leds[led_index].next_index - 1, 0);
                    }

                    led_update_kb(host_keyboard_led_state());
                }
            }

        } else {
            blink_leds[led_index].flip = false;
            blink_leds[led_index].time = 0;
        }
    }

    return true;
}

void set_led_state(void) {
    if (led_inited) {
        if (blink_leds[0].remain_time == 0) {
            // writePin(D2, keymap_config.no_gui);
            // writePin(C11, get_highest_layer(default_layer_state) == 2);
        }
    }
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (process_record_user(keycode, record) != true) {
        return false;
    }
    switch (keycode) {
        case RM_VALU: {
            if (record->event.pressed && (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS)) {
                blink_led_set(0, 0);
                dprintf("brightness out in\r\n");
            }
        } break;
        case RM_VALD: {
            if (record->event.pressed && (rgb_matrix_get_val() == 0x00)) {
                blink_led_set(0, 0);
                dprintf("brightness out in\r\n");
            }
        } break;
        case KC_END: {
            if (record->event.pressed) {
                extern uint8_t rgb_test_en;
                if (rgb_test_en) {
                    rgb_test_en = false;
                    return false;
                }
            }
        } break;
        case KC_DOWN: {
            if (record->event.pressed) {
                extern uint8_t rgb_test_en;
                extern uint8_t rgb_test_index;
                if (rgb_test_en) {
                    rgb_test_index++;
                    if (rgb_test_index > 4) rgb_test_index = 1;
                    return false;
                }
            }
        } break;
    }
#ifdef BT_MODE_ENABLE
    if (process_record_bt(keycode, record) != true) {
        return false;
    }
#endif
    return true;
}

void lp_recovery_hook(void) {
    // extern void open_rgb(void);

    // bt_switch_mode(DEVS_USB, dev_info.last_devs, false);
    // open_rgb();
}

void matrix_init_kb(void) {
#ifdef RGB_DRIVER_SDB_PIN
    gpio_set_pin_output_open_drain(RGB_DRIVER_SDB_PIN);
    gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#endif

#ifdef BT_MODE_ENABLE
    bt_init();
    led_config_all();
    blink_led_advanced();
#endif
    matrix_init_user();
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
    set_led_state();
#endif
    matrix_scan_user();
}

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif
}

#ifdef RGB_MATRIX_ENABLE
typedef struct {
    uint32_t interval;
    uint32_t remain_time;
    uint32_t total_time;
    uint32_t time;
    uint8_t  index;
    uint8_t  flip;
    uint8_t  next_index;
    RGB      color;
    RGB      has_been_color;
} blink_rgb_t;
blink_rgb_t blink_rgbs[] = {
    {.index = 46, .interval = 250, .total_time = 1000, .time = 0, .color = {.r = 0xFF, .g = 0xFF, .b = 0xFF}}, // A
    {.index = 47, .interval = 250, .total_time = 1000, .time = 0, .color = {.r = 0xFF, .g = 0xFF, .b = 0xFF}}, // S
    {.index = 28, .interval = 250, .total_time = 1000, .time = 0, .color = {.r = 0xFF, .g = 0xFF, .b = 0xFF}}, // Backspace
    {.index = 0, .interval = 250, .total_time = 1000, .time = 0, .color = {.r = 0xFF, .g = 0x00, .b = 0x00}},  // ESC
};

uint8_t rgb_test_en    = false;
uint8_t rgb_test_index = 0;

static const uint8_t rgb_test_color_table[][3] = {
    {RGB_WHITE},
    {RGB_RED},
    {RGB_GREEN},
    {RGB_BLUE},
};

#    define NUM_BLINK_RGBS (sizeof(blink_rgbs) / sizeof(blink_rgb_t))

void blink_rgb_set(uint8_t index, uint8_t next_index) {
    if (index < NUM_BLINK_RGBS) {
        blink_rgbs[index].remain_time = blink_rgbs[index].total_time;
        blink_rgbs[index].next_index  = next_index;
    }
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }
#    ifdef BT_MODE_ENABLE

#    endif
    if ((rgb_test_en) && (rgb_test_index > 0)) {
        // clang-format off
        for (uint8_t i = led_min; i < led_max; i++) {
            rgb_matrix_set_color(i, rgb_test_color_table[rgb_test_index - 1][0],
            rgb_test_color_table[rgb_test_index - 1][1],
            rgb_test_color_table[rgb_test_index - 1][2]);
        }
        // clang-format on
        return false;
    }

#    ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#    endif

    for (uint8_t rgb_index = 0; rgb_index < NUM_BLINK_RGBS; rgb_index++) {
        if (blink_rgbs[rgb_index].remain_time != 0) {
            RGB rgb;

            rgb = blink_rgbs[rgb_index].has_been_color;

            if ((blink_rgbs[rgb_index].time == 0) || (timer_elapsed32(blink_rgbs[rgb_index].time) >= blink_rgbs[rgb_index].interval)) {
                if (!blink_rgbs[rgb_index].flip) {
                    rgb                                  = (RGB){.r = blink_rgbs[rgb_index].color.r, .g = blink_rgbs[rgb_index].color.g, .b = blink_rgbs[rgb_index].color.b};
                    blink_rgbs[rgb_index].has_been_color = rgb;
                } else {
                    rgb                                  = (RGB){.r = 0, .g = 0, .b = 0};
                    blink_rgbs[rgb_index].has_been_color = rgb;
                }
                blink_rgbs[rgb_index].flip = !blink_rgbs[rgb_index].flip;
                blink_rgbs[rgb_index].time = timer_read32();

                if (blink_rgbs[rgb_index].remain_time >= blink_rgbs[rgb_index].interval)
                    blink_rgbs[rgb_index].remain_time -= blink_rgbs[rgb_index].interval;
                else
                    blink_rgbs[rgb_index].remain_time = 0;

                if (blink_rgbs[rgb_index].remain_time == 0) {
                    if (blink_rgbs[rgb_index].next_index > 0) {
                        blink_rgb_set(blink_rgbs[rgb_index].next_index - 1, 0);
                    }

                    // rgb test
                    if (blink_rgbs[rgb_index].index == 16) {
                        if (rgb_test_en != true) rgb_test_en = true;
                    }
                }
            }

            rgb_matrix_set_color(blink_rgbs[rgb_index].index, rgb.r, rgb.g, rgb.b);
        } else {
            blink_rgbs[rgb_index].flip = false;
            blink_rgbs[rgb_index].time = 0;
        }
    }
    // caps lock red
    if ((host_keyboard_led_state().caps_lock) && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        RGB_MATRIX_INDICATOR_SET_COLOR(44, 255, 0, 0);
    }
    // GUI lock red
    if (keymap_config.no_gui) {
        RGB_MATRIX_INDICATOR_SET_COLOR(73, 255, 0, 0);
    }

    return true;
}
#endif
