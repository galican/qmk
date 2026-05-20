/* Copyright (C) 2023 Westberry Technology (ChangZhou) Corp., Ltd
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

#include QMK_KEYBOARD_H

#include "quantum.h"
#include "uart.h"
#include "report.h"
#include "usb_main.h"
#include "command.h"
#include "common/bt_task.h"
#include "lib/lib8tion/lib8tion.h"

#define NUM_LONG_PRESS_KEYS (sizeof(long_pressed_keys) / sizeof(long_pressed_keys_t))

#ifdef BT_DEBUG_MODE
#    define BT_DEBUG_INFO(fmt, ...) dprintf(fmt, ##__VA_ARGS__)
#else
#    define BT_DEBUG_INFO(fmt, ...)
#endif

typedef struct {
    uint32_t press_time;
    uint16_t keycode;
    void (*event_cb)(uint16_t);
} long_pressed_keys_t;

uint32_t bt_init_time = 0;

dev_info_t dev_info = {0};
bts_info_t bts_info = {
    .bt_name        = {"JK680-$", "JK680-$", "JK680-$"},
    .uart_init      = uart_init,
    .uart_read      = uart_read,
    .uart_transmit  = uart_transmit,
    .uart_receive   = uart_receive,
    .uart_available = uart_available,
    .timer_read32   = timer_read32,
};

static void long_pressed_keys_hook(void);
static void long_pressed_keys_cb(uint16_t keycode);
static bool process_record_other(uint16_t keycode, keyrecord_t *record);
static void bt_scan_mode(void);
static void bt_used_pin_init(void);

#ifdef RGB_MATRIX_ENABLE
static void open_rgb(void);
static void close_rgb(void);
#endif

// blink led set
static uint8_t  all_blink_cnt      = 0;
static uint32_t all_blink_time     = 0;
static RGB      all_blink_color    = {0};
static uint8_t  single_blink_cnt   = 0;
static uint8_t  single_blink_index = 0;
static RGB      single_blink_color = {0};
static uint32_t single_blink_time  = 0;

static uint32_t USB_switch_time = 0;
static uint8_t  USB_blink_cnt   = 0;

static uint32_t key_press_time = 0;
static uint32_t close_rgb_time = 0;

static bool bak_rgb_toggle = false;
static bool sober          = true;

bool kb_sleep_flag = false;

static const uint8_t rgb_index_table[]          = {USB_LED_INDEX, BT1_LED_INDEX, BT2_LED_INDEX, BT3_LED_INDEX, BT4_LED_INDEX};
static const uint8_t rgb_index_color_table[][3] = {
    USB_LED_COLOR, BT1_LED_COLOR, BT2_LED_COLOR, BT3_LED_COLOR, BT4_LED_COLOR,
};

static uint32_t last_total_time           = 0;
static uint8_t  indicator_status          = 2;
static uint8_t  indicator_reset_last_time = false;

static uint32_t EE_CLR_press_cnt  = 0;
static uint32_t EE_CLR_press_time = 0;
static bool     EE_CLR_flag       = false;

static bool query_vol_flag = false;

// static bool    rgb_test_en    = false;
// static uint8_t rgb_test_index = 0;

// static const uint8_t rgb_test_color_table[][3] = {
//     {RGB_WHITE},
//     {RGB_RED},
//     {RGB_GREEN},
//     {RGB_BLUE},
// };

// clang-format off
long_pressed_keys_t long_pressed_keys[] = {
  {.keycode = BT_HOST1, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = BT_HOST2, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = BT_HOST3, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = BT_2_4G, .press_time = 0, .event_cb = long_pressed_keys_cb},
//   {.keycode = OS_SW, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = EE_CLR, .press_time = 0, .event_cb = long_pressed_keys_cb},
};
// clang-format on

bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
#ifdef CAPS_LOCK_LED_PIN
        gpio_set_pin_output_push_pull(CAPS_LOCK_LED_PIN);
#endif

#ifdef GUI_LOCK_LED_PIN
        gpio_set_pin_output_push_pull(GUI_LOCK_LED_PIN);
#endif

#ifdef ALL_KEY_LOCK_PIN
        gpio_set_pin_output_push_pull(ALL_KEY_LOCK_PIN);
#endif
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
#ifdef CAPS_LOCK_LED_PIN
        gpio_write_pin_low(CAPS_LOCK_LED_PIN);
#endif

#ifdef GUI_LOCK_LED_PIN
        gpio_write_pin_low(GUI_LOCK_LED_PIN);
#endif

#ifdef ALL_KEY_LOCK_PIN
        gpio_write_pin_low(ALL_KEY_LOCK_PIN);
#endif
        led_inited = false;
    }
}

extern void register_mouse(uint8_t mouse_keycode, bool pressed);
/** \brief Utilities for actions. (FIXME: Needs better description)
 *
 * FIXME: Needs documentation.
 */
__attribute__((weak)) void register_code(uint8_t code) {
    if (dev_info.devs) {
        bts_process_keys(code, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {
            wait_ms(1);
        }
    } else {
        if (code == KC_NO) {
            return;

#ifdef LOCKING_SUPPORT_ENABLE
        } else if (KC_LOCKING_CAPS_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            // Resync: ignore if caps lock already is on
            if (host_keyboard_leds() & (1 << USB_LED_CAPS_LOCK)) return;
#    endif
            add_key(KC_CAPS_LOCK);
            send_keyboard_report();
            wait_ms(TAP_HOLD_CAPS_DELAY);
            del_key(KC_CAPS_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_NUM_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (host_keyboard_leds() & (1 << USB_LED_NUM_LOCK)) return;
#    endif
            add_key(KC_NUM_LOCK);
            send_keyboard_report();
            wait_ms(100);
            del_key(KC_NUM_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_SCROLL_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (host_keyboard_leds() & (1 << USB_LED_SCROLL_LOCK)) return;
#    endif
            add_key(KC_SCROLL_LOCK);
            send_keyboard_report();
            wait_ms(100);
            del_key(KC_SCROLL_LOCK);
            send_keyboard_report();
#endif

        } else if (IS_BASIC_KEYCODE(code)) {
            // TODO: should push command_proc out of this block?
            if (command_proc(code)) return;

            // Force a new key press if the key is already pressed
            // without this, keys with the same keycode, but different
            // modifiers will be reported incorrectly, see issue #1708
            if (is_key_pressed(code)) {
                del_key(code);
                send_keyboard_report();
            }
            add_key(code);
            send_keyboard_report();
        } else if (IS_MODIFIER_KEYCODE(code)) {
            add_mods(MOD_BIT(code));
            send_keyboard_report();

#ifdef EXTRAKEY_ENABLE
        } else if (IS_SYSTEM_KEYCODE(code)) {
            host_system_send(KEYCODE2SYSTEM(code));
        } else if (IS_CONSUMER_KEYCODE(code)) {
            host_consumer_send(KEYCODE2CONSUMER(code));
#endif

        } else if (IS_MOUSE_KEYCODE(code)) {
            register_mouse(code, true);
        }
    }
}

/** \brief Utilities for actions. (FIXME: Needs better description)
 *
 * FIXME: Needs documentation.
 */
__attribute__((weak)) void unregister_code(uint8_t code) {
    if (dev_info.devs) {
        bts_process_keys(code, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {
            wait_ms(1);
        }
    } else {
        if (code == KC_NO) {
            return;

#ifdef LOCKING_SUPPORT_ENABLE
        } else if (KC_LOCKING_CAPS_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            // Resync: ignore if caps lock already is off
            if (!(host_keyboard_leds() & (1 << USB_LED_CAPS_LOCK))) return;
#    endif
            add_key(KC_CAPS_LOCK);
            send_keyboard_report();
            del_key(KC_CAPS_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_NUM_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (!(host_keyboard_leds() & (1 << USB_LED_NUM_LOCK))) return;
#    endif
            add_key(KC_NUM_LOCK);
            send_keyboard_report();
            del_key(KC_NUM_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_SCROLL_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (!(host_keyboard_leds() & (1 << USB_LED_SCROLL_LOCK))) return;
#    endif
            add_key(KC_SCROLL_LOCK);
            send_keyboard_report();
            del_key(KC_SCROLL_LOCK);
            send_keyboard_report();
#endif

        } else if (IS_BASIC_KEYCODE(code)) {
            del_key(code);
            send_keyboard_report();
        } else if (IS_MODIFIER_KEYCODE(code)) {
            del_mods(MOD_BIT(code));
            send_keyboard_report();

#ifdef EXTRAKEY_ENABLE
        } else if (IS_SYSTEM_KEYCODE(code)) {
            host_system_send(0);
        } else if (IS_CONSUMER_KEYCODE(code)) {
            host_consumer_send(0);
#endif

        } else if (IS_MOUSE_KEYCODE(code)) {
            register_mouse(code, false);
        }
    }
}

extern void do_code16(uint16_t code, void (*f)(uint8_t));

__attribute__((weak)) void register_code16(uint16_t code) {
    if (dev_info.devs) {
        if (QK_MODS_GET_MODS(code) & 0x1) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RCTL, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LCTL, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x2) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RSFT, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LSFT, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x4) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RALT, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LALT, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x8) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RGUI, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LGUI, 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        bts_process_keys(QK_MODS_GET_BASIC_KEYCODE(code), 1, dev_info.devs, keymap_config.no_gui, KEY_NUM);
    } else {
        if (IS_MODIFIER_KEYCODE(code) || code == KC_NO) {
            do_code16(code, register_mods);
        } else {
            do_code16(code, register_weak_mods);
        }
        register_code(code);
    }
}

__attribute__((weak)) void unregister_code16(uint16_t code) {
    if (dev_info.devs) {
        if (QK_MODS_GET_MODS(code) & 0x1) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RCTL, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LCTL, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x2) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RSFT, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LSFT, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x4) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RALT, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LALT, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        if (QK_MODS_GET_MODS(code) & 0x8) {
            if (QK_MODS_GET_MODS(code) & 0x10)
                bts_process_keys(KC_RGUI, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            else
                bts_process_keys(KC_LGUI, 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
        }
        bts_process_keys(QK_MODS_GET_BASIC_KEYCODE(code), 0, dev_info.devs, keymap_config.no_gui, KEY_NUM);
    } else {
        unregister_code(code);
        if (IS_MODIFIER_KEYCODE(code) || code == KC_NO) {
            do_code16(code, unregister_mods);
        } else {
            do_code16(code, unregister_weak_mods);
        }
    }
}

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        bts_task(dev_info.devs);
        chThdSleepMilliseconds(1);
    }
}

/**
 * @brief bluetooth 初始化函数
 * @param None
 * @return None
 */
void bt_init(void) {
    bts_init(&bts_info);
    bt_used_pin_init();

    // Read the user config from EEPROM
    dev_info.raw = eeconfig_read_user();
    if (!dev_info.raw) {
        dev_info.devs      = DEVS_USB;
        dev_info.last_devs = DEVS_HOST1;
        eeconfig_update_user(dev_info.raw);
    }

    chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);

    bts_send_name(DEVS_HOST1);
    wait_ms(10);
    bt_scan_mode();

    if (dev_info.devs != DEVS_USB) {
        usbDisconnectBus(&USB_DRIVER);
        usbStop(&USB_DRIVER);
    }

    gpio_set_pin_output(A14);
    if (dev_info.devs == DEVS_USB) {
        gpio_write_pin_low(A14);
    } else {
        gpio_write_pin_high(A14);
    }

    bt_init_time = timer_read32();
}

/**
 * @brief bluetooth交互任务
 * @param event 当前事件
 * @return None
 */
void bt_task(void) {
    static uint32_t last_time = 0;

    if ((bt_init_time != 0) && (timer_elapsed32(bt_init_time) >= 2000)) {
        bt_init_time = 0;

        // bts_send_vendor(v_en_sleep_wl);
        bts_send_vendor(v_en_sleep_bt);

        switch (dev_info.devs) {
            case DEVS_HOST1: {
                bts_send_vendor(v_host1);
            } break;
            case DEVS_HOST2: {
                bts_send_vendor(v_host2);
            } break;
            case DEVS_HOST3: {
                bts_send_vendor(v_host3);
            } break;
            case DEVS_2_4G: {
                bts_send_vendor(v_2_4g);
            } break;
            default: {
                bts_send_vendor(v_usb);
                dev_info.devs = DEVS_USB;
                eeconfig_update_user(dev_info.raw);
            } break;
        }
    }

    /* Execute every 1ms */
    if (timer_elapsed32(last_time) >= 1) {
        last_time = timer_read32();

        if (dev_info.devs != DEVS_USB) {
            uint8_t keyboard_led_state = 0;
            led_t  *kb_leds            = (led_t *)&keyboard_led_state;
            kb_leds->raw               = bts_info.bt_info.indictor_rgb_s;
            usb_device_state_set_leds(keyboard_led_state);

#ifdef RGB_MATRIX_ENABLE
            close_rgb();
#endif
        }
    }

    long_pressed_keys_hook();
    bt_scan_mode();
}

uint32_t pressed_time = 0;

bool process_record_bt(uint16_t keycode, keyrecord_t *record) {
    bool retval = true;
    // clang-format off
    if (record->event.pressed) {
        BT_DEBUG_INFO("\n\nkeycode = [0x%x], pressed time: [%d]\n\n", keycode, record->event.time);
        BT_DEBUG_INFO("\n devs     = [%d] \
                    \n sleeped       = [%d] \
                    \n low_vol       = [%d] \
                    \n low_vol_offed = [%d] \
                    \n normal_vol    = [%d] \
                    \n pairing       = [%d] \
                    \n paired        = [%d] \
                    \n come_back     = [%d] \
                    \n come_back_err = [%d] \
                    \n mode_switched = [%d] \
                    \n pvol          = [%d]\n\n\n",
                    dev_info.devs,
                    bts_info.bt_info.sleeped,
                    bts_info.bt_info.low_vol,
                    bts_info.bt_info.low_vol_offed,
                    bts_info.bt_info.normal_vol,
                    bts_info.bt_info.pairing,
                    bts_info.bt_info.paired,
                    bts_info.bt_info.come_back,
                    bts_info.bt_info.come_back_err,
                    bts_info.bt_info.mode_switched,
                    bts_info.bt_info.pvol);
        // clang-format on
        pressed_time = timer_read32();

        if (indicator_status != 0) {
            last_total_time = timer_read32();
        }
    }
    retval = process_record_other(keycode, record);

    if (dev_info.devs != DEVS_USB) {
        if (retval != false) {
            while (bts_is_busy()) {
                wait_ms(1);
            }

            if ((keycode > QK_MODS) && (keycode <= QK_MODS_MAX)) {
                if (QK_MODS_GET_MODS(keycode) & 0x1) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                    else
                        bts_process_keys(KC_LCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x2) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                    else
                        bts_process_keys(KC_LSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x4) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RALT, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                    else
                        bts_process_keys(KC_LALT, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x8) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                    else
                        bts_process_keys(KC_LGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
                }
                retval = bts_process_keys(QK_MODS_GET_BASIC_KEYCODE(keycode), record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            } else if (IS_BASIC_KEYCODE(keycode)) {
                if (record->event.pressed) {
                    register_code(keycode);
                } else {
                    unregister_code(keycode);
                }
            } else {
                retval = bts_process_keys(keycode, record->event.pressed, dev_info.devs, keymap_config.no_gui, KEY_NUM);
            }
        }
    }

#ifdef RGB_MATRIX_ENABLE
    open_rgb();
#endif

    return retval;
}

void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    bool usb_sws = !!last_mode ? !now_mode : !!now_mode;

    if (usb_sws) {
        if (!!now_mode) {
            usbDisconnectBus(&USB_DRIVER);
            usbStop(&USB_DRIVER);
        } else {
            init_usb_driver(&USB_DRIVER);
        }
    }

    dev_info.devs = now_mode;
    if ((dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
        dev_info.last_devs = dev_info.devs;
    } else if (dev_info.devs == DEVS_USB) {
        USB_switch_time = timer_read32();
        USB_blink_cnt   = 0;
    }

    if (dev_info.devs == DEVS_USB) {
        gpio_write_pin_low(A14);
    } else {
        gpio_write_pin_high(A14);
    }
    bts_info.bt_info.pairing       = false;
    bts_info.bt_info.paired        = false;
    bts_info.bt_info.come_back     = false;
    bts_info.bt_info.come_back_err = false;
    bts_info.bt_info.mode_switched = false;

    bts_info.bt_info.indictor_rgb_s = 0;
    eeconfig_update_user(dev_info.raw);

    switch (dev_info.devs) {
        case DEVS_HOST1: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host1);
            }
        } break;
        case DEVS_HOST2: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = 0;
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host2);
            }
        } break;
        case DEVS_HOST3: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host3);
            }
        } break;
        case DEVS_2_4G: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_2_4g);
            }
        } break;
        case DEVS_USB: {
            bts_send_vendor(v_usb);
        } break;
        default:
            break;
    }
}

static bool process_record_other(uint16_t keycode, keyrecord_t *record) {
    for (uint8_t i = 0; i < NUM_LONG_PRESS_KEYS; i++) {
        if (keycode == long_pressed_keys[i].keycode) {
            if (record->event.pressed) {
                long_pressed_keys[i].press_time = timer_read32();
            } else {
                long_pressed_keys[i].press_time = 0;
            }
            break;
        }
    }

    switch (keycode) {
        case BT_HOST1: {
            if (record->event.pressed) {
                if ((dev_info.devs != DEVS_HOST1) && !gpio_read_pin(BT_MODE_SW_PIN)) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
                }
            }
        } break;
        case BT_HOST2: {
            if (record->event.pressed) {
                if ((dev_info.devs != DEVS_HOST2) && !gpio_read_pin(BT_MODE_SW_PIN)) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST2, false);
                }
            }
        } break;
        case BT_HOST3: {
            if (record->event.pressed) {
                if ((dev_info.devs != DEVS_HOST3) && !gpio_read_pin(BT_MODE_SW_PIN)) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST3, false);
                }
            }
        } break;
        case BT_2_4G: {
            if (record->event.pressed) {
                if ((dev_info.devs != DEVS_2_4G) && !gpio_read_pin(RF_MODE_SW_PIN)) {
                    bt_switch_mode(dev_info.devs, DEVS_2_4G, false);
                }
            }
        } break;
        case BT_VOL: {
            if (record->event.pressed) {
                bts_send_vendor(v_query_vol);
                query_vol_flag = true;
            } else {
                query_vol_flag = false;
            }
        } break;

        case EE_CLR: {
        } break;

        case OS_SW: {
            if (record->event.pressed) {
                if (get_highest_layer(default_layer_state) == 0) {
                    set_single_persistent_default_layer(3);
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(&keymap_config);
                } else if (get_highest_layer(default_layer_state) == 3) {
                    set_single_persistent_default_layer(0);
                }
            }
        } break;

        default:
            return true;
    }

    return false;
}

static void long_pressed_keys_cb(uint16_t keycode) {
    switch (keycode) {
        case BT_HOST1: {
            if (dev_info.devs == DEVS_HOST1) {
                bt_switch_mode(dev_info.devs, DEVS_HOST1, true);
            }
        } break;
        case BT_HOST2: {
            if (dev_info.devs == DEVS_HOST2) {
                bt_switch_mode(dev_info.devs, DEVS_HOST2, true);
            }
        } break;
        case BT_HOST3: {
            if (dev_info.devs == DEVS_HOST3) {
                bt_switch_mode(dev_info.devs, DEVS_HOST3, true);
            }
        } break;
        case BT_2_4G: {
            if (dev_info.devs == DEVS_2_4G) {
                bt_switch_mode(dev_info.devs, DEVS_2_4G, true);
            }
        } break;

        case EE_CLR: {
            if (!EE_CLR_flag) {
                EE_CLR_flag       = true;
                EE_CLR_press_time = timer_read32();
                EE_CLR_press_cnt  = 1;
            }
        } break;

        default:
            break;
    }
}

static void long_pressed_keys_hook(void) {
    for (uint8_t i = 0; i < NUM_LONG_PRESS_KEYS; i++) {
        if ((long_pressed_keys[i].press_time != 0) && (timer_elapsed32(long_pressed_keys[i].press_time) >= (3 * 1000))) {
            long_pressed_keys[i].event_cb(long_pressed_keys[i].keycode);
            long_pressed_keys[i].press_time = 0;
        }
    }
}

static void bt_used_pin_init(void) {
#ifdef BT_MODE_SW_PIN
    gpio_set_pin_input_high(BT_MODE_SW_PIN);
    gpio_set_pin_input_high(RF_MODE_SW_PIN);
#endif

#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    gpio_set_pin_input_high(BT_CABLE_PIN);
    gpio_set_pin_input(BT_CHARGE_PIN);
#endif

#ifdef RGB_DRIVER_SDB_PIN
    gpio_set_pin_output_push_pull(RGB_DRIVER_SDB_PIN);
    gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#endif

#ifdef CAPS_LOCK_LED_PIN
    gpio_set_pin_output_push_pull(CAPS_LOCK_LED_PIN);
    gpio_write_pin_low(CAPS_LOCK_LED_PIN);
#endif

#ifdef GUI_LOCK_LED_PIN
    gpio_set_pin_output_push_pull(GUI_LOCK_LED_PIN);
    gpio_write_pin_low(GUI_LOCK_LED_PIN);
#endif

#ifdef ALL_KEY_LOCK_PIN
    gpio_set_pin_output_push_pull(ALL_KEY_LOCK_PIN);
    gpio_write_pin_low(ALL_KEY_LOCK_PIN);
#endif
}

/**
 * @brief 根据波动开关判断工作模式
 * @param None
 * @return None
 */
static void bt_scan_mode(void) {
#ifdef BT_MODE_SW_PIN
    if (!gpio_read_pin(BT_MODE_SW_PIN) && gpio_read_pin(RF_MODE_SW_PIN)) {
        if ((dev_info.devs == DEVS_USB) || (dev_info.devs == DEVS_2_4G)) {
            bt_switch_mode(dev_info.devs, dev_info.last_devs, false); // BT mode
        }
    }
    if (!gpio_read_pin(RF_MODE_SW_PIN) && gpio_read_pin(BT_MODE_SW_PIN)) {
        if (dev_info.devs != DEVS_2_4G) {
            bt_switch_mode(dev_info.devs, DEVS_2_4G, false); // 2_4G mode
        }
    }
    if (gpio_read_pin(BT_MODE_SW_PIN) && gpio_read_pin(RF_MODE_SW_PIN)) {
        if (dev_info.devs != DEVS_USB) bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
    }
#endif
}

static void close_rgb(void) {
    if (!key_press_time) {
        key_press_time = timer_read32();
        return;
    }

    if (sober) {
        if (kb_sleep_flag || (timer_elapsed32(key_press_time) >= (5 * 60 * 1000))) {
            bak_rgb_toggle = rgb_matrix_config.enable;
            sober          = false;
            close_rgb_time = timer_read32();
            rgb_matrix_disable_noeeprom();
#ifdef RGB_DRIVER_SDB_PIN
            gpio_write_pin_low(RGB_DRIVER_SDB_PIN);
#endif
        }
    } else {
        if (!rgb_matrix_config.enable) {
            if (timer_elapsed32(close_rgb_time) >= ENTRY_STOP_TIMEOUT) {
                /* Turn off all indicators led */
                if (led_inited) {
                    led_deconfig_all();
                }
#ifdef ENTRY_STOP_MODE
                lp_system_sleep();
#endif
                extern void open_rgb(void);
                open_rgb();
            }
        }
    }
}

void open_rgb(void) {
    key_press_time = timer_read32();

    if (!sober) {
#ifdef RGB_DRIVER_SDB_PIN
        gpio_write_pin_high(RGB_DRIVER_SDB_PIN);
#endif

        if (bak_rgb_toggle) {
            kb_sleep_flag = false;
            extern bool low_vol_offed_sleep;
            low_vol_offed_sleep = false;
            rgb_matrix_enable_noeeprom();
        }

        if (!led_inited) {
            led_config_all();
        }

        sober = true;
    }
}

// Device connection indication
static void indicator_device_connection(void) {
    if (dev_info.devs != DEVS_USB) {
        if ((get_highest_layer(default_layer_state | layer_state) == 1) || (get_highest_layer(default_layer_state | layer_state) == 4)) {
            rgb_matrix_set_color(rgb_index_table[dev_info.devs], rgb_index_color_table[dev_info.devs][0], rgb_index_color_table[dev_info.devs][1], rgb_index_color_table[dev_info.devs][2]);
        }
    }
}

// Factory reset indication
static void indicator_factory_reset(void) {
    if (EE_CLR_flag) {
        if (timer_elapsed32(EE_CLR_press_time) >= 300) {
            EE_CLR_press_time = timer_read32();
            EE_CLR_press_cnt++;
        }

        if (EE_CLR_press_cnt >= 7) {
            EE_CLR_press_time = 0;
            EE_CLR_press_cnt  = 0;
            EE_CLR_flag       = false;

            eeconfig_init();

            dev_info.sled_mode = 0;
            dev_info.aled_mode = 0;
            dev_info.sled_color = 0;
            dev_info.aled_color = 0;
            eeconfig_update_user(dev_info.raw);

            if (keymap_config.no_gui) {
                keymap_config.no_gui = false;
            }

            if (indicator_status != 0) {
                last_total_time = timer_read32();
            }
        }

        uint8_t brightness = (EE_CLR_press_cnt % 2) ? 100 : 0;

        rgb_matrix_set_color_all(brightness, brightness, brightness);
    }
}

// Query battery voltage every 4 seconds
static void battery_voltage_query(void) {
    static uint32_t query_vol_time = 0;
    if (gpio_read_pin(BT_CABLE_PIN)) {
        if (!bt_init_time && !kb_sleep_flag && bts_info.bt_info.paired && (timer_elapsed32(query_vol_time) > 4000)) {
            query_vol_time = timer_read32();
            bts_send_vendor(v_query_vol);
        }
    }
}

// Display battery voltage on LEDs
static void battery_voltage_display(void) {
    if (query_vol_flag && gpio_read_pin(BT_CABLE_PIN)) {
        rgb_matrix_set_color_all(0, 0, 0);

        uint8_t pvol      = bts_info.bt_info.pvol;
        uint8_t led_count = (pvol < 10) ? 1 : ((pvol / 10) > 10 ? 10 : (pvol / 10));

        for (uint8_t i = 1; i < led_count; i++) {
            rgb_matrix_set_color(i, 0, 100, 0);
        }
    }
}

// Low battery breathing effect & long press to sleep
static void battery_low_warning(void) {
    if (!gpio_read_pin(BT_CABLE_PIN)) {
        if (!gpio_read_pin(BT_CHARGE_PIN)) {
            rgb_matrix_set_color(PWR_LED_INDEX, 100, 0, 0);
        }
    } else {
        static bool     Low_power_blink = false;
        static uint32_t Low_power_time  = 0;

        if (bts_info.bt_info.low_vol) {
            if (timer_elapsed32(Low_power_time) >= 1000) {
                Low_power_blink = !Low_power_blink;
                Low_power_time  = timer_read32();
            }
            if (Low_power_blink) {
                rgb_matrix_set_color(PWR_LED_INDEX, 100, 0, 0);
            } else {
                rgb_matrix_set_color(PWR_LED_INDEX, 0, 0, 0);
            }
        } else {
            Low_power_blink = 0;
        }

        if (bts_info.bt_info.low_vol_offed) {
            if (timer_elapsed32(pressed_time) >= 2000) {
                kb_sleep_flag = true;
            }
            extern bool low_vol_offed_sleep;
            low_vol_offed_sleep = true;
        }
    }
}

// All indicator blink
static void indicator_all_blink(void) {
    if (!all_blink_cnt) return;

    rgb_matrix_set_color_all(0, 0, 0);
    if (timer_elapsed32(all_blink_time) > 300) {
        all_blink_time = timer_read32();
        all_blink_cnt--;
    }

    if (all_blink_cnt % 2) {
        rgb_matrix_set_color_all(all_blink_color.r, all_blink_color.g, all_blink_color.b);
    }
}

// Single indicator blink
static void indicator_single_blink(void) {
    if (!single_blink_cnt) return;

    if (timer_elapsed32(single_blink_time) > 300) {
        single_blink_time = timer_read32();
        single_blink_cnt--;
    }

    if (single_blink_cnt % 2) {
        rgb_matrix_set_color(single_blink_index, single_blink_color.r, single_blink_color.g, single_blink_color.b);
    } else {
        rgb_matrix_set_color(single_blink_index, 0, 0, 0);
    }
}

// Bluetooth connection indication
// static void indicator_usb_connection(void) {
//     static uint32_t USB_blink_time = 0;
//     static bool     USB_blink      = false;

//     if ((USB_DRIVER.state != USB_ACTIVE)) {
//         if (USB_blink_cnt <= 20) {
//             if (timer_elapsed32(USB_blink_time) >= 500) {
//                 USB_blink_cnt++;
//                 USB_blink      = !USB_blink;
//                 USB_blink_time = timer_read32();
//             }
//             uint8_t brightness = USB_blink ? 100 : 0;
//             rgb_matrix_set_color(rgb_index_table[DEVS_USB], brightness, brightness, brightness);
//         }
//         USB_switch_time = timer_read32();
//     } else {
//         if (timer_elapsed32(USB_switch_time) < (3 * 1000)) {
//             rgb_matrix_set_color(rgb_index_table[DEVS_USB], 100, 100, 100);
//         }
//     }
// }

// static void indicator_rgb_test(void) {
//     if (rgb_test_en && (rgb_test_index > 0)) {
//         rgb_matrix_set_color_all(rgb_test_color_table[rgb_test_index - 1][0], rgb_test_color_table[rgb_test_index - 1][1], rgb_test_color_table[rgb_test_index - 1][2]);
//     }
// }

// Bluetooth connection indication
static void indicator_bluetooth_connection(void) {
    if (dev_info.devs != DEVS_USB) {
        uint8_t         rgb_index      = rgb_index_table[dev_info.devs];
        static uint32_t last_time      = 0;
        static uint32_t last_long_time = 0;
        static uint8_t  last_status    = 0;
        static bool     rgb_flip       = false;
        static RGB      rgb            = {0};

        if (last_status != indicator_status) {
            last_status     = indicator_status;
            last_total_time = timer_read32();
        }

        if (indicator_reset_last_time != false) {
            indicator_reset_last_time = false;
            last_time                 = 0;
        }

        switch (indicator_status) {
            case 1: { // 闪烁模式 5Hz 重置
                if ((last_time == 0) || (timer_elapsed32(last_time) >= 200)) {
                    last_time = timer_read32();
                    rgb_flip  = !rgb_flip;
                    rgb       = rgb_flip ? (RGB){rgb_index_color_table[dev_info.devs][0], rgb_index_color_table[dev_info.devs][1], rgb_index_color_table[dev_info.devs][2]} : (RGB){0, 0, 0};
                }

                if (bts_info.bt_info.paired) {
                    last_long_time   = timer_read32();
                    indicator_status = 3;
                    break;
                }

                if (timer_elapsed32(last_total_time) >= (60 * 1000)) {
                    indicator_status = 0;
                    kb_sleep_flag    = true;
                }
            } break;

            case 2: { // 闪烁模式 2Hz 回连
                if ((last_time == 0) || (timer_elapsed32(last_time) >= 500)) {
                    last_time = timer_read32();
                    rgb_flip  = !rgb_flip;
                    rgb       = rgb_flip ? (RGB){rgb_index_color_table[dev_info.devs][0], rgb_index_color_table[dev_info.devs][1], rgb_index_color_table[dev_info.devs][2]} : (RGB){0, 0, 0};
                }

                if (bts_info.bt_info.paired) {
                    last_long_time   = timer_read32();
                    indicator_status = 3;
                    break;
                }

                if (timer_elapsed32(last_total_time) >= (10 * 1000)) {
                    indicator_status = 0;
                    kb_sleep_flag    = true;
                }
            } break;

            case 3: { // 长亮模式
                if (timer_elapsed32(last_long_time) < (2 * 1000)) {
                    rgb = (RGB){rgb_index_color_table[dev_info.devs][0], rgb_index_color_table[dev_info.devs][1], rgb_index_color_table[dev_info.devs][2]};
                } else {
                    indicator_status = 0;
                }
            } break;

            case 4: { // 长灭模式
                rgb = (RGB){0, 0, 0};
            } break;

            default:
                rgb_flip = false;
                if (!kb_sleep_flag) {
                    if (!bts_info.bt_info.paired) {
                        if (!bts_info.bt_info.pairing) {
                            indicator_status = 2;
                            break;
                        }
                        indicator_status = 2;
                        if (!bts_info.bt_info.pairing) {
                            if (dev_info.devs == DEVS_2_4G) {
                                bt_switch_mode(DEVS_USB, DEVS_2_4G, false);
                            } else {
                                bt_switch_mode(DEVS_USB, dev_info.last_devs, false);
                            }
                        }
                        break;
                    }
                }
        }

        if (indicator_status) rgb_matrix_set_color(rgb_index, rgb.r, rgb.g, rgb.b);
    }
}

uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max) {
    battery_voltage_query();

    battery_low_warning();

    battery_voltage_display();

    indicator_device_connection();

    indicator_bluetooth_connection();

    indicator_all_blink();

    indicator_single_blink();

    // indicator_rgb_test();

    indicator_factory_reset();

    return true;
}
