#include QMK_KEYBOARD_H
#include "wireless.h"
#include "module.h"
#include "hfdkb_indicator_common.h"
#include "usb_main.h"
#include "lib/lib8tion/lib8tion.h"

typedef union {
    uint32_t raw;
    struct {
        uint8_t devs : 3;
        uint8_t last_devs : 3;
    };
} confinfo_t;
confinfo_t confinfo;

static uint32_t post_init_timer = 0;

uint32_t eeconfig_confinfo_read(void) {
    return eeconfig_read_kb();
}

void eeconfig_confinfo_update(uint32_t raw) {
    eeconfig_update_kb(raw);
}

void eeconfig_confinfo_default(void) {
#ifdef WIRELESS_ENABLE
    confinfo.devs = DEVS_USB;
#endif

    eeconfig_confinfo_update(confinfo.raw);
}

void eeconfig_confinfo_init(void) {
    confinfo.raw = eeconfig_confinfo_read();
    if (!confinfo.raw) {
        eeconfig_confinfo_default();
    }
}

void wireless_post_init(void) {
    eeconfig_confinfo_init();

#ifdef USB_POWER_EN_PIN
    if (confinfo.devs == DEVS_USB && gpio_read_pin(BT_CABLE_PIN)) {
        gpio_write_pin_low(USB_POWER_EN_PIN);
    } else {
        gpio_write_pin_high(USB_POWER_EN_PIN);
    }
    gpio_set_pin_output(USB_POWER_EN_PIN);
#endif

#ifdef WIRELESS_ENABLE
    wireless_init();
    md_send_devinfo(MD_BT_NAME);
    wait_ms(10);
    wireless_devs_change(!confinfo.devs, confinfo.devs, false);
    post_init_timer = timer_read32();
#endif
}

void wireless_post_task(void) {
    // auto switching devs
    if (post_init_timer && timer_elapsed32(post_init_timer) >= 100) {
        md_send_devctrl(MD_SND_CMD_DEVCTRL_FW_VERSION);   // get the module fw version.
        md_send_devctrl(MD_SND_CMD_DEVCTRL_SLEEP_BT_EN);  // timeout 30min to sleep in bt mode, enable
        md_send_devctrl(MD_SND_CMD_DEVCTRL_SLEEP_2G4_EN); // timeout 30min to sleep in 2.4g mode, enable
        wireless_devs_change(!confinfo.devs, confinfo.devs, false);
        post_init_timer = 0x00;
    }
}

void wireless_wakeup_init(void) {
    wireless_devs_change(wireless_get_current_devs(), wireless_get_current_devs(), false);
}

static bool low_vol_warning = false;
static bool low_vol_off     = false;

static void wls_factory_reset_feedback(void) {
#ifdef RGB_MATRIX_ENABLE
    indicator_start(IND_FACTORY_RESET, &ind_reset_config);
    indicator_start(IND_FACTORY_RESET_LOGO, &ind_reset_logo_config);
#endif
}

#ifndef WLS_KEYCODE_PAIR_TIME
#    define WLS_KEYCODE_PAIR_TIME 3000
#endif

#ifndef WLS_FACTORY_RESET_HOLD_TIME
#    define WLS_FACTORY_RESET_HOLD_TIME 3000
#endif

typedef struct {
    uint16_t keycode;
    uint32_t hold_ms;
    void (*on_press)(uint16_t keycode);
    deferred_exec_callback on_hold;
    deferred_token         token;
    bool                   pressed;
} wls_hold_key_t;

static void wls_hold_select_device(uint16_t keycode) {
    uint8_t dev;

    switch (keycode) {
        case KC_BT1:
            dev = DEVS_BT1;
            break;
        case KC_BT2:
            dev = DEVS_BT2;
            break;
        case KC_BT3:
            dev = DEVS_BT3;
            break;
        case KC_2G4:
            dev = DEVS_2G4;
            break;
        default:
            return;
    }

    if (wireless_get_current_devs() != dev) {
        wireless_devs_change(wireless_get_current_devs(), dev, false);
    }
}

static uint32_t wls_process_long_press(uint32_t trigger_time, void *cb_arg) {
    uint16_t keycode = *((uint16_t *)cb_arg);

    switch (keycode) {
        case KC_BT1: {
            wireless_devs_change(wireless_get_current_devs(), DEVS_BT1, true);
        } break;
        case KC_BT2: {
            wireless_devs_change(wireless_get_current_devs(), DEVS_BT2, true);
        } break;
        case KC_BT3: {
            wireless_devs_change(wireless_get_current_devs(), DEVS_BT3, true);
        } break;
        case KC_2G4: {
            wireless_devs_change(wireless_get_current_devs(), DEVS_2G4, true);
        } break;
        default:
            break;
    }

    return 0;
}

static void wls_factory_reset(void) {
    eeconfig_init();
    eeconfig_update_rgb_matrix_default();

    if (keymap_config.no_gui) {
        keymap_config.no_gui = false;
    }
}

static uint32_t wls_factory_reset_long_press(uint32_t trigger_time, void *cb_arg) {
    (void)trigger_time;
    (void)cb_arg;

    wls_factory_reset();
    wls_factory_reset_feedback();
    return 0;
}

static wls_hold_key_t wls_hold_keys[] = {
    {
        .keycode  = KC_BT1,
        .hold_ms  = WLS_KEYCODE_PAIR_TIME,
        .on_press = wls_hold_select_device,
        .on_hold  = wls_process_long_press,
    },
    {
        .keycode  = KC_BT2,
        .hold_ms  = WLS_KEYCODE_PAIR_TIME,
        .on_press = wls_hold_select_device,
        .on_hold  = wls_process_long_press,
    },
    {
        .keycode  = KC_BT3,
        .hold_ms  = WLS_KEYCODE_PAIR_TIME,
        .on_press = wls_hold_select_device,
        .on_hold  = wls_process_long_press,
    },
    {
        .keycode  = KC_2G4,
        .hold_ms  = WLS_KEYCODE_PAIR_TIME,
        .on_press = wls_hold_select_device,
        .on_hold  = wls_process_long_press,
    },
    {
        .keycode = EE_CLR,
        .hold_ms = WLS_FACTORY_RESET_HOLD_TIME,
        .on_hold = wls_factory_reset_long_press,
    },
};

static uint32_t wls_hold_callback(uint32_t trigger_time, void *cb_arg) {
    wls_hold_key_t *key = cb_arg;

    key->token = INVALID_DEFERRED_TOKEN;

    if (key->pressed && key->on_hold != NULL) {
        key->on_hold(trigger_time, &key->keycode);
    }

    return 0;
}

static bool wls_hold_process_record(uint16_t keycode, keyrecord_t *record) {
    for (size_t i = 0; i < ARRAY_SIZE(wls_hold_keys); ++i) {
        wls_hold_key_t *key = &wls_hold_keys[i];

        if (key->keycode != keycode) {
            continue;
        }

        if (record->event.pressed) {
            if (!key->pressed) {
                key->pressed = true;

                if (key->on_press != NULL) {
                    key->on_press(key->keycode);
                }

                key->token = defer_exec(key->hold_ms, wls_hold_callback, key);
            }
        } else {
            key->pressed = false;

            if (key->token != INVALID_DEFERRED_TOKEN) {
                cancel_deferred_exec(key->token);
                key->token = INVALID_DEFERRED_TOKEN;
            }
        }

        return false;
    }

    return true;
}

bool wireless_process_record(uint16_t keycode, keyrecord_t *record) {
    if (!wls_hold_process_record(keycode, record)) {
        return false;
    }

    switch (keycode) {
        case KC_USB:
            if (record->event.pressed) {
                clear_keyboard();
                layer_clear();
                wireless_devs_change(wireless_get_current_devs(), DEVS_USB, false);
            }
            return false;

        default:
            return true;
    }
}

bool     query_vol_flag = false;
uint32_t key_press_time = 0;

static bool     sober          = true;
static bool     bak_rgb_toggle = false;
static uint32_t close_rgb_time = 0;

void open_rgb(void) {
    key_press_time = timer_read32();
#ifdef LED_POWER_EN_PIN
    gpio_write_pin_low(LED_POWER_EN_PIN);
#endif
    if (!sober) {
        if (bak_rgb_toggle) {
            rgb_matrix_enable_noeeprom();
        }
        sober = true;
    }
}

void close_rgb(void) {
    if (!key_press_time) {
        key_press_time = timer_read32();
        return;
    }

    if (sober) {
        if (timer_elapsed32(key_press_time) >= (1 * 60 * 1000)) {
            bak_rgb_toggle = rgb_matrix_config.enable;
            sober          = false;
            close_rgb_time = timer_read32();
            rgb_matrix_disable_noeeprom();
#ifdef LED_POWER_EN_PIN
            gpio_write_pin_high(LED_POWER_EN_PIN);
#endif
        }
    } else {
        if (timer_elapsed32(close_rgb_time) >= (5 * 60 * 1000)) {
            lpwr_set_state(LPWR_PRESLEEP);
        }
    }
}

void matrix_scan_kb(void) {
    if (confinfo.devs != DEVS_USB) {
        close_rgb();
    }
    matrix_scan_user();
}

#ifdef RGB_MATRIX_ENABLE

#    ifdef WIRELESS_ENABLE

static indicator_config_t wireless_indicator_config;

static bool     wireless_waiting    = false;
static uint32_t wireless_started    = 0;
static uint32_t wireless_timeout    = 0;
static uint8_t  wireless_last_state = MD_STATE_NONE;

static void wireless_indicator_begin(uint8_t dev, bool pairing) {
    indicator_config_t cfg = {
        .led_count = 1,
        .effect    = IND_BLINK,
        .priority  = 60,
        .off_mode  = IND_OFF_BLACK,
    };

    switch (dev) {
        case DEVS_BT1:
            cfg.first_led = LED_HOST_BT1_INDEX;
            cfg.color     = (RGB)LED_HOST_BT1_COLOR;
            break;
        case DEVS_BT2:
            cfg.first_led = LED_HOST_BT2_INDEX;
            cfg.color     = (RGB)LED_HOST_BT2_COLOR;
            break;
        case DEVS_BT3:
            cfg.first_led = LED_HOST_BT3_INDEX;
            cfg.color     = (RGB)LED_HOST_BT3_COLOR;
            break;
        case DEVS_2G4:
            cfg.first_led = LED_HOST_2G4_INDEX;
            cfg.color     = (RGB)LED_HOST_2G4_COLOR;
            break;
        default:
            wireless_waiting = false;
            indicator_stop(IND_WIRELESS);
            return;
    }

    if (dev == DEVS_2G4) {
        cfg.on_ms       = pairing ? HFDKB_IND_2G4_PAIR_ON_MS : HFDKB_IND_2G4_RECONNECT_ON_MS;
        cfg.off_ms      = pairing ? HFDKB_IND_2G4_PAIR_OFF_MS : HFDKB_IND_2G4_RECONNECT_OFF_MS;
        cfg.duration_ms = pairing ? HFDKB_IND_2G4_PAIR_DURATION_MS : HFDKB_IND_2G4_RECONNECT_DURATION_MS;
    } else {
        cfg.on_ms       = pairing ? HFDKB_IND_BT_PAIR_ON_MS : HFDKB_IND_BT_RECONNECT_ON_MS;
        cfg.off_ms      = pairing ? HFDKB_IND_BT_PAIR_OFF_MS : HFDKB_IND_BT_RECONNECT_OFF_MS;
        cfg.duration_ms = pairing ? HFDKB_IND_BT_PAIR_DURATION_MS : HFDKB_IND_BT_RECONNECT_DURATION_MS;
    }

    wireless_indicator_config = cfg;
    wireless_started          = timer_read32();
    wireless_timeout          = cfg.duration_ms;
    wireless_waiting          = true;

    indicator_start(IND_WIRELESS, &cfg);
}

void wireless_devs_change_kb(uint8_t old_devs, uint8_t new_devs, bool reset) {
    (void)old_devs;

    if (confinfo.devs != new_devs) {
        confinfo.devs = new_devs;
        eeconfig_confinfo_update(confinfo.raw);
    }

    wireless_last_state = *md_getp_state();
    wireless_indicator_begin(new_devs, reset);
}

static void wireless_indicator_task(void) {
    uint8_t dev   = wireless_get_current_devs();
    uint8_t state = *md_getp_state();

    if (dev == DEVS_USB) {
        wireless_waiting    = false;
        wireless_last_state = state;
        indicator_stop(IND_WIRELESS);
        return;
    }

    if (state != wireless_last_state) {
        if (state == MD_STATE_CONNECTED) {
            if (wireless_indicator_config.led_count != 0) {
                indicator_config_t cfg = wireless_indicator_config;

                cfg.effect      = IND_SOLID;
                cfg.duration_ms = HFDKB_IND_CONNECTED_DURATION_MS;

                indicator_start(IND_WIRELESS, &cfg);
            }

            wireless_waiting = false;
        } else if (state == MD_STATE_PAIRING) {
            wireless_indicator_begin(dev, true);
        } else if (state == MD_STATE_DISCONNECTED && !wireless_waiting) {
            wireless_indicator_begin(dev, false);
        }

        wireless_last_state = state;
    }

    // 无线连接超时属于业务逻辑，不放在 RGB 渲染函数中。
    if (wireless_waiting && wireless_timeout != 0 && timer_elapsed32(wireless_started) >= wireless_timeout) {
        wireless_waiting = false;
        indicator_stop(IND_WIRELESS);

        if (state != MD_STATE_CONNECTED) {
            lpwr_set_state(LPWR_PRESLEEP);
        }
    }
}

#    endif // WIRELESS_ENABLE

static void hfdkb_indicator_task(void) {
    indicator_set(IND_CAPS_LOCK, host_keyboard_led_state().caps_lock, &ind_caps_config);

    indicator_set(IND_GUI_LOCK, keymap_config.no_gui, &ind_gui_config);

#    ifdef WIRELESS_ENABLE
    wireless_indicator_task();

    // 沿用原代码的判断：BT_CABLE_PIN 高表示电池供电。
    bool battery_power = confinfo.devs != DEVS_USB && gpio_read_pin(BT_CABLE_PIN);

    if (battery_power) {
        uint8_t battery = *md_getp_bat();

        if (battery < 1 && !low_vol_off) {
            low_vol_off = true;
            lpwr_set_state(LPWR_PRESLEEP);
        }

        // 保留原先低电告警的锁存行为，接入电源后清除。
        if (battery <= 10) {
            low_vol_warning = true;
        }
    } else {
        low_vol_warning = false;
        low_vol_off     = false;
    }

    indicator_set(IND_LOW_BATTERY, battery_power && low_vol_warning && !low_vol_off, &ind_low_bat_config);
#    endif

    indicator_task();
}

bool wireless_indicators_advanced(uint8_t led_min, uint8_t led_max) {
    static uint8_t pvol = 0;

    pvol = *md_getp_bat();

    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
        rgb_matrix_set_color_all(0, 0, 0);
    } else {
        // Logo led effect
        uint8_t time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_get_speed() / 4, 1));
        for (uint8_t i = 68; i <= 74; i++) {
            HSV hsv = {g_led_config.point[i].x - time, 255, rgb_matrix_get_val() / 3};
            RGB rgb = hsv_to_rgb(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
        }
    }

    if (confinfo.devs != DEVS_USB) {
        if (query_vol_flag) {
            for (uint8_t i = 68; i <= 74; i++) {
                rgb_matrix_set_color(i, 0, 0, 0);
            }

            uint8_t query_index[] = {74, 73, 72, 71, 70, 69, 68};
            uint8_t led_count     = 0;

            if (pvol >= 95)
                led_count = 7;
            else if (pvol >= 80)
                led_count = 6;
            else if (pvol >= 60)
                led_count = 5;
            else if (pvol >= 40)
                led_count = 4;
            else if (pvol >= 20)
                led_count = 3;
            else if (pvol > 10)
                led_count = 2;
            else if (pvol > 0)
                led_count = 1;
            else
                led_count = 0;

            RGB color = (RGB){14, 14, 14};
            for (uint8_t i = 0; i < led_count; i++) {
                rgb_matrix_set_color(query_index[i], color.r, color.g, color.b);
            }
        }
    }

    // 保留用户灯效回调；系统指示在它之后叠加。
    bool user_result = rgb_matrix_indicators_advanced_user(led_min, led_max);

    // uint8_t layer = get_highest_layer(default_layer_state | layer_state);

    // if (layer == 1 || layer == 4) {
    //     uint8_t dev = confinfo.devs;

    //     if (dev < ARRAY_SIZE(rgb_index_table)) {
    //         uint8_t index = rgb_index_table[dev];

    //         if (index < RGB_MATRIX_LED_COUNT && index >= led_min && index < led_max) {
    //             rgb_matrix_set_color(index, rgb_index_color_table[dev][0], rgb_index_color_table[dev][1], rgb_index_color_table[dev][2]);
    //         }
    //     }
    // }

    // 最后统一叠加，保证优先级只由引擎决定。
    indicator_render(led_min, led_max);

    return user_result;
}

void md_devs_change(uint8_t devs, bool reset) {
    switch (devs) {
        case DEVS_USB: {
            md_send_devctrl(MD_SND_CMD_DEVCTRL_USB);
        } break;
        case DEVS_2G4: {
            if (reset) {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_PAIR);
            } else {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_2G4);
            }
        } break;
        case DEVS_BT1: {
            if (reset) {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_PAIR);
            } else {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_BT1);
            }
        } break;
        case DEVS_BT2: {
            if (reset) {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_PAIR);
            } else {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_BT2);
            }
        } break;
        case DEVS_BT3: {
            if (reset) {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_PAIR);
            } else {
                md_send_devctrl(MD_SND_CMD_DEVCTRL_BT3);
            }
        } break;
        default:
            break;
    }
}

#endif // RGB_MATRIX_ENABLE

void wireless_send_nkro(report_nkro_t *report) {
    static report_keyboard_t temp_report_keyboard                 = {0};
    uint8_t                  wls_report_nkro[MD_SND_CMD_NKRO_LEN] = {0};

#ifdef NKRO_ENABLE

    if (report != NULL) {
        report_nkro_t temp_report_nkro = *report;
        uint8_t       key_count        = 0;

        temp_report_keyboard.mods = temp_report_nkro.mods;
        for (uint8_t i = 0; i < NKRO_REPORT_BITS; i++) {
            key_count += __builtin_popcount(temp_report_nkro.bits[i]);
        }

        /*
         * Use NKRO for sending when more than 6 keys are pressed
         * to solve the issue of the lack of a protocol flag in wireless mode.
         */

        for (uint8_t i = 0; i < key_count; i++) {
            uint8_t usageid;
            uint8_t idx, n = 0;

            for (n = 0; n < NKRO_REPORT_BITS && !temp_report_nkro.bits[n]; n++) {
            }
            usageid = (n << 3) | biton(temp_report_nkro.bits[n]);
            del_key_bit(&temp_report_nkro, usageid);

            for (idx = 0; idx < WLS_KEYBOARD_REPORT_KEYS; idx++) {
                if (temp_report_keyboard.keys[idx] == usageid) {
                    goto next;
                }
            }

            for (idx = 0; idx < WLS_KEYBOARD_REPORT_KEYS; idx++) {
                if (temp_report_keyboard.keys[idx] == 0x00) {
                    temp_report_keyboard.keys[idx] = usageid;
                    break;
                }
            }
        next:
            if (idx == WLS_KEYBOARD_REPORT_KEYS && (usageid < (MD_SND_CMD_NKRO_LEN * 8))) {
                wls_report_nkro[usageid / 8] |= 0x01 << (usageid % 8);
            }
        }

        temp_report_nkro = *report;

        // find key up and del it.
        uint8_t nkro_keys = key_count;
        for (uint8_t i = 0; i < WLS_KEYBOARD_REPORT_KEYS; i++) {
            report_nkro_t found_report_nkro;
            uint8_t       usageid = 0x00;
            uint8_t       n;

            found_report_nkro = temp_report_nkro;

            for (uint8_t c = 0; c < nkro_keys; c++) {
                for (n = 0; n < NKRO_REPORT_BITS && !found_report_nkro.bits[n]; n++) {
                }
                usageid = (n << 3) | biton(found_report_nkro.bits[n]);
                del_key_bit(&found_report_nkro, usageid);
                if (usageid == temp_report_keyboard.keys[i]) {
                    del_key_bit(&temp_report_nkro, usageid);
                    nkro_keys--;
                    break;
                }
            }

            if (usageid != temp_report_keyboard.keys[i]) {
                temp_report_keyboard.keys[i] = 0x00;
            }
        }
    } else {
        memset(&temp_report_keyboard, 0, sizeof(temp_report_keyboard));
    }
#endif
    void wireless_task(void);
    bool smsg_is_busy(void);
    while (smsg_is_busy()) {
        wireless_task();
    }
    extern host_driver_t wireless_driver;
    wireless_driver.send_keyboard(&temp_report_keyboard);
    md_send_nkro(wls_report_nkro);
}

#ifdef WIRELESS_ENABLE
void housekeeping_task_wls(void) {
    if (confinfo.devs == DEVS_USB) {
        if ((USB_DRIVER.state == USB_SUSPENDED) && (USB_DRIVER.saved_state == USB_ACTIVE)) {
            print("[s]");
            while (USB_DRIVER.state == USB_SUSPENDED) {
                /* Do this in the suspended state */
                suspend_power_down(); // on AVR this deep sleeps for 15ms
                /* Remote wakeup */
                if (suspend_wakeup_condition()) {
                    usbWakeupHost(&USB_DRIVER);
                    restart_usb_driver(&USB_DRIVER);
                }
            }
            /* Woken up */
            // variables has been already cleared by the wakeup hook
            send_keyboard_report();
#    ifdef MOUSEKEY_ENABLE
            mousekey_send();
#    endif /* MOUSEKEY_ENABLE */
        }
    }
}
#endif

void wireless_kb_task(void) {
    static uint32_t usb_suspend_timer = 0;
    static uint32_t usb_suspend       = false;

#ifdef RGB_MATRIX_ENABLE
    hfdkb_indicator_task();
#endif

#ifdef WIRELESS_ENABLE
    housekeeping_task_wls();
#endif

    if (confinfo.devs == DEVS_USB) {
        if (usb_suspend) {
            if (suspend_wakeup_condition()) {
                // usbWakeupHost(&USB_DRIVER);
                // restart_usb_driver(&USB_DRIVER);
                usb_suspend       = false;
                usb_suspend_timer = 0;
#ifdef LED_POWER_EN_PIN
                gpio_write_pin_low(LED_POWER_EN_PIN);
#endif
            }
        }

        if ((USB_DRIVER.state != USB_ACTIVE)) {
            if (!usb_suspend_timer) {
                usb_suspend_timer = timer_read32();
            } else if (timer_elapsed32(usb_suspend_timer) > 10000) {
                usb_suspend_timer = 0;
                if (!usb_suspend) {
                    usb_suspend = true;
#ifdef LED_POWER_EN_PIN
                    gpio_write_pin_high(LED_POWER_EN_PIN);
#endif
                    // lpwr_set_state(LPWR_PRESLEEP);
                }
            }
        } else {
            if (usb_suspend) {
                usb_suspend_timer = 0;
                usb_suspend       = false;

#ifdef LED_POWER_EN_PIN
                gpio_write_pin_low(LED_POWER_EN_PIN);
#endif
            }
        }
    }
}
