#pragma once

#include "quantum.h"

#ifdef RGB_MATRIX_ENABLE

typedef enum {
    IND_WIRELESS,
    IND_CAPS_LOCK,
    IND_GUI_LOCK,
    IND_LOW_BATTERY,
    IND_FACTORY_RESET,
    IND_FACTORY_RESET_LOGO,
    IND_CUSTOM_1,
    IND_COUNT,
} indicator_id_t;

typedef enum {
    IND_SOLID,
    IND_BLINK,
} indicator_effect_t;

typedef enum {
    IND_OFF_BLACK,
    IND_OFF_TRANSPARENT,
} indicator_off_mode_t;

typedef struct {
    // 目标范围：[first_led, first_led + led_count)
    // 单灯：led_count = 1
    // 全部：first_led = 0，led_count = RGB_MATRIX_LED_COUNT
    uint8_t first_led;
    uint8_t led_count;

    indicator_effect_t effect;
    RGB                color;

    uint16_t on_ms;
    uint16_t off_ms;

    // 0 表示持续运行，直到主动停止。
    uint32_t duration_ms;

    // 数值越大，优先级越高。
    uint8_t priority;

    indicator_off_mode_t off_mode;
} indicator_config_t;

// 每次调用都重新开始；配置会复制，不保存调用者指针。
// 配置无效时返回 false，不改变已有状态。
bool indicator_start(indicator_id_t id, const indicator_config_t *config);

void indicator_stop(indicator_id_t id);

// 用于大写、锁 Win、低电等持续状态。
// 重复 enabled=true 不会重启正在运行的效果。
// 如需改变正在运行的配置，调用 indicator_start。
void indicator_set(indicator_id_t id, bool enabled, const indicator_config_t *config);

void indicator_task(void);
void indicator_render(uint8_t led_min, uint8_t led_max);

#endif
