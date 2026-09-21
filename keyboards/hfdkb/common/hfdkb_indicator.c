#include "hfdkb_indicator.h"

#ifdef RGB_MATRIX_ENABLE

typedef struct {
    bool               active;
    uint32_t           started_at;
    indicator_config_t config;
} indicator_state_t;

static indicator_state_t indicators[IND_COUNT];

static bool indicator_id_valid(indicator_id_t id) {
    return (unsigned)id < IND_COUNT;
}

bool indicator_start(indicator_id_t id, const indicator_config_t *config) {
    if (!indicator_id_valid(id) || config == NULL) {
        return false;
    }

    if (config->led_count == 0 || (uint16_t)config->first_led + config->led_count > RGB_MATRIX_LED_COUNT) {
        return false;
    }

    if (config->effect != IND_SOLID && config->effect != IND_BLINK) {
        return false;
    }

    if (config->effect == IND_BLINK && (config->on_ms == 0 || config->off_ms == 0)) {
        return false;
    }

    if (config->off_mode != IND_OFF_BLACK && config->off_mode != IND_OFF_TRANSPARENT) {
        return false;
    }

    indicator_state_t *state = &indicators[id];

    state->config     = *config;
    state->started_at = timer_read32();
    state->active     = true;

    return true;
}

void indicator_stop(indicator_id_t id) {
    if (indicator_id_valid(id)) {
        indicators[id].active = false;
    }
}

void indicator_set(indicator_id_t id, bool enabled, const indicator_config_t *config) {
    if (!indicator_id_valid(id)) {
        return;
    }

    if (!enabled) {
        indicator_stop(id);
    } else if (!indicators[id].active) {
        indicator_start(id, config);
    }
}

void indicator_task(void) {
    uint32_t now = timer_read32();

    for (uint8_t i = 0; i < IND_COUNT; ++i) {
        indicator_state_t *state = &indicators[i];

        if (state->active && state->config.duration_ms != 0 && (uint32_t)(now - state->started_at) >= state->config.duration_ms) {
            state->active = false;
        }
    }
}

void indicator_render(uint8_t led_min, uint8_t led_max) {
    uint32_t now = timer_read32();

    for (uint16_t led = led_min; led < led_max && led < RGB_MATRIX_LED_COUNT; ++led) {
        bool    found    = false;
        uint8_t priority = 0;
        RGB     output   = {0, 0, 0};

        for (uint8_t i = 0; i < IND_COUNT; ++i) {
            const indicator_state_t  *state = &indicators[i];
            const indicator_config_t *cfg   = &state->config;

            if (!state->active) {
                continue;
            }

            if (led < cfg->first_led || led >= (uint16_t)cfg->first_led + cfg->led_count) {
                continue;
            }

            uint32_t elapsed = now - state->started_at;

            // 即使 task 尚未运行，也不绘制已经到期的效果。
            if (cfg->duration_ms != 0 && elapsed >= cfg->duration_ms) {
                continue;
            }

            bool on = true;

            if (cfg->effect == IND_BLINK) {
                uint32_t period = (uint32_t)cfg->on_ms + cfg->off_ms;
                on              = elapsed % period < cfg->on_ms;
            }

            if (!on && cfg->off_mode == IND_OFF_TRANSPARENT) {
                continue;
            }

            // 优先级相同时，枚举中靠后的槽位覆盖靠前的槽位。
            if (!found || cfg->priority >= priority) {
                found    = true;
                priority = cfg->priority;
                output   = on ? cfg->color : (RGB){0, 0, 0};
            }
        }

        if (found) {
            rgb_matrix_set_color(led, output.r, output.g, output.b);
        }
    }
}

#endif
