// !!! DO NOT ADD #pragma once !!! //

RGB_MATRIX_EFFECT(EFFECT_ROLL_WHITE)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

static bool EFFECT_ROLL_WHITE(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    // 与 QMK cycle_left_right 使用相同的速度计算方式。
    uint8_t time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_config.speed / 4, 1));

    // 用户设置的亮度仍然有效。
    uint8_t max_brightness = rgb_matrix_config.hsv.v;

    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();

        // 横坐标与时间共同形成从左向右移动的平滑亮度波。
        uint8_t phase      = g_led_config.point[i].x - time;
        uint8_t brightness = scale8(sin8(phase), max_brightness);

        // R、G、B 相等，因此始终是白光。
        rgb_matrix_set_color(i, brightness / 2, brightness, brightness);
    }

    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
