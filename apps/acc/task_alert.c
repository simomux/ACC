#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_alert.h"

/* --- LED helpers --- */

static void led_rgb_init(void) {
    gpio_init(LED_R_PIN);
    gpio_init(LED_G_PIN);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
}

static void led_rgb_set(bool r, bool g, bool b) {
    gpio_put(LED_R_PIN, r);
    gpio_put(LED_G_PIN, g);
    gpio_put(LED_B_PIN, b);
}

/* --- Buzzer helpers --- */

static uint buzzer_slice;

static void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(buzzer_slice, false);
}

/* Start the buzzer at a given frequency with 50% duty cycle */
static void buzzer_on(uint freq_hz) {
    uint32_t clk = 125000000;  /* default system clock */
    uint32_t wrap = clk / freq_hz - 1;
    /* Keep wrap within 16-bit range using clock divider */
    uint16_t div = 1;
    while (wrap > 65535) {
        div++;
        wrap = clk / (freq_hz * div) - 1;
    }
    pwm_set_clkdiv(buzzer_slice, (float)div);
    pwm_set_wrap(buzzer_slice, (uint16_t)wrap);
    pwm_set_gpio_level(BUZZER_PIN, (uint16_t)(wrap / 2)); /* 50% duty */
    pwm_set_enabled(buzzer_slice, true);
}

static void buzzer_off(void) {
    pwm_set_enabled(buzzer_slice, false);
}

/* --- Alert task --- */

void vTaskAlert(void *params) {
    (void)params;

    led_rgb_init();
    buzzer_init();
    printf("vTaskAlert started\n");

    float distance  = 0.0f;
    float threshold = THRESHOLD_MAX_CM;
    uint  tick_count = 0;  /* counts task iterations for beep toggling */

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);

        tick_count++;

        if (distance > threshold * 2.0f) {
            /* Far away — green, buzzer off */
            led_rgb_set(false, true, false);
            buzzer_off();
        } else if (distance > threshold) {
            /* Approaching — yellow, slow beep (toggle every 5 ticks = ~2 Hz) */
            led_rgb_set(true, true, false);
            if ((tick_count % 10) < 5) {
                buzzer_on(1000);
            } else {
                buzzer_off();
            }
        } else {
            /* Too close — red, fast beep (toggle every 1 tick = ~10 Hz) */
            led_rgb_set(true, false, false);
            if (tick_count % 2) {
                buzzer_on(3000);
            } else {
                buzzer_off();
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(ALERT_PERIOD_MS));
    }
}
