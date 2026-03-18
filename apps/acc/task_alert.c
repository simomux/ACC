#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_alert.h"

#define LED_PWM_WRAP  255   // Using PWM instead of boolen to light RGB LED for more visible colors

static void led_rgb_init(void) {
    gpio_set_function(LED_R_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_G_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_B_PIN, GPIO_FUNC_PWM);

    uint slices[] = {
        pwm_gpio_to_slice_num(LED_R_PIN),
        pwm_gpio_to_slice_num(LED_G_PIN),
        pwm_gpio_to_slice_num(LED_B_PIN)
    };
    for (int i = 0; i < 3; i++) {
        pwm_set_wrap(slices[i], LED_PWM_WRAP);
        pwm_set_enabled(slices[i], true);
    }
    pwm_set_gpio_level(LED_R_PIN, 0);
    pwm_set_gpio_level(LED_G_PIN, 0);
    pwm_set_gpio_level(LED_B_PIN, 0);
}

static void led_rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(LED_R_PIN, r);
    pwm_set_gpio_level(LED_G_PIN, g);
    pwm_set_gpio_level(LED_B_PIN, b);
}

static uint buzzer_slice;

static void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(buzzer_slice, false);
}

/* Frequency is set by adjusting the PWM wrap value. For low frequencies the
   wrap would exceed 16 bits, so we scale up the clock divider until it fits. */
static void buzzer_on(uint freq_hz, uint duty_div) {
    uint32_t clk = 125000000;
    uint32_t wrap = clk / freq_hz - 1;
    uint16_t div = 1;
    while (wrap > 65535) {
        div++;
        wrap = clk / (freq_hz * div) - 1;
    }
    pwm_set_clkdiv(buzzer_slice, (float)div);
    pwm_set_wrap(buzzer_slice, (uint16_t)wrap);
    pwm_set_gpio_level(BUZZER_PIN, (uint16_t)(wrap / duty_div));
    pwm_set_enabled(buzzer_slice, true);
}

static void buzzer_off(void) {
    pwm_set_enabled(buzzer_slice, false);
}

static void mute_button_init(void) {
    gpio_init(MUTE_BUTTON_PIN);
    gpio_set_dir(MUTE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MUTE_BUTTON_PIN);
}

void vTaskAlert(void *params) {
    (void)params;

    led_rgb_init();
    buzzer_init();
    mute_button_init();
    printf("vTaskAlert started\n");

    float distance  = 0.0f;
    float threshold = THRESHOLD_MAX_CM;
    bool  braking   = false;
    bool  muted     = false;
    bool  btn_prev  = true;  // pull-up: idle = HIGH
    uint  tick_count = 0;

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);
        xQueuePeek(xQueueBrake,     &braking,   0);

        // Toggle mute on falling edge
        bool btn_now = gpio_get(MUTE_BUTTON_PIN);
        if (btn_prev && !btn_now) {
            muted = !muted;
            printf("Buzzer %s\n", muted ? "MUTED" : "UNMUTED");
        }
        btn_prev = btn_now;

        // Warning zone starts 30% beyond the brake threshold plus a fixed 20 cm
        float warning_dist = threshold * 1.3f + 20.0f;

        // tick_count drives the buzzer blink patterns without a separate timer
        tick_count++;

        if (braking) {
            if (tick_count % 4 < 2) {
                led_rgb_set(255, 0, 0);
            } else {
                led_rgb_set(0, 0, 0);
            }
            if (!muted) buzzer_on(4000, 4); else buzzer_off();
        } else if (distance > warning_dist) {
            led_rgb_set(0, 255, 0);
            buzzer_off();
        } else if (distance > threshold) {
            // Approaching — slow beep (~2 Hz)
            led_rgb_set(255, 60, 0);
            if (!muted && (tick_count % 10) < 5) {
                buzzer_on(1000, 8);
            } else {
                buzzer_off();
            }
        } else {
            // Too close — fast beep (~10 Hz)
            led_rgb_set(255, 0, 0);
            if (!muted && (tick_count % 2)) {
                buzzer_on(3000, 4);
            } else {
                buzzer_off();
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(ALERT_PERIOD_MS));
    }
}
