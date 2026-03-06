#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_led.h"

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

void vTaskLED(void *params) {
    (void)params;

    led_rgb_init();
    printf("vTaskLED started\n");

    float distance = 0.0f;
    float threshold = THRESHOLD_MAX_CM;

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);

        if (distance > threshold * 2.0f) {
            /* Far away — green */
            led_rgb_set(false, true, false);
        } else if (distance > threshold) {
            /* Approaching — yellow */
            led_rgb_set(true, true, false);
        } else {
            /* Too close — red */
            led_rgb_set(true, false, false);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}
