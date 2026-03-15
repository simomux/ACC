#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_sensor.h"

/*
 * HC-SR04 measurement:
 *   1. Send 10us HIGH pulse on trigger
 *   2. Measure echo pulse duration
 *   3. distance_cm = duration_us / 58
 */

#define ECHO_TIMEOUT_US  25000  /* ~4.3m max range */

static float measure_distance_cm(void) {
    /* Send trigger pulse */
    gpio_put(SENSOR_TRIGGER_PIN, true);
    busy_wait_us(10);
    gpio_put(SENSOR_TRIGGER_PIN, false);

    /* Wait for echo to go HIGH */
    absolute_time_t deadline = make_timeout_time_us(ECHO_TIMEOUT_US);
    while (!gpio_get(SENSOR_ECHO_PIN)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
            return -1.0f;
    }

    /* Measure echo HIGH duration */
    absolute_time_t start = get_absolute_time();
    while (gpio_get(SENSOR_ECHO_PIN)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    int64_t duration_us = absolute_time_diff_us(start, get_absolute_time());
    return (float)duration_us / 58.0f;
}

/* Median-of-3 filter: rejects outlier readings from the HC-SR04 */
static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    if (a > b) { b = a; }
    return b;
}

#define INTER_MEASUREMENT_MS  10  /* gap between readings for echo decay */

void vTaskSensor(void *params) {
    (void)params;

    gpio_init(SENSOR_TRIGGER_PIN);
    gpio_set_dir(SENSOR_TRIGGER_PIN, GPIO_OUT);
    gpio_put(SENSOR_TRIGGER_PIN, false);

    gpio_init(SENSOR_ECHO_PIN);
    gpio_set_dir(SENSOR_ECHO_PIN, GPIO_IN);

    printf("vTaskSensor started\n");

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        /* Take 3 rapid measurements */
        float samples[3];
        int valid = 0;
        for (int i = 0; i < 3; i++) {
            samples[i] = measure_distance_cm();
            if (samples[i] >= 0.0f) valid++;
            if (i < 2) vTaskDelay(pdMS_TO_TICKS(INTER_MEASUREMENT_MS));
        }

        /* Publish first valid raw sample (pre-filter) for dashboard comparison */
        for (int i = 0; i < 3; i++) {
            if (samples[i] >= 0.0f) {
                xQueueOverwrite(xQueueDistanceRaw, &samples[i]);
                break;
            }
        }

        if (valid == 3) {
            float distance = median3(samples[0], samples[1], samples[2]);
            xQueueOverwrite(xQueueDistance, &distance);
        } else if (valid > 0) {
            /* If only some readings valid, use first valid one */
            for (int i = 0; i < 3; i++) {
                if (samples[i] >= 0.0f) {
                    xQueueOverwrite(xQueueDistance, &samples[i]);
                    break;
                }
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
