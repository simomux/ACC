#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_sensor.h"



#define ECHO_TIMEOUT_US  25000  /* ~4.3m max range */

static float measure_distance_cm(void) {
    gpio_put(SENSOR_TRIGGER_PIN, true);
    busy_wait_us(10);   // Cannot use vTaskDelay because we would wait 1ms
    gpio_put(SENSOR_TRIGGER_PIN, false);

    absolute_time_t deadline = make_timeout_time_us(ECHO_TIMEOUT_US);
    while (!gpio_get(SENSOR_ECHO_PIN)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
            return -1.0f;
    }

    absolute_time_t start = get_absolute_time();
    while (gpio_get(SENSOR_ECHO_PIN)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    int64_t duration_us = absolute_time_diff_us(start, get_absolute_time());
    return (float)duration_us / 58.0f;
}

/* Median-of-3 filter: rejects single outlier readings caused by reflections
   or the HC-SR04 occasionally returning a spurious short pulse. */
static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    if (a > b) { b = a; }
    return b;
}

/* Gap between back-to-back pings: lets the ultrasonic echo fully decay before
   the next trigger so the sensor does not pick up its own previous pulse. */
#define INTER_MEASUREMENT_MS  10

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
        float samples[3];
        int valid = 0;
        for (int i = 0; i < 3; i++) {
            samples[i] = measure_distance_cm();
            if (samples[i] >= 0.0f) valid++;
            if (i < 2) vTaskDelay(pdMS_TO_TICKS(INTER_MEASUREMENT_MS));
        }

        /* Publish the unfiltered first valid sample for dashboard comparison
           against the median-filtered value (used in the dashboard graph. */
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
