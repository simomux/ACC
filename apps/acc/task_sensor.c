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
        float distance = measure_distance_cm();
        if (distance >= 0.0f) {
            xQueueOverwrite(xQueueDistance, &distance);
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
