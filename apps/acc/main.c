#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "common.h"
#include "task_sensor.h"
#include "task_dimmer.h"
#include "task_led.h"

/* Shared mailbox queues */
QueueHandle_t xQueueDistance;
QueueHandle_t xQueueThreshold;

#define SENSOR_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3UL )
#define DIMMER_TASK_PRIORITY    ( tskIDLE_PRIORITY + 2UL )
#define LED_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2UL )
#define DEBUG_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

#define TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE )

void vTaskDebug(void *params) {
    (void)params;
    float distance = 0.0f;
    float threshold = 0.0f;
    for (;;) {
        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);
        printf("dist=%.1f cm  thr=%.1f cm\n", distance, threshold);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("Starting ACC...\n");

    /* Create mailbox queues (depth 1) */
    xQueueDistance  = xQueueCreate(1, sizeof(float));
    xQueueThreshold = xQueueCreate(1, sizeof(float));

    /* Seed initial values */
    float init_dist = 999.0f;
    float init_thr  = 50.0f;
    xQueueOverwrite(xQueueDistance,  &init_dist);
    xQueueOverwrite(xQueueThreshold, &init_thr);

    /* Create tasks */
    xTaskCreate(vTaskSensor, "Sensor", TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(vTaskDimmer, "Dimmer", TASK_STACK_SIZE, NULL, DIMMER_TASK_PRIORITY, NULL);
    xTaskCreate(vTaskLED,    "LED",    TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY,    NULL);
    xTaskCreate(vTaskDebug,  "Debug",  TASK_STACK_SIZE, NULL, DEBUG_TASK_PRIORITY,  NULL);

    vTaskStartScheduler();
    return 0;
}
