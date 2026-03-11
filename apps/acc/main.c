#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "common.h"
#include "task_sensor.h"
#include "task_dimmer.h"
#include "task_alert.h"
#include "task_brake.h"
#include "task_oled.h"
#include "i2c_manager.h"

/* Shared mailbox queues */
QueueHandle_t xQueueDistance;
QueueHandle_t xQueueThreshold;
QueueHandle_t xQueueBrake;
QueueHandle_t xQueueLux;


#define I2C_TASK_PRIORITY       ( tskIDLE_PRIORITY + 4UL )
#define SENSOR_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3UL )
#define BRAKE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 3UL )
#define DIMMER_TASK_PRIORITY    ( tskIDLE_PRIORITY + 2UL )
#define ALERT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2UL )
#define DEBUG_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )
#define OLED_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )

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
    /* Wait until USB serial is connected (or timeout after 10s) */
    for (int i = 0; i < 100 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }
    printf("Starting ACC...\n");

    /* Create mailbox queues (depth 1) */
    xQueueDistance  = xQueueCreate(1, sizeof(float));
    xQueueThreshold = xQueueCreate(1, sizeof(float));
    xQueueBrake     = xQueueCreate(1, sizeof(bool));
    xQueueLux       = xQueueCreate(1, sizeof(float));

    /* Seed initial values */
    float init_dist = 999.0f;
    float init_thr  = 50.0f;
    bool  init_brk  = false;
    float init_lux  = 0.0f;
    xQueueOverwrite(xQueueDistance,  &init_dist);
    xQueueOverwrite(xQueueThreshold, &init_thr);
    xQueueOverwrite(xQueueBrake,     &init_brk);
    xQueueOverwrite(xQueueLux,       &init_lux);
    /* Create tasks */

    //xTaskCreate(vTaskI2CManager, "I2C",    4096, NULL, I2C_TASK_PRIORITY,    NULL);
    xTaskCreate(vTaskSensor,     "Sensor", TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(vTaskBrake,      "Brake",  512, NULL, BRAKE_TASK_PRIORITY,  NULL);
    xTaskCreate(vTaskDimmer,     "Dimmer", TASK_STACK_SIZE, NULL, DIMMER_TASK_PRIORITY, NULL);
    xTaskCreate(vTaskAlert,      "Alert",  TASK_STACK_SIZE, NULL, ALERT_TASK_PRIORITY,  NULL);
    xTaskCreate(vTaskDebug,      "Debug",  TASK_STACK_SIZE, NULL, DEBUG_TASK_PRIORITY,  NULL);
    xTaskCreate(vTaskOled,       "OLED",   4096,            NULL, OLED_TASK_PRIORITY,   NULL);

    vTaskStartScheduler();
    return 0;
}
