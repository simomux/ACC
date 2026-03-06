#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_dimmer.h"

void vTaskDimmer(void *params) {
    (void)params;

    adc_init();
    adc_gpio_init(DIMMER_ADC_PIN);
    adc_select_input(DIMMER_ADC_INPUT);

    printf("vTaskDimmer started\n");

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        uint16_t raw = adc_read();  /* 0-4095 (12-bit) */
        /* Map ADC range to threshold in cm */
        float threshold = THRESHOLD_MIN_CM
            + (float)raw / 4095.0f * (THRESHOLD_MAX_CM - THRESHOLD_MIN_CM);

        xQueueOverwrite(xQueueThreshold, &threshold);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(DIMMER_PERIOD_MS));
    }
}
