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
        uint16_t raw = adc_read();
        
        /* Quadratic curve (x²) maps the potentiometer range non-linearly:
           the lower half of the knob covers the near-distance range where
           precision matters most, while the upper half spans the wider
           distances where coarse steps are acceptable. This was done to
           increase sensitivity in the lower range. */
        float normalized = (float)raw / 4095.0f;
        float curved = normalized * normalized;
        float threshold = THRESHOLD_MIN_CM
            + curved * (THRESHOLD_MAX_CM - THRESHOLD_MIN_CM);

        xQueueOverwrite(xQueueThreshold, &threshold);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(DIMMER_PERIOD_MS));
    }
}
