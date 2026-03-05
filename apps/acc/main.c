#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_PIN 15

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

#define MAIN_TASK_STACK_SIZE    configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE   configMINIMAL_STACK_SIZE

void blink_task(__unused void *params) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool on = false;
    printf("blink_task starts\n");
    while (true) {
        gpio_put(LED_PIN, on);
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void main_task(__unused void *params) {
    xTaskCreate(blink_task, "BlinkThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    int count = 0;
    while (true) {
        printf("Hello from ACC count=%u\n", count++);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("Starting ACC...\n");
    xTaskCreate(main_task, "MainThread", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL);
    vTaskStartScheduler();
    return 0;
}
