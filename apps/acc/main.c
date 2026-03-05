#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

/* RGB LED pins */
#define LED_R_PIN 13
#define LED_G_PIN 14
#define LED_B_PIN 15

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define LED_TASK_PRIORITY       ( tskIDLE_PRIORITY + 1UL )

#define MAIN_TASK_STACK_SIZE    configMINIMAL_STACK_SIZE
#define LED_TASK_STACK_SIZE     configMINIMAL_STACK_SIZE

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

void led_task(__unused void *params) {
    led_rgb_init();
    printf("led_task starts\n");
    while (true) {
        led_rgb_set(false, true, false);   /* Green */
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_rgb_set(true, true, false);    /* Yellow (R+G) */
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_rgb_set(true, false, false);   /* Red */
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_rgb_set(false, false, false);  /* Off */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void main_task(__unused void *params) {
    xTaskCreate(led_task, "LedTask", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
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
