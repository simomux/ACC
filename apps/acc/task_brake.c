#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_brake.h"

#define BH1750_BAUDRATE  100000

#define BH1750_POWER_ON   0x01
#define BH1750_CONT_HRES  0x10  /* Continuous high-resolution mode (1 lx) */

static bool bh1750_init(void) {
    i2c_init(BH1750_I2C_BUS, BH1750_BAUDRATE);
    gpio_set_function(BH1750_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BH1750_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BH1750_SDA_PIN);
    gpio_pull_up(BH1750_SCL_PIN);

    uint8_t cmd = BH1750_POWER_ON;
    if (i2c_write_blocking(BH1750_I2C_BUS, BH1750_ADDR, &cmd, 1, false) < 0) {
        printf("BH1750 power-on failed\n");
        return false;
    }

    cmd = BH1750_CONT_HRES;
    if (i2c_write_blocking(BH1750_I2C_BUS, BH1750_ADDR, &cmd, 1, false) < 0) {
        printf("BH1750 mode set failed\n");
        return false;
    }

    printf("BH1750 initialized (addr=0x%02X)\n", BH1750_ADDR);
    return true;
}

/* BH1750 returns 16-bit raw value MSB-first. Lux = raw / 1.2 (per datasheet). */
static float bh1750_read_lux(void) {
    uint8_t buf[2];
    if (i2c_read_blocking(BH1750_I2C_BUS, BH1750_ADDR, buf, 2, false) < 0)
        return -1.0f;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (float)raw / 1.2f;
}

void vTaskBrake(void *params) {
    (void)params;

    if (!bh1750_init()) {
        printf("BH1750 init failed!\n");
        vTaskDelete(NULL);
        return;
    }

    printf("vTaskBrake started\n");

    /* Wait for the BH1750 to complete its first measurement (≥120 ms in
       continuous high-res mode) then capture ambient light as a baseline.
       All subsequent readings are compared against this value so the brake
       detection adapts to the environment (tunnel, night, direct sunlight). */
    vTaskDelay(pdMS_TO_TICKS(200));
    float ambient = bh1750_read_lux();
    if (ambient < 0.0f) ambient = 0.0f;
    printf("Ambient light: %.0f lux\n", ambient);

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        float lux    = bh1750_read_lux();
        bool braking = false;
        xQueueOverwrite(xQueueLux, &lux);

        if (lux >= 0.0f)
            braking = (lux > ambient + BRAKE_LUX_THRESHOLD);

        xQueueOverwrite(xQueueBrake, &braking);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
}
