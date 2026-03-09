#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_brake.h"

/* BH1750 commands */
#define BH1750_POWER_ON     0x01
#define BH1750_CONT_HRES    0x10  /* Continuous high-resolution mode (1 lx) */

static bool bh1750_init(void) {
    /* Initialize I2C at 100 kHz (more reliable for startup) */
    i2c_init(BH1750_I2C, 100 * 1000);
    gpio_set_function(BH1750_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BH1750_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BH1750_SDA_PIN);
    gpio_pull_up(BH1750_SCL_PIN);

    /* Power on the sensor */
    uint8_t cmd = BH1750_POWER_ON;
    int ret = i2c_write_blocking(BH1750_I2C, BH1750_ADDR, &cmd, 1, false);
    if (ret < 0) {
        printf("BH1750 power-on failed (err %d)\n", ret);
        return false;
    }

    /* Set continuous high-resolution mode */
    cmd = BH1750_CONT_HRES;
    ret = i2c_write_blocking(BH1750_I2C, BH1750_ADDR, &cmd, 1, false);
    if (ret < 0) {
        printf("BH1750 mode set failed (err %d)\n", ret);
        return false;
    }

    printf("BH1750 initialized (addr=0x%02X)\n", BH1750_ADDR);
    return true;
}

/* Read illuminance in lux. Returns -1 on error. */
static float bh1750_read_lux(void) {
    uint8_t buf[2];
    if (i2c_read_blocking(BH1750_I2C, BH1750_ADDR, buf, 2, false) < 0)
        return -1.0f;

    /* BH1750 returns 16-bit raw value, MSB first. Lux = raw / 1.2 */
    uint16_t raw = (buf[0] << 8) | buf[1];
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

    /* Capture ambient light level at startup as baseline */
    vTaskDelay(pdMS_TO_TICKS(200));  /* wait for first BH1750 measurement */
    float ambient = bh1750_read_lux();
    if (ambient < 0.0f) ambient = 0.0f;
    printf("Ambient light: %.0f lux\n", ambient);

    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t print_counter = 0;
    for (;;) {
        float lux = bh1750_read_lux();
        bool braking = false;

        if (lux >= 0.0f) {
            /* Brake detected if current light exceeds ambient + threshold */
            braking = (lux > ambient + BRAKE_LUX_THRESHOLD);
        }

        /* Print debug every ~500ms (every 5 iterations at 100ms period) */
        if (++print_counter % 5 == 0) {
            printf("BH1750: lux=%.0f ambient=%.0f thr=%.0f brake=%s\n",
                   lux, ambient, ambient + BRAKE_LUX_THRESHOLD,
                   braking ? "YES" : "no");
        }

        xQueueOverwrite(xQueueBrake, &braking);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
}
