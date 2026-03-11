#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "task_brake.h"

/* ------------------------------------------------------------------ */
/*  BH1750 I2C bus — dedicated i2c1, independent from OLED             */
/* ------------------------------------------------------------------ */

#define BH1750_BAUDRATE  100000

/* ------------------------------------------------------------------ */
/*  BH1750 constants                                                    */
/* ------------------------------------------------------------------ */

#define BH1750_POWER_ON   0x01
#define BH1750_CONT_HRES  0x10  /* Continuous high-resolution mode (1 lx) */

/* ------------------------------------------------------------------ */
/*  BH1750 driver                                                       */
/* ------------------------------------------------------------------ */

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

/* Read illuminance in lux. Returns -1 on error. */
static float bh1750_read_lux(void) {
    uint8_t buf[2];
    if (i2c_read_blocking(BH1750_I2C_BUS, BH1750_ADDR, buf, 2, false) < 0)
        return -1.0f;

    /* BH1750 returns 16-bit raw value, MSB first. Lux = raw / 1.2 */
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (float)raw / 1.2f;
}

/* ------------------------------------------------------------------ */
/*  Task                                                                */
/* ------------------------------------------------------------------ */

void vTaskBrake(void *params) {
    (void)params;

    if (!bh1750_init()) {
        printf("BH1750 init failed!\n");
        vTaskDelete(NULL);
        return;
    }

    printf("vTaskBrake started\n");

    /* Capture ambient light level at startup as baseline */
    vTaskDelay(pdMS_TO_TICKS(200));
    float ambient = bh1750_read_lux();
    if (ambient < 0.0f) ambient = 0.0f;
    printf("Ambient light: %.0f lux\n", ambient);

    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t print_counter = 0;
    for (;;) {
        float lux    = bh1750_read_lux();
        bool braking = false;
        xQueueOverwrite(xQueueLux, &lux);

        if (lux >= 0.0f)
            braking = (lux > ambient + BRAKE_LUX_THRESHOLD);

        if (++print_counter % 5 == 0) {
            printf("BH1750: lux=%.0f ambient=%.0f thr=%.0f brake=%s\n",
                   lux, ambient, ambient + BRAKE_LUX_THRESHOLD,
                   braking ? "YES" : "no");
        }

        xQueueOverwrite(xQueueBrake, &braking);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
}

/* ================================================================== */
/*  ARCHIVED — I2C manager approach (single shared bus via queue)      */
/*                                                                      */
/*  Kept for reference. Was abandoned because the OLED's bulk writes   */
/*  flooded the request queue, starving BH1750 reads and causing       */
/*  deadlocks even with timeouts and fire-and-forget variants.         */
/*  Solution: dedicate one I2C bus per device family.                  */
/* ================================================================== */

/*
#include "i2c_manager.h"

static QueueHandle_t xBrakeReply;

static bool i2c_transact(I2COpType op,
                         uint8_t   addr,
                         uint8_t  *tx,  uint8_t tx_len,
                         uint8_t   rx_len,
                         uint8_t  *rx_out)
{
    I2CRequest req = {
        .op          = op,
        .addr        = addr,
        .tx_len      = tx_len,
        .rx_len      = rx_len,
        .xReplyQueue = xBrakeReply,
    };
    if (tx && tx_len)
        memcpy(req.tx_buf, tx, tx_len);

    if (xQueueSendToBack(xQueueI2CRequest, &req, pdMS_TO_TICKS(50)) != pdTRUE) {
        printf("[Brake] I2C request queue full!\n");
        return false;
    }

    I2CReply reply;
    if (xQueueReceive(xBrakeReply, &reply, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("[Brake] I2C reply timeout!\n");
        return false;
    }

    if (rx_out && reply.ok)
        memcpy(rx_out, reply.rx_buf, rx_len);

    return reply.ok;
}

static bool bh1750_init(void) {
    uint8_t cmd = BH1750_POWER_ON;
    if (!i2c_transact(I2C_OP_WRITE, BH1750_ADDR, &cmd, 1, 0, NULL)) {
        printf("BH1750 power-on failed\n");
        return false;
    }
    cmd = BH1750_CONT_HRES;
    if (!i2c_transact(I2C_OP_WRITE, BH1750_ADDR, &cmd, 1, 0, NULL)) {
        printf("BH1750 mode set failed\n");
        return false;
    }
    printf("BH1750 initialized (addr=0x%02X)\n", BH1750_ADDR);
    return true;
}

static float bh1750_read_lux(void) {
    uint8_t buf[2] = {0};
    if (!i2c_transact(I2C_OP_READ, BH1750_ADDR, NULL, 0, 2, buf))
        return -1.0f;
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (float)raw / 1.2f;
}

void vTaskBrake(void *params) {
    (void)params;
    xBrakeReply = xQueueCreate(1, sizeof(I2CReply));
    if (!bh1750_init()) {
        printf("BH1750 init failed!\n");
        vTaskDelete(NULL);
        return;
    }
    printf("vTaskBrake started\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    float ambient = bh1750_read_lux();
    if (ambient < 0.0f) ambient = 0.0f;
    printf("Ambient light: %.0f lux\n", ambient);
    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t print_counter = 0;
    for (;;) {
        float lux    = bh1750_read_lux();
        bool braking = false;
        xQueueOverwrite(xQueueLux, &lux);
        if (lux >= 0.0f)
            braking = (lux > ambient + BRAKE_LUX_THRESHOLD);
        if (++print_counter % 5 == 0) {
            printf("BH1750: lux=%.0f ambient=%.0f thr=%.0f brake=%s\n",
                   lux, ambient, ambient + BRAKE_LUX_THRESHOLD,
                   braking ? "YES" : "no");
        }
        xQueueOverwrite(xQueueBrake, &braking);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
}
*/