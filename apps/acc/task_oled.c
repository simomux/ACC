#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "u8g2.h"

/* ------------------------------------------------------------------ */
/*  OLED I2C bus — dedicated i2c1, independent from BH1750             */
/* ------------------------------------------------------------------ */

#define OLED_BAUDRATE       100000
#define OLED_I2C_TIMEOUT_MS 50      /* max ms per I2C write before bus reset */

/* Refresh rate cap: one frame every N ms                              */
#define OLED_REFRESH_MS 500

static u8g2_t u8g2;
static uint32_t frame_count = 0;

/* ------------------------------------------------------------------ */
/*  Bus reset — low-level only, no u8g2 calls (safe to call from       */
/*  inside the u8g2 callback without recursion risk)                   */
/* ------------------------------------------------------------------ */

static void oled_bus_reset(void)
{
    i2c_deinit(OLED_I2C_BUS);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_init(OLED_I2C_BUS, OLED_BAUDRATE);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
}

/* ------------------------------------------------------------------ */
/*  I2C write with timeout + automatic bus + u8g2 reset on any error.  */
/*  in_recovery flag breaks the recursion:                             */
/*    oled_i2c_write → u8g2_InitDisplay → callback → oled_i2c_write   */
/* ------------------------------------------------------------------ */

static void oled_i2c_write(const uint8_t *buf, uint8_t len)
{
    static bool in_recovery = false;

    absolute_time_t timeout = make_timeout_time_ms(OLED_I2C_TIMEOUT_MS);
    int ret = i2c_write_blocking_until(OLED_I2C_BUS, OLED_ADDR,
                                       buf, len, false, timeout);
    if (ret < 0 && !in_recovery) {
        in_recovery = true;
        printf("[OLED] I2C error %d — resetting bus\n", ret);
        oled_bus_reset();
        /* Re-init display so its internal state matches the bus reset.
           Calls back into u8g2 callbacks — in_recovery prevents re-entry. */
        u8g2_InitDisplay(&u8g2);
        u8g2_SetPowerSave(&u8g2, 0);
        in_recovery = false;
    }
}

/* ------------------------------------------------------------------ */
/*  u8g2 I2C byte callback                                              */
/* ------------------------------------------------------------------ */

uint8_t u8x8_byte_pico_i2c(u8x8_t *u8x8,
                            uint8_t  msg,
                            uint8_t  arg_int,
                            void    *arg_ptr)
{
    static uint8_t buffer[32];
    static uint8_t buf_idx;

    switch (msg)
    {
        case U8X8_MSG_BYTE_INIT:
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;

        case U8X8_MSG_BYTE_SEND:
        {
            uint8_t *data = (uint8_t *)arg_ptr;
            while (arg_int--) {
                buffer[buf_idx++] = *data++;
                if (buf_idx == sizeof(buffer)) {
                    oled_i2c_write(buffer, buf_idx);
                    buf_idx = 0;  /* always reset after flush */
                }
            }
            break;
        }

        case U8X8_MSG_BYTE_END_TRANSFER:
            if (buf_idx > 0)
                oled_i2c_write(buffer, buf_idx);
            buf_idx = 0;  /* always reset, even if write failed */
            break;

        default:
            return 0;
    }

    return 1;
}

/* ------------------------------------------------------------------ */
/*  u8g2 delay / GPIO callback                                          */
/* ------------------------------------------------------------------ */

uint8_t u8x8_gpio_and_delay_pico(u8x8_t *u8x8,
                                  uint8_t  msg,
                                  uint8_t  arg_int,
                                  void    *arg_ptr)
{
    switch (msg)
    {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;

        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
    }

    return 1;
}

/* ------------------------------------------------------------------ */
/*  OLED init                                                           */
/* ------------------------------------------------------------------ */

static void oled_init(void)
{
    printf("[OLED] init start\n");

    i2c_init(OLED_I2C_BUS, OLED_BAUDRATE);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    printf("[OLED] i2c init done (bus=%s sda=%d scl=%d baud=%d)\n",
           (OLED_I2C_BUS == i2c0) ? "i2c0" : "i2c1",
           OLED_SDA_PIN, OLED_SCL_PIN, OLED_BAUDRATE);

    u8g2_Setup_sh1106_i2c_128x64_noname_1(
        &u8g2,
        U8G2_R0,
        u8x8_byte_pico_i2c,
        u8x8_gpio_and_delay_pico
    );
    printf("[OLED] u8g2 setup done\n");

    u8g2_SetI2CAddress(&u8g2, OLED_ADDR << 1);
    printf("[OLED] calling InitDisplay...\n");
    u8g2_InitDisplay(&u8g2);
    printf("[OLED] InitDisplay done\n");

    u8g2_SetPowerSave(&u8g2, 0);
    printf("[OLED] PowerSave off done\n");

    printf("[OLED] ready\n");
}

/* ------------------------------------------------------------------ */
/*  Task                                                                */
/* ------------------------------------------------------------------ */

void vTaskOled(void *pvParameters)
{
    (void)pvParameters;

    printf("[OLED] task started, stack hwm: %lu words\n",
           (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    oled_init();

    printf("[OLED] entering loop\n");

    float distance  = 0.0f;
    float threshold = 0.0f;
    float lux       = 0.0f;
    bool  brake     = false;
    char  buf[32];

    TickType_t xLastWake = xTaskGetTickCount();

    while (1)
    {
        printf("[OLED] frame #%lu start\n", (unsigned long)frame_count);


        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);
        xQueuePeek(xQueueBrake,     &brake,     0);
        xQueuePeek(xQueueLux,       &lux,       0);
        vTaskDelay(pdMS_TO_TICKS(5)); // Attesa di 5ms dopo la lettura del BH1750

        u8g2_FirstPage(&u8g2);
        do {

            u8g2_SetFont(&u8g2, u8g2_font_6x13_tr);

            int line_height  = u8g2_GetAscent(&u8g2) - u8g2_GetDescent(&u8g2);
            int total_height = line_height * 4;
            int y            = (64 - total_height) / 2 + line_height;

            snprintf(buf, sizeof(buf), "Dist: %.1f cm", distance);
            u8g2_DrawStr(&u8g2, 2, y, buf);

            y += line_height;
            snprintf(buf, sizeof(buf), "Thr: %.1f cm", threshold);
            u8g2_DrawStr(&u8g2, 2, y, buf);

            y += line_height;
            snprintf(buf, sizeof(buf), "Lux: %.0f", lux);
            u8g2_DrawStr(&u8g2, 2, y, buf);

            y += line_height;
            snprintf(buf, sizeof(buf), "Brake: %s", brake ? "TRUE" : "FALSE");
            u8g2_DrawStr(&u8g2, 2, y, buf);

            u8g2_DrawFrame(&u8g2, 0, 0, 128, 64);

            printf("[OLED] calling SendBuffer...\n");
        
        } while (u8g2_NextPage(&u8g2));
        printf("[OLED] SendBuffer done\n");

        frame_count++;
        printf("[OLED] frame #%lu complete, stack hwm: %lu words\n",
               (unsigned long)(frame_count - 1),
               (unsigned long)uxTaskGetStackHighWaterMark(NULL));

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(OLED_REFRESH_MS));
        printf("[OLED] delay done\n");
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

static QueueHandle_t xOledReply;

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
        .xReplyQueue = xOledReply,
    };
    if (tx && tx_len)
        memcpy(req.tx_buf, tx, tx_len);

    if (xQueueSendToBack(xQueueI2CRequest, &req, pdMS_TO_TICKS(50)) != pdTRUE) {
        printf("[OLED] I2C request queue full!\n");
        return false;
    }

    I2CReply reply;
    if (xQueueReceive(xOledReply, &reply, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("[OLED] I2C reply timeout!\n");
        return false;
    }

    if (rx_out && reply.ok)
        memcpy(rx_out, reply.rx_buf, rx_len);

    return reply.ok;
}

// u8x8_byte_pico_i2c callback (manager version):
// called i2c_transact(I2C_OP_WRITE, OLED_ADDR, buffer, buf_idx, 0, NULL)
// on each flush — flooded the queue with ~30 requests per SendBuffer.

// In vTaskOled:
// xOledReply = xQueueCreate(1, sizeof(I2CReply));
// oled_init() called without i2c_init() (owned by manager)
*/