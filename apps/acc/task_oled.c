#include "common.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "u8g2.h"
#include <stdio.h>

#define OLED_I2C i2c0
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 5
#define OLED_ADDR 0x3C

static u8g2_t u8g2;



/* ---------------- I2C callback ---------------- */

uint8_t u8x8_byte_pico_i2c(
        u8x8_t *u8x8,
        uint8_t msg,
        uint8_t arg_int,
        void *arg_ptr)
{
    static uint8_t buffer[32];
    static uint8_t buf_idx;

    switch(msg)
    {
        case U8X8_MSG_BYTE_INIT:
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;

        case U8X8_MSG_BYTE_SEND:
        {
            uint8_t *data = (uint8_t*)arg_ptr;

            while(arg_int--)
            {
                buffer[buf_idx++] = *data++;

                if(buf_idx == sizeof(buffer))
                {
                    i2c_write_blocking(OLED_I2C, OLED_ADDR, buffer, buf_idx, false);
                    buf_idx = 0;
                }
            }
        }
        break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            if(buf_idx > 0)
            {
                i2c_write_blocking(OLED_I2C, OLED_ADDR, buffer, buf_idx, false);
            }
            break;

        default:
            return 0;
    }

    return 1;
}





/* ---------------- delay callback ---------------- */

uint8_t u8x8_gpio_and_delay_pico(
        u8x8_t *u8x8,
        uint8_t msg,
        uint8_t arg_int,
        void *arg_ptr)
{
    switch(msg)
    {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;

        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
    }

    return 1;
}





/* ---------------- OLED INIT ---------------- */

static void oled_init()
{
    printf("[OLED] init\n");

    i2c_init(OLED_I2C, 400000);

    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);


    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_pico_i2c,
        u8x8_gpio_and_delay_pico
    );

    u8g2_SetI2CAddress(&u8g2, OLED_ADDR << 1);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    printf("[OLED] ready\n");
}





/* ---------------- TASK ---------------- */

void vTaskOled(void *pvParameters)
{
    oled_init();

    float distance = 0.0f;
    float threshold = 0.0f;
    float lux = 0.0f;
    bool brake = false;
    char buf[32];

    while (1)
    {
        // Leggi valori dalle queue (non blocca)
        xQueuePeek(xQueueDistance,  &distance,  0);
        xQueuePeek(xQueueThreshold, &threshold, 0);
        xQueuePeek(xQueueBrake,     &brake,     0);

        // Per la luminosità: puoi aggiornare questa variabile da un task, oppure qui metti un valore dummy o condividi una queue se vuoi il valore reale
        // lux = ...

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_6x13_tr); // Font più piccolo

        snprintf(buf, sizeof(buf), "Dist: %.1f cm", distance);
        u8g2_DrawStr(&u8g2, 2, 10, buf); // Più in alto

        snprintf(buf, sizeof(buf), "Thr:  %.1f cm", threshold);
        u8g2_DrawStr(&u8g2, 2, 24, buf);

        snprintf(buf, sizeof(buf), "Lux:  %.0f", lux);
        u8g2_DrawStr(&u8g2, 2, 38, buf);

        snprintf(buf, sizeof(buf), "Brake: %s", brake ? "TRUE" : "FALSE");
        u8g2_DrawStr(&u8g2, 2, 52, buf);

        u8g2_DrawFrame(&u8g2, 0, 0, 128, 64);
        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(300)); // Aggiorna meno spesso
    }
}