#ifndef COMMON_H
#define COMMON_H

#include "FreeRTOS.h"
#include "queue.h"

/* HC-SR04 ultrasonic sensor */
#define SENSOR_TRIGGER_PIN  2   // GP2 - Physical 4
#define SENSOR_ECHO_PIN     3   // GP3 - Physical 5

/* Potentiometer (ADC) */
#define DIMMER_ADC_PIN      26  // GP26 - Physical 31
#define DIMMER_ADC_INPUT    0

/* RGB LED pins (common cathode — active HIGH) */
#define LED_R_PIN           13  // GP13 - Physical 17
#define LED_G_PIN           14  // GP14 - Physical 19
#define LED_B_PIN           15  // GP15 - Physical 20

/* Passive buzzer (PWM driven) */
#define BUZZER_PIN          16  // GP16 - Physical 21

/* Distance range for the dimmer (cm) */
#define THRESHOLD_MIN_CM    5
#define THRESHOLD_MAX_CM    200

/* Task periods (ms) */
#define SENSOR_PERIOD_MS    60
#define DIMMER_PERIOD_MS    200
#define ALERT_PERIOD_MS     100

/* Shared queues (mailbox style, depth 1) */
extern QueueHandle_t xQueueDistance;   /* float, cm */
extern QueueHandle_t xQueueThreshold;  /* float, cm */

#endif
