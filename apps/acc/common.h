#ifndef COMMON_H
#define COMMON_H

#include "FreeRTOS.h"
#include "queue.h"

/* HC-SR04 ultrasonic sensor */
#define SENSOR_TRIGGER_PIN  2
#define SENSOR_ECHO_PIN     3

/* Potentiometer (ADC) */
#define DIMMER_ADC_PIN      26  // GP26
#define DIMMER_ADC_INPUT    0

/* RGB LED pins (common cathode — active HIGH) */
#define LED_R_PIN           13  // GP13
#define LED_G_PIN           14  // GP14
#define LED_B_PIN           15  // GP15

/* Distance range for the dimmer (cm) */
#define THRESHOLD_MIN_CM    5
#define THRESHOLD_MAX_CM    100

/* Task periods (ms) */
#define SENSOR_PERIOD_MS    60
#define DIMMER_PERIOD_MS    200
#define LED_PERIOD_MS       100

/* Shared queues (mailbox style, depth 1) */
extern QueueHandle_t xQueueDistance;   /* float, cm */
extern QueueHandle_t xQueueThreshold;  /* float, cm */

#endif
