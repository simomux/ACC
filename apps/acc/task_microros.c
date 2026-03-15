#include <stdio.h>
#include <time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "task_microros.h"

/* Shared mailbox queues defined in main.c */
extern QueueHandle_t xQueueDistance;
extern QueueHandle_t xQueueDistanceRaw;
extern QueueHandle_t xQueueThreshold;
extern QueueHandle_t xQueueBrake;
extern QueueHandle_t xQueueLux;

/* ── clock_gettime stub ──────────────────────────────────────────────────────
 * libmicroros.a was compiled against POSIX and calls clock_gettime internally.
 * On bare metal there is no OS to provide it, so we implement it using the
 * Pico hardware timer (64-bit µs counter, never overflows in practice).
 * ─────────────────────────────────────────────────────────────────────────── */
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;
    uint64_t us = time_us_64();
    tp->tv_sec  = (time_t)(us / 1000000u);
    tp->tv_nsec = (long)((us % 1000000u) * 1000u);
    return 0;
}

/* ── USB CDC transport ───────────────────────────────────────────────────────
 * Uses the Pico SDK stdio USB layer (already initialised by stdio_init_all).
 * No external UART adapter needed — same cable as power.
 * IMPORTANT: remove all printf() calls from other tasks while micro-ROS is
 * running; stray bytes corrupt the binary framing.
 * ─────────────────────────────────────────────────────────────────────────── */
static bool usb_transport_open(struct uxrCustomTransport *t)  { (void)t; return true; }
static bool usb_transport_close(struct uxrCustomTransport *t) { (void)t; return true; }

static size_t usb_transport_write(struct uxrCustomTransport *t,
                                   const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t; (void)err;
    for (size_t i = 0; i < len; i++) putchar_raw((int)buf[i]);
    return len;
}

static size_t usb_transport_read(struct uxrCustomTransport *t,
                                  uint8_t *buf, size_t len, int timeout_ms, uint8_t *err)
{
    (void)t; (void)err;
    uint64_t deadline = time_us_64() + (uint64_t)timeout_ms * 1000u;
    size_t i = 0;
    while (i < len) {
        int remaining = (int)(deadline - time_us_64());
        if (remaining <= 0) break;
        int c = getchar_timeout_us((uint32_t)remaining);
        if (c == PICO_ERROR_TIMEOUT) break;
        buf[i++] = (uint8_t)c;
    }
    return i;
}

/* ── helpers ─────────────────────────────────────────────────────────────── */
#define RCCHECK(fn) if ((fn) != RCL_RET_OK) { return false; }

static bool microros_init(rcl_node_t      *node,
                           rclc_support_t  *support,
                           rcl_publisher_t *pub_dist,
                           rcl_publisher_t *pub_dist_raw,
                           rcl_publisher_t *pub_thr,
                           rcl_publisher_t *pub_lux,
                           rcl_publisher_t *pub_brake,
                           rcl_allocator_t *allocator)
{
    RCCHECK(rclc_support_init(support, 0, NULL, allocator));
    RCCHECK(rclc_node_init_default(node, "acc_node", "", support));

    RCCHECK(rclc_publisher_init_best_effort(pub_dist, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "acc/distance"));
    RCCHECK(rclc_publisher_init_best_effort(pub_dist_raw, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "acc/distance_raw"));
    RCCHECK(rclc_publisher_init_best_effort(pub_thr, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "acc/threshold"));
    RCCHECK(rclc_publisher_init_best_effort(pub_lux, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "acc/lux"));
    RCCHECK(rclc_publisher_init_best_effort(pub_brake, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "acc/brake"));

    return true;
}

static void microros_fini(rcl_node_t      *node,
                           rclc_support_t  *support,
                           rcl_publisher_t *pub_dist,
                           rcl_publisher_t *pub_dist_raw,
                           rcl_publisher_t *pub_thr,
                           rcl_publisher_t *pub_lux,
                           rcl_publisher_t *pub_brake)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    rcl_publisher_fini(pub_dist,     node);
    rcl_publisher_fini(pub_dist_raw, node);
    rcl_publisher_fini(pub_thr,      node);
    rcl_publisher_fini(pub_lux,      node);
    rcl_publisher_fini(pub_brake,    node);
    rcl_node_fini(node);
    rclc_support_fini(support);
#pragma GCC diagnostic pop
}

/* ── task ────────────────────────────────────────────────────────────────── */
void vTaskMicroROS(void *pvParameters)
{
    (void)pvParameters;

    rmw_uros_set_custom_transport(
        true, NULL,
        usb_transport_open,
        usb_transport_close,
        usb_transport_write,
        usb_transport_read
    );

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;
    rcl_node_t      node;
    rcl_publisher_t pub_dist, pub_dist_raw, pub_thr, pub_lux, pub_brake;

    std_msgs__msg__Float32 msg_dist, msg_dist_raw, msg_thr, msg_lux;
    std_msgs__msg__Bool    msg_brake;

    bool connected = false;
    TickType_t xLastPublish;

    for (;;) {
        if (!connected) {
            if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
                if (microros_init(&node, &support,
                                  &pub_dist, &pub_dist_raw,
                                  &pub_thr, &pub_lux, &pub_brake,
                                  &allocator)) {
                    connected = true;
                    xLastPublish = xTaskGetTickCount();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        float dist = 0.0f, dist_raw = 0.0f, thr = 0.0f, lux = 0.0f;
        bool  brake = false;

        xQueuePeek(xQueueDistance,    &dist,     0);
        xQueuePeek(xQueueDistanceRaw, &dist_raw, 0);
        xQueuePeek(xQueueThreshold,   &thr,      0);
        xQueuePeek(xQueueLux,         &lux,      0);
        xQueuePeek(xQueueBrake,       &brake,    0);

        msg_dist.data     = dist;
        msg_dist_raw.data = dist_raw;
        msg_thr.data      = thr;
        msg_lux.data      = lux;
        msg_brake.data    = brake;

        bool ok = true;
        if (rcl_publish(&pub_dist,     &msg_dist,     NULL) != RCL_RET_OK) ok = false;
        if (rcl_publish(&pub_dist_raw, &msg_dist_raw, NULL) != RCL_RET_OK) ok = false;
        if (rcl_publish(&pub_thr,      &msg_thr,      NULL) != RCL_RET_OK) ok = false;
        if (rcl_publish(&pub_lux,      &msg_lux,      NULL) != RCL_RET_OK) ok = false;
        if (rcl_publish(&pub_brake,    &msg_brake,    NULL) != RCL_RET_OK) ok = false;

        if (!ok) {
            microros_fini(&node, &support,
                          &pub_dist, &pub_dist_raw,
                          &pub_thr, &pub_lux, &pub_brake);
            connected = false;
            continue;
        }

        vTaskDelayUntil(&xLastPublish, pdMS_TO_TICKS(100));
    }
}
