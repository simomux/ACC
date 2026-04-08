#!/usr/bin/env python3
"""
mock_pico.py  –  Simulated Pico W telemetry publisher
======================================================
Publishes the same ROS 2 topics that the real Pico W firmware publishes,
so you can test the dashboard without hardware.

Topics published (mirroring acc_node.c):
  acc/distance      Float32  – median-filtered distance (cm)
  acc/distance_raw  Float32  – raw ultrasonic reading   (cm)
  acc/threshold     Float32  – brake threshold          (cm)
  acc/lux           Float32  – light sensor reading     (lux)
  acc/brake         Bool     – brake alert active

Scenarios (loop, ~120 s total):
  0 –  20 s  Idle at 200 cm, lux low (no car in front)
 20 –  50 s  Slow approach 200→40 cm, brake triggered near the end
 50 –  70 s  Object at 40 cm, occasional brake, lux spikes (brake light)
 70 – 100 s  Gradual departure 40→180 cm, brake clears
100 – 120 s  Back to idle

Run:
    conda activate ros2jazzy
    source ~/microros_jazzy_ws/install/local_setup.zsh
    python3 mock_pico.py
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool

PUBLISH_HZ  = 10        # match Pico firmware rate
THRESHOLD   = 50.0      # cm  (mirrors default in firmware)


class MockPico(Node):
    def __init__(self):
        super().__init__('mock_pico')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_dist     = self.create_publisher(Float32, 'acc/distance',     qos)
        self._pub_dist_raw = self.create_publisher(Float32, 'acc/distance_raw', qos)
        self._pub_thr      = self.create_publisher(Float32, 'acc/threshold',    qos)
        self._pub_lux      = self.create_publisher(Float32, 'acc/lux',          qos)
        self._pub_brake    = self.create_publisher(Bool,    'acc/brake',        qos)

        self._t0 = time.monotonic()
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f'Mock Pico started — publishing at {PUBLISH_HZ} Hz on acc/* topics'
        )

    # ── scenario ──────────────────────────────────────────────────────────────
    def _scenario(self, t: float):
        """Return (dist_filtered, dist_raw, lux, brake) for elapsed time t."""

        # ── segment 0-20 s: idle at ~200 cm ──────────────────────────────────
        if t < 20.0:
            base  = 200.0 + 8.0 * math.sin(t * 0.4)          # gentle sway
            noise = random.gauss(0, 2.5)
            lux   = random.uniform(20, 35)
            brake = False

        # ── segment 20-50 s: slow approach 200 → 40 cm ───────────────────────
        elif t < 50.0:
            frac  = (t - 20.0) / 30.0                          # 0→1
            base  = 200.0 - 160.0 * frac                       # 200→40
            base += 5.0 * math.sin(t * 1.2)                   # road oscillation
            noise = random.gauss(0, 3.0)
            # brake light comes on as we get close
            lux   = random.uniform(20, 35) + max(0, (frac - 0.8) * 200)
            brake = base < THRESHOLD and frac > 0.9

        # ── segment 50-70 s: close follow, intermittent braking ──────────────
        elif t < 70.0:
            base  = 42.0 + 6.0 * math.sin(t * 2.0)
            noise = random.gauss(0, 2.0)
            # periodic brake-light flashes (~every 4 s)
            lux   = 180.0 if math.sin(t * (2 * math.pi / 4.0)) > 0.85 else random.uniform(20, 40)
            brake = base < THRESHOLD

        # ── segment 70-100 s: gradual departure 40 → 180 cm ─────────────────
        elif t < 100.0:
            frac  = (t - 70.0) / 30.0
            base  = 40.0 + 140.0 * frac
            base += 4.0 * math.sin(t * 0.8)
            noise = random.gauss(0, 2.5)
            lux   = random.uniform(20, 35)
            brake = False

        # ── segment 100-120 s: back to idle ──────────────────────────────────
        else:
            t_wrapped = t % 120.0                              # loop scenario
            base  = 190.0 + 10.0 * math.sin(t_wrapped * 0.3)
            noise = random.gauss(0, 2.0)
            lux   = random.uniform(15, 30)
            brake = False

        dist_raw      = max(2.0, base + noise)
        dist_filtered = max(2.0, base)                         # "median" removes the noise
        return dist_filtered, dist_raw, lux, brake

    # ── publish ───────────────────────────────────────────────────────────────
    def _publish(self):
        t = (time.monotonic() - self._t0) % 120.0  # loop every 120 s

        dist_f, dist_r, lux, brake = self._scenario(t)

        def f32(v):
            m = Float32()
            m.data = float(v)
            return m

        self._pub_dist.publish(f32(dist_f))
        self._pub_dist_raw.publish(f32(dist_r))
        self._pub_thr.publish(f32(THRESHOLD))
        self._pub_lux.publish(f32(lux))

        b = Bool()
        b.data = bool(brake)
        self._pub_brake.publish(b)


def main():
    rclpy.init()
    node = MockPico()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
