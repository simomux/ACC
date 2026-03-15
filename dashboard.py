#!/usr/bin/env python3
"""
ACC Telemetry Dashboard
=======================
Subscribes to micro-ROS topics published by the Pico W and shows:
  - Distance: raw (pre-filter) vs median-filtered, with threshold line
  - Light sensor: lux over time with brake-event markers
  - Stats panel: outliers rejected, brake events, false positives,
                 filter noise reduction, alert level

Requirements:
    pip install matplotlib
    (rclpy comes with your ROS 2 installation)

Run:
    # Terminal 1 — micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

    # Terminal 2 — dashboard
    python3 dashboard.py
"""

import threading
import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# ── tunables ──────────────────────────────────────────────────────────────────
WINDOW_S        = 30        # seconds of history shown in graphs
HZ              = 10        # Pico publish rate
WINDOW_SAMPLES  = WINDOW_S * HZ
OUTLIER_THRESH  = 5.0       # cm delta between raw and filtered → counts as outlier
FALSE_POS_MAX_S = 0.4       # brake event shorter than this → false positive
# ─────────────────────────────────────────────────────────────────────────────

matplotlib.rcParams.update({
    'figure.facecolor': '#1a1a2e',
    'axes.facecolor':   '#16213e',
    'axes.edgecolor':   '#4a4a8a',
    'axes.labelcolor':  '#e0e0e0',
    'xtick.color':      '#a0a0c0',
    'ytick.color':      '#a0a0c0',
    'grid.color':       '#2a2a4a',
    'grid.linestyle':   '--',
    'grid.alpha':       0.5,
    'text.color':       '#e0e0e0',
    'lines.linewidth':  1.8,
})

# ── shared state (written by ROS thread, read by matplotlib thread) ───────────
lock = threading.Lock()

buf_dist_raw = deque([0.0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_dist     = deque([0.0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_thr      = deque([50.0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_lux      = deque([0.0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_brake    = deque([False] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)

stats = {
    'outliers':      0,
    'brake_events':  0,
    'false_pos':     0,
    'last_brake_on': None,   # timestamp of last rising edge
    'noise_sum':     0.0,
    'noise_count':   0,
    'samples':       0,
    'dist':          0.0,
    'dist_raw':      0.0,
    'thr':           50.0,
    'lux':           0.0,
    'brake':         False,
}

# ── ROS 2 node ────────────────────────────────────────────────────────────────
class AccSubscriber(Node):
    def __init__(self):
        super().__init__('acc_dashboard')
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Float32, 'acc/distance',     self._cb_dist,     qos)
        self.create_subscription(Float32, 'acc/distance_raw', self._cb_dist_raw, qos)
        self.create_subscription(Float32, 'acc/threshold',    self._cb_thr,      qos)
        self.create_subscription(Float32, 'acc/lux',          self._cb_lux,      qos)
        self.create_subscription(Bool,    'acc/brake',        self._cb_brake,    qos)

    def _cb_dist(self, msg):
        with lock:
            buf_dist.append(msg.data)
            stats['dist'] = msg.data
            # outlier detection vs latest raw
            delta = abs(stats['dist_raw'] - msg.data)
            if delta > OUTLIER_THRESH:
                stats['outliers'] += 1
            # noise accumulator
            stats['noise_sum']   += delta
            stats['noise_count'] += 1
            stats['samples']     += 1

    def _cb_dist_raw(self, msg):
        with lock:
            buf_dist_raw.append(msg.data)
            stats['dist_raw'] = msg.data

    def _cb_thr(self, msg):
        with lock:
            buf_thr.append(msg.data)
            stats['thr'] = msg.data

    def _cb_lux(self, msg):
        with lock:
            buf_lux.append(msg.data)
            stats['lux'] = msg.data

    def _cb_brake(self, msg):
        with lock:
            prev = buf_brake[-1] if buf_brake else False
            buf_brake.append(msg.data)
            stats['brake'] = msg.data
            now = self.get_clock().now().nanoseconds * 1e-9

            if msg.data and not prev:
                # rising edge → new brake event
                stats['brake_events'] += 1
                stats['last_brake_on'] = now

            if not msg.data and prev and stats['last_brake_on'] is not None:
                # falling edge → check duration
                duration = now - stats['last_brake_on']
                if duration < FALSE_POS_MAX_S:
                    stats['false_pos'] += 1
                stats['last_brake_on'] = None


def _ros_spin():
    rclpy.init()
    node = AccSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── matplotlib layout ─────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 7), constrained_layout=True)
fig.patch.set_facecolor('#1a1a2e')
fig.suptitle('ACC Telemetry  ·  Pico W / FreeRTOS / micro-ROS',
             color='#c0c0ff', fontsize=13, fontweight='bold')

gs = gridspec.GridSpec(2, 2, figure=fig, width_ratios=[3, 1.1], hspace=0.35)

ax_dist  = fig.add_subplot(gs[0, 0])
ax_lux   = fig.add_subplot(gs[1, 0])
ax_stats = fig.add_subplot(gs[:, 1])

for ax in (ax_dist, ax_lux):
    ax.grid(True)

ax_stats.axis('off')

# distance plot
ax_dist.set_title('Distance  —  raw vs median-filtered', color='#c0c0ff', fontsize=10)
ax_dist.set_ylabel('cm')
ax_dist.set_xlim(0, WINDOW_SAMPLES)
ax_dist.set_ylim(0, 250)

line_raw,      = ax_dist.plot([], [], color='#ff6b6b', alpha=0.55, lw=1.2, label='raw')
line_filtered, = ax_dist.plot([], [], color='#4ecdc4', lw=2.0,    label='filtered')
line_thr,      = ax_dist.plot([], [], color='#ffe66d', lw=1.5, ls='--', label='threshold')
ax_dist.legend(loc='upper right', fontsize=8,
               facecolor='#1a1a2e', edgecolor='#4a4a8a', labelcolor='#e0e0e0')

# zone fill (redrawn each frame)
fill_danger = None
fill_warn   = None

# lux plot
ax_lux.set_title('Light Sensor  —  lux  (● brake events)', color='#c0c0ff', fontsize=10)
ax_lux.set_ylabel('lux')
ax_lux.set_xlim(0, WINDOW_SAMPLES)
ax_lux.set_ylim(0, 1)      # auto-scaled each frame

line_lux,    = ax_lux.plot([], [], color='#ffd93d', lw=1.8, label='lux')
scat_brake   = ax_lux.scatter([], [], color='#ff4757', s=40, zorder=5, label='brake')
ax_lux.legend(loc='upper right', fontsize=8,
              facecolor='#1a1a2e', edgecolor='#4a4a8a', labelcolor='#e0e0e0')

# stats panel
stats_text = ax_stats.text(
    0.05, 0.97, '', transform=ax_stats.transAxes,
    va='top', ha='left', fontsize=9.5,
    fontfamily='monospace', color='#e0e0e0',
    linespacing=1.8,
)

x_data = list(range(WINDOW_SAMPLES))


def _alert_level(dist, thr):
    """Return (label, colour) for the current alert state."""
    if dist > thr * 1.3 + 20:
        return 'SAFE',         '#2ecc71'
    if dist > thr:
        return 'APPROACHING',  '#f39c12'
    return     'CRITICAL',     '#e74c3c'


def _noise_reduction(s):
    if s['noise_count'] == 0 or s['dist_raw'] == 0:
        return 0.0
    avg_noise = s['noise_sum'] / s['noise_count']
    # express as % of the average raw value (rough estimate)
    return min(avg_noise / max(s['dist_raw'], 1.0) * 100.0, 99.9)


def animate(_frame):
    global fill_danger, fill_warn

    with lock:
        d_raw  = list(buf_dist_raw)
        d_filt = list(buf_dist)
        d_thr  = list(buf_thr)
        d_lux  = list(buf_lux)
        d_brk  = list(buf_brake)
        s      = dict(stats)

    # ── distance plot ─────────────────────────────────────────────────────
    line_raw.set_data(x_data, d_raw)
    line_filtered.set_data(x_data, d_filt)
    line_thr.set_data(x_data, d_thr)

    # coloured zone between filtered line and threshold
    if fill_danger:
        fill_danger.remove()
    if fill_warn:
        fill_warn.remove()

    thr_val = d_thr[-1] if d_thr else 50.0
    fill_danger = ax_dist.fill_between(x_data, 0, d_filt,
                                       where=[v < thr_val for v in d_filt],
                                       color='#e74c3c', alpha=0.15)
    fill_warn   = ax_dist.fill_between(x_data, 0, d_filt,
                                       where=[thr_val <= v <= thr_val * 1.3 + 20
                                              for v in d_filt],
                                       color='#f39c12', alpha=0.10)

    ax_dist.set_ylim(0, max(max(d_raw + [0]) * 1.15, thr_val * 1.5, 60))

    # ── lux plot ──────────────────────────────────────────────────────────
    line_lux.set_data(x_data, d_lux)

    brk_x = [i for i, b in enumerate(d_brk) if b]
    brk_y = [d_lux[i] for i in brk_x]
    scat_brake.set_offsets(list(zip(brk_x, brk_y)) if brk_x else [[0, 0]])
    scat_brake.set_sizes([40] * len(brk_x) if brk_x else [0])

    lux_max = max(max(d_lux + [1]) * 1.2, 10)
    ax_lux.set_ylim(0, lux_max)

    # ── stats panel ───────────────────────────────────────────────────────
    level, level_color = _alert_level(s['dist'], s['thr'])
    noise_pct = _noise_reduction(s)

    bar_len  = 14
    filled   = int(noise_pct / 100 * bar_len)
    bar      = '█' * filled + '░' * (bar_len - filled)

    txt = (
        f"┌─ Live Values ──────────────┐\n"
        f"│ Distance  {s['dist']:>7.1f} cm       │\n"
        f"│ Raw       {s['dist_raw']:>7.1f} cm       │\n"
        f"│ Threshold {s['thr']:>7.1f} cm       │\n"
        f"│ Lux       {s['lux']:>7.0f} lx       │\n"
        f"│ Brake     {'  YES ●' if s['brake'] else '   no  '}         │\n"
        f"└────────────────────────────┘\n"
        f"\n"
        f"┌─ Filter Stats ─────────────┐\n"
        f"│ Outliers rejected  {s['outliers']:>5d}   │\n"
        f"│ Noise reduction          │\n"
        f"│  [{bar}]  │\n"
        f"│  {noise_pct:>5.1f} %                  │\n"
        f"└────────────────────────────┘\n"
        f"\n"
        f"┌─ Brake Detection ──────────┐\n"
        f"│ Events detected    {s['brake_events']:>5d}   │\n"
        f"│ False positives    {s['false_pos']:>5d}   │\n"
        f"└────────────────────────────┘\n"
        f"\n"
        f"┌─ Alert Level ──────────────┐\n"
        f"│  {level:<27}│\n"
        f"└────────────────────────────┘\n"
        f"\n"
        f" Samples received: {s['samples']:>6d}"
    )

    stats_text.set_text(txt)
    stats_text.set_color('#e0e0e0')

    # colour the alert level line
    lines = txt.split('\n')
    # re-draw just the level label in colour — easier via a second text object
    # (we use a simple overlay approach)
    ax_stats.texts[1].remove() if len(ax_stats.texts) > 1 else None
    level_y = 0.97 - (txt[:txt.find(level)].count('\n')) * 0.038
    ax_stats.text(0.09, level_y, level,
                  transform=ax_stats.transAxes,
                  va='top', ha='left', fontsize=9.5,
                  fontfamily='monospace', color=level_color, fontweight='bold')

    return line_raw, line_filtered, line_thr, line_lux, scat_brake, stats_text


if __name__ == '__main__':
    ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    ros_thread.start()

    ani = FuncAnimation(fig, animate, interval=100, blit=False, cache_frame_data=False)
    plt.show()
