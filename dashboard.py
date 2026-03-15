#!/usr/bin/env python3
"""
ACC Telemetry Dashboard
=======================
Subscribes to micro-ROS topics published by the Pico W and shows:
  - Distance: raw (pre-filter) vs median-filtered, with threshold line
  - Relative velocity (EMA-smoothed derivative of filtered distance)
  - Light sensor: lux over time with brake-event markers
  - Stats panel: outliers rejected, brake events, false positives,
                 filter noise reduction, relative velocity, TTC, alert level

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
import time
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
V_EMA_ALPHA     = 0.15      # EMA smoothing factor for relative velocity (lower = smoother)
V_NOISE_FLOOR   = 1.0       # cm/s — below this |v| is treated as zero for TTC
HYST_MARGIN_CM  = 10.0      # cm extra buffer to exit a worse zone (spatial hysteresis)
HYST_FRAMES     = 3         # consecutive frames needed to confirm a degradation
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
buf_vel      = deque([0.0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)

stats = {
    'outliers':      0,
    'brake_events':  0,
    'false_pos':     0,
    'last_brake_on': None,
    'noise_sum':     0.0,
    'noise_count':   0,
    'samples':       0,
    'dist':          0.0,
    'dist_raw':      0.0,
    'thr':           50.0,
    'lux':           0.0,
    'brake':         False,
    'v_rel':         0.0,   # EMA-smoothed relative velocity (cm/s, negative = approaching)
    'ttc':           float('inf'),  # Time To Collision (s)
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
        self._prev_dist = None
        self._prev_time = None

    def _cb_dist(self, msg):
        with lock:
            buf_dist.append(msg.data)
            stats['dist'] = msg.data

            # outlier detection vs latest raw
            delta = abs(stats['dist_raw'] - msg.data)
            if delta > OUTLIER_THRESH:
                stats['outliers'] += 1
            stats['noise_sum']   += delta
            stats['noise_count'] += 1
            stats['samples']     += 1

            # ── relative velocity (EMA of discrete derivative) ────────────
            now = time.monotonic()
            if self._prev_dist is not None:
                dt = now - self._prev_time
                if dt >= 0.01:   # guard against duplicate/fast callbacks
                    v_raw = (msg.data - self._prev_dist) / dt   # cm/s
                    stats['v_rel'] = (V_EMA_ALPHA * v_raw
                                      + (1.0 - V_EMA_ALPHA) * stats['v_rel'])
            self._prev_dist = msg.data
            self._prev_time = now
            buf_vel.append(stats['v_rel'])

            # ── TTC: meaningful only when approaching (v_rel < -noise_floor)
            v = stats['v_rel']
            stats['ttc'] = (-msg.data / v) if v < -V_NOISE_FLOOR else float('inf')

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
                stats['brake_events'] += 1
                stats['last_brake_on'] = now

            if not msg.data and prev and stats['last_brake_on'] is not None:
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

# distance plot (primary axis — cm)
ax_dist.set_title('Distance  —  raw vs filtered  |  relative velocity',
                   color='#c0c0ff', fontsize=10)
ax_dist.set_ylabel('cm')
ax_dist.set_xlim(0, WINDOW_SAMPLES)
ax_dist.set_ylim(0, 250)

line_raw,      = ax_dist.plot([], [], color='#ff6b6b', alpha=0.55, lw=1.2, label='raw')
line_filtered, = ax_dist.plot([], [], color='#4ecdc4', lw=2.0,    label='filtered')
line_thr,      = ax_dist.plot([], [], color='#ffe66d', lw=1.5, ls='--', label='threshold')

# velocity secondary axis (cm/s) — shares x with ax_dist
ax_vel = ax_dist.twinx()
ax_vel.set_ylabel('cm/s', color='#c39bd3', fontsize=8)
ax_vel.tick_params(axis='y', colors='#c39bd3', labelsize=7)
ax_vel.spines['right'].set_color('#4a4a8a')
line_vel, = ax_vel.plot([], [], color='#c39bd3', lw=1.2, ls=':', alpha=0.85, label='vel.')

# combined legend
handles = [line_raw, line_filtered, line_thr, line_vel]
ax_dist.legend(handles=handles, loc='upper right', fontsize=8,
               facecolor='#1a1a2e', edgecolor='#4a4a8a', labelcolor='#e0e0e0')

# zone fill (redrawn each frame)
fill_danger = None
fill_warn   = None

# lux plot
ax_lux.set_title('Light Sensor  —  lux  (● brake events)', color='#c0c0ff', fontsize=10)
ax_lux.set_ylabel('lux')
ax_lux.set_xlim(0, WINDOW_SAMPLES)
ax_lux.set_ylim(0, 1)

line_lux,  = ax_lux.plot([], [], color='#ffd93d', lw=1.8, label='lux')
scat_brake = ax_lux.scatter([], [], color='#ff4757', s=40, zorder=5, label='brake')
ax_lux.legend(loc='upper right', fontsize=8,
              facecolor='#1a1a2e', edgecolor='#4a4a8a', labelcolor='#e0e0e0')

# ── stats panel — one text object per row, exact y coordinates ────────────────
_LC = '#8888cc'   # label colour
_VC = '#e0e0e0'   # value colour
_HC = '#a0a0ff'   # section header colour
_FS = 9.0
_FH = 8.5


def _lbl(y, text, color=_LC, size=_FS, bold=False):
    return ax_stats.text(0.04, y, text, transform=ax_stats.transAxes,
                         va='top', ha='left', fontsize=size, color=color,
                         fontweight='bold' if bold else 'normal')


def _val(y):
    return ax_stats.text(0.96, y, '', transform=ax_stats.transAxes,
                         va='top', ha='right', fontsize=_FS, color=_VC,
                         fontweight='bold', fontfamily='monospace')


def _sep(y):
    ax_stats.axhline(y, xmin=0.04, xmax=0.96, color='#3a3a6a', lw=0.8)


# Live Values (compact 0.045 spacing to fit 7 rows)
_lbl(0.97, 'LIVE VALUES', color=_HC, size=_FH, bold=True)
tv_dist  = _val(0.915); _lbl(0.915, 'Distance')
tv_raw   = _val(0.870); _lbl(0.870, 'Raw')
tv_thr   = _val(0.825); _lbl(0.825, 'Threshold')
tv_lux   = _val(0.780); _lbl(0.780, 'Lux')
tv_brake = _val(0.735); _lbl(0.735, 'Brake')
tv_vrel  = _val(0.690); _lbl(0.690, 'Vel. relative')
tv_ttc   = _val(0.645); _lbl(0.645, 'TTC')

# Filter Stats
_sep(0.610)
_lbl(0.600, 'FILTER STATS', color=_HC, size=_FH, bold=True)
tv_out       = _val(0.545); _lbl(0.545, 'Outliers rejected')
tv_noise_pct = _val(0.500); _lbl(0.500, 'Noise reduction')
tv_noise_bar = ax_stats.text(0.04, 0.460, '', transform=ax_stats.transAxes,
                              va='top', ha='left', fontsize=_FS,
                              color='#4ecdc4', fontfamily='monospace')

# Brake Detection
_sep(0.420)
_lbl(0.410, 'BRAKE DETECTION', color=_HC, size=_FH, bold=True)
tv_events = _val(0.355); _lbl(0.355, 'Events detected')
tv_fpos   = _val(0.310); _lbl(0.310, 'False positives')

# Alert Level
_sep(0.270)
_lbl(0.260, 'ALERT LEVEL', color=_HC, size=_FH, bold=True)
alert_patch = mpatches.FancyBboxPatch(
    (0.04, 0.130), 0.92, 0.115,
    boxstyle='round,pad=0.02', transform=ax_stats.transAxes,
    facecolor='#2ecc71', edgecolor='none', zorder=3, clip_on=False,
)
ax_stats.add_patch(alert_patch)
alert_text = ax_stats.text(
    0.5, 0.1875, 'SAFE', transform=ax_stats.transAxes,
    va='center', ha='center', fontsize=11, fontweight='bold',
    color='white', zorder=4,
)

# Samples
_sep(0.085)
tv_samples = _val(0.070); _lbl(0.070, 'Samples received')

x_data = list(range(WINDOW_SAMPLES))


_LEVEL_COLORS = {'SAFE': '#2ecc71', 'APPROACHING': '#f39c12', 'CRITICAL': '#e74c3c'}
_LEVEL_RANK   = {'SAFE': 0, 'APPROACHING': 1, 'CRITICAL': 2}
_alert_state   = 'SAFE'
_worse_counter = 0


def _alert_level(dist, thr, brake):
    """Stateful alert level with spatial hysteresis + temporal debounce.

    Degradation (SAFE→APPROACHING→CRITICAL) requires HYST_FRAMES consecutive
    frames inside the worse zone before committing.  Improvement is immediate
    but needs the distance to clear the entry threshold by HYST_MARGIN_CM
    (asymmetric thresholds prevent oscillation at zone boundaries).
    Brake always forces CRITICAL instantly regardless of distance.
    """
    global _alert_state, _worse_counter

    if brake:
        _alert_state = 'CRITICAL'
        _worse_counter = 0
        return _alert_state, _LEVEL_COLORS[_alert_state]

    # Entry thresholds (to get worse) — same as before
    entry_approaching = thr * 1.3 + 20   # dist drops below → APPROACHING
    entry_critical    = thr              # dist drops below → CRITICAL

    # Exit thresholds (to get better) — entry + hysteresis margin
    exit_approaching  = entry_approaching + HYST_MARGIN_CM
    exit_critical     = entry_critical    + HYST_MARGIN_CM

    # Determine target zone based on current state and asymmetric thresholds
    if _alert_state == 'SAFE':
        target = 'APPROACHING' if dist <= entry_approaching else 'SAFE'

    elif _alert_state == 'APPROACHING':
        if dist <= entry_critical:
            target = 'CRITICAL'
        elif dist > exit_approaching:
            target = 'SAFE'
        else:
            target = 'APPROACHING'

    else:  # CRITICAL
        target = 'APPROACHING' if dist > exit_critical else 'CRITICAL'

    # Temporal debounce: only apply delay when degrading
    if _LEVEL_RANK[target] > _LEVEL_RANK[_alert_state]:
        _worse_counter += 1
        if _worse_counter < HYST_FRAMES:
            return _alert_state, _LEVEL_COLORS[_alert_state]  # hold current
        _alert_state   = target
        _worse_counter = 0
    else:
        _alert_state   = target   # improvements are immediate
        _worse_counter = 0

    return _alert_state, _LEVEL_COLORS[_alert_state]


def _ttc_color(ttc):
    """Colour-code TTC: green > 3 s, orange 1–3 s, red < 1 s."""
    if ttc == float('inf') or ttc > 3.0:
        return '#2ecc71'
    if ttc > 1.0:
        return '#f39c12'
    return '#e74c3c'


def _noise_reduction(s):
    if s['noise_count'] == 0 or s['dist_raw'] == 0:
        return 0.0
    avg_noise = s['noise_sum'] / s['noise_count']
    return min(avg_noise / max(s['dist_raw'], 1.0) * 100.0, 99.9)


def animate(_frame):
    global fill_danger, fill_warn

    with lock:
        d_raw  = list(buf_dist_raw)
        d_filt = list(buf_dist)
        d_thr  = list(buf_thr)
        d_lux  = list(buf_lux)
        d_brk  = list(buf_brake)
        d_vel  = list(buf_vel)
        s      = dict(stats)

    # ── distance plot ─────────────────────────────────────────────────────
    line_raw.set_data(x_data, d_raw)
    line_filtered.set_data(x_data, d_filt)
    line_thr.set_data(x_data, d_thr)

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

    # ── velocity secondary axis ────────────────────────────────────────────
    line_vel.set_data(x_data, d_vel)
    vel_abs_max = max(max(abs(v) for v in d_vel), 10.0)
    ax_vel.set_ylim(-vel_abs_max * 1.2, vel_abs_max * 1.2)
    # zero reference line (drawn once would persist; use axhline on ax_vel)
    ax_vel.axhline(0, color='#c39bd3', lw=0.4, alpha=0.4)

    # ── lux plot ──────────────────────────────────────────────────────────
    line_lux.set_data(x_data, d_lux)

    brk_x = [i for i, b in enumerate(d_brk) if b]
    brk_y = [d_lux[i] for i in brk_x]
    scat_brake.set_offsets(list(zip(brk_x, brk_y)) if brk_x else [[0, 0]])
    scat_brake.set_sizes([40] * len(brk_x) if brk_x else [0])

    lux_max = max(max(d_lux + [1]) * 1.2, 10)
    ax_lux.set_ylim(0, lux_max)

    # ── stats panel ───────────────────────────────────────────────────────
    level, level_color = _alert_level(s['dist'], s['thr'], s['brake'])
    noise_pct = _noise_reduction(s)

    tv_dist.set_text(f"{s['dist']:.1f} cm")
    tv_raw.set_text(f"{s['dist_raw']:.1f} cm")
    tv_thr.set_text(f"{s['thr']:.1f} cm")
    tv_lux.set_text(f"{s['lux']:.0f} lx")
    tv_brake.set_text('YES ●' if s['brake'] else 'no')
    tv_brake.set_color('#ff4757' if s['brake'] else _VC)

    # relative velocity: green = moving away, red = approaching
    v = s['v_rel']
    tv_vrel.set_text(f"{v:+.1f} cm/s")
    tv_vrel.set_color('#2ecc71' if v >= 0 else '#e74c3c')

    # TTC
    ttc = s['ttc']
    tv_ttc.set_text(f"{ttc:.1f} s" if ttc != float('inf') else '∞')
    tv_ttc.set_color(_ttc_color(ttc))

    tv_out.set_text(str(s['outliers']))
    bar_len = 14
    filled  = int(noise_pct / 100 * bar_len)
    tv_noise_bar.set_text('█' * filled + '░' * (bar_len - filled))
    tv_noise_pct.set_text(f"{noise_pct:.1f} %")

    tv_events.set_text(str(s['brake_events']))
    tv_fpos.set_text(str(s['false_pos']))

    alert_patch.set_facecolor(level_color)
    alert_text.set_text(level)

    tv_samples.set_text(str(s['samples']))

    return line_raw, line_filtered, line_thr, line_vel, line_lux, scat_brake, alert_text


if __name__ == '__main__':
    ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    ros_thread.start()

    ani = FuncAnimation(fig, animate, interval=100, blit=False, cache_frame_data=False)
    plt.show()
