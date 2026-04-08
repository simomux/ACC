#!/usr/bin/env python3
"""
ACC Web Dashboard
=================
Flask + SSE + Plotly.js — rendering in the Mac browser, data served from the VM.

Requirements:
    pip install flask
    (rclpy comes with your ROS 2 installation)

Run:
    python3 dashboard_web.py
Then open: http://192.168.64.10:5000
"""

import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool
from flask import Flask, Response

# ── tunables (identical to dashboard.py) ─────────────────────────────────────
WINDOW_S        = 30
HZ              = 10
WINDOW_SAMPLES  = WINDOW_S * HZ
OUTLIER_THRESH  = 5.0
FALSE_POS_MAX_S = 0.4
V_REGRESSION_N  = 10
V_NOISE_FLOOR   = 1.0
HYST_MARGIN_CM  = 10.0
HYST_FRAMES     = 3
# ─────────────────────────────────────────────────────────────────────────────

lock = threading.Lock()

buf_dist_raw = deque([0.0]   * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_dist     = deque([0.0]   * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_thr      = deque([50.0]  * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_lux      = deque([0.0]   * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_brake    = deque([False] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
buf_vel      = deque([0.0]   * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)

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
    'v_rel':         0.0,
    'ttc':           float('inf'),
}

# ── alert level with hysteresis + temporal debounce (same as dashboard.py) ────
_alert_state   = 'SAFE'
_worse_counter = 0
_LEVEL_RANK    = {'SAFE': 0, 'APPROACHING': 1, 'CRITICAL': 2}
_LEVEL_COLORS  = {'SAFE': '#2ecc71', 'APPROACHING': '#f39c12', 'CRITICAL': '#e74c3c'}

def _alert_level(dist, thr, brake):
    global _alert_state, _worse_counter
    if brake:
        _alert_state = 'CRITICAL'
        _worse_counter = 0
        return _alert_state, _LEVEL_COLORS[_alert_state]

    entry_approaching = thr * 1.3 + 20
    entry_critical    = thr
    exit_approaching  = entry_approaching + HYST_MARGIN_CM
    exit_critical     = entry_critical    + HYST_MARGIN_CM

    if _alert_state == 'SAFE':
        target = 'APPROACHING' if dist <= entry_approaching else 'SAFE'
    elif _alert_state == 'APPROACHING':
        if dist <= entry_critical:
            target = 'CRITICAL'
        elif dist > exit_approaching:
            target = 'SAFE'
        else:
            target = 'APPROACHING'
    else:
        target = 'APPROACHING' if dist > exit_critical else 'CRITICAL'

    if _LEVEL_RANK[target] > _LEVEL_RANK[_alert_state]:
        _worse_counter += 1
        if _worse_counter < HYST_FRAMES:
            return _alert_state, _LEVEL_COLORS[_alert_state]
        _alert_state   = target
        _worse_counter = 0
    else:
        _alert_state   = target
        _worse_counter = 0
    return _alert_state, _LEVEL_COLORS[_alert_state]


def _noise_reduction(s):
    if s['noise_count'] == 0 or s['dist_raw'] == 0:
        return 0.0
    avg_noise = s['noise_sum'] / s['noise_count']
    return min(avg_noise / max(s['dist_raw'], 1.0) * 100.0, 99.9)


def _ttc_color(ttc):
    if ttc == float('inf') or ttc > 3.0:
        return '#2ecc71'
    if ttc > 1.0:
        return '#f39c12'
    return '#e74c3c'


# ── ROS 2 node (identical logic to dashboard.py) ─────────────────────────────
class AccSubscriber(Node):
    def __init__(self):
        super().__init__('acc_dashboard_web')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Float32, 'acc/distance',     self._cb_dist,     qos)
        self.create_subscription(Float32, 'acc/distance_raw', self._cb_dist_raw, qos)
        self.create_subscription(Float32, 'acc/threshold',    self._cb_thr,      qos)
        self.create_subscription(Float32, 'acc/lux',          self._cb_lux,      qos)
        self.create_subscription(Bool,    'acc/brake',        self._cb_brake,    qos)
        self._reg_dist = deque(maxlen=V_REGRESSION_N)
        self._reg_time = deque(maxlen=V_REGRESSION_N)

    def _cb_dist(self, msg):
        with lock:
            buf_dist.append(msg.data)
            stats['dist'] = msg.data
            delta = abs(stats['dist_raw'] - msg.data)
            if delta > OUTLIER_THRESH:
                stats['outliers'] += 1
            stats['noise_sum']   += delta
            stats['noise_count'] += 1
            stats['samples']     += 1

            now = time.monotonic()
            self._reg_dist.append(msg.data)
            self._reg_time.append(now)
            if len(self._reg_dist) >= 4:
                n      = len(self._reg_dist)
                t0     = self._reg_time[0]
                ts     = [t - t0 for t in self._reg_time]
                ds     = list(self._reg_dist)
                mean_t = sum(ts) / n
                mean_d = sum(ds) / n
                num    = sum((ti - mean_t) * (di - mean_d) for ti, di in zip(ts, ds))
                den    = sum((ti - mean_t) ** 2 for ti in ts)
                if den > 0:
                    stats['v_rel'] = num / den
            buf_vel.append(stats['v_rel'])
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
            now = time.monotonic()
            if msg.data and not prev:
                stats['brake_events'] += 1
                stats['last_brake_on'] = now
            if not msg.data and prev and stats['last_brake_on'] is not None:
                if (now - stats['last_brake_on']) < FALSE_POS_MAX_S:
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


# ── Flask ─────────────────────────────────────────────────────────────────────
app = Flask(__name__)

HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ACC Dashboard</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  background: #1a1a2e; color: #e0e0e0;
  font-family: 'Courier New', monospace;
  padding: 8px; height: 100vh;
  display: flex; flex-direction: column; gap: 8px;
}
h1 { text-align:center; color:#c0c0ff; font-size:13px; letter-spacing:2px; }
.main {
  display: grid;
  grid-template-columns: 3fr 1fr;
  gap: 8px;
  flex: 1;
  min-height: 0;
}
.left { display:flex; flex-direction:column; gap:8px; min-height:0; }
.chart-box { flex:1; min-height:0; background:#16213e; border-radius:6px; overflow:hidden; }
.stats {
  background:#16213e; border-radius:6px; padding:10px 14px;
  display:flex; flex-direction:column; gap:3px; overflow-y:auto;
}
.section {
  color:#a0a0ff; font-size:9px; text-transform:uppercase; letter-spacing:1px;
  border-bottom:1px solid #3a3a6a; padding-bottom:2px; margin-top:8px;
}
.section:first-child { margin-top:0; }
.row { display:flex; justify-content:space-between; align-items:baseline; gap:4px; }
.lbl { color:#8888cc; font-size:13px; }
.val { font-size:14px; font-weight:bold; white-space:nowrap; }
.alert-box {
  margin-top:auto; text-align:center; padding:8px 4px;
  border-radius:4px; font-size:14px; font-weight:bold;
  letter-spacing:2px; transition:background-color 0.3s;
}
#noise-bar { color:#4ecdc4; font-size:10px; letter-spacing:-1px; }
</style>
</head>
<body>
<h1>ACC Telemetry &nbsp;·&nbsp; Pico W / FreeRTOS / micro-ROS</h1>
<div class="main">
  <div class="left">
    <div class="chart-box" id="dist-div"></div>
    <div class="chart-box" id="lux-div"></div>
  </div>
  <div class="stats">
    <div class="section">Live Values</div>
    <div class="row"><span class="lbl">Distance</span>  <span class="val" id="v-dist">--</span></div>
    <div class="row"><span class="lbl">Raw</span>       <span class="val" id="v-raw">--</span></div>
    <div class="row"><span class="lbl">Threshold</span> <span class="val" id="v-thr">--</span></div>
    <div class="row"><span class="lbl">Lux</span>       <span class="val" id="v-lux">--</span></div>
    <div class="row"><span class="lbl">Brake</span>     <span class="val" id="v-brake">--</span></div>
    <div class="row"><span class="lbl">Velocity</span>  <span class="val" id="v-vel">--</span></div>
    <div class="row"><span class="lbl">TTC</span>       <span class="val" id="v-ttc">--</span></div>
    <div class="section">Filter &amp; Brake</div>
    <div class="row"><span class="lbl">Outliers</span>      <span class="val" id="v-out">--</span></div>
    <div class="row"><span class="lbl">Noise reduction</span><span class="val" id="v-noise-pct">--</span></div>
    <div class="row"><span class="lbl">Noise bar</span>     <span id="noise-bar"></span></div>
    <div class="row"><span class="lbl">Brake events</span>  <span class="val" id="v-events">--</span></div>
    <div class="row"><span class="lbl">False positives</span><span class="val" id="v-fpos">--</span></div>
    <div class="row"><span class="lbl">Samples</span>       <span class="val" id="v-samples">--</span></div>
    <div class="alert-box" id="alert-box">SAFE</div>
  </div>
</div>

<script>
const N   = """ + str(WINDOW_SAMPLES) + r""";
const xs  = Array.from({length:N}, (_,i) => i - N);

// rolling buffers (maintained in JS)
let rawBuf   = new Array(N).fill(0);
let filtBuf  = new Array(N).fill(0);
let thrBuf   = new Array(N).fill(50);
let velBuf   = new Array(N).fill(0);
let luxBuf   = new Array(N).fill(0);
let brakeBuf = new Array(N).fill(false);

function push(arr, val) { arr.push(val); arr.shift(); }

const pCfg = {responsive:true, displayModeBar:false};

const distLayout = {
  paper_bgcolor:'#16213e', plot_bgcolor:'#16213e',
  font:{color:'#e0e0e0',size:9},
  title:{text:'Distance — raw vs filtered  |  velocity', font:{size:10,color:'#c0c0ff'}, y:0.97},
  margin:{t:26,b:28,l:44,r:42},
  xaxis:{color:'#a0a0c0', gridcolor:'#2a2a4a', zeroline:false, tickfont:{size:8}},
  yaxis:{title:'cm', color:'#a0a0c0', gridcolor:'#2a2a4a', zeroline:false},
  yaxis2:{title:'cm/s', overlaying:'y', side:'right', color:'#c39bd3',
          gridcolor:'transparent', zeroline:true, zerolinecolor:'#c39bd3',
          zerolinewidth:0.5, tickfont:{size:7,color:'#c39bd3'}},
  legend:{bgcolor:'rgba(0,0,0,0)', font:{size:8}},
  shapes:[],
};
const luxLayout = {
  paper_bgcolor:'#16213e', plot_bgcolor:'#16213e',
  font:{color:'#e0e0e0',size:9},
  title:{text:'Light Sensor — lux  (● brake events)', font:{size:10,color:'#c0c0ff'}, y:0.97},
  margin:{t:26,b:28,l:44,r:10},
  xaxis:{color:'#a0a0c0', gridcolor:'#2a2a4a', zeroline:false, tickfont:{size:8}},
  yaxis:{title:'lux', color:'#a0a0c0', gridcolor:'#2a2a4a', zeroline:false},
  legend:{bgcolor:'rgba(0,0,0,0)', font:{size:8}},
};

Plotly.newPlot('dist-div', [
  {name:'raw',       x:xs, y:[...rawBuf],  mode:'lines', line:{color:'#ff6b6b',width:1.2}, opacity:0.6},
  {name:'filtered',  x:xs, y:[...filtBuf], mode:'lines', line:{color:'#4ecdc4',width:2}},
  {name:'threshold', x:xs, y:[...thrBuf],  mode:'lines', line:{color:'#ffe66d',width:1.5,dash:'dash'}},
  {name:'vel',       x:xs, y:[...velBuf],  mode:'lines', line:{color:'#c39bd3',width:1,dash:'dot'}, opacity:0.85, yaxis:'y2'},
], distLayout, pCfg);

Plotly.newPlot('lux-div', [
  {name:'lux',          x:xs, y:[...luxBuf], mode:'lines',   line:{color:'#ffd93d',width:1.8}},
  {name:'brake events', x:[], y:[],           mode:'markers', marker:{color:'#ff4757',size:8}},
], luxLayout, pCfg);

const es = new EventSource('/stream');
es.onmessage = e => {
  const d = JSON.parse(e.data);

  push(rawBuf,   d.dist_raw);
  push(filtBuf,  d.dist);
  push(thrBuf,   d.threshold);
  push(velBuf,   d.v_rel);
  push(luxBuf,   d.lux);
  push(brakeBuf, d.brake);

  // brake markers: indices where brake=true in the rolling window
  const brakeXs = [], brakeYs = [];
  brakeBuf.forEach((b, i) => { if(b){ brakeXs.push(xs[i]); brakeYs.push(luxBuf[i]); }});

  // zone fills as layout shapes
  const thr    = d.threshold;
  const shapes = [
    {type:'rect', xref:'paper', yref:'y', x0:0, x1:1, y0:0, y1:thr,
     fillcolor:'#e74c3c', opacity:0.08, line:{width:0}, layer:'below'},
    {type:'rect', xref:'paper', yref:'y', x0:0, x1:1, y0:thr, y1:thr*1.3+20,
     fillcolor:'#f39c12', opacity:0.05, line:{width:0}, layer:'below'},
  ];
  const yMax   = Math.max(Math.max(...rawBuf)*1.15, thr*1.5, 60);
  const velAbs = Math.max(Math.max(...velBuf.map(Math.abs)), 10)*1.2;

  Plotly.react('dist-div', [
    {name:'raw',       x:xs, y:[...rawBuf],  mode:'lines', line:{color:'#ff6b6b',width:1.2}, opacity:0.6},
    {name:'filtered',  x:xs, y:[...filtBuf], mode:'lines', line:{color:'#4ecdc4',width:2}},
    {name:'threshold', x:xs, y:[...thrBuf],  mode:'lines', line:{color:'#ffe66d',width:1.5,dash:'dash'}},
    {name:'vel',       x:xs, y:[...velBuf],  mode:'lines', line:{color:'#c39bd3',width:1,dash:'dot'}, opacity:0.85, yaxis:'y2'},
  ], {...distLayout, shapes,
      yaxis:{...distLayout.yaxis, range:[0,yMax]},
      yaxis2:{...distLayout.yaxis2, range:[-velAbs,velAbs]}});

  const luxMax = Math.max(Math.max(...luxBuf)*1.2, 10);
  Plotly.react('lux-div', [
    {name:'lux',          x:xs, y:[...luxBuf], mode:'lines',   line:{color:'#ffd93d',width:1.8}},
    {name:'brake events', x:brakeXs, y:brakeYs, mode:'markers', marker:{color:'#ff4757',size:8}},
  ], {...luxLayout, yaxis:{...luxLayout.yaxis, range:[0,luxMax]}});

  // ── stats panel ──────────────────────────────────────────────────────────
  document.getElementById('v-dist').textContent    = d.dist.toFixed(1)+' cm';
  document.getElementById('v-raw').textContent     = d.dist_raw.toFixed(1)+' cm';
  document.getElementById('v-thr').textContent     = d.threshold.toFixed(1)+' cm';
  document.getElementById('v-lux').textContent     = d.lux.toFixed(0)+' lx';
  document.getElementById('v-samples').textContent = d.samples;
  document.getElementById('v-out').textContent     = d.outliers;
  document.getElementById('v-events').textContent  = d.brake_events;
  document.getElementById('v-fpos').textContent    = d.false_pos;
  document.getElementById('v-noise-pct').textContent = d.noise_pct.toFixed(1)+' %';

  const filled = Math.round(d.noise_pct / 100 * 16);
  document.getElementById('noise-bar').textContent = '█'.repeat(filled) + '░'.repeat(16-filled);

  const brakeEl = document.getElementById('v-brake');
  brakeEl.textContent = d.brake ? 'YES ●' : 'no';
  brakeEl.style.color = d.brake ? '#ff4757' : '#e0e0e0';

  const velEl = document.getElementById('v-vel');
  velEl.textContent   = (d.v_rel >= 0 ? '+' : '') + d.v_rel.toFixed(1)+' cm/s';
  velEl.style.color   = d.v_rel >= 0 ? '#2ecc71' : '#e74c3c';

  const ttcEl = document.getElementById('v-ttc');
  ttcEl.textContent   = d.ttc === null ? '∞' : d.ttc.toFixed(1)+' s';
  ttcEl.style.color   = d.ttc_color;

  const alertBox = document.getElementById('alert-box');
  alertBox.textContent        = d.alert_level;
  alertBox.style.backgroundColor = d.alert_color;
};
</script>
</body>
</html>
"""


@app.route('/')
def index():
    return HTML


@app.route('/stream')
def stream():
    def generate():
        while True:
            with lock:
                s = dict(stats)
            level, color = _alert_level(s['dist'], s['thr'], s['brake'])
            noise_pct    = _noise_reduction(s)
            ttc          = s['ttc']
            payload = {
                'dist':         s['dist'],
                'dist_raw':     s['dist_raw'],
                'threshold':    s['thr'],
                'lux':          s['lux'],
                'brake':        s['brake'],
                'v_rel':        round(s['v_rel'], 2),
                'ttc':          None if ttc == float('inf') else round(ttc, 1),
                'ttc_color':    _ttc_color(ttc),
                'outliers':     s['outliers'],
                'noise_pct':    noise_pct,
                'brake_events': s['brake_events'],
                'false_pos':    s['false_pos'],
                'samples':      s['samples'],
                'alert_level':  level,
                'alert_color':  color,
            }
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.1)

    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})


if __name__ == '__main__':
    threading.Thread(target=_ros_spin, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, threaded=True)
