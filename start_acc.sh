#!/usr/bin/env bash
# start_acc.sh -- runs inside WSL

SERIAL_PORT=/dev/ttyACM0
BAUD=115200
AGENT_LOG=/tmp/microros_agent.log
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== ACC Launcher ==="

# ── ROS 2 ────────────────────────────────────────────────────────────────────
echo "[1/5] Sourcing ROS 2 Jazzy..."
if ! source /opt/ros/jazzy/setup.bash 2>/dev/null; then
    echo "[error] ROS 2 not found at /opt/ros/jazzy"
    exec bash; exit 1
fi

# ── micro-ROS workspace ───────────────────────────────────────────────────────
echo "[2/5] Sourcing micro-ROS workspace..."
if ! source ~/microros_ws/install/local_setup.bash 2>/dev/null; then
    echo "[error] micro-ROS workspace not found at ~/microros_ws"
    echo "        Build it with: ros2 run micro_ros_setup build_agent.sh"
    exec bash; exit 1
fi

# ── wait for serial port ──────────────────────────────────────────────────────
echo "[3/5] Waiting for $SERIAL_PORT..."
for i in $(seq 1 20); do
    [ -e "$SERIAL_PORT" ] && break
    sleep 0.5
done

if [ ! -e "$SERIAL_PORT" ]; then
    echo "[error] $SERIAL_PORT not found after 10s. Is the Pico attached via usbipd?"
    exec bash; exit 1
fi
echo "       Found $SERIAL_PORT"

# ── permissions ───────────────────────────────────────────────────────────────
if [ ! -w "$SERIAL_PORT" ]; then
    echo "       Fixing permissions..."
    sudo chmod a+rw "$SERIAL_PORT"
fi

# ── micro-ROS agent ───────────────────────────────────────────────────────────
echo "[4/5] Starting micro-ROS agent (log: $AGENT_LOG)..."
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev "$SERIAL_PORT" -b "$BAUD" > "$AGENT_LOG" 2>&1 &
AGENT_PID=$!
echo "       PID $AGENT_PID"

cleanup() {
    echo "Stopping agent (PID $AGENT_PID)..."
    kill "$AGENT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1

# ── dashboard ─────────────────────────────────────────────────────────────────
echo "[5/5] Launching dashboard..."
python3 "$SCRIPT_DIR/dashboard.py"

exec bash
