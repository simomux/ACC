#!/usr/bin/env bash
# start_acc_vm.sh -- ACC launcher (runs inside the Ubuntu VM)
#
# Usage:
#   ./start_acc_vm.sh           # hardware mode (real Pico)
#   ./start_acc_vm.sh --mock    # mock mode (no hardware needed)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/microros_ws"
AGENT_LOG="/tmp/microros_agent.log"
SERIAL_PORT="/dev/ttyACM0"
BAUD=115200
MOCK=0

for arg in "$@"; do
    [[ "$arg" == "--mock" ]] && MOCK=1
done

echo "=== ACC Launcher (VM) ==="
(( MOCK )) && echo "    [mock mode — no hardware required]"

# ── Source ROS 2 + micro-ROS workspace ───────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# ── Display locale della VM (ignora eventuale forwarding X11 da SSH) ──────────
export DISPLAY=:0


# ── 1. Serial port detection (hardware mode only) ─────────────────────────────
if (( ! MOCK )); then
    echo "[1/3] Waiting for Pico on $SERIAL_PORT..."
    echo "      (plug in the Pico, or pass through the USB device in UTM)"
    while [[ ! -e "$SERIAL_PORT" ]]; do
        sleep 0.5
        printf "."
    done
    printf "\n       Found Pico at %s\n" "$SERIAL_PORT"
else
    echo "[1/3] Skipped (mock mode)"
fi

# ── 2. Start agent or mock publisher ─────────────────────────────────────────
PUBLISHER_PID=""
cleanup() {
    if [[ -n "$PUBLISHER_PID" ]]; then
        printf "\nStopping publisher (PID %s)...\n" "$PUBLISHER_PID"
        kill "$PUBLISHER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if (( MOCK )); then
    echo "[2/3] Starting mock Pico publisher (log: $AGENT_LOG)..."
    python3 "$SCRIPT_DIR/mock_pico.py" > "$AGENT_LOG" 2>&1 &
    PUBLISHER_PID=$!
else
    echo "[2/3] Starting micro-ROS agent (log: $AGENT_LOG)..."
    ros2 run micro_ros_agent micro_ros_agent serial \
        --dev "$SERIAL_PORT" -b "$BAUD" > "$AGENT_LOG" 2>&1 &
    PUBLISHER_PID=$!
fi
echo "       PID $PUBLISHER_PID"

sleep 1

# ── 3. Launch dashboard ───────────────────────────────────────────────────────
echo "[3/3] Launching web dashboard on http://192.168.64.10:5000 ..."
python3 "$SCRIPT_DIR/dashboard_web.py"