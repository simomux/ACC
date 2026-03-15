# Adaptive Cruise Control

Demo of an ACC system on a Raspberry Pi Pico W using FreeRTOS with dual-core task scheduling and ROS 2 telemetry via micro-ROS.

- **Core 0** — sensor tasks: HC-SR04 (distance, median-filtered), BH1750 (brake light detection), potentiometer (threshold), RGB LED + buzzer (alert), OLED display
- **Core 1** — micro-ROS task: publishes telemetry over USB serial to a ROS 2 agent on the PC
- **Dashboard** — Python + matplotlib live dashboard with raw vs filtered distance, lux graph, filter statistics and brake event detection

## Hardware

| Component | Pin(s) |
| --------- | ------ |
| HC-SR04 Trigger | GP2 |
| HC-SR04 Echo | GP3 |
| BH1750 SDA / SCL | GP4 / GP5 (i2c0) |
| Potentiometer | GP26 (ADC0) |
| RGB LED R / G / B | GP13 / GP14 / GP15 |
| Buzzer | GP16 |
| Mute button | GP17 |
| SH1106 OLED SDA / SCL | GP18 / GP19 (i2c1) |

---

## Setup (Windows 10/11 + WSL2)

All development and runtime steps run inside **WSL2 (Ubuntu 24.04)**. Windows is only needed for flashing the firmware and forwarding the USB device.

### 1. Windows — install usbipd-win

Run once from **PowerShell (Administrator)**:

```powershell
winget install usbipd
```

> Required to forward USB devices from Windows to WSL2.

### 2. WSL — system dependencies

```bash
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib build-essential git python3 \
    python3-matplotlib
```

### 3. WSL — Pico SDK

```bash
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
echo 'export PICO_SDK_PATH=$HOME/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### 4. WSL — ROS 2 Jazzy

Follow the official installation guide for your distro:
[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 5. WSL — micro-ROS agent (build from source)

The `ros-jazzy-micro-ros-agent` apt package does not exist yet; build it once:

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

Verify:

```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent --help
```

### 6. WSL — USB serial permissions

```bash
sudo usermod -a -G dialout $USER
# restart WSL after this (exit and reopen the terminal)
```

---

## Clone and build

```bash
git clone --recurse-submodules https://github.com/simomux/ACC.git ~/ACC
cd ~/ACC
mkdir build && cd build
cmake -DPICO_BOARD=pico_w ..
make -j$(nproc)
```

The firmware is generated at `build/acc.uf2`.

---

## Flash

Hold **BOOTSEL** on the Pico, plug in USB — it mounts as `RPI-RP2`.

**From Windows Explorer**: drag and drop `build/acc.uf2` onto the `RPI-RP2` drive.

**From WSL** (after attaching via usbipd, see below):

```bash
cp build/acc.uf2 /mnt/d/RPI-RP2/   # adjust drive letter as needed
```

---

## Run

### Automatic (recommended)

From **Windows Explorer**, double-click `start_acc.bat`.

> Using the `.bat` launcher avoids the PowerShell UNC security prompt that appears when running `.ps1` files directly from the WSL filesystem.

The script will:

1. Self-elevate to Administrator
2. Poll for the Pico W — if it is detected in **BOOTSEL mode** (e.g. while flashing) it prints a warning and keeps waiting; once you unplug and replug normally it continues automatically
3. Bind and attach the USB device to WSL via usbipd
4. Open a WSL terminal that starts the micro-ROS agent and the live dashboard

**Typical workflow when flashing + running in the same session:**

1. Hold **BOOTSEL**, plug in the Pico — `start_acc.bat` detects it and prints:

   ```text
   Pico is in BOOTSEL mode -- flash the firmware, then unplug and replug normally.
   ```

2. Copy `build/acc.uf2` to the `RPI-RP2` drive (the Pico reboots automatically)
3. The script detects the Pico in normal mode and launches the dashboard — no restart needed

### Manual

**PowerShell (Administrator)** — forward USB to WSL:

```powershell
usbipd list                          # find the Pico's BUSID (e.g. 2-9)
usbipd bind --busid <BUSID>          # one-time, marks device as shareable
usbipd attach --wsl --busid <BUSID>  # attach to WSL (repeat each session)
```

**WSL** — start agent and dashboard:

```bash
cd ~/ACC && bash start_acc.sh
```

### Verify ROS 2 topics

In a separate WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

Expected topics:

```text
/acc/brake
/acc/distance
/acc/distance_raw
/acc/lux
/acc/threshold
```

---

## Dashboard

The dashboard (`dashboard.py`) shows:

- **Distance graph** — raw HC-SR04 sample vs median-filtered output vs threshold line; coloured zones (red = critical, orange = approaching)
- **Lux graph** — ambient light over time with brake detection events marked
- **Stats panel** — outliers rejected by the filter, noise reduction %, brake events count, false positives, current alert level

Requires only `python3-matplotlib` (installed via apt in step 2).

---

## Architecture

```text
┌─────────────────────────── Pico W ─────────────────────────┐
│                                                            │
│  Core 0                          Core 1                    │
│  ├─ vTaskSensor  (60ms)          └─ vTaskMicroROS (100ms)  │
│  │    HC-SR04, median-of-3              │                  │
│  ├─ vTaskBrake   (100ms)                │ USB serial       │
│  │    BH1750 lux + brake detect         │ (UART framing)   │
│  ├─ vTaskDimmer  (200ms)                ▼                  │
│  │    ADC potentiometer          micro-ROS agent           │
│  ├─ vTaskAlert   (100ms)                │                  │
│  │    RGB LED + buzzer                  │ ROS 2 topics     │
│  └─ vTaskOled    (500ms)                ▼                  │
│       SH1106 128x64              dashboard.py              │
│                                  (matplotlib)              │
└────────────────────────────────────────────────────────────┘
```

## ROS 2 topics

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `/acc/distance` | `std_msgs/Float32` | Median-filtered distance (cm) |
| `/acc/distance_raw` | `std_msgs/Float32` | Raw HC-SR04 sample pre-filter (cm) |
| `/acc/threshold` | `std_msgs/Float32` | Current alert threshold (cm) |
| `/acc/lux` | `std_msgs/Float32` | Ambient light (lux) |
| `/acc/brake` | `std_msgs/Bool` | Brake light detected |
