# Adaptive Cruise Control

Demo of an ACC system on a Raspberry Pi Pico W using FreeRTOS with dual-core task scheduling and ROS 2 telemetry via micro-ROS.

- **Core 1** - `vTaskMicroROS` (pinned): publishes telemetry over USB serial to a ROS 2 agent on the PC
- **FreeRTOS scheduler** - all other tasks run dynamically: HC-SR04 (distance, median-filtered), BH1750 (brake light detection), potentiometer (threshold), RGB LED + buzzer (alert), OLED display
- **Dashboard** - Browser-based live dashboard (Flask + Plotly.js) with raw vs filtered distance, velocity, lux graph, filter statistics and brake event detection

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

### 1. Windows - install usbipd-win

Run once from **PowerShell (Administrator)**:

```powershell
winget install usbipd
```

> Required to forward USB devices from Windows to WSL2.

### 2. WSL - system dependencies

```bash
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib build-essential git python3 \
    python3-matplotlib
```

### 3. WSL - Pico SDK

```bash
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
echo 'export PICO_SDK_PATH=$HOME/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### 4. WSL - ROS 2 Jazzy

Follow the official installation guide for your distro:
[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 5. WSL - micro-ROS agent (build from source)

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

### 6. WSL - USB serial permissions

```bash
sudo usermod -a -G dialout $USER
# restart WSL after this (exit and reopen the terminal)
```

---

## Setup (macOS ARM - Apple Silicon + UTM VM)

The recommended macOS setup runs the ROS 2 stack inside an **Ubuntu 24.04 VM** managed by [UTM](https://mac.getutm.app). The micro-ROS agent runs in the VM and receives the Pico's USB serial stream via UTM's USB passthrough. The web dashboard is served by Flask inside the VM and opens in the Mac's browser - rendering is hardware-accelerated on the Mac with no display forwarding needed.

```text
Pico W ──USB passthrough──► Ubuntu 24.04 VM (UTM)
                              micro-ROS agent
                              Flask + Plotly.js server
                                    │ HTTP (192.168.64.10:5000)
                              Safari / Chrome (Mac)
```

> ROS 2 Jazzy runs natively on macOS via RoboStack/conda, but the micro-ROS agent must be built from source and its build system requires library versions that are incompatible with the RoboStack conda packages. No precompiled binary exists for macOS. A Linux VM sidesteps this entirely since the apt packages are consistent with the micro-ROS build system.

### 1. Create the Ubuntu 24.04 VM in UTM

1. Download [UTM](https://mac.getutm.app) and install it
2. Create a new VM: **Virtualize → Linux**, select the Ubuntu 24.04 ISO
3. Assign at least **4 GB RAM** (8 GB recommended) and **30 GB** disk
4. Boot and complete the Ubuntu installer

### 2. VM - system setup

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y openssh-server python3-flask

# USB serial access without sudo (log out and back in after this)
sudo usermod -a -G dialout $USER
```

### 3. VM - ROS 2 and micro-ROS agent

Follow steps **4** and **5** of the [WSL setup](#4-wsl---ros-2-jazzy) above - the commands are identical on Ubuntu 24.04.
Use `~/microros_ws` as the workspace directory.

### 4. Copy scripts to the VM

```zsh
scp ~/ACC/start_acc_vm.sh ~/ACC/dashboard_web.py ~/ACC/mock_pico.py ubuntu@192.168.64.10:~/
ssh ubuntu@192.168.64.10 "chmod +x ~/start_acc_vm.sh"
```

### 5. UTM - USB passthrough for the Pico

**USB passthrough:**

With the VM running, plug in the Pico, then click the **USB icon** in the UTM toolbar and select the Pico. The device appears as `/dev/ttyACM0` immediately.

### 6. Run on macOS

```zsh
cd ~/ACC

# With the real Pico connected via USB:
./start_acc_mac.sh

# Without hardware (120-second simulated telemetry scenario):
./start_acc_mac.sh --mock
```

`start_acc_mac.sh` SSHes into the VM, starts the agent (or mock publisher), launches the Flask server, and opens the browser on the Mac automatically once the server is ready.

### Verify ROS 2 topics

From a terminal inside the VM:

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 topic list
```

---

## Clone and build

```bash
git clone --recurse-submodules https://github.com/simomux/ACC.git ~/ACC
cd ~/ACC
mkdir build && cd build
cmake -DPICO_BOARD=pico_w ..
make -j$(nproc)          # Linux/WSL
make -j$(sysctl -n hw.logicalcpu)   # macOS
```

The firmware is generated at `build/acc.uf2`.

---

## Flash

Hold **BOOTSEL** on the Pico, plug in USB - it mounts as `RPI-RP2`.

**From Windows Explorer**: drag and drop `build/acc.uf2` onto the `RPI-RP2` drive.

---

## Run

### Windows + WSL

> [!NOTE]
> **First time only - manual bind required.**
> Before using the automatic script you need to mark the device as shareable once.
> Open **PowerShell (Administrator)** and run:
> ```powershell
> usbipd list                      # find the Pico's BUSID (e.g. 2-9)
> usbipd bind --busid <BUSID>      # mark device as shareable (one-time)
> ```
> After this, `start_acc.bat` will handle the attach automatically every session.

Run `start_acc.bat`. The script will:

1. Self-elevate to Administrator
2. Poll for the Pico W - if detected in **BOOTSEL mode** it waits; once you replug normally it continues
3. Bind and attach the USB device to WSL via usbipd
4. Open a WSL terminal that starts the micro-ROS agent and the matplotlib dashboard

### macOS

```zsh
cd ~/ACC
./start_acc_mac.sh          # real Pico
./start_acc_mac.sh --mock   # no hardware (120s simulated scenario)
```

`start_acc_mac.sh` SSHes into the VM, waits for the Pico on `/dev/ttyACM0`, starts the micro-ROS agent, launches the Flask server, and opens the browser automatically once the dashboard is ready.

---

## Dashboard

Two dashboard implementations are provided depending on the platform.

### WSL / Windows - `dashboard.py` (matplotlib)

Runs directly inside WSL. Started automatically by `start_acc.sh` / `start_acc.bat`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
python3 ~/ACC/dashboard.py
```

Requires `python3-matplotlib` (installed in WSL setup step 2).

### macOS - `dashboard_web.py` (Flask + Plotly.js)

`dashboard_web.py` is a Flask server that streams telemetry to the browser via Server-Sent Events and renders charts with Plotly.js. Open `http://192.168.64.10:5000` from the Mac browser after starting the system. Started automatically by `start_acc_mac.sh`.

Requires `python3-flask` in the VM (`sudo apt install -y python3-flask`).

---

## ROS 2 topics

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `/acc/distance` | `std_msgs/Float32` | Median-filtered distance (cm) |
| `/acc/distance_raw` | `std_msgs/Float32` | Raw HC-SR04 sample pre-filter (cm) |
| `/acc/threshold` | `std_msgs/Float32` | Current alert threshold (cm) |
| `/acc/lux` | `std_msgs/Float32` | Ambient light (lux) |
| `/acc/brake` | `std_msgs/Bool` | Brake light detected |
