# Adaptive Cruise Control

Simple demo of an ACC system on a Raspberry Pi Pico W using FreeRTOS.  
Multiple tasks monitor the distance from the vehicle ahead (HC-SR04 ultrasonic sensor), detect brake lights (BH1750 light sensor), and alert the driver via an RGB LED and a passive buzzer.  
A potentiometer (dimmer) lets the user adjust the distance threshold at runtime.

## Prerequisites

### 1. Configure user groups (Linux)

```bash
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER

# Allow mounting the Pico without sudo
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0003", MODE="0660", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-rpi-pico.rules > /dev/null
```

> Reboot after applying these changes.

### 2. Install the Pico SDK

```bash
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3

git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

echo 'export PICO_SDK_PATH=$HOME/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

## Clone

```bash
git clone --recurse-submodules https://github.com/simomux/ACC.git
```

## Build

```bash
cd ACC
mkdir build && cd build
cmake -DPICO_BOARD=pico_w ..
make -j
```

The firmware will be generated at `build/acc.uf2`.

## Flash

Hold the **BOOTSEL** button on the Pico, plug in the USB cable, then:

### Linux

```bash
cp build/acc.uf2 $(findmnt -rn -o TARGET -S LABEL=RPI-RP2)/
```

### Windows

Drag and drop `acc.uf2` onto the `RPI-RP2` drive.

### WSL

To forward the Pico's USB to WSL, run these commands from **PowerShell (as Administrator)** on the Windows host:

```powershell
usbipd list                          # find the Pico's BUSID
usbipd bind --busid <BUSID>          # share the device (one-time)
usbipd attach --wsl --busid <BUSID>  # attach to WSL
```

> Requires [usbipd-win](https://github.com/dorssel/usbipd-win) installed on Windows.
