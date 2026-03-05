# Adaptive Cruise Control

Simple demo for an ACC system on a raspberry pi pico w using a ultrasound sensor with freeRTOS and various tasks to monitor a constant distance from the veichle upfront, while signaling the distance to the user via LED.

## Dependencies

### 0. Configure Groups

``` bash
# add user to groups
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER

# To mount PICO without sudo
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0003", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-rpi-pico.rules > /dev/null
```

(reboot your system after)

### 1. Install Pico SDK

First, make sure the Pico SDK is properly installed and configured:

```bash
# Install dependencies
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone this repository

```bash
git clone --recursive -b noros https://github.com/cscribano/RTES_freertos_PICO.git
```

## Compile the code

```bash
cd ACC
mkdir build
cd build
cmake -DPICO_BOARD=pico_w ..
make
```

## Flash an example (linux)

To flash hold the boot button, plug the USB and run:

```bash
cp main.uf2 $(findmnt -rn -o TARGET -S LABEL=RPI-RP2)/
```

### On windows simply drag the file in the disk after connecting in bootsel mode