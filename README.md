Betaflight, refer to the original repository at https://github.com/betaflight/betaflight

![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo.svg#gh-light-mode-only)
![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo_dark.svg#gh-dark-mode-only)

The repository is forked from [f5e4e49](https://github.com/betaflight/betaflight/commit/f5e4e49547719d2e3ff9d3c14d4d04f834ba9e5d) commit, and used for testing and development of the TFC1 flight controller, which is based on RaspberryPi RP2354B.

## TFC1 Flight Controller

![](https://www.t2tx.com/images/tfc1-1_hu_80d2fd48e300b812.webp)

`TFC1` is a flight controller board designed for fixed-wing drones, developed by [T2T Inc](https://www.t2t.io). It is based on the Raspberry Pi RP2354B microcontroller, which features a dual-core Arm Cortex-M0+ processor running at up to 133 MHz, 264 KB of RAM, and 8 MB of flash memory. The TFC1 flight controller is equipped with various sensors and interfaces, including an IMU (Inertial Measurement Unit), barometer, magnetometer, and multiple UART ports for communication with peripherals.


## Quick Development Guide for Betaflight on TFC1

Clone the repsository and install the necessary tools for development:

```bash
$ git clone --recursive https://github.com/t2t-io/betaflight-tfc1-20260519.git
$ cd betaflight-tfc1-20260519
$ make arm_sdk_install # Install the ARM SDK toolchain
$ make picotool_install # Install the Picotool utility for flashing the firmware
```

To build the firmware for the TFC1 flight controller, run the following command:

```bash
$ make clean && PICO_TRACE=1 make TFC1_RP2354B_CC
```

After the build process is complete, you need to flash the firmware image file (uf2) to the TFC1 flight controller. Since the TFC1 flight controller is based on the Raspberry Pi RP2354B, you can press the BOOT button on the TFC1 and power it on to enter the bootloader mode. In this mode, the TFC1 is mounted as a USB mass storage device on your computer, and you can copy the generated uf2 file to the TFC1 to flash the firmware. Use the following command to copy the uf2 file to the TFC1:

```bash
$ cp -vf ./obj/betaflight_2026.6.0-alpha_RP2350B_TFC1_RP2354B_CC.uf2 /Volumes/RP2350/
```
