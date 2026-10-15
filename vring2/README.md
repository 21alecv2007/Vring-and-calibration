# VRing 2 Firmware for Arm-Guided Calibration

This folder contains the ring firmware used by the calibration GUI.

| File or folder | Purpose |
|---|---|
| `vring2.ino` | XIAO nRF52840 firmware: IMU sampling, BLE telemetry, haptics, button input, and persistent BLE calibration storage |
| `MadgwickAHRS.cpp`, `MadgwickAHRS.h` | Orientation filter used by the firmware |
| `schematics/` | Current as-built wiring diagram, including the D10 pushbutton |

## Calibration connection

The arm calibration GUI is located at
`../cyton-robot-control/outputs/vring_cyton_control`. It reads the ring over
Bluetooth LE and writes calibration constants to the ring's internal storage.
Once this firmware version has been flashed, later calibration runs do not
require another ring flash.

## Flashing requirements

Use a Seeed Studio XIAO nRF52840 Sense with the Bluefruit (non-mbed) Arduino
core. The firmware also requires the Seeed LSM6DS3, Adafruit PDM, and
arduino-libopus libraries.

See [the schematic](schematics/README.md) before connecting the battery,
motor, touch electrodes, or button.
