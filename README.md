# VRing and Cyton Calibration

This repository contains the software, firmware, schematics, and calibration
records for a Robai Cyton robot arm and a VRing 2 Bluetooth sensor ring.

## Project layout

| Folder | Purpose |
|---|---|
| `cyton-robot-control/` | Cyton arm gateway firmware, desktop control GUIs, IMU-calibration workflow, and recorded calibration data |
| `vring2/` | VRing 2 firmware, BLE tools, calibration tester, block-sorter demo, and hardware schematics |

## Start here

To run the combined arm and ring controller on macOS:

```bash
cd cyton-robot-control
bash outputs/vring_cyton_control/launch_vring_gui.sh
```

The ring firmware is `vring2/vring2.ino`. The arm controller and ring tools are
described in their respective READMEs below.

## Safety

The arm uses high-torque DYNAMIXEL servos. Begin with torque disabled, keep the
arm clear of people and objects, and use the controller's STOP control before
making wiring or mechanical changes.
