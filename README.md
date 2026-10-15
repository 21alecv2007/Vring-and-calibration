# VRing Arm Calibration

This repository contains the minimum software, firmware, and wiring reference
needed to calibrate a VRing 2 IMU using a Robai Cyton arm.

## What is included

| Folder | Contents |
|---|---|
| `cyton-robot-control/` | The desktop calibration GUI plus the Arduino Uno and M5StickC Plus firmware that move the arm safely |
| `vring2/` | The VRing firmware, orientation filter, and current hardware schematic |

## Start the calibration GUI

```bash
cd cyton-robot-control
bash setup_calibration.sh
bash outputs/vring_cyton_control/launch_vring_gui.sh
```

Run the setup command once on a new computer. It creates a local `.venv` and
installs the required Python packages. The GUI connects to the arm over the M5
USB serial bridge and to the ring over Bluetooth LE. See the calibration GUI
README for wiring and safety notes.

## Safety

The Cyton uses high-torque DYNAMIXEL servos. Keep the work area clear, begin
with torque disabled, and use the GUI's STOP control before touching wiring or
the arm.
