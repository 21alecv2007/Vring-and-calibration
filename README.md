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

## Simple calibration steps

Before starting, flash the ring with the `vring2/vring2.ino` firmware from this
repository. This version supports the BLE calibration data written by the GUI.

> **Ring mounting photo:** add the upcoming fixture photo here. Mount the ring
> in the same orientation shown in that photo for every trial.

1. Connect the M5 bridge to the computer by USB, power the arm, and mount the
   ring in the calibration fixture. Keep the arm's work area clear.
2. Run the setup and launch commands above.
3. In the GUI, connect the Cyton serial gateway, scan for the VRing, and
   connect to the correct ring.
4. Confirm arm telemetry is healthy, enable torque, then click **New CSV**.
5. Click **Run 6-Position Capture** once. Each run waits for every arm pose to
   settle, holds it for three seconds, and records the midpoint sample; it also
   records the required gyro rotations.
6. Remove the ring from the fixture, then put it back in before starting the
   next trial. This gives a slightly different real-world placement for the
   three-trial average.
7. Repeat steps 5–6 until there are three complete trials in the same CSV.
8. Click **Save 3-Trial Accel + Gyro Calibration to Ring**. The GUI validates
   the three trials and writes the new calibration to the ring over Bluetooth.

The saved constants replace the ring's active calibration. The CSV data stays
on the computer and is ignored by Git.

## Reflash the M5 bridge

Connect the M5StickC Plus by USB, then run this from `cyton-robot-control`:

```bash
bash flash_m5_bridge.sh
```

The script installs PlatformIO in the local `.venv` if necessary and uploads
the bridge firmware. If more than one serial device is connected, give the M5
port explicitly, for example `bash flash_m5_bridge.sh /dev/cu.usbserial-XXXX`.

## Reflash the Uno gateway

Connect the Arduino Uno by USB, set the DYNAMIXEL Shield UART switch to
`UPLOAD`, then run this from `cyton-robot-control`:

```bash
bash flash_uno_gateway.sh
```

The script asks for a final confirmation before flashing. After a successful
upload, return the shield switch to `DYNAMIXEL` before controlling the arm.

## Safety

The Cyton uses high-torque DYNAMIXEL servos. Keep the work area clear, begin
with torque disabled, and use the GUI's STOP control before touching wiring or
the arm.
