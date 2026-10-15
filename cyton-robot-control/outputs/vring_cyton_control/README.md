# Unified Cyton Arm + VRing 2 Controller

Desktop interface bridging the **Robai Cyton robot arm** (servos 8–15) with the **VRing 2 smart ring** over BLE.

```text
Mac USB Serial (115,200) -> M5StickC Bridge -> Arduino Uno Gateway -> DYNAMIXEL Servos 8–15
Mac Bluetooth LE (CoreBluetooth) -> VRing 2 (XIAO nRF52840 + LSM6DS3 IMU)
```

## M5-to-arm wiring

Use the M5StickC Plus bottom HAT header. The Uno's DYNAMIXEL Shield owns the
servo bus; the M5 only provides the computer-to-Uno serial bridge.

```text
M5 G26 (TX) --------------------------> Uno/shield software-UART RX (D7)

Uno/shield software-UART TX (D8) -- 1 kOhm --+--> M5 G36 (RX)
                                                |
                                               2 kOhm
                                                |
M5 GND -----------------------------------------+---- Uno/shield GND
```

- The 1 kOhm / 2 kOhm divider lowers the Uno's 5 V TX signal to about 3.3 V
  for the M5 input.
- Do not connect M5 5 V, 3.3 V, BAT, or G0 to the Uno/shield.
- Power the M5 over USB or from its battery, the Uno separately, and the
  DYNAMIXEL chain from its dedicated regulated 12 V supply.

## Quick Start

```bash
# From cyton-robot-control root:
outputs/vring_cyton_control/launch_vring_gui.sh
```

## Features

1. **Full Robot Arm Control & Safety:**
   - Real-time telemetry: present positions, goals, moving state, motor voltage, and motor temperature.
   - Per-servo nudges, speed settings, and individual torque toggles.
   - Large Red **STOP** button and **Escape** shortcut to immediately disable torque across all joints.

2. **Starting Pose & Presets:**
   - **Straightened Starting Setup:** Arm is oriented straight vertical, with the base roll and gripper/wrist roll aligned to the user's reference setup:
     `8: 3539, 9: 2048, 10: 2048, 11: 2048, 12: 2048, 13: 2048, 14: 838, 15: 250`
   - **Home (All 2048):** Canonical upright home pose.
   - **User Example (Bent):** The bent elbow/wrist configuration (`8:3539, 9:1901, 10:1915, 11:2748, 12:2026, 13:2384, 14:838, 15:250`).

3. **VRing 2 Live BLE Telemetry:**
   - **IMU Orientation:** Reconstructed Roll, Pitch, and Yaw in degrees, and 4-element unit quaternion ($q_w, q_x, q_y, q_z$).
   - **Accelerometer:** 3-axis acceleration in $g$ and vector magnitude $|\vec{a}|$ (verifies $1.0g$ at rest).
   - **Touch & Hardware:** Live capacitive touch channels 1 & 2, battery voltage ($mV$) and percentage, and onboard temperature.
   - **Haptic Test:** Slider and pulse button to trigger the ring's vibration motor.

4. **IMU Calibration Data Logger:**
   - Click **"📸 Record Data Point"** to snapshot current robot joint encoder positions alongside simultaneous VRring IMU values.
   - Automatically writes to `vring_calibration_data.csv`.
