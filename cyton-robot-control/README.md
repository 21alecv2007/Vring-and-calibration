# Cyton Arm Control for VRing Calibration

This folder contains only the arm-control components required by the VRing
calibration workflow.

| Folder | Purpose |
|---|---|
| `outputs/cyton_gateway_uno/` | Arduino Uno + DYNAMIXEL Shield gateway firmware |
| `outputs/m5_uart_bridge/` | M5StickC Plus USB-to-UART bridge firmware |
| `outputs/vring_cyton_control/` | Desktop GUI that moves the arm, records IMU data, and sends ring calibration values over BLE |

Start the desktop application from this folder:

```bash
bash setup_calibration.sh
bash outputs/vring_cyton_control/launch_vring_gui.sh
```

`setup_calibration.sh` only needs to be run once per computer. It creates the
local Python environment and installs `bleak` and `pyserial`.

## Reflash the M5 bridge

With the M5StickC Plus connected by USB, run:

```bash
bash flash_m5_bridge.sh
```

The script installs PlatformIO locally when needed, then builds and uploads the
firmware in `outputs/m5_uart_bridge`. If automatic port selection cannot choose
one device, pass the port as the first argument.

Use the individual READMEs for the wiring and firmware-flashing steps. The
gateway disables servo torque at boot; always confirm the arm is clear before
enabling torque.
