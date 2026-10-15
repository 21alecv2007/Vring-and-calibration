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
bash outputs/vring_cyton_control/launch_vring_gui.sh
```

Use the individual READMEs for the wiring and firmware-flashing steps. The
gateway disables servo torque at boot; always confirm the arm is clear before
enabling torque.
