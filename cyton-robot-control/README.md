# Cyton robot control

Safety-first control stack for a Robai Cyton arm (eight ROBOTIS DYNAMIXEL
servos, IDs 8–15) driven from a Windows, macOS, or Linux computer:

```text
Computer USB serial (115,200 baud)
    -> M5StickC Plus (transparent USB/UART bridge)
    -> 57,600-baud 3.3 V UART
    -> Arduino Uno + ROBOTIS DYNAMIXEL Shield (text command gateway)
    -> 1 Mbps Protocol 1.0 DYNAMIXEL bus
    -> Cyton arm
```

| Directory | What it is |
|---|---|
| `outputs/cyton_gateway_uno` | PlatformIO firmware for the Uno gateway: torque off at boot, limit-checked moves, `ENABLE`/`MOVE`/`GROUP`/`STOP` protocol |
| `outputs/m5_uart_bridge` | PlatformIO firmware for the first-generation M5StickC Plus USB/UART bridge |
| `outputs/cyton_control_gui` | Python/Tkinter desktop controller with per-servo and grouped control, telemetry, and a big STOP |
| `outputs/cyton_ik` | Inverse-kinematics controller: target end-effector position + orientation, 3-D preview, coordinated moves, calibration (numpy) |
| `outputs/dynamixel_safe_scan` | Arduino IDE sketch: PING-only bus discovery |
| `outputs/cyton_readonly_inventory` | Arduino IDE sketch: read-only servo configuration/telemetry dump |
| `outputs/dynamixel_slow_test` | Arduino IDE sketch: low-torque single-servo movement test |

Full hardware notes, wiring, the gateway command protocol, setup steps for each
platform, and safety invariants are in [CLAUDE.md](CLAUDE.md).

## Quick start

```bash
python3 -m venv .venv
.venv/bin/pip install pyserial platformio numpy  # Windows: .venv\Scripts\pip
outputs/cyton_control_gui/launch_gui.sh          # joint-space GUI  (Windows: launch_gui.bat)
outputs/cyton_ik/launch_ik_gui.sh                # Cartesian IK GUI (Windows: launch_ik_gui.bat)
```

Always begin a session with `STOP` and confirm every servo reports `TORQUE 0`
before enabling anything.
