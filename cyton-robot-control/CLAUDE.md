# Cyton Robot Control Project Handoff

## Objective

Continue developing and controlling a Robai Cyton arm from a Windows, macOS,
or Linux computer. The physical
control path is:

```text
Mac/PC USB serial
    -> M5StickC Plus (first generation)
    -> 57,600-baud 3.3 V UART
    -> Arduino Uno + ROBOTIS DYNAMIXEL Shield
    -> 1 Mbps Protocol 1.0 DYNAMIXEL bus
    -> Cyton arm, IDs 8 through 15
```

The M5 is a transparent USB/UART bridge. The Uno owns the half-duplex
DYNAMIXEL bus and implements a safety-oriented text command protocol.

## Hardware identification

- Controller: Arduino Uno with ROBOTIS DYNAMIXEL Shield
- Bridge: original M5StickC Plus with ESP32-PICO-D4 and AXP192
- Arm: Robai Cyton, eight DYNAMIXEL servos in a daisy chain
- Bus: DYNAMIXEL Protocol 1.0 at 1,000,000 baud
- Detected devices:

| ID | Model number | Model |
|---:|---:|---|
| 8 | 29 | MX-28 |
| 9 | 310 | MX-64 |
| 10 | 29 | MX-28 |
| 11 | 29 | MX-28 |
| 12 | 29 | MX-28 |
| 13 | 29 | MX-28 |
| 14 | 29 | MX-28 |
| 15 | 12 | AX-12A gripper |

Typical observed supply voltage was 11.6–12.1 V with no load. Temperatures were
roughly 30–41 °C during testing.

## Physical wiring

Use the M5StickC Plus bottom HAT header:

```text
M5 G26 (TX) --------------------------> Shield/Uno SW UART RX (Arduino D7)

Shield/Uno SW UART TX (Arduino D8) -- 1 kOhm --+--> M5 G36 (RX)
                                                |
                                               2 kOhm
                                                |
M5 GND -----------------------------------------+---- Uno/shield GND
```

- The 1 kOhm / 2 kOhm divider reduces the Uno's 5 V TX signal to about 3.33 V.
- G36 is input-only and is used as the M5 UART RX pin.
- G26 is the M5 UART TX pin.
- Do not connect M5 5VOUT, 5VIN, 3V3, BAT, or G0.
- Power the M5 through USB or its battery.
- Power the Uno separately.
- Use a dedicated regulated 12 V supply for the DYNAMIXEL chain through the
  shield power terminal. Keep the shield VIN jumper removed.
- During normal operation, the shield UART switch stays at `DYNAMIXEL` and the
  DYNAMIXEL power switch stays on.
- Only move the UART switch to `UPLOAD` while flashing the Uno.

## Firmware

### Arduino Uno gateway

Location: `outputs/cyton_gateway_uno`

- PlatformIO environment: `uno`
- M5/host link: SoftwareSerial RX D7, TX D8, 57,600 baud
- DYNAMIXEL link: hardware UART, Protocol 1.0, 1 Mbps
- V2 (2026-09-04): `PONG CYTON_GATEWAY_V2`. `STOP` is checked for between
  servos inside SCAN, STATUS, ENABLE ALL and GROUP and aborts them
  (`OK STOP TORQUE_OFF_ALL ABORTED`); lines containing non-printable bytes
  (garbled by a reply in progress) are dropped with `ERR garbled_line_ignored`
  instead of being parsed; `HELP` ends with `OK HELP` so a paced bridge can
  detect its end.
- Torque is disabled on every boot.
- DYNAMIXEL reads and writes retry up to three times.
- Movement is rejected outside each servo's stored CW/CCW limits.
- Moving speed is restricted to raw values 1–100.
- Enabling torque first copies present position into goal position, preventing
  an initial jump.

### M5StickC Plus bridge

Location: `outputs/m5_uart_bridge`

- PlatformIO environment: `m5stick-c`
- USB serial: 115,200 baud
- Uno UART: 57,600 baud, RX G36, TX G26
- V2 (2026-09-04, flashed and verified): line-paced, no longer a transparent
  byte bridge. Host lines are queued (16 deep) and forwarded one at a time;
  the next goes out only after the Uno's terminating reply (`OK…`, `ERR…`,
  `PONG`, `READY`) or 1.5 s. `STOP` bypasses the queue, purges pending
  MOVE/GROUP/ENABLE lines, and is resent every 120 ms until `OK STOP`
  arrives. On boot (which happens whenever the host opens the port) it sends a
  bare newline and a `STOP`. `M5` (never forwarded) answers with queue state.
  Uno -> host bytes pass through unchanged. Bridge-local errors are prefixed
  `M5 ERR`.
- This is the first-generation M5StickC Plus, not the Plus2.

### Desktop GUI

Location: `outputs/cyton_control_gui`

- Python/Tkinter application using pyserial
- Automatically connects, sends `STOP`, scans IDs, reads limits, and polls
  telemetry.
- Per-servo enable/disable, goal, speed, nudge, and move controls
- Group enable and group movement
- Dynamic slider mode streams the newest position at up to 10 Hz.
- The large STOP button and Escape key disable torque on all servos.
- Closing the GUI also sends `STOP`.

### IK (Cartesian) controller

Location: `outputs/cyton_ik` (see its README for the model table, modes, and
the calibration procedure).

- `cyton_kinematics.py`: numpy FK / Jacobian / damped-least-squares IK for
  the Cyton Gamma 300 geometry (community URDF: 7 revolute joints, 532 mm
  reach, tool point between the gripper fingers). Modes: full 6-D, position +
  approach axis (roll free), position only. Servo raw <-> radian calibration
  is a JSON file (`cyton_calibration.json`) with per-joint ID, zero offset and
  sign.
- `cyton_ik_gui.py`: Tkinter controller on the same gateway protocol: live FK
  pose, target XYZ (mm) + roll/pitch/yaw (deg) with nudges, a draggable
  translate gizmo on the target in the 3-D view (centre knob = XY plane,
  axis knobs = single axis, wheel = Z) plus keyboard jog, 3-D wireframe of
  current vs solved pose, coordinated per-joint-speed moves, ~8 Hz tracking
  loop (sends whenever the solution changes, offers to enable torque, pauses
  visibly when it cannot), gripper, calibration panel. STOP / Escape / close
  all send `STOP`.
- `test_kinematics.py`: run it after any change to the model or solver.
- Joint -> servo mapping = chain order (shoulder_roll = ID 8 … wrist_roll =
  ID 14, gripper 15), sign +1, zero ≈ raw 2048. Source: the ROS
  dynamixel_controllers yaml for the Cyton Gamma 300 (IDs 0-7 there, same
  order, no flipped joints, zero ticks 2045-2070). Per-joint zero offsets are
  still worth refining with "Zero ALL here" in the straight-up pose; the arm
  has not yet been driven through IK from this Mac.

## Gateway commands

Commands are newline-terminated ASCII sent to the M5 USB serial port at
115,200 baud:

```text
PING
SCAN
STATUS
STATUS 15
LIMITS 15
ENABLE 15
ENABLE ALL
DISABLE 15
DISABLE ALL
MOVE 15 300 50
GROUP 10 p8 p9 p10 p11 p12 p13 p14 p15
STOP
HELP
```

`MOVE` requires that the target servo already be enabled. `GROUP` requires all
eight servos to be enabled and takes one raw goal for each ID 8–15.

**One command in flight at a time.** The Uno's SoftwareSerial transmit blocks
interrupts, so a host command that arrives while the Uno is still printing a
reply (a full `STATUS` report takes ~300 ms) is silently lost or garbled into
`ERR unknown_command_use_HELP`. Measured 2026-09-04: a `PING` sent 250 ms
after `STATUS` vanished; at 400 ms it produced the ERR. This applies to
`STOP` too. Fixed at the firmware level the same day: the M5 bridge V2 paces
host lines itself (verified: a `PING` at any offset after `STATUS` now always
answers), so plain terminals are safe, and the Uno V2 checks for a `STOP`
between servos mid-report. That check is best-effort: SoftwareSerial
reception is also disturbed by the 1 Mbps DYNAMIXEL UART interrupts, so a
STOP copy arriving during a transaction is usually corrupted and it is a
120 ms retry that lands. Measured with the bus live: a full `STATUS` takes
~150 ms and a STOP sent at any point during it is acknowledged within 250 ms;
the bridge then holds the next command ≤200 ms for late acks of retried
copies. The arm is never left energised because the bridge keeps retrying
until `OK STOP`. A reliable mid-command abort would need the host link on
AltSoftSerial (hardware input capture), i.e. rewiring M5 G26 -> Uno D8 and
Uno D9 -> divider -> M5 G36. The host GUIs additionally share a `SerialLink`
queue (one in flight, `STOP` priority, `STATUS`/`GROUP`/`MOVE` coalescing)
as belt and braces and so they still behave with a V1 bridge.

## Gripper findings

- Gripper ID15 has stored angle limits CW 0 and CCW 515.
- Do not expand that persistent limit without checking the mechanism first.
- The gripper was initially mechanically stuck. While stuck, a move toward 515
  stopped near 160 and local voltage fell to about 10.7 V, indicating high load.
- The mechanism was subsequently freed and reported working.
- The GUI's former default speed of 10 is only about 1.1 RPM on an AX-12A.
  For gripper testing, start around raw speed 50 and increase only while watching
  for binding, voltage sag, or rising temperature.

## Setup (Windows, macOS, Linux)

The repo is cross-platform. Serial ports are auto-detected everywhere; nothing
is hard-coded to `COMx` or `/dev/cu.*`.

1. Create a virtual environment at the repo root and install the tooling:

   ```bash
   python3 -m venv .venv
   .venv/bin/pip install pyserial platformio numpy  # Windows: .venv\Scripts\pip
   ```

   On macOS with Homebrew Python, Tk is a separate package:
   `brew install python-tk@3.14` (match your Python minor version).

2. Start the GUI with the platform launcher, which uses `.venv` when present:

   - macOS: double-click `outputs/cyton_control_gui/launch_gui.command` or run
     `outputs/cyton_control_gui/launch_gui.sh`
   - Linux: `outputs/cyton_control_gui/launch_gui.sh`
   - Windows: `outputs\cyton_control_gui\launch_gui.bat`

   The GUI selects the first USB-serial bridge it finds (FTDI/CP210x/CH9102,
   `usbserial`, `ttyUSB`, ...) and skips Bluetooth/debug ports. Pass a port as
   the first argument to force one, e.g. `launch_gui.sh /dev/cu.usbserial-0001`.

3. Build or flash firmware from each firmware directory. `upload_port` is
   omitted from both `platformio.ini` files so PlatformIO auto-detects the
   board; override with `--upload-port` when several boards are attached:

   ```bash
   cd outputs/cyton_gateway_uno && ../../.venv/bin/pio run -t upload
   cd outputs/m5_uart_bridge   && ../../.venv/bin/pio run -t upload
   ```

   The Uno build pulls `DynamixelShield` and `Dynamixel2Arduino` from the
   PlatformIO registry, so no Arduino IDE library install is needed.

4. To upload the Uno, physically set the shield UART switch to `UPLOAD`, flash
   `outputs/cyton_gateway_uno`, then return the switch to `DYNAMIXEL`.

5. Serial port names by platform: Windows `COMx`; macOS `/dev/cu.usbserial-*`
   (FTDI/CP210x) or `/dev/cu.usbmodem*` (Uno, CH9102); Linux `/dev/ttyUSB*`
   or `/dev/ttyACM*` (add yourself to the `dialout` group).

## Safety invariants

- Begin every session with `STOP` and confirm telemetry reports `TORQUE 0`.
- Never energize the complete arm unless it is supported and its workspace is
  clear.
- Use `ENABLE` before `MOVE`; it holds the present position first.
- Keep dynamic slider mode off until the intended servo and direction are
  verified.
- Do not raise gripper torque or angle limits to overcome a mechanical jam.
- Never connect the 12 V servo supply to an M5 or Arduino logic-power pin.

## Last verified state

- M5 USB bridge and Uno gateway were both successfully flashed.
- Computer -> M5 -> Uno communication returned `PONG CYTON_GATEWAY_V1`.
- Three consecutive scans found all IDs 8–15 with correct model numbers.
- Full telemetry worked through the permanent M5 connection.
- The GUI ran successfully on Windows and held the M5 serial port.
- 2026-09-04: Mac -> M5 -> Uno link verified (`PONG`, `OK STOP`), servos
  reported OFFLINE at that moment (bus unpowered or UART switch on UPLOAD).
- 2026-09-04: M5 bridge V2 (line-paced) flashed and verified on the arm.
  Uno gateway V2 built; flashing needs the shield UART switch on UPLOAD.
- 2026-09-04: IK controller added (`outputs/cyton_ik`), unit-tested, GUI
  smoke-tested with synthetic telemetry. Servo calibration NOT yet done.
- 2026-09-04: project moved to macOS. Both firmwares build with PlatformIO
  (`uno`: 72% flash, `m5stick-c`: 20% flash) and the GUI starts on macOS with
  Tk 9 and correct port auto-detection. Not yet re-tested against the arm
  from the Mac.
- Normal shield switch position is `DYNAMIXEL`.
