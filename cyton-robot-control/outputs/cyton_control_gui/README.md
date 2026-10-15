# Cyton Robot Controller

Desktop GUI for the M5StickC Plus → Arduino Uno → DYNAMIXEL gateway.

## Start

Requires Python 3 with Tk and `pyserial` (the repo-level `.venv` is used
automatically by the launchers when it exists).

- Windows: double-click `launch_gui.bat`
- macOS: double-click `launch_gui.command`
- Linux / any shell: `./launch_gui.sh`
- Manual: `python cyton_control_gui.py [port]`

The application auto-selects the first USB-serial bridge (FTDI, CP210x,
CH9102, `usbserial`, `ttyUSB`, ...) at 115,200 baud and connects
automatically. An optional command-line argument forces a specific port.
It sends `STOP` during initialization and again when the window closes.

## Safety behavior

- Torque is never automatically enabled.
- `Enable all` asks for confirmation and the Uno copies each present position
  into its goal before enabling torque.
- The large STOP button and Escape key disable torque on IDs 8–15.
- Position limits come from each servo's configured CW/CCW registers.
- Speed is restricted to the Uno gateway's 1–100 range.
- Dynamic slider transmission is disabled by default. When enabled, an
  already-enabled servo follows its slider continuously at a rate-limited
  10 commands per second.

## Command pacing

The Uno cannot receive while it transmits (SoftwareSerial), so the GUI's
`SerialLink` sends one command at a time and waits for the gateway's
terminating reply (`OK…`, `ERR…`, `PONG`) or a 1.5 s timeout. `STOP` is
queued with priority and drops any pending motion commands; repeated
`STATUS`, `GROUP` and per-servo `MOVE` commands replace their pending copy.
The IK controller imports this class, so any new tool should reuse it.
