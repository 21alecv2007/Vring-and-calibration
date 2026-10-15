"""Safe desktop controller for the M5StickC/Uno/Cyton DYNAMIXEL gateway."""

from __future__ import annotations

import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


SERVO_IDS = tuple(range(8, 16))
MODEL_NAMES = {12: "AX-12A", 29: "MX-28", 310: "MX-64"}
STATUS_RE = re.compile(
    r"^ID (\d+) MODEL (\S+) POS (-?\d+) GOAL (-?\d+) TORQUE (\d+) "
    r"MOVING (\d+) VOLT_X10 (-?\d+) TEMP_C (-?\d+)$"
)
SCAN_RE = re.compile(r"^ID (\d+) MODEL (\S+)$")
LIMITS_RE = re.compile(r"^ID (\d+) CW (-?\d+) CCW (-?\d+)$")

# Device names / descriptions that look like a USB-serial bridge (M5StickC Plus
# uses an FTDI or CH9102 part). Works for Windows COMx, macOS /dev/cu.*, and
# Linux /dev/tty*.
USB_SERIAL_HINT = re.compile(
    r"usbserial|usbmodem|wchusbserial|SLAB_USBtoUART|ttyUSB|ttyACM|FTDI|FT232|CP210|CH9102|CH340|M5|Silicon Labs|USB.?Serial",
    re.IGNORECASE,
)
IGNORED_PORT_HINT = re.compile(r"Bluetooth|debug-console|Jabra|JLab", re.IGNORECASE)


def preferred_port(ports: list) -> str | None:
    """Pick the most likely M5 bridge port, or None when nothing plausible exists."""
    candidates = [
        p for p in ports
        if not IGNORED_PORT_HINT.search(f"{p.device} {p.description or ''}")
    ]
    for p in candidates:
        if USB_SERIAL_HINT.search(f"{p.device} {p.description or ''} {p.manufacturer or ''}"):
            return p.device
    return None


REPLY_TIMEOUT_S = 1.5   # a gateway command with no terminating reply is abandoned after this


class SerialLink:
    """Serial port + a strict one-command-at-a-time queue.

    The Uno talks to the M5 over SoftwareSerial, whose transmit routine blocks
    interrupts. A host command arriving while the Uno is still printing the
    previous reply (a STATUS report takes a few hundred ms) is garbled and
    surfaces as "ERR unknown_command" — or, for STOP, is silently lost. So
    commands are queued and the next one is written only after the gateway's
    terminating reply (OK…, ERR…, PONG) or a timeout. STOP jumps the queue and
    discards pending motion commands; coalesced commands (STATUS, MOVE per
    servo, GROUP) replace an older pending copy instead of piling up.
    """

    TERMINATORS = ("OK", "ERR", "PONG", "READY")

    def __init__(self, events: queue.Queue[tuple[str, str]]) -> None:
        self.events = events
        self.port: serial.Serial | None = None
        self.thread: threading.Thread | None = None
        self.running = False
        self.lock = threading.Lock()
        self.pending: list[tuple[str, str | None]] = []   # (command, coalesce_key)
        self.in_flight: str | None = None
        self.sent_at = 0.0

    @property
    def connected(self) -> bool:
        return bool(self.port and self.port.is_open and self.running)

    def connect(self, port_name: str) -> None:
        self.disconnect()
        port = serial.Serial(
            port_name,
            115200,
            timeout=0.15,
            write_timeout=0.5,
            rtscts=False,
            dsrdtr=False,
        )
        port.dtr = False
        port.rts = False
        self.port = port
        self.pending.clear()
        self.in_flight = None
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.running = False
        port, self.port = self.port, None
        with self.lock:
            self.pending.clear()
            self.in_flight = None
        if port:
            try:
                port.close()
            except serial.SerialException:
                pass

    def flush_line(self) -> None:
        """Write a bare newline immediately, terminating any junk in the Uno's line buffer."""
        if not self.connected or not self.port:
            raise serial.SerialException("Not connected")
        with self.lock:
            self.port.write(b"\n")
            self.port.flush()

    def send(self, command: str, *, coalesce: str | None = None, priority: bool = False) -> None:
        """Queue a command. Raises SerialException when not connected."""
        if not self.connected or not self.port:
            raise serial.SerialException("Not connected")
        command = command.strip()
        if not command:
            return
        with self.lock:
            if priority:
                # STOP: drop every pending motion command, go first.
                self.pending = [(c, k) for c, k in self.pending if k is None or k in ("STATUS", "LIMITS", "SCAN")]
                self.pending.insert(0, (command, coalesce))
            else:
                if coalesce is not None:
                    self.pending = [(c, k) for c, k in self.pending if k != coalesce]
                self.pending.append((command, coalesce))
            self._dispatch_locked()

    def pending_count(self) -> int:
        with self.lock:
            return len(self.pending) + (1 if self.in_flight else 0)

    def tick(self) -> None:
        """Call periodically: abandons a command whose reply never came."""
        with self.lock:
            if self.in_flight is not None and time.monotonic() - self.sent_at > REPLY_TIMEOUT_S:
                self.events.put(("timeout", self.in_flight))
                self.in_flight = None
                self._dispatch_locked()

    def _dispatch_locked(self) -> None:
        if self.in_flight is not None or not self.pending or not self.port:
            return
        command, _ = self.pending.pop(0)
        try:
            self.port.write((command + "\n").encode("ascii"))
            self.port.flush()
        except (serial.SerialException, OSError) as exc:
            self.events.put(("error", str(exc)))
            return
        self.in_flight = command
        self.sent_at = time.monotonic()
        self.events.put(("tx", command))

    def _reader(self) -> None:
        assert self.port is not None
        port = self.port
        try:
            while self.running and port.is_open:
                data = port.readline()
                if data:
                    line = data.decode("ascii", errors="replace").strip()
                    if line:
                        self.events.put(("rx", line))
                        if line.startswith(self.TERMINATORS):
                            with self.lock:
                                self.in_flight = None
                                self._dispatch_locked()
        except (serial.SerialException, OSError) as exc:
            if self.running:
                self.events.put(("error", str(exc)))


class ServoRow:
    def __init__(self, app: "CytonControlApp", parent: ttk.Frame, row: int, servo_id: int) -> None:
        self.app = app
        self.id = servo_id
        self.model = "—"
        self.present = 0
        self.goal = 0
        self.cw = 0
        self.ccw = 4095 if servo_id != 15 else 1023
        self.torque = False
        self.initialized = False
        self.suppress_scale = False
        self.live_after: str | None = None
        self.last_live_sent = 0.0

        self.id_label = ttk.Label(parent, text=f"ID {servo_id}\n—", width=11)
        self.id_label.grid(row=row, column=0, padx=(4, 8), pady=5, sticky="w")

        self.position_var = tk.StringVar(value="—")
        ttk.Label(parent, textvariable=self.position_var, width=8, anchor="center").grid(
            row=row, column=1, padx=3
        )

        self.goal_var = tk.StringVar(value="0")
        self.goal_spin = ttk.Spinbox(
            parent,
            from_=self.cw,
            to=self.ccw,
            textvariable=self.goal_var,
            width=7,
            command=self._spin_changed,
        )
        self.goal_spin.grid(row=row, column=2, padx=3)
        self.goal_spin.bind("<Return>", lambda _event: self._entry_commit())
        self.goal_spin.bind("<FocusOut>", lambda _event: self._entry_commit())

        self.scale = ttk.Scale(
            parent,
            from_=self.cw,
            to=self.ccw,
            orient="horizontal",
            command=self._scale_changed,
        )
        self.scale.grid(row=row, column=3, padx=5, sticky="ew")

        self.speed_var = tk.IntVar(value=10)
        ttk.Spinbox(parent, from_=1, to=100, textvariable=self.speed_var, width=5).grid(
            row=row, column=4, padx=3
        )

        self.torque_button = ttk.Button(parent, text="Enable", width=9, command=self.toggle_torque)
        self.torque_button.grid(row=row, column=5, padx=3)
        ttk.Button(parent, text="Move", width=7, command=self.move).grid(row=row, column=6, padx=3)
        ttk.Button(parent, text="−10", width=5, command=lambda: self.nudge(-10)).grid(
            row=row, column=7, padx=2
        )
        ttk.Button(parent, text="+10", width=5, command=lambda: self.nudge(10)).grid(
            row=row, column=8, padx=2
        )

        self.health_var = tk.StringVar(value="offline")
        ttk.Label(parent, textvariable=self.health_var, width=19).grid(
            row=row, column=9, padx=(7, 3), sticky="w"
        )

    def set_limits(self, cw: int, ccw: int) -> None:
        if ccw <= cw:
            return
        self.cw, self.ccw = cw, ccw
        self.scale.configure(from_=cw, to=ccw)
        self.goal_spin.configure(from_=cw, to=ccw)

    def update_status(
        self,
        model: str,
        position: int,
        reported_goal: int,
        torque: bool,
        moving: bool,
        voltage_x10: int,
        temperature: int,
    ) -> None:
        try:
            number = int(model)
            self.model = MODEL_NAMES.get(number, f"model {number}")
        except ValueError:
            self.model = model
        self.id_label.configure(text=f"ID {self.id}\n{self.model}")
        self.present = position
        self.position_var.set(str(position))
        self.torque = torque
        self.torque_button.configure(text="Disable" if torque else "Enable")
        state = "moving" if moving else "idle"
        self.health_var.set(f"{voltage_x10 / 10:.1f} V  {temperature}°C  {state}")
        if not self.initialized:
            self.set_goal(position)
            self.initialized = True
        self.app.refresh_arm_state()

    def set_goal(self, value: int) -> None:
        value = max(self.cw, min(self.ccw, int(value)))
        self.goal = value
        self.goal_var.set(str(value))
        self.suppress_scale = True
        self.scale.set(value)
        self.suppress_scale = False

    def _valid_goal(self) -> int | None:
        try:
            value = int(self.goal_var.get())
        except ValueError:
            self.app.set_status(f"ID {self.id}: goal must be an integer", error=True)
            return None
        if not self.cw <= value <= self.ccw:
            self.app.set_status(f"ID {self.id}: goal must be within {self.cw}–{self.ccw}", error=True)
            return None
        return value

    def _valid_speed(self) -> int | None:
        try:
            speed = int(self.speed_var.get())
        except (ValueError, tk.TclError):
            speed = 0
        if not 1 <= speed <= 100:
            self.app.set_status("Speed must be from 1 to 100", error=True)
            return None
        return speed

    def _scale_changed(self, raw: str) -> None:
        if self.suppress_scale:
            return
        self.goal = int(round(float(raw)))
        self.goal_var.set(str(self.goal))
        if self.app.live_var.get() and self.torque:
            self._request_live_move()

    def _request_live_move(self) -> None:
        """Send at most 10 commands/second while retaining the newest goal."""
        if self.live_after is not None:
            return
        elapsed = time.monotonic() - self.last_live_sent
        remaining_ms = max(0, int((0.100 - elapsed) * 1000))
        if remaining_ms == 0:
            self._send_live_move()
        else:
            self.live_after = self.app.root.after(remaining_ms, self._send_live_move)

    def _send_live_move(self) -> None:
        self.live_after = None
        if not self.app.live_var.get() or not self.torque:
            return
        goal, speed = self._valid_goal(), self._valid_speed()
        if goal is None or speed is None:
            return
        if self.app.send(f"MOVE {self.id} {goal} {speed}", quiet=True, coalesce=f"MOVE {self.id}"):
            self.last_live_sent = time.monotonic()

    def _spin_changed(self) -> None:
        self._entry_commit()

    def _entry_commit(self) -> None:
        value = self._valid_goal()
        if value is not None:
            self.set_goal(value)

    def toggle_torque(self) -> None:
        self.app.send(f"{'DISABLE' if self.torque else 'ENABLE'} {self.id}")
        self.app.root.after(350, lambda: self.app.send("STATUS", quiet=True))

    def move(self) -> None:
        goal, speed = self._valid_goal(), self._valid_speed()
        if goal is None or speed is None:
            return
        if not self.torque:
            self.app.set_status(f"ID {self.id}: enable torque first", error=True)
            return
        self.app.send(f"MOVE {self.id} {goal} {speed}")

    def nudge(self, amount: int) -> None:
        self.set_goal(self.goal + amount)
        self.move()


class CytonControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Cyton Robot Controller")
        self.root.geometry("1240x790")
        self.root.minsize(1010, 650)
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.link = SerialLink(self.events)
        self.closing = False
        self.poll_enabled = tk.BooleanVar(value=True)
        self.live_var = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar(value="")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="Torque is off. Connect to begin.")

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.stop_all())
        self.root.after(50, self._drain_events)
        self.root.after(1400, self._poll_status)
        self.root.after(300, self.connect)

    def _build_ui(self) -> None:
        style = ttk.Style()
        # Derive from the platform default font so it looks native on
        # Windows, macOS, and Linux instead of assuming Segoe UI exists.
        base = tkfont.nametofont("TkDefaultFont")
        family = base.actual("family")
        style.configure("Stop.TButton", font=(family, 12, "bold"), padding=9)
        style.configure("Header.TLabel", font=(family, 16, "bold"))

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        top = ttk.Frame(outer)
        top.pack(fill="x")
        ttk.Label(top, text="Cyton Robot Controller", style="Header.TLabel").pack(side="left")
        ttk.Label(top, textvariable=self.connection_var).pack(side="right", padx=(8, 0))

        connection = ttk.Frame(outer)
        connection.pack(fill="x", pady=(10, 8))
        ttk.Label(connection, text="M5 serial port:").pack(side="left")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=28, state="normal")
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(connection, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(connection, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=6)
        ttk.Checkbutton(connection, text="Poll telemetry", variable=self.poll_enabled).pack(
            side="left", padx=(18, 4)
        )
        ttk.Checkbutton(
            connection,
            text="Dynamic sliders (10 Hz)",
            variable=self.live_var,
            command=self._dynamic_mode_changed,
        ).pack(side="left", padx=4)
        ttk.Label(connection, text="Esc = STOP").pack(side="right")

        safety = ttk.Frame(outer)
        safety.pack(fill="x", pady=(2, 10))
        ttk.Button(safety, text="STOP — TORQUE OFF ALL", style="Stop.TButton", command=self.stop_all).pack(
            side="left", fill="x", expand=True
        )
        ttk.Button(safety, text="Enable all (hold current)", command=self.enable_all).pack(
            side="left", padx=(10, 0)
        )
        ttk.Button(safety, text="Move all to goals", command=self.move_all).pack(side="left", padx=(6, 0))
        ttk.Button(safety, text="Goals ← current", command=self.copy_current).pack(side="left", padx=(6, 0))

        table = ttk.LabelFrame(outer, text="Servos", padding=7)
        table.pack(fill="both", expand=True)
        table.columnconfigure(3, weight=1)
        headers = ("Servo", "Position", "Goal", "Position slider", "Speed", "Torque", "", "", "", "Health")
        for column, text in enumerate(headers):
            ttk.Label(table, text=text, anchor="center").grid(row=0, column=column, padx=3, sticky="ew")
        ttk.Separator(table).grid(row=1, column=0, columnspan=10, sticky="ew", pady=(3, 2))
        self.rows = {
            servo_id: ServoRow(self, table, index + 2, servo_id)
            for index, servo_id in enumerate(SERVO_IDS)
        }

        bottom = ttk.Frame(outer)
        bottom.pack(fill="x", pady=(9, 0))
        ttk.Label(bottom, textvariable=self.status_var).pack(side="left")

        console_frame = ttk.LabelFrame(outer, text="Gateway log / manual command", padding=6)
        console_frame.pack(fill="x", pady=(8, 0))
        self.log = tk.Text(console_frame, height=6, wrap="none", state="disabled")
        self.log.pack(fill="x")
        command_line = ttk.Frame(console_frame)
        command_line.pack(fill="x", pady=(5, 0))
        self.manual_var = tk.StringVar()
        entry = ttk.Entry(command_line, textvariable=self.manual_var)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(command_line, text="Send", command=self.send_manual).pack(side="left", padx=(6, 0))
        ttk.Button(command_line, text="Help", command=lambda: self.send("HELP")).pack(side="left", padx=(6, 0))

    def _refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        labels = [f"{item.device} — {item.description}" for item in ports]
        self.port_combo["values"] = labels
        current = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not current or current not in {p.device for p in ports}:
            preferred = preferred_port(ports)
            if preferred:
                self.port_var.set(preferred)
            elif labels:
                self.set_status("No USB-serial bridge detected; pick the M5 port manually", error=True)
            else:
                self.set_status("No serial ports found; plug in the M5 and press Refresh", error=True)

    def connect(self) -> None:
        if self.link.connected:
            self.link.disconnect()
            self.connection_var.set("Disconnected")
            self.connect_button.configure(text="Connect")
            self.set_status("Disconnected; servo state unchanged.")
            return
        port_name = self.port_var.get().split()[0].strip() if self.port_var.get() else ""
        if not port_name:
            self.set_status("Choose the M5 serial port, then press Connect", error=True)
            return
        try:
            self.link.connect(port_name)
        except (serial.SerialException, OSError) as exc:
            self.set_status(f"Could not open {port_name}: {exc}", error=True)
            return
        self.connection_var.set(f"Connected: {port_name} @ 115200")
        self.connect_button.configure(text="Disconnect")
        self.set_status("Connected. Initializing with torque off…")
        self.root.after(900, self._initialize_gateway)

    def _initialize_gateway(self) -> None:
        if not self.link.connected:
            return
        # Opening the port resets the M5; terminate any junk left in the Uno's
        # line buffer before the safety STOP. The queue then paces the rest.
        try:
            self.link.flush_line()
        except (serial.SerialException, OSError):
            return
        self.send("STOP", priority=True)
        self.send("STOP", quiet=True)
        self.send("SCAN", quiet=True, coalesce="SCAN")
        for servo_id in SERVO_IDS:
            self.send(f"LIMITS {servo_id}", quiet=True, coalesce=f"LIMITS {servo_id}")
        self.send("STATUS", quiet=True, coalesce="STATUS")

    def send(self, command: str, quiet: bool = False, *, coalesce: str | None = None,
             priority: bool = False) -> bool:
        try:
            self.link.send(command, coalesce=coalesce, priority=priority)
            if not quiet:
                self.set_status(f"Sent: {command}")
            return True
        except (serial.SerialException, OSError) as exc:
            self.set_status(str(exc), error=True)
            return False

    def send_manual(self) -> None:
        command = self.manual_var.get().strip()
        if command:
            self.send(command)
            self.manual_var.set("")

    def stop_all(self) -> None:
        if self.send("STOP", priority=True):
            self.live_var.set(False)
            self.set_status("STOP sent — all servo torque is being disabled")
            self.root.after(400, lambda: self.send("STATUS", quiet=True))

    def _dynamic_mode_changed(self) -> None:
        if self.live_var.get():
            self.set_status("Dynamic mode ON — enabled servos follow their sliders at up to 10 Hz")
        else:
            for row in self.rows.values():
                if row.live_after is not None:
                    self.root.after_cancel(row.live_after)
                    row.live_after = None
            self.set_status("Dynamic mode OFF")

    def enable_all(self) -> None:
        if not messagebox.askyesno(
            "Enable all servos?",
            "Each servo will first copy its present position to its goal, then enable torque. "
            "The arm should hold its current pose.",
        ):
            return
        self.send("ENABLE ALL")
        self.root.after(900, lambda: self.send("STATUS", quiet=True))

    def move_all(self) -> None:
        if not all(row.torque for row in self.rows.values()):
            self.set_status("Enable all servos before a grouped move", error=True)
            return
        goals: list[int] = []
        for row in self.rows.values():
            goal = row._valid_goal()
            if goal is None:
                return
            goals.append(goal)
        speed = min(row._valid_speed() or 1 for row in self.rows.values())
        command = "GROUP " + " ".join([str(speed), *(str(goal) for goal in goals)])
        self.send(command)

    def copy_current(self) -> None:
        for row in self.rows.values():
            if row.initialized:
                row.set_goal(row.present)
        self.set_status("All goals copied from current positions")

    def refresh_arm_state(self) -> None:
        if all(row.initialized for row in self.rows.values()):
            enabled = sum(row.torque for row in self.rows.values())
            self.set_status(f"Telemetry live — {enabled}/8 servos torque-enabled")

    def _poll_status(self) -> None:
        if not self.closing and self.link.connected and self.poll_enabled.get():
            self.send("STATUS", quiet=True, coalesce="STATUS")
        if not self.closing:
            self.root.after(1600, self._poll_status)

    def _drain_events(self) -> None:
        self.link.tick()
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "rx":
                    self._log_line(f"< {value}")
                    self._handle_line(value)
                elif kind == "tx":
                    self._log_line(f"> {value}")
                elif kind == "timeout":
                    self._log_line(f"! no reply to {value}")
                elif kind == "error":
                    self.set_status(value, error=True)
                    self.link.disconnect()
                    self.connection_var.set("Disconnected")
                    self.connect_button.configure(text="Connect")
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(50, self._drain_events)

    def _handle_line(self, line: str) -> None:
        match = STATUS_RE.match(line)
        if match:
            servo_id = int(match.group(1))
            row = self.rows.get(servo_id)
            if row:
                row.update_status(
                    model=match.group(2),
                    position=int(match.group(3)),
                    reported_goal=int(match.group(4)),
                    torque=bool(int(match.group(5))),
                    moving=bool(int(match.group(6))),
                    voltage_x10=int(match.group(7)),
                    temperature=int(match.group(8)),
                )
            return
        match = SCAN_RE.match(line)
        if match:
            servo_id = int(match.group(1))
            row = self.rows.get(servo_id)
            if row:
                model_text = match.group(2)
                try:
                    row.model = MODEL_NAMES.get(int(model_text), f"model {model_text}")
                except ValueError:
                    row.model = model_text
                row.id_label.configure(text=f"ID {servo_id}\n{row.model}")
            return
        match = LIMITS_RE.match(line)
        if match:
            servo_id = int(match.group(1))
            row = self.rows.get(servo_id)
            if row:
                row.set_limits(int(match.group(2)), int(match.group(3)))
            return
        if line.startswith("ERR"):
            self.set_status(line, error=True)
        elif line.startswith("OK") or line.startswith("PONG"):
            self.set_status(line)

    def _log_line(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 180:
            self.log.delete("1.0", "40.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_status(self, text: str, error: bool = False) -> None:
        self.status_var.set(("⚠ " if error else "") + text)

    def close(self) -> None:
        self.closing = True
        if self.link.connected:
            try:
                self.link.send("STOP", priority=True)
                self.root.after(400, self._finish_close)
                return
            except (serial.SerialException, OSError):
                pass
        self._finish_close()

    def _finish_close(self) -> None:
        self.link.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = CytonControlApp(root)
    if len(sys.argv) > 1:
        app.port_var.set(sys.argv[1])
    root.mainloop()


if __name__ == "__main__":
    main()
