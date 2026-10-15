"""Cartesian (inverse kinematics) controller for the Cyton arm.

Talks to the same M5 -> Uno gateway as cyton_control_gui.py. Reads joint
positions from STATUS telemetry, shows the live tool pose from forward
kinematics, lets you type / nudge a target position (mm) and orientation
(roll-pitch-yaw, degrees), solves IK, previews the solution in a 3-D
wireframe, and moves the arm with coordinated per-joint speeds.

Usage: python cyton_ik_gui.py [serial_port]
"""

from __future__ import annotations

import math
import os
import queue
import sys
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import numpy as np
import serial
from serial.tools import list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "cyton_control_gui"))

import cyton_kinematics as ck  # noqa: E402
from cyton_control_gui import (  # noqa: E402
    LIMITS_RE, SCAN_RE, STATUS_RE, SerialLink, preferred_port,
)

STATUS_PERIOD_MS = 700
STREAM_PERIOD_MS = 120          # ~8 Hz when tracking
MX_RPM_PER_UNIT = 0.114         # MX moving-speed raw unit


GIZMO_LEN = 0.07          # metres, length of each axis handle
GIZMO_HIT_PX = 12         # pick radius for knobs / centre
AXIS_COLOURS = ("#ff5f5f", "#6fdc6f", "#6fa8ff")


class ArmView(tk.Canvas):
    """Orthographic 3-D wireframe of the arm with drag-to-orbit and a
    draggable translate gizmo on the IK target.

    - drag empty space: orbit
    - drag the gold centre knob: move the target in the ground (XY) plane
    - drag a red / green / blue knob: move along that world axis only
    - scroll wheel: move the target up / down (Z)
    """

    def __init__(self, parent: tk.Widget, on_target_drag=None, on_scroll=None, **kw) -> None:
        super().__init__(parent, bg="#1d1f24", highlightthickness=0, **kw)
        self.azimuth = math.radians(35)
        self.elevation = math.radians(25)
        self.scale = 420.0  # px per metre
        self.current: np.ndarray | None = None
        self.target: np.ndarray | None = None
        self.current_R: np.ndarray | None = None
        self.target_pose: tuple[np.ndarray, np.ndarray] | None = None
        self.tracking = False
        self.on_target_drag = on_target_drag      # callback(new_position_m)
        self.on_scroll = on_scroll                # callback(steps)
        self._drag: tuple[int, int] | None = None
        self._drag_mode = "orbit"                 # orbit | x | y | z | plane
        self._drag_origin: np.ndarray | None = None
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<B1-Motion>", self._drag_motion)
        self.bind("<ButtonRelease-1>", lambda _e: setattr(self, "_drag", None))
        self.bind("<MouseWheel>", self._wheel)
        self.bind("<Button-4>", lambda e: self._wheel(e, 1))
        self.bind("<Button-5>", lambda e: self._wheel(e, -1))
        self.bind("<Configure>", lambda _e: self.redraw())

    # ---- picking -------------------------------------------------------
    def _knobs(self) -> list[tuple[str, float, float]]:
        """Screen positions of the gizmo knobs: [(mode, x, y), ...]."""
        if self.target_pose is None:
            return []
        p, _ = self.target_pose
        out = [("plane", *self.project(p))]
        for mode, axis in zip(("x", "y", "z"), np.eye(3)):
            out.append((mode, *self.project(p + axis * GIZMO_LEN)))
        return out

    def _press(self, event: tk.Event) -> None:
        self._drag = (event.x, event.y)
        self._drag_mode = "orbit"
        self.focus_set()
        best = GIZMO_HIT_PX
        for mode, x, y in self._knobs():
            d = math.hypot(event.x - x, event.y - y)
            if d < best:
                best, self._drag_mode = d, mode
        if self._drag_mode != "orbit" and self.target_pose is not None:
            self._drag_origin = self.target_pose[0].copy()

    def _screen_axis(self, p: np.ndarray, axis: np.ndarray) -> np.ndarray:
        """Screen-space vector (px) of one metre along `axis` at point p."""
        x0, y0 = self.project(p)
        x1, y1 = self.project(p + axis)
        return np.array([x1 - x0, y1 - y0])

    def _drag_motion(self, event: tk.Event) -> None:
        if self._drag is None:
            return
        dx, dy = event.x - self._drag[0], event.y - self._drag[1]
        if self._drag_mode == "orbit":
            self._drag = (event.x, event.y)
            self.azimuth -= dx * 0.01
            self.elevation = max(-1.5, min(1.5, self.elevation + dy * 0.01))
            self.redraw()
            return
        if self.target_pose is None or self._drag_origin is None or self.on_target_drag is None:
            return
        # Total mouse motion since the press, mapped onto the chosen constraint.
        mouse = np.array([dx, dy], dtype=float)
        p0 = self._drag_origin
        if self._drag_mode in ("x", "y", "z"):
            axis = np.eye(3)[("x", "y", "z").index(self._drag_mode)]
            sv = self._screen_axis(p0, axis)
            denom = float(sv @ sv)
            if denom < 1e-6:
                return
            new_p = p0 + axis * float(mouse @ sv) / denom
        else:  # ground plane: solve a*ex + b*ey = mouse
            ex, ey = self._screen_axis(p0, np.array([1.0, 0, 0])), self._screen_axis(p0, np.array([0, 1.0, 0]))
            A = np.column_stack([ex, ey])
            if abs(np.linalg.det(A)) < 1e-3:
                return  # view is edge-on to the ground plane
            a, b = np.linalg.solve(A, mouse)
            new_p = p0 + np.array([a, b, 0.0])
        self.on_target_drag(new_p)

    def _wheel(self, event: tk.Event, steps: int | None = None) -> None:
        if self.on_scroll is None:
            return
        if steps is None:
            steps = 1 if event.delta > 0 else -1
        self.on_scroll(steps)

    def project(self, p: np.ndarray) -> tuple[float, float]:
        ca, sa = math.cos(self.azimuth), math.sin(self.azimuth)
        ce, se = math.cos(self.elevation), math.sin(self.elevation)
        x = ca * p[0] + sa * p[1]
        y = -sa * p[0] + ca * p[1]
        z = p[2]
        sx = y
        sy = -(z * ce - x * se)
        w, h = self.winfo_width(), self.winfo_height()
        return w / 2 + sx * self.scale, h * 0.78 + sy * self.scale

    def _line(self, a: np.ndarray, b: np.ndarray, **kw) -> None:
        self.create_line(*self.project(a), *self.project(b), **kw)

    def _triad(self, p: np.ndarray, R: np.ndarray, size: float, width: int, dash: tuple | None = None) -> None:
        for axis, colour in zip(range(3), ("#ff5f5f", "#6fdc6f", "#6fa8ff")):
            self._line(p, p + R[:, axis] * size, fill=colour, width=width, dash=dash)

    def redraw(self) -> None:
        self.delete("all")
        # ground grid, 10 cm cells
        for i in range(-3, 4):
            g = i * 0.1
            self._line(np.array([g, -0.3, 0]), np.array([g, 0.3, 0]), fill="#2d3038")
            self._line(np.array([-0.3, g, 0]), np.array([0.3, g, 0]), fill="#2d3038")
        self._triad(np.zeros(3), np.eye(3), 0.08, 2)
        if self.target is not None:
            for a, b in zip(self.target[:-1], self.target[1:]):
                self._line(a, b, fill="#c9a227", width=3, dash=(6, 4))
        if self.current is not None:
            for a, b in zip(self.current[:-1], self.current[1:]):
                self._line(a, b, fill="#e8e8e8", width=5)
            for p in self.current[1:-1]:
                x, y = self.project(p)
                self.create_oval(x - 5, y - 5, x + 5, y + 5, fill="#8ab4f8", outline="")
            if self.current_R is not None:
                self._triad(self.current[-1], self.current_R, 0.05, 3)
        if self.target_pose is not None:
            p, R = self.target_pose
            self._triad(p, R, 0.045, 2, dash=(3, 3))
            # translate gizmo: world-axis handles with knobs, plus centre knob
            for axis, colour in zip(np.eye(3), AXIS_COLOURS):
                tip = p + axis * GIZMO_LEN
                self._line(p, tip, fill=colour, width=2)
                kx, ky = self.project(tip)
                self.create_oval(kx - 6, ky - 6, kx + 6, ky + 6, fill=colour, outline="#1d1f24", width=1)
            x, y = self.project(p)
            self.create_oval(x - 9, y - 9, x + 9, y + 9, fill="#c9a227", outline="#fff3c4", width=2)
        self.create_text(8, 8, anchor="nw", fill="#9aa0a6", font=("TkDefaultFont", 10), justify="left",
                         width=max(200, self.winfo_width() - 330),
                         text="Drag empty space to orbit. Drag the gold knob to move the target in the ground plane, "
                              "a red/green/blue knob to move along X/Y/Z, wheel for Z. With the view focused: "
                              "arrows = X/Y, PgUp/PgDn = Z, Q/E yaw, A/D pitch, Z/C roll.")
        if self.tracking:
            self.create_text(self.winfo_width() - 8, 8, anchor="ne", fill="#ff6b6b",
                             font=("TkDefaultFont", 12, "bold"), text="TRACKING — the arm follows this target")


class IKApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Cyton IK Controller")
        self.root.geometry("1380x900")
        self.root.minsize(1150, 760)
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.link = SerialLink(self.events)
        self.closing = False
        self.cal = ck.load_calibration()
        self.servo_ids = self.cal.servo_ids

        # Robot state
        self.raw: dict[int, int] = {}
        self.torque: dict[int, bool] = {}
        self.models: dict[int, str] = {}
        self.cw: dict[int, int] = {}
        self.ccw: dict[int, int] = {}
        self.q = np.zeros(ck.DOF)
        self.have_pose = False
        self.lower, self.upper = ck.JOINT_LOWER.copy(), ck.JOINT_UPPER.copy()
        self.solution: ck.IKResult | None = None
        self._stream_after: str | None = None
        self._last_sent_raw: list[int] | None = None
        self._outside_limits: list[str] = []

        # UI variables
        self.port_var = tk.StringVar(value="")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="Torque is off. Connect to begin.")
        self.mode_var = tk.StringVar(value="full")
        self.speed_var = tk.IntVar(value=30)
        self.track_var = tk.BooleanVar(value=False)
        self.step_mm_var = tk.DoubleVar(value=10.0)
        self.step_deg_var = tk.DoubleVar(value=5.0)
        self.target_vars = {k: tk.DoubleVar(value=0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")}
        self.pose_var = tk.StringVar(value="—")
        self.solution_var = tk.StringVar(value="No solution yet")
        self.gripper_var = tk.IntVar(value=0)
        self.joint_vars = [tk.StringVar(value="—") for _ in range(ck.DOF)]
        self.sol_vars = [tk.StringVar(value="—") for _ in range(ck.DOF)]
        self.sign_vars = [tk.IntVar(value=j.sign) for j in self.cal.joints]
        self.id_vars = [tk.IntVar(value=j.servo_id) for j in self.cal.joints]

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _e: self.stop_all())
        self.root.after(50, self._drain_events)
        self.root.after(1200, self._poll_status)
        self.root.after(300, self.connect)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        base = tkfont.nametofont("TkDefaultFont").actual("family")
        style = ttk.Style()
        style.configure("Stop.TButton", font=(base, 12, "bold"), padding=8)
        style.configure("Header.TLabel", font=(base, 15, "bold"))

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        top = ttk.Frame(outer)
        top.pack(fill="x")
        ttk.Label(top, text="Cyton IK Controller", style="Header.TLabel").pack(side="left")
        ttk.Label(top, textvariable=self.connection_var).pack(side="right")

        conn = ttk.Frame(outer)
        conn.pack(fill="x", pady=(8, 6))
        ttk.Label(conn, text="M5 serial port:").pack(side="left")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=30)
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(conn, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=6)
        ttk.Label(conn, text="Esc = STOP").pack(side="right")

        safety = ttk.Frame(outer)
        safety.pack(fill="x", pady=(0, 8))
        ttk.Button(safety, text="STOP — TORQUE OFF ALL", style="Stop.TButton", command=self.stop_all).pack(
            side="left", fill="x", expand=True)
        ttk.Button(safety, text="Enable arm (hold pose)", command=self.enable_all).pack(side="left", padx=(8, 0))
        ttk.Button(safety, text="Disable arm", command=lambda: self.send("DISABLE ALL")).pack(side="left", padx=(6, 0))

        cmdline = ttk.Frame(outer)
        cmdline.pack(side="bottom", fill="x", pady=(4, 0))
        self.manual_var = tk.StringVar()
        entry = ttk.Entry(cmdline, textvariable=self.manual_var)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", lambda _e: self.send_manual())
        ttk.Button(cmdline, text="Send", command=self.send_manual).pack(side="left", padx=(6, 0))
        self.log = tk.Text(outer, height=4, wrap="none", state="disabled")
        self.log.pack(side="bottom", fill="x", pady=(4, 0))
        bottom = ttk.Frame(outer)
        bottom.pack(side="bottom", fill="x", pady=(8, 0))
        ttk.Label(bottom, textvariable=self.status_var, wraplength=1300).pack(side="left")

        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="y")
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))

        # ---- current pose
        cur = ttk.LabelFrame(left, text="Current tool pose (forward kinematics)", padding=6)
        cur.pack(fill="x")
        ttk.Label(cur, textvariable=self.pose_var, font=("TkFixedFont", 11)).pack(anchor="w")
        ttk.Button(cur, text="Target ← current", command=self.copy_current_to_target).pack(anchor="w", pady=(4, 0))

        # ---- target pose
        tgt = ttk.LabelFrame(left, text="Target tool pose", padding=6)
        tgt.pack(fill="x", pady=(8, 0))
        for row, (key, label, unit) in enumerate((
            ("x", "X", "mm"), ("y", "Y", "mm"), ("z", "Z", "mm"),
            ("roll", "Roll", "°"), ("pitch", "Pitch", "°"), ("yaw", "Yaw", "°"),
        )):
            ttk.Label(tgt, text=f"{label} ({unit})", width=10).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(tgt, textvariable=self.target_vars[key], width=9)
            entry.grid(row=row, column=1, padx=3)
            entry.bind("<Return>", lambda _e: self.solve())
            ttk.Button(tgt, text="−", width=3, command=lambda k=key: self.nudge(k, -1)).grid(row=row, column=2)
            ttk.Button(tgt, text="+", width=3, command=lambda k=key: self.nudge(k, +1)).grid(row=row, column=3)
        steps = ttk.Frame(tgt)
        steps.grid(row=6, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Label(steps, text="Step:").pack(side="left")
        ttk.Entry(steps, textvariable=self.step_mm_var, width=5).pack(side="left", padx=(4, 0))
        ttk.Label(steps, text="mm").pack(side="left", padx=(2, 8))
        ttk.Entry(steps, textvariable=self.step_deg_var, width=5).pack(side="left")
        ttk.Label(steps, text="°").pack(side="left", padx=(2, 0))
        ttk.Label(tgt, text="Orientation = roll·pitch·yaw of the tool frame (Rz·Ry·Rx), tool z = approach axis",
                  wraplength=300, foreground="#666").grid(row=7, column=0, columnspan=4, sticky="w", pady=(4, 0))

        # ---- IK options
        opt = ttk.LabelFrame(left, text="Inverse kinematics", padding=6)
        opt.pack(fill="x", pady=(8, 0))
        for text, value in (("Position + full orientation", "full"),
                            ("Position + approach axis (roll free)", "pointing"),
                            ("Position only", "position")):
            ttk.Radiobutton(opt, text=text, value=value, variable=self.mode_var, command=self.solve).pack(anchor="w")
        ttk.Label(opt, textvariable=self.solution_var, wraplength=300).pack(anchor="w", pady=(4, 0))
        buttons = ttk.Frame(opt)
        buttons.pack(fill="x", pady=(6, 0))
        ttk.Button(buttons, text="Solve", command=self.solve).pack(side="left")
        self.go_button = ttk.Button(buttons, text="Go to target", command=self.go)
        self.go_button.pack(side="left", padx=6)
        ttk.Label(buttons, text="max speed").pack(side="left", padx=(6, 2))
        ttk.Spinbox(buttons, from_=1, to=100, textvariable=self.speed_var, width=4).pack(side="left")
        ttk.Checkbutton(opt, text="Track target continuously (arm follows every target change, ~8 Hz)",
                        variable=self.track_var, command=self._track_changed).pack(anchor="w", pady=(6, 0))
        self.track_status_var = tk.StringVar(value="")
        ttk.Label(opt, textvariable=self.track_status_var, foreground="#b00020", wraplength=300).pack(anchor="w")

        # ---- gripper
        grip = ttk.LabelFrame(left, text=f"Gripper (ID {self.cal.gripper_id})", padding=6)
        grip.pack(fill="x", pady=(8, 0))
        ttk.Scale(grip, from_=self.cal.gripper_open_raw, to=self.cal.gripper_closed_raw,
                  variable=self.gripper_var, orient="horizontal").pack(fill="x")
        gb = ttk.Frame(grip)
        gb.pack(fill="x")
        ttk.Button(gb, text="Open", command=lambda: self.gripper_to(self.cal.gripper_open_raw)).pack(side="left")
        ttk.Button(gb, text="Close", command=lambda: self.gripper_to(self.cal.gripper_closed_raw)).pack(side="left", padx=4)
        ttk.Button(gb, text="Go", command=lambda: self.gripper_to(self.gripper_var.get())).pack(side="left")
        ttk.Label(gb, text="speed 50").pack(side="right")

        # ---- right: view + joints + calibration
        self.view = ArmView(right, height=300, on_target_drag=self._target_dragged, on_scroll=self._target_scrolled)
        self.view.pack(fill="both", expand=True)
        for key, (axis, sign) in {"<Left>": ("y", +1), "<Right>": ("y", -1), "<Up>": ("x", +1), "<Down>": ("x", -1),
                                  "<Prior>": ("z", +1), "<Next>": ("z", -1),
                                  "q": ("yaw", +1), "e": ("yaw", -1), "a": ("pitch", +1), "d": ("pitch", -1),
                                  "z": ("roll", +1), "c": ("roll", -1)}.items():
            self.view.bind(key, lambda _e, k=axis, sg=sign: self.nudge(k, sg))

        joints = ttk.LabelFrame(right, text="Joints  (angle now / IK solution, degrees)", padding=6)
        joints.pack(fill="x", pady=(8, 0))
        headers = ("Joint", "Servo", "Now", "Solution", "Torque", "Sign", "Zero here")
        for col, text in enumerate(headers):
            ttk.Label(joints, text=text).grid(row=0, column=col, padx=4)
        self.torque_labels = []
        for i, name in enumerate(ck.JOINT_NAMES):
            ttk.Label(joints, text=name).grid(row=i + 1, column=0, sticky="w", padx=4)
            ttk.Spinbox(joints, from_=1, to=253, textvariable=self.id_vars[i], width=4,
                        command=self._calibration_changed).grid(row=i + 1, column=1)
            ttk.Label(joints, textvariable=self.joint_vars[i], width=9, anchor="e").grid(row=i + 1, column=2)
            ttk.Label(joints, textvariable=self.sol_vars[i], width=9, anchor="e").grid(row=i + 1, column=3)
            lbl = ttk.Label(joints, text="off", width=5)
            lbl.grid(row=i + 1, column=4)
            self.torque_labels.append(lbl)
            ttk.Checkbutton(joints, text="flip", variable=self.sign_vars[i], onvalue=-1, offvalue=1,
                            command=self._calibration_changed).grid(row=i + 1, column=5)
            ttk.Button(joints, text="zero", width=5, command=lambda k=i: self.zero_joint(k)).grid(row=i + 1, column=6)
        calrow = ttk.Frame(joints)
        calrow.grid(row=ck.DOF + 1, column=0, columnspan=7, sticky="w", pady=(6, 0))
        ttk.Button(calrow, text="Zero ALL here (arm straight up = URDF zero)", command=self.zero_all).pack(side="left")
        ttk.Button(calrow, text="Save calibration", command=self.save_calibration).pack(side="left", padx=6)
        ttk.Label(calrow, text=os.path.basename(ck.DEFAULT_CALIBRATION_PATH), foreground="#666").pack(side="left")


    # ------------------------------------------------------------ connection
    def _refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        self.port_combo["values"] = [f"{p.device} — {p.description}" for p in ports]
        current = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not current or current not in {p.device for p in ports}:
            preferred = preferred_port(ports)
            if preferred:
                self.port_var.set(preferred)

    def connect(self) -> None:
        if self.link.connected:
            self.link.disconnect()
            self.connection_var.set("Disconnected")
            self.connect_button.configure(text="Connect")
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
        self.set_status("Connected. Sending STOP, scanning, reading limits…")
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
        for sid in self.servo_ids + [self.cal.gripper_id]:
            self.send(f"LIMITS {sid}", quiet=True, coalesce=f"LIMITS {sid}")
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
        self.track_var.set(False)
        self.view.tracking = False
        self.track_status_var.set("")
        if self.send("STOP", priority=True):
            self.set_status("STOP sent — all servo torque is being disabled")

    def enable_all(self) -> None:
        if not messagebox.askyesno("Enable arm?", "Every servo will hold its present position, then torque on. "
                                   "Make sure the workspace is clear."):
            return
        self.send("ENABLE ALL")
        self.root.after(900, lambda: self.send("STATUS", quiet=True))

    # -------------------------------------------------------------- telemetry
    def _poll_status(self) -> None:
        if not self.closing and self.link.connected:
            self.send("STATUS", quiet=True, coalesce="STATUS")
        if not self.closing:
            self.root.after(STATUS_PERIOD_MS, self._poll_status)

    def _drain_events(self) -> None:
        self.link.tick()
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "rx":
                    self._log(f"< {value}")
                    self._handle_line(value)
                elif kind == "tx":
                    self._log(f"> {value}")
                elif kind == "timeout":
                    self._log(f"! no reply to {value}")
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
        m = STATUS_RE.match(line)
        if m:
            sid = int(m.group(1))
            self.models[sid] = m.group(2)
            self.raw[sid] = int(m.group(3))
            self.torque[sid] = bool(int(m.group(5)))
            if sid == self.servo_ids[-1] or line.endswith("OK STATUS"):
                self._update_from_raw()
            return
        m = LIMITS_RE.match(line)
        if m:
            sid = int(m.group(1))
            self.cw[sid], self.ccw[sid] = int(m.group(2)), int(m.group(3))
            self._update_limits()
            return
        if line.startswith("OK STATUS"):
            self._update_from_raw()
        elif line.startswith("ERR"):
            self.set_status(line, error=True)
        elif line.startswith("OK") or line.startswith("PONG"):
            self.set_status(line)

    def _update_limits(self) -> None:
        if all(sid in self.cw for sid in self.servo_ids):
            cw = [self.cw[s] for s in self.servo_ids]
            ccw = [self.ccw[s] for s in self.servo_ids]
            self.lower, self.upper = self.cal.limits_from_servo(cw, ccw)

    def _update_from_raw(self) -> None:
        if not all(sid in self.raw for sid in self.servo_ids):
            return
        raw = [self.raw[s] for s in self.servo_ids]
        self.q = self.cal.raw_to_rad(raw)
        self.have_pose = True
        for i in range(ck.DOF):
            self.joint_vars[i].set(f"{math.degrees(self.q[i]):7.1f}")
            on = self.torque.get(self.servo_ids[i], False)
            self.torque_labels[i].configure(text="ON" if on else "off", foreground="#1a7f37" if on else "#666")
        p, R = ck.tool_pose(self.q)
        rpy = ck.matrix_to_rpy(R)
        self.pose_var.set(
            f"x {p[0]*1000:7.1f}  y {p[1]*1000:7.1f}  z {p[2]*1000:7.1f} mm\n"
            f"roll {math.degrees(rpy[0]):6.1f}  pitch {math.degrees(rpy[1]):6.1f}  yaw {math.degrees(rpy[2]):6.1f} °"
        )
        self.view.current = ck.link_points(self.q)
        self.view.current_R = R
        self.view.redraw()
        outside = [ck.JOINT_NAMES[i] for i in range(ck.DOF)
                   if self.q[i] < self.lower[i] - 1e-9 or self.q[i] > self.upper[i] + 1e-9]
        if outside != self._outside_limits:
            self._outside_limits = outside
            if outside:
                self.set_status("Joint(s) read outside the model limits: " + ", ".join(outside)
                                + " — zero offsets probably need calibrating (see README)", error=True)
        if not any(v.get() for v in self.target_vars.values()):
            self.copy_current_to_target()

    # ------------------------------------------------------------------- IK
    def _target(self) -> tuple[np.ndarray, np.ndarray]:
        try:
            p = np.array([self.target_vars[k].get() for k in ("x", "y", "z")]) / 1000.0
            rpy = [math.radians(self.target_vars[k].get()) for k in ("roll", "pitch", "yaw")]
        except tk.TclError:
            raise ValueError("target fields must be numbers")
        return p, ck.rpy_to_matrix(*rpy)

    def copy_current_to_target(self) -> None:
        if not self.have_pose:
            return
        p, R = ck.tool_pose(self.q)
        rpy = ck.matrix_to_rpy(R)
        for k, v in zip(("x", "y", "z"), p):
            self.target_vars[k].set(round(float(v) * 1000.0, 1))
        for k, v in zip(("roll", "pitch", "yaw"), rpy):
            self.target_vars[k].set(round(math.degrees(v), 1))
        self.solve()

    def nudge(self, key: str, direction: int) -> None:
        step = self.step_mm_var.get() if key in ("x", "y", "z") else self.step_deg_var.get()
        self.target_vars[key].set(round(self.target_vars[key].get() + direction * step, 2))
        self.solve()

    def _target_dragged(self, new_p: np.ndarray) -> None:
        for k, v in zip(("x", "y", "z"), new_p):
            self.target_vars[k].set(round(float(v) * 1000.0, 1))
        self.solve()

    def _target_scrolled(self, steps: int) -> None:
        self.nudge("z", steps)

    def solve(self) -> ck.IKResult | None:
        try:
            p, R = self._target()
        except ValueError as exc:
            self.set_status(str(exc), error=True)
            return None
        seed = self.q if self.have_pose else ck.REST_POSE
        if self.solution is not None and self.solution.converged:
            seed = self.solution.q  # continuity: warm-start from the last answer
        res = ck.solve_ik(p, R, seed, mode=self.mode_var.get(), lower=self.lower, upper=self.upper)
        self.solution = res
        for i in range(ck.DOF):
            self.sol_vars[i].set(f"{math.degrees(res.q[i]):7.1f}")
        self.view.target = ck.link_points(res.q)
        self.view.target_pose = (p, R)
        self.view.redraw()
        text = (f"{'Converged' if res.converged else 'NOT converged'} in {res.iterations} it — "
                f"pos err {res.position_error*1000:.2f} mm, rot err {math.degrees(res.orientation_error):.2f}°")
        if res.at_limit:
            text += " — at limit: " + ", ".join(res.at_limit)
        if np.linalg.norm(p) > ck.reach():
            text += f" — target beyond reach ({ck.reach()*1000:.0f} mm)"
        self.solution_var.set(text)
        return res

    # ----------------------------------------------------------------- motion
    def _arm_ready(self) -> bool:
        if not self.link.connected:
            self.set_status("Not connected", error=True)
            return False
        if not self.have_pose:
            self.set_status("No telemetry yet", error=True)
            return False
        if not all(self.torque.get(s, False) for s in self.servo_ids):
            self.set_status("Enable the arm first (torque is off on at least one joint)", error=True)
            return False
        return True

    def go(self) -> None:
        res = self.solve()
        if res is None:
            return
        if not res.converged and not messagebox.askyesno(
                "IK did not converge", "The solver did not reach the target. Move to the closest pose anyway?"):
            return
        if not self._arm_ready():
            return
        self._send_coordinated(res.q, int(self.speed_var.get()))

    def _send_coordinated(self, q: np.ndarray, max_speed: int, announce: bool = True) -> None:
        """One MOVE per joint with speeds scaled so all joints finish together."""
        max_speed = max(1, min(100, max_speed))
        goal_raw = self.cal.rad_to_raw(q)
        now_raw = [self.raw[s] for s in self.servo_ids]
        deltas = [abs(g - n) for g, n in zip(goal_raw, now_raw)]
        biggest = max(deltas) or 1
        for sid, goal, delta in zip(self.servo_ids, goal_raw, deltas):
            goal = max(self.cw.get(sid, 0), min(self.ccw.get(sid, 4095), goal))
            speed = max(1, int(round(max_speed * delta / biggest)))
            self.send(f"MOVE {sid} {goal} {speed}", quiet=True, coalesce=f"MOVE {sid}")
        if announce:
            seconds = biggest / 4096 * 60 / (max_speed * MX_RPM_PER_UNIT)
            self.set_status(f"Moving (largest joint change {biggest} ticks, ≈{seconds:.1f} s at speed {max_speed})")

    def _track_changed(self) -> None:
        if self.track_var.get():
            if not self.link.connected or not self.have_pose:
                self.track_var.set(False)
                self.track_status_var.set("Connect and wait for telemetry first")
                return
            if not all(self.torque.get(s, False) for s in self.servo_ids):
                if messagebox.askyesno("Enable the arm?",
                                       "Tracking needs torque on every joint. Enable all servos now "
                                       "(each holds its present position first)?"):
                    self.send("ENABLE ALL")
                    self.send("STATUS", quiet=True, coalesce="STATUS")
                else:
                    self.track_var.set(False)
                    return
            self._last_sent_raw = None
            self.view.tracking = True
            self.track_status_var.set("TRACKING: drag the gizmo, nudge, or type a target and the arm follows")
            self.set_status("Tracking ON")
            if self._stream_after is None:
                self._track_tick()
        else:
            self.view.tracking = False
            self.track_status_var.set("")
            self.set_status("Tracking OFF")
        self.view.redraw()

    def _track_tick(self) -> None:
        """Runs every STREAM_PERIOD_MS while tracking: sends the newest solution if it changed."""
        self._stream_after = None
        if self.closing or not self.track_var.get():
            return
        self._stream_after = self.root.after(STREAM_PERIOD_MS, self._track_tick)
        if self.solution is None or not self.link.connected:
            return
        if not all(self.torque.get(s, False) for s in self.servo_ids):
            self.track_status_var.set("Waiting for torque on all joints…")
            return
        if not self.solution.converged:
            self.track_status_var.set("Paused: IK has not converged for this target")
            return
        goal_raw = self.cal.rad_to_raw(self.solution.q)
        if goal_raw == self._last_sent_raw:
            return
        self._last_sent_raw = goal_raw
        gripper_raw = self.raw.get(self.cal.gripper_id, 0)
        # GROUP needs all 8 servos torqued; ID order is 8..15 in the firmware.
        if self.torque.get(self.cal.gripper_id, False):
            self.send(f"GROUP {int(self.speed_var.get())} " + " ".join(map(str, goal_raw + [gripper_raw])),
                      quiet=True, coalesce="GROUP")
        else:
            self._send_coordinated(self.solution.q, int(self.speed_var.get()), announce=False)
        self.track_status_var.set(f"TRACKING: sent {time.strftime('%H:%M:%S')}")

    def gripper_to(self, raw: int) -> None:
        gid = self.cal.gripper_id
        if not self.torque.get(gid, False):
            self.send(f"ENABLE {gid}")
            self.root.after(400, lambda: self.send(f"MOVE {gid} {int(raw)} 50"))
        else:
            self.send(f"MOVE {gid} {int(raw)} 50")
        self.gripper_var.set(int(raw))

    # ------------------------------------------------------------ calibration
    def _calibration_changed(self) -> None:
        for i, j in enumerate(self.cal.joints):
            j.sign = int(self.sign_vars[i].get())
            j.servo_id = int(self.id_vars[i].get())
        self.servo_ids = self.cal.servo_ids
        self._update_limits()
        self._update_from_raw()

    def zero_joint(self, index: int) -> None:
        sid = self.servo_ids[index]
        if sid not in self.raw:
            self.set_status("No telemetry for that servo yet", error=True)
            return
        self.cal.joints[index].zero_raw = self.raw[sid]
        self._update_limits()
        self._update_from_raw()
        self.set_status(f"{ck.JOINT_NAMES[index]} zero set to raw {self.raw[sid]} (unsaved)")

    def zero_all(self) -> None:
        if not self.have_pose:
            self.set_status("No telemetry yet", error=True)
            return
        if not messagebox.askyesno("Zero all joints?", "Set the CURRENT pose as the model's zero pose "
                                   "(arm pointing straight up, gripper aligned with the base x axis)?"):
            return
        for i in range(ck.DOF):
            self.cal.joints[i].zero_raw = self.raw[self.servo_ids[i]]
        self._update_limits()
        self._update_from_raw()
        self.set_status("All joint zeros set from the current pose (unsaved)")

    def save_calibration(self) -> None:
        self._calibration_changed()
        self.cal.save(ck.DEFAULT_CALIBRATION_PATH)
        self.set_status(f"Calibration saved to {ck.DEFAULT_CALIBRATION_PATH}")

    # ---------------------------------------------------------------- misc
    def _log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 200:
            self.log.delete("1.0", "50.0")
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
    app = IKApp(root)
    if len(sys.argv) > 1:
        app.port_var.set(sys.argv[1])
    root.mainloop()


if __name__ == "__main__":
    main()
