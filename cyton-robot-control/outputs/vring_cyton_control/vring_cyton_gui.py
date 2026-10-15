"""Unified Cyton Robot Controller + VRing 2 BLE Telemetry Dashboard.

Combines:
1. Serial gateway control for the Cyton robot arm (servos 8-15) via M5StickC bridge.
2. Bluetooth Low Energy (BLE) interface for VRing 2 (LSM6DS3 IMU, touch, battery).
3. Preset management including the straightened vertical starting setup.
4. Synchronous data logging for IMU calibration.
"""

from __future__ import annotations

import asyncio
import csv
import datetime
import json
import math
import os
import queue
import re
import shutil
import statistics
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import font as tkfont
from tkinter import messagebox, ttk
from typing import Any

import serial
from serial.tools import list_ports

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.device import BLEDevice
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False


# ==============================================================================
# Robot Gateway Definitions
# ==============================================================================
SERVO_IDS = tuple(range(8, 16))
MODEL_NAMES = {12: "AX-12A", 29: "MX-28", 310: "MX-64"}
STATUS_RE = re.compile(
    r"^ID (\d+) MODEL (\S+) POS (-?\d+) GOAL (-?\d+) TORQUE (\d+) "
    r"MOVING (\d+) VOLT_X10 (-?\d+) TEMP_C (-?\d+)$"
)
SCAN_RE = re.compile(r"^ID (\d+) MODEL (\S+)$")
LIMITS_RE = re.compile(r"^ID (\d+) CW (-?\d+) CCW (-?\d+)$")

USB_SERIAL_HINT = re.compile(
    r"usbserial|usbmodem|wchusbserial|SLAB_USBtoUART|ttyUSB|ttyACM|FTDI|FT232|CP210|CH9102|CH340|M5|Silicon Labs|USB.?Serial",
    re.IGNORECASE,
)
IGNORED_PORT_HINT = re.compile(r"Bluetooth|debug-console|Jabra|JLab", re.IGNORECASE)

REPLY_TIMEOUT_S = 1.5


# ==============================================================================
# Pose Presets
# ==============================================================================
# Starting pose: Straight vertical arm with user-specified reference roll & gripper
PRESET_STRAIGHTENED = {
    8: 3539,  # Shoulder Roll (from user example)
    9: 2048,  # Shoulder Pitch (straight vertical)
    10: 2048, # Elbow Roll (straight)
    11: 2048, # Elbow Pitch (straight vertical - straightened from 2748)
    12: 2048, # Elbow Yaw (straight)
    13: 2048, # Wrist Pitch (straight vertical - straightened from 2384)
    14: 838,  # Wrist Roll (from user example)
    15: 250,  # Gripper (neutral grip holding ring)
}

PRESET_HOME = {
    8: 2048,
    9: 2048,
    10: 2048,
    11: 2048,
    12: 2048,
    13: 2048,
    14: 2048,
    15: 250,
}

PRESET_USER_EXAMPLE = {
    8: 2048,
    9: 2048,
    10: 2048,
    11: 2048,
    12: 1060,  # -90° adjusted for arm droop (1060 ticks)
    13: 2048,
    14: 2048,
    15: 400,  # User-requested gripper position for the ID12 -90° example
}

# Six known ring orientations for arm-to-IMU calibration.  Each point is held
# for 3 seconds; the current robot and ring telemetry are logged halfway through.
CALIBRATION_POSES: tuple[tuple[str, dict[int, int]], ...] = (
    ("+Y", {8: 2048, 9: 2048, 10: 2048, 11: 2048, 12: 1060, 13: 2048, 14: 2048, 15: 325}),
    ("+X", {8: 2046, 9: 2049, 10: 2043, 11: 2051, 12: 1060, 13: 2049, 14: 3065, 15: 325}),
    ("-X", {8: 2046, 9: 2049, 10: 2043, 11: 2051, 12: 3074, 13: 2049, 14: 3065, 15: 325}),
    ("-Y", {8: 2050, 9: 2050, 10: 2049, 11: 2051, 12: 3074, 13: 2049, 14: 2050, 15: 325}),
    ("-Z", {8: 2050, 9: 2053, 10: 2051, 11: 3044, 12: 2053, 13: 3015, 14: 2048, 15: 325}),
    ("+Z", {8: 2046, 9: 2050, 10: 2052, 11: 2051, 12: 2053, 13: 2012, 14: 2050, 15: 325}),
)

# Final gyro-X gain reference: start from the established ID12 -90° example,
# use the same servo-14 +89.47° orientation as +Y -> +X, then move only wrist
# pitch 90°. Keep the calibration-safe gripper target rather than the example's 400.
GYRO_GAIN_SWEEP_START = {8: 2048, 9: 2048, 10: 2048, 11: 2048, 12: 1060, 13: 2048, 14: 3065, 15: 325}
GYRO_GAIN_SWEEP_END = {**GYRO_GAIN_SWEEP_START, 13: 3072}
GYRO_GAIN_SWEEP_ANGLE_DEG = 90.0
GYRO_GAIN_TRANSITIONS = {
    # Both are the only material joint changes in their respective pose-to-pose moves.
    ("+Y", "+X"): {"name": "servo14_wrist_roll", "servo_id": 14, "start": 2048, "end": 3065},
    ("+X", "-X"): {"name": "servo12_elbow_yaw", "servo_id": 12, "start": 1060, "end": 3074},
}
for _transition in GYRO_GAIN_TRANSITIONS.values():
    _transition["reference_angle_deg"] = (
        (_transition["end"] - _transition["start"]) * 360.0 / 4096.0
    )

CALIBRATION_HOLD_MS = 3_000
CALIBRATION_CAPTURE_MS = 1_500
CALIBRATION_STATUS_POLL_MS = 250
CALIBRATION_REACH_TIMEOUT_MS = 20_000
GYRO_GAIN_SWEEP_TIMEOUT_MS = 20_000
GYRO_TRACE_MAX_GAP_TICKS = 80  # 200 ms at the ring's 400 Hz sample clock.
CALIBRATION_POSITION_TOLERANCE_TICKS = 20
# At a stationary capture pose, gravity should produce a vector magnitude near
# 1 g. Values outside this range indicate a bad or transient IMU packet.
CALIBRATION_IMU_MAGNITUDE_MIN_G = 0.75
CALIBRATION_IMU_MAGNITUDE_MAX_G = 1.25
CALIBRATION_MIN_TRIALS = 3
FIRMWARE_CALIBRATION_BEGIN = "// BEGIN AUTO ACCEL CALIBRATION"
FIRMWARE_CALIBRATION_END = "// END AUTO ACCEL CALIBRATION"
FIRMWARE_GYRO_CALIBRATION_BEGIN = "// BEGIN AUTO GYRO CALIBRATION"
FIRMWARE_GYRO_CALIBRATION_END = "// END AUTO GYRO CALIBRATION"
GYRO_SAMPLE_PERIOD_S = 1.0 / 400.0


def preferred_port(ports: list[Any]) -> str | None:
    candidates = [
        p for p in ports
        if not IGNORED_PORT_HINT.search(f"{p.device} {p.description or ''}")
    ]
    for p in candidates:
        if USB_SERIAL_HINT.search(f"{p.device} {p.description or ''} {p.manufacturer or ''}"):
            return p.device
    return None


class SerialLink:
    """Serial port with a strict one-command-at-a-time queue for Cyton gateway."""
    TERMINATORS = ("OK", "ERR", "PONG", "READY")

    def __init__(self, events: queue.Queue[tuple[str, str]]) -> None:
        self.events = events
        self.port: serial.Serial | None = None
        self.thread: threading.Thread | None = None
        self.running = False
        self.lock = threading.Lock()
        self.pending: list[tuple[str, str | None]] = []
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
        with self.lock:
            self.pending.clear()
            self.in_flight = None
        if self.port:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def send(self, command: str, coalesce_key: str | None = None) -> None:
        with self.lock:
            if command == "STOP":
                self.pending = [(c, k) for (c, k) in self.pending if c == "STOP"]
                self.pending.insert(0, ("STOP", None))
            elif coalesce_key is not None:
                self.pending = [(c, k) for (c, k) in self.pending if k != coalesce_key]
                self.pending.append((command, coalesce_key))
            else:
                self.pending.append((command, coalesce_key))
            self._dispatch_locked()

    def check_timeout(self) -> None:
        with self.lock:
            if self.in_flight is not None and (time.time() - self.sent_at) > REPLY_TIMEOUT_S:
                abandoned = self.in_flight
                self.in_flight = None
                self.events.put(("timeout", abandoned))
                self._dispatch_locked()

    def _dispatch_locked(self) -> None:
        if not self.connected or self.in_flight is not None or not self.pending:
            return
        cmd, _key = self.pending.pop(0)
        self.in_flight = cmd
        self.sent_at = time.time()
        try:
            line = (cmd + "\n").encode("ascii")
            self.port.write(line)
            self.port.flush()
            self.events.put(("tx", cmd))
        except (serial.SerialException, OSError) as exc:
            self.in_flight = None
            self.events.put(("error", str(exc)))

    def _reader(self) -> None:
        try:
            while self.running and self.port and self.port.is_open:
                data = self.port.readline()
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


# ==============================================================================
# VRing BLE Definitions & Worker
# ==============================================================================
VRING_SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
SENSOR_DATA_CHAR_UUID = "19b10002-e8f2-537e-4f6c-d104768a121a"
VIBRATION_CHAR_UUID = "19b10005-e8f2-537e-4f6c-d104768a1219"
BUTTON_INPUT_CHAR_UUID = "19b10006-e8f2-537e-4f6c-d104768a1221"
CALIBRATION_CHAR_UUID = "19b10007-e8f2-537e-4f6c-d104768a1222"
CALIBRATION_PACKET_PREFIX = b"VRC1"
CALIBRATION_PACKET_VERSION = 1


def calibration_packet_values(packet: bytes) -> tuple[float, ...] | None:
    """Validate the ring's packed persistent-calibration record."""
    if len(packet) != struct.calcsize("<4sHH12fI"):
        return None
    body, checksum_bytes = packet[:-4], packet[-4:]
    checksum = 2166136261
    for byte in body:
        checksum = ((checksum ^ byte) * 16777619) & 0xFFFFFFFF
    magic, version, _reserved, *values, stored_checksum = struct.unpack("<4sHH12fI", packet)
    if magic != CALIBRATION_PACKET_PREFIX or version != CALIBRATION_PACKET_VERSION or checksum != stored_checksum:
        return None
    return tuple(values)
TARGET_NAME = "VRING"


@dataclass
class VRingState:
    connected: bool = False
    status: str = "Disconnected"
    rssi: int = 0
    device_name: str = "VRING"
    device_address: str = ""

    # Raw sensor values
    raw_ax: int = 0
    raw_ay: int = 0
    raw_az: int = 0
    raw_qx: int = 0
    raw_qy: int = 0
    raw_qz: int = 0
    raw_gx: int = 0
    raw_gy: int = 0
    raw_gz: int = 0
    cap1: int = 0
    cap2: int = 0
    battery_mv: int = 0
    temp_c: float = 0.0
    temp_f: float = 0.0
    sample_counter: int = 0
    button_pressed: bool = False

    # Converted values
    accel_x_g: float = 0.0
    accel_y_g: float = 0.0
    accel_z_g: float = 0.0
    accel_mag_g: float = 0.0
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0

    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    # Rate tracking
    packets_received: int = 0
    packet_rate_hz: float = 0.0
    last_packet_time: float = 0.0
    calibration_values: tuple[float, ...] | None = None


def parse_vring_packet(payload: bytes) -> tuple[dict[str, Any], tuple[float, float, float], tuple[float, float, float, float]]:
    """Unpack VRing payload and compute physical units; gyro fields need firmware v2."""
    if len(payload) < 22:
        return {}, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    raw_ax = int.from_bytes(payload[0:2], "little", signed=True)
    raw_ay = int.from_bytes(payload[2:4], "little", signed=True)
    raw_az = int.from_bytes(payload[4:6], "little", signed=True)
    raw_qx = int.from_bytes(payload[6:8], "little", signed=True)
    raw_qy = int.from_bytes(payload[8:10], "little", signed=True)
    raw_qz = int.from_bytes(payload[10:12], "little", signed=True)
    cap1 = int.from_bytes(payload[12:14], "little", signed=True)
    cap2 = int.from_bytes(payload[14:16], "little", signed=True)
    battery_mv = int.from_bytes(payload[16:18], "little", signed=False)
    temp_raw = int.from_bytes(payload[18:20], "little", signed=True)
    sample_counter = int.from_bytes(payload[20:22], "little", signed=False)
    raw_gx = int.from_bytes(payload[22:24], "little", signed=True) if len(payload) >= 28 else 0
    raw_gy = int.from_bytes(payload[24:26], "little", signed=True) if len(payload) >= 28 else 0
    raw_gz = int.from_bytes(payload[26:28], "little", signed=True) if len(payload) >= 28 else 0

    # Temperature
    temp_f = temp_raw / 100.0
    temp_c = (temp_f - 32.0) * 5.0 / 9.0

    # Accelerometer: 32767 = 16g
    scale_accel = 32767.0 / 16.0
    ax_g = raw_ax / scale_accel
    ay_g = raw_ay / scale_accel
    az_g = raw_az / scale_accel
    mag_g = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
    scale_gyro = 32767.0 / 2000.0
    gx_dps = raw_gx / scale_gyro
    gy_dps = raw_gy / scale_gyro
    gz_dps = raw_gz / scale_gyro

    # Quaternion
    qx = raw_qx / 32767.0
    qy = raw_qy / 32767.0
    qz = raw_qz / 32767.0
    qw_sq = max(0.0, 1.0 - (qx * qx + qy * qy + qz * qz))
    qw = math.sqrt(qw_sq)

    # Euler angles (RPY)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    data = {
        "raw_ax": raw_ax, "raw_ay": raw_ay, "raw_az": raw_az,
        "raw_qx": raw_qx, "raw_qy": raw_qy, "raw_qz": raw_qz,
        "raw_gx": raw_gx, "raw_gy": raw_gy, "raw_gz": raw_gz,
        "cap1": cap1, "cap2": cap2,
        "battery_mv": battery_mv,
        "temp_c": temp_c, "temp_f": temp_f,
        "sample_counter": sample_counter,
        "ax_g": ax_g, "ay_g": ay_g, "az_g": az_g, "mag_g": mag_g,
        "gx_dps": gx_dps, "gy_dps": gy_dps, "gz_dps": gz_dps,
        "gyro_available": len(payload) >= 28,
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
    }
    return data, (ax_g, ay_g, az_g), (qw, qx, qy, qz)


class BleWorker:
    """Background asyncio worker managing BLE device selection & connection to VRing."""
    def __init__(self, ble_events: queue.Queue[tuple[str, Any]]) -> None:
        self.events = ble_events
        self.thread: threading.Thread | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.running = threading.Event()
        self.client: BleakClient | None = None
        self.cmd_queue: asyncio.Queue[tuple[str, Any]] | None = None
        self.discovered: dict[str, tuple[BLEDevice, int]] = {}

    def start(self) -> None:
        if not BLEAK_AVAILABLE:
            self.events.put(("error", "bleak package is not installed"))
            return
        self.running.set()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running.clear()
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("stop", None))

    def scan(self) -> None:
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("scan", None))

    def connect(self, address: str) -> None:
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("connect", address))

    def disconnect(self) -> None:
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("disconnect", None))

    def send_vibration(self, strength: int) -> None:
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("vibrate", max(0, min(255, strength))))

    def save_calibration(self, values: tuple[float, ...]) -> None:
        if self.loop and self.cmd_queue:
            self.loop.call_soon_threadsafe(self.cmd_queue.put_nowait, ("save_calibration", values))

    def _run_loop(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        self.cmd_queue = asyncio.Queue()
        await self.cmd_queue.put(("scan", None))

        while self.running.is_set():
            try:
                if self.client and self.client.is_connected:
                    try:
                        cmd, payload = await asyncio.wait_for(self.cmd_queue.get(), timeout=0.1)
                    except asyncio.TimeoutError:
                        continue
                else:
                    cmd, payload = await self.cmd_queue.get()

                if cmd == "stop":
                    break
                elif cmd == "scan":
                    await self._do_scan()
                elif cmd == "connect":
                    await self._do_connect(payload)
                elif cmd == "disconnect":
                    await self._do_disconnect()
                elif cmd == "vibrate":
                    if self.client and self.client.is_connected:
                        try:
                            await self.client.write_gatt_char(VIBRATION_CHAR_UUID, bytes([payload]), response=False)
                        except Exception:
                            pass
                elif cmd == "save_calibration":
                    if not self.client or not self.client.is_connected:
                        self.events.put(("calibration_failed", "VRING is not connected"))
                        continue
                    body = struct.pack(
                        "<4sHH12f", CALIBRATION_PACKET_PREFIX, CALIBRATION_PACKET_VERSION, 0, *payload
                    )
                    checksum = 2166136261
                    for byte in body:
                        checksum = ((checksum ^ byte) * 16777619) & 0xFFFFFFFF
                    packet = body + struct.pack("<I", checksum)
                    try:
                        await self.client.write_gatt_char(CALIBRATION_CHAR_UUID, packet, response=True)
                        stored = bytes(await self.client.read_gatt_char(CALIBRATION_CHAR_UUID))
                        if stored != packet:
                            raise RuntimeError("ring did not confirm the saved calibration")
                        self.events.put(("calibration_saved", calibration_packet_values(stored)))
                    except Exception as exc:
                        self.events.put(("calibration_failed", str(exc)))
            except Exception as exc:
                self.events.put(("status", f"BLE loop: {exc}"))
                await asyncio.sleep(0.5)

        await self._do_disconnect()

    async def _do_scan(self) -> None:
        self.events.put(("status", "Scanning for VRING devices..."))
        discovered: dict[str, tuple[BLEDevice, int]] = {}

        def on_detect(dev: BLEDevice, adv: Any) -> None:
            name = dev.name or ""
            uuids = [str(u).lower() for u in (adv.service_uuids or [])]
            if name.startswith(TARGET_NAME) or VRING_SERVICE_UUID.lower() in uuids:
                # Bleak 0.20+ stores RSSI on AdvertisementData; older versions
                # put it on BLEDevice.  Keep the scan compatible with both.
                rssi = int(getattr(adv, "rssi", getattr(dev, "rssi", -100)))
                discovered[dev.address] = (dev, rssi)

        async def scan_for(seconds: float, *, service_filter: bool) -> None:
            # macOS 12+ often returns no advertisements to CoreBluetooth unless
            # a service UUID is supplied.  This matches the filter used by the
            # working Chrome Web Bluetooth test page.
            kwargs: dict[str, Any] = {"detection_callback": on_detect}
            if service_filter:
                kwargs["service_uuids"] = [VRING_SERVICE_UUID]
            scanner = BleakScanner(**kwargs)
            await scanner.start()
            try:
                scan_t0 = time.time()
                while (time.time() - scan_t0) < seconds and self.running.is_set():
                    await asyncio.sleep(0.2)
            finally:
                await scanner.stop()

        try:
            await scan_for(4.0, service_filter=True)
            # A few BLE stacks omit the advertised service UUID from scan
            # results.  Fall back to the device name used by the firmware.
            if not discovered:
                self.events.put(("status", "No service-filtered match; trying VRING name..."))
                await scan_for(4.0, service_filter=False)
        except Exception as exc:
            self.events.put(("status", f"Scan error: {exc}"))
            return

        self.discovered.update(discovered)
        dev_list = []
        for addr, (dev, rssi) in sorted(self.discovered.items(), key=lambda item: item[1][1], reverse=True):
            dev_list.append((addr, dev.name or TARGET_NAME, rssi))

        self.events.put(("devices", dev_list))
        if dev_list:
            self.events.put(("status", f"Found {len(dev_list)} VRING device(s)"))
        else:
            self.events.put(("status", "No VRING devices found nearby"))

    async def _do_connect(self, address: str) -> None:
        await self._do_disconnect()
        self.events.put(("status", f"Connecting to {address[-8:]}..."))

        target_dev: BLEDevice | None = None
        target_rssi = -100
        if address in self.discovered:
            target_dev, target_rssi = self.discovered[address]
        else:
            target_dev = await BleakScanner.find_device_by_address(address, timeout=6.0)

        if not target_dev:
            self.events.put(("disconnected", "Target device not found"))
            return

        client = BleakClient(target_dev, timeout=12.0)
        self.client = client
        try:
            await client.connect()
            self.events.put(("connected", (target_dev.name or TARGET_NAME, target_dev.address, target_rssi)))

            def on_sensor(_handle: int, data: bytearray) -> None:
                self.events.put(("sensor", bytes(data)))

            def on_button(_handle: int, data: bytearray) -> None:
                pressed = bool(data[0]) if data else False
                self.events.put(("button", pressed))

            await client.start_notify(SENSOR_DATA_CHAR_UUID, on_sensor)
            try:
                await client.start_notify(BUTTON_INPUT_CHAR_UUID, on_button)
            except Exception:
                pass
            try:
                values = calibration_packet_values(bytes(await client.read_gatt_char(CALIBRATION_CHAR_UUID)))
                if values is None:
                    raise RuntimeError("stored calibration record is invalid")
                self.events.put(("calibration_loaded", values))
            except Exception as exc:
                self.events.put(("status", f"Calibration storage unavailable: {exc}"))
        except Exception as exc:
            self.client = None
            self.events.put(("disconnected", f"Connect error: {exc}"))

    async def _do_disconnect(self) -> None:
        if self.client:
            try:
                if self.client.is_connected:
                    await self.client.disconnect()
            except Exception:
                pass
            self.client = None
            self.events.put(("disconnected", "Disconnected"))


# ==============================================================================
# UI Component: Servo Row
# ==============================================================================
class ServoRow:
    def __init__(self, app: "VringCytonApp", parent: ttk.Frame, row: int, servo_id: int) -> None:
        self.app = app
        self.id = servo_id
        self.model = "—"
        self.present = 0
        self.goal = 0
        self.reported_goal = 0
        self.cw = 0
        self.ccw = 4095 if servo_id != 15 else 1023
        self.torque = False
        self.moving = False
        self.initialized = False
        self.suppress_scale = False

        self.id_label = ttk.Label(parent, text=f"ID {servo_id}\n—", width=10)
        self.id_label.grid(row=row, column=0, padx=(4, 6), pady=4, sticky="w")

        self.position_var = tk.StringVar(value="—")
        ttk.Label(parent, textvariable=self.position_var, width=7, anchor="center").grid(
            row=row, column=1, padx=2
        )

        self.goal_var = tk.StringVar(value="0")
        self.goal_spin = ttk.Spinbox(
            parent,
            from_=self.cw,
            to=self.ccw,
            textvariable=self.goal_var,
            width=6,
            command=self._spin_changed,
        )
        self.goal_spin.grid(row=row, column=2, padx=2)
        self.goal_spin.bind("<Return>", lambda _event: self._entry_commit())
        self.goal_spin.bind("<FocusOut>", lambda _event: self._entry_commit())

        self.scale = ttk.Scale(
            parent,
            from_=self.cw,
            to=self.ccw,
            orient="horizontal",
            command=self._scale_changed,
        )
        self.scale.grid(row=row, column=3, padx=4, sticky="ew")

        self.speed_var = tk.IntVar(value=20)
        ttk.Spinbox(parent, from_=1, to=100, textvariable=self.speed_var, width=4).grid(
            row=row, column=4, padx=2
        )

        self.torque_button = ttk.Button(parent, text="Enable", width=7, command=self.toggle_torque)
        self.torque_button.grid(row=row, column=5, padx=2)
        ttk.Button(parent, text="Move", width=6, command=self.move).grid(row=row, column=6, padx=2)
        ttk.Button(parent, text="−10", width=4, command=lambda: self.nudge(-10)).grid(
            row=row, column=7, padx=1
        )
        ttk.Button(parent, text="+10", width=4, command=lambda: self.nudge(10)).grid(
            row=row, column=8, padx=1
        )

        self.health_var = tk.StringVar(value="offline")
        ttk.Label(parent, textvariable=self.health_var, width=17).grid(
            row=row, column=9, padx=(4, 2), sticky="w"
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
        volt_x10: int,
        temp_c: int,
    ) -> None:
        self.model = model
        self.id_label.configure(text=f"ID {self.id}\n{model}")
        self.present = position
        self.position_var.set(str(position))
        self.reported_goal = reported_goal
        self.torque = torque
        self.moving = moving

        if not self.initialized:
            self.initialized = True
            self.set_goal(position)

        self.torque_button.configure(text="Disable" if torque else "Enable")
        volts = volt_x10 / 10.0 if volt_x10 >= 0 else 0.0
        mov_str = "MOV " if moving else ""
        torq_str = "ON " if torque else "OFF"
        self.health_var.set(f"{torq_str} {mov_str}{volts:.1f}V {temp_c}°C")

    def set_goal(self, goal: int) -> None:
        clamped = max(self.cw, min(self.ccw, goal))
        self.goal = clamped
        self.goal_var.set(str(clamped))
        self.suppress_scale = True
        try:
            self.scale.set(clamped)
        finally:
            self.suppress_scale = False

    def _spin_changed(self) -> None:
        try:
            val = int(self.goal_var.get())
            self.set_goal(val)
        except ValueError:
            pass

    def _entry_commit(self) -> None:
        try:
            val = int(self.goal_var.get())
            self.set_goal(val)
        except ValueError:
            self.set_goal(self.goal)

    def _scale_changed(self, value: str) -> None:
        if self.suppress_scale:
            return
        clamped = max(self.cw, min(self.ccw, int(round(float(value)))))
        self.goal = clamped
        self.goal_var.set(str(clamped))

    def toggle_torque(self) -> None:
        if self.torque:
            self.app.send(f"DISABLE {self.id}")
        else:
            self.app.send(f"ENABLE {self.id}")

    def move(self) -> None:
        speed = max(1, min(100, self.speed_var.get()))
        self.app.send(f"MOVE {self.id} {self.goal} {speed}", coalesce_key=f"move_{self.id}")

    def nudge(self, delta: int) -> None:
        self.set_goal(self.goal + delta)
        self.move()


# ==============================================================================
# Main Unified Application
# ==============================================================================
class VringCytonApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Cyton Arm + VRing 2 Unified Controller")

        # Serial / Robot Gateway
        self.serial_events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.link = SerialLink(self.serial_events)
        self.port_var = tk.StringVar(value="")
        self.connection_var = tk.StringVar(value="Gateway: Disconnected")
        self.poll_enabled = tk.BooleanVar(value=True)

        # VRing BLE
        self.ble_events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.ble_worker = BleWorker(self.ble_events)
        self.ring_state = VRingState()
        self.selected_ring_var = tk.StringVar(value="")
        self.ring_addr_map: dict[str, str] = {}
        self.ble_info_var = tk.StringVar(value="Scanning for rings...")

        # UI state variables
        self.status_var = tk.StringVar(value="Ready. Initialized torque off.")
        self.vring_status_var = tk.StringVar(value="VRing: Disconnected")
        self.vring_accel_var = tk.StringVar(value="Accel: —")
        self.vring_gyro_var = tk.StringVar(value="Gyro: —")
        self.vring_quat_var = tk.StringVar(value="Quat: —")
        self.vring_euler_var = tk.StringVar(value="RPY: —")
        self.vring_diag_var = tk.StringVar(value="Batt: — | Temp: —")
        self.vring_touch_var = tk.StringVar(value="Touch: Cap1 —  Cap2 —")
        self.vring_rate_var = tk.StringVar(value="Rate: 0.0 Hz | Count: 0")

        # Calibration logger
        self.log_file_path = self._most_recent_calibration_csv()
        self.calibration_history_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "vring_calibration_history.json"
        )
        self.calibration_history = self._load_calibration_history()
        self.pending_history_entry: dict[str, Any] | None = None
        self.logged_points = self._calibration_csv_row_count(self.log_file_path)
        self.log_status_var = tk.StringVar(
            value=f"Log: {os.path.basename(self.log_file_path)} ({self.logged_points} pts recorded)"
        )
        self.calibration_sequence_index: int | None = None
        self.calibration_after_ids: list[str] = []
        self.calibration_wait_started = 0.0
        self.gyro_hold_samples: list[tuple[float, float, float]] = []
        self.gyro_hold_index: int | None = None
        self.gyro_sweep_stage: str | None = None
        self.gyro_sweep_started = 0.0
        self.gyro_sweep_samples: list[dict[str, float | int]] = []
        self.gyro_transition_capture: dict[str, Any] | None = None
        self.gyro_transition_samples: list[dict[str, float | int]] = []

        self._build_ui()
        self._refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.stop_all())

        # Start periodic tasks
        self.root.after(50, self._drain_serial_events)
        self.root.after(50, self._drain_ble_events)
        self.root.after(1400, self._poll_status)
        self.root.after(400, self.connect)

        # Start BLE worker if available
        if BLEAK_AVAILABLE:
            self.ble_worker.start()
        else:
            self.vring_status_var.set("VRing: bleak not installed")

    def _build_ui(self) -> None:
        style = ttk.Style()
        base = tkfont.nametofont("TkDefaultFont")
        family = base.actual("family")
        style.configure("Stop.TButton", font=(family, 12, "bold"), padding=8)
        style.configure("Header.TLabel", font=(family, 15, "bold"))
        style.configure("CardTitle.TLabel", font=(family, 10, "bold"))

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        # -------------------------------------------------------------
        # Top Bar: Title & Connection Statuses
        # -------------------------------------------------------------
        top = ttk.Frame(outer)
        top.pack(fill="x")
        ttk.Label(top, text="Cyton Robot + VRing 2 Controller", style="Header.TLabel").pack(side="left")
        ttk.Label(top, textvariable=self.connection_var).pack(side="right", padx=(8, 0))
        ttk.Label(top, text="|", foreground="#999").pack(side="right", padx=6)
        ttk.Label(top, textvariable=self.vring_status_var).pack(side="right")

        # -------------------------------------------------------------
        # Connection Controls Bar
        # -------------------------------------------------------------
        conn_bar = ttk.Frame(outer)
        conn_bar.pack(fill="x", pady=(8, 6))

        ttk.Label(conn_bar, text="Serial:").pack(side="left")
        self.port_combo = ttk.Combobox(conn_bar, textvariable=self.port_var, width=26, state="normal")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(conn_bar, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(conn_bar, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=4)

        ttk.Checkbutton(conn_bar, text="Poll Telemetry", variable=self.poll_enabled).pack(side="left", padx=(12, 4))
        ttk.Label(conn_bar, text="Esc = STOP", foreground="#b00020").pack(side="right")

        # -------------------------------------------------------------
        # Safety & Global Control Bar
        # -------------------------------------------------------------
        safety_bar = ttk.Frame(outer)
        safety_bar.pack(fill="x", pady=(2, 8))

        ttk.Button(safety_bar, text="STOP — TORQUE OFF ALL", style="Stop.TButton", command=self.stop_all).pack(
            side="left", fill="x", expand=True
        )
        ttk.Button(safety_bar, text="Enable All (Hold)", command=self.enable_all).pack(side="left", padx=(8, 0))
        ttk.Button(safety_bar, text="Move All to Goals", command=self.move_all).pack(side="left", padx=(4, 0))
        ttk.Button(safety_bar, text="Goals ← Current", command=self.copy_current).pack(side="left", padx=(4, 0))

        # -------------------------------------------------------------
        # Main Body: Paned / Two-Column (Left: Arm Servos, Right: VRing & Presets)
        # -------------------------------------------------------------
        content = ttk.Frame(outer)
        content.pack(fill="both", expand=True)

        # LEFT COLUMN: Servos Table
        left_frame = ttk.Frame(content)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))

        table = ttk.LabelFrame(left_frame, text="Cyton Servos (IDs 8–15)", padding=6)
        table.pack(fill="both", expand=True)
        table.columnconfigure(3, weight=1)

        headers = ("Servo", "Pos", "Goal", "Position Slider", "Spd", "Torque", "", "", "", "Status")
        for col, text in enumerate(headers):
            ttk.Label(table, text=text, anchor="center").grid(row=0, column=col, padx=2, sticky="ew")
        ttk.Separator(table).grid(row=1, column=0, columnspan=10, sticky="ew", pady=(2, 2))

        self.rows = {
            servo_id: ServoRow(self, table, idx + 2, servo_id)
            for idx, servo_id in enumerate(SERVO_IDS)
        }

        # Preset Buttons Subframe under the table
        preset_frame = ttk.LabelFrame(left_frame, text="Pose Presets & Calibration Setup", padding=6)
        preset_frame.pack(fill="x", pady=(6, 0))

        ttk.Button(
            preset_frame,
            text="Straightened Starting Setup",
            command=lambda: self.load_preset(PRESET_STRAIGHTENED, name="Straightened Starting Pose"),
        ).pack(side="left", padx=3)

        ttk.Button(
            preset_frame,
            text="Home (All 2048)",
            command=lambda: self.load_preset(PRESET_HOME, name="Home Straight Up"),
        ).pack(side="left", padx=3)

        ttk.Button(
            preset_frame,
            text="Example (ID12 -90°)",
            command=lambda: self.load_preset(PRESET_USER_EXAMPLE, name="Home + ID12 -90° [1060], gripper 400"),
        ).pack(side="left", padx=3)

        ttk.Button(
            preset_frame,
            text="Execute Preset Move",
            command=self.move_all,
        ).pack(side="right", padx=3)

        # RIGHT COLUMN: VRing 2 Telemetry & Sync Logging
        right_frame = ttk.Frame(content, width=360)
        right_frame.pack(side="right", fill="both", padx=(6, 0))

        vring_card = ttk.LabelFrame(right_frame, text="VRing 2 Live BLE Telemetry", padding=8)
        vring_card.pack(fill="both", expand=True)

        # VRing Device Selector
        dev_box = ttk.LabelFrame(vring_card, text="VRing Bluetooth Device", padding=6)
        dev_box.pack(fill="x", pady=(0, 6))

        dev_row = ttk.Frame(dev_box)
        dev_row.pack(fill="x")
        self.ring_combo = ttk.Combobox(dev_row, textvariable=self.selected_ring_var, state="readonly", width=18)
        self.ring_combo.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.scan_btn = ttk.Button(dev_row, text="Scan", width=5, command=self.scan_rings)
        self.scan_btn.pack(side="left", padx=2)
        self.ble_connect_btn = ttk.Button(dev_row, text="Connect", width=8, command=self.toggle_ring_connect)
        self.ble_connect_btn.pack(side="left", padx=(2, 0))

        ttk.Label(dev_box, textvariable=self.ble_info_var, foreground="#555", wraplength=320).pack(anchor="w", pady=(3, 0))

        # IMU Orientation Card
        ori_box = ttk.LabelFrame(vring_card, text="IMU Orientation (Madgwick AHRS)", padding=6)
        ori_box.pack(fill="x", pady=(0, 6))
        ttk.Label(ori_box, textvariable=self.vring_euler_var, font=(family, 11, "bold")).pack(anchor="w")
        ttk.Label(ori_box, textvariable=self.vring_quat_var, foreground="#555").pack(anchor="w")

        # Accelerometer Card
        acc_box = ttk.LabelFrame(vring_card, text="LSM6DS3 Accelerometer (±16g)", padding=6)
        acc_box.pack(fill="x", pady=(0, 6))
        ttk.Label(acc_box, textvariable=self.vring_accel_var, font=(family, 11)).pack(anchor="w")

        gyro_box = ttk.LabelFrame(vring_card, text="LSM6DS3 Gyroscope (±2000°/s)", padding=6)
        gyro_box.pack(fill="x", pady=(0, 6))
        ttk.Label(gyro_box, textvariable=self.vring_gyro_var, font=(family, 11)).pack(anchor="w")

        # Touch & Battery Card
        touch_box = ttk.LabelFrame(vring_card, text="Touch, Battery & Environment", padding=6)
        touch_box.pack(fill="x", pady=(0, 6))
        ttk.Label(touch_box, textvariable=self.vring_touch_var).pack(anchor="w")
        ttk.Label(touch_box, textvariable=self.vring_diag_var).pack(anchor="w")
        ttk.Label(touch_box, textvariable=self.vring_rate_var, foreground="#777").pack(anchor="w")

        # Haptics Control
        haptic_box = ttk.LabelFrame(vring_card, text="Ring Haptic Vibration Test", padding=6)
        haptic_box.pack(fill="x", pady=(0, 6))
        haptic_inner = ttk.Frame(haptic_box)
        haptic_inner.pack(fill="x")
        self.vibe_power_var = tk.IntVar(value=150)
        ttk.Scale(haptic_inner, from_=0, to=255, variable=self.vibe_power_var, orient="horizontal").pack(
            side="left", fill="x", expand=True, padx=(0, 6)
        )
        ttk.Button(haptic_inner, text="Pulse Buzz", command=self._pulse_buzz).pack(side="right")

        # Calibration Data Logging Card
        log_box = ttk.LabelFrame(vring_card, text="Arm + IMU Calibration Sync Logger", padding=6)
        log_box.pack(fill="x", pady=(6, 0))
        ttk.Label(log_box, textvariable=self.log_status_var, wraplength=320).pack(anchor="w", pady=(0, 4))
        log_btns = ttk.Frame(log_box)
        log_btns.pack(fill="x")
        ttk.Button(log_btns, text="📸 Record Data Point", command=self.record_calibration_point).pack(
            side="left", fill="x", expand=True, padx=(0, 4)
        )
        ttk.Button(log_btns, text="New CSV", command=self.start_new_calibration_csv).pack(side="left", padx=(0, 4))
        ttk.Button(log_btns, text="Clear CSV", command=self.clear_calibration_csv).pack(side="left", padx=(0, 4))
        ttk.Button(log_btns, text="Open CSV", command=self._open_log_folder).pack(side="right")

        sequence_btns = ttk.Frame(log_box)
        sequence_btns.pack(fill="x", pady=(4, 0))
        ttk.Button(
            sequence_btns,
            text="Run 6-Position Capture",
            command=self.start_calibration_sequence,
        ).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(
            sequence_btns,
            text="Abort + STOP",
            command=self.abort_calibration_sequence,
        ).pack(side="right")
        ttk.Button(
            log_box,
            text="Save 3-Trial Accel + Gyro Calibration to Ring",
            command=self.apply_csv_calibration_to_firmware,
        ).pack(fill="x", pady=(4, 0))
        tk.Button(
            log_box,
            text="CLEAR ALL CALIBRATION HISTORY",
            foreground="#b00020",
            activeforeground="#b00020",
            command=self.clear_calibration_history,
        ).pack(fill="x", pady=(4, 0))

        # -------------------------------------------------------------
        # Bottom Console / Gateway Log
        # -------------------------------------------------------------
        bottom_status = ttk.Frame(outer)
        bottom_status.pack(fill="x", pady=(6, 0))
        ttk.Label(bottom_status, textvariable=self.status_var).pack(side="left")

        console_frame = ttk.LabelFrame(outer, text="Gateway Log / Manual Command", padding=4)
        console_frame.pack(fill="x", pady=(4, 0))
        self.log = tk.Text(console_frame, height=4, wrap="none", state="disabled")
        self.log.pack(fill="x")

        command_line = ttk.Frame(console_frame)
        command_line.pack(fill="x", pady=(3, 0))
        self.manual_var = tk.StringVar()
        entry = ttk.Entry(command_line, textvariable=self.manual_var)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(command_line, text="Send", command=self.send_manual).pack(side="left", padx=(4, 0))
        ttk.Button(command_line, text="Scan Bus", command=lambda: self.send("SCAN")).pack(side="left", padx=(4, 0))
        ttk.Button(command_line, text="Help", command=lambda: self.send("HELP")).pack(side="left", padx=(4, 0))

    # ------------------------------------------------------------------
    # Serial Port Management
    # ------------------------------------------------------------------
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
                self.set_status("No USB-serial bridge detected; select port manually", error=True)
            else:
                self.set_status("No serial ports found; plug in the M5 bridge", error=True)

    def connect(self) -> None:
        if self.link.connected:
            return
        port_name = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not port_name:
            self.set_status("Pick a valid serial port first", error=True)
            return
        try:
            self.link.connect(port_name)
            self.connection_var.set(f"Gateway: {os.path.basename(port_name)}")
            self.connect_button.configure(text="Disconnect", command=self.disconnect)
            self.set_status(f"Connected to {port_name}; sending STOP...")
            self.send("STOP")
            self.send("PING")
            self.send("SCAN")
            for sid in SERVO_IDS:
                self.send(f"LIMITS {sid}")
            self.send("STATUS")
        except (serial.SerialException, OSError) as exc:
            self.set_status(f"Connect failed: {exc}", error=True)

    def disconnect(self) -> None:
        if self.link.connected:
            try:
                self.send("STOP")
            except Exception:
                pass
        self.link.disconnect()
        self.connection_var.set("Gateway: Disconnected")
        self.connect_button.configure(text="Connect", command=self.connect)
        self.set_status("Disconnected from serial gateway")

    def send(self, cmd: str, coalesce_key: str | None = None) -> None:
        self.link.send(cmd, coalesce_key=coalesce_key)

    def send_manual(self) -> None:
        text = self.manual_var.get().strip()
        if text:
            self.send(text)
            self.manual_var.set("")

    def stop_all(self) -> None:
        self.send("STOP")
        self.set_status("STOP sent: torque disabled on all servos")

    def enable_all(self) -> None:
        if messagebox.askyesno(
            "Enable All Servos",
            "Each servo will first copy its present position into goal, then enable torque.\nProceed?",
            parent=self.root,
        ):
            self.send("ENABLE ALL")

    def move_all(self) -> None:
        goals = [str(self.rows[sid].goal) for sid in SERVO_IDS]
        speed = max(1, min(100, self.rows[SERVO_IDS[0]].speed_var.get()))
        cmd = f"GROUP {speed} " + " ".join(goals)
        self.send(cmd, coalesce_key="group_move")
        self.set_status(f"Sent GROUP move (speed {speed})")

    def copy_current(self) -> None:
        for sid in SERVO_IDS:
            row = self.rows[sid]
            row.set_goal(row.present)
        self.set_status("All goals copied from current positions")

    def load_preset(self, preset: dict[int, int], name: str = "Preset") -> None:
        for sid, goal in preset.items():
            if sid in self.rows:
                self.rows[sid].set_goal(goal)
        self.set_status(f"Loaded '{name}' into servo goals")

    # ------------------------------------------------------------------
    # Telemetry Polling & Event Drain
    # ------------------------------------------------------------------
    def _poll_status(self) -> None:
        if self.link.connected and self.poll_enabled.get():
            self.send("STATUS", coalesce_key="status")
        self.link.check_timeout()
        self.root.after(1400, self._poll_status)

    def _drain_serial_events(self) -> None:
        try:
            while True:
                kind, payload = self.serial_events.get_nowait()
                if kind == "rx":
                    self._append_log(f"< {payload}")
                    self._handle_serial_line(payload)
                elif kind == "tx":
                    self._append_log(f"> {payload}")
                elif kind == "timeout":
                    self.set_status(f"Command timed out: {payload}", error=True)
                elif kind == "error":
                    self.set_status(f"Serial error: {payload}", error=True)
                    self.disconnect()
        except queue.Empty:
            pass
        self.root.after(40, self._drain_serial_events)

    def _handle_serial_line(self, line: str) -> None:
        m = STATUS_RE.match(line)
        if m:
            sid = int(m.group(1))
            if sid in self.rows:
                self.rows[sid].update_status(
                    model=m.group(2),
                    position=int(m.group(3)),
                    reported_goal=int(m.group(4)),
                    torque=bool(int(m.group(5))),
                    moving=bool(int(m.group(6))),
                    volt_x10=int(m.group(7)),
                    temp_c=int(m.group(8)),
                )
            return

        m_scan = SCAN_RE.match(line)
        if m_scan:
            sid = int(m_scan.group(1))
            if sid in self.rows:
                self.rows[sid].model = m_scan.group(2)
                self.rows[sid].id_label.configure(text=f"ID {sid}\n{m_scan.group(2)}")
            return

        m_lim = LIMITS_RE.match(line)
        if m_lim:
            sid = int(m_lim.group(1))
            if sid in self.rows:
                self.rows[sid].set_limits(int(m_lim.group(2)), int(m_lim.group(3)))
            return

        if line.startswith("OK STOP"):
            for row in self.rows.values():
                row.torque = False
                row.torque_button.configure(text="Enable")
            self.set_status("All servos stopped (torque off)")

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_status(self, msg: str, error: bool = False) -> None:
        self.status_var.set(msg)

    # ------------------------------------------------------------------
    # VRing BLE Event Drain & UI Update
    # ------------------------------------------------------------------
    def scan_rings(self) -> None:
        """Ask the BLE worker to discover nearby VRING devices."""
        if not BLEAK_AVAILABLE:
            self.ble_info_var.set("BLE is unavailable: install the bleak package")
            return
        if self.ring_state.connected:
            self.ble_info_var.set("Disconnect the current VRING before scanning again")
            return
        self.ble_info_var.set("Scanning for nearby VRING devices...")
        self.ble_worker.scan()

    def toggle_ring_connect(self) -> None:
        """Connect to the selected ring, or disconnect the current one."""
        if not BLEAK_AVAILABLE:
            self.ble_info_var.set("BLE is unavailable: install the bleak package")
            return
        if self.ring_state.connected:
            self.ble_worker.disconnect()
            self.ble_info_var.set("Disconnecting from VRING...")
            return

        selection = self.selected_ring_var.get()
        address = self.ring_addr_map.get(selection)
        if not address:
            self.ble_info_var.set("Choose a discovered VRING device, then click Connect")
            return
        self.ble_info_var.set(f"Connecting to {selection}...")
        self.ble_worker.connect(address)

    def _drain_ble_events(self) -> None:
        try:
            while True:
                kind, payload = self.ble_events.get_nowait()
                if kind == "devices":
                    self.ring_addr_map.clear()
                    labels: list[str] = []
                    for address, name, rssi in payload:
                        label = f"{name} ({address[-8:]}, {rssi} dBm)"
                        self.ring_addr_map[label] = address
                        labels.append(label)
                    self.ring_combo["values"] = labels
                    if labels:
                        if self.selected_ring_var.get() not in self.ring_addr_map:
                            self.selected_ring_var.set(labels[0])
                        self.ble_info_var.set(f"Found {len(labels)} VRING device(s)")
                    else:
                        self.selected_ring_var.set("")
                        self.ble_info_var.set("No VRING devices found; check that the ring is powered")
                elif kind == "connected":
                    name, addr, rssi = payload
                    self.ring_state.connected = True
                    self.ring_state.device_name = name
                    self.ring_state.device_address = addr
                    self.ring_state.rssi = rssi
                    self.vring_status_var.set(f"VRing: Connected ({name}, {rssi} dBm)")
                    self.ble_connect_btn.configure(text="Disconnect")
                    self.scan_btn.configure(state="disabled")
                elif kind == "disconnected":
                    self.ring_state.connected = False
                    self.ring_state.calibration_values = None
                    self.vring_status_var.set(f"VRing: Disconnected ({payload})")
                    self.ble_connect_btn.configure(text="Connect")
                    self.scan_btn.configure(state="normal")
                elif kind == "status":
                    self.vring_status_var.set(f"VRing: {payload}")
                elif kind == "calibration_loaded":
                    self.ring_state.calibration_values = payload
                elif kind == "calibration_saved":
                    self.ring_state.calibration_values = payload
                    self._commit_pending_calibration_history()
                elif kind == "calibration_failed":
                    self.pending_history_entry = None
                    self.set_status(f"Could not save ring calibration: {payload}", error=True)
                elif kind == "button":
                    self.ring_state.button_pressed = payload
                elif kind == "sensor":
                    self._update_vring_data(payload)
        except queue.Empty:
            pass
        self.root.after(30, self._drain_ble_events)

    def _update_vring_data(self, payload: bytes) -> None:
        now = time.time()
        data, accel_vec, quat_vec = parse_vring_packet(payload)
        if not data:
            return

        # Rate tracking
        if self.ring_state.last_packet_time > 0:
            dt = now - self.ring_state.last_packet_time
            if dt > 0:
                inst_rate = 1.0 / dt
                self.ring_state.packet_rate_hz = 0.9 * self.ring_state.packet_rate_hz + 0.1 * inst_rate
        self.ring_state.last_packet_time = now
        self.ring_state.packets_received += 1

        # Store in state
        for k, v in data.items():
            setattr(self.ring_state, k, v)
        # The packet's compact names differ from the CSV state's descriptive
        # names.  Copy them explicitly so the logger records live values rather
        # than the dataclass defaults.
        self.ring_state.accel_x_g = data["ax_g"]
        self.ring_state.accel_y_g = data["ay_g"]
        self.ring_state.accel_z_g = data["az_g"]
        self.ring_state.accel_mag_g = data["mag_g"]
        self.ring_state.gyro_x_dps = data["gx_dps"]
        self.ring_state.gyro_y_dps = data["gy_dps"]
        self.ring_state.gyro_z_dps = data["gz_dps"]
        self.ring_state.quat_w = data["qw"]
        self.ring_state.quat_x = data["qx"]
        self.ring_state.quat_y = data["qy"]
        self.ring_state.quat_z = data["qz"]

        # Update UI strings
        self.vring_euler_var.set(
            f"Roll: {data['roll_deg']:+6.1f}°  Pitch: {data['pitch_deg']:+6.1f}°  Yaw: {data['yaw_deg']:+6.1f}°"
        )
        self.vring_quat_var.set(
            f"w: {quat_vec[0]:.3f}  x: {quat_vec[1]:.3f}  y: {quat_vec[2]:.3f}  z: {quat_vec[3]:.3f}"
        )
        self.vring_accel_var.set(
            f"X: {data['ax_g']:+5.2f}g   Y: {data['ay_g']:+5.2f}g   Z: {data['az_g']:+5.2f}g  (|a|: {data['mag_g']:.2f}g)"
        )
        gyro_label = "Gyro:" if data["gyro_available"] else "Gyro: reflash ring"
        self.vring_gyro_var.set(
            f"{gyro_label} X: {data['gx_dps']:+6.2f}°/s  Y: {data['gy_dps']:+6.2f}°/s  Z: {data['gz_dps']:+6.2f}°/s"
        )
        if self.gyro_hold_index is not None and data["gyro_available"]:
            self.gyro_hold_samples.append((data["gx_dps"], data["gy_dps"], data["gz_dps"]))
        if self.gyro_sweep_stage == "recording" and data["gyro_available"]:
            self.gyro_sweep_samples.append({
                "sample_counter": data["sample_counter"],
                "gyro_x_dps": data["gx_dps"],
                "gyro_y_dps": data["gy_dps"],
                "gyro_z_dps": data["gz_dps"],
            })
        if self.gyro_transition_capture is not None and data["gyro_available"]:
            self.gyro_transition_samples.append({
                "sample_counter": data["sample_counter"],
                "gyro_x_dps": data["gx_dps"],
                "gyro_y_dps": data["gy_dps"],
                "gyro_z_dps": data["gz_dps"],
            })
        batt_pct = max(0, min(100, int((data['battery_mv'] - 3400) / (4200 - 3400) * 100)))
        btn_txt = " [BTN PRESSED]" if self.ring_state.button_pressed else ""
        self.vring_diag_var.set(
            f"Batt: {data['battery_mv']} mV ({batt_pct}%) | Temp: {data['temp_c']:.1f}°C ({data['temp_f']:.1f}°F){btn_txt}"
        )
        self.vring_touch_var.set(
            f"Touch: Cap1: {data['cap1']}  Cap2: {data['cap2']}"
        )
        self.vring_rate_var.set(
            f"Rate: {self.ring_state.packet_rate_hz:.1f} Hz | Counter: {data['sample_counter']}"
        )

    def _pulse_buzz(self) -> None:
        pwr = self.vibe_power_var.get()
        self.ble_worker.send_vibration(pwr)
        # Turn off vibration after 200 ms
        self.root.after(200, lambda: self.ble_worker.send_vibration(0))

    # ------------------------------------------------------------------
    # Data Logging for IMU Calibration
    # ------------------------------------------------------------------
    def _most_recent_calibration_csv(self) -> str:
        folder = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(folder, filename)
            for filename in os.listdir(folder)
            if filename.startswith("vring_calibration_") and filename.endswith(".csv")
            and os.path.getsize(os.path.join(folder, filename)) > 0
        ]
        if candidates:
            return max(candidates, key=os.path.getmtime)
        return os.path.join(folder, "vring_calibration_data.csv")

    @staticmethod
    def _calibration_csv_row_count(path: str) -> int:
        try:
            with open(path, newline="", encoding="utf-8") as f:
                return sum(1 for _row in csv.DictReader(f))
        except OSError:
            return 0

    def start_new_calibration_csv(self) -> None:
        """Select a fresh timestamped file without overwriting prior captures."""
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        folder = os.path.dirname(os.path.abspath(__file__))
        self.log_file_path = os.path.join(folder, f"vring_calibration_{stamp}.csv")
        self.logged_points = 0
        self.log_status_var.set(f"Log: {os.path.basename(self.log_file_path)} (new; 0 pts)")
        self.set_status("New calibration CSV selected; it will be created at the first capture")

    def clear_calibration_csv(self) -> None:
        """Erase rows in the currently selected CSV after an explicit confirmation."""
        filename = os.path.basename(self.log_file_path)
        if not messagebox.askyesno(
            "Clear calibration CSV",
            f"Permanently erase all recorded rows in {filename}?",
            icon="warning",
            parent=self.root,
        ):
            return
        try:
            # Truncate rather than remove the file so its selected path remains valid.
            with open(self.log_file_path, "w", encoding="utf-8", newline=""):
                pass
            self.logged_points = 0
            self.log_status_var.set(f"Log: {filename} (cleared; 0 pts)")
            self.set_status(f"Cleared {filename}")
        except OSError as exc:
            self.set_status(f"Could not clear CSV: {exc}", error=True)

    def _load_calibration_history(self) -> list[dict[str, Any]]:
        try:
            with open(self.calibration_history_path, encoding="utf-8") as f:
                data = json.load(f)
            batches = data.get("batches", [])
            if not isinstance(batches, list):
                raise ValueError("batches is not a list")
            valid = [batch for batch in batches if isinstance(batch, dict) and len(batch.get("values", [])) == 12]
            if len(valid) != len(batches):
                raise ValueError("contains an invalid calibration batch")
            return valid
        except FileNotFoundError:
            return []
        except (OSError, ValueError, json.JSONDecodeError):
            return []

    def _write_calibration_history(self) -> None:
        with open(self.calibration_history_path, "w", encoding="utf-8") as f:
            json.dump({"version": 1, "batches": self.calibration_history}, f, indent=2)

    def _commit_pending_calibration_history(self) -> None:
        if self.pending_history_entry is None:
            return
        self.calibration_history.append(self.pending_history_entry)
        self.pending_history_entry = None
        try:
            self._write_calibration_history()
        except OSError as exc:
            self.set_status(f"Calibration saved to ring, but could not save desktop history: {exc}", error=True)
            return
        self.set_status(
            f"Calibration saved to the ring and added to history ({len(self.calibration_history)} batches). "
            "It will survive power cycles; no reflash needed."
        )

    def clear_calibration_history(self) -> None:
        if not messagebox.askyesno(
            "Clear calibration history",
            "This removes every accumulated calibration batch from this computer. "
            "The ring's currently active calibration will remain in use. Continue?",
            icon="warning",
            parent=self.root,
        ):
            return
        if not messagebox.askyesno(
            "Final warning: clear all history",
            "This cannot be undone. Future saves will start a new averaging baseline. Clear all calibration history?",
            icon="warning",
            parent=self.root,
        ):
            return
        self.calibration_history = []
        self.pending_history_entry = None
        try:
            self._write_calibration_history()
            self.set_status("All desktop calibration history cleared. The ring's active constants were not changed.")
        except OSError as exc:
            self.set_status(f"Could not clear calibration history: {exc}", error=True)

    def start_calibration_sequence(self) -> None:
        """Move through the six supplied poses and capture one row at each midpoint."""
        if self.calibration_sequence_index is not None:
            self.set_status("Six-position capture is already running", error=True)
            return
        if not self.link.connected:
            self.set_status("Connect the Cyton gateway before starting the capture", error=True)
            return
        if not self.ring_state.connected:
            self.set_status("Connect the VRING before starting the capture", error=True)
            return
        if not all(row.torque for row in self.rows.values()):
            self.set_status("Enable all servos manually and confirm telemetry before starting", error=True)
            return
        if not messagebox.askyesno(
            "Run six-position calibration capture",
            "The arm will move through six stored positions. It waits until telemetry confirms "
            "each target is reached before beginning a 3-second hold, then saves one IMU/servo "
            "row 1.5 seconds into that hold.\n\n"
            "Confirm the workspace is clear, the arm is supported, and the gripper moves freely without heating.",
            parent=self.root,
        ):
            return

        self.calibration_sequence_index = 0
        self.gyro_sweep_stage = None
        self.gyro_sweep_samples.clear()
        self.gyro_transition_capture = None
        self.gyro_transition_samples.clear()
        self._run_next_calibration_pose()

    def _run_next_calibration_pose(self) -> None:
        if self.calibration_sequence_index is None:
            return
        index = self.calibration_sequence_index
        if index >= len(CALIBRATION_POSES):
            self._start_gyro_gain_sweep()
            return

        label, pose = CALIBRATION_POSES[index]
        self.load_preset(pose, name=f"Calibration {label} ({index + 1}/{len(CALIBRATION_POSES)})")
        if index > 0:
            previous_label, _previous_pose = CALIBRATION_POSES[index - 1]
            transition = GYRO_GAIN_TRANSITIONS.get((previous_label, label))
            if transition is not None:
                self.gyro_transition_capture = dict(transition, from_pose=previous_label, to_pose=label)
                self.gyro_transition_samples.clear()
        self.move_all()
        self.calibration_wait_started = time.monotonic()
        self.set_status(f"Moving to {label}; waiting for all servos to reach target")
        self.calibration_after_ids.append(
            self.root.after(CALIBRATION_STATUS_POLL_MS, lambda: self._wait_for_calibration_pose(index, pose))
        )

    def _start_gyro_gain_sweep(self) -> None:
        """Return near center, then record the single-joint 90 degree wrist-pitch turn."""
        if self.calibration_sequence_index is None:
            return
        self.gyro_sweep_stage = "move_to_start"
        self.load_preset(GYRO_GAIN_SWEEP_START, name="Gyro X gain start (servo 14 rotated, servo 13 at 2048)")
        self.move_all()
        self.gyro_sweep_started = time.monotonic()
        self.set_status("Moving to gyro X gain start; servo 14 uses the existing +89.47° calibration orientation")
        self.calibration_after_ids.append(
            self.root.after(CALIBRATION_STATUS_POLL_MS, self._wait_for_gyro_gain_sweep_start)
        )

    def _wait_for_gyro_gain_sweep_start(self) -> None:
        if self.gyro_sweep_stage != "move_to_start":
            return
        self.send("STATUS", coalesce_key="status")
        at_start = all(
            not self.rows[servo_id].moving
            and abs(self.rows[servo_id].present - target) <= CALIBRATION_POSITION_TOLERANCE_TICKS
            for servo_id, target in GYRO_GAIN_SWEEP_START.items()
        )
        if at_start:
            self.gyro_sweep_samples.clear()
            self.gyro_sweep_stage = "recording"
            self.load_preset(GYRO_GAIN_SWEEP_END, name="Gyro gain sweep: servo 13 +90°")
            self.move_all()
            self.gyro_sweep_started = time.monotonic()
            self.set_status("Recording gyro sweep: servo 13 moving 2048 → 3072 (nominal +90.0°)")
            self.calibration_after_ids.append(
                self.root.after(CALIBRATION_STATUS_POLL_MS, self._wait_for_gyro_gain_sweep_end)
            )
            return
        if (time.monotonic() - self.gyro_sweep_started) * 1000 >= GYRO_GAIN_SWEEP_TIMEOUT_MS:
            self.abort_calibration_sequence(reason="Gyro gain sweep did not reach its start pose within 20 seconds")
            return
        self.calibration_after_ids.append(
            self.root.after(CALIBRATION_STATUS_POLL_MS, self._wait_for_gyro_gain_sweep_start)
        )

    def _wait_for_gyro_gain_sweep_end(self) -> None:
        if self.gyro_sweep_stage != "recording":
            return
        self.send("STATUS", coalesce_key="status")
        at_end = all(
            not self.rows[servo_id].moving
            and abs(self.rows[servo_id].present - target) <= CALIBRATION_POSITION_TOLERANCE_TICKS
            for servo_id, target in GYRO_GAIN_SWEEP_END.items()
        )
        if at_end:
            self.gyro_sweep_stage = None
            self._write_gyro_gain_sweep_csv()
            self.calibration_sequence_index = None
            self.calibration_after_ids.clear()
            self.set_status(
                f"Calibration complete: {self.logged_points} stationary points and "
                f"{len(self.gyro_sweep_samples)} gyro sweep samples saved"
            )
            return
        if (time.monotonic() - self.gyro_sweep_started) * 1000 >= GYRO_GAIN_SWEEP_TIMEOUT_MS:
            self.abort_calibration_sequence(reason="Gyro gain sweep did not reach its 90° end pose within 20 seconds")
            return
        self.calibration_after_ids.append(
            self.root.after(CALIBRATION_STATUS_POLL_MS, self._wait_for_gyro_gain_sweep_end)
        )

    def _write_gyro_gain_sweep_csv(self) -> None:
        """Write raw ring-timed gyro samples needed to integrate the known 90 degree turn."""
        if len(self.gyro_sweep_samples) < 10:
            self.set_status("Gyro sweep completed but fewer than 10 gyro samples were received", error=True)
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        folder = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(folder, f"vring_gyro_gain_sweep_{stamp}.csv")
        fieldnames = ["stationary_csv", "reference_angle_deg", "sample_counter", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for sample in self.gyro_sweep_samples:
                    writer.writerow({
                        "stationary_csv": os.path.basename(self.log_file_path),
                        "reference_angle_deg": GYRO_GAIN_SWEEP_ANGLE_DEG,
                        **sample,
                    })
        except OSError as exc:
            self.set_status(f"Could not save gyro gain sweep: {exc}", error=True)

    def _wait_for_calibration_pose(self, index: int, pose: dict[int, int]) -> None:
        """Begin the hold only after fresh telemetry shows every joint at its target."""
        if self.calibration_sequence_index != index:
            return

        self.send("STATUS", coalesce_key="status")
        at_target = all(
            not self.rows[servo_id].moving
            and abs(self.rows[servo_id].present - target) <= CALIBRATION_POSITION_TOLERANCE_TICKS
            for servo_id, target in pose.items()
        )
        if at_target:
            label, _pose = CALIBRATION_POSES[index]
            self._finish_gyro_transition_capture()
            self.gyro_hold_index = index
            self.gyro_hold_samples.clear()
            self.set_status(f"{label} reached; holding 3.0 s and averaging stationary gyro through 1.5 s")
            self.calibration_after_ids.append(
                self.root.after(CALIBRATION_CAPTURE_MS, lambda: self._capture_calibration_pose(index))
            )
            self.calibration_after_ids.append(
                self.root.after(CALIBRATION_HOLD_MS, self._advance_calibration_pose)
            )
            return

        if (time.monotonic() - self.calibration_wait_started) * 1000 >= CALIBRATION_REACH_TIMEOUT_MS:
            label, _pose = CALIBRATION_POSES[index]
            self.abort_calibration_sequence(reason=f"{label} did not reach its target within 20 seconds")
            return

        self.calibration_after_ids.append(
            self.root.after(CALIBRATION_STATUS_POLL_MS, lambda: self._wait_for_calibration_pose(index, pose))
        )

    def _finish_gyro_transition_capture(self) -> None:
        """Save one raw trace for a known single-joint transition, if this pose ended one."""
        capture = self.gyro_transition_capture
        self.gyro_transition_capture = None
        if capture is None:
            return
        if len(self.gyro_transition_samples) < 10:
            self.set_status(f"{capture['name']} transition had fewer than 10 gyro samples", error=True)
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            f"vring_gyro_gain_{capture['name']}_{stamp}.csv",
        )
        fieldnames = [
            "stationary_csv", "transition", "servo_id", "servo_start_ticks", "servo_end_ticks",
            "reference_angle_deg", "sample_counter", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
        ]
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for sample in self.gyro_transition_samples:
                    writer.writerow({
                        "stationary_csv": os.path.basename(self.log_file_path),
                        "transition": f"{capture['from_pose']} to {capture['to_pose']}",
                        "servo_id": capture["servo_id"],
                        "servo_start_ticks": capture["start"],
                        "servo_end_ticks": capture["end"],
                        "reference_angle_deg": f"{capture['reference_angle_deg']:.6f}",
                        **sample,
                    })
        except OSError as exc:
            self.set_status(f"Could not save {capture['name']} gyro trace: {exc}", error=True)

    def _capture_calibration_pose(self, index: int) -> None:
        if self.calibration_sequence_index != index:
            return
        label, _pose = CALIBRATION_POSES[index]
        if not self._imu_sample_is_valid():
            magnitude = self.ring_state.accel_mag_g
            self.abort_calibration_sequence(
                reason=(
                    f"{label} rejected: IMU magnitude {magnitude:.2f} g is outside "
                    f"{CALIBRATION_IMU_MAGNITUDE_MIN_G:.2f}-{CALIBRATION_IMU_MAGNITUDE_MAX_G:.2f} g"
                )
            )
            return
        if self.gyro_hold_index != index or len(self.gyro_hold_samples) < 10:
            self.abort_calibration_sequence(
                reason=f"{label} rejected: fewer than 10 stationary gyro samples; reflash the ring and reconnect"
            )
            return
        gyro_average = tuple(
            statistics.fmean(sample[axis] for sample in self.gyro_hold_samples)
            for axis in range(3)
        )
        gyro_sample_count = len(self.gyro_hold_samples)
        self.gyro_hold_index = None
        self.record_calibration_point(
            pose_label=label,
            gyro_bias_dps=gyro_average,
            gyro_sample_count=gyro_sample_count,
        )

    def _imu_sample_is_valid(self) -> bool:
        magnitude = self.ring_state.accel_mag_g
        return (
            math.isfinite(magnitude)
            and CALIBRATION_IMU_MAGNITUDE_MIN_G <= magnitude <= CALIBRATION_IMU_MAGNITUDE_MAX_G
        )

    def apply_csv_calibration_to_firmware(self) -> None:
        """Apply three fresh accel holds and three ring-timed gyro rotations."""
        samples: dict[str, list[dict[str, float]]] = {label: [] for label, _pose in CALIBRATION_POSES}
        try:
            with open(self.log_file_path, newline="", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    label = row.get("calibration_pose", "")
                    if label not in samples:
                        continue
                    if row.get("all_servos_settled") != "1" or row.get("imu_sample_valid") != "1":
                        continue
                    sample = {axis: float(row[axis]) for axis in ("accel_x_g", "accel_y_g", "accel_z_g")}
                    sample.update({axis: float(row[axis]) for axis in (
                        "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps"
                    )})
                    sample["gyro_hold_samples"] = float(row.get("gyro_hold_samples", "0"))
                    samples[label].append(sample)
        except (OSError, ValueError) as exc:
            self.set_status(f"Could not read calibration CSV: {exc}", error=True)
            return

        missing = [label for label, rows in samples.items() if len(rows) < CALIBRATION_MIN_TRIALS]
        if missing:
            self.set_status(
                f"Need {CALIBRATION_MIN_TRIALS} settled, valid rows for each pose; missing: {', '.join(missing)}",
                error=True,
            )
            return

        # Use the latest three valid captures per pose so an older run in the same
        # CSV cannot silently dominate a newly collected three-trial calibration.
        means = {
            label: {
                axis: statistics.fmean(sample[axis] for sample in rows[-CALIBRATION_MIN_TRIALS:])
                for axis in ("accel_x_g", "accel_y_g", "accel_z_g")
            }
            for label, rows in samples.items()
        }
        biases: list[float] = []
        gains: list[float] = []
        for axis_name, positive, negative in (
            ("accel_x_g", "+X", "-X"),
            ("accel_y_g", "+Y", "-Y"),
            ("accel_z_g", "+Z", "-Z"),
        ):
            plus = means[positive][axis_name]
            minus = means[negative][axis_name]
            bias = (plus + minus) / 2.0
            gain = (plus - minus) / 2.0
            if not 0.80 <= gain <= 1.20:
                self.set_status(f"Rejected {axis_name}: calculated gain {gain:.4f} is implausible", error=True)
                return
            biases.append(bias)
            gains.append(gain)

        stored = self.ring_state.calibration_values
        if stored is None:
            self.set_status(
                "The ring has not supplied its stored calibration. Reconnect after flashing the Bluetooth-storage firmware.",
                error=True,
            )
            return
        existing_accel_bias = list(stored[0:3])
        existing_accel_gain = list(stored[3:6])
        existing_gyro_bias = list(stored[6:9])
        existing_gyro_gain = list(stored[9:12])

        # These are incremental corrections because the CSV contains values already
        # corrected by the firmware.  Compose them with the installed constants so a
        # repeat run refines calibration instead of erasing the prior one.
        accel_bias = [existing_accel_bias[i] + existing_accel_gain[i] * biases[i] for i in range(3)]
        accel_gain = [existing_accel_gain[i] * gains[i] for i in range(3)]

        gyro_rows = [row for pose_rows in samples.values() for row in pose_rows[-CALIBRATION_MIN_TRIALS:]]
        gyro_weights = [max(1.0, row["gyro_hold_samples"]) for row in gyro_rows]
        gyro_incremental_bias = [
            sum(row[axis] * weight for row, weight in zip(gyro_rows, gyro_weights)) / sum(gyro_weights)
            for axis in ("gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps")
        ]

        trace_specs = {
            "servo14_wrist_roll": 2,  # Z
            "servo12_elbow_yaw": 1,   # Y
            "sweep": 0,               # X
        }
        trace_values: dict[str, list[float]] = {name: [] for name in trace_specs}
        trace_folder = os.path.dirname(os.path.abspath(__file__))
        stationary_name = os.path.basename(self.log_file_path)
        try:
            for filename in sorted(os.listdir(trace_folder)):
                if not (filename.startswith("vring_gyro_gain_") and filename.endswith(".csv")):
                    continue
                if filename.startswith("vring_gyro_gain_servo14_wrist_roll_"):
                    trace_name = "servo14_wrist_roll"
                elif filename.startswith("vring_gyro_gain_servo12_elbow_yaw_"):
                    trace_name = "servo12_elbow_yaw"
                elif filename.startswith("vring_gyro_gain_sweep_"):
                    trace_name = "sweep"
                else:
                    continue
                with open(os.path.join(trace_folder, filename), newline="", encoding="utf-8") as f:
                    rows = list(csv.DictReader(f))
                if not rows or rows[0].get("stationary_csv") != stationary_name:
                    continue
                if len(rows) < 10:
                    continue
                reference_angle = abs(float(rows[0]["reference_angle_deg"]))
                if reference_angle < 10.0:
                    continue
                integrated = [0.0, 0.0, 0.0]
                previous = rows[0]
                for current in rows[1:]:
                    delta_ticks = (int(current["sample_counter"]) - int(previous["sample_counter"])) & 0xFFFF
                    # BLE can occasionally drop several packets.  The ring counter
                    # still supplies exact elapsed time, so trapezoidal integration
                    # remains sound across a short gap during these smooth arm moves.
                    # Reject only a materially long gap (over 200 ms).
                    if not 1 <= delta_ticks <= GYRO_TRACE_MAX_GAP_TICKS:
                        raise ValueError(
                            f"{filename} has a ring sample-counter gap over "
                            f"{GYRO_TRACE_MAX_GAP_TICKS / 400.0:.2f} seconds"
                        )
                    dt = delta_ticks * GYRO_SAMPLE_PERIOD_S
                    for axis, bias in enumerate(gyro_incremental_bias):
                        key = ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")[axis]
                        integrated[axis] += 0.5 * ((float(previous[key]) - bias) + (float(current[key]) - bias)) * dt
                    previous = current
                magnitude = math.sqrt(sum(value * value for value in integrated))
                dominant_axis = trace_specs[trace_name]
                if magnitude <= 0.0 or abs(integrated[dominant_axis]) < 0.80 * magnitude:
                    raise ValueError(f"{filename} did not rotate predominantly around its expected gyro axis")
                ratio = magnitude / reference_angle
                if not 0.75 <= ratio <= 1.25:
                    raise ValueError(f"{filename} produced an implausible gyro gain ratio {ratio:.3f}")
                trace_values[trace_name].append(ratio)
        except (OSError, ValueError, KeyError) as exc:
            self.set_status(f"Could not use gyro rotation traces: {exc}", error=True)
            return

        missing_traces = [name for name, values in trace_values.items() if len(values) < CALIBRATION_MIN_TRIALS]
        if missing_traces:
            self.set_status(
                "Need three gyro gain traces from this CSV for: " + ", ".join(missing_traces)
                + ". Start a new CSV and run the full sequence three times.",
                error=True,
            )
            return
        gyro_incremental_gain = [0.0, 0.0, 0.0]
        for trace_name, axis in trace_specs.items():
            gyro_incremental_gain[axis] = statistics.fmean(trace_values[trace_name][-CALIBRATION_MIN_TRIALS:])
        gyro_bias = [existing_gyro_bias[i] + existing_gyro_gain[i] * gyro_incremental_bias[i] for i in range(3)]
        gyro_gain = [existing_gyro_gain[i] * gyro_incremental_gain[i] for i in range(3)]

        # Each saved item is one complete group of three trials expressed in the
        # ring's uncorrected sensor units. Averaging groups is therefore stable
        # even though later CSVs were recorded with the previous calibration active.
        candidate_values = accel_bias + accel_gain + gyro_bias + gyro_gain
        if self.calibration_history:
            prior_average = [
                statistics.fmean(float(batch["values"][axis]) for batch in self.calibration_history)
                for axis in range(12)
            ]
            tolerances = [0.002] * 6 + [0.10] * 3 + [0.002] * 3
            if any(abs(prior_average[axis] - stored[axis]) > tolerances[axis] for axis in range(12)):
                self.set_status(
                    "Stored ring calibration does not match this computer's history. "
                    "Use Clear All Calibration History before starting a new baseline.",
                    error=True,
                )
                return
        all_batches = self.calibration_history + [{"values": candidate_values}]
        final_values = [
            statistics.fmean(float(batch["values"][axis]) for batch in all_batches)
            for axis in range(12)
        ]
        accel_bias, accel_gain = final_values[0:3], final_values[3:6]
        gyro_bias, gyro_gain = final_values[6:9], final_values[9:12]
        candidate_history_entry = {
            "timestamp": datetime.datetime.now().isoformat(),
            "csv": os.path.basename(self.log_file_path),
            "values": candidate_values,
        }

        summary = (
            f"Calculated from {len(all_batches)} equal three-trial batch(es):\n\n"
            f"Accelerometer X/Y/Z: bias {accel_bias[0]:+.5f}, {accel_bias[1]:+.5f}, {accel_bias[2]:+.5f} g; "
            f"gain {accel_gain[0]:.5f}, {accel_gain[1]:.5f}, {accel_gain[2]:.5f}\n"
            f"Gyroscope X/Y/Z: bias {gyro_bias[0]:+.4f}, {gyro_bias[1]:+.4f}, {gyro_bias[2]:+.4f} dps; "
            f"gain {gyro_gain[0]:.5f}, {gyro_gain[1]:.5f}, {gyro_gain[2]:.5f}\n\n"
            "This will send the validated values to the connected ring, where they will be stored across power cycles. "
            "Apply these values?"
        )
        if not messagebox.askyesno("Save accelerometer + gyro calibration", summary, parent=self.root):
            return
        if not self.ring_state.connected:
            self.set_status("Connect the VRING before saving calibration over Bluetooth", error=True)
            return
        self.pending_history_entry = candidate_history_entry
        self.ble_worker.save_calibration(tuple(accel_bias + accel_gain + gyro_bias + gyro_gain))
        self.set_status("Sending accelerometer + gyro calibration to the ring...")

    def _advance_calibration_pose(self) -> None:
        if self.calibration_sequence_index is None:
            return
        self.calibration_sequence_index += 1
        self._run_next_calibration_pose()

    def abort_calibration_sequence(self, reason: str = "Six-position capture aborted") -> None:
        was_running = self.calibration_sequence_index is not None
        self.calibration_sequence_index = None
        self.gyro_sweep_stage = None
        self.gyro_transition_capture = None
        for after_id in self.calibration_after_ids:
            try:
                self.root.after_cancel(after_id)
            except tk.TclError:
                pass
        self.calibration_after_ids.clear()
        if was_running:
            self.stop_all()
            self.set_status(f"{reason}; STOP sent", error=True)

    def record_calibration_point(
        self,
        pose_label: str = "manual",
        gyro_bias_dps: tuple[float, float, float] | None = None,
        gyro_sample_count: int = 0,
    ) -> None:
        timestamp = datetime.datetime.now().isoformat()

        pose_targets = next(
            (pose for label, pose in CALIBRATION_POSES if label == pose_label),
            {servo_id: self.rows[servo_id].goal for servo_id in SERVO_IDS},
        )
        position_errors = {
            servo_id: self.rows[servo_id].present - pose_targets[servo_id]
            for servo_id in SERVO_IDS
        }
        settled = all(
            self.rows[servo_id].torque
            and not self.rows[servo_id].moving
            and abs(position_errors[servo_id]) <= CALIBRATION_POSITION_TOLERANCE_TICKS
            for servo_id in SERVO_IDS
        )
        gyro_bias = gyro_bias_dps or (
            self.ring_state.gyro_x_dps,
            self.ring_state.gyro_y_dps,
            self.ring_state.gyro_z_dps,
        )

        row = {
            "timestamp": timestamp,
            "calibration_pose": pose_label,
            "all_servos_settled": int(settled),
            "imu_sample_valid": int(self._imu_sample_is_valid()),
            "max_position_error_ticks": max(abs(error) for error in position_errors.values()),
            # Robot Joints
            # VRing IMU
            "accel_x_g": f"{self.ring_state.accel_x_g:.4f}",
            "accel_y_g": f"{self.ring_state.accel_y_g:.4f}",
            "accel_z_g": f"{self.ring_state.accel_z_g:.4f}",
            "accel_mag_g": f"{self.ring_state.accel_mag_g:.4f}",
            "gyro_bias_x_dps": f"{gyro_bias[0]:.5f}",
            "gyro_bias_y_dps": f"{gyro_bias[1]:.5f}",
            "gyro_bias_z_dps": f"{gyro_bias[2]:.5f}",
            "gyro_hold_samples": gyro_sample_count,
            "quat_w": f"{self.ring_state.quat_w:.4f}",
            "quat_x": f"{self.ring_state.quat_x:.4f}",
            "quat_y": f"{self.ring_state.quat_y:.4f}",
            "quat_z": f"{self.ring_state.quat_z:.4f}",
            "roll_deg": f"{self.ring_state.roll_deg:.2f}",
            "pitch_deg": f"{self.ring_state.pitch_deg:.2f}",
            "yaw_deg": f"{self.ring_state.yaw_deg:.2f}",
            "raw_ax": self.ring_state.raw_ax,
            "raw_ay": self.ring_state.raw_ay,
            "raw_az": self.ring_state.raw_az,
            "sample_counter": self.ring_state.sample_counter,
        }
        for servo_id in SERVO_IDS:
            servo = self.rows[servo_id]
            row[f"servo_{servo_id}_target"] = pose_targets[servo_id]
            row[f"servo_{servo_id}_reported_goal"] = servo.reported_goal
            row[f"servo_{servo_id}_pos"] = servo.present
            row[f"servo_{servo_id}_error_ticks"] = position_errors[servo_id]
            row[f"servo_{servo_id}_moving"] = int(servo.moving)

        fieldnames = list(row.keys())
        file_exists = os.path.exists(self.log_file_path)
        if file_exists:
            try:
                with open(self.log_file_path, newline="", encoding="utf-8") as existing_file:
                    existing_header = next(csv.reader(existing_file), [])
            except OSError as exc:
                self.set_status(f"Could not inspect CSV: {exc}", error=True)
                return
            if existing_header != fieldnames:
                # Never append rows under a different header: that would shift
                # fields and silently corrupt a capture set.
                self.start_new_calibration_csv()
                file_exists = False

        try:
            with open(self.log_file_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                if not file_exists:
                    writer.writeheader()
                writer.writerow(row)
            self.logged_points += 1
            self.log_status_var.set(
                f"Log: {os.path.basename(self.log_file_path)} ({self.logged_points} pts recorded)"
            )
            self.set_status(f"Saved calibration data point #{self.logged_points}")
        except Exception as exc:
            self.set_status(f"Logging failed: {exc}", error=True)

    def _open_log_folder(self) -> None:
        folder = os.path.dirname(self.log_file_path)
        if sys.platform == "darwin":
            os.system(f'open "{folder}"')
        elif sys.platform.startswith("linux"):
            os.system(f'xdg-open "{folder}"')
        elif sys.platform == "win32":
            os.system(f'explorer "{folder}"')

    def close(self) -> None:
        self.abort_calibration_sequence()
        self.disconnect()
        self.ble_worker.stop()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    root.geometry("1180x680")
    _app = VringCytonApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
