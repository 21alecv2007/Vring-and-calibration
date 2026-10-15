"""Live VRing IMU calibration tester.

Controls:
  Click a discovered ring to connect.
  Space: start/stop a 60-second stationary drift check.
  R: reset the inertial position model at the current orientation.
  V: force a zero-velocity update (ZUPT).

Dependencies: python3 -m pip install pygame bleak
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import pygame
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice


TARGET_NAME = "VRING"
SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
SENSOR_UUID = "19b10002-e8f2-537e-4f6c-d104768a121a"
BUTTON_UUID = "19b10006-e8f2-537e-4f6c-d104768a1221"
SCREEN_W, SCREEN_H = 1180, 760
RING_SAMPLE_HZ = 400.0
STATIONARY_GYRO_DPS = 1.5
STATIONARY_ACCEL_TOLERANCE_G = 0.08
DRIFT_TEST_SECONDS = 60.0
POSITION_PIXELS_PER_METER = 180.0


@dataclass
class Sensor:
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    gx_dps: float = 0.0
    gy_dps: float = 0.0
    gz_dps: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    counter: int = 0
    connected: bool = False
    status: str = "Scanning for VRING..."
    updated_at: float = 0.0


class SharedRing:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.sensor = Sensor()
        self.devices: dict[str, BLEDevice] = {}
        self.rssi: dict[str, int] = {}
        self.selected: str | None = None

    def update_sensor(self, payload: bytes) -> None:
        if len(payload) < 28:
            return
        scale_accel = 32767.0 / 16.0
        scale_gyro = 32767.0 / 2000.0
        with self.lock:
            self.sensor.ax_g = int.from_bytes(payload[0:2], "little", signed=True) / scale_accel
            self.sensor.ay_g = int.from_bytes(payload[2:4], "little", signed=True) / scale_accel
            self.sensor.az_g = int.from_bytes(payload[4:6], "little", signed=True) / scale_accel
            self.sensor.qx = int.from_bytes(payload[6:8], "little", signed=True) / 32767.0
            self.sensor.qy = int.from_bytes(payload[8:10], "little", signed=True) / 32767.0
            self.sensor.qz = int.from_bytes(payload[10:12], "little", signed=True) / 32767.0
            self.sensor.counter = int.from_bytes(payload[20:22], "little", signed=False)
            self.sensor.gx_dps = int.from_bytes(payload[22:24], "little", signed=True) / scale_gyro
            self.sensor.gy_dps = int.from_bytes(payload[24:26], "little", signed=True) / scale_gyro
            self.sensor.gz_dps = int.from_bytes(payload[26:28], "little", signed=True) / scale_gyro
            self.sensor.connected = True
            self.sensor.updated_at = time.monotonic()

    def snapshot(self) -> Sensor:
        with self.lock:
            return Sensor(**self.sensor.__dict__)

    def set_status(self, text: str, connected: bool | None = None) -> None:
        with self.lock:
            self.sensor.status = text
            if connected is not None:
                self.sensor.connected = connected

    def add_device(self, device: BLEDevice, rssi: int) -> None:
        with self.lock:
            self.devices[device.address] = device
            self.rssi[device.address] = rssi

    def device_rows(self) -> list[tuple[str, str, int]]:
        with self.lock:
            return sorted(
                ((address, device.name or TARGET_NAME, self.rssi.get(address, -100)) for address, device in self.devices.items()),
                key=lambda row: row[2], reverse=True,
            )


async def ble_worker(shared: SharedRing, alive: threading.Event) -> None:
    def detected(device: BLEDevice, advertisement: Any) -> None:
        uuids = [value.lower() for value in (advertisement.service_uuids or [])]
        if (device.name or "").startswith(TARGET_NAME) or SERVICE_UUID in uuids:
            shared.add_device(device, advertisement.rssi)

    scanner = BleakScanner(detection_callback=detected)
    while alive.is_set():
        try:
            shared.set_status("Scanning for VRING...", connected=False)
            await scanner.start()
            while alive.is_set() and shared.selected is None:
                await asyncio.sleep(0.15)
            await scanner.stop()
            if not alive.is_set():
                return
            with shared.lock:
                address = shared.selected
                device = shared.devices.get(address or "")
            if device is None:
                with shared.lock:
                    shared.selected = None
                continue
            shared.set_status(f"Connecting to {device.name or TARGET_NAME}...")
            async with BleakClient(device, timeout=15.0) as client:
                def on_sensor(_handle: int, data: bytearray) -> None:
                    shared.update_sensor(bytes(data))
                await client.start_notify(SENSOR_UUID, on_sensor)
                try:
                    await client.start_notify(BUTTON_UUID, lambda _handle, _data: None)
                except Exception:
                    pass
                shared.set_status(f"Connected to {device.name or TARGET_NAME}", connected=True)
                while alive.is_set() and client.is_connected:
                    await asyncio.sleep(0.1)
            shared.set_status("Disconnected. Scanning again.", connected=False)
            with shared.lock:
                shared.selected = None
        except Exception as exc:
            shared.set_status(f"BLE error: {exc}", connected=False)
            with shared.lock:
                shared.selected = None
            await asyncio.sleep(1.0)


def quaternion(sensor: Sensor) -> tuple[float, float, float, float] | None:
    vector_sq = sensor.qx * sensor.qx + sensor.qy * sensor.qy + sensor.qz * sensor.qz
    if vector_sq > 1.002:
        return None
    return (math.sqrt(max(0.0, 1.0 - vector_sq)), sensor.qx, sensor.qy, sensor.qz)


def euler_deg(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def rotate_body_to_world(q: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate a body vector by the reported quaternion using q*v*q-conjugate."""
    w, x, y, z = q
    vx, vy, vz = vector
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("VRing Calibration Tester")
    font = pygame.font.SysFont(None, 25)
    large = pygame.font.SysFont(None, 38)
    clock = pygame.time.Clock()
    shared, alive = SharedRing(), threading.Event()
    alive.set()
    threading.Thread(target=lambda: asyncio.run(ble_worker(shared, alive)), daemon=True).start()

    last_counter: int | None = None
    velocity = [0.0, 0.0, 0.0]
    position = [0.0, 0.0, 0.0]
    gyro_angle = [0.0, 0.0, 0.0]
    drift_started: float | None = None
    drift_start_euler: tuple[float, float, float] | None = None
    picker: list[tuple[pygame.Rect, str]] = []

    def reset_model(sensor: Sensor) -> None:
        nonlocal last_counter
        velocity[:] = [0.0, 0.0, 0.0]
        position[:] = [0.0, 0.0, 0.0]
        gyro_angle[:] = [0.0, 0.0, 0.0]
        last_counter = sensor.counter

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                alive.clear()
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                sensor = shared.snapshot()
                if event.key == pygame.K_r:
                    reset_model(sensor)
                elif event.key == pygame.K_v:
                    velocity[:] = [0.0, 0.0, 0.0]
                elif event.key == pygame.K_SPACE and sensor.connected:
                    if drift_started is None:
                        reset_model(sensor)
                        drift_started = time.monotonic()
                        q = quaternion(sensor)
                        drift_start_euler = euler_deg(q) if q else None
                    else:
                        drift_started = None
                        drift_start_euler = None
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for rect, address in picker:
                    if rect.collidepoint(event.pos):
                        with shared.lock:
                            shared.selected = address
                        break

        sensor = shared.snapshot()
        q = quaternion(sensor)
        accel_mag = math.sqrt(sensor.ax_g**2 + sensor.ay_g**2 + sensor.az_g**2)
        gyro_mag = math.sqrt(sensor.gx_dps**2 + sensor.gy_dps**2 + sensor.gz_dps**2)
        stationary = gyro_mag < STATIONARY_GYRO_DPS and abs(accel_mag - 1.0) < STATIONARY_ACCEL_TOLERANCE_G

        if sensor.connected and last_counter is not None:
            ticks = (sensor.counter - last_counter) & 0xFFFF
            if 0 < ticks <= 80 and q is not None:
                dt = ticks / RING_SAMPLE_HZ
                for axis, rate in enumerate((sensor.gx_dps, sensor.gy_dps, sensor.gz_dps)):
                    gyro_angle[axis] += rate * dt
                # Rotate measured acceleration to world coordinates and remove
                # one gravity vector. This is a diagnostic model, not reliable
                # free-space tracking: tiny bias becomes quadratic position drift.
                world_accel = list(rotate_body_to_world(q, (sensor.ax_g, sensor.ay_g, sensor.az_g)))
                world_accel[2] -= 1.0
                if stationary:
                    velocity[:] = [0.0, 0.0, 0.0]  # zero-velocity update
                else:
                    for axis in range(3):
                        velocity[axis] += world_accel[axis] * 9.80665 * dt
                        position[axis] += velocity[axis] * dt
            last_counter = sensor.counter
        elif sensor.connected:
            last_counter = sensor.counter

        if drift_started is not None and time.monotonic() - drift_started >= DRIFT_TEST_SECONDS:
            drift_started = None

        screen.fill((18, 22, 32))
        def text(value: str, x: int, y: int, color: tuple[int, int, int] = (232, 236, 244), big: bool = False) -> None:
            screen.blit((large if big else font).render(value, True, color), (x, y))

        text("VRing Calibration Tester", 24, 18, big=True)
        text(sensor.status, 25, 64, (145, 205, 255) if sensor.connected else (255, 186, 104))
        text(f"Accel:  X {sensor.ax_g:+.4f}g   Y {sensor.ay_g:+.4f}g   Z {sensor.az_g:+.4f}g   |a| {accel_mag:.4f}g", 25, 108)
        text(f"Gyro:   X {sensor.gx_dps:+.3f}°/s   Y {sensor.gy_dps:+.3f}°/s   Z {sensor.gz_dps:+.3f}°/s   |ω| {gyro_mag:.3f}°/s", 25, 138)
        text(f"Stationary test: {'YES (ZUPT active)' if stationary else 'NO'}", 25, 168, (120, 220, 145) if stationary else (255, 190, 110))

        if q:
            roll, pitch, yaw = euler_deg(q)
            text(f"Fused orientation: Roll {roll:+.1f}°   Pitch {pitch:+.1f}°   Yaw {yaw:+.1f}°", 25, 208)
        else:
            text("Fused orientation: invalid quaternion", 25, 208, (255, 120, 120))

        text("Gyro-only integrated change since reset", 25, 258, (180, 205, 250))
        text(f"X {gyro_angle[0]:+.2f}°   Y {gyro_angle[1]:+.2f}°   Z {gyro_angle[2]:+.2f}°", 25, 286)
        if drift_started is not None:
            elapsed = time.monotonic() - drift_started
            text(f"DRIFT TEST RUNNING: {elapsed:.1f}/{DRIFT_TEST_SECONDS:.0f}s — keep ring still", 25, 320, (255, 224, 120))
            if drift_start_euler and q:
                now_euler = euler_deg(q)
                delta = [now_euler[i] - drift_start_euler[i] for i in range(3)]
                text(f"Fused attitude change: R {delta[0]:+.2f}°  P {delta[1]:+.2f}°  Y {delta[2]:+.2f}°", 25, 348)
        else:
            text("Space: start 60 s drift test   R: reset model   V: zero velocity", 25, 320, (185, 190, 205))

        panel = pygame.Rect(620, 90, 520, 500)
        pygame.draw.rect(screen, (28, 35, 50), panel, border_radius=12)
        pygame.draw.rect(screen, (92, 132, 190), panel, 2, border_radius=12)
        text("Inertial position model", panel.x + 18, panel.y + 16, (190, 215, 255))
        text("Diagnostic only — translation will drift without external tracking.", panel.x + 18, panel.y + 48, (255, 190, 110))
        text(f"Position: X {position[0]:+.3f} m  Y {position[1]:+.3f} m  Z {position[2]:+.3f} m", panel.x + 18, panel.y + 82)
        text(f"Velocity: X {velocity[0]:+.3f}  Y {velocity[1]:+.3f}  Z {velocity[2]:+.3f} m/s", panel.x + 18, panel.y + 110)
        center = (panel.centerx, panel.y + 330)
        pygame.draw.line(screen, (80, 92, 115), (panel.x + 20, center[1]), (panel.right - 20, center[1]), 1)
        pygame.draw.line(screen, (80, 92, 115), (center[0], panel.y + 160), (center[0], panel.bottom - 28), 1)
        dot_x = max(panel.x + 30, min(panel.right - 30, center[0] + position[0] * POSITION_PIXELS_PER_METER))
        dot_y = max(panel.y + 170, min(panel.bottom - 38, center[1] - position[1] * POSITION_PIXELS_PER_METER))
        pygame.draw.circle(screen, (105, 220, 190), (int(dot_x), int(dot_y)), 10)
        text("X/Y modeled position", panel.x + 18, panel.bottom - 24, (175, 185, 205))

        picker = []
        if not sensor.connected:
            rows = shared.device_rows()
            text("Click a discovered VRING to connect:", 25, 420, (210, 220, 240))
            for index, (address, name, rssi) in enumerate(rows[:6]):
                rect = pygame.Rect(25, 452 + 38 * index, 540, 32)
                pygame.draw.rect(screen, (45, 60, 86), rect, border_radius=6)
                text(f"{name}  {address}  {rssi} dBm", rect.x + 8, rect.y + 6)
                picker.append((rect, address))
        pygame.display.flip()


if __name__ == "__main__":
    main()
