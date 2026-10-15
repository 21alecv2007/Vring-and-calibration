"""VRing pygame prototype - drift-free fused-orientation tilt pointer.

Dependencies:
  pip install pygame bleak
"""

from __future__ import annotations

import asyncio
import functools
import math
import random
import threading
import time
import traceback
from dataclasses import dataclass

import pygame
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice


TARGET_NAME = "VRING"
# The ring used for this sorter. It is selected automatically when discovered;
# the on-screen picker remains available as a fallback.
PREFERRED_ADDRESS_SUFFIX = "2F"
VRING_SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
SENSOR_DATA_CHAR_UUID = "19b10002-e8f2-537e-4f6c-d104768a121a"
BUTTON_INPUT_CHAR_UUID = "19b10006-e8f2-537e-4f6c-d104768a1221"

CAP_ENGAGE_THRESHOLD = 18
CAP_RELEASE_THRESHOLD = 13
CAP_EMA_ALPHA = 0.35
CAP_RELEASE_DEBOUNCE_SECONDS = 0.08
CALIBRATION_CAP_GRACE_SECONDS = 0.4
SCREEN_W = 1280
SCREEN_H = 720
GROUND_Y = SCREEN_H - 24
CURSOR_GAIN_PIXELS_PER_RADIAN = 700.0
CURSOR_SMOOTHING = 0.22
POINTER_DEADZONE_RADIANS = math.radians(1.5)
GRAVITY = 1800.0
AIR_DRAG = 0.999
BLOCK_SIZE = 44
SPAWN_INTERVAL = 1.2
MAX_BLOCKS = 12
CALIBRATION_SECONDS = 2.0
CALIBRATION_STABILITY_DELTA = 3.0  # Max roll + pitch deviation (degrees) during the baseline hold
BLE_CONNECT_TIMEOUT_SECONDS = 20.0

COLORS = [
    (235, 87, 87),
    (66, 153, 225),
    (72, 187, 120),
    (246, 173, 85),
]


@dataclass
class SensorState:
    accel_x: int = 0
    accel_y: int = 0
    accel_z: int = 0
    quat_x: int = 0
    quat_y: int = 0
    quat_z: int = 0
    cap1: int = 0
    cap2: int = 0
    button_pressed: bool = False
    sample_counter: int = 0
    battery_mv: int = 0
    temp_f: float = 0.0
    connected: bool = False
    last_update: float = 0.0
    connection_status: str = "Scanning for VRING..."
    connection_rssi: int = 0


@dataclass
class RingDevice:
    address: str
    name: str
    rssi: int


@dataclass
class BinZone:
    color: tuple[int, int, int]
    rect: pygame.Rect
    inner: pygame.Rect
    name: str


class TextCache:
    """LRU cache for rendered fonts to prevent massive per-frame object allocation."""
    def __init__(self, font: pygame.font.Font) -> None:
        self.font = font
        self.render = functools.lru_cache(maxsize=256)(self._render)

    def _render(self, text: str, color: tuple[int, int, int]) -> pygame.Surface:
        return self.font.render(text, True, color)


class Block:
    def __init__(self, x: float, y: float, color: tuple[int, int, int]):
        self.x = x
        self.y = y
        self.vx = random.uniform(-80, 80)
        self.vy = random.uniform(-20, 20)
        self.color = color
        self.size = BLOCK_SIZE
        self.grabbed = False
        self.spawn_time = time.time()

    @property
    def rect(self) -> pygame.Rect:
        return pygame.Rect(int(self.x), int(self.y), self.size, self.size)

    def center_on(self, x: float, y: float) -> None:
        self.x = x - self.size / 2
        self.y = y - self.size / 2


def quaternion_tilt(state: SensorState) -> tuple[float, float] | None:
    """Return roll and pitch in radians from the ring's compact quaternion."""
    x = state.quat_x / 32767.0
    y = state.quat_y / 32767.0
    z = state.quat_z / 32767.0
    vector_sq = x * x + y * y + z * z
    if vector_sq > 1.001:
        return None
    w = math.sqrt(max(0.0, 1.0 - vector_sq))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def wrapped_angle_delta(angle: float, reference: float) -> float:
    """Shortest signed angle difference, avoiding a wraparound cursor jump."""
    return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi


class SharedInput:
    def __init__(self) -> None:
        self.state = SensorState()
        self.lock = threading.Lock()
        self.rings: list[RingDevice] = []
        self.ble_devices: dict[str, BLEDevice] = {}
        self.selected_address: str | None = None

    def update_sensor(self, payload: bytes) -> None:
        if len(payload) < 22:
            return
        with self.lock:
            self.state.accel_x = int.from_bytes(payload[0:2], "little", signed=True)
            self.state.accel_y = int.from_bytes(payload[2:4], "little", signed=True)
            self.state.accel_z = int.from_bytes(payload[4:6], "little", signed=True)
            self.state.quat_x = int.from_bytes(payload[6:8], "little", signed=True)
            self.state.quat_y = int.from_bytes(payload[8:10], "little", signed=True)
            self.state.quat_z = int.from_bytes(payload[10:12], "little", signed=True)
            self.state.cap1 = int.from_bytes(payload[12:14], "little", signed=True)
            self.state.cap2 = int.from_bytes(payload[14:16], "little", signed=True)
            self.state.battery_mv = int.from_bytes(payload[16:18], "little", signed=False)
            self.state.temp_f = int.from_bytes(payload[18:20], "little", signed=True) / 100.0
            self.state.sample_counter = int.from_bytes(payload[20:22], "little", signed=False)
            self.state.connected = True
            self.state.last_update = time.time()

    def update_button(self, payload: bytes) -> None:
        pressed = bool(payload[0]) if payload else False
        with self.lock:
            self.state.button_pressed = pressed
            self.state.connected = True
            self.state.last_update = time.time()

    def update_status(self, status: str, rssi: int | None = None) -> None:
        with self.lock:
            self.state.connection_status = status
            if rssi is not None:
                self.state.connection_rssi = rssi

    def set_disconnected(self, status: str) -> None:
        with self.lock:
            self.state.connected = False
            self.state.connection_status = status

    def add_discovered_device(self, device: BLEDevice, rssi: int) -> None:
        name = device.name or TARGET_NAME
        with self.lock:
            self.ble_devices[device.address] = device
            existing = next((r for r in self.rings if r.address == device.address), None)
            if existing:
                existing.rssi = rssi
            else:
                self.rings.append(RingDevice(device.address, name, rssi))
            self.rings.sort(key=lambda r: r.rssi, reverse=True)

    def select_ring(self, address: str | None) -> None:
        with self.lock:
            self.selected_address = address
            if address is not None:
                self.state.connection_status = "Connecting to selected VRING..."

    def selected_ring(self) -> str | None:
        with self.lock:
            if self.selected_address is None:
                preferred = next(
                    (ring.address for ring in self.rings
                     if ring.address.upper().endswith(PREFERRED_ADDRESS_SUFFIX)),
                    None,
                )
                if preferred is not None:
                    self.selected_address = preferred
                    self.state.connection_status = (
                        f"Connecting to preferred VRING (*{PREFERRED_ADDRESS_SUFFIX})..."
                    )
            return self.selected_address

    def get_ble_device(self, address: str) -> BLEDevice | None:
        with self.lock:
            return self.ble_devices.get(address)

    def available_rings(self) -> list[RingDevice]:
        with self.lock:
            return list(self.rings)

    def snapshot(self) -> SensorState:
        with self.lock:
            return SensorState(**self.state.__dict__)


def build_bins() -> list[BinZone]:
    bins: list[BinZone] = []
    margin = 20
    gap = 12
    width = 280
    top = SCREEN_H - 170
    height = 140
    labels = ["RED", "BLUE", "GREEN", "YELLOW"]
    for index, color in enumerate(COLORS):
        x = margin + index * (width + gap)
        outer = pygame.Rect(x, top, width, height)
        inner = pygame.Rect(x + 14, top + 12, width - 28, height - 12)
        bins.append(BinZone(color=color, rect=outer, inner=inner, name=labels[index]))
    return bins


def build_walls(bins: list[BinZone]) -> list[pygame.Rect]:
    walls = [pygame.Rect(0, GROUND_Y, SCREEN_W, SCREEN_H - GROUND_Y)]
    for zone in bins:
        walls.append(pygame.Rect(zone.rect.left, zone.rect.top, 10, zone.rect.height))
        walls.append(pygame.Rect(zone.rect.right - 10, zone.rect.top, 10, zone.rect.height))
        walls.append(pygame.Rect(zone.rect.left, zone.rect.bottom - 10, zone.rect.width, 10))
    return walls


def resolve_block_vs_wall(block: Block, wall: pygame.Rect) -> None:
    rect = block.rect
    if not rect.colliderect(wall):
        return

    overlap_left = rect.right - wall.left
    overlap_right = wall.right - rect.left
    overlap_top = rect.bottom - wall.top
    overlap_bottom = wall.bottom - rect.top
    min_overlap = min(overlap_left, overlap_right, overlap_top, overlap_bottom)

    if min_overlap == overlap_left:
        block.x -= overlap_left
        block.vx = min(0.0, block.vx)
    elif min_overlap == overlap_right:
        block.x += overlap_right
        block.vx = max(0.0, block.vx)
    elif min_overlap == overlap_top:
        block.y -= overlap_top
        block.vy = min(0.0, block.vy)
    else:
        block.y += overlap_bottom
        block.vy = max(0.0, block.vy)


def resolve_block_vs_block(a: Block, b: Block) -> None:
    dx = a.x - b.x
    dy = a.y - b.y
    dist_sq = dx * dx + dy * dy
    if dist_sq > 2.0 * (a.size * a.size):
        return

    rect_a = a.rect
    rect_b = b.rect
    if not rect_a.colliderect(rect_b):
        return

    overlap_x = min(rect_a.right - rect_b.left, rect_b.right - rect_a.left)
    overlap_y = min(rect_a.bottom - rect_b.top, rect_b.bottom - rect_a.top)

    if overlap_x < overlap_y:
        push = overlap_x / 2.0
        if rect_a.centerx < rect_b.centerx:
            a.x -= push
            b.x += push
        else:
            a.x += push
            b.x -= push
        a.vx, b.vx = b.vx * 0.35, a.vx * 0.35
    else:
        push = overlap_y / 2.0
        if rect_a.centery < rect_b.centery:
            a.y -= push
            b.y += push
            a.vy = min(0.0, a.vy)
            b.vy = max(0.0, b.vy)
        else:
            a.y += push
            b.y -= push
            a.vy = max(0.0, a.vy)
            b.vy = min(0.0, b.vy)


async def ble_task(shared: SharedInput, running: threading.Event) -> None:
    def detection_callback(device: BLEDevice, advertisement: getattr(BleakScanner, "AdvertisementData", any)) -> None:
        name = device.name or TARGET_NAME
        advertised_uuids = [uuid.lower() for uuid in advertisement.service_uuids]
        if name.startswith(TARGET_NAME) or VRING_SERVICE_UUID in advertised_uuids:
            shared.add_discovered_device(device, advertisement.rssi)

    scanner = BleakScanner(detection_callback=detection_callback)

    while running.is_set():
        try:
            shared.update_status("Scanning for VRING devices...")
            await scanner.start()

            while running.is_set() and shared.selected_ring() is None:
                await asyncio.sleep(0.2)

            await scanner.stop()

            if not running.is_set():
                break

            selected_address = shared.selected_ring()
            device = shared.get_ble_device(selected_address)

            if device is None:
                shared.update_status("Selected VRING is not currently visible. Scanning again...")
                shared.select_ring(None)
                continue

            with shared.lock:
                rssi = next((r.rssi for r in shared.rings if r.address == selected_address), -100)
            
            shared.update_status(f"Connecting to {device.name or TARGET_NAME} ({rssi} dBm)...", rssi)

            client = BleakClient(device, timeout=BLE_CONNECT_TIMEOUT_SECONDS)
            try:
                await asyncio.wait_for(client.connect(), timeout=BLE_CONNECT_TIMEOUT_SECONDS)
            except asyncio.TimeoutError:
                shared.set_disconnected(
                    f"Connecting timed out after {BLE_CONNECT_TIMEOUT_SECONDS:.0f}s. Retrying..."
                )
                shared.select_ring(None)
                await asyncio.sleep(1.0)
                continue

            try:
                with shared.lock:
                    shared.state.connected = True
                    shared.state.last_update = time.time()
                    shared.state.connection_status = f"Connected to {device.name or TARGET_NAME} ({rssi} dBm)"
                    shared.state.connection_rssi = rssi

                def on_sensor(_: int, data: bytearray) -> None:
                    shared.update_sensor(bytes(data))

                def on_button(_: int, data: bytearray) -> None:
                    shared.update_button(bytes(data))

                await client.start_notify(SENSOR_DATA_CHAR_UUID, on_sensor)
                await client.start_notify(BUTTON_INPUT_CHAR_UUID, on_button)

                try:
                    shared.update_sensor(bytes(await client.read_gatt_char(SENSOR_DATA_CHAR_UUID)))
                except Exception:
                    pass

                while running.is_set() and client.is_connected:
                    await asyncio.sleep(0.1)

                if running.is_set():
                    shared.set_disconnected("Disconnected from VRING. Scanning again...")
                    shared.select_ring(None)
            finally:
                try:
                    if client.is_connected:
                        await client.disconnect()
                except Exception:
                    pass
        except Exception as error:
            traceback.print_exc()
            shared.set_disconnected(f"Bluetooth error: {error}. Scanning again...")
            shared.select_ring(None)
            await asyncio.sleep(1.0)


def start_ble_thread(shared: SharedInput, running: threading.Event) -> threading.Thread:
    def runner() -> None:
        asyncio.run(ble_task(shared, running))

    thread = threading.Thread(target=runner, daemon=True)
    thread.start()
    return thread


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("VRing Block Sorter")
    clock = pygame.time.Clock()
    
    font_renderer = TextCache(pygame.font.SysFont(None, 24))
    big_font_renderer = TextCache(pygame.font.SysFont(None, 36))
    
    mouse_visible = False
    pygame.mouse.set_visible(mouse_visible)

    shared = SharedInput()
    running = threading.Event()
    running.set()
    start_ble_thread(shared, running)

    bins = build_bins()
    walls = build_walls(bins)
    blocks: list[Block] = []
    score = 0
    last_spawn = time.time()
    cursor_x = SCREEN_W / 2
    cursor_y = SCREEN_H / 2
    cursor_vx = 0.0
    cursor_vy = 0.0
    cursor_active = False
    
    # Neutral hand orientation captured from the fused Madgwick quaternion.
    ref_roll = 0.0
    ref_pitch = 0.0
    calibration_tilt_samples: list[tuple[float, float]] = []

    grabbed: Block | None = None
    prev_button = False
    prev_cap = False
    cap_release_started: float | None = None
    calibration_cap_release_started: float | None = None
    cap2_ema: float | None = None
    cap_engaged = False
    calibrating = True
    calibration_started: float | None = None
    last_calibration_sample: int | None = None
    calibration_last_drift = 0.0
    picker_rows: list[tuple[pygame.Rect, str]] = []
    was_connected = False

    def spawn_block() -> None:
        color = random.choice(COLORS)
        x = random.uniform(60, SCREEN_W - 60 - BLOCK_SIZE)
        blocks.append(Block(x, 40, color))

    for _ in range(6):
        spawn_block()

    while True:
        dt = clock.tick(60) / 1000.0
        if dt <= 0:
            dt = 1 / 60

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running.clear()
                pygame.quit()
                return
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for rect, address in picker_rows:
                    if rect.collidepoint(event.pos):
                        shared.select_ring(address)
                        break

        state = shared.snapshot()
        
        should_be_visible = not state.connected
        if mouse_visible != should_be_visible:
            pygame.mouse.set_visible(should_be_visible)
            mouse_visible = should_be_visible

        if not state.connected and was_connected:
            if grabbed is not None:
                grabbed.vx = 0.0
                grabbed.vy = 0.0
                grabbed.grabbed = False
                grabbed = None
            cursor_active = False
            prev_cap = False
            prev_button = False
            cap_release_started = None
            calibration_cap_release_started = None
            cap_engaged = False
            cap2_ema = None
            if calibrating:
                calibration_started = None
                calibration_tilt_samples.clear()
                last_calibration_sample = None
                calibration_last_drift = 0.0
        was_connected = state.connected

        if cap2_ema is None:
            cap2_ema = float(state.cap2)
        else:
            cap2_ema += (state.cap2 - cap2_ema) * CAP_EMA_ALPHA

        if state.connected:
            if cap_engaged:
                if cap2_ema < CAP_RELEASE_THRESHOLD:
                    cap_engaged = False
            else:
                if cap2_ema >= CAP_ENGAGE_THRESHOLD:
                    cap_engaged = True
        else:
            cap_engaged = False

        raw_cap_active = cap_engaged

        if raw_cap_active:
            cap_release_started = None
            cap_active = True
            calibration_cap_release_started = None
            calibration_cap_active = True
        else:
            if cap_release_started is None:
                cap_release_started = time.time()
            cap_active = (time.time() - cap_release_started) < CAP_RELEASE_DEBOUNCE_SECONDS

            if calibration_cap_release_started is None:
                calibration_cap_release_started = time.time()
            calibration_cap_active = (
                time.time() - calibration_cap_release_started
            ) < CALIBRATION_CAP_GRACE_SECONDS

        if calibrating:
            tilt = quaternion_tilt(state)
            if state.connected and tilt is not None and calibration_cap_active and state.sample_counter != last_calibration_sample:
                last_calibration_sample = state.sample_counter
                if calibration_started is None:
                    calibration_started = time.time()
                    calibration_tilt_samples = [tilt]
                    calibration_last_drift = 0.0
                else:
                    calibration_tilt_samples.append(tilt)
                    avg_roll = sum(s[0] for s in calibration_tilt_samples) / len(calibration_tilt_samples)
                    avg_pitch = sum(s[1] for s in calibration_tilt_samples) / len(calibration_tilt_samples)
                    calibration_last_drift = math.degrees(
                        abs(wrapped_angle_delta(tilt[0], avg_roll))
                        + abs(wrapped_angle_delta(tilt[1], avg_pitch))
                    )

                    # Do not accept a moving hand/ring as the cursor's neutral
                    # tilt. Start the two-second hold over after a 3° change.
                    if calibration_last_drift > CALIBRATION_STABILITY_DELTA:
                        calibration_started = time.time()
                        calibration_tilt_samples = [tilt]
                        calibration_last_drift = 0.0

                if time.time() - calibration_started >= CALIBRATION_SECONDS:
                    ref_roll = sum(s[0] for s in calibration_tilt_samples) / len(calibration_tilt_samples)
                    ref_pitch = sum(s[1] for s in calibration_tilt_samples) / len(calibration_tilt_samples)
                    calibrating = False
                    cursor_active = cap_active
                    prev_cap = cap_active
                    prev_button = state.button_pressed
                    if cursor_active:
                        cursor_x = SCREEN_W / 2
                        cursor_y = SCREEN_H / 2
            else:
                if not calibration_cap_active:
                    calibration_started = None
                    calibration_tilt_samples.clear()
                    last_calibration_sample = None

        if not calibrating and cap_active and not prev_cap:
            tilt = quaternion_tilt(state)
            if tilt is not None:
                cursor_active = True
                cursor_x = SCREEN_W / 2
                cursor_y = SCREEN_H / 2
                ref_roll, ref_pitch = tilt
        elif not calibrating and not cap_active and prev_cap:
            cursor_active = False
            if grabbed is not None:
                grabbed.vx = cursor_vx * 1.2
                grabbed.vy = cursor_vy * 1.2
                grabbed.grabbed = False
                grabbed = None

        if not calibrating:
            prev_cap = cap_active

        if cursor_active and not calibrating:
            # Quaternion roll/pitch is gyro-smoothed and gravity-corrected. The
            # cursor is always relative to the neutral orientation at cap engage,
            # so it does not integrate velocity or accumulate position drift.
            tilt = quaternion_tilt(state)
            if tilt is None:
                cursor_active = False
            else:
                roll, pitch = tilt
                dx = wrapped_angle_delta(pitch, ref_pitch)
                dy = wrapped_angle_delta(roll, ref_roll)

                if abs(dx) < POINTER_DEADZONE_RADIANS:
                    dx = 0.0
                if abs(dy) < POINTER_DEADZONE_RADIANS:
                    dy = 0.0

                target_x = SCREEN_W / 2 + dx * CURSOR_GAIN_PIXELS_PER_RADIAN
                target_y = SCREEN_H / 2 + dy * CURSOR_GAIN_PIXELS_PER_RADIAN
                
                new_x = cursor_x + (target_x - cursor_x) * CURSOR_SMOOTHING
                new_y = cursor_y + (target_y - cursor_y) * CURSOR_SMOOTHING
                cursor_vx = (new_x - cursor_x) / dt
                cursor_vy = (new_y - cursor_y) / dt
                cursor_x = max(24, min(SCREEN_W - 24, new_x))
                cursor_y = max(24, min(SCREEN_H - 24, new_y))
        else:
            cursor_vx = 0.0
            cursor_vy = 0.0

        button_pressed = state.button_pressed
        if button_pressed and not prev_button and cursor_active and not calibrating:
            hit = None
            for block in reversed(blocks):
                if block.rect.collidepoint(int(cursor_x), int(cursor_y)):
                    hit = block
                    break

            if grabbed is None and hit is not None:
                grabbed = hit
                grabbed.grabbed = True
                grabbed.center_on(cursor_x, cursor_y)
                grabbed.vx = 0.0
                grabbed.vy = 0.0
            elif grabbed is not None:
                grabbed.vx = cursor_vx * 1.2
                grabbed.vy = cursor_vy * 1.2
                grabbed.grabbed = False
                grabbed = None

        if not calibrating:
            prev_button = button_pressed

        if time.time() - last_spawn > SPAWN_INTERVAL and len(blocks) < MAX_BLOCKS:
            spawn_block()
            last_spawn = time.time()

        for block in blocks:
            if block.grabbed:
                block.center_on(cursor_x, cursor_y)
                block.vx = 0.0
                block.vy = 0.0
                continue

            block.vy += GRAVITY * dt
            block.x += block.vx * dt
            block.y += block.vy * dt
            block.vx *= AIR_DRAG
            block.vy *= AIR_DRAG

            for wall in walls:
                resolve_block_vs_wall(block, wall)

        for i, a in enumerate(blocks):
            if a.grabbed:
                continue
            for b in blocks[i + 1 :]:
                if b.grabbed:
                    continue
                resolve_block_vs_block(a, b)

        remaining: list[Block] = []
        for block in blocks:
            scored = False
            if not block.grabbed:
                for zone in bins:
                    if block.rect.colliderect(zone.inner) and block.color == zone.color:
                        if abs(block.vx) < 40 and abs(block.vy) < 60:
                            score += 1
                            scored = True
                            break
            if not scored:
                remaining.append(block)
        blocks = remaining

        screen.fill((18, 20, 28))

        for zone in bins:
            pygame.draw.rect(screen, zone.color, zone.rect, 4, border_radius=10)
            label = big_font_renderer.render(zone.name, zone.color)
            screen.blit(label, (zone.rect.x + 14, zone.rect.y - 30))

        pygame.draw.rect(screen, (70, 78, 96), pygame.Rect(0, GROUND_Y, SCREEN_W, SCREEN_H - GROUND_Y))

        for block in blocks:
            pygame.draw.rect(screen, block.color, block.rect, border_radius=6)
            pygame.draw.rect(screen, (20, 20, 20), block.rect, 2, border_radius=6)

        if cursor_active:
            pygame.draw.circle(screen, (245, 245, 245), (int(cursor_x), int(cursor_y)), 12, 2)
            pygame.draw.line(screen, (245, 245, 245), (int(cursor_x) - 18, int(cursor_y)), (int(cursor_x) + 18, int(cursor_y)), 2)
            pygame.draw.line(screen, (245, 245, 245), (int(cursor_x), int(cursor_y) - 18), (int(cursor_x), int(cursor_y) + 18), 2)

        hud_lines = [
            f"Connected: {'yes' if state.connected else 'no'}",
            f"Cap2: {state.cap2} (raw)  ema {cap2_ema:.1f}  engaged={cap_engaged}",
            f"Button: {'pressed' if state.button_pressed else 'released'}",
            f"Accel: {state.accel_x}, {state.accel_y}, {state.accel_z}",
            f"Quat: {state.quat_x}, {state.quat_y}, {state.quat_z}",
            f"Score: {score}",
            f"Blocks: {len(blocks)}",
        ]
        for index, line in enumerate(hud_lines):
            surf = font_renderer.render(line, (235, 235, 235))
            screen.blit(surf, (18, 16 + index * 22))

        if not state.connected:
            msg = font_renderer.render("Waiting for VRING BLE notifications...", (255, 180, 90))
            screen.blit(msg, (18, SCREEN_H - 42))

        status_line = font_renderer.render(state.connection_status, (200, 220, 255))
        screen.blit(status_line, (18, SCREEN_H - 66))

        if calibrating:
            overlay = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
            overlay.fill((8, 10, 16, 165))
            screen.blit(overlay, (0, 0))
            title = big_font_renderer.render("Calibration", (245, 245, 245))
            screen.blit(title, (SCREEN_W // 2 - title.get_width() // 2, 180))
            lines = [
                "Hold capacitive sensor 2 still to calibrate the accelerometer.",
                f"Cap 2 engage/release: {CAP_ENGAGE_THRESHOLD}/{CAP_RELEASE_THRESHOLD}",
                f"Time: {0.0 if calibration_started is None else min(CALIBRATION_SECONDS, time.time() - calibration_started):.1f}/{CALIBRATION_SECONDS:.1f}s",
                f"Drift variance: {calibration_last_drift:.0f}",
                "Release and re-hold if it drifts.",
            ]
            for index, line in enumerate(lines):
                surf = font_renderer.render(line, (225, 225, 225))
                screen.blit(surf, (SCREEN_W // 2 - surf.get_width() // 2, 240 + index * 28))

        picker_rows = []
        if not state.connected:
            panel = pygame.Rect(SCREEN_W // 2 - 310, 370, 620, 250)
            pygame.draw.rect(screen, (28, 34, 48), panel, border_radius=12)
            pygame.draw.rect(screen, (145, 185, 245), panel, 2, border_radius=12)
            title = font_renderer.render("Available VRING devices: click one to connect", (245, 245, 245))
            screen.blit(title, (panel.x + 18, panel.y + 16))
            rings = shared.available_rings()
            if not rings:
                message = font_renderer.render("Scanning... no VRING advertisements received yet.", (210, 210, 210))
                screen.blit(message, (panel.x + 18, panel.y + 60))
            for index, ring in enumerate(rings[:5]):
                row = pygame.Rect(panel.x + 14, panel.y + 52 + index * 36, panel.width - 28, 30)
                pygame.draw.rect(screen, (48, 60, 82), row, border_radius=6)
                text = font_renderer.render(f"{ring.name}  |  {ring.address}  |  {ring.rssi} dBm", (235, 240, 250))
                screen.blit(text, (row.x + 10, row.y + 6))
                picker_rows.append((row, ring.address))

        pygame.display.flip()


if __name__ == "__main__":
    main()
