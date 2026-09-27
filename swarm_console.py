#!/usr/bin/env python3
"""Lightweight Gazebo Transport RGB-D viewer for the terminal swarm console.

The terminal dashboard lives in swarm_mission.py; this module displays only
camera tiles belonging to drones currently assigned SURVEY. ROS 2 is not used.
"""

import importlib
import math
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


STALE_SECONDS = 3.0
MAX_DEPTH_METRES = 30.0
CAMERA_FPS = max(1, min(10, int(os.getenv("SWARM_CAMERA_FPS", "3"))))


def drone_for_topic(topic):
    """Only an explicit model/drone ID can associate a feed with a vehicle."""
    for pattern in (r"(?:^|/)drone[_-]?(\d+)(?:/|$)",
                    r"(?:^|/)model/(?:gz_)?x500(?:_depth)?_(\d+)(?:/|$)",
                    r"(?:^|/)(?:uav|vehicle)[_-]?(\d+)(?:/|$)"):
        match = re.search(pattern, topic.lower())
        if match:
            return int(match.group(1))
    return None


def image_array(message, kind):
    """Decode Gazebo Image bytes with row stride; depth stays in metres."""
    h, w, step = message.height, message.width, message.step
    if h < 1 or w < 1 or step < 1 or h * step > len(message.data):
        return None
    enum = message.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type
    value = enum.values_by_number.get(message.pixel_format_type)
    fmt = value.name if value else ''
    if kind == 'depth':
        if fmt == 'R_FLOAT32' and step >= w * 4 and step % 4 == 0:
            return np.frombuffer(message.data, np.float32, h * (step // 4))\
                .reshape(h, step // 4)[:, :w].copy()
        if fmt in ('L_INT16', 'R_UINT16') and step >= w * 2 and step % 2 == 0:
            return np.frombuffer(message.data, np.uint16, h * (step // 2))\
                .reshape(h, step // 2)[:, :w].copy() * 0.001
        return None
    channels = {'RGB_INT8': 3, 'BGR_INT8': 3, 'RGBA_INT8': 4,
                'BGRA_INT8': 4}.get(fmt)
    if channels is None or step < w * channels:
        return None
    rgb = np.frombuffer(message.data, np.uint8, h * step).reshape(h, step)
    rgb = np.ascontiguousarray(rgb[:, :w * channels].reshape(h, w, channels))
    if cv2 is None:
        return rgb
    if fmt == 'RGB_INT8':
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if fmt == 'RGBA_INT8':
        return cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
    if fmt == 'BGRA_INT8':
        return cv2.cvtColor(rgb, cv2.COLOR_BGRA2BGR)
    return rgb


class GazeboCameraViewer:
    def __init__(self, num_drones):
        self.num_drones = num_drones
        self.active_ids = set()
        self.frames = {}
        self.discovered_topics = {}
        self.ambiguous_depth = False
        self.error = ''
        self.camera_warning = ''
        self.clock_error = ''
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.node = None
        self.image_type = None
        self.clock_type = None
        self.subscribed = {}
        self.sim_seconds = None
        self.sim_updated_at = 0.0
        self.clock_source = 'WALL'
        self.window_visible = False
        self.window_closed = False
        self.display_available = bool(os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY'))

    def start(self):
        if os.getenv('CAMERA_VIEWER', '1') == '0':
            self.error = 'disabled by CAMERA_VIEWER=0 (clock still monitored)'
        elif cv2 is None:
            self.error = 'install opencv-python in mavsdk_venv to see RGB-D window'
        elif not self.display_available:
            self.error = 'no desktop display: terminal telemetry remains available'
        try:
            # Apt Gazebo Python bindings are commonly hidden by a venv.
            system_packages = '/usr/lib/python3/dist-packages'
            if system_packages not in sys.path:
                sys.path.append(system_packages)
            transport = importlib.import_module('gz.transport13')
            messages = importlib.import_module('gz.msgs10.image_pb2')
            clocks = importlib.import_module('gz.msgs10.clock_pb2')
            self.node = transport.Node()
            self.image_type = messages.Image
            self.clock_type = clocks.Clock
            self.subscribe_options = transport.SubscribeOptions
        except (ImportError, OSError, AttributeError) as exc:
            self.error = ('Gazebo Python transport missing: install python3-gz-transport13 '
                          f'and python3-gz-msgs10 ({exc})')
            self.clock_error = 'Gazebo clock unavailable; using wall time'
            return
        self.thread = threading.Thread(target=self._poll_topics, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        if cv2 is not None and self.window_visible:
            try:
                cv2.destroyWindow('UAV-X | SURVEY RGB-D')
                cv2.waitKey(1)
            except Exception:
                pass
        self.node = None

    def update_active(self, drone_ids):
        with self.lock:
            self.active_ids = set(drone_ids)
            for key in list(self.frames):
                if key[0] not in self.active_ids:
                    del self.frames[key]

    def _topic_list(self):
        # Gazebo Python bindings vary by release; the CLI lists exactly what
        # the running Gazebo server publishes, without a ROS bridge.
        result = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                                text=True, timeout=4, check=False)
        if result.returncode:
            raise RuntimeError((result.stderr or 'gz topic -l failed').strip())
        return result.stdout.splitlines()

    def _poll_topics(self):
        while not self.stop_event.is_set():
            try:
                topics = self._topic_list()
                with self.lock:
                    active = set(self.active_ids)
                if not any('clock' in t for t in topics):
                    self.clock_error = 'Gazebo /clock not published; timing falls back to wall'
                for topic in topics:
                    if topic in self.subscribed:
                        continue
                    low = topic.lower()
                    if low == '/clock' or re.fullmatch(r'/world/[^/]+/clock', low):
                        if self._subscribe(self.clock_type, topic, self._on_clock):
                            self.subscribed[topic] = ('clock', None)
                        continue
                    if not any(s in low for s in ('image', 'depth_camera')):
                        continue
                    if 'camera_info' in low or 'points' in low:
                        continue
                    drone_id = drone_for_topic(topic)
                    sensor_path = re.sub(r'/model/[^/]+', '', low)
                    kind = 'depth' if 'depth' in sensor_path else 'rgb'
                    if drone_id is None:
                        if kind == 'depth':
                            self.ambiguous_depth = True
                        continue
                    if drone_id not in active or drone_id >= self.num_drones:
                        continue
                    callback = lambda message, d=drone_id, k=kind, t=topic: self._on_image(message, d, k, t)
                    if self._subscribe(self.image_type, topic, callback):
                        self.subscribed[topic] = (kind, drone_id)
                        self.discovered_topics[topic] = drone_id
                # Unsubscribe on role change, avoiding image traffic from idle relays.
                for topic, (kind, drone_id) in list(self.subscribed.items()):
                    if kind == 'clock' or drone_id in active:
                        continue
                    if hasattr(self.node, 'unsubscribe'):
                        self.node.unsubscribe(topic)
                        del self.subscribed[topic]
                        self.discovered_topics.pop(topic, None)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.clock_error = f'Gazebo discovery: {exc}'
            except Exception as exc:
                self.camera_warning = f'Gazebo camera discovery error: {exc}'
            self.stop_event.wait(3)

    def _subscribe(self, msg_type, topic, callback):
        options = self.subscribe_options()
        options.msgs_per_sec = CAMERA_FPS if msg_type == self.image_type else 10
        return self.node.subscribe(msg_type, topic, callback, options)

    def _on_clock(self, message):
        stamp = message.sim
        with self.lock:
            self.sim_seconds = stamp.sec + stamp.nsec / 1e9
            self.sim_updated_at = time.monotonic()
        self.clock_error = ''

    def sim_time(self):
        with self.lock:
            return self.sim_seconds

    def choose_clock(self):
        with self.lock:
            ready = self.sim_seconds is not None and time.monotonic() - self.sim_updated_at < 5
        self.clock_source = 'SIM' if ready else 'WALL'
        return self.sim_time if ready else time.monotonic

    def _on_image(self, message, drone_id, kind, topic):
        with self.lock:
            if drone_id not in self.active_ids:
                return
        try:
            frame = image_array(message, kind)
            if frame is None:
                self.camera_warning = f'unsupported {kind} pixel format on {topic}'
                return
            with self.lock:
                if drone_id in self.active_ids:
                    self.frames[(drone_id, kind)] = (frame, time.monotonic(), topic)
        except Exception as exc:
            self.camera_warning = f'camera decoding error on {topic}: {exc}'

    def status(self, drone_id):
        now = time.monotonic()
        with self.lock:
            rgb = self.frames.get((drone_id, 'rgb'))
            depth = self.frames.get((drone_id, 'depth'))
        rgb_ok = rgb is not None and now - rgb[1] < STALE_SECONDS
        dep_ok = depth is not None and now - depth[1] < STALE_SECONDS
        if rgb_ok and dep_ok:
            return 'RGB+DEP'
        if rgb_ok:
            return 'RGB/DEP?'
        if dep_ok:
            return 'DEP/RGB?'
        return 'WAIT'

    def depth_distance(self, drone_id):
        with self.lock:
            item = self.frames.get((drone_id, 'depth'))
        if item is None or time.monotonic() - item[1] >= STALE_SECONDS:
            return None
        values = item[0]
        valid = values[np.isfinite(values) & (values > 0)]
        if not valid.size:
            return None
        k = max(1, int(valid.size * 0.05))
        return float(np.median(np.partition(valid, k - 1)[:k]))

    def render(self):
        """Call from the asyncio main thread; OpenCV UI must stay on that thread."""
        if cv2 is None or self.error or not self.display_available or self.window_closed:
            return
        now = time.monotonic()
        with self.lock:
            active = sorted(self.active_ids)
            frames = {key: value for key, value in self.frames.items() if key[0] in active}
        if not active:
            if self.window_visible:
                cv2.destroyWindow('UAV-X | SURVEY RGB-D')
                self.window_visible = False
                cv2.waitKey(1)
            return
        tiles = []
        for drone_id in active:
            rgb = frames.get((drone_id, 'rgb'))
            depth = frames.get((drone_id, 'depth'))
            rgb_ok = rgb is not None and now - rgb[1] < STALE_SECONDS
            dep_ok = depth is not None and now - depth[1] < STALE_SECONDS
            left = cv2.resize(rgb[0], (280, 180)) if rgb_ok else np.zeros((180, 280, 3), np.uint8)
            if dep_ok:
                z = np.nan_to_num(depth[0], nan=0.0, posinf=0.0, neginf=0.0)
                colored = (255 * (1 - np.clip(z, 0, MAX_DEPTH_METRES) / MAX_DEPTH_METRES)).astype(np.uint8)
                right = cv2.resize(cv2.applyColorMap(colored, cv2.COLORMAP_TURBO),
                                   (280, 180))
            else:
                right = np.zeros((180, 280, 3), np.uint8)
            tile = np.hstack((left, right))
            cv2.rectangle(tile, (0, 0), (560, 27), (0, 0, 0), -1)
            cv2.putText(tile, f'DRONE {drone_id:02d} SURVEY | RGB {"LIVE" if rgb_ok else "WAIT"} '
                        f'| DEPTH {"LIVE" if dep_ok else "WAIT"}', (5, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, .49, (255, 255, 255), 1)
            tiles.append(tile)
        columns = min(2, len(tiles))
        rows = math.ceil(len(tiles) / columns)
        display = np.zeros((rows * 180, columns * 560, 3), np.uint8)
        for idx, tile in enumerate(tiles):
            row, col = divmod(idx, columns)
            display[row * 180:(row + 1) * 180, col * 560:(col + 1) * 560] = tile
        cv2.imshow('UAV-X | SURVEY RGB-D', display)
        self.window_visible = True
        if cv2.waitKey(1) & 0xff in (ord('q'), 27):
            cv2.destroyWindow('UAV-X | SURVEY RGB-D')
            self.window_visible = False
            self.window_closed = True