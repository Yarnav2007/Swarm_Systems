# #!/usr/bin/env python3
# """
# swarm_mission.py

# 15-drone BVLOS swarm mission with:
#   - MAVSDK control / telemetry
#   - role assignment: SURVEY / RELAY / RESERVE / RTH
#   - 100 m maximum communication-hop constraint
#   - live POI detection + reporting latency
#   - live terminal dashboard for judges
#   - JSONL telemetry logging
#   - Gazebo GUI operation
#   - ROS 2 camera/depth viewer for active survey drones

# The camera viewer is independent of the Gazebo GUI. POI detections are synthetic
# proximity checks in this proof of concept, not camera object recognition.

# The exact camera topic names can vary with the Gazebo/ROS bridge setup. The
# viewer auto-discovers sensor_msgs/msg/Image topics and associates topics whose
# names contain a vehicle/model index (x500_0, drone_0, etc.) with that drone.
# """

# import asyncio
# import contextlib
# import json
# import math
# import os
# import re
# import threading
# import time
# from dataclasses import dataclass
# from enum import Enum

# import numpy as np
# try:
#     import cv2
# except ImportError:
#     cv2 = None

# # When the mission runs from mavsdk_venv, expose the system ROS 2 Jazzy Python packages.
# ros_py = "/opt/ros/jazzy/lib/python3.12/site-packages"
# if os.path.isdir(ros_py) and ros_py not in os.sys.path:
#     os.sys.path.append(ros_py)

# from mavsdk import System
# from mavsdk.offboard import OffboardError, VelocityNedYaw
# from mavsdk.action import ActionError

# # ROS 2 camera viewer is optional at import time so that MAVSDK mission control
# # still reports an understandable status if ROS/camera packages are unavailable.
# try:
#     import rclpy
#     from rclpy.executors import SingleThreadedExecutor
#     from rclpy.node import Node
#     from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
#     from sensor_msgs.msg import Image as RosImage
#     ROS_CAMERA_AVAILABLE = True
# except Exception:
#     ROS_CAMERA_AVAILABLE = False
#     Node = object
#     RosImage = object


# # ============================================================
# # MISSION CONFIGURATION
# # ============================================================

# GCS_POSITION = (0.0, 0.0, 0.0)

# AREA_OFFSET_M = 75.0
# AREA_WIDTH_M = 1000.0
# AREA_HEIGHT_M = 1000.0
# AREA_X_MIN = GCS_POSITION[0] + AREA_OFFSET_M
# AREA_X_MAX = AREA_X_MIN + AREA_WIDTH_M
# AREA_Y_MIN = GCS_POSITION[1] - AREA_HEIGHT_M / 2.0
# AREA_Y_MAX = GCS_POSITION[1] + AREA_HEIGHT_M / 2.0

# MISSION_DURATION_S = 45 * 60
# UAV_MAX_FLIGHT_TIME_S = 20 * 60
# BATTERY_SWAP_S = float(os.getenv("SWARM_BATTERY_SWAP_S", "120"))
# RTH_RESERVE_S = 90.0
# LANDING_TIMEOUT_S = 120.0
# CONNECT_TIMEOUT_S = 180.0
# CONNECT_ATTEMPT_S = 20.0
# CONNECT_RETRY_S = 5.0
# STARTUP_SETTLE_S = 30.0
# MIN_READY_DRONES = 12
# MAX_POI_REPORT_LATENCY_S = 10.0

# # Communication requirement: no network hop may exceed 100 m.
# MAX_COMM_RANGE_M = 100.0
# RELAY_SAFETY_MARGIN_M = 0.6
# RELAY_HOP_SPACING_M = MAX_COMM_RANGE_M - RELAY_SAFETY_MARGIN_M
# FIRST_RELAY_DISTANCE_M = 95.0

# MAX_ALTITUDE_M = 100.0
# MAX_SPEED_MPS = 5.0
# MIN_SEPARATION_M = 20.0
# CRUISE_ALTITUDE_M = 20.0
# COMMANDED_SPEED_MPS = 4.0

# NUM_POIS = 10
# POI_DETECTION_RADIUS_M = 15.0
# SURVEY_LANE_SPACING_M = 28.0
# SCENARIO_SEED = int(os.getenv("SWARM_SCENARIO_SEED", "42"))

# NUM_DRONES = int(os.getenv("SWARM_NUM_DRONES", "15"))
# GROUND_SPAWN_SPACING_M = float(os.getenv("SWARM_SPACING_M", "25"))
# NUM_SURVEY_CELLS = 4

# BASE_MAVLINK_PORT = 14540
# EXTRA_MAVLINK_PORT_BASE = int(os.getenv("SWARM_EXTRA_PORT_BASE", "14640"))
# BASE_GRPC_PORT = 50051
# SETPOINT_PERIOD_S = 0.1
# ARRIVAL_TOLERANCE_M = 5.0
# COORD_LOOP_PERIOD_S = 1.0
# POSITION_STALE_S = 5.0


# def mavlink_port(drone_id: int) -> int:
#     return (BASE_MAVLINK_PORT + drone_id if drone_id < 10
#             else EXTRA_MAVLINK_PORT_BASE + drone_id)

# # Camera configuration.
# CAMERA_VIEWER_ENABLED = os.getenv("CAMERA_VIEWER", "1") != "0"
# CAMERA_WINDOW_TITLE = "UAV-X | LIVE SURVEY CAMERA + DEPTH"
# CAMERA_STALE_S = 2.0
# CAMERA_RENDER_PERIOD_S = 0.08
# DEPTH_MAX_DISPLAY_M = 30.0


# # ============================================================
# # COORDINATE CONVENTION
# # ============================================================

# def grid_spawn_position(i: int, n: int, spacing: float) -> tuple[float, float]:
#     cols = math.ceil(math.sqrt(n))
#     rows = math.ceil(n / cols)
#     col = i % cols
#     row = i // cols
#     x = (col - (cols - 1) / 2.0) * spacing
#     y = (row - (rows - 1) / 2.0) * spacing
#     return x, y


# def world_to_local_ned(world_x, world_y, world_z, spawn_x, spawn_y):
#     delta_x = world_x - spawn_x
#     delta_y = world_y - spawn_y
#     target_n = delta_y
#     target_e = delta_x
#     target_d = -world_z
#     return target_n, target_e, target_d


# def local_ned_to_world(n, e, d, spawn_x, spawn_y):
#     world_x = spawn_x + e
#     world_y = spawn_y + n
#     world_z = -d
#     return world_x, world_y, world_z


# def normalize_battery_pct(value: float) -> float:
#     return value * 100.0 if value <= 1.0 else value


# # ============================================================
# # ROLES
# # ============================================================

# class Role(str, Enum):
#     STANDBY = "STANDBY"
#     SURVEY = "SURVEY"
#     RELAY = "RELAY"
#     RTH = "RTH"
#     GROUNDED = "GROUNDED"


# @dataclass
# class DroneState:
#     id: int
#     role: Role = Role.STANDBY
#     assigned_cell: int | None = None


# class RoleManager:
#     def __init__(self, num_drones: int, num_survey_cells: int):
#         self.num_survey_cells = num_survey_cells
#         self.drones: dict[int, DroneState] = {i: DroneState(id=i) for i in range(num_drones)}

#     def reserve_pool(self, agents):
#         return [d for d in self.drones.values()
#                 if d.role == Role.STANDBY and agents[d.id].ready
#                 and agents[d.id].connected and agents[d.id].position_fresh()
#                 and not agents[d.id].failure
#                 and not agents[d.id].landed and not agents[d.id].armed
#                 and (not agents[d.id].battery_valid or agents[d.id].battery_pct > 30)]

#     def fill_survey(self, agents, survey_progress, limit):
#         assigned = []
#         for cell in range(self.num_survey_cells):
#             if len(self.active_survey()) >= limit:
#                 break
#             if survey_progress[cell]['complete'] or any(
#                     d.role == Role.SURVEY and d.assigned_cell == cell
#                     for d in self.drones.values()):
#                 continue
#             pool = self.reserve_pool(agents)
#             if not pool:
#                 break
#             chosen = pool[0]
#             chosen.role = Role.SURVEY
#             chosen.assigned_cell = cell
#             assigned.append((chosen.id, cell, survey_progress[cell]['idx']))
#         return assigned

#     def active_relays(self):
#         return [d for d in self.drones.values() if d.role == Role.RELAY]

#     def active_survey(self):
#         return [d for d in self.drones.values() if d.role == Role.SURVEY]

#     def rebalance_relays(self, required_relay_count: int, agents):
#         current = self.active_relays()
#         if len(current) < required_relay_count:
#             for drone in self.reserve_pool(agents)[:required_relay_count - len(current)]:
#                 drone.role = Role.RELAY
#         # Keep already deployed relays in flight; they land during RTH.

#     def promote_replacement(self, failed_drone_id: int, agents):
#         failed = self.drones[failed_drone_id]
#         failed_role, failed_cell = failed.role, failed.assigned_cell
#         old_agent = agents[failed_drone_id]
#         failed.role = (Role.RTH if old_agent.armed else
#                        Role.GROUNDED if old_agent.failure or old_agent.landed
#                        else Role.STANDBY)
#         failed.assigned_cell = None
#         reserve = [d for d in self.reserve_pool(agents) if d.id != failed_drone_id]
#         if reserve and failed_role in (Role.SURVEY, Role.RELAY):
#             replacement = reserve[0]
#             replacement.role = failed_role
#             replacement.assigned_cell = failed_cell
#             return replacement.id
#         return None


# # ============================================================
# # SURVEY PATTERN
# # ============================================================

# def cell_bounds(cell_index: int, num_cells: int):
#     strip_width = AREA_WIDTH_M / num_cells
#     x_min = AREA_X_MIN + cell_index * strip_width
#     x_max = x_min + strip_width
#     return x_min, x_max, AREA_Y_MIN, AREA_Y_MAX


# def lawnmower_waypoints(cell_index: int, num_cells: int, lane_spacing: float = 40.0):
#     x_min, x_max, y_min, y_max = cell_bounds(cell_index, num_cells)
#     waypoints = []
#     x = x_min + lane_spacing / 2
#     going_up = True
#     lanes = []
#     while x <= x_max - lane_spacing / 2:
#         lanes.append(x)
#         x += lane_spacing
#     if not lanes or x_max - lanes[-1] > POI_DETECTION_RADIUS_M:
#         lanes.append(x_max - POI_DETECTION_RADIUS_M)
#     for x in lanes:
#         if going_up:
#             waypoints += [(x, y_min), (x, y_max)]
#         else:
#             waypoints += [(x, y_max), (x, y_min)]
#         going_up = not going_up
#     return waypoints


# def planned_survey_distance_m():
#     total = 0.0
#     for cell in range(NUM_SURVEY_CELLS):
#         wps = lawnmower_waypoints(cell, NUM_SURVEY_CELLS, SURVEY_LANE_SPACING_M)
#         total += sum(math.dist(a, b) for a, b in zip(wps, wps[1:]))
#     return total


# # ============================================================
# # POI SIMULATION
# # ============================================================

# @dataclass
# class POI:
#     id: int
#     x: float
#     y: float
#     spawn_time_s: float
#     priority: int = 1
#     announced: bool = False
#     detected: bool = False
#     detected_by: int | None = None
#     detected_at_s: float = 0.0
#     reported_at_s: float = 0.0


# class POISimulator:
#     def __init__(self, num_pois: int = NUM_POIS,
#                  mission_duration_s: float = MISSION_DURATION_S,
#                  seed: int | None = None):
#         import random
#         rng = random.Random(seed)
#         self.start_time: float | None = None
#         self.pois: list[POI] = []
#         for i in range(num_pois):
#             x = rng.uniform(AREA_X_MIN, AREA_X_MAX)
#             y = rng.uniform(AREA_Y_MIN, AREA_Y_MAX)
#             spawn_t = rng.uniform(0, mission_duration_s * 0.8)
#             priority = 3 if rng.random() < 0.25 else 1
#             self.pois.append(POI(id=i, x=x, y=y, spawn_time_s=spawn_t,
#                                  priority=priority))

#     def elapsed(self) -> float:
#         return time.monotonic() - self.start_time if self.start_time is not None else 0.0

#     def start(self):
#         self.start_time = time.monotonic()

#     def active_pois(self):
#         t = self.elapsed()
#         return [p for p in self.pois if p.spawn_time_s <= t and not p.detected]

#     def newly_announced(self):
#         new = [p for p in self.active_pois() if not p.announced]
#         for poi in new:
#             poi.announced = True
#         return new

#     def check_detections(self, survey_positions: dict[int, tuple[float, float]]):
#         newly_detected = []
#         t = self.elapsed()
#         for poi in self.active_pois():
#             for drone_id, (dx, dy) in survey_positions.items():
#                 if math.dist((dx, dy), (poi.x, poi.y)) <= POI_DETECTION_RADIUS_M:
#                     poi.detected = True
#                     poi.detected_by = drone_id
#                     poi.detected_at_s = t
#                     newly_detected.append(poi)
#                     break
#         return newly_detected

#     def mark_reported(self, poi: POI):
#         poi.reported_at_s = self.elapsed()

#     def summary(self) -> str:
#         found = sum(1 for p in self.pois if p.detected)
#         reported = sum(1 for p in self.pois if p.reported_at_s > 0)
#         return f"{found}/{len(self.pois)} found, {reported}/{len(self.pois)} reported"

#     def worst_latency(self) -> float:
#         values = [p.reported_at_s - p.detected_at_s for p in self.pois if p.reported_at_s > 0]
#         return max(values) if values else 0.0


# # ============================================================
# # RELAY / NETWORK
# # ============================================================

# def plan_relay_chain(frontier_point, gcs=GCS_POSITION[:2], hop_spacing=RELAY_HOP_SPACING_M):
#     gx, gy = gcs
#     fx, fy = frontier_point
#     total_dist = math.dist((gx, gy), (fx, fy))
#     # The first link climbs from the ground station to flight altitude.
#     first_link_horizontal = math.sqrt(MAX_COMM_RANGE_M ** 2 - CRUISE_ALTITUDE_M ** 2)
#     if total_dist <= first_link_horizontal:
#         return []
#     distances = [FIRST_RELAY_DISTANCE_M]
#     while total_dist - distances[-1] > hop_spacing:
#         distances.append(distances[-1] + hop_spacing)
#     return [(gx + distance / total_dist * (fx - gx),
#              gy + distance / total_dist * (fy - gy))
#             for distance in distances]


# def is_connected(point, gcs, relay_positions, max_range=MAX_COMM_RANGE_M):
#     nodes = [gcs] + list(relay_positions) + [point]
#     n = len(nodes)
#     visited = [False] * n
#     visited[0] = True
#     frontier = [0]
#     while frontier:
#         next_frontier = []
#         for i in frontier:
#             for j in range(n):
#                 if not visited[j] and math.dist(nodes[i], nodes[j]) <= max_range:
#                     visited[j] = True
#                     next_frontier.append(j)
#         frontier = next_frontier
#     return visited[n - 1]


# def network_hop_distances(relay_positions, survey_positions, gcs=GCS_POSITION[:2]):
#     """Return the physical links used by the linear GCS->relay->survey chain."""
#     points = [gcs] + list(relay_positions)
#     hops = []
#     for a, b in zip(points, points[1:]):
#         hops.append(math.dist(a, b))
#     if relay_positions:
#         last_relay = relay_positions[-1]
#     else:
#         last_relay = gcs
#     for pos in survey_positions.values():
#         hops.append(math.dist(last_relay, pos))
#     return hops


# def min_pairwise_distance(positions: list[tuple[float, float]]) -> float:
#     if len(positions) < 2:
#         return float("inf")
#     best = float("inf")
#     for i in range(len(positions)):
#         for j in range(i + 1, len(positions)):
#             best = min(best, math.dist(positions[i], positions[j]))
#     return best


# def max_pairwise_distance(positions: list[tuple[float, float]]) -> float:
#     if len(positions) < 2:
#         return 0.0
#     best = 0.0
#     for i in range(len(positions)):
#         for j in range(i + 1, len(positions)):
#             best = max(best, math.dist(positions[i], positions[j]))
#     return best


# # ============================================================
# # TELEMETRY LOGGING
# # ============================================================

# class TelemetryLogger:
#     def __init__(self, log_dir: str | None = None):
#         if log_dir is None:
#             log_dir = os.getenv("SWARM_LOG_DIR", "logs")
#         os.makedirs(log_dir, exist_ok=True)
#         fname = f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
#         self.path = os.path.join(log_dir, fname)
#         self._f = open(self.path, "a", buffering=1)

#     def log(self, record: dict):
#         self._f.write(json.dumps(record) + "\n")

#     def close(self):
#         if not self._f.closed:
#             self._f.close()


# # ============================================================
# # LIVE ROS 2 CAMERA + DEPTH VIEWER
# # ============================================================

# class CameraViewer:
#     """Background ROS 2 viewer for active survey-drone camera/depth topics."""

#     def __init__(self, num_drones: int):
#         self.num_drones = num_drones
#         self.active_ids: set[int] = set()
#         self.frames: dict[tuple[int, str], tuple[np.ndarray, float, str]] = {}
#         self.subscriptions = {}
#         self.lock = threading.Lock()
#         self.stop_event = threading.Event()
#         self.thread: threading.Thread | None = None
#         self.node = None
#         self.executor = None
#         self.error = ""
#         self.discovered_topics: dict[str, int] = {}

#     def start(self):
#         if not CAMERA_VIEWER_ENABLED:
#             self.error = "disabled by CAMERA_VIEWER=0"
#             return
#         if cv2 is None:
#             self.error = "OpenCV unavailable; install opencv-python for camera viewing"
#             return
#         if not ROS_CAMERA_AVAILABLE:
#             self.error = "ROS 2 camera packages unavailable (rclpy/sensor_msgs)"
#             return
#         self.thread = threading.Thread(target=self._thread_main, daemon=True)
#         self.thread.start()

#     def stop(self):
#         self.stop_event.set()
#         if self.thread and self.thread.is_alive():
#             self.thread.join(timeout=2.0)
#         try:
#             cv2.destroyAllWindows()
#         except Exception:
#             pass

#     def update_active(self, drone_ids):
#         with self.lock:
#             self.active_ids = set(drone_ids)

#     def status(self, drone_id: int) -> str:
#         if self.error:
#             return "N/A"
#         now = time.monotonic()
#         with self.lock:
#             rgb = self.frames.get((drone_id, "rgb"))
#             depth = self.frames.get((drone_id, "depth"))
#         rgb_ok = rgb is not None and now - rgb[1] < CAMERA_STALE_S
#         depth_ok = depth is not None and now - depth[1] < CAMERA_STALE_S
#         if rgb_ok and depth_ok:
#             return "RGB+DEP"
#         if rgb_ok:
#             return "RGB"
#         if depth_ok:
#             return "DEP"
#         return "WAIT"

#     def depth_distance(self, drone_id: int) -> float | None:
#         with self.lock:
#             item = self.frames.get((drone_id, "depth"))
#         if item is None or time.monotonic() - item[1] >= CAMERA_STALE_S:
#             return None
#         frame = item[0]
#         if frame.ndim == 3:
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         values = frame.astype(np.float32)
#         if frame.dtype == np.uint16:
#             values *= 0.001
#         valid = values[np.isfinite(values) & (values > 0)]
#         if valid.size == 0:
#             return None
#         # Median of the closest 5% gives a stable "nearest visible depth" metric.
#         valid.sort()
#         k = max(1, int(valid.size * 0.05))
#         return float(np.median(valid[:k]))

#     def _thread_main(self):
#         try:
#             rclpy.init(args=None)
#             self.node = _CameraNode(self)
#             self.executor = SingleThreadedExecutor()
#             self.executor.add_node(self.node)
#             while rclpy.ok() and not self.stop_event.is_set():
#                 self.executor.spin_once(timeout_sec=0.05)
#                 self._render()
#             self.executor.remove_node(self.node)
#             self.node.destroy_node()
#             if rclpy.ok():
#                 rclpy.shutdown()
#         except Exception as exc:
#             self.error = f"{type(exc).__name__}: {exc}"

#     def _render(self):
#         now = time.monotonic()
#         with self.lock:
#             active = sorted(self.active_ids)
#             records = {
#                 drone_id: {
#                     "rgb": self.frames.get((drone_id, "rgb")),
#                     "depth": self.frames.get((drone_id, "depth")),
#                 }
#                 for drone_id in active
#             }

#         if not active:
#             return

#         tiles = []
#         for drone_id in active:
#             rec = records[drone_id]
#             rgb_item = rec["rgb"]
#             depth_item = rec["depth"]
#             rgb_ok = rgb_item is not None and now - rgb_item[1] < CAMERA_STALE_S
#             depth_ok = depth_item is not None and now - depth_item[1] < CAMERA_STALE_S

#             if rgb_ok:
#                 rgb = rgb_item[0]
#                 rgb = self._resize_keep(rgb, 360, 240)
#             else:
#                 rgb = np.zeros((240, 360, 3), dtype=np.uint8)
#                 cv2.putText(rgb, "RGB WAIT", (115, 125), cv2.FONT_HERSHEY_SIMPLEX,
#                             0.8, (255, 255, 255), 2, cv2.LINE_AA)

#             if depth_ok:
#                 depth = self._depth_display(depth_item[0], 360, 240)
#             else:
#                 depth = np.zeros((240, 360, 3), dtype=np.uint8)
#                 cv2.putText(depth, "DEPTH WAIT", (95, 125), cv2.FONT_HERSHEY_SIMPLEX,
#                             0.8, (255, 255, 255), 2, cv2.LINE_AA)

#             tile = np.hstack([rgb, depth])
#             label = f"DRONE {drone_id:02d} | RGB {'OK' if rgb_ok else '---'} | DEPTH {'OK' if depth_ok else '---'}"
#             cv2.rectangle(tile, (0, 0), (tile.shape[1], 32), (0, 0, 0), -1)
#             cv2.putText(tile, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
#                         0.62, (255, 255, 255), 2, cv2.LINE_AA)
#             tiles.append(tile)

#         cols = 2 if len(tiles) <= 4 else 3
#         rows = math.ceil(len(tiles) / cols)
#         tile_h, tile_w = tiles[0].shape[:2]
#         mosaic = np.zeros((rows * tile_h, cols * tile_w, 3), dtype=np.uint8)
#         for i, tile in enumerate(tiles):
#             r, c = divmod(i, cols)
#             mosaic[r * tile_h:(r + 1) * tile_h,
#                    c * tile_w:(c + 1) * tile_w] = tile

#         cv2.imshow(CAMERA_WINDOW_TITLE, mosaic)
#         key = cv2.waitKey(1) & 0xFF
#         if key in (ord("q"), 27):
#             # q closes the viewer window but does not stop the mission.
#             cv2.destroyWindow(CAMERA_WINDOW_TITLE)

#     @staticmethod
#     def _resize_keep(frame, width, height):
#         return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

#     @staticmethod
#     def _depth_display(frame, width, height):
#         if frame.ndim == 3:
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         depth = frame.astype(np.float32)
#         depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
#         # Common ROS depth images are millimetres for 16UC1 and metres for 32FC1.
#         if frame.dtype == np.uint16:
#             depth *= 0.001
#         depth = np.clip(depth, 0.0, DEPTH_MAX_DISPLAY_M)
#         norm = (depth / DEPTH_MAX_DISPLAY_M * 255.0).astype(np.uint8)
#         norm = 255 - norm
#         depth_color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
#         return cv2.resize(depth_color, (width, height), interpolation=cv2.INTER_NEAREST)


# class _CameraNode(Node):
#     def __init__(self, viewer: CameraViewer):
#         super().__init__("uav_x_survey_camera_viewer")
#         self.viewer = viewer
#         self._subs = {}
#         self._timer = self.create_timer(1.0, self._discover_topics)
#         self._discover_topics()

#     def _discover_topics(self):
#         for topic, types in self.get_topic_names_and_types():
#             if topic in self._subs:
#                 continue
#             if "sensor_msgs/msg/Image" not in types:
#                 continue
#             low = topic.lower()
#             if not any(k in low for k in ("camera", "image", "rgb", "depth")):
#                 continue
#             drone_id = self._infer_drone_id(topic)
#             if drone_id is None or not (0 <= drone_id < self.viewer.num_drones):
#                 continue
#             kind = "depth" if "depth" in low else "rgb"
#             qos = QoSProfile(depth=1)
#             qos.reliability = ReliabilityPolicy.BEST_EFFORT
#             qos.durability = DurabilityPolicy.VOLATILE
#             callback = lambda msg, t=topic, d=drone_id, k=kind: self._image_callback(msg, t, d, k)
#             self._subs[topic] = self.create_subscription(RosImage, topic, callback, qos)
#             self.viewer.discovered_topics[topic] = drone_id

#     def _image_callback(self, msg: RosImage, topic: str, drone_id: int, kind: str):
#         frame = self._image_to_numpy(msg)
#         if frame is None:
#             return
#         with self.viewer.lock:
#             self.viewer.frames[(drone_id, kind)] = (frame, time.monotonic(), topic)

#     @staticmethod
#     def _image_to_numpy(msg: RosImage):
#         try:
#             encoding = msg.encoding.lower()
#             data = np.frombuffer(msg.data, dtype=np.uint8)
#             step = msg.step
#             h, w = msg.height, msg.width

#             if encoding in ("bgr8", "rgb8"):
#                 channels = 3
#                 arr = data.reshape(h, step)[:, :w * channels].reshape(h, w, channels)
#                 arr = np.ascontiguousarray(arr)
#                 if encoding == "rgb8":
#                     arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
#                 return arr

#             if encoding in ("mono8", "8uc1"):
#                 arr = data.reshape(h, step)[:, :w].reshape(h, w)
#                 return cv2.cvtColor(np.ascontiguousarray(arr), cv2.COLOR_GRAY2BGR)

#             if encoding in ("16uc1", "mono16"):
#                 raw = np.frombuffer(msg.data, dtype=np.uint16)
#                 row_width = step // 2
#                 arr = raw.reshape(h, row_width)[:, :w].reshape(h, w)
#                 return np.ascontiguousarray(arr)

#             if encoding in ("32fc1", "32fc"):
#                 raw = np.frombuffer(msg.data, dtype=np.float32)
#                 row_width = step // 4
#                 arr = raw.reshape(h, row_width)[:, :w].reshape(h, w)
#                 return np.ascontiguousarray(arr)

#             # Generic fallback for uncommon encodings with 3 bytes/pixel.
#             if "rgb" in encoding or "bgr" in encoding:
#                 channels = 3
#                 arr = data.reshape(h, step)[:, :w * channels].reshape(h, w, channels)
#                 arr = np.ascontiguousarray(arr)
#                 if "rgb" in encoding and "bgr" not in encoding:
#                     arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
#                 return arr
#         except Exception:
#             return None
#         return None

#     @staticmethod
#     def _infer_drone_id(topic: str):
#         patterns = [
#             r"x500(?:_depth)?[_-](\d+)",
#             r"(?:drone|uav|vehicle|model)[_-](\d+)",
#             r"(?:^|[/_-])(?:drone|uav|vehicle)(\d+)(?:[/_-]|$)",
#         ]
#         for pattern in patterns:
#             match = re.search(pattern, topic.lower())
#             if match:
#                 return int(match.group(1))

#         # Fallback: use a numeric path component immediately before a model/link/sensor path.
#         parts = [p for p in topic.strip("/").split("/") if p]
#         for part in parts:
#             m = re.fullmatch(r"(?:x500|vehicle|drone|uav)[_-]?(\d+)", part.lower())
#             if m:
#                 return int(m.group(1))
#         return None


# # ============================================================
# # MISSION TELEMETRY
# # ============================================================

# class CoverageTracker:
#     """Synthetic proximity coverage of 20 m sample cells in the 1 km arena."""
#     def __init__(self, resolution_m=20.0):
#         self.resolution_m = resolution_m
#         self.columns = math.ceil(AREA_WIDTH_M / resolution_m)
#         self.rows = math.ceil(AREA_HEIGHT_M / resolution_m)
#         self.visited = set()

#     def observe(self, x, y):
#         col = int((x - AREA_X_MIN) / self.resolution_m)
#         row = int((y - AREA_Y_MIN) / self.resolution_m)
#         for cx in range(max(0, col - 1), min(self.columns, col + 2)):
#             for cy in range(max(0, row - 1), min(self.rows, row + 2)):
#                 px = AREA_X_MIN + (cx + 0.5) * self.resolution_m
#                 py = AREA_Y_MIN + (cy + 0.5) * self.resolution_m
#                 if math.dist((x, y), (px, py)) <= POI_DETECTION_RADIUS_M:
#                     self.visited.add((cx, cy))

#     def fraction(self):
#         return len(self.visited) / (self.columns * self.rows)


# def fmt_time(seconds):
#     minutes, secs = divmod(max(0, int(seconds)), 60)
#     return f"{minutes:02d}:{secs:02d}"


# def connected_network(agents, role_mgr):
#     """Geometric range model over current airborne, fresh telemetry positions."""
#     active = {
#         d.id: agents[d.id].position_world
#         for d in role_mgr.drones.values()
#         if agents[d.id].armed and agents[d.id].connected
#         and agents[d.id].position_fresh()
#     }
#     relay_ids = {d.id for d in role_mgr.active_relays()} & active.keys()
#     reached = set()
#     frontier = [GCS_POSITION]
#     used_links = []
#     while frontier:
#         origin = frontier.pop(0)
#         for drone_id in relay_ids - reached:
#             pos = active[drone_id]
#             distance = math.dist(origin, pos)
#             if distance <= MAX_COMM_RANGE_M:
#                 reached.add(drone_id)
#                 frontier.append(pos)
#                 used_links.append(distance)
#     link = {}
#     for drone_id, pos in active.items():
#         link[drone_id] = drone_id in reached or any(
#             math.dist(pos, src) <= MAX_COMM_RANGE_M
#             for src in [GCS_POSITION] + [active[i] for i in reached]
#         )
#     for d in role_mgr.drones.values():
#         link.setdefault(d.id, False)
#     return link, max(used_links, default=0.0)


# def relay_targets(survey_positions, budget):
#     """Merge 90 m radial relay chains; excess capacity remains visibly unmet."""
#     result = []
#     for pos in survey_positions.values():
#         for point in plan_relay_chain(pos[:2]):
#             if all(math.dist(point, existing) >= 45.0 for existing in result):
#                 result.append(point)
#     return result[:budget], len(result)


# def match_relays_to_waypoints(relays, agents, waypoints):
#     """Greedy minimum-distance matching to avoid swapping an established chain."""
#     pairs = sorted(
#         (math.dist(agents[d.id].position_world[:2], waypoint), d.id, index)
#         for d in relays for index, waypoint in enumerate(waypoints))
#     assigned = {}
#     occupied = set()
#     for _, drone_id, index in pairs:
#         if drone_id not in assigned and index not in occupied:
#             assigned[drone_id] = waypoints[index]
#             occupied.add(index)
#     return assigned


# def minimum_separation(agents):
#     airborne = [a.position_world for a in agents.values() if a.in_air and a.position_fresh()]
#     return min_pairwise_distance(airborne)


# def render_dashboard(agents, role_mgr, poi_sim, elapsed, links, used_hop,
#                      required_relays, viewer, events, min_sep, tlog,
#                      coverage, survey_progress, metrics):
#     alive = [a for a in agents.values() if a.connected and a.position_fresh()]
#     armed = [a for a in agents.values() if a.armed]
#     max_speed = max((a.speed_mps for a in alive if a.in_air), default=0.0)
#     max_alt = max((a.position_world[2] for a in alive if a.in_air), default=0.0)
#     longest = max((a.flight_time() for a in agents.values()), default=0.0)
#     fresh_batteries = [a.battery_pct for a in alive if a.battery_valid]
#     min_battery = min(fresh_batteries) if fresh_batteries else None
#     outside = sum(1 for a in alive if a.in_air and not inside_geofence(a.position_world))
#     network_ok = all(links[d.id] for d in role_mgr.active_survey() if agents[d.id].armed)
#     network_ok &= len(role_mgr.active_relays()) >= required_relays
#     over_latency = sum(1 for p in poi_sim.pois if p.detected and p.reported_at_s == 0
#                        and poi_sim.elapsed() - p.detected_at_s > MAX_POI_REPORT_LATENCY_S)
#     late = sum(1 for p in poi_sim.pois if p.reported_at_s > 0 and
#                p.reported_at_s - p.detected_at_s > MAX_POI_REPORT_LATENCY_S)
#     sep_text = 'N/A' if math.isinf(min_sep) else f'{min_sep:.1f}'
#     batt_text = 'N/A' if min_battery is None else f'{min_battery:.0f}%'
#     print(f"\nUAV-X | MISSION WALL T+{fmt_time(elapsed)}/{fmt_time(MISSION_DURATION_S)} "
#           f"({fmt_time(MISSION_DURATION_S-elapsed)} remaining) | "
#           f"{poi_sim.summary()} | log {tlog.path}", flush=True)
#     ready = [a.id for a in agents.values() if a.ready]
#     pending = [a.id for a in agents.values() if not a.ready and not a.landed]
#     print(f"Fleet {len(alive)}/{len(agents)} telemetry | ready {len(ready)} | pending {pending} "
#           f"| armed {len(armed)} | survey {len(role_mgr.active_survey())} "
#           f"| relay {len(role_mgr.active_relays())}/{required_relays} required "
#           f"| network {'OK' if network_ok else 'DEGRADED'} | longest used relay hop {used_hop:.1f}/{MAX_COMM_RANGE_M:.0f}m")
#     strips = ' '.join(f'{cell}:{progress["idx"]}/{len(progress["wps"])}'
#                       for cell, progress in survey_progress.items())
#     link_sample_s = metrics['link_ok_s'] + metrics['link_down_s']
#     availability = metrics['link_ok_s'] / link_sample_s * 100 if link_sample_s else 0.0
#     print(f"Coverage (synthetic 20m grid): {coverage.fraction()*100:.1f}% | "
#           f"strip waypoints {strips} | connectivity availability "
#           f"{availability:.1f}% | downtime "
#           f"{metrics['link_down_s']:.0f}s | role reallocations {metrics['reallocations']}")
#     print(f"Battery swaps modeled: {sum(max(0, a.sorties-1) for a in agents.values())} "
#           f"({BATTERY_SWAP_S:.0f}s ground service) | PDR: N/A (no packet model) "
#           f"| separation breaches: {metrics['collision_events']}")
#     priority_total = sum(p.priority for p in poi_sim.pois)
#     priority_reported = sum(p.priority for p in poi_sim.pois if p.reported_at_s > 0)
#     recovery = [delay for a in agents.values() for delay in a.recovery_durations]
#     print(f"Priority score {priority_reported}/{priority_total} | "
#           f"worst link recovery {max(recovery, default=0.0):.1f}s | "
#           f"actual collision count: N/A (no collision sensor)")
#     print(f"Limits: flight {fmt_time(longest)}/{fmt_time(UAV_MAX_FLIGHT_TIME_S)} | "
#           f"speed {max_speed:.1f}/{MAX_SPEED_MPS:.0f}m/s | altitude {max_alt:.1f}/{MAX_ALTITUDE_M:.0f}m | "
#           f"separation {sep_text}/{MIN_SEPARATION_M:.0f}m | outside fence {outside} | min battery {batt_text}")
#     print(f"POI: detected/reported {poi_sim.summary()} | worst reporting delay "
#           f"{poi_sim.worst_latency():.1f}/{MAX_POI_REPORT_LATENCY_S:.0f}s | late {late} | overdue {over_latency} "
#           f"| camera {'OFF' if viewer.error else 'ON'} ({len(viewer.discovered_topics)} topics)")
#     print('ID  ROLE       X      Y     Z  SPEED  BATT  FLIGHT  LINK  CAMERA  STATE/RETRIES')
#     for d in role_mgr.drones.values():
#         a = agents[d.id]
#         x, y, z = a.position_world
#         state = ('LAND' if a.landing else 'PAUSED' if a.pause_for_link
#                  else 'AIR' if a.in_air else 'READY' if a.ready else 'WAIT')
#         if a.failure:
#             state = 'ERROR'
#         if not a.ready and not a.armed:
#             state = f'RETRY({a.connection_retries})'
#         elif a.landed and a.service_ready_at:
#             state = f'SERVICE({max(0, int(a.service_ready_at-time.monotonic()))}s)'
#         print(f'{d.id:02d}  {d.role.value:<9} {x:6.0f} {y:6.0f} {z:5.0f} '
#               f'{a.speed_mps:6.1f} {a.battery_pct:5.0f}% {fmt_time(a.flight_time()):>7} '
#               f'{"UP" if links[d.id] else "--":>5} {viewer.status(d.id) if d.role == Role.SURVEY else "--":>7} {state}')
#     for event in events[-3:]:
#         print('EVENT ' + event)
#     if viewer.error:
#         print('CAMERA ' + viewer.error)


# # ============================================================
# # DRONE AGENT
# # ============================================================

# def inside_geofence(pos):
#     x, y, z = pos
#     return (-75 <= x <= AREA_X_MAX and AREA_Y_MIN <= y <= AREA_Y_MAX
#             and -2 <= z <= MAX_ALTITUDE_M)


# class DroneAgent:
#     def __init__(self, drone_id, spawn_xy):
#         self.id = drone_id
#         self.spawn_x, self.spawn_y = spawn_xy
#         self.system = None
#         self.position_world = (self.spawn_x, self.spawn_y, 0.0)
#         self.target_world = self.position_world
#         self.last_position_at = 0.0
#         self.battery_pct = 100.0
#         self.raw_battery_pct = None
#         self.battery_source = 'PX4'
#         self.battery_valid = False
#         self.speed_mps = 0.0
#         self.armed = False
#         self.connected = False
#         self.ready = False
#         self.ready_event = asyncio.Event()
#         self.connection_error = ''
#         self.connection_retries = 0
#         self.link_lost_at = None
#         self.recovery_durations = []
#         self.in_air = False
#         self.landing = False
#         self.landed = False
#         self.failure = ''
#         self.flight_start = None
#         self.flight_end = None
#         self.sorties = 0
#         self.service_ready_at = None
#         self._stop = False
#         self.peers = {}
#         self.pause_for_link = False

#     async def connect(self, events):
#         port = mavlink_port(self.id)
#         async def wait_link():
#             async for state in self.system.core.connection_state():
#                 self.connected = state.is_connected
#                 if state.is_connected:
#                     return

#         async def wait_health():
#             async for health in self.system.telemetry.health():
#                 if health.is_global_position_ok and health.is_home_position_ok:
#                     return

#         while not self._stop:
#             try:
#                 if self.system is None:
#                     self.system = System(port=BASE_GRPC_PORT + self.id)
#                     try:
#                         await asyncio.wait_for(
#                             self.system.connect(system_address=f'udpin://127.0.0.1:{port}'),
#                             timeout=CONNECT_ATTEMPT_S)
#                     except Exception:
#                         self.system = None
#                         raise
#                 await asyncio.wait_for(wait_link(), timeout=CONNECT_ATTEMPT_S)
#                 await asyncio.wait_for(wait_health(), timeout=CONNECT_ATTEMPT_S)
#                 self.ready = True
#                 self.ready_event.set()
#                 self.connection_error = ''
#                 if self.link_lost_at is not None:
#                     self.recovery_durations.append(time.monotonic() - self.link_lost_at)
#                     self.link_lost_at = None
#                 events.append(f'drone {self.id} READY on UDP {port}')
#                 # Keep watching after readiness; a recovered vehicle returns to standby.
#                 async for state in self.system.core.connection_state():
#                     if not state.is_connected:
#                         self.connected = False
#                         self.ready = False
#                         self.link_lost_at = time.monotonic()
#                         self.connection_error = 'MAVLink link lost'
#                         events.append(f'drone {self.id} disconnected; retrying in background')
#                         break
#             except asyncio.CancelledError:
#                 raise
#             except Exception as exc:
#                 self.ready = False
#                 self.connection_error = f'{type(exc).__name__}: {exc}'
#             if self._stop:
#                 break
#             self.connection_retries += 1
#             events.append(f'drone {self.id} connection retry {self.connection_retries}: '
#                           f'{self.connection_error or "waiting for link/position"}')
#             await asyncio.sleep(CONNECT_RETRY_S)

#     def position_fresh(self):
#         return bool(self.last_position_at and time.monotonic() - self.last_position_at < POSITION_STALE_S)

#     async def arm_and_start_offboard(self):
#         try:
#             await self.system.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
#             await self.system.action.arm()
#             self.armed = True
#             self.flight_start = time.monotonic()
#             self.flight_end = None
#             self.sorties += 1
#             await self.system.offboard.start()
#             return True
#         except (ActionError, OffboardError) as exc:
#             self.failure = f'arm/offboard: {exc}'
#             if self.armed:
#                 with contextlib.suppress(Exception):
#                     await self.system.action.disarm()
#                 self.flight_end = time.monotonic()
#             self.armed = False
#             return False

#     def set_world_target(self, x, y, z):
#         self.target_world = (min(AREA_X_MAX, max(-75.0, x)),
#                              min(AREA_Y_MAX, max(AREA_Y_MIN, y)),
#                              min(MAX_ALTITUDE_M, max(0.0, z)))

#     async def stream_setpoints(self):
#         while not self._stop:
#             if self.armed and not self.landing:
#                 x, y, z = self.position_world
#                 tx, ty, tz = self.target_world
#                 dx, dy, dz = tx - x, ty - y, tz - z
#                 distance = math.hypot(dx, dy)
#                 speed = min(COMMANDED_SPEED_MPS, distance * 0.6)
#                 vx = speed * dx / distance if distance > 0.1 else 0.0
#                 vy = speed * dy / distance if distance > 0.1 else 0.0
#                 vz = max(-1.5, min(1.5, dz * 0.5))
#                 if self.pause_for_link and self.in_air:
#                     vx = vy = 0.0
#                 # Stop motion into the 20 m safety bubble around a live vehicle.
#                 for other in self.peers.values():
#                     if other.id == self.id or not other.in_air or not other.position_fresh():
#                         continue
#                     ox, oy, oz = other.position_world
#                     current = math.dist((x, y, z), (ox, oy, oz))
#                     future = math.dist((x + vx * 3, y + vy * 3, z + vz * 3),
#                                        (ox, oy, oz))
#                     if future < MIN_SEPARATION_M + 4 and future < current:
#                         vx = vy = vz = 0.0
#                         break
#                 try:
#                     # World X maps to local East, world Y to local North.
#                     await self.system.offboard.set_velocity_ned(VelocityNedYaw(
#                         vy, vx, -vz, 0))
#                 except Exception as exc:
#                     self.failure = f'setpoint: {exc}'
#             await asyncio.sleep(SETPOINT_PERIOD_S)

#     async def track_position(self):
#         async for pv in self.system.telemetry.position_velocity_ned():
#             if self._stop:
#                 break
#             self.position_world = local_ned_to_world(
#                 pv.position.north_m, pv.position.east_m, pv.position.down_m,
#                 self.spawn_x, self.spawn_y)
#             self.speed_mps = math.sqrt(pv.velocity.north_m_s ** 2 +
#                                        pv.velocity.east_m_s ** 2 + pv.velocity.down_m_s ** 2)
#             self.last_position_at = time.monotonic()

#     async def track_battery(self):
#         async for batt in self.system.telemetry.battery():
#             if self._stop:
#                 break
#             self.raw_battery_pct = normalize_battery_pct(batt.remaining_percent)
#             if self.battery_source == 'PX4':
#                 self.battery_pct = self.raw_battery_pct
#             self.battery_valid = True

#     async def track_air(self):
#         async for flying in self.system.telemetry.in_air():
#             if self._stop:
#                 break
#             self.in_air = flying
#             if self.landing and not flying:
#                 self.flight_end = time.monotonic()
#                 self.armed = False
#                 self.landed = True
#                 if self.service_ready_at is None:
#                     self.service_ready_at = time.monotonic() + BATTERY_SWAP_S

#     async def land(self):
#         if self.landing or not self.armed:
#             return
#         self.landing = True
#         with contextlib.suppress(Exception):
#             await self.system.offboard.stop()
#         try:
#             await self.system.action.land()
#         except ActionError as exc:
#             self.failure = f'land: {exc}'

#     def flight_time(self):
#         if self.flight_start is None:
#             return 0.0
#         return (self.flight_end or time.monotonic()) - self.flight_start

#     def complete_simulated_battery_swap(self):
#         if not self.landed or self.service_ready_at is None:
#             return False
#         if time.monotonic() < self.service_ready_at:
#             return False
#         self.battery_source = 'SIMULATED_SWAP'
#         self.battery_pct = 100.0
#         self.battery_valid = True
#         self.landing = False
#         self.landed = False
#         self.service_ready_at = None
#         return True

#     def request_stop(self):
#         self._stop = True


# async def run_agent(agent, role_mgr, events):
#     await agent.ready_event.wait()
#     trackers = [asyncio.create_task(agent.track_position()),
#                 asyncio.create_task(agent.track_battery()),
#                 asyncio.create_task(agent.track_air())]
#     streamer = None
#     try:
#         while not agent._stop:
#             role = role_mgr.drones[agent.id].role
#             if role == Role.GROUNDED and agent.landed and agent.ready:
#                 if agent.complete_simulated_battery_swap():
#                     role_mgr.drones[agent.id].role = Role.STANDBY
#                     events.append(f'drone {agent.id} simulated pack swap complete; '
#                                   f'available for a new sortie')
#             if agent.battery_source == 'SIMULATED_SWAP' and agent.armed:
#                 agent.battery_pct = max(0.0, 100.0 * (
#                     1.0 - agent.flight_time() / UAV_MAX_FLIGHT_TIME_S))
#             if role in (Role.SURVEY, Role.RELAY) and not agent.armed and not agent.landed and not agent.failure:
#                 if not agent.position_fresh():
#                     await asyncio.sleep(0.5)
#                     continue
#                 if await agent.arm_and_start_offboard():
#                     events.append(f'drone {agent.id} airborne task started ({role.value})')
#                     if streamer is None or streamer.done():
#                         streamer = asyncio.create_task(agent.stream_setpoints())
#             elif role == Role.RTH and agent.armed and not agent.landing:
#                 x, y, z = agent.position_world
#                 if math.dist((x, y), (agent.spawn_x, agent.spawn_y)) <= 5.0 and z <= 10:
#                     await agent.land()
#                     events.append(f'drone {agent.id} landing at start area')
#             await asyncio.sleep(0.5)
#     except asyncio.CancelledError:
#         raise
#     except Exception as exc:
#         agent.failure = f'{type(exc).__name__}: {exc}'
#         events.append(f'drone {agent.id} ERROR: {agent.failure}')
#     finally:
#         for task in trackers + ([streamer] if streamer else []):
#             task.cancel()
#         await asyncio.gather(*trackers, *([streamer] if streamer else []), return_exceptions=True)


# # ============================================================
# # MISSION COORDINATOR
# # ============================================================

# async def coordination_loop(agents, role_mgr, poi_sim, tlog, viewer, events):
#     poi_sim.start()
#     mission_start = time.monotonic()
#     survey_progress = {
#         cell: {'wps': lawnmower_waypoints(cell, role_mgr.num_survey_cells,
#                                         SURVEY_LANE_SPACING_M),
#                'idx': 0, 'complete': False}
#         for cell in range(role_mgr.num_survey_cells)
#     }
#     coverage = CoverageTracker()
#     metrics = {'link_ok_s': 0.0, 'link_down_s': 0.0, 'reallocations': 0,
#                'collision_events': 0, 'min_separation_m': None}
#     safety_violations = set()
#     last_elapsed = 0.0
#     last_dashboard = -10.0
#     last_warning = {}
#     logged_events = 0
#     pending_reports = []
#     rth_all = False
#     survey_limit = min(NUM_SURVEY_CELLS, max(1, len([a for a in agents.values() if a.ready]) // 3))
#     last_capacity_warning = -100.0
#     separation_violation = False

#     def event(message, elapsed):
#         events.append(f'T+{fmt_time(elapsed)} {message}')

#     def warn_once(key, message, elapsed):
#         if elapsed - last_warning.get(key, -100.0) >= 10.0:
#             event(message, elapsed)
#             last_warning[key] = elapsed

#     def eligible_surveys():
#         return [d for d in role_mgr.active_survey() if agents[d.id].ready
#                 and agents[d.id].position_fresh() and not agents[d.id].failure]

#     while True:
#         elapsed = time.monotonic() - mission_start
#         dt = max(0.0, elapsed - last_elapsed)
#         last_elapsed = elapsed
#         announced = poi_sim.newly_announced()
#         for poi in announced:
#             event(f'NEW POI-{poi.id:02d} priority {poi.priority} at '
#                   f'({poi.x:.0f},{poi.y:.0f})', elapsed)

#         # Reassign a failed/disconnected vehicle before giving any new instructions.
#         for d in list(role_mgr.drones.values()):
#             a = agents[d.id]
#             if d.role not in (Role.SURVEY, Role.RELAY):
#                 continue
#             if not a.ready or a.failure or (a.armed and not a.position_fresh()):
#                 old_role = d.role
#                 replacement = role_mgr.promote_replacement(d.id, agents)
#                 metrics['reallocations'] += 1
#                 event(f'drone {d.id} {old_role.value} unavailable; replacement '
#                       f'{replacement if replacement is not None else "pending"}', elapsed)

#         if not rth_all:
#             # A strip belongs to the fleet, not to a drone: a replacement resumes its index.
#             for d in role_mgr.active_survey():
#                 if d.assigned_cell is None or not survey_progress[d.assigned_cell]['complete']:
#                     continue
#                 next_cell = next((c for c, p in survey_progress.items()
#                                   if not p['complete'] and not any(
#                                       s.id != d.id and s.assigned_cell == c
#                                       for s in role_mgr.active_survey())), None)
#                 if next_cell is not None:
#                     event(f'drone {d.id} moves from complete strip {d.assigned_cell} '
#                           f'to strip {next_cell}', elapsed)
#                     d.assigned_cell = next_cell
#                     metrics['reallocations'] += 1
#                 else:
#                     d.role = Role.RTH if agents[d.id].armed else Role.GROUNDED
#             for drone_id, cell, waypoint in role_mgr.fill_survey(
#                     agents, survey_progress, survey_limit):
#                 event(f'drone {drone_id} assigned strip {cell}, resumes waypoint '
#                       f'{waypoint}/{len(survey_progress[cell]["wps"])}', elapsed)
#                 metrics['reallocations'] += 1

#         # Assign announced targets to the nearest available surveyor, highest priority first.
#         active = eligible_surveys()
#         poi_targets = {}
#         available = set(d.id for d in active)
#         for poi in sorted(poi_sim.active_pois(),
#                           key=lambda p: (-p.priority, p.spawn_time_s)):
#             if not available:
#                 break
#             drone_id = min(available,
#                            key=lambda i: math.dist(agents[i].position_world[:2],
#                                                    (poi.x, poi.y)))
#             poi_targets[drone_id] = poi
#             available.remove(drone_id)

#         survey_positions = {}
#         projected_positions = {}
#         for d in active:
#             a = agents[d.id]
#             progress = survey_progress[d.assigned_cell]
#             if progress['complete']:
#                 continue
#             if d.id in poi_targets:
#                 poi = poi_targets[d.id]
#                 tx, ty = poi.x, poi.y
#             else:
#                 tx, ty = progress['wps'][progress['idx']]
#                 if (a.in_air and math.dist(a.position_world[:2], (tx, ty))
#                         <= ARRIVAL_TOLERANCE_M):
#                     progress['idx'] += 1
#                     if progress['idx'] == len(progress['wps']):
#                         progress['complete'] = True
#                         event(f'strip {d.assigned_cell} sweep complete', elapsed)
#                         continue
#                     tx, ty = progress['wps'][progress['idx']]
#             a.set_world_target(tx, ty, CRUISE_ALTITUDE_M)
#             if a.armed:
#                 current = a.position_world
#                 survey_positions[d.id] = current
#                 distance = math.dist(current[:2], (tx, ty))
#                 lookahead = min(distance, 75.0)
#                 projected_positions[d.id] = (
#                     current[0] + (tx - current[0]) * lookahead / max(1.0, distance),
#                     current[1] + (ty - current[1]) * lookahead / max(1.0, distance),
#                     CRUISE_ALTITUDE_M)
#                 coverage.observe(*current[:2])

#         # Trade active survey capacity for relays when the geometric network needs it.
#         budget = len(role_mgr.active_relays()) + len(role_mgr.reserve_pool(agents))
#         relay_waypoints, required_relays = relay_targets(projected_positions, budget)
#         while required_relays > budget and len(survey_positions) > 1 and not rth_all:
#             victim_id = max(survey_positions,
#                             key=lambda i: (poi_targets.get(i, None) is None,
#                                            math.dist(GCS_POSITION[:2], survey_positions[i][:2])))
#             victim = role_mgr.drones[victim_id]
#             old_cell = victim.assigned_cell
#             victim.role = Role.RELAY
#             victim.assigned_cell = None
#             survey_limit = max(1, survey_limit - 1)
#             survey_positions.pop(victim_id)
#             projected_positions.pop(victim_id)
#             poi_targets.pop(victim_id, None)
#             metrics['reallocations'] += 1
#             event(f'reassign drone {victim_id} strip {old_cell} -> RELAY; '
#                   f'waypoint {survey_progress[old_cell]["idx"]} preserved', elapsed)
#             budget = len(role_mgr.active_relays()) + len(role_mgr.reserve_pool(agents))
#             relay_waypoints, required_relays = relay_targets(projected_positions, budget)

#         if required_relays > budget:
#             last_capacity_warning = elapsed
#             warn_once('relay_capacity',
#                       f'RELAY SHORTAGE {budget}/{required_relays}; survey holds at link edge', elapsed)
#         elif elapsed - last_capacity_warning > 45 and survey_limit < NUM_SURVEY_CELLS:
#             if len(role_mgr.reserve_pool(agents)) >= 2:
#                 survey_limit += 1
#                 last_capacity_warning = elapsed
#                 event(f'capacity recovered; survey slots now {survey_limit}', elapsed)

#         if not rth_all:
#             before = len(role_mgr.active_relays())
#             role_mgr.rebalance_relays(len(relay_waypoints), agents)
#             metrics['reallocations'] += len(role_mgr.active_relays()) - before
#         relay_assignment = match_relays_to_waypoints(
#             role_mgr.active_relays(), agents, relay_waypoints)
#         for d in role_mgr.active_relays():
#             a = agents[d.id]
#             if d.id in relay_assignment:
#                 a.set_world_target(*relay_assignment[d.id], CRUISE_ALTITUDE_M)
#             else:
#                 a.set_world_target(*a.position_world)

#         links, used_hop = connected_network(agents, role_mgr)
#         live_relays = [agents[d.id].position_world for d in role_mgr.active_relays()
#                        if links[d.id] and agents[d.id].position_fresh()]
#         def next_position(agent):
#             x, y, z = agent.position_world
#             tx, ty, _ = agent.target_world
#             distance = math.dist((x, y), (tx, ty))
#             step = min(8.0, distance)
#             return (x + (tx-x) * step / max(distance, 1.0),
#                     y + (ty-y) * step / max(distance, 1.0),
#                     CRUISE_ALTITUDE_M)
#         for d in role_mgr.active_survey():
#             a = agents[d.id]
#             a.pause_for_link = a.in_air and (
#                 not links[d.id] or not is_connected(
#                     next_position(a), GCS_POSITION, live_relays))
#         for d in role_mgr.active_relays():
#             a = agents[d.id]
#             upstream = [pos for pos in live_relays
#                         if pos is not a.position_world and pos != a.position_world]
#             a.pause_for_link = a.in_air and links[d.id] and not is_connected(
#                 next_position(a), GCS_POSITION, upstream)
#         active_airborne = [d for d in role_mgr.active_survey() if agents[d.id].in_air]
#         if active_airborne:
#             if all(links[d.id] for d in active_airborne):
#                 metrics['link_ok_s'] += dt
#             else:
#                 metrics['link_down_s'] += dt

#         # Turn over drones early enough for their own travel distance and landing reserve.
#         for d in list(role_mgr.drones.values()):
#             a = agents[d.id]
#             if d.role not in (Role.SURVEY, Role.RELAY) or not a.armed:
#                 continue
#             return_s = (math.dist(a.position_world[:2], (a.spawn_x, a.spawn_y)) /
#                         COMMANDED_SPEED_MPS + abs(a.position_world[2]) / 1.5 +
#                         RTH_RESERVE_S)
#             if (a.flight_time() + return_s >= UAV_MAX_FLIGHT_TIME_S or
#                     elapsed + return_s >= MISSION_DURATION_S or
#                     a.battery_valid and a.battery_pct <= 20):
#                 old_role, old_cell = d.role, d.assigned_cell
#                 replacement = role_mgr.promote_replacement(d.id, agents) if not rth_all else None
#                 metrics['reallocations'] += 1
#                 event(f'drone {d.id} {old_role.value} RTH for time/battery; '
#                       f'cell {old_cell}, replacement '
#                       f'{replacement if replacement is not None else "pending"}', elapsed)

#         for poi in poi_sim.check_detections({
#                 d.id: agents[d.id].position_world[:2] for d in role_mgr.active_survey()
#                 if agents[d.id].in_air and agents[d.id].position_fresh()}):
#             event(f'POI-{poi.id:02d} priority {poi.priority} detected by drone '
#                   f'{poi.detected_by} ({poi.x:.0f},{poi.y:.0f})', elapsed)
#             pending_reports.append(poi)
#         still_pending = []
#         links, used_hop = connected_network(agents, role_mgr)
#         for poi in pending_reports:
#             if links.get(poi.detected_by, False):
#                 poi_sim.mark_reported(poi)
#                 latency = poi.reported_at_s - poi.detected_at_s
#                 event(f'POI-{poi.id:02d} reported to GCS in {latency:.1f}s '
#                       f'[{'OK' if latency <= MAX_POI_REPORT_LATENCY_S else 'LATE'}]', elapsed)
#             else:
#                 still_pending.append(poi)
#         pending_reports = still_pending

#         all_strips_done = all(p['complete'] for p in survey_progress.values())
#         all_pois_reported = all(p.reported_at_s > 0 for p in poi_sim.pois)
#         if not rth_all and (elapsed >= MISSION_DURATION_S - RTH_RESERVE_S or
#                             all_strips_done and all_pois_reported):
#             rth_all = True
#             event('area/POI task complete or mission window closing; all vehicles RTH', elapsed)
#             for d in role_mgr.drones.values():
#                 if d.role != Role.GROUNDED:
#                     d.role = Role.RTH if agents[d.id].armed else Role.GROUNDED

#         for d in role_mgr.drones.values():
#             if d.role != Role.RTH:
#                 continue
#             a = agents[d.id]
#             x, y = a.position_world[:2]
#             near_home = math.dist((x, y), (a.spawn_x, a.spawn_y)) <= 5
#             a.set_world_target(a.spawn_x, a.spawn_y,
#                                8 if near_home else CRUISE_ALTITUDE_M)
#             a.pause_for_link = False
#             if a.landed:
#                 d.role = Role.GROUNDED

#         min_sep = minimum_separation(agents)
#         if math.isfinite(min_sep):
#             best = metrics['min_separation_m']
#             metrics['min_separation_m'] = min(best, min_sep) if best is not None else min_sep
#         if min_sep < MIN_SEPARATION_M:
#             if not separation_violation:
#                 metrics['collision_events'] += 1
#             separation_violation = True
#             warn_once('separation', f'SEPARATION VIOLATION {min_sep:.1f}m', elapsed)
#         else:
#             separation_violation = False
#         for d in role_mgr.drones.values():
#             a = agents[d.id]
#             if not a.position_fresh() or not a.in_air:
#                 continue
#             if a.speed_mps > MAX_SPEED_MPS + 0.1:
#                 warn_once(f'speed{d.id}', f'drone {d.id} SPEED {a.speed_mps:.1f}m/s', elapsed)
#                 safety_violations.add(('speed', d.id))
#             if not inside_geofence(a.position_world):
#                 warn_once(f'fence{d.id}', f'drone {d.id} OUTSIDE GEOFENCE', elapsed)
#                 safety_violations.add(('fence', d.id))
#                 d.role = Role.RTH
#             if a.flight_time() > UAV_MAX_FLIGHT_TIME_S:
#                 warn_once(f'time{d.id}', f'drone {d.id} FLIGHT LIMIT EXCEEDED', elapsed)
#                 safety_violations.add(('flight_time', d.id))
#             if a.battery_valid and a.battery_pct <= 0:
#                 safety_violations.add(('battery_empty', d.id))

#         viewer.update_active([d.id for d in role_mgr.active_survey()])
#         for d in role_mgr.drones.values():
#             a = agents[d.id]
#             x, y, z = a.position_world
#             tlog.log({
#                 'mission_elapsed_s': round(elapsed, 2), 'drone_id': d.id,
#                 'role': d.role.value, 'cell': d.assigned_cell,
#                 'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2),
#                 'position_fresh': a.position_fresh(), 'speed_mps': round(a.speed_mps, 2),
#                 'battery_pct': round(a.battery_pct, 1) if a.battery_valid else None,
#                 'raw_px4_battery_pct': round(a.raw_battery_pct, 1)
#                     if a.raw_battery_pct is not None else None,
#                 'battery_source': a.battery_source, 'sortie': a.sorties,
#                 'flight_time_s': round(a.flight_time(), 1), 'link_up': links[d.id],
#                 'connected': a.connected, 'ready': a.ready,
#                 'connection_retries': a.connection_retries,
#                 'armed': a.armed, 'in_air': a.in_air,
#                 'relay_count': len(role_mgr.active_relays()),
#                 'required_relays': required_relays,
#                 'used_relay_hop_m': round(used_hop, 2),
#                 'fleet_min_separation_m':
#                     round(min_sep, 2) if math.isfinite(min_sep) else None,
#                 'coverage_pct': round(coverage.fraction() * 100, 2),
#                 'survey_progress': {str(cell): p['idx'] for cell, p in survey_progress.items()},
#                 'poi_detected': sum(p.detected for p in poi_sim.pois),
#                 'poi_reported': sum(p.reported_at_s > 0 for p in poi_sim.pois),
#                 'network_uptime_s': round(metrics['link_ok_s'], 2),
#                 'network_downtime_s': round(metrics['link_down_s'], 2),
#                 'relay_reallocations': metrics['reallocations'],
#                 'priority_score': sum(p.priority for p in poi_sim.pois
#                                       if p.reported_at_s > 0),
#                 'worst_link_recovery_s': max(
#                     (delay for drone in agents.values()
#                      for delay in drone.recovery_durations), default=0.0),
#                 'camera_status': viewer.status(d.id), 'error': a.failure,
#             })
#         if elapsed - last_dashboard >= 2:
#             render_dashboard(agents, role_mgr, poi_sim, elapsed, links, used_hop,
#                              required_relays, viewer, events, min_sep, tlog,
#                              coverage, survey_progress, metrics)
#             last_dashboard = elapsed
#         for message in events[logged_events:]:
#             tlog.log({'mission_elapsed_s': round(elapsed, 2), 'event': message})
#         logged_events = len(events)
#         if rth_all and (all(not a.armed for a in agents.values()) or
#                         elapsed >= MISSION_DURATION_S + LANDING_TIMEOUT_S):
#             break
#         await asyncio.sleep(COORD_LOOP_PERIOD_S)

#     timely = all(p.reported_at_s > 0 and
#                  p.reported_at_s - p.detected_at_s <= MAX_POI_REPORT_LATENCY_S
#                  for p in poi_sim.pois)
#     success = (all(not a.armed for a in agents.values()) and timely and
#                all(p['complete'] for p in survey_progress.values()) and
#                coverage.fraction() >= 0.99 and elapsed <= MISSION_DURATION_S and
#                all(a.flight_time() <= UAV_MAX_FLIGHT_TIME_S for a in agents.values()) and
#                metrics['collision_events'] == 0 and not safety_violations and
#                metrics['link_down_s'] <= COORD_LOOP_PERIOD_S)
#     tlog.log({'mission_elapsed_s': round(elapsed, 2), 'summary': {
#         'success': success, 'poi_reported': sum(p.reported_at_s > 0 for p in poi_sim.pois),
#         'coverage_pct': round(coverage.fraction() * 100, 2),
#         'completed_strips': sum(p['complete'] for p in survey_progress.values()),
#         'network_uptime_s': round(metrics['link_ok_s'], 2),
#         'network_downtime_s': round(metrics['link_down_s'], 2),
#         'reallocations': metrics['reallocations'],
#         'priority_weighted_score': (
#             sum(p.priority for p in poi_sim.pois if p.reported_at_s > 0) /
#             sum(p.priority for p in poi_sim.pois)),
#         'worst_link_recovery_s': max(
#             (delay for a in agents.values() for delay in a.recovery_durations),
#             default=0.0),
#         'separation_events': metrics['collision_events'],
#         'collision_count': None,
#         'safety_violations': sorted(f'{name}:{drone_id}'
#                                     for name, drone_id in safety_violations),
#         'minimum_separation_m': metrics['min_separation_m'],
#         'packet_delivery_ratio': None,  # No packet-level simulator is connected.
#     }})
#     return success


# async def main():
#     if NUM_DRONES < MIN_READY_DRONES or GROUND_SPAWN_SPACING_M < MIN_SEPARATION_M:
#         raise ValueError('Need at least 12 drones and >=20m launch spacing')
#     agents = {i: DroneAgent(i, grid_spawn_position(i, NUM_DRONES, GROUND_SPAWN_SPACING_M))
#               for i in range(NUM_DRONES)}
#     for agent in agents.values():
#         agent.peers = agents
#     role_mgr = RoleManager(NUM_DRONES, NUM_SURVEY_CELLS)
#     poi_sim = POISimulator(seed=SCENARIO_SEED)
#     tlog = TelemetryLogger()
#     viewer = CameraViewer(NUM_DRONES)
#     events = []
#     print('Mission: 10 random timed POIs | 1000x1000m area 75m from GCS | 45min window', flush=True)
#     print('Constraints: <=20min/UAV, <=5m/s, <=100m altitude/hop, >=20m separation, <=10s POI report', flush=True)
#     print('Timing assumption: PX4/Gazebo runs at 1x; mission elapsed uses monotonic wall time.', flush=True)
#     print(f'Scenario seed: {SCENARIO_SEED} | minimum launch-ready: {MIN_READY_DRONES}/{NUM_DRONES}', flush=True)
#     print(f'Planned sweep: {planned_survey_distance_m()/1000:.1f} km across four strips; '
#           f'{planned_survey_distance_m()/COMMANDED_SPEED_MPS/60:.0f} '
#           f'survey-drone minutes at {COMMANDED_SPEED_MPS:.0f}m/s, '
#           'plus transit, relays and returns.', flush=True)
#     print(f'Allowing {STARTUP_SETTLE_S:.0f}s connection settle; '
#           f'launch when at least {MIN_READY_DRONES} are ready, '
#           f'abort threshold after {CONNECT_TIMEOUT_S:.0f}s.', flush=True)
#     connect_tasks = [asyncio.create_task(a.connect(events)) for a in agents.values()]
#     connection_start = time.monotonic()
#     last_report = -10
#     while True:
#         now = time.monotonic()
#         ready = [a.id for a in agents.values() if a.ready]
#         if (now - connection_start >= STARTUP_SETTLE_S and
#                 len(ready) >= MIN_READY_DRONES):
#             break
#         if now - connection_start >= CONNECT_TIMEOUT_S:
#             break
#         if now - last_report >= 2:
#             pending = [a.id for a in agents.values() if not a.ready]
#             print(f'Connection readiness {len(ready)}/{NUM_DRONES}; '
#                   f'minimum {MIN_READY_DRONES}; retrying {pending}', flush=True)
#             last_report = now
#         await asyncio.sleep(0.5)
#     if len(ready) < MIN_READY_DRONES:
#         print(f'ABORT: only {len(ready)}/{MIN_READY_DRONES} ready after '
#               f'{CONNECT_TIMEOUT_S:.0f}s; no drone was armed.', flush=True)
#         for task in connect_tasks:
#             task.cancel()
#         await asyncio.gather(*connect_tasks, return_exceptions=True)
#         tlog.close()
#         return 1
#     pending = [a.id for a in agents.values() if not a.ready]
#     for drone_id in pending:
#         agents[drone_id].link_lost_at = time.monotonic()
#     print(f'Mission starts with {len(ready)}/{NUM_DRONES} ready. '
#           f'Background retries continue for {pending}.', flush=True)
#     viewer.start()
#     flight_tasks = [asyncio.create_task(run_agent(a, role_mgr, events)) for a in agents.values()]
#     try:
#         success = await coordination_loop(agents, role_mgr, poi_sim, tlog, viewer, events)
#         print('Mission result:', 'COMPLETE' if success else 'INCOMPLETE; inspect live warnings and log', flush=True)
#         return 0 if success else 1
#     finally:
#         for agent in agents.values():
#             if agent.armed and not agent.landing:
#                 with contextlib.suppress(Exception):
#                     await agent.land()
#         deadline = time.monotonic() + 30.0
#         while any(a.in_air for a in agents.values()) and time.monotonic() < deadline:
#             await asyncio.sleep(0.5)
#         for agent in agents.values():
#             agent.request_stop()
#         for task in flight_tasks:
#             task.cancel()
#         await asyncio.gather(*flight_tasks, return_exceptions=True)
#         for task in connect_tasks:
#             task.cancel()
#         await asyncio.gather(*connect_tasks, return_exceptions=True)
#         viewer.stop()
#         tlog.close()


# if __name__ == '__main__':
#     try:
#         raise SystemExit(asyncio.run(main()))
#     except KeyboardInterrupt:
#         print('\nMission interrupted; landing requested.')





#!/usr/bin/env python3
"""
swarm_mission.py

15-drone BVLOS swarm mission with:
  - MAVSDK control / telemetry
  - role assignment: SURVEY / RELAY / RESERVE / RTH
  - 100 m maximum communication-hop constraint
  - live POI detection + reporting latency
  - live terminal dashboard for judges
  - JSONL telemetry logging
  - headless or GUI Gazebo with the same terminal console
  - Gazebo Transport RGB-D viewer for active survey drones

The camera viewer is independent of the Gazebo GUI. POI detections remain
synthetic proximity checks, not camera object recognition. Camera attribution
requires a model-scoped Gazebo image topic. No ROS 2 bridge is required.
"""

import asyncio
import contextlib
import json
import math
import os
import time
from dataclasses import dataclass
from enum import Enum

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.action import ActionError

from swarm_console import GazeboCameraViewer


# ============================================================
# MISSION CONFIGURATION
# ============================================================

GCS_POSITION = (0.0, 0.0, 0.0)

AREA_OFFSET_M = 75.0
AREA_WIDTH_M = 1000.0
AREA_HEIGHT_M = 1000.0
AREA_X_MIN = GCS_POSITION[0] + AREA_OFFSET_M
AREA_X_MAX = AREA_X_MIN + AREA_WIDTH_M
AREA_Y_MIN = GCS_POSITION[1] - AREA_HEIGHT_M / 2.0
AREA_Y_MAX = GCS_POSITION[1] + AREA_HEIGHT_M / 2.0

MISSION_DURATION_S = 45 * 60
UAV_MAX_FLIGHT_TIME_S = 20 * 60
BATTERY_SWAP_S = float(os.getenv("SWARM_BATTERY_SWAP_S", "120"))
RTH_RESERVE_S = 90.0
LANDING_TIMEOUT_S = 120.0
CONNECT_TIMEOUT_S = 180.0
CONNECT_ATTEMPT_S = 20.0
CONNECT_RETRY_S = 5.0
STARTUP_SETTLE_S = 30.0
MIN_READY_DRONES = 12
MAX_POI_REPORT_LATENCY_S = 10.0

# Communication requirement: no network hop may exceed 100 m.
MAX_COMM_RANGE_M = 100.0
RELAY_SAFETY_MARGIN_M = 0.6
RELAY_HOP_SPACING_M = MAX_COMM_RANGE_M - RELAY_SAFETY_MARGIN_M
FIRST_RELAY_DISTANCE_M = 95.0

MAX_ALTITUDE_M = 100.0
MAX_SPEED_MPS = 5.0
MIN_SEPARATION_M = 20.0
CRUISE_ALTITUDE_M = 20.0
COMMANDED_SPEED_MPS = 4.0

NUM_POIS = 10
POI_DETECTION_RADIUS_M = 15.0
SURVEY_LANE_SPACING_M = 28.0
SCENARIO_SEED = int(os.getenv("SWARM_SCENARIO_SEED", "42"))

NUM_DRONES = int(os.getenv("SWARM_NUM_DRONES", "15"))
GROUND_SPAWN_SPACING_M = float(os.getenv("SWARM_SPACING_M", "25"))
NUM_SURVEY_CELLS = 4

BASE_MAVLINK_PORT = 14540
EXTRA_MAVLINK_PORT_BASE = int(os.getenv("SWARM_EXTRA_PORT_BASE", "14640"))
BASE_GRPC_PORT = 50051
SETPOINT_PERIOD_S = 0.1
ARRIVAL_TOLERANCE_M = 5.0
COORD_LOOP_PERIOD_S = 1.0
POSITION_STALE_S = 5.0


def mavlink_port(drone_id: int) -> int:
    return (BASE_MAVLINK_PORT + drone_id if drone_id < 10
            else EXTRA_MAVLINK_PORT_BASE + drone_id)

# ============================================================
# COORDINATE CONVENTION
# ============================================================

def grid_spawn_position(i: int, n: int, spacing: float) -> tuple[float, float]:
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    col = i % cols
    row = i // cols
    x = (col - (cols - 1) / 2.0) * spacing
    y = (row - (rows - 1) / 2.0) * spacing
    return x, y


def world_to_local_ned(world_x, world_y, world_z, spawn_x, spawn_y):
    delta_x = world_x - spawn_x
    delta_y = world_y - spawn_y
    target_n = delta_y
    target_e = delta_x
    target_d = -world_z
    return target_n, target_e, target_d


def local_ned_to_world(n, e, d, spawn_x, spawn_y):
    world_x = spawn_x + e
    world_y = spawn_y + n
    world_z = -d
    return world_x, world_y, world_z


def normalize_battery_pct(value: float) -> float:
    return value * 100.0 if value <= 1.0 else value


# ============================================================
# ROLES
# ============================================================

class Role(str, Enum):
    STANDBY = "STANDBY"
    SURVEY = "SURVEY"
    RELAY = "RELAY"
    RTH = "RTH"
    GROUNDED = "GROUNDED"


@dataclass
class DroneState:
    id: int
    role: Role = Role.STANDBY
    assigned_cell: int | None = None


class RoleManager:
    def __init__(self, num_drones: int, num_survey_cells: int):
        self.num_survey_cells = num_survey_cells
        self.drones: dict[int, DroneState] = {i: DroneState(id=i) for i in range(num_drones)}

    def reserve_pool(self, agents):
        return [d for d in self.drones.values()
                if d.role == Role.STANDBY and agents[d.id].ready
                and agents[d.id].connected and agents[d.id].position_fresh()
                and not agents[d.id].failure
                and not agents[d.id].landed and not agents[d.id].armed
                and (not agents[d.id].battery_valid or agents[d.id].battery_pct > 30)]

    def fill_survey(self, agents, survey_progress, limit):
        assigned = []
        for cell in range(self.num_survey_cells):
            if len(self.active_survey()) >= limit:
                break
            if survey_progress[cell]['complete'] or any(
                    d.role == Role.SURVEY and d.assigned_cell == cell
                    for d in self.drones.values()):
                continue
            pool = self.reserve_pool(agents)
            if not pool:
                break
            chosen = pool[0]
            chosen.role = Role.SURVEY
            chosen.assigned_cell = cell
            assigned.append((chosen.id, cell, survey_progress[cell]['idx']))
        return assigned

    def active_relays(self):
        return [d for d in self.drones.values() if d.role == Role.RELAY]

    def active_survey(self):
        return [d for d in self.drones.values() if d.role == Role.SURVEY]

    def rebalance_relays(self, required_relay_count: int, agents):
        current = self.active_relays()
        if len(current) < required_relay_count:
            for drone in self.reserve_pool(agents)[:required_relay_count - len(current)]:
                drone.role = Role.RELAY
        # Keep already deployed relays in flight; they land during RTH.

    def promote_replacement(self, failed_drone_id: int, agents):
        failed = self.drones[failed_drone_id]
        failed_role, failed_cell = failed.role, failed.assigned_cell
        old_agent = agents[failed_drone_id]
        failed.role = (Role.RTH if old_agent.armed else
                       Role.GROUNDED if old_agent.failure or old_agent.landed
                       else Role.STANDBY)
        failed.assigned_cell = None
        reserve = [d for d in self.reserve_pool(agents) if d.id != failed_drone_id]
        if reserve and failed_role in (Role.SURVEY, Role.RELAY):
            replacement = reserve[0]
            replacement.role = failed_role
            replacement.assigned_cell = failed_cell
            return replacement.id
        return None


# ============================================================
# SURVEY PATTERN
# ============================================================

def cell_bounds(cell_index: int, num_cells: int):
    strip_width = AREA_WIDTH_M / num_cells
    x_min = AREA_X_MIN + cell_index * strip_width
    x_max = x_min + strip_width
    return x_min, x_max, AREA_Y_MIN, AREA_Y_MAX


def lawnmower_waypoints(cell_index: int, num_cells: int, lane_spacing: float = 40.0):
    x_min, x_max, y_min, y_max = cell_bounds(cell_index, num_cells)
    waypoints = []
    x = x_min + lane_spacing / 2
    going_up = True
    lanes = []
    while x <= x_max - lane_spacing / 2:
        lanes.append(x)
        x += lane_spacing
    if not lanes or x_max - lanes[-1] > POI_DETECTION_RADIUS_M:
        lanes.append(x_max - POI_DETECTION_RADIUS_M)
    for x in lanes:
        if going_up:
            waypoints += [(x, y_min), (x, y_max)]
        else:
            waypoints += [(x, y_max), (x, y_min)]
        going_up = not going_up
    return waypoints


def planned_survey_distance_m():
    total = 0.0
    for cell in range(NUM_SURVEY_CELLS):
        wps = lawnmower_waypoints(cell, NUM_SURVEY_CELLS, SURVEY_LANE_SPACING_M)
        total += sum(math.dist(a, b) for a, b in zip(wps, wps[1:]))
    return total


# ============================================================
# POI SIMULATION
# ============================================================

@dataclass
class POI:
    id: int
    x: float
    y: float
    spawn_time_s: float
    priority: int = 1
    announced: bool = False
    detected: bool = False
    detected_by: int | None = None
    detected_at_s: float = 0.0
    reported_at_s: float = 0.0


class POISimulator:
    def __init__(self, num_pois: int = NUM_POIS,
                 mission_duration_s: float = MISSION_DURATION_S,
                 seed: int | None = None):
        import random
        rng = random.Random(seed)
        self.start_time: float | None = None
        self.now = time.monotonic
        self.pois: list[POI] = []
        for i in range(num_pois):
            x = rng.uniform(AREA_X_MIN, AREA_X_MAX)
            y = rng.uniform(AREA_Y_MIN, AREA_Y_MAX)
            spawn_t = rng.uniform(0, mission_duration_s * 0.8)
            priority = 3 if rng.random() < 0.25 else 1
            self.pois.append(POI(id=i, x=x, y=y, spawn_time_s=spawn_t,
                                 priority=priority))

    def elapsed(self) -> float:
        return self.now() - self.start_time if self.start_time is not None else 0.0

    def start(self):
        self.start_time = self.now()

    def active_pois(self):
        t = self.elapsed()
        return [p for p in self.pois if p.spawn_time_s <= t and not p.detected]

    def newly_announced(self):
        new = [p for p in self.active_pois() if not p.announced]
        for poi in new:
            poi.announced = True
        return new

    def check_detections(self, survey_positions: dict[int, tuple[float, float]]):
        newly_detected = []
        t = self.elapsed()
        for poi in self.active_pois():
            for drone_id, (dx, dy) in survey_positions.items():
                if math.dist((dx, dy), (poi.x, poi.y)) <= POI_DETECTION_RADIUS_M:
                    poi.detected = True
                    poi.detected_by = drone_id
                    poi.detected_at_s = t
                    newly_detected.append(poi)
                    break
        return newly_detected

    def mark_reported(self, poi: POI):
        poi.reported_at_s = self.elapsed()

    def summary(self) -> str:
        found = sum(1 for p in self.pois if p.detected)
        reported = sum(1 for p in self.pois if p.reported_at_s > 0)
        return f"{found}/{len(self.pois)} found, {reported}/{len(self.pois)} reported"

    def worst_latency(self) -> float:
        values = [p.reported_at_s - p.detected_at_s for p in self.pois if p.reported_at_s > 0]
        return max(values) if values else 0.0


# ============================================================
# RELAY / NETWORK
# ============================================================

def plan_relay_chain(frontier_point, gcs=GCS_POSITION[:2], hop_spacing=RELAY_HOP_SPACING_M):
    gx, gy = gcs
    fx, fy = frontier_point
    total_dist = math.dist((gx, gy), (fx, fy))
    # The first link climbs from the ground station to flight altitude.
    first_link_horizontal = math.sqrt(MAX_COMM_RANGE_M ** 2 - CRUISE_ALTITUDE_M ** 2)
    if total_dist <= first_link_horizontal:
        return []
    distances = [FIRST_RELAY_DISTANCE_M]
    while total_dist - distances[-1] > hop_spacing:
        distances.append(distances[-1] + hop_spacing)
    return [(gx + distance / total_dist * (fx - gx),
             gy + distance / total_dist * (fy - gy))
            for distance in distances]


def is_connected(point, gcs, relay_positions, max_range=MAX_COMM_RANGE_M):
    nodes = [gcs] + list(relay_positions) + [point]
    n = len(nodes)
    visited = [False] * n
    visited[0] = True
    frontier = [0]
    while frontier:
        next_frontier = []
        for i in frontier:
            for j in range(n):
                if not visited[j] and math.dist(nodes[i], nodes[j]) <= max_range:
                    visited[j] = True
                    next_frontier.append(j)
        frontier = next_frontier
    return visited[n - 1]


def network_hop_distances(relay_positions, survey_positions, gcs=GCS_POSITION[:2]):
    """Return the physical links used by the linear GCS->relay->survey chain."""
    points = [gcs] + list(relay_positions)
    hops = []
    for a, b in zip(points, points[1:]):
        hops.append(math.dist(a, b))
    if relay_positions:
        last_relay = relay_positions[-1]
    else:
        last_relay = gcs
    for pos in survey_positions.values():
        hops.append(math.dist(last_relay, pos))
    return hops


def min_pairwise_distance(positions: list[tuple[float, float]]) -> float:
    if len(positions) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            best = min(best, math.dist(positions[i], positions[j]))
    return best


def max_pairwise_distance(positions: list[tuple[float, float]]) -> float:
    if len(positions) < 2:
        return 0.0
    best = 0.0
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            best = max(best, math.dist(positions[i], positions[j]))
    return best


# ============================================================
# TELEMETRY LOGGING
# ============================================================

class TelemetryLogger:
    def __init__(self, log_dir: str | None = None):
        if log_dir is None:
            log_dir = os.getenv("SWARM_LOG_DIR", "logs")
        os.makedirs(log_dir, exist_ok=True)
        fname = f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.path = os.path.join(log_dir, fname)
        self._f = open(self.path, "a", buffering=1)

    def log(self, record: dict):
        self._f.write(json.dumps(record) + "\n")

    def close(self):
        if not self._f.closed:
            self._f.close()


# ============================================================
# MISSION TELEMETRY
# ============================================================

class CoverageTracker:
    """Synthetic proximity coverage of 20 m sample cells in the 1 km arena."""
    def __init__(self, resolution_m=20.0):
        self.resolution_m = resolution_m
        self.columns = math.ceil(AREA_WIDTH_M / resolution_m)
        self.rows = math.ceil(AREA_HEIGHT_M / resolution_m)
        self.visited = set()

    def observe(self, x, y):
        col = int((x - AREA_X_MIN) / self.resolution_m)
        row = int((y - AREA_Y_MIN) / self.resolution_m)
        for cx in range(max(0, col - 1), min(self.columns, col + 2)):
            for cy in range(max(0, row - 1), min(self.rows, row + 2)):
                px = AREA_X_MIN + (cx + 0.5) * self.resolution_m
                py = AREA_Y_MIN + (cy + 0.5) * self.resolution_m
                if math.dist((x, y), (px, py)) <= POI_DETECTION_RADIUS_M:
                    self.visited.add((cx, cy))

    def fraction(self):
        return len(self.visited) / (self.columns * self.rows)


def fmt_time(seconds):
    minutes, secs = divmod(max(0, int(seconds)), 60)
    return f"{minutes:02d}:{secs:02d}"


def connected_network(agents, role_mgr):
    """Geometric range model over current airborne, fresh telemetry positions."""
    active = {
        d.id: agents[d.id].position_world
        for d in role_mgr.drones.values()
        if agents[d.id].armed and agents[d.id].connected
        and agents[d.id].position_fresh()
    }
    relay_ids = {d.id for d in role_mgr.active_relays()} & active.keys()
    reached = set()
    frontier = [GCS_POSITION]
    used_links = []
    while frontier:
        origin = frontier.pop(0)
        for drone_id in relay_ids - reached:
            pos = active[drone_id]
            distance = math.dist(origin, pos)
            if distance <= MAX_COMM_RANGE_M:
                reached.add(drone_id)
                frontier.append(pos)
                used_links.append(distance)
    link = {}
    for drone_id, pos in active.items():
        link[drone_id] = drone_id in reached or any(
            math.dist(pos, src) <= MAX_COMM_RANGE_M
            for src in [GCS_POSITION] + [active[i] for i in reached]
        )
    for d in role_mgr.drones.values():
        link.setdefault(d.id, False)
    return link, max(used_links, default=0.0)


def relay_targets(survey_positions, budget):
    """Merge 90 m radial relay chains; excess capacity remains visibly unmet."""
    result = []
    for pos in survey_positions.values():
        for point in plan_relay_chain(pos[:2]):
            if all(math.dist(point, existing) >= 45.0 for existing in result):
                result.append(point)
    return result[:budget], len(result)


def match_relays_to_waypoints(relays, agents, waypoints):
    """Greedy minimum-distance matching to avoid swapping an established chain."""
    pairs = sorted(
        (math.dist(agents[d.id].position_world[:2], waypoint), d.id, index)
        for d in relays for index, waypoint in enumerate(waypoints))
    assigned = {}
    occupied = set()
    for _, drone_id, index in pairs:
        if drone_id not in assigned and index not in occupied:
            assigned[drone_id] = waypoints[index]
            occupied.add(index)
    return assigned


def minimum_separation(agents):
    airborne = [a.position_world for a in agents.values() if a.in_air and a.position_fresh()]
    return min_pairwise_distance(airborne)


def render_dashboard(agents, role_mgr, poi_sim, elapsed, links, used_hop,
                     required_relays, viewer, events, min_sep, tlog,
                     coverage, survey_progress, metrics):
    alive = [a for a in agents.values() if a.connected and a.position_fresh()]
    armed = [a for a in agents.values() if a.armed]
    max_speed = max((a.speed_mps for a in alive if a.in_air), default=0.0)
    max_alt = max((a.position_world[2] for a in alive if a.in_air), default=0.0)
    longest = max((a.flight_time() for a in agents.values()), default=0.0)
    fresh_batteries = [a.battery_pct for a in alive if a.battery_valid]
    min_battery = min(fresh_batteries) if fresh_batteries else None
    outside = sum(1 for a in alive if a.in_air and not inside_geofence(a.position_world))
    network_ok = all(links[d.id] for d in role_mgr.active_survey() if agents[d.id].armed)
    network_ok &= len(role_mgr.active_relays()) >= required_relays
    over_latency = sum(1 for p in poi_sim.pois if p.detected and p.reported_at_s == 0
                       and poi_sim.elapsed() - p.detected_at_s > MAX_POI_REPORT_LATENCY_S)
    late = sum(1 for p in poi_sim.pois if p.reported_at_s > 0 and
               p.reported_at_s - p.detected_at_s > MAX_POI_REPORT_LATENCY_S)
    sep_text = 'N/A' if math.isinf(min_sep) else f'{min_sep:.1f}'
    batt_text = 'N/A' if min_battery is None else f'{min_battery:.0f}%'
    print(f"\nUAV-X | MISSION {viewer.clock_source} T+{fmt_time(elapsed)}/{fmt_time(MISSION_DURATION_S)} "
          f"({fmt_time(MISSION_DURATION_S-elapsed)} remaining) | "
          f"{poi_sim.summary()} | log {tlog.path}", flush=True)
    if viewer.clock_source == 'SIM':
        clock_age = time.monotonic() - viewer.sim_updated_at
        wall_elapsed = max(0.001, time.monotonic()-metrics['wall_start'])
        print(f"Gazebo simulation clock: {viewer.sim_time():.1f}s | "
              f"last clock update {clock_age:.1f}s ago | "
              f"wall elapsed {fmt_time(wall_elapsed)} | average RTF {elapsed/wall_elapsed:.2f}x")
    else:
        print('Gazebo clock unavailable; 45-minute timing is a wall-time fallback.')
    ready = [a.id for a in agents.values() if a.ready]
    pending = [a.id for a in agents.values() if not a.ready and not a.landed]
    print(f"Fleet {len(alive)}/{len(agents)} telemetry | ready {len(ready)} | pending {pending} "
          f"| armed {len(armed)} | survey {len(role_mgr.active_survey())} "
          f"| relay {len(role_mgr.active_relays())}/{required_relays} required "
          f"| network {'OK' if network_ok else 'DEGRADED'} | longest used relay hop {used_hop:.1f}/{MAX_COMM_RANGE_M:.0f}m")
    strips = ' '.join(f'{cell}:{progress["idx"]}/{len(progress["wps"])}'
                      for cell, progress in survey_progress.items())
    link_sample_s = metrics['link_ok_s'] + metrics['link_down_s']
    availability = metrics['link_ok_s'] / link_sample_s * 100 if link_sample_s else 0.0
    print(f"Coverage (synthetic 20m grid): {coverage.fraction()*100:.1f}% | "
          f"strip waypoints {strips} | connectivity availability "
          f"{availability:.1f}% | downtime "
          f"{metrics['link_down_s']:.0f}s | role reallocations {metrics['reallocations']}")
    print(f"Battery swaps modeled: {sum(max(0, a.sorties-1) for a in agents.values())} "
          f"({BATTERY_SWAP_S:.0f}s ground service) | PDR: N/A (no packet model) "
          f"| separation breaches: {metrics['collision_events']}")
    priority_total = sum(p.priority for p in poi_sim.pois)
    priority_reported = sum(p.priority for p in poi_sim.pois if p.reported_at_s > 0)
    recovery = [delay for a in agents.values() for delay in a.recovery_durations]
    print(f"Priority score {priority_reported}/{priority_total} | "
          f"worst link recovery {max(recovery, default=0.0):.1f}s | "
          f"actual collision count: N/A (no collision sensor)")
    print(f"Limits: flight {fmt_time(longest)}/{fmt_time(UAV_MAX_FLIGHT_TIME_S)} | "
          f"speed {max_speed:.1f}/{MAX_SPEED_MPS:.0f}m/s | altitude {max_alt:.1f}/{MAX_ALTITUDE_M:.0f}m | "
          f"separation {sep_text}/{MIN_SEPARATION_M:.0f}m | outside fence {outside} | min battery {batt_text}")
    print(f"POI: detected/reported {poi_sim.summary()} | worst reporting delay "
          f"{poi_sim.worst_latency():.1f}/{MAX_POI_REPORT_LATENCY_S:.0f}s | late {late} | overdue {over_latency} "
          f"| camera {'OFF' if viewer.error else 'ON'} ({len(viewer.discovered_topics)} topics)")
    print('Gazebo world frame: X=East Y=North Z=Up, metres; GCS=(0,0,0). '
          'A * marks stale position.')
    print('ID  ROLE       X      Y     Z  SPEED  BATT  FLIGHT  LINK  CAMERA  STATE/RETRIES')
    for d in role_mgr.drones.values():
        a = agents[d.id]
        x, y, z = a.position_world
        state = ('LAND' if a.landing else 'PAUSED' if a.pause_for_link
                 else 'AIR' if a.in_air else 'READY' if a.ready else 'WAIT')
        if a.failure:
            state = 'ERROR'
        if not a.ready and not a.armed:
            state = f'RETRY({a.connection_retries})'
        elif a.landed and a.service_ready_at:
            state = f'SERVICE({max(0, int(a.service_ready_at-a.mission_now()))}s)'
        print(f'{d.id:02d}  {d.role.value:<9} {x:6.0f} {y:6.0f} {z:5.0f}'
              f'{" " if a.position_fresh() else "*"} '
              f'{a.speed_mps:6.1f} {a.battery_pct:5.0f}% {fmt_time(a.flight_time()):>7} '
              f'{"UP" if links[d.id] else "--":>5} {viewer.status(d.id) if d.role == Role.SURVEY else "--":>7} {state}')
    ids = sorted(agents)
    print('Inter-drone 3-D separation (m); lower triangle; -- = stale/no telemetry:')
    print(' ID ' + ' '.join(f'{i:>5}' for i in ids))
    pairs = []
    for row_id in ids:
        row = []
        for col_id in ids:
            if col_id >= row_id:
                row.append('    .')
            elif agents[row_id].position_fresh() and agents[col_id].position_fresh():
                distance = math.dist(agents[row_id].position_world,
                                     agents[col_id].position_world)
                row.append(f'{distance:5.0f}')
                pairs.append((distance, col_id, row_id))
            else:
                row.append('   --')
        print(f'{row_id:>3} ' + ' '.join(row))
    print('Closest pairs: ' + ', '.join(f'{i:02d}-{j:02d} {d:.1f}m'
                                      for d, i, j in sorted(pairs)[:5]))
    for event in events[-3:]:
        print('EVENT ' + event)
    if viewer.error:
        print('CAMERA ' + viewer.error)
    if viewer.camera_warning:
        print('CAMERA ' + viewer.camera_warning)
    if viewer.ambiguous_depth:
        print('CAMERA shared /depth_camera is ambiguous; provide a namespaced OakD-Lite model.')
    if viewer.clock_error:
        print('CLOCK ' + viewer.clock_error)


# ============================================================
# DRONE AGENT
# ============================================================

def inside_geofence(pos):
    x, y, z = pos
    return (-75 <= x <= AREA_X_MAX and AREA_Y_MIN <= y <= AREA_Y_MAX
            and -2 <= z <= MAX_ALTITUDE_M)


class DroneAgent:
    def __init__(self, drone_id, spawn_xy):
        self.id = drone_id
        self.spawn_x, self.spawn_y = spawn_xy
        self.system = None
        self.position_world = (self.spawn_x, self.spawn_y, 0.0)
        self.target_world = self.position_world
        self.last_position_at = 0.0
        self.battery_pct = 100.0
        self.raw_battery_pct = None
        self.battery_source = 'PX4'
        self.battery_valid = False
        self.speed_mps = 0.0
        self.armed = False
        self.connected = False
        self.ready = False
        self.ready_event = asyncio.Event()
        self.connection_error = ''
        self.connection_retries = 0
        self.link_lost_at = None
        self.recovery_durations = []
        self.in_air = False
        self.landing = False
        self.landed = False
        self.failure = ''
        self.flight_start = None
        self.flight_end = None
        self.sorties = 0
        self.service_ready_at = None
        self._stop = False
        self.peers = {}
        self.pause_for_link = False
        self.mission_now = time.monotonic

    async def connect(self, events):
        port = mavlink_port(self.id)
        async def wait_link():
            async for state in self.system.core.connection_state():
                self.connected = state.is_connected
                if state.is_connected:
                    return

        async def wait_health():
            async for health in self.system.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    return

        while not self._stop:
            try:
                if self.system is None:
                    self.system = System(port=BASE_GRPC_PORT + self.id)
                    try:
                        await asyncio.wait_for(
                            self.system.connect(system_address=f'udpin://127.0.0.1:{port}'),
                            timeout=CONNECT_ATTEMPT_S)
                    except Exception:
                        self.system = None
                        raise
                await asyncio.wait_for(wait_link(), timeout=CONNECT_ATTEMPT_S)
                await asyncio.wait_for(wait_health(), timeout=CONNECT_ATTEMPT_S)
                self.ready = True
                self.ready_event.set()
                self.connection_error = ''
                if self.link_lost_at is not None:
                    self.recovery_durations.append(time.monotonic() - self.link_lost_at)
                    self.link_lost_at = None
                events.append(f'drone {self.id} READY on UDP {port}')
                # Keep watching after readiness; a recovered vehicle returns to standby.
                async for state in self.system.core.connection_state():
                    if not state.is_connected:
                        self.connected = False
                        self.ready = False
                        self.link_lost_at = time.monotonic()
                        self.connection_error = 'MAVLink link lost'
                        events.append(f'drone {self.id} disconnected; retrying in background')
                        break
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.ready = False
                self.connection_error = f'{type(exc).__name__}: {exc}'
            if self._stop:
                break
            self.connection_retries += 1
            events.append(f'drone {self.id} connection retry {self.connection_retries}: '
                          f'{self.connection_error or "waiting for link/position"}')
            await asyncio.sleep(CONNECT_RETRY_S)

    def position_fresh(self):
        return bool(self.last_position_at and time.monotonic() - self.last_position_at < POSITION_STALE_S)

    async def arm_and_start_offboard(self):
        try:
            await self.system.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await self.system.action.arm()
            self.armed = True
            self.flight_start = self.mission_now()
            self.flight_end = None
            self.sorties += 1
            await self.system.offboard.start()
            return True
        except (ActionError, OffboardError) as exc:
            self.failure = f'arm/offboard: {exc}'
            if self.armed:
                with contextlib.suppress(Exception):
                    await self.system.action.disarm()
                self.flight_end = self.mission_now()
            self.armed = False
            return False

    def set_world_target(self, x, y, z):
        self.target_world = (min(AREA_X_MAX, max(-75.0, x)),
                             min(AREA_Y_MAX, max(AREA_Y_MIN, y)),
                             min(MAX_ALTITUDE_M, max(0.0, z)))

    async def stream_setpoints(self):
        while not self._stop:
            if self.armed and not self.landing:
                x, y, z = self.position_world
                tx, ty, tz = self.target_world
                dx, dy, dz = tx - x, ty - y, tz - z
                distance = math.hypot(dx, dy)
                speed = min(COMMANDED_SPEED_MPS, distance * 0.6)
                vx = speed * dx / distance if distance > 0.1 else 0.0
                vy = speed * dy / distance if distance > 0.1 else 0.0
                vz = max(-1.5, min(1.5, dz * 0.5))
                if self.pause_for_link and self.in_air:
                    vx = vy = 0.0
                # Stop motion into the 20 m safety bubble around a live vehicle.
                for other in self.peers.values():
                    if other.id == self.id or not other.in_air or not other.position_fresh():
                        continue
                    ox, oy, oz = other.position_world
                    current = math.dist((x, y, z), (ox, oy, oz))
                    future = math.dist((x + vx * 3, y + vy * 3, z + vz * 3),
                                       (ox, oy, oz))
                    if future < MIN_SEPARATION_M + 4 and future < current:
                        vx = vy = vz = 0.0
                        break
                try:
                    # World X maps to local East, world Y to local North.
                    await self.system.offboard.set_velocity_ned(VelocityNedYaw(
                        vy, vx, -vz, 0))
                except Exception as exc:
                    self.failure = f'setpoint: {exc}'
            await asyncio.sleep(SETPOINT_PERIOD_S)

    async def track_position(self):
        async for pv in self.system.telemetry.position_velocity_ned():
            if self._stop:
                break
            self.position_world = local_ned_to_world(
                pv.position.north_m, pv.position.east_m, pv.position.down_m,
                self.spawn_x, self.spawn_y)
            self.speed_mps = math.sqrt(pv.velocity.north_m_s ** 2 +
                                       pv.velocity.east_m_s ** 2 + pv.velocity.down_m_s ** 2)
            self.last_position_at = time.monotonic()

    async def track_battery(self):
        async for batt in self.system.telemetry.battery():
            if self._stop:
                break
            self.raw_battery_pct = normalize_battery_pct(batt.remaining_percent)
            if self.battery_source == 'PX4':
                self.battery_pct = self.raw_battery_pct
            self.battery_valid = True

    async def track_air(self):
        async for flying in self.system.telemetry.in_air():
            if self._stop:
                break
            self.in_air = flying
            if self.landing and not flying:
                self.flight_end = self.mission_now()
                self.armed = False
                self.landed = True
                if self.service_ready_at is None:
                    self.service_ready_at = self.mission_now() + BATTERY_SWAP_S

    async def land(self):
        if self.landing or not self.armed:
            return
        self.landing = True
        with contextlib.suppress(Exception):
            await self.system.offboard.stop()
        try:
            await self.system.action.land()
        except ActionError as exc:
            self.failure = f'land: {exc}'

    def flight_time(self):
        if self.flight_start is None:
            return 0.0
        return (self.flight_end if self.flight_end is not None else self.mission_now()) - self.flight_start

    def complete_simulated_battery_swap(self):
        if not self.landed or self.service_ready_at is None:
            return False
        if self.mission_now() < self.service_ready_at:
            return False
        self.battery_source = 'SIMULATED_SWAP'
        self.battery_pct = 100.0
        self.battery_valid = True
        self.landing = False
        self.landed = False
        self.service_ready_at = None
        return True

    def request_stop(self):
        self._stop = True


async def run_agent(agent, role_mgr, events):
    await agent.ready_event.wait()
    trackers = [asyncio.create_task(agent.track_position()),
                asyncio.create_task(agent.track_battery()),
                asyncio.create_task(agent.track_air())]
    streamer = None
    try:
        while not agent._stop:
            role = role_mgr.drones[agent.id].role
            if role == Role.GROUNDED and agent.landed and agent.ready:
                if agent.complete_simulated_battery_swap():
                    role_mgr.drones[agent.id].role = Role.STANDBY
                    events.append(f'drone {agent.id} simulated pack swap complete; '
                                  f'available for a new sortie')
            if agent.battery_source == 'SIMULATED_SWAP' and agent.armed:
                agent.battery_pct = max(0.0, 100.0 * (
                    1.0 - agent.flight_time() / UAV_MAX_FLIGHT_TIME_S))
            if role in (Role.SURVEY, Role.RELAY) and not agent.armed and not agent.landed and not agent.failure:
                if not agent.position_fresh():
                    await asyncio.sleep(0.5)
                    continue
                if await agent.arm_and_start_offboard():
                    events.append(f'drone {agent.id} airborne task started ({role.value})')
                    if streamer is None or streamer.done():
                        streamer = asyncio.create_task(agent.stream_setpoints())
            elif role == Role.RTH and agent.armed and not agent.landing:
                x, y, z = agent.position_world
                if math.dist((x, y), (agent.spawn_x, agent.spawn_y)) <= 5.0 and z <= 10:
                    await agent.land()
                    events.append(f'drone {agent.id} landing at start area')
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        agent.failure = f'{type(exc).__name__}: {exc}'
        events.append(f'drone {agent.id} ERROR: {agent.failure}')
    finally:
        for task in trackers + ([streamer] if streamer else []):
            task.cancel()
        await asyncio.gather(*trackers, *([streamer] if streamer else []), return_exceptions=True)


# ============================================================
# MISSION COORDINATOR
# ============================================================

async def coordination_loop(agents, role_mgr, poi_sim, tlog, viewer, events):
    clock = viewer.choose_clock()
    for agent in agents.values():
        agent.mission_now = clock
    poi_sim.now = clock
    poi_sim.start()
    mission_start = clock()
    survey_progress = {
        cell: {'wps': lawnmower_waypoints(cell, role_mgr.num_survey_cells,
                                        SURVEY_LANE_SPACING_M),
               'idx': 0, 'complete': False}
        for cell in range(role_mgr.num_survey_cells)
    }
    coverage = CoverageTracker()
    metrics = {'link_ok_s': 0.0, 'link_down_s': 0.0, 'reallocations': 0,
               'wall_start': time.monotonic(),
               'collision_events': 0, 'min_separation_m': None}
    safety_violations = set()
    last_elapsed = 0.0
    last_dashboard_wall = -10.0
    last_warning = {}
    logged_events = 0
    pending_reports = []
    rth_all = False
    survey_limit = min(NUM_SURVEY_CELLS, max(1, len([a for a in agents.values() if a.ready]) // 3))
    last_capacity_warning = -100.0
    separation_violation = False

    def event(message, elapsed):
        events.append(f'T+{fmt_time(elapsed)} {message}')

    def warn_once(key, message, elapsed):
        if elapsed - last_warning.get(key, -100.0) >= 10.0:
            event(message, elapsed)
            last_warning[key] = elapsed

    def eligible_surveys():
        return [d for d in role_mgr.active_survey() if agents[d.id].ready
                and agents[d.id].position_fresh() and not agents[d.id].failure]

    while True:
        elapsed = max(0.0, clock() - mission_start)
        dt = max(0.0, elapsed - last_elapsed)
        last_elapsed = elapsed
        if (viewer.clock_source == 'SIM' and viewer.sim_updated_at and
                time.monotonic() - viewer.sim_updated_at > 90):
            event('Gazebo clock stalled for >90s; mission cannot advance', elapsed)
            rth_all = True
            for d in role_mgr.drones.values():
                d.role = Role.RTH if agents[d.id].armed else Role.GROUNDED
            break
        announced = poi_sim.newly_announced()
        for poi in announced:
            event(f'NEW POI-{poi.id:02d} priority {poi.priority} at '
                  f'({poi.x:.0f},{poi.y:.0f})', elapsed)

        # Reassign a failed/disconnected vehicle before giving any new instructions.
        for d in list(role_mgr.drones.values()):
            a = agents[d.id]
            if d.role not in (Role.SURVEY, Role.RELAY):
                continue
            if not a.ready or a.failure or (a.armed and not a.position_fresh()):
                old_role = d.role
                replacement = role_mgr.promote_replacement(d.id, agents)
                metrics['reallocations'] += 1
                event(f'drone {d.id} {old_role.value} unavailable; replacement '
                      f'{replacement if replacement is not None else "pending"}', elapsed)

        if not rth_all:
            # A strip belongs to the fleet, not to a drone: a replacement resumes its index.
            for d in role_mgr.active_survey():
                if d.assigned_cell is None or not survey_progress[d.assigned_cell]['complete']:
                    continue
                next_cell = next((c for c, p in survey_progress.items()
                                  if not p['complete'] and not any(
                                      s.id != d.id and s.assigned_cell == c
                                      for s in role_mgr.active_survey())), None)
                if next_cell is not None:
                    event(f'drone {d.id} moves from complete strip {d.assigned_cell} '
                          f'to strip {next_cell}', elapsed)
                    d.assigned_cell = next_cell
                    metrics['reallocations'] += 1
                else:
                    d.role = Role.RTH if agents[d.id].armed else Role.GROUNDED
            for drone_id, cell, waypoint in role_mgr.fill_survey(
                    agents, survey_progress, survey_limit):
                event(f'drone {drone_id} assigned strip {cell}, resumes waypoint '
                      f'{waypoint}/{len(survey_progress[cell]["wps"])}', elapsed)
                metrics['reallocations'] += 1

        # Assign announced targets to the nearest available surveyor, highest priority first.
        active = eligible_surveys()
        poi_targets = {}
        available = set(d.id for d in active)
        for poi in sorted(poi_sim.active_pois(),
                          key=lambda p: (-p.priority, p.spawn_time_s)):
            if not available:
                break
            drone_id = min(available,
                           key=lambda i: math.dist(agents[i].position_world[:2],
                                                   (poi.x, poi.y)))
            poi_targets[drone_id] = poi
            available.remove(drone_id)

        survey_positions = {}
        projected_positions = {}
        for d in active:
            a = agents[d.id]
            progress = survey_progress[d.assigned_cell]
            if progress['complete']:
                continue
            if d.id in poi_targets:
                poi = poi_targets[d.id]
                tx, ty = poi.x, poi.y
            else:
                tx, ty = progress['wps'][progress['idx']]
                if (a.in_air and math.dist(a.position_world[:2], (tx, ty))
                        <= ARRIVAL_TOLERANCE_M):
                    progress['idx'] += 1
                    if progress['idx'] == len(progress['wps']):
                        progress['complete'] = True
                        event(f'strip {d.assigned_cell} sweep complete', elapsed)
                        continue
                    tx, ty = progress['wps'][progress['idx']]
            a.set_world_target(tx, ty, CRUISE_ALTITUDE_M)
            if a.armed:
                current = a.position_world
                survey_positions[d.id] = current
                distance = math.dist(current[:2], (tx, ty))
                lookahead = min(distance, 75.0)
                projected_positions[d.id] = (
                    current[0] + (tx - current[0]) * lookahead / max(1.0, distance),
                    current[1] + (ty - current[1]) * lookahead / max(1.0, distance),
                    CRUISE_ALTITUDE_M)
                coverage.observe(*current[:2])

        # Trade active survey capacity for relays when the geometric network needs it.
        budget = len(role_mgr.active_relays()) + len(role_mgr.reserve_pool(agents))
        relay_waypoints, required_relays = relay_targets(projected_positions, budget)
        while required_relays > budget and len(survey_positions) > 1 and not rth_all:
            victim_id = max(survey_positions,
                            key=lambda i: (poi_targets.get(i, None) is None,
                                           math.dist(GCS_POSITION[:2], survey_positions[i][:2])))
            victim = role_mgr.drones[victim_id]
            old_cell = victim.assigned_cell
            victim.role = Role.RELAY
            victim.assigned_cell = None
            survey_limit = max(1, survey_limit - 1)
            survey_positions.pop(victim_id)
            projected_positions.pop(victim_id)
            poi_targets.pop(victim_id, None)
            metrics['reallocations'] += 1
            event(f'reassign drone {victim_id} strip {old_cell} -> RELAY; '
                  f'waypoint {survey_progress[old_cell]["idx"]} preserved', elapsed)
            budget = len(role_mgr.active_relays()) + len(role_mgr.reserve_pool(agents))
            relay_waypoints, required_relays = relay_targets(projected_positions, budget)

        if required_relays > budget:
            last_capacity_warning = elapsed
            warn_once('relay_capacity',
                      f'RELAY SHORTAGE {budget}/{required_relays}; survey holds at link edge', elapsed)
        elif elapsed - last_capacity_warning > 45 and survey_limit < NUM_SURVEY_CELLS:
            if len(role_mgr.reserve_pool(agents)) >= 2:
                survey_limit += 1
                last_capacity_warning = elapsed
                event(f'capacity recovered; survey slots now {survey_limit}', elapsed)

        if not rth_all:
            before = len(role_mgr.active_relays())
            role_mgr.rebalance_relays(len(relay_waypoints), agents)
            metrics['reallocations'] += len(role_mgr.active_relays()) - before
        relay_assignment = match_relays_to_waypoints(
            role_mgr.active_relays(), agents, relay_waypoints)
        for d in role_mgr.active_relays():
            a = agents[d.id]
            if d.id in relay_assignment:
                a.set_world_target(*relay_assignment[d.id], CRUISE_ALTITUDE_M)
            else:
                a.set_world_target(*a.position_world)

        links, used_hop = connected_network(agents, role_mgr)
        live_relays = [agents[d.id].position_world for d in role_mgr.active_relays()
                       if links[d.id] and agents[d.id].position_fresh()]
        def next_position(agent):
            x, y, z = agent.position_world
            tx, ty, _ = agent.target_world
            distance = math.dist((x, y), (tx, ty))
            step = min(8.0, distance)
            return (x + (tx-x) * step / max(distance, 1.0),
                    y + (ty-y) * step / max(distance, 1.0),
                    CRUISE_ALTITUDE_M)
        for d in role_mgr.active_survey():
            a = agents[d.id]
            a.pause_for_link = a.in_air and (
                not links[d.id] or not is_connected(
                    next_position(a), GCS_POSITION, live_relays))
        for d in role_mgr.active_relays():
            a = agents[d.id]
            upstream = [pos for pos in live_relays
                        if pos is not a.position_world and pos != a.position_world]
            a.pause_for_link = a.in_air and links[d.id] and not is_connected(
                next_position(a), GCS_POSITION, upstream)
        active_airborne = [d for d in role_mgr.active_survey() if agents[d.id].in_air]
        if active_airborne:
            if all(links[d.id] for d in active_airborne):
                metrics['link_ok_s'] += dt
            else:
                metrics['link_down_s'] += dt

        # Turn over drones early enough for their own travel distance and landing reserve.
        for d in list(role_mgr.drones.values()):
            a = agents[d.id]
            if d.role not in (Role.SURVEY, Role.RELAY) or not a.armed:
                continue
            return_s = (math.dist(a.position_world[:2], (a.spawn_x, a.spawn_y)) /
                        COMMANDED_SPEED_MPS + abs(a.position_world[2]) / 1.5 +
                        RTH_RESERVE_S)
            if (a.flight_time() + return_s >= UAV_MAX_FLIGHT_TIME_S or
                    elapsed + return_s >= MISSION_DURATION_S or
                    a.battery_valid and a.battery_pct <= 20):
                old_role, old_cell = d.role, d.assigned_cell
                replacement = role_mgr.promote_replacement(d.id, agents) if not rth_all else None
                metrics['reallocations'] += 1
                event(f'drone {d.id} {old_role.value} RTH for time/battery; '
                      f'cell {old_cell}, replacement '
                      f'{replacement if replacement is not None else "pending"}', elapsed)

        for poi in poi_sim.check_detections({
                d.id: agents[d.id].position_world[:2] for d in role_mgr.active_survey()
                if agents[d.id].in_air and agents[d.id].position_fresh()}):
            event(f'POI-{poi.id:02d} priority {poi.priority} detected by drone '
                  f'{poi.detected_by} ({poi.x:.0f},{poi.y:.0f})', elapsed)
            pending_reports.append(poi)
        still_pending = []
        links, used_hop = connected_network(agents, role_mgr)
        for poi in pending_reports:
            if links.get(poi.detected_by, False):
                poi_sim.mark_reported(poi)
                latency = poi.reported_at_s - poi.detected_at_s
                event(f'POI-{poi.id:02d} reported to GCS in {latency:.1f}s '
                      f'[{'OK' if latency <= MAX_POI_REPORT_LATENCY_S else 'LATE'}]', elapsed)
            else:
                still_pending.append(poi)
        pending_reports = still_pending

        all_strips_done = all(p['complete'] for p in survey_progress.values())
        all_pois_reported = all(p.reported_at_s > 0 for p in poi_sim.pois)
        if not rth_all and (elapsed >= MISSION_DURATION_S - RTH_RESERVE_S or
                            all_strips_done and all_pois_reported):
            rth_all = True
            event('area/POI task complete or mission window closing; all vehicles RTH', elapsed)
            for d in role_mgr.drones.values():
                if d.role != Role.GROUNDED:
                    d.role = Role.RTH if agents[d.id].armed else Role.GROUNDED

        for d in role_mgr.drones.values():
            if d.role != Role.RTH:
                continue
            a = agents[d.id]
            x, y = a.position_world[:2]
            near_home = math.dist((x, y), (a.spawn_x, a.spawn_y)) <= 5
            a.set_world_target(a.spawn_x, a.spawn_y,
                               8 if near_home else CRUISE_ALTITUDE_M)
            a.pause_for_link = False
            if a.landed:
                d.role = Role.GROUNDED

        min_sep = minimum_separation(agents)
        if math.isfinite(min_sep):
            best = metrics['min_separation_m']
            metrics['min_separation_m'] = min(best, min_sep) if best is not None else min_sep
        if min_sep < MIN_SEPARATION_M:
            if not separation_violation:
                metrics['collision_events'] += 1
            separation_violation = True
            warn_once('separation', f'SEPARATION VIOLATION {min_sep:.1f}m', elapsed)
        else:
            separation_violation = False
        for d in role_mgr.drones.values():
            a = agents[d.id]
            if not a.position_fresh() or not a.in_air:
                continue
            if a.speed_mps > MAX_SPEED_MPS + 0.1:
                warn_once(f'speed{d.id}', f'drone {d.id} SPEED {a.speed_mps:.1f}m/s', elapsed)
                safety_violations.add(('speed', d.id))
            if not inside_geofence(a.position_world):
                warn_once(f'fence{d.id}', f'drone {d.id} OUTSIDE GEOFENCE', elapsed)
                safety_violations.add(('fence', d.id))
                d.role = Role.RTH
            if a.flight_time() > UAV_MAX_FLIGHT_TIME_S:
                warn_once(f'time{d.id}', f'drone {d.id} FLIGHT LIMIT EXCEEDED', elapsed)
                safety_violations.add(('flight_time', d.id))
            if a.battery_valid and a.battery_pct <= 0:
                safety_violations.add(('battery_empty', d.id))

        viewer.update_active([d.id for d in role_mgr.active_survey()])
        fresh_ids = [i for i, a in agents.items() if a.position_fresh()]
        tlog.log({'mission_elapsed_s': round(elapsed, 2),
                  'wall_elapsed_s': round(time.monotonic() - metrics['wall_start'], 2),
                  'clock_source': viewer.clock_source,
                  'gazebo_sim_seconds': viewer.sim_time(),
                  'inter_drone_3d_m': {
                      f'{i:02d}-{j:02d}': round(math.dist(agents[i].position_world,
                                                        agents[j].position_world), 2)
                      for n, i in enumerate(fresh_ids) for j in fresh_ids[n+1:]}})
        for d in role_mgr.drones.values():
            a = agents[d.id]
            x, y, z = a.position_world
            tlog.log({
                'mission_elapsed_s': round(elapsed, 2), 'drone_id': d.id,
                'role': d.role.value, 'cell': d.assigned_cell,
                'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2),
                'position_fresh': a.position_fresh(), 'speed_mps': round(a.speed_mps, 2),
                'battery_pct': round(a.battery_pct, 1) if a.battery_valid else None,
                'raw_px4_battery_pct': round(a.raw_battery_pct, 1)
                    if a.raw_battery_pct is not None else None,
                'battery_source': a.battery_source, 'sortie': a.sorties,
                'flight_time_s': round(a.flight_time(), 1), 'link_up': links[d.id],
                'connected': a.connected, 'ready': a.ready,
                'connection_retries': a.connection_retries,
                'armed': a.armed, 'in_air': a.in_air,
                'relay_count': len(role_mgr.active_relays()),
                'required_relays': required_relays,
                'used_relay_hop_m': round(used_hop, 2),
                'fleet_min_separation_m':
                    round(min_sep, 2) if math.isfinite(min_sep) else None,
                'coverage_pct': round(coverage.fraction() * 100, 2),
                'survey_progress': {str(cell): p['idx'] for cell, p in survey_progress.items()},
                'poi_detected': sum(p.detected for p in poi_sim.pois),
                'poi_reported': sum(p.reported_at_s > 0 for p in poi_sim.pois),
                'network_uptime_s': round(metrics['link_ok_s'], 2),
                'network_downtime_s': round(metrics['link_down_s'], 2),
                'relay_reallocations': metrics['reallocations'],
                'priority_score': sum(p.priority for p in poi_sim.pois
                                      if p.reported_at_s > 0),
                'worst_link_recovery_s': max(
                    (delay for drone in agents.values()
                     for delay in drone.recovery_durations), default=0.0),
                'camera_status': viewer.status(d.id), 'error': a.failure,
            })
        if time.monotonic() - last_dashboard_wall >= 2:
            render_dashboard(agents, role_mgr, poi_sim, elapsed, links, used_hop,
                             required_relays, viewer, events, min_sep, tlog,
                             coverage, survey_progress, metrics)
            last_dashboard_wall = time.monotonic()
        for message in events[logged_events:]:
            tlog.log({'mission_elapsed_s': round(elapsed, 2), 'event': message})
        logged_events = len(events)
        if rth_all and (all(not a.armed for a in agents.values()) or
                        elapsed >= MISSION_DURATION_S + LANDING_TIMEOUT_S):
            break
        await asyncio.sleep(COORD_LOOP_PERIOD_S)

    timely = all(p.reported_at_s > 0 and
                 p.reported_at_s - p.detected_at_s <= MAX_POI_REPORT_LATENCY_S
                 for p in poi_sim.pois)
    success = (all(not a.armed for a in agents.values()) and timely and
               all(p['complete'] for p in survey_progress.values()) and
               coverage.fraction() >= 0.99 and elapsed <= MISSION_DURATION_S and
               all(a.flight_time() <= UAV_MAX_FLIGHT_TIME_S for a in agents.values()) and
               metrics['collision_events'] == 0 and not safety_violations and
               metrics['link_down_s'] <= COORD_LOOP_PERIOD_S)
    tlog.log({'mission_elapsed_s': round(elapsed, 2), 'summary': {
        'success': success, 'poi_reported': sum(p.reported_at_s > 0 for p in poi_sim.pois),
        'coverage_pct': round(coverage.fraction() * 100, 2),
        'completed_strips': sum(p['complete'] for p in survey_progress.values()),
        'network_uptime_s': round(metrics['link_ok_s'], 2),
        'network_downtime_s': round(metrics['link_down_s'], 2),
        'reallocations': metrics['reallocations'],
        'priority_weighted_score': (
            sum(p.priority for p in poi_sim.pois if p.reported_at_s > 0) /
            sum(p.priority for p in poi_sim.pois)),
        'worst_link_recovery_s': max(
            (delay for a in agents.values() for delay in a.recovery_durations),
            default=0.0),
        'separation_events': metrics['collision_events'],
        'collision_count': None,
        'safety_violations': sorted(f'{name}:{drone_id}'
                                    for name, drone_id in safety_violations),
        'minimum_separation_m': metrics['min_separation_m'],
        'packet_delivery_ratio': None,  # No packet-level simulator is connected.
        'clock_source': viewer.clock_source,
        'gazebo_sim_seconds': viewer.sim_time(),
    }})
    return success


async def main():
    if NUM_DRONES < MIN_READY_DRONES or GROUND_SPAWN_SPACING_M < MIN_SEPARATION_M:
        raise ValueError('Need at least 12 drones and >=20m launch spacing')
    agents = {i: DroneAgent(i, grid_spawn_position(i, NUM_DRONES, GROUND_SPAWN_SPACING_M))
              for i in range(NUM_DRONES)}
    for agent in agents.values():
        agent.peers = agents
    role_mgr = RoleManager(NUM_DRONES, NUM_SURVEY_CELLS)
    poi_sim = POISimulator(seed=SCENARIO_SEED)
    tlog = TelemetryLogger()
    viewer = GazeboCameraViewer(NUM_DRONES)
    events = []
    print('Mission: 10 random timed POIs | 1000x1000m area 75m from GCS | 45min window', flush=True)
    print('Constraints: <=20min/UAV, <=5m/s, <=100m altitude/hop, >=20m separation, <=10s POI report', flush=True)
    print('Mission, flight, POI, and battery-service timers use Gazebo simulated time '
          'when /clock is available; wall-time fallback is labeled.', flush=True)
    print(f'Scenario seed: {SCENARIO_SEED} | minimum launch-ready: {MIN_READY_DRONES}/{NUM_DRONES}', flush=True)
    print(f'Planned sweep: {planned_survey_distance_m()/1000:.1f} km across four strips; '
          f'{planned_survey_distance_m()/COMMANDED_SPEED_MPS/60:.0f} '
          f'survey-drone minutes at {COMMANDED_SPEED_MPS:.0f}m/s, '
          'plus transit, relays and returns.', flush=True)
    print(f'Allowing {STARTUP_SETTLE_S:.0f}s connection settle; '
          f'launch when at least {MIN_READY_DRONES} are ready, '
          f'abort threshold after {CONNECT_TIMEOUT_S:.0f}s.', flush=True)
    viewer.start()
    connect_tasks = [asyncio.create_task(a.connect(events)) for a in agents.values()]
    connection_start = time.monotonic()
    last_report = -10
    while True:
        now = time.monotonic()
        ready = [a.id for a in agents.values() if a.ready]
        if (now - connection_start >= STARTUP_SETTLE_S and
                len(ready) >= MIN_READY_DRONES):
            break
        if now - connection_start >= CONNECT_TIMEOUT_S:
            break
        if now - last_report >= 2:
            pending = [a.id for a in agents.values() if not a.ready]
            print(f'Connection readiness {len(ready)}/{NUM_DRONES}; '
                  f'minimum {MIN_READY_DRONES}; retrying {pending}; '
                  f'wait {fmt_time(now-connection_start)}/{fmt_time(CONNECT_TIMEOUT_S)} '
                  f'| Gazebo {viewer.sim_time() if viewer.sim_time() is not None else "WAIT"}', flush=True)
            last_report = now
        await asyncio.sleep(0.5)
    if len(ready) < MIN_READY_DRONES:
        print(f'ABORT: only {len(ready)}/{MIN_READY_DRONES} ready after '
              f'{CONNECT_TIMEOUT_S:.0f}s; no drone was armed.', flush=True)
        for task in connect_tasks:
            task.cancel()
        await asyncio.gather(*connect_tasks, return_exceptions=True)
        viewer.stop()
        tlog.close()
        return 1
    pending = [a.id for a in agents.values() if not a.ready]
    for drone_id in pending:
        agents[drone_id].link_lost_at = time.monotonic()
    print(f'Mission starts with {len(ready)}/{NUM_DRONES} ready. '
          f'Background retries continue for {pending}.', flush=True)
    flight_tasks = [asyncio.create_task(run_agent(a, role_mgr, events)) for a in agents.values()]
    async def camera_display():
        while True:
            try:
                viewer.render()
            except Exception as exc:
                viewer.error = f'camera window: {exc}'
            await asyncio.sleep(0.2)
    camera_task = asyncio.create_task(camera_display())
    try:
        success = await coordination_loop(agents, role_mgr, poi_sim, tlog, viewer, events)
        print('Mission result:', 'COMPLETE' if success else 'INCOMPLETE; inspect live warnings and log', flush=True)
        return 0 if success else 1
    finally:
        for agent in agents.values():
            if agent.armed and not agent.landing:
                with contextlib.suppress(Exception):
                    await agent.land()
        deadline = time.monotonic() + 30.0
        while any(a.in_air for a in agents.values()) and time.monotonic() < deadline:
            await asyncio.sleep(0.5)
        for agent in agents.values():
            agent.request_stop()
        for task in flight_tasks:
            task.cancel()
        await asyncio.gather(*flight_tasks, return_exceptions=True)
        for task in connect_tasks:
            task.cancel()
        await asyncio.gather(*connect_tasks, return_exceptions=True)
        camera_task.cancel()
        await asyncio.gather(camera_task, return_exceptions=True)
        viewer.stop()
        tlog.close()


if __name__ == '__main__':
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print('\nMission interrupted; landing requested.')
