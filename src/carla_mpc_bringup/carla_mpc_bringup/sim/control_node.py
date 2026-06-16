#!/usr/bin/env python3
"""Spawn the ego rig, run the controller, drive CARLA, and publish telemetry.

This is the active half of the testbed. It connects to a CARLA server started
with ``--ros2`` and applies
synchronous fixed-step settings, spawns the ego ("hero") plus its sensor rig and
opts them into CARLA's native ROS2 publisher, and builds a route for the
controller to track (random town routes or a fixed Leaderboard route).

It is the single world ticker: every tick it reads the ego state, asks the active
controller (PID, PID++, MPC, or MPCC) for a command, applies it to CARLA, and
republishes the reference path, the look-ahead target, and a stream of control
telemetry for the Foxglove dashboard.

Control runs in the ticking process by design: in synchronous mode the command
must be applied before each tick, so computing it in a separate node would race
the tick. The controller stays ROS-free behind ControllerInterface; this node is
the only place CARLA and ROS meet on the control path. The separate, read-only
gt_pose_publisher owns the actual driven trajectory (/carla/hero/gt/{path,odom})
and the gt_map->hero TF anchor.

Run via launch, or standalone:
    ros2 run carla_mpc_bringup control_node --controller pid
Prerequisite: CARLA already running with ``--ros2``.
"""
from __future__ import annotations

import argparse
import logging
import math
import sys

import numpy as np
import threading
import time
from pathlib import Path

try:
    import carla
except ImportError:
    sys.stderr.write(
        "ERROR: 'carla' module not importable. Install the wheel matching your "
        "Python (see setup_env.sh / README).\n")
    sys.exit(2)

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from visualization_msgs.msg import Marker

from carla_mpc_bringup.control.controller_interface import ControllerInterface, VehicleState
from carla_mpc_bringup.control.mpc_controller import MPCController
from carla_mpc_bringup.control.pid_controller import PIDController
from carla_mpc_bringup.sim.lane_invasion import LaneInvasionMonitor
from carla_mpc_bringup.core.config_loader import (
    load_carla_config,
    load_controller_config,
    load_vehicle_config,
)
from carla_mpc_bringup.core.frame_conventions import euler_deg_to_matrix, mount_to_matrix
from carla_mpc_bringup.eval.run_logger import RunLogger
from carla_mpc_bringup.eval.metrics import summary_text
from carla_mpc_bringup.sim.route_loader import load_route
from carla_mpc_bringup.sim.route_planner import RoutePlanner
from carla_mpc_bringup.sim.sensor_factory import enable_sensor_ros, spawn_sensor

logger = logging.getLogger("control_node")

MAP_FRAME = "gt_map"          # ROS world frame, shared with gt_pose_publisher
SETTLE_SECONDS = 0.5          # let the spawn drop settle (ROS off) before driving
# Extend the route this far (m) before its end enters the controller look-ahead, so
# the car flows through targets without stopping. Sized as a practical buffer
# relative to the MPC look-ahead (N*dt*v_cruise ~ 50*0.06*42 = 126 m at the
# 150 km/h cruise cap, where v_cruise = min(target_speed_kmh/3.6, v_max)).
EXTEND_MARGIN_M = 100.0
# Lane-invasion masking thresholds. A crossing belongs to the reference (not a real
# departure) when the car is on its trajectory: within LANE_ON_PATH_M cross-track is
# "on path" (mask anything -- lane change or on-path junction turn); a detected reference
# lane change masks up to LANE_LAG_M to allow tracking lag. Beyond that is a corner-cut or
# drift and counts.
LANE_ON_PATH_M = 0.7
LANE_LAG_M = 1.5


def carla_to_ros_xy(x: float, y: float) -> tuple[float, float]:
    """Convert CARLA xy (left-handed, Y right) to ROS xy (REP-103, Y left) by negating Y."""
    return x, -y


def jet(t: float) -> tuple[float, float, float]:
    """Map a value in [0, 1] to jet-ish RGB (blue low -> green -> red high) for heatmaps."""
    t = min(1.0, max(0.0, t))
    cl = lambda u: min(1.0, max(0.0, u))  # noqa: E731
    return cl(1.5 - abs(4 * t - 3)), cl(1.5 - abs(4 * t - 2)), cl(1.5 - abs(4 * t - 1))


def sim_time_to_rostime(t_sec: float) -> RosTime:
    """Convert a float sim time in seconds to a ROS builtin_interfaces Time."""
    sec = int(t_sec)
    return RosTime(sec=sec, nanosec=int((t_sec - sec) * 1e9))


class ControlNode(Node):
    """Own the ROS publishers and the controller/CARLA glue; the tick loop lives in main."""

    def __init__(self, controller_name: str):
        super().__init__("control_node")
        self.set_parameters([rclpy.Parameter("use_sim_time", value=True)])
        ns = "/controller"

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # The camera overlay needs the route geometry at real terrain z, published on
        # /reference_path_3d; the 3D-scene reference itself is the speed heatmap.
        self.pub_ref_3d = self.create_publisher(PathMsg, f"{ns}/reference_path_3d", latched)
        self._z_offset = 0.0          # added to the speed heatmap z so it lands on the driven path (set in main)
        # Latched horizon. Minor display artifact: occasional "snatching" where the leading
        # (current) point updates a tick before the rest re-renders, so the line briefly starts
        # from an older point. Viewer-side timing only -- the bag data is geometrically clean.
        self.pub_pred = self.create_publisher(PathMsg, f"{ns}/predicted_path", latched)
        self.pub_goal = self.create_publisher(Marker, f"{ns}/goal_point", latched)
        # Latched precomputed-trajectory heatmap: the per-trajectory annotation a controller
        # will drive -- the path coloured by speed and by curve angle/curvature.
        self.pub_heat_speed = self.create_publisher(Marker, f"{ns}/heatmap_speed", latched)
        self.pub_heat_curve = self.create_publisher(Marker, f"{ns}/heatmap_curve", latched)
        # Lane crossing: a blink Bool (Foxglove Indicator panel), a flashing 3D marker, and
        # the crossed marking type (recorded for review).
        self.pub_lane = self.create_publisher(Bool, f"{ns}/lane_invasion", 10)
        self.pub_lane_marker = self.create_publisher(Marker, f"{ns}/lane_invasion_marker", 10)
        self.pub_lane_type = self.create_publisher(String, f"{ns}/lane_invasion_type", 10)

        # Scalar telemetry for the Foxglove plots: one topic per channel, value in .data.
        def f32(name):
            return self.create_publisher(Float32, f"{ns}/{name}", 10)
        self.pub = {
            "speed_kmh": f32("speed_kmh"),
            "target_speed_kmh": f32("target_speed_kmh"),
            "throttle": f32("throttle"),
            "brake": f32("brake"),
            "steer": f32("steer"),
            "cross_track_error": f32("cross_track_error"),
            "heading_error_deg": f32("heading_error_deg"),
            "solve_time_ms": f32("solve_time_ms"),
        }
        self.get_logger().info(
            f"control_node up (controller={controller_name}); publishing under {ns}/*")

    def publish_reference(self, route_xyz, stamp: RosTime):
        """Publish the 3D reference path and cache it for terrain elevation lookups."""
        # Cache the reference (CARLA coords, 3D) so publish_predicted can lift the OCP horizon
        # to the terrain and publish_heatmap can place the speed heatmap on the road.
        self._ref_xyz = np.asarray(route_xyz, dtype=float) if len(route_xyz) else None
        msg = PathMsg()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = stamp
        for (cx, cy, cz) in route_xyz:
            rx, ry = carla_to_ros_xy(cx, cy)
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = rx, ry, cz + 0.2
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_ref_3d.publish(msg)

    def publish_goal(self, gx: float, gy: float, stamp: RosTime):
        """Publish the goal point as a sphere marker."""
        rx, ry = carla_to_ros_xy(gx, gy)
        self.pub_goal.publish(self._sphere(rx, ry, 1.5, (0.1, 0.4, 1.0, 0.9),
                                           "goal", 1, scale=2.5, stamp=stamp))

    def publish_predicted(self, pred_xy: list[tuple[float, float]], stamp: RosTime):
        """Publish the OCP horizon lifted to terrain elevation, or an empty path to clear it."""
        if not pred_xy:
            # No prediction (e.g. PID) -> publish an empty path so the latched topic clears any
            # stale horizon left by a previous controller (the orange MPCC line must not linger).
            empty = PathMsg()
            empty.header.frame_id = MAP_FRAME
            empty.header.stamp = stamp
            self.pub_pred.publish(empty)
            return
        # The OCP horizon is 2D (x, y). Lift each point to the terrain elevation via the nearest
        # cached reference waypoint, so the projected path follows the ground on elevated maps
        # (Town11/12) instead of lying flat. Fall back to a flat z if there is no reference.
        ref = getattr(self, "_ref_xyz", None)
        pred = np.asarray(pred_xy, dtype=float)
        if ref is not None and len(ref):
            d = (pred[:, None, 0] - ref[None, :, 0]) ** 2 + (pred[:, None, 1] - ref[None, :, 1]) ** 2
            zs = ref[d.argmin(axis=1), 2]
        else:
            zs = np.full(len(pred), 0.3)
        msg = PathMsg()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = stamp
        for (cx, cy), cz in zip(pred_xy, zs):
            rx, ry = carla_to_ros_xy(cx, cy)
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = rx, ry, float(cz) + 0.3
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_pred.publish(msg)

    def publish_heatmap(self, heat, stamp):
        """Publish the precomputed trajectory annotation as two coloured polylines.

        One polyline is coloured by speed, the other by |curve angle / curvature|.
        Drawn in 3D on the terrain (per-point z from the cached reference) plus a
        constant offset so it lands on the driven path. Latched, so it persists and
        refreshes on route extension.

        Parameters
        ----------
        heat : list of (x, y, speed, secondary) in CARLA coords, or None
            Per-point trajectory annotation; ``secondary`` is the curve metric.
        """
        if not heat:
            return
        import numpy as np
        a = np.asarray(heat, dtype=float)
        zs = self._terrain_z(a[:, 0], a[:, 1]) + self._z_offset
        self.pub_heat_speed.publish(self._heat_marker(a[:, 0], a[:, 1], a[:, 2],
                                                      "heatmap_speed", zs, stamp))
        self.pub_heat_curve.publish(self._heat_marker(a[:, 0], a[:, 1], np.abs(a[:, 3]),
                                                      "heatmap_curve", zs, stamp))

    def _terrain_z(self, xs, ys):
        """Look up per-point terrain z from the cached 3D reference (nearest waypoint); flat fallback if none."""
        import numpy as np
        ref = getattr(self, "_ref_xyz", None)
        if ref is None or not len(ref):
            return np.full(len(xs), 0.4)
        d = (np.asarray(xs)[:, None] - ref[None, :, 0]) ** 2 + (np.asarray(ys)[:, None] - ref[None, :, 1]) ** 2
        return ref[d.argmin(axis=1), 2]

    def _heat_marker(self, xs, ys, vals, ns, zs, stamp):
        import numpy as np
        fin = vals[np.isfinite(vals)]                       # exclude non-finite values from the colour range
        lo, hi = (float(fin.min()), float(fin.max())) if fin.size else (0.0, 1.0)
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, 0, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.7
        m.pose.orientation.w = 1.0
        for x, y, v, zz in zip(xs, ys, vals, zs):
            rx, ry = carla_to_ros_xy(float(x), float(y))
            m.points.append(Point(x=rx, y=ry, z=float(zz)))
            t = (float(v) - lo) / (hi - lo + 1e-9) if np.isfinite(v) else 0.0
            r, g, b = jet(t)
            m.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))
        return m

    def publish_lane_invasion(self, active, blink_on, label, ego_xy, z, stamp):
        """Publish the blink Bool (drives the Foxglove Indicator) and a flashing red 3D text
        marker above the car while a crossing is recent."""
        self.pub_lane.publish(Bool(data=bool(blink_on)))
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = stamp
        m.ns, m.id, m.type = "lane_invasion", 0, Marker.TEXT_VIEW_FACING
        if active:
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = float(ego_xy[0]), float(ego_xy[1])
            m.pose.position.z = float(z) + 2.6
            m.pose.orientation.w = 1.0
            m.scale.z = 1.6
            m.color = ColorRGBA(r=1.0, g=0.09, b=0.27, a=1.0 if blink_on else 0.12)
            m.text = f"⚠ LINE CROSSED\n{label}" if label else "⚠ LINE CROSSED"
        else:
            m.action = Marker.DELETE
        self.pub_lane_marker.publish(m)

    def publish_scalars(self, **kw):
        """Publish each non-None keyword value on its matching Float32 telemetry topic."""
        for name, val in kw.items():
            if val is not None and name in self.pub:
                self.pub[name].publish(Float32(data=float(val)))

    def _sphere(self, x, y, z, rgba, ns, mid, scale, stamp):
        """Build a sphere Marker at (x, y, z) with the given RGBA colour and scale."""
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
        return m


# ----------------------------------------------------------------- CARLA helpers
def apply_world_settings(world, carla_cfg, no_rendering):
    """Apply synchronous fixed-step and no-rendering settings to the CARLA world."""
    s = world.get_settings()
    s.synchronous_mode = carla_cfg.synchronous_mode
    s.fixed_delta_seconds = carla_cfg.fixed_delta_seconds
    s.no_rendering_mode = no_rendering
    world.apply_settings(s)
    logger.info("World: sync=%s dt=%.4fs (%.0f Hz) no_rendering=%s",
                carla_cfg.synchronous_mode, carla_cfg.fixed_delta_seconds,
                1.0 / carla_cfg.fixed_delta_seconds, no_rendering)


def spawn_ego(world, veh_cfg, transform):
    """Spawn the ego vehicle at the given transform and return the actor."""
    bp = world.get_blueprint_library().find(veh_cfg.blueprint)
    bp.set_attribute("role_name", veh_cfg.role_name)
    bp.set_attribute("ros_name", veh_cfg.ros_name)   # native-publisher topic namespace
    ego = world.try_spawn_actor(bp, transform)
    if ego is None:
        raise RuntimeError(f"Failed to spawn ego at {transform.location}.")
    logger.info("Spawned ego %s (role=%s).", veh_cfg.blueprint, veh_cfg.role_name)
    return ego


def lane_transform(carla_map, loc):
    """Snap a location to the nearest driving lane to get a valid on-road spawn pose."""
    wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    t = wp.transform
    return carla.Transform(carla.Location(t.location.x, t.location.y, t.location.z + 0.3),
                           t.rotation)


def build_controller(name, ego, ctrl_cfg, dt, fixed_speed) -> ControllerInterface:
    """Construct the controller named by ``name`` (pid|pidpp|mpc|mpcc)."""
    name = name.lower()
    if name == "pid":
        return PIDController(ego, ctrl_cfg.pid, dt, fixed_target_speed_kmh=fixed_speed)
    if name == "pidpp":
        from carla_mpc_bringup.control.pidplus_controller import PIDPlusController
        return PIDPlusController(ego, ctrl_cfg.pidpp, dt, fixed_target_speed_kmh=fixed_speed)
    if name == "mpc":
        return MPCController(ego, ctrl_cfg.mpc, dt, fixed_target_speed_kmh=fixed_speed)
    if name == "mpcc":
        from carla_mpc_bringup.control.mpcc_controller import MPCCController
        return MPCCController(ego, ctrl_cfg.mpcc, dt, fixed_target_speed_kmh=fixed_speed)
    raise ValueError(f"Unknown controller '{name}' (expected pid|pidpp|mpc|mpcc).")


def route_xyz(route) -> list[tuple[float, float, float]]:
    """Extract [(carla_x, carla_y, carla_z), ...] from a [(Waypoint, RoadOption), ...] route.

    Keeps the waypoint elevation so the published paths sit on the terrain
    (Town11/12 have real elevation).
    """
    return [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp, _ in route]


def trim_by_arc(pts, keep_m: float):
    """Return the first ``keep_m`` metres (cumulative xy arc length) of a point sequence.

    Cuts the reference-only route-end buffer out of everything displayed (reference
    path, heatmaps): the buffer exists for the solver, but the user should only ever
    see the real route.
    """
    if not pts or keep_m <= 0.0 or len(pts) < 2:
        return pts
    a = np.asarray([[p[0], p[1]] for p in pts], dtype=float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])
    return list(pts[: int(np.searchsorted(s, keep_m)) + 1])


def make_state(ego, t_sec: float) -> VehicleState:
    """Read the ego transform and velocity into a VehicleState (yaw in radians)."""
    tf = ego.get_transform()
    v = ego.get_velocity()
    return VehicleState(
        x=tf.location.x, y=tf.location.y, z=tf.location.z,
        yaw=math.radians(tf.rotation.yaw),
        vx=v.x, vy=v.y, vz=v.z, t=t_sec)


# ------------------------------------------------------------------------- args
def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the control node."""
    cfg = _default_config_dir()
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--vehicle-config", default=str(cfg / "vehicle.yaml"))
    p.add_argument("--carla-config", default=str(cfg / "carla.yaml"))
    p.add_argument("--controller-config", default=str(cfg / "controller.yaml"))
    p.add_argument("--controller", default="pid", help="pid | pidpp | mpc | mpcc")
    p.add_argument("--speed-cap", dest="speed_cap", default=0.0, type=float,
                   help="override the mpc/mpcc cruise-speed cap (km/h) for this run (0 = config value). "
                        "Runtime-only (no solver rebuild); labels the run <controller>@<cap> for the report.")
    p.add_argument("--host", default=None)
    p.add_argument("--port", default=None, type=int)
    p.add_argument("--map", default=None, help="override carla.yaml/route map")
    p.add_argument("--seed", default=None, type=int, help="override route/spawn RNG seed (benchmark)")
    p.add_argument("--spawn-index", dest="spawn_index", default=None, type=int,
                   help="override map spawn-point index (benchmark)")
    p.add_argument("--route", default=None,
                   help="explicit route XML (overrides carla.yaml route.file): its <waypoints> are "
                        "traced into a lane-level reference; the town comes from the XML")
    p.add_argument("--route-id", dest="route_id", default="",
                   help="route id within --route (empty = first)")
    p.add_argument("--condition", default=None, help="named condition or CARLA preset")
    p.add_argument("--scenario-group", dest="scenario_group", default="",
                   help="scenario-list name (scenarios.yaml top-level key) this run belongs to; the "
                        "report aggregates separately per group. '' = main.")
    p.add_argument("--sensors", default="", help="comma-separated sensor whitelist")
    p.add_argument("--record-camera", dest="record_camera", action="store_true",
                   help="record the driver-view to camera.mp4 (incremental/Ctrl+C-safe; the Foxglove "
                        "camera+trajectory overlay if running, else raw); forces rendering ON")
    p.add_argument("--no-rendering-mode", dest="no_rendering", default="auto",
                   choices=["auto", "on", "off"])
    p.add_argument("--warmup", default=-1.0, type=float,
                   help="seconds to hold still before driving (<0 = carla.yaml)")
    p.add_argument("--max-distance", default=0.0, type=float,
                   help="stop the run after this driven distance (m); 0 = unbounded. "
                        "Use for bounded, replicable benchmark runs (same road segment).")
    p.add_argument("--max-duration", default=0.0, type=float,
                   help="stop the run after this sim duration (s); 0 = unbounded.")
    p.add_argument("--route-extend", dest="route_extend", default="true",
                   help="extend the route with random goals at the end (true|false). false for "
                        "deterministic scenario runs -- the random extender can stitch on 180-deg "
                        "cusps no controller can follow. "
                        "Launch-friendly value flag (works through ros2 launch).")
    p.add_argument("--verbose", "-v", action="store_true")
    args, _ = p.parse_known_args()
    return args


def _default_config_dir() -> Path:
    """Return the installed share config dir, falling back to the in-tree config dir."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("carla_mpc_bringup")) / "config"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config"


# ------------------------------------------------------------------------- main
def main() -> int:
    """Connect to CARLA, spawn the rig, and run the tick loop until stop or Ctrl-C."""
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    # Accept "<controller>@<kmh>" (e.g. mpcc@130) directly in --controller; the launch
    # (controller:=mpcc@130) and the benchmark both pass it through here, same as --speed-cap.
    if "@" in args.controller:
        base, _, cap = args.controller.partition("@")
        args.controller = base
        if cap:
            try:
                args.speed_cap = float(cap)
            except ValueError:
                pass

    veh_cfg = load_vehicle_config(args.vehicle_config)
    carla_cfg = load_carla_config(args.carla_config)
    ctrl_cfg = load_controller_config(args.controller_config)
    # Optional per-run speed cap (e.g. the 130 km/h mpc/mpcc variant for elevated routes).
    # target_speed_kmh is the curvature-profile cap and is runtime-only: it is not baked into
    # the acados solver, so the same cached solver is reused (no rebuild). Only mpc/mpcc carry
    # this field; pid/pidpp are left untouched.
    cap_applied = args.speed_cap > 0 and args.controller in ("mpc", "mpcc")
    if cap_applied:
        getattr(ctrl_cfg, args.controller).target_speed_kmh = args.speed_cap
    ctrl_label = f"{args.controller}@{args.speed_cap:.0f}" if cap_applied else args.controller
    if args.seed is not None and args.seed >= 0:   # benchmark override (-1 = use config)
        carla_cfg.seed = args.seed
    if args.spawn_index is not None and args.spawn_index >= 0:
        carla_cfg.spawn_index = args.spawn_index
    host = args.host or carla_cfg.host
    port = args.port or carla_cfg.port

    # Resolve the route source up front (its town can override the map).
    route_obj = None
    route_file = args.route or (carla_cfg.route_file if carla_cfg.use_route_file else None)
    if route_file and not Path(route_file).exists():       # resolve a relative --route against the share config dir
        try:
            from ament_index_python.packages import get_package_share_directory
            alt = Path(get_package_share_directory("carla_mpc_bringup")) / "config" / route_file
            if alt.exists():
                route_file = str(alt)
        except Exception:                                  # noqa: BLE001
            pass
    if route_file:
        route_obj = load_route(route_file, args.route_id or carla_cfg.route_id)
        logger.info("File route '%s' (%s): %d waypoints.",
                    route_obj.id, route_obj.town, len(route_obj.waypoints))
    target_map = args.map or (route_obj.town if route_obj else carla_cfg.map)

    client = carla.Client(host, port)
    client.set_timeout(carla_cfg.timeout)
    world = client.get_world()
    if target_map and target_map not in world.get_map().name:
        logger.info("Loading map %s ...", target_map)
        world = client.load_world(target_map)
    carla_map = world.get_map()
    original_settings = world.get_settings()

    # Sensor set + rendering decision: rendering can go off when no camera is in the rig.
    to_spawn = veh_cfg.enabled_sensors()
    whitelist = {n.strip() for n in args.sensors.split(",") if n.strip()}
    if whitelist:
        to_spawn = [s for s in to_spawn if s.name in whitelist]
    has_camera = any(s.type.startswith("camera.") for s in to_spawn)
    no_rendering = (not has_camera) if args.no_rendering == "auto" else (args.no_rendering == "on")
    if args.record_camera and not has_camera:            # --record-camera needs a camera in the rig
        cam = veh_cfg.find("cam_front")
        if cam is not None:
            to_spawn.append(cam); has_camera = True
        else:
            logger.warning("--record-camera: no 'cam_front' in vehicle.yaml; recording disabled.")
    if args.record_camera and has_camera:
        no_rendering = False                             # cameras render nothing under no_rendering_mode
        logger.info("--record-camera: rendering forced ON; camera.mp4 will be written.")

    rclpy.init()
    node = ControlNode(args.controller)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # Human-readable run log (logs/runs/<ts>_<map>_<controller>/): metrics, plots, and
    # CSV, written at shutdown and distinct from the opt-in rosbag. Built up front so the
    # `finally` block can always flush whatever was driven.
    if args.controller == "mpc":
        _model, _tgt, _alat = ctrl_cfg.mpc.model, ctrl_cfg.mpc.target_speed_kmh, ctrl_cfg.mpc.a_lat_max
    elif args.controller == "mpcc":
        _model, _tgt, _alat = "mpcc", ctrl_cfg.mpcc.target_speed_kmh, ctrl_cfg.mpcc.a_lat_max
    else:
        _model, _tgt, _alat = None, ctrl_cfg.pid.default_target_speed_kmh, ctrl_cfg.mpc.a_lat_max
    scenario = {
        "controller": ctrl_label,
        "model": _model,
        "map": target_map,
        "seed": carla_cfg.seed,
        "spawn_index": carla_cfg.spawn_index,
        "condition": args.condition or carla_cfg.weather,
        "group": args.scenario_group or "main",
        "dt": carla_cfg.fixed_delta_seconds,
        "target_speed_kmh": _tgt,
        "fixed_target_kmh": carla_cfg.target_speed_kmh,
        "a_lat_max": _alat,
        "max_distance_m": args.max_distance,
    }
    run_name = f"{time.strftime('%Y%m%d-%H%M%S')}_{target_map}_{ctrl_label.replace('@', '')}"
    log_dir = Path("logs") / "runs" / run_name
    run_logger = RunLogger()
    lane_total = 0
    cam_recorder = None
    if args.record_camera:
        from carla_mpc_bringup.eval.camera_recorder import CameraRecorder, ros_image_to_rgb
        from sensor_msgs.msg import Image as _ImageMsg
        cam_recorder = CameraRecorder(log_dir / "camera.mp4")
        node.create_subscription(            # the Foxglove overlay frames (camera + projected trajectory)
            _ImageMsg, "/controller/camera_overlay/image",
            lambda m: cam_recorder.add(ros_image_to_rgb(m), "overlay",
                                       m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 10)

    actors: list = []
    try:
        apply_world_settings(world, carla_cfg, no_rendering)
        from carla_mpc_bringup.sim import weather as weather_mod
        weather_mod.apply_named(world, args.condition or carla_cfg.weather)

        # Spawn pose: route start (file) or a map spawn point (random).
        if route_obj is not None:
            spawn_tf = lane_transform(
                carla_map, carla.Location(*route_obj.start))
        else:
            pts = carla_map.get_spawn_points()
            spawn_tf = pts[carla_cfg.spawn_index % len(pts)]
        ego = spawn_ego(world, veh_cfg, spawn_tf)
        actors.append(ego)

        # Spawn sensors with ROS deferred, let the drop settle, then enable ROS.
        sensor_actors = []
        for s in to_spawn:
            a = spawn_sensor(world, ego, s, enable_ros=False)
            actors.append(a)
            sensor_actors.append((a, s))
        ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        for _ in range(max(1, int(SETTLE_SECONDS / carla_cfg.fixed_delta_seconds))):
            world.tick()
        for a, s in sensor_actors:
            enable_sensor_ros(a, s)
        world.tick()
        logger.info("Rig up: ego + %d sensors. Topics under /carla/%s/...",
                    len(sensor_actors), veh_cfg.ros_name)

        # Camera recording: the overlay subscription (above) captures the Foxglove frames (camera +
        # trajectory). Also tap the RAW camera as a fallback for headless runs with no overlay node
        # (the recorder ignores raw frames once any overlay frame has been seen).
        if cam_recorder is not None:
            cam_pair = next(((a, s) for a, s in sensor_actors if s.type.startswith("camera.")), None)
            if cam_pair is not None:
                from carla_mpc_bringup.eval.camera_recorder import carla_image_to_rgb
                from carla_mpc_bringup.viz.predicted_path_overlay import draw_paths_on_frame
                # Camera rig (mirrors predicted_path_overlay): mount->body matrix and pinhole K from
                # fov, so the reference/predicted paths can be projected onto the raw frame in-process.
                # The benchmark has no overlay node, so this is what puts the trajectory in camera.mp4
                # there; a launch with the overlay node still wins (its frames feed 'overlay').
                _cs = cam_pair[1]
                _Tbc = mount_to_matrix(_cs.mount); _Rbc, _tbc = _Tbc[:3, :3], _Tbc[:3, 3]
                _a = _cs.attributes
                _W = int(_a.get("image_size_x", 960)); _H = int(_a.get("image_size_y", 540))
                _fx = _W / (2.0 * np.tan(np.radians(float(_a.get("fov", 90.0))) / 2.0))
                _K = np.array([[_fx, 0.0, _W / 2.0], [0.0, _fx, _H / 2.0], [0.0, 0.0, 1.0]])

                def _on_cam(img):
                    rgb = carla_image_to_rgb(img)
                    st = getattr(node, "_overlay_state", None)
                    if st is not None and not cam_recorder._seen_overlay:   # no external overlay node
                        rgb = np.ascontiguousarray(rgb)                    # make the view writable for cv2
                        eR, eT, ref, pred = st
                        draw_paths_on_frame(rgb, eR, eT, ref, pred, _Rbc, _tbc, _K)
                    cam_recorder.add(rgb, "raw", img.timestamp)
                cam_pair[0].listen(_on_cam)
                logger.info("Recording camera -> camera.mp4 (paths drawn in-process; overlay node wins if running)")

        # Lane-crossing monitor: lane_invasion has no native ROS2 publisher, so we
        # listen in Python and blink it into Foxglove (Bool + flashing 3D marker).
        lane_monitor = LaneInvasionMonitor()
        for a, s in sensor_actors:
            if "lane_invasion" in s.blueprint:
                lane_monitor.attach(a)
                logger.info("Lane-invasion monitor attached (blinks on line crossings).")

        # Route.
        planner = RoutePlanner(world, resolution=carla_cfg.route_resolution,
                               seed=carla_cfg.seed)
        ego.apply_control(carla.VehicleControl(hand_brake=False))
        route_end_buffer_m = 0.0      # reference-only road appended past a file route's goal
        real_route_len_m = 0.0
        route_loops = False           # restart-on-arrival applies only to routes that truly close
        if route_obj is not None:
            route = planner.file_route(route_obj)
            # `route.loop: true` (the config default) restarts the reference on arrival, which is
            # correct only for a route that actually closes (tail back at the head, e.g. a lap).
            # For a point-to-point route the restart re-frames the reference onto the route head
            # every tick once within 100 m of the goal, so the controller would chase a reference
            # pointing back at the start and react erratically near the goal.
            head = route[0][0].transform.location
            tail = route[-1][0].transform.location
            route_loops = bool(carla_cfg.route_loop) and head.distance(tail) < 30.0
            if carla_cfg.route_loop and not route_loops:
                logger.info("Route does not close (head-tail %.0f m) -> completing at the goal "
                            "(route.loop only applies to closed-loop routes).", head.distance(tail))
            if not route_loops:
                # Always give a file route ~120 m of real road past its goal. If the reference ends
                # at the goal, the horizon folds onto the last point as the car arrives (degenerate
                # QP -> ACADOS_MINSTEP -> MPC/MPCC stop reacting just before the goal). The buffer is
                # reference-only: a deterministic forward lane-following stub (no GRP, no cusp; the
                # goal stays put), the run stops at the real goal (completion check in the loop), and
                # it is trimmed from the published viz -- never driven, never shown.
                real_route_len_m = planner.remaining_distance(ego.get_location())
                try:
                    route = planner.extend_forward(EXTEND_MARGIN_M + 20.0)
                    route_end_buffer_m = max(0.0, planner.remaining_distance(ego.get_location())
                                             - real_route_len_m)
                except Exception as e:  # noqa: BLE001  dead-end past the goal: degrade, don't abort
                    logger.warning("Route-end buffer failed (%s); reference ends at the goal.", e)
        else:
            origin = carla_map.get_waypoint(ego.get_location()).transform.location
            route = planner.random_route(origin)
            # Buffer: append ~100 m past the curated endpoint so a bounded spawn+seed run
            # (route-extend false) ends at max_distance while still cruising, instead of
            # decelerating to the route-end stop. That stop pollutes short-route speed metrics
            # (e.g. the double-lane-change: avg 22 vs a real cruise ~69).
            if str(args.route_extend).lower() == "false" and args.max_distance > 0:
                route = planner.extend_random(ego.get_location(), min_length_m=EXTEND_MARGIN_M)
        dt = carla_cfg.fixed_delta_seconds
        controller = build_controller(args.controller, ego, ctrl_cfg, dt,
                                      carla_cfg.target_speed_kmh)
        controller.set_reference(route)

        stamp0 = sim_time_to_rostime(world.get_snapshot().timestamp.elapsed_seconds)
        # Display only the real route: the route-end buffer (reference-only road past the goal) is
        # cut from the published reference and heatmaps. trim is a no-op when there is no buffer.
        rxyz = trim_by_arc(route_xyz(route), real_route_len_m if route_end_buffer_m > 0 else 0.0)
        # The reference (lane z) and the driven path (ego origin z) start at different heights;
        # shift the speed heatmap onto the driven path by their constant start difference.
        node._z_offset = float(ego.get_location().z - rxyz[0][2]) if rxyz else 0.0
        node.publish_reference(rxyz, stamp0)
        node.publish_heatmap(trim_by_arc(controller.trajectory_heatmap(),
                                         real_route_len_m if route_end_buffer_m > 0 else 0.0), stamp0)
        if planner.goal is not None:
            node.publish_goal(planner.goal.x, planner.goal.y, stamp0)

        # Warm-up hold: keep the brake on for the configured number of seconds before driving.
        warmup_s = args.warmup if args.warmup >= 0 else carla_cfg.warmup_seconds
        for _ in range(int(warmup_s / dt)):
            ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            world.tick()
        ego.apply_control(carla.VehicleControl(hand_brake=False))

        # ---- main control loop ------------------------------------------------
        rt = carla_cfg.realtime_factor
        target_dt = (dt / rt) if rt and rt > 0 else 0.0
        next_t = time.perf_counter()
        logger.info("Driving with %s. Ctrl-C to stop%s.", args.controller,
                    (f" (auto-stop at {args.max_distance:.0f} m)" if args.max_distance > 0
                     else f" (auto-stop at {args.max_duration:.0f} s)" if args.max_duration > 0
                     else ""))
        ticks = 0
        last_lane_cnt = 0
        real_lane_t = -1.0e9        # time of the last real (un-masked) lane crossing
        real_lane_label = ""
        real_lane_count = 0         # real marking crossings + off-lane departures (metric + blink)
        last_cte_abs = 0.0          # previous |cross-track| for rising-edge departure detection
        dist_traveled = 0.0
        prev_xy = None
        t_start = None
        while rclpy.ok():
            world.tick()
            snap = world.get_snapshot()
            stamp = sim_time_to_rostime(snap.timestamp.elapsed_seconds)
            state = make_state(ego, snap.timestamp.elapsed_seconds)

            cmd = controller.step(state).clamp()
            ego.apply_control(carla.VehicleControl(
                throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer))

            dbg = controller.get_debug()
            node.publish_scalars(
                speed_kmh=state.speed * 3.6,
                target_speed_kmh=dbg.target_speed_kmh,
                throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer,
                cross_track_error=dbg.cross_track_error_m,
                heading_error_deg=dbg.heading_error_deg,
                solve_time_ms=dbg.solve_time_ms)
            # Always publish (empty list for PID) so the latched horizon clears under controllers
            # that have no prediction -- otherwise a previous MPCC run's orange line lingers.
            node.publish_predicted(dbg.predicted_path_xy or [], stamp)

            if cam_recorder is not None:        # snapshot pose + paths for the in-process camera overlay
                _tf = ego.get_transform()       # CARLA transform -> ROS world<-body (negate Y, pitch, yaw)
                node._overlay_state = (
                    euler_deg_to_matrix(_tf.rotation.roll, -_tf.rotation.pitch, -_tf.rotation.yaw),
                    np.array([_tf.location.x, -_tf.location.y, _tf.location.z]),
                    getattr(node, "_ref_xyz", None),
                    list(dbg.predicted_path_xy or []))

            # Lane crossing: count and blink only real departures. A crossing belongs to the
            # reference (legit), not the car, when the car is on its trajectory, so mask it. Two
            # ways to be on trajectory: (a) small cross-track (the car follows the path through
            # whatever it does -- a lane change or a junction turn that legitimately crosses a
            # marking); (b) a detected reference lane change (CHANGELANE / lane_id shift) even with
            # some tracking lag. A corner-cut or drift leaves the path (large cross-track, no lane
            # change) and still counts. Pure RoadOption masking cannot see case (a).
            now_s = snap.timestamp.elapsed_seconds
            _, _, lane_label, lane_cnt = lane_monitor.state(now_s)
            if lane_cnt != last_lane_cnt:
                cte = abs(dbg.cross_track_error_m) if dbg.cross_track_error_m is not None else 0.0
                on_path = cte < LANE_ON_PATH_M
                ref_lane_change = cte < LANE_LAG_M and planner.in_lane_change(ego.get_location())
                if not (on_path or ref_lane_change):    # a real crossing, not the reference's own
                    real_lane_t, real_lane_label = now_s, lane_label
                    real_lane_count += 1
                    node.pub_lane_type.publish(String(data=lane_label))
                    logger.info("Lane crossed: %s (off path %.2f m)", lane_label, cte)
                last_lane_cnt = lane_cnt
            # Off-lane departure: a reckless corner-cut bulges off the lane mid-curve without
            # crossing a marking, so the crossing count misses it. Blink and count on the rising
            # edge of |cross-track| past a clear off-lane threshold; the cooldown avoids
            # double-counting an already-counted marking crossing.
            cte_abs_now = abs(dbg.cross_track_error_m) if dbg.cross_track_error_m is not None else 0.0
            if cte_abs_now > 1.5 and last_cte_abs <= 1.5 and (now_s - real_lane_t) > 1.0:
                real_lane_t, real_lane_label = now_s, f"off-lane {cte_abs_now:.1f} m"
                real_lane_count += 1
                node.pub_lane_type.publish(String(data=real_lane_label))
                logger.info("Off-lane departure: %.1f m (curve-cut/drift)", cte_abs_now)
            last_cte_abs = cte_abs_now
            dt_evt = now_s - real_lane_t                 # blink window measured from real events
            active = 0.0 <= dt_evt < 2.0
            blink_on = active and int(dt_evt * 8.0) % 2 == 0
            lx, ly = carla_to_ros_xy(state.x, state.y)
            node.publish_lane_invasion(active, blink_on, real_lane_label, (lx, ly), state.z, stamp)
            lane_total = real_lane_count

            # Accumulate the human-readable run log (cheap append, written at shutdown).
            nan = float("nan")
            # Measured lateral accel from CARLA's physics (body frame), not differentiated from
            # yaw, so the jerk metric is a single derivative of a direct signal rather than the
            # noise-amplified, speed-biased double-diff of yaw.
            acc = ego.get_acceleration()
            a_lat_meas = -math.sin(state.yaw) * acc.x + math.cos(state.yaw) * acc.y
            run_logger.record(
                t=snap.timestamp.elapsed_seconds, x=state.x, y=state.y, yaw=state.yaw,
                speed_kmh=state.speed * 3.6,
                target_speed_kmh=dbg.target_speed_kmh if dbg.target_speed_kmh is not None else nan,
                throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer,
                cross_track_m=dbg.cross_track_error_m if dbg.cross_track_error_m is not None else nan,
                heading_error_deg=dbg.heading_error_deg if dbg.heading_error_deg is not None else nan,
                solve_time_ms=dbg.solve_time_ms if dbg.solve_time_ms is not None else nan,
                lat_accel_ms2=a_lat_meas,
                # Real (masked) count, equal to the metric; the raw sensor count includes on-path
                # dashed crossings and would yield false circles for MPC/MPCC.
                lane_invasions_total=lane_total)

            # Continuous driving (random mode): extend the route through the old endpoint before
            # the end enters the controller's look-ahead, so the car never decelerates to a stop
            # at a target and flows through it instead. The margin exceeds the MPC horizon
            # distance (N*dt*v_max).
            if str(args.route_extend).lower() != "false" and route_obj is None \
                    and planner.remaining_distance(ego.get_location()) < EXTEND_MARGIN_M:
                route = planner.extend_random(ego.get_location(), min_length_m=EXTEND_MARGIN_M)
                controller.set_reference(route)
                node.publish_reference(route_xyz(route), stamp)
                node.publish_heatmap(controller.trajectory_heatmap(), stamp)
                node.publish_goal(planner.goal.x, planner.goal.y, stamp)
            elif route_obj is not None and route_loops \
                    and planner.remaining_distance(ego.get_location()) < EXTEND_MARGIN_M:
                controller.set_reference(route)              # restart the fixed closed-loop route
                node.publish_reference(route_xyz(route), stamp)
                node.publish_heatmap(controller.trajectory_heatmap(), stamp)

            # Real-time pacing.
            if target_dt > 0.0:
                next_t += target_dt
                slp = next_t - time.perf_counter()
                if slp > 0:
                    time.sleep(slp)
                else:
                    next_t = time.perf_counter()
            ticks += 1
            if ticks % 2000 == 0:
                logger.info("ticks=%d  speed=%.1f km/h  dist=%.0f m", ticks,
                            state.speed * 3.6, dist_traveled)

            # Bounded run (benchmarking): stop after max distance/duration; the finally block
            # flushes the log and we exit cleanly.
            if prev_xy is not None:
                dist_traveled += math.hypot(state.x - prev_xy[0], state.y - prev_xy[1])
            prev_xy = (state.x, state.y)
            if t_start is None:
                t_start = snap.timestamp.elapsed_seconds
            if args.max_distance > 0 and dist_traveled >= args.max_distance:
                logger.info("Reached max distance %.0f m -> stopping.", args.max_distance)
                break
            # File-route completion: stop at the real goal, i.e. remaining route minus the appended
            # reference-only buffer < 1 m. Cannot be skipped: past the goal the difference goes
            # negative (still < 1), so worst case the car stops a metre or two past the goal, never
            # early. Fires for any route_extend (file routes are never extended mid-run);
            # route_loop restarts instead (handled above).
            if route_obj is not None and not route_loops \
                    and planner.remaining_distance(ego.get_location()) - route_end_buffer_m < 1.0:
                logger.info("Route complete (%.0f m driven) -> stopping.", dist_traveled)
                break
            # No-extend (scenario) runs: stop cleanly when the curated route is exhausted, so the
            # car doesn't stall at the end (target speed -> 0) and crawl to the outer timeout.
            # Without this a route shorter than --max-distance never ends.
            if str(args.route_extend).lower() == "false" and route_obj is None \
                    and planner.remaining_distance(ego.get_location()) < 12.0:
                logger.info("Reached end of route (%.0f m driven) -> stopping.", dist_traveled)
                break
            if args.max_duration > 0 and (snap.timestamp.elapsed_seconds - t_start) >= args.max_duration:
                logger.info("Reached max duration %.0f s -> stopping.", args.max_duration)
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Finalize the camera video first: a quick .release() (frames are already streamed to
        # disk), so a Ctrl+C can't lose it behind the slower replay.gif/metrics write below.
        if cam_recorder is not None:
            cam_recorder.close()
            if cam_recorder.n_frames:
                logger.info("Camera video -> %s (%d frames)", cam_recorder.path, cam_recorder.n_frames)
        # Persist the run's CORE data (metrics.json + series.csv + summary.txt) BEFORE touching
        # CARLA. Actor destroy below can hit a server time-out that surfaces as a C++
        # TimeoutException -> std::terminate, aborting the process uncatchably (Python `except`
        # can't trap it); writing first means such an abort no longer discards the run. The
        # benchmark only needs metrics.json + series.csv, both written here, and this part is fast
        # so the window where actors still exist during a write is negligible.
        metrics = None
        if len(run_logger) > 1:
            try:
                scenario["lane_invasions"] = lane_total
                metrics = run_logger.write_core(log_dir, scenario)
                logger.info("Run log -> %s/ (%d samples)", log_dir, len(run_logger))
            except (Exception, KeyboardInterrupt) as e:  # noqa: BLE001
                logger.warning("run-log core write interrupted/failed: %s", e)  # camera.mp4 already saved
        # Destroy actors and restore world settings. rclpy traps SIGINT and SIGTERM, so the process
        # can't self-terminate while matplotlib encodes the replay.gif below, and `ros2 launch`
        # escalates SIGINT->SIGTERM->SIGKILL after ~15 s. Cleaning up before the slow plot/GIF render
        # means a SIGKILL mid-encode still leaves no orphan ego frozen in the world.
        try:
            world.apply_settings(original_settings)   # restore async first: server free-runs so the
        except Exception:  # noqa: BLE001            # destroy() below applies at once (no tick wait)
            pass
        logger.info("Cleaning up %d actors...", len(actors))
        for a in reversed(actors):
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        # Slow, expendable artifacts (plots, replay.gif) AFTER cleanup: the world is clean and the
        # core data is already on disk, so a failure or kill here costs only the figures, not the run.
        if metrics is not None:
            try:
                run_logger.write_artifacts(log_dir, scenario)
                print("\n" + summary_text(metrics))
            except (Exception, KeyboardInterrupt) as e:  # noqa: BLE001
                logger.warning("run-log artifacts write interrupted/failed: %s", e)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
