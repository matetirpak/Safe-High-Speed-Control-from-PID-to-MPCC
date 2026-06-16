# Architecture

`carla_mpc_bringup` is a ROS 2 Humble package that drives a Carla 0.9.16 ego
along a reference route with a swappable controller (PID, PID++, MPC, or MPCC),
streams telemetry and 3D visualization to Foxglove, and logs every run for
offline evaluation. This document describes how the running system is wired:
the processes, the controller seam, the data flow, the TF tree, and the
coordinate conventions.

## Components

There are three kinds of processes: the **simulator**, **our package's nodes**,
and the **Foxglove** bridge.

### Simulator

- **Carla server** (`./CarlaUE4.sh --ros2`). With `--ros2` it runs a *native*
  Fast-DDS publisher inside the simulator (no `carla_ros_bridge`). It publishes
  every opted-in sensor, `/clock`, and its own `/tf` (`hero → {sensors}`), and it
  accepts vehicle control applied through the Python API.

### Our package (`carla_mpc_bringup`)

| Process | Type | Role |
|---|---|---|
| `control_node` | rclpy node **+** Carla Python API | The active core. Connects to Carla, sets synchronous mode, spawns the ego + sensor rig, builds a route, and **is the single world ticker**: every tick it reads the ego state, asks the active controller for a command, applies it to Carla, and republishes the reference path + goal point + control telemetry. |
| `gt_pose_publisher` | rclpy node (+ Carla API) | Read-only. Publishes the **actual** driven path/odom (`/carla/hero/gt/{path,odom}`) and the `gt_map → hero` TF anchor that lets the live sensor data render. |
| `ego_model_publisher` | rclpy node | The 3D car you see in Foxglove: a `MarkerArray` in the `hero` frame (`frame_locked`, so it rides the live anchor). |
| `predicted_path_overlay` | rclpy node | Projects the predicted (orange) + reference (blue) paths into the front camera → `/controller/camera_overlay/image`. **Time-syncs** the ego pose to the image stamp (buffers odom, interpolates) so the projection doesn't slide on the road. |
| `RunLogger` (`eval/run_logger.py`) | in `control_node` | Writes the **human-readable run log** to `logs/runs/<run>/` every run: metrics.json + series.csv + plots + summary.txt. `eval/metrics.py` and `eval/benchmark.py` are pure and reused by `run_benchmark` / `benchmark_report` (→ timestamped `logs/benchmarks/<ts>/`). See [evaluation.md](evaluation.md). |
| `record_run` / `review_run` | tools | **Opt-in** rosbag (`record:=true`) → `runs/` for Foxglove replay; review it offline. See [recording.md](recording.md). |

The controllers live in `control/` and are **ROS-free**, behind
`ControllerInterface` (`control/controller_interface.py`). A single launch flag,
`controller:=pid|pidpp|mpc|mpcc`, selects which one `control_node` builds; the
default is `pid`. Each implements `set_reference(route)`, `step(VehicleState) →
ControlCommand`, and optionally `get_debug()` / `trajectory_heatmap()`.

| Class | File | Role |
|---|---|---|
| `PIDController` | `control/pid_controller.py` | Curvature-aware PID path follower (`controller:=pid`, baseline). Wraps CARLA's `agents.navigation` PID law. Its curve-speed profile, steering look-ahead, and cross-track are **precomputed once per trajectory** in `AnnotatedTrajectory` (no per-tick route scan). |
| `PIDPlusController` | `control/pidplus_controller.py` | Stronger self-contained PID (`controller:=pidpp`). Subclasses `PIDController` and adds a lateral curvature feed-forward, a Stanley cross-track term, and a steer-rate limit, plus a self-contained longitudinal speed PID with filtered/clamped derivative and anti-windup. Config-gated under `pidpp:` (degrades to the baseline look-ahead PID with the add-ons zeroed). |
| `AnnotatedTrajectory` | `control/trajectory.py` | **PID-only** precompute: arc-length spline + curvature + sharpest-angle-ahead profile + cross-track / arc-length lookups + the speed heatmap. Rebuilt on each `set_reference` (forward-local projection survives the appended route). The MPC/MPCC use `ReferencePath` instead. |
| `ReferencePath` | `control/reference_path.py` | **MPC/MPCC** arc-length reference: route (x, y) → `s → (x, y, psi, kappa, v_ref)`, with a friction-limited two-pass speed profile (`v = sqrt(a_lat_max / |kappa|)`, capped at cruise, smoothed backward to brake before curves and forward to accelerate out). `window()` returns the next `N+1` reference states the OCP tracks. |
| `MPCController` | `control/mpc_controller.py` | acados SQP-RTI tracking MPC (`controller:=mpc`). Default **dynamic bicycle** model (`model: dyn`; `kin_cg` kinematic fallback, also `kin_rear` / `blended`). See [mpc.md](mpc.md) and [dynamics.md](dynamics.md). |
| `MPCCController` | `control/mpcc_controller.py` | acados SQP-RTI **MPCC** (`controller:=mpcc`): dynamic bicycle + path-progress `theta`, contouring + progress; path passed as per-stage arc-linearised params (no regen). Curve speed via a friction-limited `v_theta` feed-forward. |
| `RoutePlanner` | `sim/route_planner.py` | CARLA `GlobalRoutePlanner` wrapper: random continuous routes (extended **forward** through the endpoint) or a fixed Leaderboard route (parsed by `sim/route_loader.py`). Forward-local progress tracking. |
| `LaneInvasionMonitor` | `sim/lane_invasion.py` | Listens to the `lane_invasion` event sensor (no native ROS2) and turns line crossings into a blinking Bool + flashing 3D marker. |

### Foxglove

- **foxglove_bridge**: WebSocket server on `:8765` exposing the whole ROS graph.
  Import `foxglove/carla_mpc_layout.json` for the control dashboard.
- **Lane-crossing indicator**: an **Indicator** panel bound to
  `/controller/lane_invasion` (Bool) flashes red ("⚠ LINE CROSSED") when the car
  crosses a lane marking, plus a flashing 3D text marker over the car. The marking
  type (Solid/Broken/…) is logged + published on `/controller/lane_invasion_type`.
- **Trajectory heatmap** is the precomputed annotation a controller will drive,
  published as colour-coded polylines: `/controller/heatmap_speed` (path coloured by
  the planned speed, jet: blue=slow → red=fast) and `/controller/heatmap_curve`
  (coloured by curve angle / curvature, toggle in the 3D panel). Latched; refreshed
  on each route extension. For the PID it's the precomputed curve-speed + angle; for
  the MPC it's the friction-limited reference speed + curvature.
- **Live telemetry plots**: Plot panels track the control commands
  (`/controller/{throttle,brake,steer}`), measured vs target speed
  (`/controller/{speed_kmh,target_speed_kmh}`), the tracking errors
  (`/controller/{cross_track_error,heading_error_deg}`), and the controller solve
  time (`/controller/solve_time_ms`). All are `std_msgs/Float32`.

## Why control runs *inside* the ticker

In **synchronous mode** the control command must be applied immediately before
`world.tick()`. If the controller were a separate node, its command would arrive
asynchronously and race the tick: sometimes a tick fires with a stale command,
producing jitter and non-determinism. So `control_node` computes and applies the
command in the same loop that ticks the world. The controller is still decoupled
(ROS-free, swappable behind `ControllerInterface`); only the *thin glue* between
CARLA and the controller lives in `control_node`. This is also why we keep
`gt_pose_publisher` separate and read-only: it never ticks.

> Note: CARLA 0.9.16's native ROS2 *vehicle control subscription* is known to be
> unreliable (carla#9408), so we apply control via the Python API on the actor we
> already hold, which is the robust path.

## Data flow

```text
                 ┌──────────────────── CARLA server (--ros2) ────────────────────┐
                 │  /carla/hero/cam_front/image (+/camera_info)   (20 Hz)         │
 control_node ──▶│  /clock      /tf (hero → sensors)                              │
  apply_control  │                                                               │
  + world.tick() └───────┬───────────────────────────────────────┬───────────────┘
                         │ ego state (Carla API)                  │ vehicle pose (Carla API)
                         ▼                                        ▼
                   controller.step(VehicleState)            gt_pose_publisher
                   └─ ControlCommand → apply_control        ├─ /carla/hero/gt/{path,odom}
                   └─ ControllerDebug → publish:            └─ TF gt_map → hero
                      /controller/reference_path_3d (Path)
                      /controller/predicted_path (Path, MPC/MPCC)
                      /controller/goal_point (Marker)
                      /controller/lane_invasion (Bool, blink) +
                        /controller/lane_invasion_marker (Marker) + /controller/lane_invasion_type (String)
                      /controller/heatmap_{speed,curve} (Marker, colour-coded path)
                      /controller/{speed_kmh,target_speed_kmh,
                                   throttle,brake,steer,
                                   cross_track_error,heading_error_deg,
                                   solve_time_ms} (Float32)
                                          │
                   ego_model_publisher ─ /ego_model (MarkerArray, frame `hero`)
                                          │
                         everything ─────▶ foxglove_bridge :8765 ─▶ Foxglove
```

## The TF tree

CARLA's native publisher broadcasts the rigid sensor tree `hero → {sensors}`.
`gt_pose_publisher` anchors that tree to the world by publishing `gt_map → hero`
from ground truth every sample. So:

```text
gt_map ──(gt_pose_publisher, live)──▶ hero ──(CARLA native /tf)──▶ cam_front, collision, lane_invasion, ...
```

- **`gt_map`** is the CARLA world expressed in ROS convention (REP-103).
- **`hero`** is the ego body; the ego car model and all sensors hang off it.
- All path topics (`/carla/hero/gt/path`, `/controller/reference_path_3d`,
  `/controller/predicted_path`) are published **in `gt_map`**, so they overlay
  directly. The Foxglove 3D panel follows `hero` (chase view) while drawing those
  `gt_map` paths through the `gt_map → hero` edge.

There is no estimator here, so there is no `global` frame and no one-shot
alignment. `gt_map → hero` is the whole story.

## Coordinate conventions

| Frame | Handedness | Axes | Units |
|---|---|---|---|
| Carla world/actor | left-handed | X fwd, Y **right**, Z up | degrees |
| ROS body / `gt_map` (REP-103) | right-handed | X fwd, Y left, Z up | radians |

The conversion is a single rule applied at the I/O boundary: **negate Y** (and
negate pitch/yaw) when going Carla → ROS. Sensor mounts in `vehicle.yaml` are
authored in ROS convention and converted at spawn by `ros_mount_to_carla` in
[`core/frame_conventions.py`](../src/carla_mpc_bringup/carla_mpc_bringup/core/frame_conventions.py)
(which also holds the rotation/quaternion helpers and is unit-checkable without a
running sim). The controller's route geometry runs **in the CARLA frame** (because
the route waypoints and CARLA's PID are in CARLA coordinates); only the *published*
paths/markers are converted to `gt_map`, by the `carla_to_ros_xy` helper in
[`sim/control_node.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/control_node.py).
