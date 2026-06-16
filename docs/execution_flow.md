# Execution Flow

What actually happens, in order, when you run the pipeline, and which file does
each thing. The single source of truth is
[`control_node.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/control_node.py),
which owns the CARLA world tick, the controller, and most of the telemetry.

## 0. Prerequisite: CARLA is already running

```bash
cd "$CARLA_ROOT" && ./CarlaUE4.sh -RenderOffScreen --ros2
```

`--ros2` turns on CARLA's native Fast-DDS publisher (the source of the
`/carla/hero/...` sensor topics, `/clock`, and `/tf`). Start it **first** so
`/clock` is available (transient-local) to nodes that join later.

**`--ros2` is mandatory.** The pipeline drives CARLA over the Python API, which
works **without** `--ros2`, so the world ticks, the ego moves, and the run looks
healthy. But the native publisher is the *only* source of the sensor streams:
without `--ros2` the camera, the predicted-path overlay, and `/clock` silently
have **no** data. If the front camera or the projected-path overlay is blank,
this missing flag is the first thing to check.

## 1. `ros2 launch carla_mpc_bringup carla_pid.launch.py`

[`launch/carla_pid.launch.py`](../src/carla_mpc_bringup/launch/carla_pid.launch.py)
is the orchestrator. It includes / starts:

- [`launch/spawn.launch.py`](../src/carla_mpc_bringup/launch/spawn.launch.py) →
  `control_node`, `ego_model_publisher`, and (unless `publish_gt:=false`)
  `gt_pose_publisher`.
- `predicted_path_overlay`: projects the predicted (orange) and reference (blue)
  paths into the front-camera image, republished on
  `/controller/camera_overlay/image`.
- [`launch/foxglove.launch.py`](../src/carla_mpc_bringup/launch/foxglove.launch.py) →
  `foxglove_bridge` on port `8765` (skip with `with_foxglove:=false`).
- `record_run`: only when `record:=true`; writes a rosbag for later Foxglove
  replay. Off by default (every run already writes metrics + plots to `logs/`).

Key args:

| Argument | Default | Meaning |
|---|---|---|
| `controller:=` | `pid` | which controller drives: `pid`, `pidpp`, `mpc`, `mpcc`, or a speed-capped variant like `mpcc@100` |
| `condition:=` | `''` | named condition from [`config/conditions.yaml`](../src/carla_mpc_bringup/config/conditions.yaml) (`''` = the weather in `carla.yaml`) |
| `sensors:=` | `''` | comma-separated sensor whitelist (`''` = all enabled in `vehicle.yaml`) |
| `publish_gt:=` | `true` | run the ground-truth publisher |
| `with_foxglove:=` | `true` | run the Foxglove bridge |
| `route:=` | `''` | route XML, e.g. `routes/sharp_turns.xml` (`''` = the route in `carla.yaml`) |

Config overrides: `vehicle_config:=`, `carla_config:=`, `controller_config:=`
(absolute paths to alternative YAMLs).

The benchmark knobs `map:=`, `spawn_index:=`, `seed:=`, `max_distance:=`, and
`route_extend:=true|false` are forwarded end-to-end (`carla_pid.launch.py` →
`spawn.launch.py` → `control_node` CLI args), so `run_benchmark` can pin a
deterministic route.

## 2. `control_node` builds the world and drives it

File: [`control_node.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/control_node.py).
It is an rclpy node that also holds the CARLA actor and ticks the world. Sequence:

1. **Load config**: `load_vehicle_config`, `load_carla_config`,
   `load_controller_config` (see
   [`config_loader.py`](../src/carla_mpc_bringup/carla_mpc_bringup/core/config_loader.py)).
2. **Resolve the route source**: `--route` (or, if unset, `carla.yaml` `route.file`
   when `route.mode == file`) is parsed via
   [`route_loader.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/route_loader.py);
   the route XML's town overrides `world.map`.
3. **World setup**: load the map if needed, set **synchronous mode** with
   `fixed_delta_seconds` (default 0.05 s = 20 Hz), apply weather. Rendering goes
   off automatically when no camera is in the rig (unless `no_rendering` or
   `--record-camera` forces it).
4. **Spawn the ego**: at the route start (file mode) or a map spawn point (random
   mode); `role_name` / `ros_name` = `hero` (mandatory, because CARLA's native publisher
   only fills attached-sensor topics for the ego named `hero`).
5. **Spawn sensors**: for every enabled sensor, `spawn_sensor()`
   ([`sensor_factory.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/sensor_factory.py))
   with ROS publishing **deferred** (`enable_ros=False`); tick a few times to
   settle the spawn drop; then `enable_sensor_ros()`.
6. **Build the route**: `RoutePlanner`
   ([`route_planner.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/route_planner.py))
   traces a lane-level route (`random_route` to a random goal, or `file_route`)
   as a list of `(carla.Waypoint, RoadOption)`.
7. **Build the controller**: `build_controller()` constructs the `PIDController`,
   `PIDPlusController`, `MPCController`, or `MPCCController` and calls
   `set_reference(route)`.
8. **Publish the reference once**: the 3D reference path (route geometry at real
   terrain z) goes out latched on `/controller/reference_path_3d`, and the
   speed/curvature heatmaps on `/controller/heatmap_speed` and
   `/controller/heatmap_curve`. The blue path you see in the 3D scene is the speed
   heatmap; the overlay node uses `/controller/reference_path_3d` for projection.
9. **Warm-up**: hold the ego still (brake) for `warmup_seconds` (config default
   3 s, overridable with `--warmup` / `warmup:=`), ticking, so the drop settles
   and Foxglove boots before driving.
10. **Main loop (the receding-horizon pattern)**. Every iteration:
    `world.tick()` → read the ego state into a `VehicleState` →
    `controller.step(state)` → `ego.apply_control(...)` → publish telemetry, the
    predicted horizon, and lane-invasion state. In **random** mode, on nearing
    the goal it draws a new one and re-publishes the reference (continuous town
    driving). Real-time pacing caps the loop to `realtime_factor`.

### Stop conditions

The loop ends on `Ctrl-C` or any of the following, after which the `finally`
block flushes the run log:

- `--max-distance` reached (driven metres), for bounded, replicable benchmark runs.
- `--max-duration` reached (sim seconds).
- **File routes complete automatically.** At startup ~120 m of real road is
  appended past the goal as a REFERENCE-ONLY buffer: if the reference ended at the
  goal, the MPC/MPCC horizon would fold onto the last point as the car arrives
  (degenerate QP / `ACADOS_MINSTEP`) and the controller would stop reacting just
  before the goal. The run stops AT the real goal (remaining route minus buffer
  `< 1 m`); the buffer is never driven and is trimmed from the published
  reference/heatmaps, so `--max-distance` is not needed.
- **Curated random-mode scenarios** (`--route-extend false`) stop cleanly once the
  fixed route is exhausted (remaining distance `< 12 m`), so a route shorter than
  `--max-distance` actually ends instead of the ego stalling at a zero target
  speed and crawling to the outer timeout.

When `control_node` exits (run complete or crash), the **whole launch shuts down
by itself** (`on_exit=Shutdown()` on the `control_node` action in
`spawn.launch.py`): no `Ctrl-C` needed; the Foxglove view keeps the last received
scene, so the finished trajectory stays visible.

## 3. CARLA publishes the rig

Because the ego is `hero` and each sensor called `enable_sensor_ros()`, CARLA emits
`/carla/hero/cam_front/image` (+ `/camera_info`), `/clock`, and `/tf`
(`hero → sensors`). The default rig is a front camera plus collision and
lane-invasion sensors, configured in
[`vehicle.yaml`](../src/carla_mpc_bringup/config/vehicle.yaml). The lane-invasion sensor has no native ROS2 publisher in 0.9.16, so `control_node`
listens to it in-process (see
[`lane_invasion.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/lane_invasion.py))
and blinks lane crossings into Foxglove; the collision sensor uses CARLA's native
publisher like the camera.

## 4. `gt_pose_publisher` provides the actual trajectory + anchor

File: [`gt_pose_publisher.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/gt_pose_publisher.py).
A polling loop reads the CARLA ego pose (read-only; it never ticks) and publishes,
in `gt_map`:

- `/carla/hero/gt/path`: the **actual** driven trajectory (compare against the
  reference),
- `/carla/hero/gt/odom`: instantaneous pose + body twist,
- TF `gt_map → hero`: anchors CARLA's `hero → {sensors}` tree so the live camera
  data renders in 3D.

It exits cleanly once the ego is destroyed (run complete), freezing the published
trajectory where it ended.

## 5. `ego_model_publisher` + Foxglove

`ego_model_publisher` latches a `MarkerArray` car in `hero` (markers are
`frame_locked`, so they follow `gt_map → hero`).

`foxglove_bridge` exposes the whole graph; import
[`foxglove/carla_mpc_layout.json`](../src/carla_mpc_bringup/foxglove/carla_mpc_layout.json)
and you get the 3D scene (driven vs reference, the car, the goal, the predicted
horizon), the front camera, and live plots of speed vs target, the
throttle/brake/steer commands, the tracking errors (cross-track, heading), and the
solver/PID telemetry.

## Timing summary

| Thing | Rate | Set by |
|---|---|---|
| World tick / control loop / `/clock` | 20 Hz | `carla.yaml` `world.fixed_delta_seconds = 0.05` |
| Front camera | 20 Hz | `vehicle.yaml` `cam_front` `sensor_tick: 0.05` |
| Reference path + heatmaps | on route change (latched) | `control_node` |
| Control telemetry + predicted horizon | per tick | `control_node` |
| Ground-truth path / odom / TF | `--rate` (default 100 Hz) | `gt_pose_publisher` |

Everything except `gt_pose_publisher`'s own polling clock is driven off CARLA's
synchronous tick, so the control path is deterministic. The control loop rate
**equals** the tick rate, and the PID gains are coupled to `dt`. Retune them if you
change it.
