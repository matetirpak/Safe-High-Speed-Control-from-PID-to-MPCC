# Configuration

All runtime configuration lives in five YAML files under
[`config/`](../src/carla_mpc_bringup/config), plus reusable route XMLs in
[`config/routes/`](../src/carla_mpc_bringup/config/routes):

| File | Loaded into | Covers |
|---|---|---|
| `carla.yaml` | `CarlaConfig` | Server connection, world, route source. |
| `controller.yaml` | `ControllerConfig` | PID / PID++ / MPC / MPCC gains and speed policy. |
| `vehicle.yaml` | `VehicleConfig` | Ego blueprint + sensor rig. |
| `conditions.yaml` | (weather) | Named weather / time-of-day presets. |
| `scenarios.yaml` | (benchmark) | The curated deterministic benchmark suite. |

The first three are parsed by
[`config_loader.py`](../src/carla_mpc_bringup/carla_mpc_bringup/core/config_loader.py)
into dataclasses (the single source of truth). To sanity-check the vehicle rig
and world/route without a running sim:

```bash
# prints the parsed rig + world + route (defaults to config/vehicle.yaml + config/carla.yaml)
python -m carla_mpc_bringup.core.config_loader [vehicle.yaml] [carla.yaml]
```

Launch arguments and benchmark CLI flags can override most of these values
per-run without editing the files; the per-file sections below note which.

---

## `config/carla.yaml`: the world & route source

Read by `control_node`. Synchronous fixed-step mode is mandatory: the control
command is applied right before each `world.tick()`, so control is deterministic
and race-free, and the control loop runs at the tick rate.

| Key | Meaning |
|---|---|
| `client.host` / `port` / `timeout` | Carla connection (default `localhost:2000`, 20 s timeout). |
| `world.map` | Town to load (default `Town05`). Overridden by a file route's town. |
| `world.synchronous_mode` | **Keep `true`.** Control is applied before each tick; async mode races it. |
| `world.fixed_delta_seconds` | Tick period = **control loop period**. `0.05` (20 Hz) default; `0.02` (50 Hz) is smoother at high speed but heavier with cameras. **PID gains are dt-coupled.** |
| `world.realtime_factor` | Cap sim to this × real-time (`1.0` default; `0` = uncapped). |
| `route.mode` | `random` (GlobalRoutePlanner → random goals) or `file` (a fixed Leaderboard route). In `random` mode the route is **continuously extended ~100 m before its end through the current endpoint**, so the car flows through targets without ever decelerating to a stop. |
| `route.loop` | random: keep drawing goals; file: restart the route near the end, **only if the route actually closes** (tail within ~30 m of the head, e.g. a lap). A point-to-point route ignores this and completes at its goal. |
| `route.resolution` | GlobalRoutePlanner sampling resolution (m between waypoints; `2.0` default). Smaller = denser reference. |
| `route.target_speed_kmh` | `0` = use the controller's curvature-aware speed policy; `>0` = constant cruise speed (overrides the policy). |
| `route.file` / `route.id` | Used when `mode: file`, e.g. `routes/highway_speed.xml`. `id` selects a route within the XML (`""` = first). |
| `weather.preset` | A Carla preset (`ClearNoon`, …) or a named condition from `conditions.yaml`. |
| `traffic.*` | **Not used in this phase**: the controller drives, not the Traffic Manager. Background traffic is not spawned. |
| `spawn.seed` | Route RNG seed (reproducibility; default `42`). |
| `spawn.spawn_index` | Map spawn-point index for random-mode start (default `0`). |
| `spawn.warmup_seconds` | Brief brake-hold at spawn so the drop settles (and Foxglove boots) before driving. |

### Per-run bounding arguments

These gate bounded, deterministic runs and are what the scenario suite drives
the controller with. They are passed at launch / on the `control_node` CLI,
**not** stored in `carla.yaml`:

| Arg | Meaning |
|---|---|
| `route_extend` | `true` (default) keeps stitching random goals onto the route end so it never stops; `false` = clean deterministic run with no random ~100 m extension (which could splice on a 180-deg cusp no controller can follow). The scenario suite forces this `false`. |
| `max_distance` | Auto-stop after this driven distance (m); `0` (default) = unbounded. Bounds a benchmark run to a fixed road segment for replicable comparison. **Not needed for `route:=` runs**: file routes auto-complete at their goal. |
| `route` | Path to a route XML (e.g. `routes/sharp_turns.xml`); `""` falls back to the `carla.yaml` route. |

```bash
ros2 launch carla_mpc_bringup carla_pid.launch.py route_extend:=false max_distance:=200
```

File routes (`route:=` / `--route`) need no bounds: `control_node` appends
~100 m of reference-only road past the goal (so the MPC/MPCC horizon never
reaches the reference end; a clamped end degenerates the QP near the goal) and
stops the run at the **real** goal. The buffer is never driven and never
displayed.

---

## `config/controller.yaml`: gains & speed policy

This file holds four blocks, one per controller, selected with the launch
`controller:=` argument (`pid` | `pidpp` | `mpc` | `mpcc`).

### `pid:` baseline PID path follower (`PidConfig`)

The proven defaults from the `controls-for-trajectory-following` reference.

**Gains**

- `long_kp` / `long_ki` / `long_kd`: longitudinal speed PID → throttle/brake in [−1, 1] (`0.10 / 0.15 / 0.25`).
- `lat_kp` / `lat_ki` / `lat_kd`: lateral heading PID → steer in [−1, 1] (`1.20 / 0.00 / 0.10`).
- `max_throttle` / `max_brake` / `max_steer`: output clamps (all `1.0`).

**Curve-speed policy.** When `a_lat_max > 0` (the default), a forward-backward
**velocity profile** sets the curve speed from a lateral-acceleration cap,
replacing the older angle-ahead heuristic:

- `a_lat_max`: lateral-accel cap for the curve speed (`7.0`, ≈0.7 g, well under the ~15.8 grip limit). `0` falls back to the legacy heuristic.
- `curve_speed_factor`: corner at this fraction of the lateral limit (`0.85`, a safety margin).
- `a_accel`: forward-pass acceleration bound for the profile (`3.0` m/s²).

**Legacy angle-ahead heuristic** (active only when `a_lat_max: 0`). Scan the
route ahead, find the sharpest upcoming turn, and slow down before it:

- `default_target_speed_kmh`: cruise speed on straights (`120`).
- `min_speed_kmh`: floor in the sharpest curves (`10`).
- `no_slowdown_below_deg` / `max_turn_angle_deg`: the curve-angle band mapped to [default, min] speed (`15` / `90`).
- `angle_sliding_window_n`: smoothing window on the detected curve angle (`10`).
- `angle_to_speed_exponent`: `>1` brakes harder for sharp curves (`1.4`).
- `curve_safety_margin_m`, `decel_ms2`: braking-distance model before a curve (`20`, `3.0`).
- `long_lookahead_sec` / `min_long_lookahead_dist`: speed-policy look-ahead (`3.0` s, floored at `20` m).
- `lat_lookahead_sec`: steering target look-ahead (pure-pursuit-like; grows with speed; `0.5` s).

**High-steer spin-out guard** (a reference *wart*, kept for parity; MPC removes
the need for it): when `|steer| > high_steer_threshold` (`0.2`), throttle is
capped to `high_steer_throttle_cap` (`0.3`).

**Actuation smoothing**: an exponentially-weighted average of the last
`cmd_smooth_n` commands (`8`) with `cmd_smooth_decay` (`0.6`); newest command
gets the most weight, higher decay = smoother but more lag. `n <= 1` disables
it. Retune `n` if you change the tick rate.

### `pidpp:` PID++ (`controller:=pidpp`)

The baseline PID plus two near-free add-ons (see
[pid_controllers.md](pid_controllers.md)). It **inherits all `pid:` values** and
overrides only the keys below:

- `lat_kff`: lateral curvature feed-forward gain, `δ_ff = lat_kff · atan(L·κ)`, so the PID only trims the residual (`0.9`).
- `steer_rate_max`: steer-rate limit in normalized steer/s; the base PID bypasses Carla's slew limit, so it is re-added here (`2.0`).
- `stanley_k_e` / `stanley_k_soft`: optional Stanley cross-track term (`0.0` = off, low-speed softening `1.0`).
- `d_filter_alpha` / `long_d_max`: longitudinal derivative low-pass and clamp (`0.7`, `0.6`).
- `long_kff`: longitudinal feed-forward on `d(target)/tick` so it brakes on a dropping target and throttles on a rising one (`0.60`).
- `a_lat_max` / `curve_speed_factor` / `a_accel`: its own velocity-profile tuning (`14.0` / `0.85` / `5.0`).

### `mpc:` kinematic/dynamic bicycle MPC (`controller:=mpc`)

A bicycle tracking OCP solved by acados SQP-RTI. Full reference in
[mpc.md](mpc.md#tuning-knobs-configcontrolleryaml-mpc). Horizon: `N: 50` ×
`dt: 0.06` (≈3.0 s lookahead). Key knobs:

- `model`: `dyn` (dynamic bicycle, default, fixes the high-speed weave), `kin_cg`/`kin_rear` (kinematic fallbacks), or `blended`.
- `Cf` / `Cr` / `Iz`: dynamic tyre stiffness and yaw inertia (Model-3 estimates; identify from Carla with `identify_grip`).
- `target_speed_kmh` (`150`) + `a_lat_max` (`13`, the grip budget): set the speed. The MPC drives as fast as `v_curve = √(a_lat_max/κ)` allows, capped at the straight cruise.
- `curve_speed_factor` (`0.85`): corner at 85 % of the grip limit for margin; `1.0` corners *at* the limit → wide overshoot. It multiplies the grip-limited curve speed.
- `a_long_acc` / `a_long_dec` (`6.0` / `7.0`): forward-/braking-accel shaping the reachable reference pace (pre-braking before curves).
- `reference_smooth_m` / `lane_change_smooth_m` (`2.5` / `6.0`): path-xy smoothing; extra smoothing on lane-change arcs for a gentle S with no brake/jerk.
- `q_lat` / `q_lon` / `q_psi` / `q_v` (`20 / 1 / 8 / 4`): cross-track, along-track, heading, and speed tracking weights.
- `r_a` / `r_delta` / `r_ddelta` (`0.3 / 4 / 30`): accel, steer-magnitude, and steer-change penalties (`r_ddelta` kills micro chatter).

### `mpcc:` Model Predictive Contouring Control (`controller:=mpcc`)

A dynamic bicycle plus a path-progress state `θ` that maximises progress, with a
per-stage progress-speed target from the friction-limited curve speed. See
[mpc.md](mpc.md#mpcc-controllermpcc). Horizon: `N: 50` × `dt: 0.05`
(≈2.5 s lookahead). Key knobs:

- `q_c` / `q_l` (`20 / 2`): contouring (keep on path) and lag (keep `θ` tied to real arc length) weights.
- `target_speed_kmh` (`180`) + `a_lat_max` (`15`, the grip budget): feed the friction-limited curve speed.
- `curve_speed_factor` (`0.90`): same role as in MPC; do **not** raise to `1.0` (overshoot).
- `v_blend` (`4.0`): kinematic↔dynamic model blend speed for low-speed launch stability.
- `a_grip` (`16.0`) + `grip_slack` (`100`): soft friction-circle constraint radius and its slack penalty (safety backstop, set above `a_lat_max`).
- `q_vtheta` / `q_vx` (`4 / 4`): progress-speed and actual-speed tracking.
- `r_ddelta` (`65`): the main anti-wobble knob.
- `Cf` / `Cr` / `Iz`: same dynamic tyre/inertia as the MPC.

---

## `config/vehicle.yaml`: the ego + sensor rig

A **lean** rig by design (cameras are the real-time cost). The ego is a Tesla
Model 3 (`vehicle.tesla.model3`). `mount` is
in ROS body convention (REP-103: X forward, Y left, Z up; metres; degrees),
converted to Carla's left-handed frame at spawn.

| Sensor | Default | Why |
|---|---|---|
| `cam_front` | **on** | Driver view (Foxglove Image panel). 960×540 @ 20 Hz. |
| `collision` | **on** | Crash detection for control QA. |
| `lane_invasion` | **on** | Line-crossing detection. Event sensor (no native ROS2) → `control_node` Python-listens and **blinks** it in Foxglove: a `Bool` flag on `/controller/lane_invasion`, a 3D `Marker` on `/controller/lane_invasion_marker`, and the crossed-line type as a `String` on `/controller/lane_invasion_type`. |

> Note: there is no IMU sensor in the default rig; the controller works from
> ground-truth pose.

Per-sensor keys: `enabled` (spawn or not), `ros` (publish or not), `type`
(→ `sensor.<type>`), `mount`, `attributes` (Carla blueprint attributes).
`vehicle.role_name` / `ros_name` **must be `hero`**: Carla 0.9.16's native
publisher only fills the vehicle segment of attached-sensor topics for the ego
named `hero` (carla#9278). Topics follow
`/carla/<vehicle.ros_name>/<sensor.name>/<channel>`.

Select a subset at launch without editing the file:

```bash
ros2 launch carla_mpc_bringup carla_pid.launch.py sensors:=cam_front
```

---

## `config/conditions.yaml`: named weather / time-of-day

Curated `carla.WeatherParameters` presets spanning cloud/light, precipitation +
wet roads, fog/visibility, and sun elevation (noon → sunset → night), plus the
nasty composites (foggy/rainy night). Values are explicit weather fields so runs
are reproducible. Available names: `clear_noon`, `cloudy_noon`, `wet_noon`,
`soft_rain_noon`, `hard_rain_noon`, `fog_noon`, `clear_sunset`, `clear_night`,
`foggy_night`, `rainy_night`.

```bash
ros2 launch carla_mpc_bringup carla_pid.launch.py condition:=rainy_night
```

Wet/rainy/night conditions are a good stress test for a high-speed tracker:
Carla applies reduced road grip, which the controllers' fixed-stiffness tyre
model does not anticipate, exposing tracking degradation.

---

## `config/scenarios.yaml`: the curated benchmark suite

A research-level suite of deterministic, reproducible drives, consumed by
`run_benchmark --scenarios`. The benchmark loops *scenarios × controllers ×
repeats* and `benchmark_report` aggregates the results.

### Scenario groups

Every **top-level YAML list is a scenario group**, named by its key. The report
aggregates each group **separately** (its own Metrics and Speed tables, titled
with the list name and rendered as per-group plots), so different operating
regimes are not pooled into one misleading number. The shipped groups are:

- `flat_2d`: full-speed, no elevation (the default operating regime).
- `elevated_3d`: Town11 rural drives with real elevation (speed-sensitive; the default cap struggles on the curvy/hilly finish, a 100 km/h cap is clean).

Each scenario is one deterministic drive defined by an explicit-waypoint route
XML (route extension is forced **off** for repeatability):

| Field | Meaning |
|---|---|
| `name` | Tags the run (its `--condition`); the report breaks down per-scenario by it. |
| `map` | Carla town to load. |
| `route` | Explicit-waypoint XML in `config/routes/`, built with `route_lab` from atlas spawn-point numbers. The route defines the drive length, so routed scenarios need no `distance`. |

Shipped scenarios:

| Group | Scenario | Map | Route | Feature |
|---|---|---|---|---|
| `flat_2d` | `sharp_turns` | Town07 | `routes/sharp_turns.xml` | Rural tight corners. |
| `flat_2d` | `highway_speed` | Town04 | `routes/highway_speed.xml` | Sustained high-speed cruise. |
| `flat_2d` | `highway_lane_changes` | Town06 | `routes/highway_lane_changes.xml` | Multi-lane highway with lane changes. |
| `flat_2d` | `roundabout` | Town03 | `routes/roundabout.xml` | Full roundabout loop. |
| `elevated_3d` | `country_road` | Town11 | `routes/country_road.xml` | Rural drive with elevation. |
| `elevated_3d` | `curvy_downhill` | Town11 | `routes/curvy_downhill.xml` | The curvy downhill finish near sp1582. |

### Running the benchmark

```bash
ros2 run carla_mpc_bringup run_benchmark \
    --scenarios src/carla_mpc_bringup/config/scenarios.yaml \
    --controllers pid,mpc,mpcc --repeats 3
```

Useful flags:

- `--groups <a,b>`: restrict the run to a subset of scenario groups (default: all). An **unknown group name aborts** before any run.
- `--controllers` accepts **`@<kmh>` speed-cap variants**: append `@<kmh>` to an `mpc`/`mpcc` controller to cap its straight-line speed, e.g. `--controllers mpcc,mpcc@100`. Each capped run shows up as its own report column, so the cap is a column and the route regime is a group.
- `--repeats <n>`: multiple samples per scenario for stats.

```bash
# only the elevated routes, comparing full-speed vs capped MPCC
ros2 run carla_mpc_bringup run_benchmark \
    --scenarios src/carla_mpc_bringup/config/scenarios.yaml \
    --groups elevated_3d --controllers mpcc,mpcc@100 --repeats 3
```

---

## `config/routes/`: reproducible benchmark routes

Explicit-waypoint route XMLs used by `route.mode: file` and by the scenario
suite. Build and verify routes with `route_lab`:

```bash
# build a route from atlas spawn-point numbers
ros2 run carla_mpc_bringup route_lab --town Town04 --spawns "i,j,k" --save highway_speed

# verify an existing route (writes a preview under logs/route_lab/)
ros2 run carla_mpc_bringup route_lab --route routes/highway_speed.xml
```

Use a route with `route.mode: file` + `route.file: routes/<name>.xml` (or the
launch `route:=routes/<name>.xml` argument) for a deterministic, repeatable run,
the right setup for comparing PID, PID++, MPC, and MPCC quantitatively.
