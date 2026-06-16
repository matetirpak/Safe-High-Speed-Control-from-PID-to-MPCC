# Entrypoints: every way to run this package

The complete reference for running `carla_mpc_bringup`: each command, its parameters, and the logs
it produces. **A CARLA server (0.9.16) must be running** for anything that touches the simulator
(everything except `benchmark_report` and `review_run`). Run **one** CARLA client at a time: two
clients sharing a world corrupt each other's runs.

## Contents
- [Environment setup](#environment-setup)
- [Quick reference](#quick-reference)
- [`control_node`: a single run](#control_node-a-single-run)
- [Launch files: run + visualize](#launch-files-run--visualize)
- [`run_benchmark`: scenario sweep](#run_benchmark-scenario-sweep)
- [`benchmark_report`: (re)aggregate](#benchmark_report-reaggregate)
- [Scenario routes & maps (map_atlas / route_lab / build_routes)](#scenario-routes--maps)
- [`identify_grip`: tyre/grip ID](#identify_grip-tyregrip-id)
- [`record_run` / `review_run`](#record_run--review_run)
- [Other nodes](#other-nodes)
- [Output reference (what the logs contain)](#output-reference)
- [Lane-invasion detection](#lane-invasion-detection)

---

## Environment setup

One sourced script sets up ROS 2, the workspace, the venv (CARLA wheel), CARLA's PythonAPI, and
acados:

```bash
cd ~/autonomousdriving/mpc
source setup_env.sh        # ROS2 + install/ + .venv + CARLA_ROOT/PythonAPI + acados + RMW/DOMAIN
```

Equivalent explicit form (what `setup_env.sh` does, useful in scripts/cron):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export CARLA_ROOT=$HOME/autonomousdriving/Carla/CARLA_0.9.16
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$PWD/.venv/lib/python3.10/site-packages:$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH"
```

Start CARLA with its native ROS 2 publisher (headless is fine):
```bash
$CARLA_ROOT/CarlaUE4.sh -RenderOffScreen --ros2 &
```

`--ros2` enables CARLA 0.9.16's built-in DDS bridge, which this package relies on for sensor
topics. Keep `ROS_DOMAIN_ID` identical in the CARLA shell and the control shell. The
[`carla_auto_reboot.sh`](../carla_auto_reboot.sh) helper keeps a `-RenderOffScreen --ros2` server
alive and restarts it on a crash.

After editing source: `colcon build` (fast, ~1 s). After editing **baked** MPC weights
(`q_*`, `r_*`, `a_lat_max`, `r_ddelta`) also clear the acados cache: `rm -rf ~/.cache/carla_mpc_acados/`.

---

## Quick reference

| Command | Purpose | CARLA? | Output |
|---|---|---|---|
| `ros2 run carla_mpc_bringup control_node` | one controlled run | yes | `logs/runs/<ts>_<map>_<ctrl>/` |
| `ros2 launch carla_mpc_bringup carla_pid.launch.py` | run **+ RViz/Foxglove viz** | yes | same + live topics |
| `ros2 run carla_mpc_bringup run_benchmark` | scenario × controller sweep | yes | `logs/benchmarks/<ts>/` |
| `ros2 run carla_mpc_bringup benchmark_report` | (re)build a report from runs | no | `logs/benchmarks/<ts>/` |
| `ros2 run carla_mpc_bringup identify_grip` | tyre cornering-stiffness / grip ID | yes | YAML (stdout or `--out`) |
| `ros2 run carla_mpc_bringup record_run` | record sensors/inputs to a bag | yes | `runs/<name>/` |
| `ros2 run carla_mpc_bringup review_run` | summarize/plot a recorded `record_run` bag | no | `summary.json` (+ `review.png`) |
| `ros2 run carla_mpc_bringup map_atlas` | per-town atlas (spawn pts + junctions) | yes | `logs/map_atlas/<Town>.{png,json}` |
| `ros2 run carla_mpc_bringup route_lab` | build/verify a scenario route + plot | yes | `logs/route_lab/<name>.png` (+ route XML) |
| `ros2 run carla_mpc_bringup build_routes` | generate a feature route (lane-graph walk) | yes | `config/routes/<scenario>.xml` |

---

## `control_node`: a single run

```bash
ros2 run carla_mpc_bringup control_node --controller mpcc --map Town03 --spawn-index 18 \
  --seed 1 --max-distance 200 --route-extend false --no-rendering-mode on
```

| Argument | Default | Meaning |
|---|---|---|
| `--controller` | `pid` | `pid` \| `pidpp` \| `mpc` \| `mpcc` |
| `--map` | (carla.yaml) | town to load, e.g. `Town03` |
| `--spawn-index` | (carla.yaml) | spawn-point index (route start) |
| `--seed` | (carla.yaml) | route/spawn RNG seed (reproducible benchmarks) |
| `--route` | – | explicit-waypoint route XML, e.g. `routes/sharp_turns.xml` (overrides spawn+seed; resolved against the installed config dir, or an absolute path) |
| `--route-id` | `""` | which `<route id>` to use if the XML defines several |
| `--max-distance` | `0.0` | stop after N metres (`0` = unbounded) |
| `--max-duration` | `0.0` | stop after N seconds (`0` = unbounded) |
| `--route-extend` | `true` | `true` = keep appending route (endless driving); `false` = drive the fixed route once then stop |
| `--speed-cap` | `0.0` | override the mpc/mpcc cruise-speed cap (km/h) for this run (`0` = config value); runtime-only, labels the run `<controller>@<cap>`. Equivalent to passing `--controller mpcc@<kmh>`. |
| `--condition` | – | named condition or a CARLA weather preset (e.g. `WetNoon`) |
| `--no-rendering-mode` | `auto` | `on` (headless, fastest) \| `off` \| `auto` |
| `--sensors` | `""` | comma-separated whitelist of sensor names from `vehicle.yaml` (e.g. `cam_front`); empty = all enabled sensors |
| `--record-camera` | off | record the driver-view to `camera.mp4` (**incremental → Ctrl+C-safe**; the Foxglove camera+trajectory overlay if the overlay node is running, else raw `cam_front`); **forces rendering ON** |
| `--warmup` | `-1.0` | seconds to hold still before driving (`-1` = carla.yaml default) |
| `--scenario-group` | `""` | scenario-group name (scenarios.yaml top-level key) this run belongs to; the report aggregates separately per group (`""` = `main`) |
| `--host` / `--port` | localhost / 2000 | CARLA connection |
| `--vehicle-config` / `--carla-config` / `--controller-config` | `config/*.yaml` | override config files |
| `--verbose`, `-v` | off | debug logging |

**Produces** `logs/runs/<timestamp>_<map>_<controller>/`:
- `series.csv`: per-tick log (see [columns](#seriescsv-columns))
- `metrics.json`: computed metrics + scenario + events
- `summary.txt`: human-readable metric summary
- `replay.gif`: top-down driven-path animation (no ffmpeg needed)
- `camera.mp4`: driver-view video, **only with `--record-camera`**: the actual Foxglove frames (camera + projected reference [blue] / predicted [orange] trajectory) when the overlay node runs (live launch), else the raw `cam_front` feed (headless benchmark). Encoded **incrementally** → a Ctrl+C on a long run keeps it.
- `plots/`: `trajectory.png` (path + lane-violation markers + start), speed/cte/etc. plots

---

## Launch files: run + visualize

For interactive driving with RViz + Foxglove (vs. the bare `control_node`):

```bash
ros2 launch carla_mpc_bringup carla_pid.launch.py controller:=mpcc with_foxglove:=true
```

`carla_pid.launch.py` arguments (launch syntax `name:=value`):

| Arg | Default | Meaning |
|---|---|---|
| `controller` | `pid` | `pid` \| `pidpp` \| `mpc` \| `mpcc` |
| `map` | `""` | override map (`""` = carla.yaml) |
| `spawn_index` / `seed` | `-1` | `-1` = carla.yaml default |
| `route` | `""` | explicit-waypoint route XML, e.g. `routes/sharp_turns.xml` (`""` = carla.yaml route) |
| `max_distance` / `route_extend` / `warmup` | `0.0` / `true` / `-1.0` | as in `control_node` |
| `condition` / `sensors` / `no_rendering` | `""` / `""` / `auto` | as in `control_node` |
| `use_sim_time` | `true` | sim clock for all nodes |
| `publish_gt` | `true` | publish ground-truth pose/path for RViz |
| `with_foxglove` / `foxglove_port` | `true` / `8765` | Foxglove bridge |
| `record` / `record_inputs` / `record_compress` | `false` | wrap the run in `record_run` |
| `record_camera` | `false` | save the driver-view `camera.mp4` per run (forces rendering ON) |

Other launch files: `spawn.launch.py` (the headless core: `control_node` + `gt_pose_publisher` +
`ego_model_publisher`, no Foxglove bridge or camera overlay; `carla_pid.launch.py` includes it),
`foxglove.launch.py` (Foxglove bridge alone).

---

## `run_benchmark`: scenario sweep

Runs every (controller × scenario × repeat), collects the runs, and builds a report. The suite in
`config/scenarios.yaml` is route-based: each scenario names an explicit `route:` XML built with
route_lab (see [Scenario routes & maps](#scenario-routes--maps)). Routed scenarios need no
`distance` (the route defines the length); ad-hoc spawn+seed runs (no `--scenarios`) use `--distance`.

**Scenario groups.** Every top-level list in `scenarios.yaml` is a scenario *group*, named by its key,
and the report aggregates each group **separately** (its own Metrics + Speed tables, titled by the list
name). The current suite has two groups:

| Group | Scenarios | Notes |
|---|---|---|
| `flat_2d` | `sharp_turns` (Town07), `highway_speed` (Town04), `highway_lane_changes` (Town06), `roundabout` (Town03) | full-speed flat routes, the default operating regime |
| `elevated_3d` | `country_road` (Town11), `curvy_downhill` (Town11) | Town11 rural drives with real elevation, kept as their own group |

```bash
ros2 run carla_mpc_bringup run_benchmark \
  --scenarios src/carla_mpc_bringup/config/scenarios.yaml \
  --controllers pid,pidpp,mpc,mpcc --repeats 1 \
  --out logs/benchmarks/rigorous_$(date +%H%M%S)
```

| Argument | Default | Meaning |
|---|---|---|
| `--controllers` | `pid,mpc,mpcc` | comma list to compare. Append `@<kmh>` for an **mpc/mpcc speed-cap variant**, e.g. `pid,pidpp,mpc,mpcc,mpcc@100`: the capped run keeps the curvature profile but caps cruise at `<kmh>` (runtime-only, same acados solver), and shows as its own `mpcc@100` column. Use it to compare a full-speed run against a tamer cap on the elevated routes (e.g. MPCC@180 vs MPCC@100). |
| `--scenarios` | – | scenarios YAML; **if set, overrides** `--map/--seeds/--spawns/--distance` per scenario |
| `--groups` | `""` (all) | comma list of scenario groups (scenarios.yaml top-level keys) to run; an **unknown name aborts** before any run |
| `--map` | `Town05` | map (ad-hoc mode, no `--scenarios`) |
| `--seeds` | `42` | comma list of seeds (ad-hoc mode) |
| `--spawns` | `0` | comma list of spawn indices (ad-hoc mode) |
| `--distance` | `800.0` | max metres per run (ad-hoc mode) |
| `--condition` | `""` | weather/condition for all runs |
| `--repeats` | `1` | repeats per cell (mean ± std over repeats) |
| `--timeout` | `300.0` | hard wall-clock cap per run (s) |
| `--out` | `logs/benchmarks/<ts>` | report directory |
| `--report-only` | off | skip running; just aggregate existing `logs/runs/*` |
| `--record-camera` | off | save each run's `camera.mp4` (driver-view; forces rendering ON; slower + more storage) |

Each cell shells out to `control_node` (single client, sequential, hence safe). Roughly *N_runs × ~2 min*.

**Produces** `logs/benchmarks/<ts>/`:
- `summary.csv`: one row per controller, aggregated across all scenarios
- `summary_by_scenario.csv`: one row per (scenario, controller), `_mean`/`_std` over repeats (only when more than one scenario is present)
- `report.html` / `report.md`: per-group Metrics + Speed-by-curvature tables, a per-scenario breakdown, and the driven-path overlay
- `plots/`: `paths_by_scenario.png` (per-scenario overlay, lane invasions marked with a per-controller shape + colour), plus `metrics_<group>.png` and `speed_by_curvature_<group>.png` per scenario group (bare `metrics.png` / `speed_by_curvature.png` when there is a single group)
- `runs/`: the individual run dirs, co-located (only the runs this invocation produced; otherwise they remain in `logs/runs/`)

---

## `benchmark_report`: (re)aggregate

Rebuild a report from existing run dirs without re-running CARLA (e.g. after a code fix to the report):

```bash
ros2 run carla_mpc_bringup benchmark_report 'logs/runs/20260609-14*' --out logs/benchmarks/redo
ros2 run carla_mpc_bringup benchmark_report --controllers pid,pidpp     # default runs = logs/runs/*
```

| Argument | Default | Meaning |
|---|---|---|
| `runs` (positional) | `logs/runs/*` | run dirs or globs to include |
| `--out` | `logs/benchmarks/<ts>` | report dir |
| `--controllers` | `""` | comma filter |

> Note: a report only reflects the runs it reads. If `paths_by_scenario.png` looks wrong, check that
> the right run dirs were globbed (stale runs in `logs/runs/` get picked up by the default glob).

---

## Scenario routes & maps

Tools for designing and verifying explicit-waypoint scenario routes **by hand**, against real CARLA
coordinates (you pick points by their atlas spawn-point **number**, never raw coords). All load a
world (one CARLA client at a time).

### `map_atlas`: per-town atlas (spawn points + junctions)

```bash
ros2 run carla_mpc_bringup map_atlas                                  # the scenario towns
ros2 run carla_mpc_bringup map_atlas --towns Town04,Town11,Town12     # specific towns
```

Writes `logs/map_atlas/<Town>.{png,json}` per town: a top-down plot (solid road network, **blue
spawn-point indices**, orange junction rings) and the raw JSON: `spawn_points` `[{i,x,y,z,yaw}]`
and `junctions` `[{id,x,y,ext_x,ext_y,n_paths}]`. **Design routes by the blue spawn numbers.** Big
maps (Town11/12) render at ~8000 px; zoom in to read labels.

| Argument | Default | Meaning |
|---|---|---|
| `--towns` | scenario set | comma list of towns |
| `--out` | `logs/map_atlas` | output dir |
| `--host` / `--port` | localhost / 2000 | CARLA connection |

### `route_lab`: build / verify a route, plot it

```bash
# build a route through atlas spawn numbers (the GRP fills the lanes between them) + save the XML:
ros2 run carla_mpc_bringup route_lab --town Town07 --spawns "90,43,100,24" --save sharp_turns
# verify an existing route XML:
ros2 run carla_mpc_bringup route_lab --route routes/sharp_turns.xml
```

Traces the route via the road planner, **measures** it (length, sharp turns ≥45°, junctions entered,
lane changes, min curve radius, loop-back) and plots `logs/route_lab/<name>.png` (route **coloured by
distance from start**). With `--save NAME` it also writes `config/routes/NAME.xml`, storing the
**spawn numbers** in the header *and* a `<!-- spawn N -->` on each point, so you edit the route by
number, not coordinates. Watch for `min curve radius = 0 / max step = 180`: that's a **cusp**. The
GRP U-turned at a point whose lane faces away from your approach (drop or replace that spawn).

| Argument | Default | Meaning |
|---|---|---|
| `--town` + `--spawns "i,j,k"` | – | build a route through these atlas spawn indices |
| `--route` | – | verify an existing route XML instead |
| `--json` | – | `{town, waypoints:[{x,y,z}]}` instead of spawn numbers |
| `--save NAME` | – | also write `config/routes/NAME.xml` |
| `--name` | derived | output plot name |

### `build_routes`: deterministic generated routes

```bash
ros2 run carla_mpc_bringup build_routes --scenario sharp_turns        # | roundabout | highway
```

Lane-graph **walk** generators (turn-greedy / roundabout lap / highway ramp) that write
`config/routes/<scenario>.xml` automatically. Use `route_lab --spawns` for hand-picked routes;
use `build_routes` to auto-generate feature-dense routes.

### Wiring a route into the benchmark

Add the scenario under a group (top-level list) in `config/scenarios.yaml`; no `distance`, the route
defines the length:

```yaml
flat_2d:
  - name: sharp_turns
    map: Town07
    route: routes/sharp_turns.xml

elevated_3d:
  - name: country_road
    map: Town11
    route: routes/country_road.xml
```

`colcon build` installs the XML, then `run_benchmark` (or `control_node --route routes/sharp_turns.xml`,
or `carla_pid.launch.py route:=routes/sharp_turns.xml`) drives it.

---

## `identify_grip`: tyre/grip ID

Steady-state circular tests to measure cornering stiffness and the friction-limited lateral grip
(`a_lat_max`). See [`tools/identify_grip.py`](../src/carla_mpc_bringup/carla_mpc_bringup/tools/identify_grip.py).

```bash
ros2 run carla_mpc_bringup identify_grip --map Town06 --spawn-index 0 \
  --steers 0.15,0.25,0.35 --out /tmp/grip.yaml
```

| Argument | Default | Meaning |
|---|---|---|
| `--map` | (current) | load this town first |
| `--spawn-index` | `0` | spawn point |
| `--steers` | `0.15,0.25,0.35` | steer angles to sweep, as **fractions of the vehicle's max steer** (smaller = larger radius) |
| `--out` | – | write the identified-params YAML (else only printed): an `identified:` block with `grip_ms2`, `grip_g`, `Cf`, `Cr`, `recommend_a_lat_max` |
| `--host` / `--port` | localhost / 2000 | CARLA connection |

For the default Model 3 the measured grip is **~15 m/s² (~1.6 g)**, which confirms the configured
`a_lat_max` (MPCC `a_lat_max: 15`, MPC `a_lat_max: 13`). The **grip** is the reliable output; Cf is
usable but Cr is not reliably identifiable from tight circles (the rear barely slips). With `--out`,
the tool also writes a `<out>_raw.csv` (columns `vx,vy,r,a_y,delta`) of the raw samples for offline
re-analysis.

---

## `record_run` / `review_run`

`record_run` wraps a drive and saves sensor/input streams to a structured bag:

```bash
ros2 run carla_mpc_bringup record_run --controller pidpp --inputs --condition WetNoon --note "wet test"
```
Key args: `--record-dir`, `--controller`, `--inputs` (log throttle/brake/steer), `--condition`,
`--scenario`, `--note`, `--name` (folder name), `--compress` (zstd), config overrides. Output:
`runs/<name>/` (or `$MPC_RUNS_DIR`) with the bag, a manifest, and a config snapshot.

`review_run` summarizes/plots a recorded `record_run` bag (no CARLA). It reads the rosbag2 control
telemetry, prints a summary, and writes `summary.json` into the run dir; it expects a `record_run`
output (a `runs/<name>/` run dir, which holds the bag) or a bag directory directly, not a
`control_node` run dir (those have no bag):

```bash
ros2 run carla_mpc_bringup review_run runs/wet_test --plot
```
`run` (positional, required); `--plot` also writes `review.png` alongside `summary.json`.

---

## Other nodes

Usually started by the launch files, runnable standalone for debugging:

| Node | Role | Started by |
|---|---|---|
| `gt_pose_publisher` | ground-truth ego pose/path + `gt_map→hero` TF for RViz | `spawn.launch.py` |
| `ego_model_publisher` | ego car `MarkerArray` in the `hero` frame | `spawn.launch.py` |
| `predicted_path_overlay` | project the predicted (orange) + reference (blue) paths into the camera image | `carla_pid.launch.py` |

---

## Output reference

### `series.csv` columns
`t`, `x`, `y`, `yaw`, `speed_kmh`, `target_speed_kmh`, `throttle`, `brake`, `steer`,
`cross_track_m`, `heading_error_deg`, `solve_time_ms`, `lat_accel_ms2`
(measured from physics, not differentiated yaw), `lane_invasions_total` (cumulative **real**
invasion count; see below).

### `metrics.json` structure
`metrics.json` nests its fields by topic (computed in [`eval/metrics.py`](../src/carla_mpc_bringup/carla_mpc_bringup/eval/metrics.py)):

- `scenario`: the run's identity (controller, map, seed, spawn_index, group, `condition`, target_speed_kmh, a_lat_max, and a precomputed `lane_invasions`)
- `samples`: number of logged ticks
- `lane_departures`: post-hoc count of `|cte| > 1.5 m` rising edges (corner-cuts / big drifts)
- `trip`: `duration_s`, `distance_m`, `avg_speed_kmh` (averaged while actually driving, ramps excluded), `max_speed_kmh`
- `tracking`: `cross_track_m` and `heading_error_deg`, each as a `{mean,rms,max,…}` stats block
- `speed`: `speed_kmh` and `speed_tracking_err_kmh` stats
- `comfort`: `lat_accel_ms2` (and `lat_accel_g`), `long_accel_ms2`, `lat_jerk_ms3`, `steer`, `steer_reversals_per_s`; compare the peak lateral accel to the ~15 m/s² grip
- `solver`: `solve_ms_mean` / `solve_ms_p50` / `solve_ms_p99` / `solve_ms_max` (MPC/MPCC)
- `events`: `lane_invasions`, the masked real-crossing count (the safety metric; see [Lane-invasion detection](#lane-invasion-detection))

The benchmark report aggregates these into the `TABLE_METRICS` labels (`avg_speed_kmh`, `distance_m`,
`cte_rms_m`, `cte_max_m`, `heading_rms_deg`, `lat_accel_max`, `lat_jerk_rms`, `steer_rev_per_s`,
`solve_p99_ms`, `lane_invasions`, `lane_departures`).

---

## Lane-invasion detection

How the "lane invasions" metric and the circles in `paths_by_scenario.png` are produced
([`sim/control_node.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/control_node.py)):

1. CARLA's `sensor.other.lane_invasion` fires whenever the ego body crosses **any** lane marking.
   A monitor keeps a **raw** running count and the crossed-marking label.
2. Each new raw crossing is **masked** if it is the *reference's* crossing, not a real departure:
   - **on-path:** `|cross_track| < 0.7 m` (`LANE_ON_PATH_M`); the car is on its trajectory through a
     legitimate lane change *or* a junction turn that crosses a dashed line.
   - **reference lane change:** `|cross_track| < 1.5 m` (`LANE_LAG_M`) **and** the planner reports an
     active lane change (tracking lag during a planned manoeuvre).
   - otherwise → a corner-**cut**/drift → counts as a real invasion (`real_lane_count += 1`).
3. **Off-lane departure:** a reckless cut can bulge off-lane mid-curve *without* crossing a marking,
   so the rising edge of `|cross_track| > 1.5 m` (1 s cooldown) also counts.
4. `real_lane_count` is the metric (`metrics.lane_invasions`), drives the RViz blink, and is logged
   each tick as `lane_invasions_total`. The report **circles** the points where `lane_invasions_total`
   increments, so circles == the metric.

> **Circles always match the metric.** The series logs the masked `real_lane_count` (not the raw
> sensor count), so the circles the report draws on `paths_by_scenario.png` correspond exactly to the
> `lane_invasions` metric. The raw sensor count would include on-path dashed-line crossings, which
> MPC/MPCC trip often while driving multi-lane roads on-path, and would draw false circles where the
> masked metric reads 0; logging the masked count avoids that mismatch.
