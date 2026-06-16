# Evaluation: metrics, logs, and benchmarks

Every run writes a **human-readable log directory** to `logs/runs/<ts>_<map>_<controller>/`:
metrics, the full per-step series, and PNG plots. This is *not* a ROS bag; it is the
processed, comparable output you actually read. The raw rosbag is **opt-in**
(`record:=true`, for Foxglove time-travel replay), because the bag duplicates the live ROS
streams without summarising them.

The same pure metrics code that summarises one run is reused to aggregate many runs into a
controller-vs-controller **benchmark report**.

## Output layout

```text
logs/
  runs/                                  # one folder per run (kept here so logs/ stays tidy)
    20260606-045227_Town05_mpc/
      metrics.json   # scenario (inputs) + metrics (outputs), structured -> the benchmark unit
      summary.txt    # the same, as a one-page human-readable digest
      series.csv     # every per-step value recorded each tick (t, pose, speed, control,
                     #   errors, lateral accel, solve time…); the plots are subsets of these
      plots/         # trajectory (speed heatmap), speed, tracking, control, accel, solve_time
      replay.gif     # low-res top-down replay (path + speed cursor); skipped if it fails
  benchmarks/                            # one TIMESTAMPED folder per report (never overwritten)
    20260606-045227/
      report.html               # styled, browsable headline (colour-graded tables + plots)
      report.md                 # the same tables in Markdown
      summary.csv               # pooled per-controller table
      summary_by_scenario.csv   # per-scenario x controller breakdown (only when >1 scenario)
      runs/                     # the run logs that fed this report (when --out is set)
      plots/                    # metrics.png, speed_by_curvature.png, paths_by_scenario.png
```

## How a run log is produced

- [`eval/metrics.py`](../src/carla_mpc_bringup/carla_mpc_bringup/eval/metrics.py) holds
  **pure** functions (no ROS/CARLA/I-O): a per-step series → a structured metrics dict, plus
  `summary_text()`. Derived signals (longitudinal/lateral accel, jerk, steering smoothness)
  are computed here in `derived_series()` from the raw series. Being pure, it is unit-tested
  and **reused by the benchmark aggregator** (many runs), not just one run.
- [`eval/run_logger.py`](../src/carla_mpc_bringup/carla_mpc_bringup/eval/run_logger.py):
  `RunLogger.record(**sample)` is called every control tick (a cheap append); `write()` runs
  once at shutdown (out of the hot loop) and does the numpy/matplotlib work, writing
  `metrics.json`, `summary.txt`, `series.csv`, `plots/`, and `replay.gif`.
- [`sim/control_node.py`](../src/carla_mpc_bringup/carla_mpc_bringup/sim/control_node.py)
  owns the `RunLogger`, records each tick, and flushes in its `finally` block, so a Ctrl-C'd
  run is still fully logged.

## The metrics

Each scalar metric reports **mean / rms / max / p95 / min** over finite samples (see
`_stats()` in `metrics.py`). If a plot fails to render, a `plots/ERROR.txt` is written with
the reason and the metrics/CSV are still produced.

| Group | Metric | Meaning |
|---|---|---|
| trip | `distance_m`, `duration_s`, `avg_speed_kmh`, `max_speed_kmh` | the journey; `avg_speed_kmh` is averaged *while driving* (start/end ramps and stops excluded) |
| tracking | `cross_track_m` (rms/mean/max/p95/…) | lateral deviation from the path, **the** accuracy number |
| tracking | `heading_error_deg` (rms/max/…) | heading misalignment |
| speed | `speed_kmh`, `speed_tracking_err_kmh` | how fast, and how well the target speed is held |
| comfort | `lat_accel_ms2` / `lat_accel_g` (rms/max) | cornering load; the `_g` view is the same figure divided by *g*, vs the grip budget |
| comfort | `long_accel_ms2`, `lat_jerk_ms3` | accel/brake harshness and ride smoothness |
| comfort | `steer` (rms), `steer_reversals_per_s` | **steering smoothness / weave** (high reversals = wobble) |
| solver | `solve_ms_mean` / `solve_ms_p50` / `solve_ms_p99` / `solve_ms_max` | MPC real-time headroom (empty for PID) |
| events | `lane_invasions` | masked lane-marking crossings during the run |
| (top level) | `lane_departures` | rising-edge count of `|cross_track| > 1.5 m` excursions, to catch corner-cuts a marking-crossing misses |

### Fair jerk metric

`derived_series()` *prefers* the **measured** body-frame lateral acceleration
`lat_accel_ms2` that `control_node` logs each tick: it rotates CARLA's physics
`ego.get_acceleration()` into the ego body frame
(`a_lat = -sin(yaw)·a_x + cos(yaw)·a_y`). Using a directly observed signal makes `lat_jerk` a
**single** derivative of it. The `v·d/dt(yaw)` centripetal form (a *double* derivative of
yaw, noise-amplified and speed-biased, so it over-penalises the faster controllers) is only
the **fallback** for legacy logs that lack the `lat_accel_ms2` column.
`steer_reversals_per_s` counts sign changes of the steering rate; it is the quantitative
measure of steering "wobble".

## Benchmarking: replicable controller comparison

`metrics.json["scenario"]` records the **replicable inputs** (controller, model, map, seed,
`spawn_index`, condition, target speed, `dt`, `max_distance_m`, plus the scenario `group`);
the rest of `metrics.json` is the **comparable output**. Two entry
points build on this.

### `run_benchmark`: drive then aggregate

Runs each `(scenario × controller × repeat)` as one bounded `control_node` drive on the same
seeded road, then aggregates the runs it produced into a report. Requires CARLA running with
`--ros2`; it drives `control_node` directly (the run log is written by `control_node`, so no
Foxglove/ground-truth is needed).

```bash
# A) curated SCENARIO SUITE (config/scenarios.yaml): named map+route+seed routes, each curated
#    to traverse a road feature. --scenarios overrides --seeds/--spawns/--map and passes
#    --route-extend false (deterministic, bounded routes); --repeats still applies for stats:
ros2 run carla_mpc_bringup run_benchmark --controllers pid,mpc,mpcc \
    --scenarios src/carla_mpc_bringup/config/scenarios.yaml --repeats 3

# B) or the seeds x spawns grid on a single map:
ros2 run carla_mpc_bringup run_benchmark --controllers pid,mpc,mpcc \
    --seeds 42 --spawns 0 --map Town05 --distance 800
```

Useful flags:

- `--groups <a,b>`: run only a subset of the scenario GROUPS (top-level lists in
  `scenarios.yaml`); an unknown group name aborts before any run.
- `--controllers` accepts `@<kmh>` **speed-cap variants**; e.g. `mpcc,mpcc@100` runs MPCC at
  its default cap *and* a 100 km/h cap as a separate report column (`control_node` parses the
  `@` suffix).
- `--repeats N`: multi-sample stats (mean ± std per cell).
- `--report-only`: skip running and just aggregate `logs/runs/*`.
- `--record-camera`: also save each run's driver-view `camera.mp4` (forces rendering on).

### `benchmark_report`: aggregate existing logs

```bash
ros2 run carla_mpc_bringup benchmark_report                       # all of ./logs/runs/*
ros2 run carla_mpc_bringup benchmark_report logs/runs/2026* --out logs/benchmarks/my
ros2 run carla_mpc_bringup benchmark_report --controllers pid,mpc,mpcc
```

### What the report contains

The aggregation is pure
([`eval/benchmark.py`](../src/carla_mpc_bringup/carla_mpc_bringup/eval/benchmark.py));
[`tools/benchmark_report.py`](../src/carla_mpc_bringup/carla_mpc_bringup/tools/benchmark_report.py)
renders the files, HTML, and plots.

`report.html` is the all-in-one browsable view (open it in a browser); it embeds every
table **and** every plot on one page:

- **Metrics table** per scenario group: one column per controller, **colour-graded per
  metric** (🟩 best in the row → 🟥 worst, direction-aware).
- **Speed by road curvature** table per group: the *fair, route-agnostic* speed view
  (raw average speed is biased by which sections a run drove, so speed is bucketed into
  straight / gentle / moderate / tight bands estimated from the driven path).
- **Per-scenario breakdown**: the same metrics split by route, under the group aggregate.
- **Metric glossary**: a one-line explanation of each row.
- **Embedded plots**: `metrics.png` (a bar panel per metric; ★ marks the best,
  lower-is-better panels hatched), `speed_by_curvature.png`, and `paths_by_scenario.png`
  (each controller's driven path per scenario, lane invasions marked with a per-controller
  shape + colour); the first two are
  emitted per group when there is more than one.
- **References**: every run folder that fed the report.

Alongside it: `report.md` (the group-level Metrics and Speed-by-road-curvature tables in Markdown), `summary.csv` (the pooled
per-controller table), `summary_by_scenario.csv` (the per-scenario × controller breakdown,
mean ± std per cell, written only when runs span more than one scenario), and `plots/`
(the PNGs `report.html` embeds).

### Scenario groups

`scenarios.yaml` supports **multiple top-level lists**, each a scenario **group** named by its
key (e.g. `flat_2d`, `elevated_3d`). The report aggregates each group **separately** (its own
Metrics and Speed-by-curvature tables/plots, titled with the list name), so different
operating regimes are not pooled into one misleading number (e.g. full-speed flat 2D routes
vs. speed-capped elevated 3D routes). Pair groups with `@<kmh>` variants to show, for example,
that MPCC at its default cap struggles on a curvy 3D route while `mpcc@100` aces it: the cap is
a column, the route regime is a group. Runs from a bare launch (no group) fall in `main`, and a
single-group report drops the group suffix from headings.

### Replicability

A run is bounded by `--max-distance` (or `--max-duration`), and seed+spawn+map fix the
RNG-seeded road, so every controller drives the *same* road segment. Each report goes to a
**timestamped** `logs/benchmarks/<ts>/` (never overwritten). When `--out` is given,
`run_benchmark` co-locates the run logs it produced inside `<out>/runs/` so the report is
self-contained.

## Opt-in rosbag (Foxglove replay)

```bash
ros2 launch carla_mpc_bringup carla_pid.launch.py record:=true   # also write runs/<run>/bag
ros2 run carla_mpc_bringup review_run runs/<run> --plot          # offline bag review
```

The bag path (`runs/`, [recording.md](recording.md)) is off by default because `logs/` is the
primary, human-readable record. Add `record_inputs:=true` to also capture the camera
stream (larger bags).
