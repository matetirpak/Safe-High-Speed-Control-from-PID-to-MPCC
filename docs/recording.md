# Recording and review

The primary record of every run is the human-readable log under `logs/runs/`
(structured metrics, the full plotted series, and PNG plots), written
automatically on every run. See [evaluation.md](evaluation.md) for that pipeline.

The rosbag described here is a separate, **opt-in** artefact (`record:=true`). It
captures the live ROS streams verbatim for Foxglove time-travel replay without
summarising them, so it is off by default and only worth enabling when you want
to scrub through a run interactively or re-publish it offline.

## What gets saved

With `record:=true`, the launch starts the `record_run` tool, which writes a
self-contained run folder:

```text
runs/<timestamp>_<scenario>_<controller>[_<condition>]/
  bag/            rosbag2: control telemetry (/controller/*), ground truth
                  (/carla/hero/gt/{path,odom}), /tf, /clock, and the ego model.
                  The camera stream is excluded unless record_inputs:=true.
  manifest.yaml   timestamp, controller, scenario, condition, host, user, ROS
                  distro, git (commit/branch/dirty, or "not-a-git-repo"), and
                  what was recorded.
  config/         snapshot of the whole config/ directory (vehicle, carla,
                  controller, scenarios, conditions YAML + routes/), so the run
                  stays reproducible even if those files are later edited.
  summary.json    written by review_run (see below)
  review.png      written by review_run --plot
```

`<scenario>` is the route-file stem when `carla.yaml` is in `file` route mode,
otherwise the map name. Always-noisy topics (`rosout`, `parameter_events`) are
dropped, and the heavy camera streams (`image`, `camera_info`) are dropped too unless
`--inputs` is given.

### Launch toggles

| Launch arg                | Effect                                                            |
| ------------------------- | ---------------------------------------------------------------- |
| `record:=true`            | Enable the rosbag (off by default; `logs/` is always written).   |
| `record_inputs:=true`     | Also record the camera stream, so the raw run is replayable (larger). |
| `record_compress:=true`   | zstd file-compress the bag (saves disk, costs CPU).              |

Run folders go under `./runs/` (git-ignored), so launch from the workspace root,
or point `MPC_RUNS_DIR` elsewhere. `record_run` replaces itself with `ros2 bag
record`, so on Ctrl-C or launch shutdown the bag finalizes cleanly.

You can also record standalone, without the launch:

```bash
ros2 run carla_mpc_bringup record_run --controller mpc --condition rainy_night
```

## Reviewing a run

```bash
ros2 run carla_mpc_bringup review_run runs/<name>          # prints + writes summary.json
ros2 run carla_mpc_bringup review_run runs/<name> --plot   # also writes review.png
```

`review_run` accepts a run directory (it locates `<run>/bag`), a bag directory
directly, or a `runs/` parent (it takes the newest child that has a bag). It
reads the `/controller/*` `Float32` scalar telemetry from the bag and reports the
metrics that matter for control debugging:

- **speed**: mean / max / final, plus the mean target speed (tracking);
- **cross-track error**: mean / p95 / max, on the absolute error (accuracy);
- **heading error**: mean / max, on the absolute error (degrees);
- **steering**: mean `|steer|`, `jitter_rms` (RMS of the per-step increment),
  and `reversals_per_s` (the micro left/right chatter metric; high = twitchy);
- **MPC solve time**: p50 / p99 / max (omitted when there is no solver trace).

A `duration_s` field records the telemetry span. The summary is printed and
written to `summary.json` in the run directory.

`review.png` (with `--plot`) stacks four time-aligned traces: speed vs target,
steering (where chatter is visible), cross-track error, and solve time.

This is the loop for tuning: run → `review_run --plot` → adjust
`config/controller.yaml` → re-run. Sharing a `runs/<name>/` folder (or just its
`summary.json` + `review.png`) lets a run be diagnosed without re-driving it.

## Replaying the bag

```bash
ros2 bag info runs/<name>/bag
ros2 bag play runs/<name>/bag      # re-publish into the graph (open Foxglove to watch)
```

With `record_inputs:=true` the camera stream is in the bag too (`/tf` and `/clock`
are recorded either way), so the raw run can be replayed for offline analysis instead
of only its scalar telemetry.
