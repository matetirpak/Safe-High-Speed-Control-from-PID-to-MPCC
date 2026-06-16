# Carla High-Speed Control Testbed: Documentation

Documentation for the `carla_mpc_bringup` package: a Carla 0.9.16 + ROS 2 Humble
+ Foxglove testbed for high-speed trajectory tracking. Four controllers run today,
selected with the `controller:=` launch argument:

| Controller | `controller:=` | What it is |
| ---------- | -------------- | ---------- |
| PID | `pid` | Lateral + longitudinal PID path follower, look-ahead heading + curvature velocity profile (default) |
| PID++ | `pidpp` | A strict extension of `pid` (same code path): adds curvature feed-forward, a steer-rate limit, optional Stanley cross-track, and a longitudinal feed-forward |
| Tracking MPC | `mpc` | Dynamic-bicycle MPC (CasADi + acados SQP-RTI) |
| Contouring MPC | `mpcc` | Model Predictive Contouring Control |

## Read these first

If you're new, read the docs in this order:

1. **[architecture.md](architecture.md)**: the components, the data flow, the
   topic graph and the TF tree.
2. **[execution_flow.md](execution_flow.md)**: exactly what happens, step by step,
   from `ros2 launch` to a car driving in Foxglove.
3. **[configuration.md](configuration.md)**: every config file and the knobs you
   will actually turn.
4. **[recording.md](recording.md)** / **[evaluation.md](evaluation.md)**: every run is
   auto-evaluated to a log directory and the metrics it reports, plus how to record an
   opt-in rosbag and review it offline (`review_run`).

## Reference

- **[entrypoints.md](entrypoints.md)**: every `console_scripts` entry point
  (`control_node`, the visualizers, and the offline tooling), with its CLI flags.
- **[pid_controllers.md](pid_controllers.md)** covers the baseline PID and the PID++
  (`pidpp`) extension that shares its code path: structure, gains, and speed policy.
- **[mpc.md](mpc.md)** covers the implemented dynamic-bicycle tracking MPC (default
  `model: dyn`; `kin_cg` / `kin_rear` / `blended` are fallbacks set in
  `controller.yaml`) and the MPCC contouring controller (CasADi + acados SQP-RTI):
  formulation, the reachable-reference design, tuning knobs, and known limitations.
  Run with `controller:=mpc` or
  `controller:=mpcc`.
- **[dynamics.md](dynamics.md)**: the vehicle models (kinematic / dynamic / blended
  bicycle), tyre models (linear / blended-linear), the `tire_friction`-vs-achievable-grip
  distinction, parameter sources, and the tracking + MPCC OCP formulations, with
  the reference sources cited.

## Benchmark comparison

`run_benchmark` loops the curated deterministic scenarios × controllers and
aggregates a per-scenario-group report:

```bash
ros2 run carla_mpc_bringup run_benchmark \
  --scenarios src/carla_mpc_bringup/config/scenarios.yaml \
  --controllers pid,mpc,mpcc --repeats 3
```

`config/scenarios.yaml` holds one or more **scenario groups**. Each top-level list
(e.g. `flat_2d`, `elevated_3d`) is a group named by its key. The report aggregates
each group **separately** (its own Metrics + Speed tables and plots, titled with the
list name) so different operating regimes aren't pooled into one misleading number.

Useful flags:

- `--groups flat_2d,elevated_3d`: run only a subset of groups (an unknown name
  aborts before any run).
- `--controllers mpcc,mpcc@100`: append `@<kmh>` to an `mpc`/`mpcc` controller for a
  **speed-cap variant**; the capped run shows up as its own report column.

The report renders a metrics table, a speed-by-curvature table, and plots per
scenario group.

## 30-second mental model

```text
CARLA (--ros2) ──native ROS2 publisher──> camera topics, /clock, /tf
   ▲ apply_control(throttle,brake,steer)                 │
   │                                                      ▼
control_node  ── the single world ticker ──  reads ego state each tick, asks the
   │            active controller (PID|PIDPP|MPC|MPCC) for a command, applies it,
   │            and publishes the reference path + a stream of control telemetry
   ▼
gt_pose_publisher: actual driven path/odom + gt_map→hero anchor
ego_model_publisher: the car you see in 3D
                                            everything ──> foxglove_bridge :8765 ──> Foxglove
```

The controller is **swappable behind `ControllerInterface`** and **ROS-free**: its
`step()` takes a `VehicleState`, returns a `ControlCommand`, and exposes telemetry
via `ControllerDebug`. The same plumbing drives all four controllers, swapped by
`controller:=`.

## How do I run it

Start Carla with `--ros2`, then from the workspace root:

```bash
source setup_env.sh
colcon build
ros2 launch carla_mpc_bringup carla_pid.launch.py
```

Open Foxglove → connect to `ws://localhost:8765` → import
`src/carla_mpc_bringup/foxglove/carla_mpc_layout.json`. Swap the controller with
`controller:=pidpp|mpc|mpcc` on the same launch file. Full detail is in
[execution_flow.md](execution_flow.md). The top-level [`../README.md`](../README.md)
covers install + run; these docs explain *how it works*.
