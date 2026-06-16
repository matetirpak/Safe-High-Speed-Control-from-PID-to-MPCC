# PID and PID++ Controllers

Two classical (non-optimization) controllers for trajectory following. **PID** is the safe,
simple baseline; **PID++** is a strict extension that tracks tighter and corners faster. They
share one code path and one config block: PID++ is literally `PIDController` plus a few add-ons.

- **Code:**
  [`control/pid_controller.py`](../src/carla_mpc_bringup/carla_mpc_bringup/control/pid_controller.py),
  [`control/pidplus_controller.py`](../src/carla_mpc_bringup/carla_mpc_bringup/control/pidplus_controller.py)
- **Config:** [`config/controller.yaml`](../src/carla_mpc_bringup/config/controller.yaml)
  (`pid:` and `pidpp:` blocks)
- **Run:** `ros2 run carla_mpc_bringup control_node --controller pid` (or `--controller pidpp`),
  or via launch `ros2 launch carla_mpc_bringup carla_pid.launch.py controller:=pidpp`. See
  [entrypoints.md](entrypoints.md).

---

## TL;DR

| | **PID** (baseline) | **PID++** (extension) |
|---|---|---|
| Class | `PIDController` | `PIDPlusController(PIDController)` |
| Curve-speed policy | forward–backward **velocity profile** | **same** profile |
| Longitudinal | CARLA `PIDLongitudinalController` | inline PID, **same gains** + anti-windup + filtered/clamped D + target feed-forward |
| Lateral feedback | CARLA `PIDLateralController` (look-ahead) | **same** look-ahead PID … |
| Lateral feed-forward | none | … **+ curvature FF** (`δ_ff = lat_kff · atan(L · κ)`) |
| Lateral cross-track | none | + optional **Stanley** term (off by default, `stanley_k_e = 0`) |
| Steer-rate limit | none (EWMA smoothing only) | explicit `steer_rate_max` |
| Actuation smoothing | EWMA over last N commands | **same** (on top of its steer-rate limit) |
| Tuning intent | reasonable, conservative speed | **faster curves + tighter tracking** |
| `a_lat_max` | 7 (~0.7 g) | 14 (~1.4 g) |

Both stay well under the measured tyre grip (~15.8 m/s² ≈ 1.6 g; measure it for your CARLA with
[`tools/identify_grip.py`](../src/carla_mpc_bringup/carla_mpc_bringup/tools/identify_grip.py), the
`identify_grip` entry point).

---

## Shared Foundation: `PIDController`

Everything below lives in the base class, so **both** controllers get it:

1. **Route preprocessing**
   ([`AnnotatedTrajectory`](../src/carla_mpc_bringup/carla_mpc_bringup/control/trajectory.py)).
   The route is annotated once (`set_reference`): an arc-length spline `s → (x, y, ψ, κ)`, plus
   cheap per-tick lookups (forward-local nearest index, cross-track, point-at-arc-length).

2. **The curve-speed policy = a forward–backward velocity profile** (the important part).
   When `a_lat_max > 0` the base builds a per-sample target-speed profile and `_speed_target_kmh`
   reads it directly. The legacy angle-ahead heuristic is kept only as the `a_lat_max = 0` fallback.
   See [§ The Curvature Velocity Profile](#the-curvature-velocity-profile) below.

3. **Steering target** = a look-ahead point on the path (`lat_lookahead_sec · v` ahead,
   pure-pursuit-like).

4. **Actuation** (`_to_command`): split a single `[-1, 1]` longitudinal command into throttle/brake,
   then smooth the actuation.

5. **Actuation smoothing.** `_to_command` runs an exponentially-weighted average over the last
   `cmd_smooth_n` `(accel, steer)` commands (newest weighted most, `cmd_smooth_decay` per step),
   so the raw per-tick chatter is removed and the output is deployable on real hardware.
   `cmd_smooth_n ≤ 1` disables it.

---

## PID: the Safe Baseline

`PIDController` = the shared foundation + CARLA's stock PID feedback laws:

- **Longitudinal:** `PIDLongitudinalController(long_kp, long_ki, long_kd)` tracks the profile speed.
- **Lateral:** `PIDLateralController(lat_kp, lat_ki, lat_kd)` steers toward the look-ahead point.

`step()` is deliberately small: sample → target speed → look-ahead steer point → two PID calls →
high-steer guard → smoothed command. With `a_lat_max = 7` it corners at ~0.7 g: a reasonable,
conservative speed with low lateral acceleration.

The **high-steer spin-out guard** caps throttle (`high_steer_throttle_cap`) when `|steer|` exceeds
`high_steer_threshold`, so the car does not spin exiting a fast corner. It is an ad-hoc
longitudinal/lateral coupling that MPC removes.

---

## PID++: the Natural Extension

`PIDPlusController(PIDController)` calls `super().__init__()` (so it inherits the profile, the
trajectory, the actuation smoothing, the speed policy) and overrides only `step()` to add the
following. Each is **config-gated** and reuses PID's own parameters:

1. **Curvature feed-forward** (`lat_kff`): `δ_ff = lat_kff · atan(L · κ)`, normalized by the
   vehicle's max steer. The PID now only trims the *residual* instead of carrying a standing
   cross-track error through every curve. `L` and the physical max-steer are read from the live
   actor (`_extract_geometry`). `lat_kff = 0` ⇒ no FF.

2. **Inline longitudinal** (`_long_step`) uses PID's **same** `long_kp/ki/kd`, plus:
   - **conditional-integration anti-windup** (don't accumulate the integral when it would push a
     saturated command further into saturation),
   - **filtered + clamped derivative on the *error*** (`d_filter_alpha`, `long_d_max`), which keeps
     the brake "kick" entering curves while rejecting noise spikes,
   - **target feed-forward** (`long_kff`): `u_ff = long_kff · d(target)/tick` (low-passed); brake
     on a dropping target, throttle on a rising one, so the mean speed sits on the profile's
     decel/accel ramps instead of lagging above them.

   This is PID's longitudinal *with three robustness refinements*, not a different controller.

3. **Stanley cross-track term** (`stanley_k_e`, `stanley_k_soft`): `atan(k_e · e_cte / (k_soft + v))`,
   normalized by max-steer. **Off by default (`stanley_k_e = 0`)**: the look-ahead PID already
   corrects cross-track, so stacking the Stanley term on top over-corrects. Kept as a switch.

4. **Steer-rate limit** (`steer_rate_max`): the base PID bypasses CARLA's slew limiter, so PID++
   re-adds an explicit one (anti-jerk). `steer_rate_max = 0` ⇒ off. The inherited actuation
   smoothing still applies on top.

The lateral law is the sum of the three terms (look-ahead heading PID + Stanley cross-track +
curvature FF), then steer-rate limited:

```text
steer = look_ahead_heading_PID            # pure-pursuit-like heading feedback
      + atan(k_e * e_cte / (k_soft + v))  # Stanley cross-track (off by default)
      + lat_kff * atan(L * kappa)         # Ackermann curvature feed-forward
```

The look-ahead heading term anticipates curve entry, so it is kept rather than replaced by a
center-axle Stanley `ψ_e`, which saturates and diverges in fast roundabout turns.

> **Sign convention.** `trajectory.cross_track` is negative for a car right of the path in CARLA's
> Y-right frame, so `e_cte > 0` means the car is left of the path and `+atan(...)` steers right,
> toward the path.

With `a_lat_max = 14` PID++ uses more of the grip budget and corners faster than the PID baseline
while staying under the measured tyre grip.

---

## The Curvature Velocity Profile

[`AnnotatedTrajectory.speed_profile`](../src/carla_mpc_bringup/carla_mpc_bringup/control/trajectory.py)
is the standard forward–backward method (MathWorks Velocity Profiler; UofT Self-Driving-Cars L5):

1. **Pointwise curvature limit:** `v = curve_speed_factor · √(a_lat_max / |κ|)`, capped at the cruise
   speed, floored at `min_speed_kmh` (so a single curvature spike at a junction can't stall the car).
2. **Backward pass:** `v[i] ≤ √(v[i+1]² + 2 · a_dec · ds)`, which guarantees it can brake *to* each
   curve in time.
3. **Forward pass:** `v[i] ≤ √(v[i-1]² + 2 · a_acc · ds)`, which bounds acceleration out of the curve.

`a_dec` is the `decel_ms2` config; `a_acc` is `a_accel`. It is O(n) over the static path, computed
once in `set_reference`.

---

## Current Tuned Values (`config/controller.yaml`)

```yaml
pid:        # safe baseline
  # Longitudinal speed PID (output: throttle/brake in [-1, 1])
  long_kp: 0.10
  long_ki: 0.15
  long_kd: 0.25
  # Lateral heading PID (output: steer in [-1, 1], look-ahead)
  lat_kp: 1.20
  lat_ki: 0.00
  lat_kd: 0.10
  # Cruise + floor
  default_target_speed_kmh: 120.0
  min_speed_kmh: 10.0
  # Forward-backward velocity profile (a_lat_max > 0 enables it)
  a_lat_max: 7.0
  curve_speed_factor: 0.85
  a_accel: 3.0
  decel_ms2: 3.0
  lat_lookahead_sec: 0.5
  # High-steer spin-out guard
  high_steer_threshold: 0.2
  high_steer_throttle_cap: 0.3
  # Actuation smoothing (EWMA over the last N commands; inherited by pidpp)
  cmd_smooth_n: 8
  cmd_smooth_decay: 0.6

pidpp:      # inherits ALL of pid:, then:
  a_lat_max: 14.0
  a_accel: 5.0          # faster curves
  lat_kff: 0.9          # curvature feed-forward
  steer_rate_max: 2.0   # anti-jerk slew limit
  stanley_k_e: 0.0
  stanley_k_soft: 1.0   # Stanley cross-track (OFF)
  d_filter_alpha: 0.7
  long_d_max: 0.6       # filtered + clamped longitudinal D
  long_kff: 0.60        # longitudinal target feed-forward
```

> Defaults change as the controllers are tuned; treat `config/controller.yaml` as the source of
> truth and read it for the live values.

---

## When to use which

Use **PID** for the simple, conservative, unambiguously-safe baseline. Use **PID++** for the same
safety envelope with faster cornering and tighter tracking, via its curvature feed-forward.

Both controllers are first-class entries in the benchmark harness: `run_benchmark --controllers
pid,pidpp,...` drives each one across the same scenario groups, and `benchmark_report` aggregates
per group. The `@<kmh>` speed-cap variants are MPC/MPCC-only; PID and PID++ carry the
fixed cruise from `default_target_speed_kmh`. See [entrypoints.md](entrypoints.md) for the harness.
