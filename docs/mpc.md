# MPC: bicycle trajectory tracking

A CasADi-defined optimal control problem (OCP) solved by acados SQP-RTI, wired
behind the same `ControllerInterface` as the PID. Run the tracking MPC with
`controller:=mpc` and the contouring variant (MPCC) with `controller:=mpcc`.

## Overview

- **Model (default `dyn`):** a 6-state **dynamic bicycle** with linear tyres,
  state `[X, Y, psi, vx, vy, r]`, input `[a, delta]`; see
  `control/vehicle_models.py` (`dynamic_bicycle_linear`). It models tyre slip and
  lateral dynamics, which gives the high-speed stability the slip-free kinematic
  model lacks. The 4-state **kinematic** model (`kin_cg`, `kinematic_bicycle_cg`)
  remains selectable as a lightweight fallback, alongside a rear-axle kinematic form
  (`kin_rear`) and a low-speed `blended` model.
- **OCP:** a NONLINEAR_LS contouring cost with a per-stage parametric reference,
  box input constraints, and SQP-RTI + partial-condensing HPIPM; see
  `control/mpc_ocp.py` (`build_tracking_solver`). The dynamic model uses an **IRK**
  integrator (explicit ERK NaNs on the stiff linear tyres), **state bounds** on
  `vx, vy, r` (keep the unsaturated-tyre QP conditioned), and the controller
  **seeds the trajectory with `vx>0`** (a zero warm-start linearises at `vx=0` →
  NaN). The kinematic model uses ERK and no state bounds.
- **Reference:** the controller's route → an arc-length spline `s ↦ (x, y, psi,
  kappa)` with a friction-limited two-pass speed profile, sampled into a per-solve
  horizon **window**; see `control/reference_path.py` (`ReferencePath`).
- **Actuation:** an analytic map `(a, delta) → (throttle, brake, steer)` from the
  vehicle's physics; see `control/actuation_map.py` (`ActuationMap`).
- **Controller:** `control/mpc_controller.py` (`MPCController`), which builds the
  solver from live `get_physics_control()`, solves each tick, applies the actuation
  map, and fills the Foxglove telemetry (predicted horizon, solve time, cost,
  tracking errors).

It is a drop-in replacement for the PID: `control_node` builds it via the same
factory, applies the command before each tick, and publishes the same topics. The
Foxglove layout already draws the predicted horizon (orange) and the solve-time
plot.

## Coordinate frame (important)

The OCP runs **entirely in CARLA coordinates** (left-handed, +Y to the right, yaw
+x→+y). The bicycle model is self-consistent there: positive `delta` and positive
CARLA `steer` both turn right (+Y), so `steer = delta / max_steer` directly, and
the reference heading is the CARLA-frame path tangent. Conversion to ROS/`gt_map`
(negate Y) happens only when `control_node` publishes the predicted/reference
paths and markers. Never mix the two.

> The slip-angle math in `vehicle_models.py` is written in the generic
> body-frame / REP-103 convention (x forward, slip `alpha` = velocity direction −
> wheel heading, `F_y = -C·alpha`). Because CARLA's frame differs from REP-103
> only by the sign of Y, the model is deployed unchanged in CARLA coordinates;
> the controller, OCP, actuation map, and reference path all operate in the CARLA
> frame.

## The OCP

Dynamic model state `x = [X, Y, psi, vx, vy, r]`, input `u = [a, delta]`,
body-frame dynamics with linear tyres (slip `α`, cornering stiffness `C`):

```text
α_f = atan2(vy + l_f·r, vx) − delta      F_yf = −C_f·α_f
α_r = atan2(vy − l_r·r, vx)              F_yr = −C_r·α_r
Ẋ = vx·cos psi − vy·sin psi              v̇x = a + vy·r
Ẏ = vx·sin psi + vy·cos psi              v̇y = −vx·r + (F_yf·cos delta + F_yr)/m
ṗsi = r                                  ṙ  = (l_f·F_yf·cos delta − l_r·F_yr)/I_z
```

(Kinematic fallback `kin_cg`: state `[X, Y, psi, v]`,
`beta = atan2(l_r·tan delta, L)`, `Ẋ = v·cos(psi+beta)`, `Ẏ = v·sin(psi+beta)`,
`ṗsi = (v/l_r)·sin beta`, `v̇ = a`.)

The cost (NONLINEAR_LS) is a **contouring decomposition** of the position error,
per stage `k`. Rotate `(X − Xref, Y − Yref)` into the path frame at `psi_ref`:

```text
e_lon =  cos(psi_ref)·dX + sin(psi_ref)·dY     (along-track, pacing)
e_lat = -sin(psi_ref)·dX + cos(psi_ref)·dY     (cross-track, stay on path)
y_k = [ e_lon, e_lat, wrap(psi-psi_ref), v-v_ref, a, delta, delta-delta_prev ]
J = Σ ½ yᵀ diag(q_lon, q_lat, q_psi, q_v, r_a, r_delta, r_ddelta) y
      + ½ y_Nᵀ (q_terminal_scale · diag(q_lon, q_lat, q_psi, q_v)) y_N
```

`q_lat` (stay on path) is weighted **high** and `q_lon` (pacing) **low**, so the
MPC tracks the path tightly while being free to choose its own speed rather than
being rigidly paced by the reference. The heading residual is wrapped with
`atan2(sin·, cos·)`. The **`delta − delta_prev`** residual penalises the steering
*change* from the last applied command (`delta_prev` is an OCP parameter refreshed
each solve). It damps micro left/right steer chatter. Constraints: `a ∈ [a_min,
a_max]`, `|delta| ≤ delta_max`, and a **soft lateral-accel limit** `|a_lat| ≤
a_lat_max` (a transient safety cap). The horizon is `N·dt = 50·0.06 = 3.0 s`.

Geometry and mass are read live from `Vehicle.get_physics_control()`: the
wheelbase `L` is the front/rear axle-midpoint distance, `l_r = L/2` (CG ≈
mid-wheelbase), plus mass and `max_steer`. Tyre stiffness (`Cf`, `Cr`) and yaw
inertia (`Iz`) are config estimates (`controller.yaml`). For best fidelity,
**identify them from CARLA** with steady-state circular tests; the `identify_grip`
tool does exactly this (it fits `Cf`, `Cr` per axle and measures the grip limit
for `a_lat_max`):

```bash
ros2 run carla_mpc_bringup identify_grip --map Town06 --spawn-index 0
```

The controller builds the dynamic `x0` from `VehicleState`: body-frame `vx, vy` by
rotating the CARLA world velocity by `−yaw`, and the yaw rate `r` by
**finite-differencing yaw** (both sign-consistent with the model *by construction*,
with no dependence on CARLA's angular-velocity convention). `vx` is floored at
`vx_min` so the tyre model isn't singular near standstill, and the measured `psi`
is re-aligned into the reference-heading branch so a `+π/−π` wrap (driving due
west) can't unwind into a phantom hard steer.

## Speed: a physical grip limit, not a prescribed target

The MPC is **not** handed a per-curve target speed. The curve speed comes from a single **physical lateral-grip budget**
`a_lat_max`: the fastest speed that keeps `v²·κ ≤ a_lat_max` is `v =
sqrt(a_lat_max / |κ|)`, capped at the straight cruise. `a_lat_max` is the one "how
hard to corner" knob. Raise it to go faster (CARLA's default Model-3 tyres grip
hard, ~1.5 g, so cornering near 1 g is fine). This friction-limited speed is the
*physically optimal* (racing-line) speed, and because the reference itself
respects the grip limit, the **planned lateral accel is bounded by construction**,
with no curve-entry spikes (which would spin the car). `a_lat` is *also* a soft OCP
constraint as a transient safety cap. For an extra margin, the per-curve target is
scaled by `curve_speed_factor` (default **0.85** for the MPC), so the car corners
at 85 % of the grip limit rather than *at* it.

`ReferencePath.window(x, y, v0, N, dt)` projects the ego onto the path and marches
the next `N+1` reference points at a *reachable* speed (a ramp from `v0` toward the
friction-limited profile, bounded by `a_long_acc`; the profile itself is two-pass
smoothed with `a_long_dec` so it pre-brakes before curves). Curvature is lightly
smoothed so waypoint-joint curvature steps don't cause an `a_lat` spike.

> Fully reference-free, **progress-maximising** speed (give the MPC only the path
> and let it pick the pace by maximising progress under tyre constraints) is the
> job of **MPCC** (below); the tracking MPC uses the friction-limited reference
> (smooth, bounded, fast).

## Tuning knobs (`config/controller.yaml`, `mpc:`)

| Knob | Effect |
|---|---|
| `model` | `dyn` (dynamic, fixes weave; default), `kin_cg`/`kin_rear` (kinematic), or `blended`. |
| `Cf` / `Cr` / `Iz` | dynamic-model tyre stiffness / yaw inertia. Estimates; identify from CARLA (`identify_grip`) for fidelity. |
| `vx_min` / `vy_max` / `r_max` | dynamic-model state bounds (QP conditioning; rarely retuned). |
| `a_lat_max` | **the speed knob**: lateral-grip budget (m/s²); curve speed = `sqrt(a_lat_max/κ)`. Raise to corner faster. Default **13**. |
| `curve_speed_factor` | fraction of the grip-limited curve speed to actually target. Default **0.85** (corner at 85 % of grip). Do not set to 1.0 (overshoot). |
| `target_speed_kmh` | straight cruise cap. Default **150**. |
| `q_lat` / `q_lon` | cross-track (stay on path, high) / along-track (pacing, low). Keep `q_lat ≫ q_lon`. |
| `q_psi` / `q_v` | heading / speed tracking weight. |
| `r_a` / `r_delta` / `r_ddelta` | input penalties; `r_ddelta` is the steering-change penalty (anti-chatter). |
| `q_terminal_scale` | terminal-cost multiplier (stability). |
| `a_max` / `a_min` / `delta_max` | input bounds. `delta_max=0` → use the vehicle's physical max. |
| `a_long_acc` / `a_long_dec` | reachable-reference accel/brake shaping (pre-brake before curves). |
| `reference_smooth_m` / `lane_change_smooth_m` | path xy smoothing / extra smoothing on lane-change arcs. |
| `N` / `dt` | horizon length / step. `N·dt` is the lookahead (`N=50`, `dt=0.06` → **3.0 s**). |
| `rti_iters` / `steer_rate_max` | SQP-RTI iterations per solve / front-wheel steer-rate cap. |
| `a_max_throttle` / `a_max_brake` / `v_max` | analytic actuation-map limits (coarse; calibrate per vehicle). |

## MPCC (`controller:=mpcc`)

**Model Predictive Contouring Control**: the reference-free-speed controller. The
state augments the bicycle model with a **path-progress** `theta`; the input
augments with a **virtual progress speed** `v_theta` (`theta_dot = v_theta`).
Instead of a time-parametrised reference, the path is **space-parametrised**: the
car maximises progress while a **contouring** cost keeps it on the path. Files:
`control/mpcc_ocp.py` (`build_mpcc_solver`), `control/mpcc_controller.py`
(`MPCCController`); config block `mpcc:` in `controller.yaml`.

State `[X, Y, psi, vx, vy, r, theta]`, input `[a, delta, v_theta]`. Errors (path
tangent at `psi_p`): contouring `e_c = -(X − x_p) sin psi_p + (Y − y_p) cos psi_p`,
lag `e_l = (X − x_p) cos psi_p + (Y − y_p) sin psi_p`. Minimising `e_c` keeps the
car on the path; `e_l` keeps `theta` synced to the real arc length. The 6-state
bicycle is a **`blended_bicycle`**: kinematic below `v_blend` (where the linear
tyre's slip angles are hypersensitive as `vx → 0` and cause a launch wobble),
dynamic above, and sigmoid-blended for a smooth low-speed launch.

**Changing-route trick:** the route is continuously extended, so a fixed path
B-spline can't be baked into the solver (it would need regeneration). Instead the
path enters as **per-stage parameters**: a local *arc-linearisation* around the
warm-start `theta_k` (`x_p = x_ref + cos psi_ref·(theta − theta_lin)`, …), updated
every solve from the spline. The solved `theta` trajectory feeds back as the next
warm-start. No regeneration, ever.

**Curve speed (a friction-limited feed-forward):** with **linear tyres there is no
grip saturation**, so a soft friction circle alone can't cap curve speed (the
optimiser games the slack). So the per-stage `v_theta` and `vx` **target** = the
friction-limited curve speed `curve_speed_factor · sqrt(a_lat_max/|kappa|)` (a
feed-forward, reusing `ReferencePath`'s profile), and a friction-circle radius
`sqrt(a_lat² + a_long²) ≤ a_grip` is a **soft safety backstop**. `curve_speed_factor`
defaults to **0.90** for MPCC (corner at 90 % of the grip limit); do not set it to
1.0, because cornering *at* the limit throws the car wide on entry. The reference is smoothed
at implicit lane changes so the car doesn't brake hard or jerk through them. Truly
emergent grip-limited speed (no feed-forward) would need a saturating Pacejka tyre,
which is too fragile for real-time SQP-RTI.

**Steering smoothness:** MPCC's progress/θ-feedback adds steering-rate
chatter, damped by the **steering** penalties: the MPCC defaults are `r_delta=4` and
`r_ddelta=65` (the steer-change penalty `r_ddelta` is more than double the MPC's 30). The progress/speed weights `q_vtheta` and
`q_vx` are both **4**; the steering penalties carry the anti-wobble, the speed
weights drive the pace.

**Horizon (anticipation):** both controllers run **`N=50`** (MPC 3.0 s @
`dt=0.06`, MPCC 2.5 s @ `dt=0.05`). `N=50` gives enough lookahead to anticipate a
roundabout or curve and slow before it: a shorter ~1.5 s horizon (≈21 m @ 50 km/h)
can't see the curve in time, so the car enters too fast and runs wide, while a much longer horizon costs solve time without improving tracking. N=50 balances anticipation against solve cost.

**MPCC config (`config/controller.yaml`, `mpcc:`):** straight cap
`target_speed_kmh=180`, grip budget `a_lat_max=15`, `curve_speed_factor=0.90`,
contouring/lag weights `q_c=20`/`q_l=2`, friction-circle backstop `a_grip=16`, and
`v_blend=4` for the kinematic↔dynamic launch blend. Robustness mirrors the dyn MPC:
IRK integrator, `vx, vy, r` state bounds, a `vx>0` trajectory seed, and body-frame
velocity + finite-diff yaw rate for `x0`. The first (cold) solve runs extra SQP
iterations so launch never applies a command from an under-converged horizon.

## Camera overlay

`predicted_path_overlay` projects the predicted (orange) and reference (blue) paths
into the front camera and republishes `/controller/camera_overlay/image` (the
Foxglove camera panel shows it). It subscribes to `/controller/predicted_path` and
`/controller/reference_path_3d` (the route with terrain z). Projection is done in
controlled frames: gt_map → ego body (from `gt/odom`) → camera link (mount from
`vehicle.yaml`) → optical → pixels. Intrinsics **K are computed from the camera
`fov` + resolution in `vehicle.yaml`** (CARLA's exact pinhole: `fx = fy = W /
(2·tan(fov/2))`, `cx = W/2`, `cy = H/2`), *not* from `/camera_info`, so a QoS or
latching mismatch on `/camera_info` can't silently disable the overlay. No cv_bridge
(numpy-2 ABI); a venv OpenCV draws it.

**Time sync:** the image arrives later than the (lightweight) odom, so the overlay
buffers odom by timestamp and **interpolates the ego pose to the image's stamp**
(and picks the predicted path nearest that stamp) before projecting; otherwise a
world-fixed path is drawn through a camera pose from a different time and *slides on
the road / jitters* (≈1.4 m at 28 m/s for 50 ms of lag).

## Known limitations

- **Tyre stiffness is estimated** (`Cf`/`Cr`/`Iz` config defaults), and the model is a
  linear-tyre approximation, not CARLA's full vehicle (Pacejka tyres, suspension), so expect
  some residual mismatch. Identify `Cf`, `Cr` from CARLA with `identify_grip` (steady-state
  circular tests) for best fidelity. If the dynamic model misbehaves, fall back to
  `model: kin_cg`.
- **Analytic actuation map** → coarse; the car may under/over-shoot speed. A
  data-driven calibration would reduce this.
- **Speed is friction-limited via `a_lat_max`** (a physical grip budget) for the
  tracking MPC, not fully reference-free. MPCC removes the per-curve time
  reference, but its curve speed is still a friction-limited feed-forward; truly
  emergent grip-limited speed would need a saturating Pacejka tyre. Curve-entry
  `a_lat` can spike at sharp route-waypoint curvature steps; lower `a_lat_max` if
  the car slides in CARLA.
- **Δδ smoothness** uses a horizon-constant `delta_prev` parameter; a
  state-augmented per-stage Δδ would be cleaner.
- **State = ground truth** from CARLA (no estimator in the loop).

## Run

```bash
# both shells: cd ~/autonomousdriving/mpc && source setup_env.sh
# shell 1: CARLA
cd $CARLA_ROOT && ./CarlaUE4.sh -RenderOffScreen --ros2
# shell 2: MPC (use controller:=mpcc for the contouring controller)
ros2 launch carla_mpc_bringup carla_pid.launch.py controller:=mpc
```

Append an `@<kmh>` speed cap to run a capped variant at runtime without rebuilding
the solver, e.g. `controller:=mpcc@100`.

The first launch builds and compiles the acados solver (logged as
`[mpc] building acados solver...`). The generated C lands in
`$CARLA_MPC_ACADOS_DIR` (default `~/.cache/carla_mpc_acados`). For Foxglove, import
the layout from
[`../src/carla_mpc_bringup/foxglove/carla_mpc_layout.json`](../src/carla_mpc_bringup/foxglove/carla_mpc_layout.json):
the 3D orange line is the predicted horizon, the front-camera panel shows the path
overlay, and the MPC solve-time plot lights up. Every run auto-records to
`logs/runs/<name>/`; review it with:

```bash
ros2 run carla_mpc_bringup review_run logs/runs/<name> --plot
```
