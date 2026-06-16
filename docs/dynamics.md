# Vehicle dynamics and the MPC formulation

How the ego is modelled and how those models become the optimal-control problems
(OCPs) the controllers solve. The **code is the ground truth**: the models live in
[`control/vehicle_models.py`](../src/carla_mpc_bringup/carla_mpc_bringup/control/vehicle_models.py),
the tracking OCP in
[`control/mpc_ocp.py`](../src/carla_mpc_bringup/carla_mpc_bringup/control/mpc_ocp.py),
and the MPCC OCP in
[`control/mpcc_ocp.py`](../src/carla_mpc_bringup/carla_mpc_bringup/control/mpcc_ocp.py).
This document explains them; for tuning knobs see
[configuration.md](configuration.md) and for the controllers' runtime wiring see
[mpc.md](mpc.md).

---

## 1. Conventions

All models share one set of conventions. Mixing in the opposite convention is the
classic source of a controller that steers *away* from the path, so they are fixed
here once.

- **Axes (ROS REP-103 body frame):** `x` forward, `y` left, `z` up; angles in
  radians. The optimisation runs in CARLA's left-handed world frame (`+y` right),
  which is self-consistent for the kinematic state; conversion to the ROS `gt_map`
  frame happens only at publish time. See [coordinate frames](architecture.md).
- **Slip angle:** `α = (velocity direction) − (wheel heading)`, so the linear tyre
  force is `F_y = −C·α`. (This is the negative of Rajamani's slip-angle sign; the
  compensating sign in `F_y` makes the physical force identical: a positive steer
  produces a positive, turning lateral force.)
- **Steering `δ`:** effective front-wheel angle in radians, *after* the actuation
  map. The OCP commands `δ`; `control/actuation_map.py` turns it into CARLA steer.
- **Longitudinal accel `a`:** the commanded powertrain/brake acceleration. In the
  body frame the longitudinal channel is `v̇_x = a + v_y·r`. The `v_y·r` term is
  the centripetal coupling and is **not** dropped (it is a ~10 % effect at high yaw
  rate).
- **`atan2`, never `atan`:** keeps slip angles and the side-slip `β` well-defined
  off the `+x` axis and for reverse motion.

---

## 2. State and input spaces

| Model | State `x` | Input `u` | Used by |
|---|---|---|---|
| `kin_cg` | `[X, Y, ψ, v]` | `[a, δ]` | tracking MPC (kinematic option) |
| `kin_rear` | `[X, Y, ψ, v]` | `[a, δ]` | tracking MPC (kinematic option) |
| `dyn` | `[X, Y, ψ, v_x, v_y, r]` | `[a, δ]` | tracking MPC (**default**) |
| `blended` | `[X, Y, ψ, v_x, v_y, r]` | `[a, δ]` | tracking MPC option; MPCC body |
| MPCC | `[X, Y, ψ, v_x, v_y, r, θ]` | `[a, δ, v_θ]` | MPCC |

`r = ψ̇` is the yaw rate; `θ` is the MPCC path-progress parameter with `θ̇ = v_θ`.

---

## 3. Kinematic bicycle

Geometric model: no tyre forces, valid while tyres are not slipping (roughly up to
~0.4 g). Two reference points are provided.

**CG-referenced** (`kinematic_bicycle_cg`, the kinematic default), with side-slip
`β`:

```
β   = atan2(l_r · tan δ, L)
Ẋ   = v · cos(ψ + β)
Ẏ   = v · sin(ψ + β)
ψ̇   = (v / l_r) · sin β
v̇   = a
```

**Rear-axle-referenced** (`kinematic_bicycle_rear`, openpilot-style), no `β`:

```
Ẋ = v · cos ψ,   Ẏ = v · sin ψ,   ψ̇ = (v / L) · tan δ,   v̇ = a
```

The two are algebraically equivalent up to the fixed `l_r` CG offset; the rear form
drops `β` because the rear-axle velocity is aligned with body `x` by the no-slip
assumption. `v̇ = a` correctly has **no** `v_y·r` term: that coupling exists only in
the dynamic model's body-frame `v_x` channel.

---

## 4. Dynamic bicycle (linear tyres)

The default model (`dyn`). Adds body-frame lateral velocity `v_y` and yaw rate `r`,
so it captures the tyre slip that the kinematic model cannot, which is what makes it stable at high speeds.

Slip angles and linear tyre forces:

```
α_f = atan2(v_y + l_f·r, v_x) − δ          F_yf = −C_f · α_f
α_r = atan2(v_y − l_r·r, v_x)              F_yr = −C_r · α_r
```

Body dynamics:

```
Ẋ   = v_x·cos ψ − v_y·sin ψ
Ẏ   = v_x·sin ψ + v_y·cos ψ
ψ̇   = r
v̇_x = a + v_y·r                                     # centripetal coupling kept
v̇_y = −v_x·r + (F_yf·cos δ + F_yr) / m
ṙ   = (l_f·F_yf·cos δ − l_r·F_yr) / I_z
```

The `cos δ` projections of the front lateral force onto the body `y` and yaw axes
are retained. The model is **singular at `v_x = 0`** (the slip-angle `atan2` becomes
ill-conditioned), so `v_x` is floored at `vx_min` and the MPCC uses the blended
model below.

**One deliberate simplification:** the front-tyre *longitudinal* projection
`−F_yf·sin δ / m` is dropped from `v̇_x`; net longitudinal force is folded into the
commanded `a` through the actuation map. This is standard for tracking-grade NMPC
and is a ≲1 % effect except at large simultaneous steer and lateral load.

---

## 5. Tyre models

Two tyre models are used: linear (tracking) and blended-linear (MPCC).

**Linear**: `F_y = −C·α`. Accurate to about `|α| ≤ 4°`; beyond that real tyres
saturate and linear over-predicts force. Closed-loop feedback absorbs the
mismatch, so this is sufficient for tracking and moderate MPCC.

**Blended** (`blended_bicycle`, the MPCC body): a sigmoid in `v_x` blends the
dynamic RHS into a kinematic-equivalent one below `v_min` (`v_blend = 4 m/s`), where
the dynamic slip angles are hypersensitive and cause a start-up steering wobble. The
kinematic branch drives `v_y` and `r` toward their kinematic values with a 0.1 s
first-order relaxation rather than algebraically forcing them, which keeps the
6-state structure and a smooth, differentiable RHS.

---

## 6. Grip, friction, and `tire_friction` vs achievable g

Note the distinction between the PhysX friction parameter and achievable grip:

> CARLA's `wheel.tire_friction` (≈ **3.5** by default for the Model 3) is a **PhysX
> tyre-model parameter**, *not* the achievable lateral acceleration. The
> vehicle-level **sustained** grip is far lower, about **1.5–1.6 g** for the Model 3.

`tire_friction` scales the contact-patch friction inside PhysX, but the achievable
steady-state lateral `g` is shaped by the lateral-stiffness curve, load transfer,
and the slip the vehicle can reach before the rear breaks away. CARLA's default 3.5
is, in the simulator's own documentation, "unrealistically high" (real asphalt is
µ ≈ 0.8–1.2), yet vehicles still corner at a realistic ~1.5 g.

**What the codebase does with this:**

- [`tools/identify_grip.py`](../src/carla_mpc_bringup/carla_mpc_bringup/tools/identify_grip.py)
  runs a textbook steady-state constant-radius (skidpad) test and reports the
  **measured sustained grip** (~1.6 g). It also prints the nominal
  `tire_friction` so the two are never conflated. Per-axle forces come from the
  steady-state balance `F_yf = (l_r/L)·m·a_y`, `F_yr = (l_f/L)·m·a_y`, and the
  lateral accel is `a_y = v_x·r`.
- That measured grip sets the **`a_lat_max`** ceiling: `mpc a_lat_max = 13`
  (~1.3 g), `mpcc a_lat_max = 15` (~1.5 g), each just under the measured limit,
  with `curve_speed_factor` taking a further margin below it.

The grip tool reports a slightly **conservative** number by construction (feedback
throttle during sampling couples a little longitudinal slip, the top 5 % of samples
are discarded, and it stops at the first sign of yaw-rate saturation). It measures
*sustained* grip, which is the right quantity for `a_lat_max`; brief transient peaks
can be marginally higher.

---

## 7. Parameters and their sources

| Symbol | Meaning | Source |
|---|---|---|
| `L` | wheelbase | live from `get_physics_control()` wheel positions |
| `l_f, l_r` | CG → front/rear axle | **approximated** `l_f = l_r = L/2` (mid-wheelbase) |
| `m` | mass | live from `get_physics_control()` |
| `I_z` | yaw inertia | config (`controller.yaml`), ≈ `m·l_f·l_r` ≈ 3745 kg·m² |
| `C_f, C_r` | cornering stiffness | config, 120000 / 150000 N/rad (Model-3 estimates) |
| `δ_max` | steer limit | config `delta_max` (0.7 rad), else the physical max |

`L`, `m`, and the physical max-steer are read live per run
(`extract_bicycle_params`); the tyre/inertia parameters are config values you can
refine with `identify_grip`.

---

## 8. Tracking MPC: the OCP

A `NONLINEAR_LS` (nonlinear least-squares) tracking cost over the chosen model,
solved by acados SQP-RTI.

**Position-error decomposition (contouring form).** Instead of penalising raw
`(X−X_r, Y−Y_r)`, the error is rotated into the path frame at the reference heading
`ψ_r`:

```
e_lon =  cos ψ_r · (X−X_r) + sin ψ_r · (Y−Y_r)      # along-track (pacing)
e_lat = −sin ψ_r · (X−X_r) + cos ψ_r · (Y−Y_r)      # cross-track (stay on path)
```

so `e_lat` (stay on the path) can be weighted high and `e_lon` (pacing) low, letting
the MPC choose its own speed. Heading error is wrapped: `ψ_err = atan2(sin(ψ−ψ_r),
cos(ψ−ψ_r))`.

**Residual vector and weights.** Stage residual and diagonal weight `W`:

```
y   = [e_lon, e_lat, ψ_err, v_err, a, δ, Δδ]
W   = diag(q_lon, q_lat, q_psi, q_v, r_a, r_delta, r_ddelta)
W_e = q_terminal_scale · diag(q_lon, q_lat, q_psi, q_v)   # terminal: state residuals only
```

Weights are scaled in the spirit of Bryson's rule (1 / max-acceptable-error²) then
hand-tuned; `q_lat ≫ q_lon` is the key choice. See [configuration.md](configuration.md).

**Constraints.**
- Box: `a ∈ [a_min, a_max]`, `δ ∈ [−δ_max, δ_max]`.
- Dynamic models additionally box `v_x, v_y, r` to keep the linear-tyre QP
  conditioned.
- A **soft** lateral-acceleration cap `|a_lat| ≤ a_lat_max` (slacked) bounds
  transient curve load: `a_lat = v_x·r` (dynamic) or `v²·sin β / l_r` (kinematic).

**Solver.** SQP-RTI, partial-condensing HPIPM, Gauss-Newton Hessian, one SQP
iteration per `solve()` call (the controller takes `rti_iters` such calls per
control step; see [configuration.md](configuration.md)). The integrator is **IRK** for the (stiff linear-tyre) dynamic
models and **ERK** for the kinematic ones. Built once at startup and warm-started.

---

## 9. MPCC: the OCP

Model Predictive Contouring Control maximises progress along the path while staying
on it. The state augments the blended dynamic bicycle with the path parameter `θ`
(`θ̇ = v_θ`), and the input adds the virtual progress speed `v_θ`.

The reference path enters as **per-stage parameters** (`x_ref, y_ref, ψ_ref, κ_ref,
θ_lin, δ_prev`), not a baked-in interpolant, so the solver never regenerates as the
route is extended. Each stage linearises the path around `θ_lin`:

```
x_p = x_ref + cos ψ_ref · (θ − θ_lin)
y_p = y_ref + sin ψ_ref · (θ − θ_lin)
ψ_p = ψ_ref + κ_ref · (θ − θ_lin)

e_c = −(X − x_p)·sin ψ_p + (Y − y_p)·cos ψ_p          # contouring error (off-path)
e_l =  (X − x_p)·cos ψ_p + (Y − y_p)·sin ψ_p          # lag error (θ vs real arc length)
```

Minimising `e_c` keeps the car on the path and `e_l` keeps `θ` synced to true arc
length. Progress is driven by a per-stage `v_θ`/`v_x` target set to the
friction-limited curve speed (a positive-target quadratic, a valid MPCC variant of
the textbook linear `−γ·v_θ` reward).

A **friction-circle** soft backstop limits combined acceleration:

```
sqrt((v_x·r)² + a² + 1e-2) ≤ a_grip          # a_grip = 16 m/s², > a_lat_max
```

Same SQP-RTI / HPIPM / IRK solver setup as the tracking OCP, with `nx = 7, nu = 3`.

---

## 10. Known approximations and limitations

These are deliberate modelling choices, documented so they are not mistaken for
bugs:

- **`l_f = l_r = L/2`**: the CG is approximated at mid-wheelbase. Exact for the
  kinematic model; for the dynamic model it makes the front/rear axle split
  symmetric (the real Model 3 is slightly rear-biased). Identify the true CG offset
  for higher fidelity.
- **Dropped front-tyre longitudinal projection** in `v̇_x` (§4): folded into `a`.
- **`Δδ` penalty is against a per-stage scalar.** `δ_prev` is the *last applied*
  steering, written to every horizon node, so `Δδ = u_k − δ_prev` penalises each
  stage against one fixed value, not the true inter-stage `u_k − u_{k-1}`. It damps
  chatter against the realised command but is not a per-stage rate cost; a true rate
  penalty needs `δ` augmented into the state. A post-solve steer-rate clip
  (`steer_rate_max`) provides the inter-step limit instead.
- **Blended-model 0.1 s relaxation constant** (§5) is a hardcoded time constant,
  independent of the integrator step.

---

## 11. Verifying the dynamics

- **Self-test:** `python vehicle_models.py --R 50 --v 10` prints each model's
  dimensions and a steady-state circle cross-check: the kinematic Ackermann steer
  `atan2(L, R)` and the dynamic understeer prediction `δ = L/R + K_us·v²/R` with the
  understeer gradient `K_us = (m/L)(l_r/C_f − l_f/C_r)`. The two should agree to
  within the understeer correction (~0.2° at R=50 m, v=10 m/s).
- **Grip / stiffness:** `ros2 run carla_mpc_bringup identify_grip --map Town06`
  (steady-state skidpad; reports grip, `tire_friction`, and rough `C_f/C_r`).
- **References:** Rajamani, *Vehicle Dynamics and Control* (kinematic/dynamic
  bicycle, understeer gradient); Liniger et al. and the AMZ Driverless papers
  (MPCC, blended model, friction circle).
