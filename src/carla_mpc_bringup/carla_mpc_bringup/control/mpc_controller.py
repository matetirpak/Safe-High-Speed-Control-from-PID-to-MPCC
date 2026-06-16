"""Trajectory-tracking MPC for a Carla ego, behind the shared controller interface.

A CasADi-defined OCP solved by acados SQP-RTI, wired behind the same
``ControllerInterface`` the PID uses, so ``controller:=mpc`` swaps it in with no
plumbing changes and reuses the existing prediction-line and solve-time telemetry.

Each control step maps a ``VehicleState`` (Carla frame) to ``x0``, marches an
``N+1``-state reference window along the path with the friction-limited speed
profile, solves the OCP, takes the first input ``u0 = (a, delta)``, converts it to
Carla actuation (throttle, brake, steer), and publishes telemetry (predicted
horizon, solve time, cost, tracking errors) via ``get_debug``.

The OCP runs entirely in Carla coordinates (left-handed, +Y to the right). The
kinematic model is self-consistent there: positive delta and positive Carla steer
both turn right (+Y), so ``steer = delta / max_steer`` directly. Conversion to the
ROS/gt_map frame happens only at publish time in ``control_node``.

Vehicle parameters (wheelbase, mass, max steer) are read live from
``Vehicle.get_physics_control()``; the CoG is approximated at mid-wheelbase (used as l_r for
both the kinematic and dynamic models). The actuation map is analytic.

Exports
-------
extract_bicycle_params : Read bicycle parameters from a live actor's physics.
MPCController : Controller-interface MPC implementation.
"""
from __future__ import annotations

import math
import time

import numpy as np

from carla_mpc_bringup.control.actuation_map import ActuationMap
from carla_mpc_bringup.control.controller_interface import (
    ControlCommand,
    ControllerDebug,
    ControllerInterface,
    VehicleState,
)
from carla_mpc_bringup.control.mpc_ocp import build_tracking_solver
from carla_mpc_bringup.control.reference_path import ReferencePath
from carla_mpc_bringup.control.vehicle_models import BicycleParams
from carla_mpc_bringup.core.config_loader import MpcConfig


def extract_bicycle_params(ego, cfg: MpcConfig) -> tuple[BicycleParams, float]:
    """Build BicycleParams and physical max-steer from the live actor's physics.

    Parameters
    ----------
    ego : carla.Vehicle
        Live actor whose ``get_physics_control()`` supplies wheelbase, mass, and
        per-wheel max steer angle.
    cfg : MpcConfig
        Supplies the inertia and tyre stiffness used by the dynamic model.

    Returns
    -------
    tuple[BicycleParams, float]
        Bicycle parameters and the physical max steer angle in radians.
    """
    pc = ego.get_physics_control()
    w = pc.wheels

    def pos(i):
        return np.array([w[i].position.x, w[i].position.y, w[i].position.z])

    # Wheelbase = distance between front and rear axle midpoints (pose-invariant;
    # cm -> m). Wheel order 0=FL, 1=FR, 2=RL, 3=RR.
    L = float(np.linalg.norm(0.5 * (pos(0) + pos(1)) - 0.5 * (pos(2) + pos(3))) * 0.01)
    if not (1.0 < L < 5.0):       # sanity fallback to a known sedan wheelbase
        L = 2.85
    l_r = 0.5 * L                  # CoG approximated at mid-wheelbase
    max_steer_rad = math.radians(float(w[0].max_steer_angle))
    if not (0.2 < max_steer_rad < 1.5):
        max_steer_rad = 1.22
    params = BicycleParams(L=L, l_r=l_r, m=float(pc.mass),
                           Iz=cfg.Iz, Cf=cfg.Cf, Cr=cfg.Cr)
    return params, max_steer_rad


class MPCController(ControllerInterface):
    name = "mpc"

    def __init__(self, ego, cfg: MpcConfig, dt_world: float,
                 fixed_target_speed_kmh: float = 0.0):
        self.cfg = cfg
        self.dt_world = dt_world
        self.fixed_target = fixed_target_speed_kmh
        self._is_dyn = cfg.model in ("dyn", "blended")
        self._prev_delta = 0.0          # last applied steer, for the steer-rate limit
        self._prev_yaw = None           # for finite-diff yaw rate in the dynamic x0
        self._inited = False            # trajectory guess seeded (dynamic model needs vx>0)
        self.params, max_steer_rad = extract_bicycle_params(ego, cfg)
        self.delta_max = cfg.delta_max if cfg.delta_max > 0 else max_steer_rad
        self.actuation = ActuationMap.from_physics(max_steer_rad, cfg)

        print(f"[mpc] vehicle: L={self.params.L:.3f} m  l_r={self.params.l_r:.3f} m  "
              f"m={self.params.m:.0f} kg  max_steer={math.degrees(max_steer_rad):.1f} deg")
        print(f"[mpc] building acados solver (model={cfg.model}, N={cfg.N}, dt={cfg.dt})...")
        t0 = time.time()
        self.solver, self.nx, self.N, self.dt = build_tracking_solver(
            self.params, cfg, self.delta_max)
        print(f"[mpc] solver ready in {time.time() - t0:.1f}s (nx={self.nx}).")

        self._ref: ReferencePath | None = None
        self._debug = ControllerDebug()

    def _x0(self, state: VehicleState, psi_ref0: float) -> np.ndarray:
        """Build the initial state from a VehicleState measurement.

        Parameters
        ----------
        state : VehicleState
            Measured pose and velocity in the Carla world frame.
        psi_ref0 : float
            Reference heading at the nearest path point, in radians.

        Returns
        -------
        numpy.ndarray
            Kinematic ``[X, Y, psi, v]`` or dynamic ``[X, Y, psi, vx, vy, r]``,
            with body-frame velocities and a finite-diff yaw rate; vx is floored
            so the linear-tyre model is not singular near standstill.

        Notes
        -----
        psi is aligned to the reference-heading frame
        (``psi_ref0 + wrapped(yaw - psi_ref0)``) so the state heading stays in the
        same 2*pi branch as the unwrapped reference and warm-start. Otherwise the
        measured yaw snaps +pi <-> -pi when heading due west, x0's psi jumps ~2*pi
        while the warm-start stays put, and the solver unwinds a full turn into a
        phantom hard steer.
        """
        psi = psi_ref0 + math.atan2(math.sin(state.yaw - psi_ref0),
                                    math.cos(state.yaw - psi_ref0))
        if not self._is_dyn:
            return np.array([state.x, state.y, psi, state.speed], dtype=float)
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        vx = c * state.vx + s * state.vy           # rotate world velocity into body longitudinal
        vy = -s * state.vx + c * state.vy          # body lateral
        if self._prev_yaw is None:
            r = 0.0
        else:
            r = math.atan2(math.sin(state.yaw - self._prev_yaw),
                           math.cos(state.yaw - self._prev_yaw)) / max(self.dt_world, 1e-3)
        self._prev_yaw = state.yaw
        vx = max(vx, self.cfg.vx_min)
        r = float(np.clip(r, -self.cfg.r_max, self.cfg.r_max))
        vy = float(np.clip(vy, -self.cfg.vy_max, self.cfg.vy_max))
        return np.array([state.x, state.y, psi, vx, vy, r], dtype=float)

    def _init_guess(self, x0: np.ndarray, ref: np.ndarray) -> None:
        """Seed the solver trajectory with vx>0 before the first solve.

        Linearising the dynamic model at vx=0 yields NaN, so the guess floors the
        longitudinal speed; once warm-started the solver self-sustains.
        """
        for k in range(self.N + 1):
            xg = np.zeros(self.nx)
            xg[0], xg[1], xg[2] = ref[k, 0], ref[k, 1], ref[k, 2]
            xg[3] = max(ref[k, 3], 5.0)            # keep v / vx > 0
            self.solver.set(k, "x", xg)
        self._inited = True

    # ----------------------------------------------------------------- interface
    def set_reference(self, route) -> None:
        if route is None:
            raise ValueError("MPCController.set_reference: route is None.")
        xy = [(wp.transform.location.x, wp.transform.location.y) for wp, _ in route]
        self._ref = ReferencePath(xy, self.cfg, fixed_target_kmh=self.fixed_target,
                                  road_options=[opt for _, opt in route])

    def step(self, state: VehicleState) -> ControlCommand:
        if self._ref is None:
            return ControlCommand()                      # coast until a route is set

        ref = self._ref.window(state.x, state.y, state.speed, self.N, self.dt)
        x0 = self._x0(state, float(ref[0, 2]))

        if not self._inited:
            self._init_guess(x0, ref)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        # p = [x_ref (4), delta_prev]; delta_prev penalises steering changes.
        for k in range(self.N + 1):
            self.solver.set(k, "p", np.append(ref[k], self._prev_delta))

        # Several SQP-RTI iterations: each re-linearises around the previous warm
        # solution, so the solve converges within the tick and never applies a
        # cold-start control. Each solve is well under the 50 ms world tick
        # (carla.yaml fixed_delta_seconds=0.05), so this stays real-time.
        t = time.perf_counter()
        status = 0
        for _ in range(max(1, self.cfg.rti_iters)):
            status = self.solver.solve()
        solve_ms = (time.perf_counter() - t) * 1e3

        u0 = self.solver.get(0, "u")
        a_des, delta_des = float(u0[0]), float(u0[1])

        # Rate-limit the commanded front-wheel angle to suppress jitter, since the
        # OCP carries no explicit steering-rate cost.
        if self.cfg.steer_rate_max > 0:
            max_step = self.cfg.steer_rate_max * self.dt_world
            delta_des = float(np.clip(delta_des, self._prev_delta - max_step,
                                      self._prev_delta + max_step))
        self._prev_delta = delta_des

        throttle, brake, steer = self.actuation.apply(a_des, delta_des, state.speed)
        cmd = ControlCommand(throttle=throttle, brake=brake, steer=steer).clamp()

        self._debug = self._make_debug(state, ref, solve_ms, status)
        return cmd

    def get_debug(self) -> ControllerDebug:
        return self._debug

    def trajectory_heatmap(self):
        """Return ``(x, y, speed_kmh, kappa)`` per strided reference sample.

        Exposes the friction-limited speed profile and curvature the MPC is
        tracking. Read-only view of ReferencePath with no effect on control.
        """
        r = self._ref
        if r is None:
            return None
        idx = np.arange(0, len(r.s), 2)
        return [(float(r.x[i]), float(r.y[i]), float(r.v[i] * 3.6), float(r.kappa[i]))
                for i in idx]

    # ----------------------------------------------------------------- telemetry
    def _make_debug(self, state, ref, solve_ms, status) -> ControllerDebug:
        # Predicted horizon in Carla xy; control_node converts it to gt_map.
        pred = [(float(self.solver.get(k, "x")[0]), float(self.solver.get(k, "x")[1]))
                for k in range(self.N + 1)]
        # Tracking errors relative to the nearest reference point ref[0].
        Xr, Yr, psir, vr = ref[0]
        cte = -math.sin(psir) * (state.x - Xr) + math.cos(psir) * (state.y - Yr)
        head_err = math.degrees(math.atan2(math.sin(state.yaw - psir),
                                           math.cos(state.yaw - psir)))
        try:
            cost = float(self.solver.get_cost())
        except Exception:  # noqa: BLE001
            cost = None
        if status != 0:
            print(f"[mpc] WARN acados status={status} (solve {solve_ms:.1f} ms)")
        return ControllerDebug(
            target_speed_kmh=float(vr) * 3.6,
            cross_track_error_m=float(cte),
            heading_error_deg=float(head_err),
            target_point_xy=(float(ref[-1, 0]), float(ref[-1, 1])),
            predicted_path_xy=pred,
            solve_time_ms=float(solve_ms),
            cost=cost,
        )
