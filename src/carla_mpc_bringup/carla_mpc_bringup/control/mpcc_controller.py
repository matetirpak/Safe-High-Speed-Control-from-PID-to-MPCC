"""Model Predictive Contouring Control (MPCC) controller for the ego.

Wrap the dynamic-bicycle OCP from ``mpcc_ocp.py``, which augments the state with a
path-progress arc-length ``theta`` and a virtual progress speed ``v_theta``. Each tick
builds ``x0 = [X, Y, psi, vx, vy, r, theta]`` (body-frame velocity, finite-diff yaw
rate, nearest arc-length on the spline), arc-linearises the path around the warm-start
``theta_k`` to set the per-stage params and the friction-limited ``v_theta`` target,
solves for ``u0 = (a, delta, v_theta)``, applies ``(a, delta)``, and feeds the solved
``theta`` trajectory back as the next warm-start.

The contouring cost keeps the car on the path, the progress target drives speed, and
the per-stage friction-limited ``v_theta`` target slows it for curves. Reuses
``ReferencePath`` for the geometry spline and speed profile.
"""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np

from carla_mpc_bringup.control.actuation_map import ActuationMap
from carla_mpc_bringup.control.controller_interface import (
    ControlCommand,
    ControllerDebug,
    ControllerInterface,
    VehicleState,
)
from carla_mpc_bringup.control.mpc_controller import extract_bicycle_params
from carla_mpc_bringup.control.mpcc_ocp import build_mpcc_solver
from carla_mpc_bringup.control.reference_path import ReferencePath
from carla_mpc_bringup.core.config_loader import MpccConfig


class MPCCController(ControllerInterface):
    """SQP-RTI contouring controller driving the ego along a ReferencePath."""

    name = "mpcc"

    def __init__(self, ego, cfg: MpccConfig, dt_world: float,
                 fixed_target_speed_kmh: float = 0.0):
        self.cfg = cfg
        self.dt_world = dt_world
        self.fixed_target = fixed_target_speed_kmh
        self._prev_delta = 0.0
        self._prev_yaw = None
        # Shallow EWMA smoothing of (accel, steer); MPCC is already smooth, so n stays small.
        # n <= 1 disables it.
        self.cmd_n = int(getattr(cfg, "cmd_smooth_n", 0))
        _decay = float(np.clip(getattr(cfg, "cmd_smooth_decay", 0.4), 0.0, 0.999))
        if self.cmd_n > 1:
            self._cmd_w = _decay ** np.arange(self.cmd_n - 1, -1, -1)
            self._a_hist: deque[float] = deque(maxlen=self.cmd_n)
            self._d_hist: deque[float] = deque(maxlen=self.cmd_n)
        else:
            self._cmd_w = None
        self._theta_ws = None              # warm-start progress trajectory (N+1,)
        self._warm = False                 # has the solver a valid previous solution to keep?
        self.params, max_steer_rad = extract_bicycle_params(ego, cfg)
        self.delta_max = cfg.delta_max if cfg.delta_max > 0 else max_steer_rad
        self.actuation = ActuationMap.from_physics(max_steer_rad, cfg)

        print(f"[mpcc] vehicle: L={self.params.L:.3f} l_r={self.params.l_r:.3f} "
              f"m={self.params.m:.0f} max_steer={math.degrees(max_steer_rad):.1f} deg")
        print(f"[mpcc] building acados MPCC solver (N={cfg.N}, dt={cfg.dt})...")
        t0 = time.time()
        self.solver, self.nx, self.N, self.dt = build_mpcc_solver(
            self.params, cfg, self.delta_max)
        print(f"[mpcc] solver ready in {time.time() - t0:.1f}s (nx={self.nx}).")

        self._ref: ReferencePath | None = None
        self._debug = ControllerDebug()

    # ----------------------------------------------------------------- interface
    def set_reference(self, route) -> None:
        if route is None:
            raise ValueError("MPCCController.set_reference: route is None.")
        xy = [(wp.transform.location.x, wp.transform.location.y) for wp, _ in route]
        self._ref = ReferencePath(xy, self.cfg, fixed_target_kmh=self.fixed_target,
                                  road_options=[opt for _, opt in route])
        self._theta_ws = None              # re-frame progress warm-start onto the new route

    def _at(self, theta: np.ndarray):
        """Interpolate (x, y, psi, kappa, v) on the spline at arc-length theta."""
        r = self._ref
        return (np.interp(theta, r.s, r.x), np.interp(theta, r.s, r.y),
                np.interp(theta, r.s, r.psi), np.interp(theta, r.s, r.kappa),
                np.interp(theta, r.s, r.v))

    def _ewma(self, hist, value: float) -> float:
        """Return the EWMA over the window `hist` (last N), newest weighted most."""
        hist.append(value)
        w = self._cmd_w[-len(hist):]
        return float(np.dot(w, np.asarray(hist, dtype=float)) / w.sum())

    def step(self, state: VehicleState) -> ControlCommand:
        if self._ref is None:
            return ControlCommand()

        # Body-frame velocity + finite-diff yaw rate (sign-safe; see mpc_controller).
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        vx = max(c * state.vx + s * state.vy, self.cfg.vx_min)
        vy = float(np.clip(-s * state.vx + c * state.vy, -self.cfg.vy_max, self.cfg.vy_max))
        if self._prev_yaw is None:
            r = 0.0
        else:
            r = math.atan2(math.sin(state.yaw - self._prev_yaw),
                           math.cos(state.yaw - self._prev_yaw)) / max(self.dt_world, 1e-3)
        self._prev_yaw = state.yaw
        r = float(np.clip(r, -self.cfg.r_max, self.cfg.r_max))

        theta0 = float(self._ref.s[self._ref._nearest_index(state.x, state.y)])  # noqa: SLF001
        # Align heading to the reference frame: psi_ref(theta0) + wrapped(yaw - psi_ref).
        # The measured yaw snaps +pi <-> -pi when heading due west; feeding that raw makes x0's
        # psi jump ~2*pi vs the continuous warm-start, so the solver unwinds a full turn into a
        # phantom hard steer toward the wall. This keeps psi continuous and in the same branch.
        psir0 = float(self._at(np.array([theta0]))[2][0])
        psi = psir0 + math.atan2(math.sin(state.yaw - psir0), math.cos(state.yaw - psir0))
        x0 = np.array([state.x, state.y, psi, vx, vy, r, theta0], dtype=float)

        # Warm-start progress trajectory (and a sane initial guess with vx>0).
        if self._theta_ws is None:
            if self._warm:
                # Route was just rebuilt (extension), not teleported: keep the previous
                # solution's spatial states (X, Y, vx, vy, r) since the car is still on this
                # road, and only re-frame theta to the new arc length and re-align psi to the
                # new (possibly re-branched) reference frame. Cold-reseeding here makes the car
                # wiggle right after the first curve or extension.
                v0 = max(vx, 5.0)
                self._theta_ws = theta0 + np.arange(self.N + 1) * self.dt * v0
                _, _, gpsi, _, _ = self._at(self._theta_ws)
                for k in range(self.N + 1):
                    xk = self.solver.get(k, "x")
                    xk[2] = gpsi[k] + math.atan2(math.sin(xk[2] - gpsi[k]),
                                                 math.cos(xk[2] - gpsi[k]))
                    xk[6] = self._theta_ws[k]
                    self.solver.set(k, "x", xk)
            else:
                # Cold start (launch, usually from standstill): seed a physically consistent
                # speed ramp from the current vx so theta/X/Y/vx agree, and seed the controls
                # too. Paired with the reachable yref clamp below, this stops the first
                # under-converged solve from trading an unreachable curve-speed target for
                # steering, which causes launch drift and a folded horizon.
                vseed = np.minimum(vx + self.cfg.a_long_acc * np.arange(self.N + 1) * self.dt,
                                   getattr(self._ref, "cruise", vx + 1.0))
                self._theta_ws = theta0 + self.dt * np.concatenate([[0.0], np.cumsum(vseed[:-1])])
                gx, gy, gpsi, _, _ = self._at(self._theta_ws)
                for k in range(self.N + 1):
                    self.solver.set(k, "x", np.array([gx[k], gy[k], gpsi[k],
                                                      vseed[k], 0.0, 0.0, self._theta_ws[k]]))
                    if k < self.N:                       # terminal stage N has no control input
                        self.solver.set(k, "u", np.array([0.0, 0.0, vseed[k]]))

        xr, yr, psir, kr, vt = self._at(self._theta_ws)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        for k in range(self.N + 1):
            self.solver.set(k, "p", np.array([xr[k], yr[k], psir[k], kr[k],
                                              self._theta_ws[k], self._prev_delta]))
            if k < self.N:
                # Clamp the speed target to what is reachable from the current speed along an
                # a_long_acc ramp. Without it, at launch the target is the full cruise speed
                # while stopped (unreachable), so the solve trades speed for steering and the
                # car drifts. Inactive once up to speed.
                vt_k = min(float(vt[k]), vx + self.cfg.a_long_acc * k * self.dt)
                self.solver.set(k, "yref", np.array([0., 0., 0., 0., 0., vt_k, vt_k]))

        t = time.perf_counter()
        status = 0
        # Converge the first (cold) solve much harder so launch never applies a command from an
        # under-converged horizon; warm ticks need only rti_iters (a one-time cost).
        n_iter = 20 if not self._warm else max(1, self.cfg.rti_iters)
        for _ in range(n_iter):
            status = self.solver.solve()
        solve_ms = (time.perf_counter() - t) * 1e3
        self._warm = True              # solver now holds a solution worth keeping across rebuilds

        u0 = self.solver.get(0, "u")
        a_des, delta_des = float(u0[0]), float(u0[1])
        if self.cfg.steer_rate_max > 0:
            mx = self.cfg.steer_rate_max * self.dt_world
            delta_des = float(np.clip(delta_des, self._prev_delta - mx, self._prev_delta + mx))
        if self._cmd_w is not None:        # shallow EWMA smoothing of the applied (accel, steer)
            a_des = self._ewma(self._a_hist, a_des)
            delta_des = self._ewma(self._d_hist, delta_des)
        self._prev_delta = delta_des

        # Feed the solved progress trajectory back as the next warm-start.
        self._theta_ws = np.array([float(self.solver.get(k, "x")[6])
                                   for k in range(self.N + 1)])

        throttle, brake, steer = self.actuation.apply(a_des, delta_des, state.speed)
        cmd = ControlCommand(throttle=throttle, brake=brake, steer=steer).clamp()
        self._debug = self._make_debug(state, theta0, vt[0], solve_ms, status)
        return cmd

    def get_debug(self) -> ControllerDebug:
        return self._debug

    def trajectory_heatmap(self):
        r = self._ref
        if r is None:
            return None
        idx = np.arange(0, len(r.s), 2)
        return [(float(r.x[i]), float(r.y[i]), float(r.v[i] * 3.6), float(r.kappa[i]))
                for i in idx]

    # ----------------------------------------------------------------- telemetry
    def _make_debug(self, state, theta0, v_target, solve_ms, status) -> ControllerDebug:
        pred = [(float(self.solver.get(k, "x")[0]), float(self.solver.get(k, "x")[1]))
                for k in range(self.N + 1)]
        xr, yr, psir, _, _ = self._at(np.array([theta0]))
        Xr, Yr, PSr = float(xr[0]), float(yr[0]), float(psir[0])
        cte = -math.sin(PSr) * (state.x - Xr) + math.cos(PSr) * (state.y - Yr)
        head = math.degrees(math.atan2(math.sin(state.yaw - PSr), math.cos(state.yaw - PSr)))
        try:
            cost = float(self.solver.get_cost())
        except Exception:  # noqa: BLE001
            cost = None
        if status != 0:
            print(f"[mpcc] WARN acados status={status} (solve {solve_ms:.1f} ms)")
        return ControllerDebug(
            target_speed_kmh=float(v_target) * 3.6,
            cross_track_error_m=float(cte),
            heading_error_deg=float(head),
            target_point_xy=(pred[-1] if pred else None),
            predicted_path_xy=pred,
            solve_time_ms=float(solve_ms),
            cost=cost,
        )

