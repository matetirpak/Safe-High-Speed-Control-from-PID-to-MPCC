"""Curvature-aware PID path follower for the Carla ego.

Wraps CARLA's `agents.navigation` PID controllers behind the
`ControllerInterface`, parameterised by `PidConfig`:

  * Longitudinal -- `PIDLongitudinalController` tracks a target speed (km/h) set
    by a curvature-aware policy: it slows ahead of the sharpest upcoming turn
    within a look-ahead, using a braking-distance margin.
  * Lateral -- `PIDLateralController` steers toward a look-ahead point on the
    path (pure-pursuit-style), the look-ahead growing with speed.

Path geometry (max angle ahead, waypoint-at-distance, point-to-segment) is
precomputed once per trajectory in `AnnotatedTrajectory` (`set_reference`), so
the per-tick `step` is O(1) lookups. When the route is extended, `set_reference`
runs again and the trajectory is rebuilt; the forward-local projection re-locks
progress so the car doesn't skip the leftover path or jump onto a looped-back
new segment.

The high-steer throttle clip (`high_steer_*`) is a hard spin-out guard: the
ad-hoc longitudinal/lateral coupling that MPC removes.
"""
from __future__ import annotations

import math
from collections import deque
from types import SimpleNamespace

import numpy as np

from agents.navigation.controller import (
    PIDLateralController,
    PIDLongitudinalController,
)

from carla_mpc_bringup.control.controller_interface import (
    ControlCommand,
    ControllerDebug,
    ControllerInterface,
    VehicleState,
)
from carla_mpc_bringup.control.trajectory import AnnotatedTrajectory
from carla_mpc_bringup.core.config_loader import PidConfig


def _target_point(x: float, y: float):
    """Build a duck-typed waypoint for CARLA's PIDLateralController.

    The controller reads `.transform.location` and forces z=0, so the steering
    target can come straight from the precomputed spline instead of a route
    Waypoint.

    Parameters
    ----------
    x, y : float
        Target point in CARLA world coordinates (m).
    """
    return SimpleNamespace(transform=SimpleNamespace(
        location=SimpleNamespace(x=float(x), y=float(y), z=0.0)))


class PIDController(ControllerInterface):
    name = "pid"

    def __init__(self, ego_vehicle, cfg: PidConfig, dt: float,
                 fixed_target_speed_kmh: float = 0.0):
        self.cfg = cfg
        self.dt = dt
        self.fixed_target_speed_kmh = fixed_target_speed_kmh

        self.long_controller = PIDLongitudinalController(
            ego_vehicle, K_P=cfg.long_kp, K_I=cfg.long_ki, K_D=cfg.long_kd, dt=dt)
        self.lat_controller = PIDLateralController(
            ego_vehicle, K_P=cfg.lat_kp, K_I=cfg.lat_ki, K_D=cfg.lat_kd, dt=dt)

        self.traj: AnnotatedTrajectory | None = None
        self._pid_target_kmh = np.zeros(0)        # per-sample curve-limited target (heatmap)
        self.max_angle_history: deque[float] = deque(maxlen=2000)
        self.fallback_speed = 0.0
        # Forward-backward curvature velocity profile. When a_lat_max>0 it replaces the
        # angle-ahead heuristic below, which over-speeds inside curves (slows before, speeds
        # up within) and causes corner-cutting and lane invasions. a_lat_max=0 keeps the heuristic.
        self.a_lat_max = float(getattr(cfg, "a_lat_max", 0.0))
        self.curve_speed_factor = float(getattr(cfg, "curve_speed_factor", 0.85))
        self.a_accel = float(getattr(cfg, "a_accel", 3.0))
        self._v_profile = None
        # Actuation smoothing: exponentially-weighted average over the last N commands (most
        # recent weighted highest) for non-chattering, deployable output. N<=1 disables it.
        self.cmd_n = int(getattr(cfg, "cmd_smooth_n", 0))
        decay = float(np.clip(getattr(cfg, "cmd_smooth_decay", 0.6), 0.0, 0.999))
        if self.cmd_n > 1:
            # weights run oldest -> newest, so the most recent command carries the largest weight.
            self._cmd_w = decay ** np.arange(self.cmd_n - 1, -1, -1)
            self._accel_hist: deque[float] = deque(maxlen=self.cmd_n)
            self._steer_hist: deque[float] = deque(maxlen=self.cmd_n)
        else:
            self._cmd_w = None
        self._debug = ControllerDebug()

    # ----------------------------------------------------------------- interface
    def set_reference(self, route) -> None:
        if route is None:
            raise ValueError("PIDController.set_reference: route is None.")
        xy = [(wp.transform.location.x, wp.transform.location.y) for wp, _ in route]
        # Precompute the curve scan over the policy's look-ahead (cruise-based, so the
        # per-tick lookup is O(1)); a sliding window over ticks still smooths it below.
        la = max(self.cfg.long_lookahead_sec * self.cfg.default_target_speed_kmh / 3.6,
                 self.cfg.min_long_lookahead_dist)
        self.traj = AnnotatedTrajectory(xy, angle_lookahead_m=la)
        self._pid_target_kmh = self._curve_limited_profile_kmh()
        if self.a_lat_max > 0.0:                   # use the curvature velocity profile policy
            self._v_profile = self.traj.speed_profile(
                self.cfg.default_target_speed_kmh, self.a_lat_max, self.curve_speed_factor,
                self.cfg.decel_ms2, self.a_accel, min_kmh=self.cfg.min_speed_kmh)

    def step(self, state: VehicleState) -> ControlCommand:
        if self.traj is None:
            raise RuntimeError("PIDController.step before set_reference.")

        samp = self.traj.sample(state.x, state.y)     # O(1) forward-local lookup

        if self.fixed_target_speed_kmh > 0.0:
            target_speed_kmh = self.fixed_target_speed_kmh
        else:
            try:
                target_speed_kmh = self._speed_target_kmh(samp, state.speed)
            except Exception as e:  # noqa: BLE001  fail safe, never crash the loop
                print(f"[pid] speed-target error: {e}")
                target_speed_kmh = self.fallback_speed

        steering_wp = _target_point(*self.traj.point_at_arclength(
            samp["s"] + self.cfg.lat_lookahead_sec * state.speed))

        pid_accel = float(self.long_controller.run_step(target_speed_kmh))
        pid_steer = float(self.lat_controller.run_step(steering_wp))

        # High-steer spin-out guard (see module docstring).
        if math.fabs(pid_steer) > self.cfg.high_steer_threshold:
            pid_accel = min(pid_accel, self.cfg.high_steer_throttle_cap)

        cmd = self._to_command(pid_accel, pid_steer)

        loc = steering_wp.transform.location
        self._debug = ControllerDebug(
            target_speed_kmh=target_speed_kmh,
            cross_track_error_m=self.traj.cross_track(state.x, state.y),
            heading_error_deg=math.degrees(math.atan2(
                math.sin(samp["psi"] - state.yaw), math.cos(samp["psi"] - state.yaw))),
            target_point_xy=(loc.x, loc.y),
        )
        return cmd

    def get_debug(self) -> ControllerDebug:
        return self._debug

    def trajectory_heatmap(self):
        """Return the precomputed PID target annotation for the Foxglove heatmap.

        Returns
        -------
        list of tuple or None
            One `(x, y, speed_kmh, angle_ahead_deg)` per strided sample, in CARLA
            world coordinates. None before a reference is set.
        """
        t = self.traj
        if t is None:
            return None
        idx = np.arange(0, len(t.s), 2)
        return [(float(t.x[i]), float(t.y[i]), float(self._pid_target_kmh[i]),
                 float(t.angle_ahead[i])) for i in idx]

    # ------------------------------------------------------------- speed policy
    def _curve_limited_profile_kmh(self) -> np.ndarray:
        """Compute the per-sample curve-limited target speed.

        This is the steady-state angle->speed map without the runtime braking
        blend, precomputed for the heatmap.

        Returns
        -------
        numpy.ndarray
            Target speed (km/h) per trajectory sample.
        """
        c = self.cfg
        ang = self.traj.angle_ahead
        span = max(c.max_turn_angle_deg - c.no_slowdown_below_deg, 1e-6)
        sev = np.clip((ang - c.no_slowdown_below_deg) / span, 0.0, 1.0)
        factor = 1.0 - sev ** c.angle_to_speed_exponent
        curve = c.min_speed_kmh + (c.default_target_speed_kmh - c.min_speed_kmh) * factor
        return np.where(ang <= c.no_slowdown_below_deg, c.default_target_speed_kmh, curve)

    def _speed_target_kmh(self, samp: dict, current_speed_ms: float) -> float:
        """Compute the curve-speed target at the current sample.

        With a_lat_max>0 this reads the forward-backward velocity profile, which
        is curvature-correct inside curves (reasonable speed, no corner-cutting).
        With a_lat_max=0 it falls back to the angle-ahead heuristic.

        Parameters
        ----------
        samp : dict
            Current trajectory sample from `AnnotatedTrajectory.sample`.
        current_speed_ms : float
            Ego speed (m/s), used for the braking-distance blend.

        Returns
        -------
        float
            Target speed (km/h).
        """
        self.max_angle_history.append(samp["angle_ahead"])       # keep the debug curve angle alive
        if self._v_profile is not None:
            return float(self._v_profile[samp["i"]])
        c = self.cfg
        recent = list(self.max_angle_history)[-c.angle_sliding_window_n:]
        abs_angle_deg = sum(recent) / len(recent)

        if abs_angle_deg <= c.no_slowdown_below_deg:
            return c.default_target_speed_kmh

        severity = min(1.0, (abs_angle_deg - c.no_slowdown_below_deg) /
                       (c.max_turn_angle_deg - c.no_slowdown_below_deg))
        factor = 1.0 - severity ** c.angle_to_speed_exponent
        curve_speed_kmh = c.min_speed_kmh + (c.default_target_speed_kmh - c.min_speed_kmh) * factor

        braking_distance = (current_speed_ms ** 2) / (2 * c.decel_ms2) + c.curve_safety_margin_m
        distance_to_curve = samp["angle_dist"]
        if distance_to_curve > braking_distance:
            return c.default_target_speed_kmh
        t = max(0.0, distance_to_curve / braking_distance)
        return curve_speed_kmh + (c.default_target_speed_kmh - curve_speed_kmh) * t

    def _ewma(self, hist: "deque[float]", value: float) -> float:
        """Push `value` and return the exponentially-weighted average of `hist`.

        Weights `self._cmd_w[-n:]` (= decay^(n-1) .. decay^0) so the newest of the
        current n samples is weighted highest.
        """
        hist.append(value)
        w = self._cmd_w[-len(hist):]
        return float(np.dot(w, np.asarray(hist, dtype=float)) / w.sum())

    def _to_command(self, pid_accel: float, pid_steer: float) -> ControlCommand:
        pid_accel = float(np.clip(pid_accel, -self.cfg.max_throttle, self.cfg.max_throttle))
        pid_steer = float(np.clip(pid_steer, -self.cfg.max_steer, self.cfg.max_steer))
        # Smooth over the last N commands to kill per-tick chatter, which makes raw
        # PID output undeployable on real hardware.
        if self._cmd_w is not None:
            pid_accel = self._ewma(self._accel_hist, pid_accel)
            pid_steer = self._ewma(self._steer_hist, pid_steer)
        if pid_accel >= 0.0:
            throttle, brake = pid_accel, 0.0
        else:
            throttle, brake = 0.0, -pid_accel
        return ControlCommand(throttle=throttle, brake=brake, steer=pid_steer).clamp()
