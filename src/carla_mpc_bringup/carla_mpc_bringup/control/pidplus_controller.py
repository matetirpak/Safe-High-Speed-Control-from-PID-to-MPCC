"""PID++ controller: the baseline PID with a stronger, self-contained tracking core.

The lateral law sums three terms, then steer-rate limits the result (the two
added terms are converted from radians to normalized steer by dividing by
max_steer_rad; the look-ahead PID term is already in normalized steer):

    steer = look_ahead_heading_PID            # pure-pursuit-like heading feedback
          + atan(k_e * e_cte / (k_soft + v))   # Stanley cross-track, drives e_cte to 0
          + lat_kff * atan(L * kappa)          # Ackermann curvature feedforward, holds the curve

The look-ahead heading term anticipates entry, so it is kept rather than replaced
by a center-axle Stanley psi_e, which saturates and diverges in fast roundabout
turns. The cross-track term drives offset to zero and the feedforward compensates
curvature.

The longitudinal law is an inline speed PID with filtered, clamped derivative on
error plus conditional-integration anti-windup. Derivative-on-error (not on
measurement) preserves the brake kick the curve-speed policy needs on a target
step-down. The integral stops winding while the actuator saturates in the same
direction, e.g. behind the high-steer throttle cap. A low-passed feed-forward on
the target's rate (long_kff) tracks the profile ramp instead of lagging above it.

Config-gated under `pidpp:`: lat_kff, long_kff, steer_rate_max, stanley_k_e/k_soft,
d_filter_alpha, long_d_max. With k_e = lat_kff = 0 it degrades to the baseline
look-ahead PID, so the add-ons A/B cleanly.

Sign convention: trajectory.cross_track is negative for a car right of the path in
Carla's Y-right frame, so e_cte > 0 means the car is left of the path and +atan(...)
steers right, toward the path.
"""
from __future__ import annotations

import math

import numpy as np

from carla_mpc_bringup.control.controller_interface import (
    ControlCommand,
    ControllerDebug,
    VehicleState,
)
from carla_mpc_bringup.control.pid_controller import PIDController, _target_point
from carla_mpc_bringup.core.config_loader import PidConfig


class PIDPlusController(PIDController):
    name = "pidpp"

    def __init__(self, ego_vehicle, cfg: PidConfig, dt: float,
                 fixed_target_speed_kmh: float = 0.0):
        super().__init__(ego_vehicle, cfg, dt, fixed_target_speed_kmh)
        self.L, self.max_steer_rad = self._extract_geometry(ego_vehicle)
        self.lat_kff = float(getattr(cfg, "lat_kff", 0.0))
        self.steer_rate_max = float(getattr(cfg, "steer_rate_max", 0.0))   # normalized steer / s; 0 = off
        self.k_e = float(getattr(cfg, "stanley_k_e", 0.0))
        self.k_soft = float(getattr(cfg, "stanley_k_soft", 1.0))
        self._prev_steer = 0.0
        self.d_alpha = float(np.clip(getattr(cfg, "d_filter_alpha", 0.7), 0.0, 0.99))
        self.d_max = float(getattr(cfg, "long_d_max", 1.0))   # clamp on the D contribution (D_max)
        self._i = 0.0          # integral accumulator (km/h * s)
        self._d_f = 0.0        # filtered error derivative (km/h / s)
        self._e_prev = None    # previous speed error (km/h)
        self._tgt_prev = None  # previous target (km/h), for the feed-forward d(target)/tick
        self._tgt_rate_f = 0.0 # low-passed target rate (rejects per-tick target/index jitter)
        # The curvature velocity profile lives in the base PIDController; PID++ inherits it
        # and its pidpp config sets a higher a_lat_max for faster curves.

    @staticmethod
    def _extract_geometry(ego) -> tuple[float, float]:
        """Extract wheelbase and physical max-steer from the live actor.

        Returns
        -------
        tuple of float
            (L, max_steer_rad): wheelbase in m and front max-steer angle in rad.

        Notes
        -----
        Wheel order is 0=FL, 1=FR, 2=RL, 3=RR with positions in cm. Falls back to
        nominal values when the actor reports out-of-range geometry.
        """
        pc = ego.get_physics_control()
        w = pc.wheels

        def pos(i):
            return np.array([w[i].position.x, w[i].position.y, w[i].position.z])

        L = float(np.linalg.norm(0.5 * (pos(0) + pos(1)) - 0.5 * (pos(2) + pos(3))) * 0.01)
        if not (1.0 < L < 5.0):
            L = 2.85
        max_steer_rad = math.radians(float(w[0].max_steer_angle))
        if not (0.2 < max_steer_rad < 1.5):
            max_steer_rad = 1.22
        return L, max_steer_rad

    # ------------------------------------------------------------- longitudinal
    def _long_step(self, target_kmh: float, v_kmh: float) -> float:
        """Compute the longitudinal command from an inline speed PID.

        Combines proportional, filtered clamped derivative-on-error, a low-passed
        target-rate feed-forward (long_kff), and conditional-integration anti-windup
        terms.

        Parameters
        ----------
        target_kmh : float
            Target speed in km/h.
        v_kmh : float
            Measured speed in km/h.

        Returns
        -------
        float
            Unclamped throttle/brake command; clamped in _to_command.

        Notes
        -----
        Derivative is taken on error, not measurement: the curve-speed policy steps
        the target down before a curve, and the resulting negative de/dt is the brake
        kick that slows the car in time. The low-pass and D_max clamp keep it from
        amplifying single-sample noise.
        """
        e = target_kmh - v_kmh
        de = 0.0 if self._e_prev is None else (e - self._e_prev) / self.dt
        self._e_prev = e
        self._d_f = self.d_alpha * self._d_f + (1.0 - self.d_alpha) * de
        d_term = float(np.clip(self.cfg.long_kd * self._d_f, -self.d_max, self.d_max))
        # Feed-forward the target's rate (low-passed to reject jitter): brake on a dropping target,
        # throttle on a rising one, so the car TRACKS the profile's ramp instead of lagging above it.
        tgt_rate = 0.0 if self._tgt_prev is None else (target_kmh - self._tgt_prev)
        self._tgt_prev = target_kmh
        self._tgt_rate_f = 0.6 * self._tgt_rate_f + 0.4 * tgt_rate
        u_ff = self.cfg.long_kff * self._tgt_rate_f
        u_pd = self.cfg.long_kp * e + d_term + u_ff
        # conditional-integration anti-windup
        i_try = self._i + e * self.dt
        u_try = u_pd + self.cfg.long_ki * i_try
        if abs(u_try) <= 1.0 or (u_try > 0.0) != (e > 0.0):
            self._i = i_try
        return u_pd + self.cfg.long_ki * self._i

    # -------------------------------------------------------------- main step
    def step(self, state: VehicleState) -> ControlCommand:
        if self.traj is None:
            raise RuntimeError("PIDPlusController.step before set_reference.")

        samp = self.traj.sample(state.x, state.y)
        e_cte = self.traj.cross_track(state.x, state.y)     # same projection -> _last_i stays put
        v = state.speed

        if self.fixed_target_speed_kmh > 0.0:
            target_kmh = self.fixed_target_speed_kmh
        else:
            try:
                target_kmh = self._speed_target_kmh(samp, v)
            except Exception as exc:  # noqa: BLE001  fail safe, never crash the loop
                print(f"[pidpp] speed-target error: {exc}")
                target_kmh = self.fallback_speed

        accel = self._long_step(target_kmh, v * 3.6)

        # --- lateral: look-ahead heading PID + Stanley cross-track + curvature FF ---
        steering_wp = _target_point(*self.traj.point_at_arclength(
            samp["s"] + self.cfg.lat_lookahead_sec * v))
        pid_steer = float(self.lat_controller.run_step(steering_wp))           # look-ahead heading feedback
        cross_steer = math.atan(self.k_e * e_cte / (self.k_soft + v)) / self.max_steer_rad
        ff_steer = self.lat_kff * math.atan(self.L * samp["kappa"]) / self.max_steer_rad
        steer = pid_steer + cross_steer + ff_steer

        # High-steer spin-out guard (parity with the baseline).
        if math.fabs(steer) > self.cfg.high_steer_threshold:
            accel = min(accel, self.cfg.high_steer_throttle_cap)

        # Steer-rate limit (the base bypasses CARLA's slew limiter).
        if self.steer_rate_max > 0.0:
            dmax = self.steer_rate_max * self.dt
            steer = float(np.clip(steer, self._prev_steer - dmax, self._prev_steer + dmax))
        self._prev_steer = steer

        cmd = self._to_command(accel, steer)

        loc = steering_wp.transform.location
        self._debug = ControllerDebug(
            target_speed_kmh=target_kmh,
            cross_track_error_m=e_cte,
            heading_error_deg=math.degrees(math.atan2(
                math.sin(samp["psi"] - state.yaw), math.cos(samp["psi"] - state.yaw))),
            target_point_xy=(loc.x, loc.y),
        )
        return cmd
