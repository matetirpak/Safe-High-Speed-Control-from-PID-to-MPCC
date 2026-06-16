"""Map MPC physical outputs to CARLA actuation commands.

Convert acceleration ``a`` [m/s^2] and steer angle ``delta`` [rad] to CARLA
``(throttle, brake, steer)`` in [0,1]/[0,1]/[-1,1]. The mapping is not the
identity: throttle/brake authority is nonlinear and speed-dependent.

The analytic map is derived from the vehicle's max steer angle and coarse
sustained accel limits. Steering is linear, as CARLA's response is well
approximated linearly across the range. Longitudinal command is piecewise on
the sign of ``a`` with a deadband plus hysteresis so the controller does not
chatter between throttle and brake.

Sign convention (CARLA, left-handed, Y right): positive ``delta`` and positive
``steer`` both mean a right turn, so ``steer = delta / max_steer`` directly.
This matches the bicycle models run in CARLA coordinates (see mpc_controller.py and mpcc_controller.py).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

DEADBAND = 0.2     # m/s^2; within |a| this band, neither throttle nor brake
EPS_HYST = 0.1     # m/s^2; hysteresis margin around the deadband


@dataclass
class ActuationLimits:
    """Vehicle actuation limits used to scale physical commands."""

    max_steer_rad: float       # physical front-wheel max steer
    a_max_throttle: float      # m/s^2, sustained accel at low speed
    a_max_brake: float         # m/s^2
    v_max: float               # m/s; sets throttle de-rating at speed


class ActuationMap:
    """Stateful map from MPC commands to CARLA actuation, with brake hysteresis."""

    def __init__(self, limits: ActuationLimits):
        self._lim = limits
        self._last_was_brake = False

    @classmethod
    def from_physics(cls, max_steer_rad: float, mpc_cfg) -> "ActuationMap":
        """Build an ActuationMap from a steer angle and the MPC config limits."""
        return cls(ActuationLimits(
            max_steer_rad=max_steer_rad,
            a_max_throttle=mpc_cfg.a_max_throttle,
            a_max_brake=mpc_cfg.a_max_brake,
            v_max=mpc_cfg.v_max,
        ))

    def apply(self, a_des: float, delta_des: float, v_x: float) -> tuple[float, float, float]:
        """Map a desired accel and steer angle to CARLA actuation.

        Parameters
        ----------
        a_des : float
            Desired longitudinal acceleration, m/s^2 (positive forward).
        delta_des : float
            Desired front-wheel steer angle, rad (positive is a right turn).
        v_x : float
            Current longitudinal speed, m/s; used to de-rate throttle.

        Returns
        -------
        tuple of float
            ``(throttle, brake, steer)`` with throttle, brake in [0, 1] and
            steer in [-1, 1] (positive is a right turn).
        """
        steer = float(np.clip(delta_des / max(self._lim.max_steer_rad, 1e-3), -1.0, 1.0))

        # Hysteresis avoids flickering between throttle and brake near a~0.
        if self._last_was_brake:
            up, down = a_des > DEADBAND + EPS_HYST, a_des < -DEADBAND
        else:
            up, down = a_des > DEADBAND, a_des < -DEADBAND - EPS_HYST

        if up:
            throttle, brake = self._a_to_throttle(a_des, v_x), 0.0
            self._last_was_brake = False
        elif down:
            throttle, brake = 0.0, self._a_to_brake(-a_des)
            self._last_was_brake = True
        else:
            throttle, brake = 0.0, 0.0
        return throttle, brake, steer

    def _a_to_throttle(self, a_pos: float, v_x: float) -> float:
        # Throttle authority falls off with speed (drag + powertrain); coarse derate
        # floored at 0.3 so some authority remains near v_max.
        derate = max(0.3, 1.0 - 0.5 * v_x / max(self._lim.v_max, 1e-3))
        a_max = self._lim.a_max_throttle * derate
        return float(np.clip(a_pos / max(a_max, 1e-3), 0.0, 1.0))

    def _a_to_brake(self, a_neg: float) -> float:
        return float(np.clip(a_neg / max(self._lim.a_max_brake, 1e-3), 0.0, 1.0))
