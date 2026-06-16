"""Controller seam shared by every controller (PID, MPC, MPCC).

``control_node`` owns the CARLA vehicle and the world tick; it asks the active
controller for a ``ControlCommand`` each step and applies it. Keeping the
interface tiny and the controller free of ROS lets a single launch flag swap PID
for MPC and lets controllers be unit-tested offline.

State is passed in as a ``VehicleState`` (a small, CARLA-free snapshot) so that
controllers do not depend on the live actor. The one exception is the reference
PID, which wraps CARLA's built-in ``agents.navigation`` PID and needs the actor,
so it takes it at construction. MPC controllers use ``VehicleState`` only.

Exports
-------
ControlCommand : Actuator command (throttle/brake/steer).
VehicleState : CARLA-free ego snapshot.
ControllerDebug : Optional telemetry republished for Foxglove.
ControllerInterface : Abstract base every controller implements.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class ControlCommand:
    """Actuator command for CARLA: throttle/brake in [0, 1], steer in [-1, 1]."""
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0

    def clamp(self) -> "ControlCommand":
        self.throttle = float(min(1.0, max(0.0, self.throttle)))
        self.brake = float(min(1.0, max(0.0, self.brake)))
        self.steer = float(min(1.0, max(-1.0, self.steer)))
        return self


@dataclass
class VehicleState:
    """CARLA-free ego snapshot in the CARLA world frame (left-handed, m/rad).

    ``control_node`` builds this from the live actor each tick. x/y/yaw are the
    planar pose; vx/vy/vz are world-frame velocity components; ``speed`` is |v|.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0          # heading in rad; CARLA yaw is left-handed
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    t: float = 0.0            # sim time in s

    @property
    def speed(self) -> float:
        return math.sqrt(self.vx * self.vx + self.vy * self.vy + self.vz * self.vz)


@dataclass
class ControllerDebug:
    """Optional telemetry the control_node republishes for Foxglove.

    Any field left None is not published. PID exposes its speed policy and
    errors; MPC adds the predicted horizon, solve time and cost.
    """
    target_speed_kmh: float | None = None
    cross_track_error_m: float | None = None
    heading_error_deg: float | None = None
    # Reference point the controller is aiming at, in CARLA world (x, y).
    target_point_xy: tuple[float, float] | None = None
    # MPC predicted ego path as CARLA world (x, y) samples.
    predicted_path_xy: list[tuple[float, float]] = field(default_factory=list)
    solve_time_ms: float | None = None
    cost: float | None = None


class ControllerInterface(ABC):
    """Minimal controller contract used by control_node."""

    name: str = "controller"

    @abstractmethod
    def set_reference(self, route) -> None:
        """Give the controller the route to track.

        Parameters
        ----------
        route : list of (carla.Waypoint, RoadOption)
            Output of CARLA's GlobalRoutePlanner. Controllers may use the
            waypoint objects (PID lateral target) or just their positions
            (MPC spline fit).
        """

    @abstractmethod
    def step(self, state: VehicleState) -> ControlCommand:
        """Compute the actuator command for the current state."""

    def get_debug(self) -> ControllerDebug:
        """Return the latest telemetry (default: empty)."""
        return ControllerDebug()

    def trajectory_heatmap(self):
        """Return the precomputed per-trajectory heatmap annotation, or None.

        The annotation is a list of (x, y, speed_kmh, secondary) in CARLA
        coords, where ``secondary`` is the curve angle (PID) or path curvature
        (MPC). Rebuilt on each ``set_reference``; defaults to None (no heatmap).
        """
        return None
