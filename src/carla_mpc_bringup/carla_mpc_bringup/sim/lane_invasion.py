"""Turn the CARLA lane-invasion sensor into a blinking indicator state.

CARLA's ``sensor.other.lane_invasion`` is an event sensor with no native ROS2
publisher in 0.9.16, so a Python listener converts each line crossing into a
short blinking window plus the crossed marking type(s). ``control_node`` polls
``state()`` each tick and publishes a Bool (the blink) and a flashing 3D marker;
the Foxglove layout binds an Indicator panel to the Bool.

The blink logic is pure and testable via ``feed()``; only ``attach()`` touches
CARLA.

Exports
-------
LaneInvasionMonitor : Track recent crossings and emit a blink state.
"""
from __future__ import annotations

import threading


class LaneInvasionMonitor:
    """Track recent lane crossings and expose a blinking indicator state."""

    def __init__(self, hold_s: float = 2.0, blink_hz: float = 4.0):
        self.hold_s = hold_s            # blink window length after a crossing, seconds
        self.blink_hz = blink_hz        # full on/off cycles per second
        self._t = -1.0e9                # sim time of the last crossing; far past so we start inactive
        self._types: list[str] = []
        self._count = 0                 # monotonic event counter for caller-side edge detection
        self._lock = threading.Lock()

    def attach(self, sensor_actor) -> None:
        """Subscribe to the lane_invasion sensor.

        The callback runs in CARLA's sensor thread, hence the lock in ``feed``.
        """
        sensor_actor.listen(self._on_event)

    def _on_event(self, event) -> None:
        types = sorted({str(m.type).split(".")[-1] for m in event.crossed_lane_markings})
        self.feed(float(event.timestamp), types or ["Unknown"])

    def feed(self, t_sec: float, types: list[str]) -> None:
        """Record a crossing at sim time ``t_sec`` with marking ``types``.

        Pure logic; callable without CARLA for unit tests.
        """
        with self._lock:
            self._t = float(t_sec)
            self._types = list(types)
            self._count += 1

    def state(self, now: float):
        """Compute the indicator state at sim time ``now``.

        Parameters
        ----------
        now : float
            Current sim time, seconds.

        Returns
        -------
        tuple
            ``(active, blink_on, label, count)`` where ``active`` is True within
            ``hold_s`` of the last crossing, ``blink_on`` is a square wave at
            ``blink_hz`` while active, ``label`` is the crossed marking type(s)
            (e.g. "Solid" or "Broken, Solid"), and ``count`` is the monotonic
            event id for caller-side edge detection.
        """
        with self._lock:
            t, types, count = self._t, list(self._types), self._count
        dt = now - t
        active = 0.0 <= dt < self.hold_s
        blink_on = active and (int(dt * self.blink_hz * 2.0) % 2 == 0)
        return active, blink_on, ", ".join(types), count
