"""Precompute the PID's route geometry once per trajectory.

Pure path geometry (curvature, heading change ahead, cross-track) is computed
when the route arrives rather than rescanned each control tick. The route is
resampled to an arc-length grid that maps:

  s -> (x, y, psi, kappa)          arc-length spline (CARLA coords)
  s -> angle_ahead, angle_dist     sharpest heading change within a look-ahead
                                   and its distance (precomputed -> O(1) lookup)

Cheap per-tick lookups (forward-local projection, cross-track, point-at-arc-length)
are exposed on top. The class is CARLA-free: it is built purely from (x, y) waypoints.

The forward-local projection (`_nearest_index`) starts at the trimmed route's head
(a few waypoints behind the ego) so progress re-locks correctly and never jumps onto a
freshly-appended segment that loops back near the car. Preserve that property if you
touch the projection. This mirrors the MPC's ReferencePath handling.

This preprocessing is PID-only: the tracking MPC keeps its own reference.
"""
from __future__ import annotations

import numpy as np

from carla_mpc_bringup.control.reference_path import _smooth_xy


class AnnotatedTrajectory:
    def __init__(self, xy, *, angle_lookahead_m: float = 40.0, resample_ds: float = 1.0,
                 smooth_m: float = 2.5):
        pts = np.asarray(xy, dtype=float)
        if pts.shape[0] < 3:
            raise ValueError("AnnotatedTrajectory needs >= 3 waypoints.")

        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s_raw = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s_raw[-1])
        n = max(3, int(total / max(resample_ds, 0.2)) + 1)
        s = np.linspace(0.0, total, n)
        x = np.interp(s, s_raw, pts[:, 0])
        y = np.interp(s, s_raw, pts[:, 1])
        x, y = _smooth_xy(x, y, smooth_m, resample_ds)   # gentle implicit lane changes

        dx, dy = np.gradient(x, s), np.gradient(y, s)
        psi = np.unwrap(np.arctan2(dy, dx))
        kappa = np.gradient(psi, s)
        if len(kappa) >= 5:
            kappa = np.convolve(kappa, np.ones(5) / 5.0, mode="same")

        angle_ahead, angle_dist = self._angle_ahead(s, psi, angle_lookahead_m)

        self.s, self.x, self.y, self.psi, self.kappa = s, x, y, psi, kappa
        self.angle_ahead, self.angle_dist = angle_ahead, angle_dist
        self.total = total
        self._last_i = 0

    @staticmethod
    def _angle_ahead(s, psi, lookahead_m):
        """Compute max heading change within a look-ahead and its distance, per sample.

        Parameters
        ----------
        s : np.ndarray
            Arc-length grid (m), monotonically increasing.
        psi : np.ndarray
            Unwrapped heading at each sample (rad).
        lookahead_m : float
            Forward window over which to search for the sharpest heading change (m).

        Returns
        -------
        angle : np.ndarray
            Max absolute heading change within the look-ahead, per sample (degrees).
        dist : np.ndarray
            Arc-length distance from each sample to where that max occurs (m).
        """
        n = len(s)
        angle = np.zeros(n)
        dist = np.zeros(n)
        j = 0
        for i in range(n):
            if j < i:
                j = i
            while j + 1 < n and (s[j + 1] - s[i]) <= lookahead_m:
                j += 1
            if j > i:
                dpsi = np.abs(psi[i + 1:j + 1] - psi[i])
                k = int(np.argmax(dpsi))
                angle[i] = float(np.degrees(dpsi[k]))
                dist[i] = float(s[i + 1 + k] - s[i])
        return angle, dist

    def _nearest_index(self, x: float, y: float) -> int:
        """Find the nearest path index within a tight forward-local window.

        The bounded window keeps progress monotonic and robust to a freshly-appended
        segment looping back near the car.
        """
        lo = max(0, self._last_i - 3)
        hi = min(len(self.s), self._last_i + 60)
        d2 = (self.x[lo:hi] - x) ** 2 + (self.y[lo:hi] - y) ** 2
        self._last_i = lo + int(np.argmin(d2))
        return self._last_i

    def sample(self, x: float, y: float) -> dict:
        """Return the precomputed annotation at the ego's nearest path point."""
        i = self._nearest_index(x, y)
        return dict(i=i, s=float(self.s[i]), psi=float(self.psi[i]),
                    kappa=float(self.kappa[i]), angle_ahead=float(self.angle_ahead[i]),
                    angle_dist=float(self.angle_dist[i]))

    def point_at_arclength(self, s_query: float) -> tuple[float, float]:
        sq = float(np.clip(s_query, 0.0, self.total))
        return float(np.interp(sq, self.s, self.x)), float(np.interp(sq, self.s, self.y))

    def cross_track(self, x: float, y: float) -> float:
        """Compute the signed lateral distance to the nearest path segment.

        Returns
        -------
        float
            Lateral offset in metres. In CARLA's Y-right frame it is positive when the
            car is left of the path, negative to the right -- the opposite of the
            screen/math convention. Mind the sign when feeding it back for steering.
        """
        i = self._nearest_index(x, y)
        lo, hi = max(0, i - 2), min(len(self.s) - 1, i + 2)
        p = np.array([x, y])
        best, best_sign = float("inf"), 0.0
        for k in range(lo, hi):
            a = np.array([self.x[k], self.y[k]])
            b = np.array([self.x[k + 1], self.y[k + 1]])
            ab = b - a
            denom = float(np.dot(ab, ab))
            if denom < 1e-9:
                continue
            t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
            foot = a + t * ab
            d = float(np.linalg.norm(p - foot))
            if d < best:
                best = d
                best_sign = 1.0 if (ab[0] * (p[1] - foot[1]) - ab[1] * (p[0] - foot[0])) < 0 else -1.0
        return 0.0 if best == float("inf") else best * best_sign

    def progress(self, x: float, y: float) -> float:
        i = self._nearest_index(x, y)
        return float(self.s[i] / self.total) if self.total > 0 else 1.0

    def speed_profile(self, cruise_kmh: float, a_lat_max: float, factor: float = 0.9,
                      a_dec: float = 3.0, a_acc: float = 3.0, min_kmh: float = 10.0) -> np.ndarray:
        """Compute a per-sample target speed from a forward-backward velocity profile.

        Uses the standard curvature-based method over the precomputed path:
          1. pointwise curvature limit  v = factor * sqrt(a_lat_max / |kappa|), capped at cruise;
          2. backward pass  v[i] <= sqrt(v[i+1]^2 + 2*a_dec*ds)  -> can brake to each curve in time;
          3. forward pass   v[i] <= sqrt(v[i-1]^2 + 2*a_acc*ds)  -> bounded acceleration out of it.
        Unlike an angle-ahead look-ahead heuristic, this stays correct inside curves, where a
        forward window wrongly sees less curve remaining and speeds back up. O(n) over the path.

        Returns
        -------
        np.ndarray
            Target speed per sample (km/h).

        References
        ----------
        MathWorks Velocity Profiler; UofT Self-Driving-Cars course L5.
        """
        cruise = cruise_kmh / 3.6
        k = np.abs(self.kappa)
        # Floor at min_kmh so a single curvature spike (route kink at a junction) can't
        # drive the target to ~0 and stall the car.
        v = np.clip(factor * np.sqrt(a_lat_max / np.maximum(k, 1e-4)), min_kmh / 3.6, cruise)
        ds = np.diff(self.s)
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], float(np.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * ds[i])))
        for i in range(1, len(v)):
            v[i] = min(v[i], float(np.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds[i - 1])))
        return v * 3.6
