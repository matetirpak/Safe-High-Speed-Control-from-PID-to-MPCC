"""Reference path and speed profile for the tracking MPC.

Take the controller's route (CARLA-world (x, y) waypoints) and build an
arc-length-parametrised reference: s -> (x, y, psi, kappa, v_ref). Everything is
in CARLA coordinates; the OCP runs there and conversion to ROS/gt_map happens at
publish time in control_node.

The speed profile is friction-limited, ``v = sqrt(a_lat_max / |kappa|)`` capped
at the cruise speed, then smoothed by a backward pass (brake before curves,
``a_long_dec``) and a forward pass (accelerate out, ``a_long_acc``) -- the
standard two-pass speed-profile generation.

Each control step, ``window()`` projects the current position onto the path and
returns the next (N+1) reference states the OCP tracks, marched forward in time
by the local ``v_ref``.
"""
from __future__ import annotations

import numpy as np


def _smooth_xy(x, y, smooth_m: float, ds: float):
    """Apply edge-preserving Gaussian smoothing to the path xy.

    GlobalRoutePlanner emits near-instant lateral jumps for implicit lane changes
    (and small steps at waypoint joints). Spreading them over ~smooth_m turns a
    step -- which otherwise forces a hard brake (curvature spike -> low speed
    profile) and a sharp steer -- into a gentle transition. mode='nearest' keeps
    the endpoints put; smooth_m <= 0 disables. A modest value rounds tight corners
    only slightly; validate tracking if you raise it.
    """
    if smooth_m <= 0:
        return x, y
    try:
        from scipy.ndimage import gaussian_filter1d
        sig = smooth_m / max(ds, 1e-3)
        return (gaussian_filter1d(x, sig, mode="nearest"),
                gaussian_filter1d(y, sig, mode="nearest"))
    except Exception:  # noqa: BLE001  smoothing is a nicety, never block a run
        return x, y


class ReferencePath:
    def __init__(self, xy: list[tuple[float, float]], mpc_cfg,
                 fixed_target_kmh: float = 0.0, resample_ds: float = 1.0,
                 road_options=None):
        pts = np.asarray(xy, dtype=float)
        if pts.shape[0] < 3:
            raise ValueError("ReferencePath needs >= 3 waypoints.")

        # Arc length of the raw polyline, then resample to ~uniform ds for stable
        # finite-difference heading/curvature.
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s_raw = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s_raw[-1])
        n = max(3, int(total / max(resample_ds, 0.2)) + 1)
        s = np.linspace(0.0, total, n)
        x = np.interp(s, s_raw, pts[:, 0])
        y = np.interp(s, s_raw, pts[:, 1])
        x, y = _smooth_xy(x, y, getattr(mpc_cfg, "reference_smooth_m", 2.5), resample_ds)

        # Lane changes (RoadOption CHANGELANE*) are abrupt lateral steps in the route.
        # Smooth those arcs harder than normal curves so the car eases through a gentle S
        # (carries speed, soft steer) instead of braking and jerking. Targeted and
        # soft-blended so real corners keep their shape.
        lc_mask = self._lane_change_mask(road_options, s_raw, s)
        if lc_mask.any():
            sx, sy = _smooth_xy(x, y, getattr(mpc_cfg, "lane_change_smooth_m", 6.0), resample_ds)
            kb = max(1, int(4.0 / max(resample_ds, 0.2)))
            w = np.convolve(lc_mask.astype(float), np.ones(2 * kb + 1) / (2 * kb + 1), mode="same")
            w = np.clip(w, 0.0, 1.0)
            x, y = (1.0 - w) * x + w * sx, (1.0 - w) * y + w * sy

        # Heading from path tangent; unwrapped so interpolation is continuous
        # (the OCP cost wraps the heading error, so unwrapped refs are fine).
        dx = np.gradient(x, s)
        dy = np.gradient(y, s)
        psi = np.unwrap(np.arctan2(dy, dx))

        # Curvature kappa = dpsi/ds, lightly smoothed so curvature steps at route
        # waypoint joints don't cause a lateral-accel spike (the speed profile then
        # eases off slightly before a sharp joint instead of hitting it at full speed).
        kappa = np.gradient(psi, s)
        if len(kappa) >= 5:
            w = np.ones(5) / 5.0
            kappa = np.convolve(kappa, w, mode="same")

        # ---- Speed profile ---------------------------------------------------
        v_cap = (fixed_target_kmh if fixed_target_kmh > 0 else mpc_cfg.target_speed_kmh) / 3.6
        v_cap = min(v_cap, mpc_cfg.v_max)
        if fixed_target_kmh > 0:
            v = np.full_like(s, v_cap)
        else:
            # Safety margin: target a fraction of the grip-limited curve speed, so a
            # curvature under-estimate, model mismatch, or being near the lane edge
            # doesn't run the car wide into roadside walls. Cornering at the limit can
            # run ~1.8 m wide and clip a wall. A Pacejka friction circle is the
            # principled fix; this is the margin until then.
            factor = getattr(mpc_cfg, "curve_speed_factor", 0.85)
            kabs = np.maximum(np.abs(kappa), 1e-4)
            v_curv = factor * np.sqrt(mpc_cfg.a_lat_max / kabs)
            v = np.clip(v_curv, mpc_cfg.v_min, v_cap)
            v = self._smooth_backward(v, s, mpc_cfg.a_long_dec)
            v = self._smooth_forward(v, s, mpc_cfg.a_long_acc)

        self.s, self.x, self.y, self.psi, self.kappa, self.v = s, x, y, psi, kappa, v
        self.total = total
        self._last_i = 0
        self._a_acc = mpc_cfg.a_long_acc      # accel bound for the reachable-speed ramp in window()
        self.cruise = v_cap                   # cruise ceiling; MPCC reads it as the cold-start seed cap

    @staticmethod
    def _lane_change_mask(road_options, s_raw, s, margin: float = 6.0):
        """Mark resampled-grid points within ``margin`` of a CHANGELANE* waypoint.

        ``road_options`` is aligned with the raw xy (so with ``s_raw``). The mask is
        empty if road options or the CARLA navigation import are unavailable.

        Returns
        -------
        numpy.ndarray
            Boolean array over the resampled grid ``s``.
        """
        mask = np.zeros(len(s), dtype=bool)
        if not road_options:
            return mask
        try:
            from agents.navigation.local_planner import RoadOption
            change = {RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT}
        except Exception:  # noqa: BLE001
            return mask
        for i, o in enumerate(road_options):
            if o in change and i < len(s_raw):
                mask |= (s >= s_raw[i] - margin) & (s <= s_raw[i] + margin)
        return mask

    @staticmethod
    def _smooth_backward(v, s, a_dec):
        """Limit deceleration into curves: pass from the end backward."""
        out = v.copy()
        for i in range(len(v) - 2, -1, -1):
            ds = s[i + 1] - s[i]
            out[i] = min(out[i], np.sqrt(out[i + 1] ** 2 + 2.0 * a_dec * ds))
        return out

    @staticmethod
    def _smooth_forward(v, s, a_acc):
        """Limit acceleration out of curves: pass from the start forward."""
        out = v.copy()
        for i in range(1, len(v)):
            ds = s[i] - s[i - 1]
            out[i] = min(out[i], np.sqrt(out[i - 1] ** 2 + 2.0 * a_acc * ds))
        return out

    # ----------------------------------------------------------------- queries
    def _nearest_index(self, x: float, y: float) -> int:
        """Find the path index closest to (x, y) within a tight forward window.

        Searching only near the last hit keeps progress advancing locally. A wide
        window lets the projection jump to a spatially-near but route-distant point
        (e.g. a freshly-appended segment that loops back near the car), which skips
        the leftover route and jitters the reference. At ~1 m resampling, 60 samples
        gives 60 m of forward reach, far beyond the per-tick motion.
        """
        lo = max(0, self._last_i - 3)
        hi = min(len(self.s), self._last_i + 60)
        d2 = (self.x[lo:hi] - x) ** 2 + (self.y[lo:hi] - y) ** 2
        i = lo + int(np.argmin(d2))
        self._last_i = i
        return i

    def at(self, s_query: float):
        s = float(np.clip(s_query, 0.0, self.total))
        return (float(np.interp(s, self.s, self.x)),
                float(np.interp(s, self.s, self.y)),
                float(np.interp(s, self.s, self.psi)),
                float(np.interp(s, self.s, self.v)))

    def window(self, x: float, y: float, v0: float, N: int, dt: float) -> np.ndarray:
        """Build the (N+1, 4) reference [X, Y, psi, v] over the horizon.

        The speed reference is the curvature-limited, two-pass-smoothed profile
        (sqrt(a_lat_max/|kappa|), capped at cruise), ramped from the current speed so
        it stays reachable. The along-track cost weight is low, so the MPC still trades
        speed against path within the grip budget rather than being rigidly paced.

        Reference positions are marched along the path at a reachable pace -- ramped
        from the current speed ``v0`` and bounded by the curvature-limited profile --
        so the longitudinal target stays attainable and the OCP stays well-
        conditioned; an unreachable position target corrupts the lateral solution.
        This pace is pure path geometry under the same a_lat limit, not a commanded
        speed.

        Parameters
        ----------
        x, y : float
            Current position in CARLA coordinates (m).
        v0 : float
            Current speed (m/s), the start of the reachable-speed ramp.
        N : int
            Horizon length; the window has N+1 stages.
        dt : float
            Stage time step (s).

        Returns
        -------
        numpy.ndarray
            Shape (N+1, 4) array of [X, Y, psi, v] references; positions in m,
            psi in rad, v in m/s.
        """
        i0 = self._nearest_index(x, y)
        ref = np.zeros((N + 1, 4))
        s_cur = float(self.s[i0])
        for k in range(N + 1):
            X, Y, PSI, V_path = self.at(s_cur)
            # Reachable speed: ramp from current speed toward the friction-limited
            # profile (V_path = sqrt(a_lat_max/kappa), capped at cruise). This bounds
            # the planned lateral accel by construction (no curve-entry spikes), with
            # a_lat_max as the single "how hard to corner" knob. The cost tracks this
            # speed but with the along-track weight low (contouring), so the MPC still
            # trades speed vs. path within the grip budget rather than being rigidly paced.
            v_ref = min(V_path, max(v0, 0.5) + self._a_acc * (k * dt))
            ref[k] = (X, Y, PSI, v_ref)
            s_cur += max(v_ref, 0.5) * dt
        return ref

    def progress(self, x: float, y: float) -> float:
        """Fraction [0,1] of the path completed at (x, y)."""
        i = self._nearest_index(x, y)
        return float(self.s[i] / self.total) if self.total > 0 else 1.0
