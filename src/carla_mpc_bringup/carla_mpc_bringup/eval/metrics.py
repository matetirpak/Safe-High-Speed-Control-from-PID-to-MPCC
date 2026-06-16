"""Reduce a per-step run recording into a structured metrics dict.

Pure numpy over the recorded series -- no ROS, CARLA, or I/O -- so the same code
serves the live ``RunLogger`` (one run) and a benchmark aggregator (many runs), and
stays trivially unit-testable. Derived signals (longitudinal acceleration,
jerk, steering smoothness) are computed here from the raw series so the logger only
records what is directly observed. Lateral acceleration prefers CARLA's measured
body-frame signal when recorded and otherwise falls back to speed * yaw-rate.

Metric groups: tracking (cross-track, heading), speed, comfort/safety (acceleration,
jerk, steering smoothness), solver (MPC solve time), events (lane crossings), and a
trip summary (distance, duration, average speed). All values are reported in metric
units (km/h, m, m/s^2, g).
"""
from __future__ import annotations

import math

import numpy as np

G = 9.80665


def _arr(series, key):
    return np.asarray(series.get(key, []), dtype=float)


def _stats(v):
    """Return mean/rms/max/p95/min over finite values (empty input -> zeros)."""
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"mean": 0.0, "rms": 0.0, "max": 0.0, "p95": 0.0, "min": 0.0}
    return {
        "mean": float(np.mean(v)),
        "rms": float(np.sqrt(np.mean(v ** 2))),
        "max": float(np.max(np.abs(v))),
        "p95": float(np.percentile(np.abs(v), 95)),
        "min": float(np.min(v)),
    }


def _deriv(v, t):
    """Differentiate ``v`` with respect to ``t``, returning a same-length, finite-safe array."""
    v, t = np.asarray(v, float), np.asarray(t, float)
    if v.size < 2:
        return np.zeros_like(v)
    # Duplicate timestamps give dt == 0; mark them NaN so the division stays finite.
    tt = t.copy()
    dt = np.diff(tt)
    dt[dt <= 0] = np.nan
    out = np.full_like(v, np.nan)
    out[1:-1] = (v[2:] - v[:-2]) / (tt[2:] - tt[:-2])
    out[0] = (v[1] - v[0]) / dt[0] if np.isfinite(dt[0]) else 0.0
    out[-1] = (v[-1] - v[-2]) / dt[-1] if np.isfinite(dt[-1]) else 0.0
    return np.nan_to_num(out)


def derived_series(series: dict) -> dict:
    """Derive time series from the raw recording, shared by metrics and plots.

    Returns
    -------
    dict
        Same-length arrays keyed ``long_acc`` and ``lat_acc`` (m/s^2),
        ``yaw_rate`` (rad/s), ``lat_jerk`` (m/s^3), and ``steer_rate``
        (steer units per second).
    """
    t = _arr(series, "t")
    speed_ms = _arr(series, "speed_kmh") / 3.6
    yaw = _arr(series, "yaw")
    steer = _arr(series, "steer")
    long_acc = _deriv(speed_ms, t)
    yaw_rate = _deriv(np.unwrap(yaw) if yaw.size == t.size else yaw, t)
    # Prefer CARLA's measured lateral accel (body frame) so jerk is a single derivative of a
    # direct signal. The fallback speed * d/dt(yaw) makes jerk a double derivative of yaw --
    # noise-amplified and speed-biased. Logs without the column fall back automatically.
    lat_meas = _arr(series, "lat_accel_ms2")
    if lat_meas.size == t.size and np.isfinite(lat_meas).any():
        lat_acc = lat_meas
    else:
        lat_acc = speed_ms * yaw_rate
    return {
        "long_acc": long_acc,
        "yaw_rate": yaw_rate,
        "lat_acc": lat_acc,
        "lat_jerk": _deriv(lat_acc, t),
        "steer_rate": _deriv(steer, t),
    }


def compute_metrics(series: dict, scenario: dict | None = None) -> dict:
    """Compute the full metrics dict for one run.

    Parameters
    ----------
    series : dict
        Raw per-step recording; keys are signal names mapping to equal-length
        sequences (``t`` in seconds, ``speed_kmh``, ``x``/``y`` in metres, etc.).
    scenario : dict, optional
        Run metadata (controller, map, seed, ...) passed through verbatim; may
        carry a precomputed ``lane_invasions`` count.

    Returns
    -------
    dict
        Nested metrics grouped under ``trip``, ``tracking``, ``speed``,
        ``comfort``, ``solver``, and ``events`` plus ``lane_departures``. Runs
        with fewer than two samples return a short stub dict instead.
    """
    t = _arr(series, "t")
    n = t.size
    scenario = dict(scenario or {})
    if n < 2:
        return {"scenario": scenario, "samples": int(n), "note": "too few samples"}

    speed_ms = _arr(series, "speed_kmh") / 3.6
    target_kmh = _arr(series, "target_speed_kmh")
    x, y = _arr(series, "x"), _arr(series, "y")
    steer = _arr(series, "steer")
    cte = _arr(series, "cross_track_m")
    head = _arr(series, "heading_error_deg")
    solve = _arr(series, "solve_time_ms")

    duration = float(t[-1] - t[0])
    seg = np.hypot(np.diff(x), np.diff(y)) if x.size == n and x.size > 1 else np.array([0.0])
    distance = float(np.sum(seg))

    d = derived_series(series)
    long_acc, lat_acc, lat_jerk = d["long_acc"], d["lat_acc"], d["lat_jerk"]

    # Steering smoothness: reversals of the steering-rate sign (weaving).
    sgn = np.sign(d["steer_rate"])
    reversals = int(np.sum((sgn[1:] != sgn[:-1]) & (sgn[1:] != 0))) if sgn.size > 1 else 0

    speed_track_err = (speed_ms - target_kmh / 3.6) if target_kmh.size == n else np.array([0.0])
    solve_v = solve[np.isfinite(solve) & (solve > 0)]
    lane = scenario.get("lane_invasions")
    if lane is None:
        lt = _arr(series, "lane_invasions_total")
        lt = lt[np.isfinite(lt)]
        lane = int(lt.max()) if lt.size else None

    # The marking-crossing count misses a car that bulges off its lane mid-curve without crossing
    # a marking (a corner-cut). Count rising edges where |cross-track| exceeds a clear off-lane
    # threshold to capture excursions the event count cannot.
    cte_abs = np.abs(_arr(series, "cross_track_m"))
    above = cte_abs > 1.5
    departures = int(np.sum(above[1:] & ~above[:-1])) if above.size > 1 else 0

    # Average speed while actually driving: exclude the start-up ramp, the route-end run-down, and
    # any mid-route stop/crawl. Take the window from the first to the last sample at >= 50% of this
    # run's peak speed (trims both ramps), then average its moving (> 5 km/h) samples. Avoids the
    # standstill/ramp bias of a plain distance/duration average.
    sp_kmh = speed_ms * 3.6
    if n and float(sp_kmh.max()) > 1.0:
        at = np.where(sp_kmh >= 0.5 * sp_kmh.max())[0]
        if at.size:
            win = sp_kmh[at[0]:at[-1] + 1]
            drv = win[win > 5.0]
            avg_drive = float(drv.mean()) if drv.size else float(win.mean())
        else:
            mov = sp_kmh[sp_kmh > 5.0]
            avg_drive = float(mov.mean()) if mov.size else 0.0
    else:
        avg_drive = 0.0

    return {
        "scenario": scenario,
        "samples": int(n),
        "lane_departures": departures,   # off-lane (|cte| > 1.5 m) excursions, catches corner-cuts
        "trip": {
            "duration_s": duration,
            "distance_m": distance,
            "avg_speed_kmh": avg_drive,   # average speed while driving (start/end ramps excluded)
            "max_speed_kmh": float(np.max(speed_ms) * 3.6),
        },
        "tracking": {
            "cross_track_m": _stats(cte),
            "heading_error_deg": _stats(head),
        },
        "speed": {
            "speed_kmh": _stats(speed_ms * 3.6),
            "speed_tracking_err_kmh": _stats(speed_track_err * 3.6),
        },
        "comfort": {
            "lat_accel_ms2": _stats(lat_acc),
            "lat_accel_g": {k: v / G for k, v in _stats(lat_acc).items()},
            "long_accel_ms2": _stats(long_acc),
            "lat_jerk_ms3": _stats(lat_jerk),
            "steer": _stats(steer),
            "steer_reversals_per_s": float(reversals / duration) if duration > 0 else 0.0,
        },
        "solver": {
            "solve_ms_mean": float(np.mean(solve_v)) if solve_v.size else None,
            "solve_ms_p50": float(np.percentile(solve_v, 50)) if solve_v.size else None,
            "solve_ms_p99": float(np.percentile(solve_v, 99)) if solve_v.size else None,
            "solve_ms_max": float(np.max(solve_v)) if solve_v.size else None,
        },
        "events": {
            "lane_invasions": int(lane) if lane is not None else None,
        },
    }


def summary_text(m: dict) -> str:
    """Format a compute_metrics() dict as a compact, human-readable one-pager."""
    if m.get("samples", 0) < 2:
        return "run too short for metrics.\n"
    s = m["scenario"]
    L = []
    L.append(f"RUN  {s.get('controller','?')}"
             + (f" ({s.get('model')})" if s.get("model") else "")
             + f"  map={s.get('map','?')}  seed={s.get('seed','?')}")
    if s.get("condition"):
        L.append(f"  condition={s['condition']}")
    tr, tk, sp, c, sv, ev = (m["trip"], m["tracking"], m["speed"],
                             m["comfort"], m["solver"], m["events"])
    L.append(f"trip      : {tr['distance_m']:.0f} m in {tr['duration_s']:.0f} s "
             f"| avg {tr['avg_speed_kmh']:.1f} km/h (driving), "
             f"max {tr['max_speed_kmh']:.1f} km/h")
    L.append(f"tracking  : cross-track rms {tk['cross_track_m']['rms']:.3f} m, "
             f"max {tk['cross_track_m']['max']:.2f} m | heading rms "
             f"{tk['heading_error_deg']['rms']:.2f} deg")
    L.append(f"speed     : mean {sp['speed_kmh']['mean']:.1f} km/h, "
             f"track-err rms {sp['speed_tracking_err_kmh']['rms']:.1f} km/h")
    L.append(f"comfort   : lat-accel rms {c['lat_accel_ms2']['rms']:.2f} m/s^2 "
             f"({c['lat_accel_g']['rms']:.2f} g), max {c['lat_accel_ms2']['max']:.2f} m/s^2 "
             f"({c['lat_accel_g']['max']:.2f} g)")
    L.append(f"            long-accel rms {c['long_accel_ms2']['rms']:.2f} m/s^2 | "
             f"lat-jerk rms {c['lat_jerk_ms3']['rms']:.2f} m/s^3 | "
             f"steer reversals {c['steer_reversals_per_s']:.1f}/s")
    if sv["solve_ms_mean"] is not None:
        L.append(f"solver    : solve {sv['solve_ms_mean']:.1f} ms mean, "
                 f"{sv['solve_ms_p99']:.1f} ms p99, {sv['solve_ms_max']:.1f} ms max")
    if ev["lane_invasions"] is not None:
        L.append(f"events    : {ev['lane_invasions']} lane crossing(s)")
    return "\n".join(L) + "\n"
