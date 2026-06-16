"""Aggregate per-run logs into controller-vs-controller benchmark tables.

Pure analysis: no ROS/CARLA/I-O beyond reading the run dirs. Consumes the per-run
``logs/runs/<run>/`` (metrics.json + series.csv) written by RunLogger and emits a
Markdown comparison across controllers.

Raw average speed is biased by which road each run drove, so speed is also binned by
road curvature estimated from the driven path -- comparing speed at the same kind of
segment. Runs that share a seed/spawn/map drove the same road, making it replicable.
See docs/evaluation.md.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

# Curvature bins (1/m) and their radius-band labels: straight / gentle / moderate / tight.
CURV_EDGES = [0.0, 0.004, 0.012, 0.03, 1e9]
CURV_LABELS = ["straight (R>250m)", "gentle (R 80-250m)", "moderate (R 33-80m)", "tight (R<33m)"]

# Metrics surfaced in the comparison table: (json path, label, lower_is_better).
TABLE_METRICS = [
    (("trip", "avg_speed_kmh"), "avg_speed_kmh", False),
    (("trip", "distance_m"), "distance_m", None),
    (("tracking", "cross_track_m", "rms"), "cte_rms_m", True),
    (("tracking", "cross_track_m", "max"), "cte_max_m", True),
    (("tracking", "heading_error_deg", "rms"), "heading_rms_deg", True),
    (("comfort", "lat_accel_ms2", "max"), "lat_accel_max", True),
    (("comfort", "lat_jerk_ms3", "rms"), "lat_jerk_rms", True),
    (("comfort", "steer_reversals_per_s"), "steer_rev_per_s", True),
    (("solver", "solve_ms_p99"), "solve_p99_ms", None),   # real-time either way, so not ranked
    (("events", "lane_invasions"), "lane_invasions", True),
    (("lane_departures",), "lane_departures", True),   # adjacent to lane_invasions (related)
]


def _dig(d, path):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def read_series(run_dir) -> dict:
    rows = list(csv.DictReader(open(Path(run_dir) / "series.csv")))
    if not rows:
        return {}
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


def road_curvature(series: dict):
    """Estimate cumulative distance and |curvature| along the driven path.

    Parameters
    ----------
    series : dict
        Column arrays from ``read_series``; needs ``x`` and ``y`` (m).

    Returns
    -------
    tuple of numpy.ndarray
        ``(dist, kappa)`` where ``dist`` is cumulative arc length (m) and
        ``kappa`` is the smoothed absolute curvature (1/m), boxcar-averaged
        over 9 samples. Both empty if fewer than 3 points are available.
    """
    x, y = series.get("x"), series.get("y")
    if x is None or len(x) < 3:
        return np.zeros(0), np.zeros(0)
    ds = np.hypot(np.diff(x), np.diff(y))
    dist = np.concatenate([[0.0], np.cumsum(ds)])
    hd = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    kap = np.abs(np.gradient(hd) / np.maximum(np.gradient(dist), 1e-3))
    return dist, np.convolve(kap, np.ones(9) / 9.0, mode="same")


def load_runs(run_dirs) -> list[dict]:
    out = []
    for d in run_dirs:
        d = Path(d)
        mj = d / "metrics.json"
        if not mj.exists():
            continue
        m = json.loads(mj.read_text())
        if m.get("samples", 0) < 2:
            continue
        out.append({"dir": str(d), "name": d.name, "scenario": m.get("scenario", {}),
                    "metrics": m, "controller": m.get("scenario", {}).get("controller", "?")})
    return out


def controllers(runs) -> list[str]:
    seen = []
    for r in runs:
        if r["controller"] not in seen:
            seen.append(r["controller"])
    return seen


def summary_table(runs) -> dict:
    """Aggregate TABLE_METRICS per controller across its runs.

    Returns
    -------
    dict
        ``{controller: {metric_label: (mean, std, n)}}``; a cell with no
        numeric samples is ``(None, None, 0)``.
    """
    out = {}
    for c in controllers(runs):
        rr = [r for r in runs if r["controller"] == c]
        agg = {}
        for path, label, _ in TABLE_METRICS:
            vals = [_dig(r["metrics"], path) for r in rr]
            vals = [float(v) for v in vals if isinstance(v, (int, float))]
            agg[label] = (float(np.mean(vals)), float(np.std(vals)), len(vals)) if vals else (None, None, 0)
        out[c] = agg
    return out


def scenario_of(run) -> str:
    """Return the scenario tag a run belongs to (the --condition / manifest condition)."""
    return run["scenario"].get("condition") or "all"


def scenarios(runs) -> list[str]:
    seen = []
    for r in runs:
        s = scenario_of(r)
        if s not in seen:
            seen.append(s)
    return seen


def group_of(run) -> str:
    """Return the scenario-list (group) a run belongs to.

    The group is the scenarios.yaml top-level list key the run came from (e.g.
    'flat_2d', 'elevated_3d'). Each list aggregates separately so different
    operating regimes (e.g. full-speed flat vs speed-capped elevated) are not
    pooled into one misleading number. Runs with no group (a bare launch, or
    pre-grouping logs) fall in 'main'.
    """
    return run["scenario"].get("group") or "main"


def groups(runs) -> list[str]:
    seen = []
    for r in runs:
        g = group_of(r)
        if g not in seen:
            seen.append(g)
    return seen


def summary_by_scenario(runs) -> dict:
    """Aggregate per scenario and controller (mean/std across that cell's seeds).

    This is the research-level breakdown; summary_table pools all scenarios
    together.

    Returns
    -------
    dict
        ``{scenario: {controller: {metric_label: (mean, std, n)}}}``.
    """
    return {s: summary_table([r for r in runs if scenario_of(r) == s]) for s in scenarios(runs)}


def speed_by_curvature(runs, moving_kmh: float = 5.0) -> dict:
    """Pool mean moving speed per curvature bin for each controller.

    The route-agnostic 'speed at the same kind of road' view; samples below
    ``moving_kmh`` (km/h) are dropped so standstills don't bias the means.

    Returns
    -------
    dict
        ``{controller: [mean_speed_kmh per CURV_LABELS bin]}``; bins with no
        samples are NaN.
    """
    out = {}
    for c in controllers(runs):
        pooled = [[] for _ in CURV_LABELS]
        for r in [r for r in runs if r["controller"] == c]:
            s = read_series(r["dir"])
            if "speed_kmh" not in s:
                continue
            _, kap = road_curvature(s)
            v = np.asarray(s["speed_kmh"], float)
            n = min(len(v), len(kap))            # guard: a partial/corrupt run can desync columns
            if n == 0:
                continue
            v, kap = v[:n], kap[:n]
            mv = v > moving_kmh
            for i in range(len(CURV_LABELS)):
                sel = mv & (kap >= CURV_EDGES[i]) & (kap < CURV_EDGES[i + 1])
                pooled[i].extend(v[sel].tolist())
        out[c] = [float(np.mean(b)) if b else float("nan") for b in pooled]
    return out


def display_name(c: str) -> str:
    """Return the human-facing controller label (UPPERCASE, pidpp -> PID++); data keys stay lowercase."""
    return {"pidpp": "PID++"}.get(c, c.upper())


def to_markdown(runs) -> str:
    """Render the full benchmark report as Markdown.

    Emits one metrics table and one speed-by-curvature table per scenario
    group. With a single 'main' group the group suffix is dropped from headings.

    Returns
    -------
    str
        The Markdown document, terminated by a trailing newline.
    """
    grps = groups(runs)
    single = grps == ["main"]
    L = [f"# Controller benchmark ({len(runs)} runs"
         + ("" if single else f", {len(grps)} scenario groups: {', '.join(grps)}") + ")", ""]
    for g in grps:
        grun = [r for r in runs if group_of(r) == g]
        tab = summary_table(grun)
        cs = list(tab.keys())
        gt = "" if single else f" — {g}"
        L.append(f"## Metrics{gt} (mean ± std across runs)")
        L.append("| metric | " + " | ".join(display_name(c) for c in cs) + " |")
        L.append("|---|" + "---|" * len(cs))
        for _, label, lower in TABLE_METRICS:
            cells = []
            for c in cs:
                mean, std, n = tab[c].get(label, (None, None, 0))
                cells.append(f"{mean:.2f} ± {std:.2f}" if mean is not None else "—")
            arrow = "" if lower is None else (" ↓" if lower else " ↑")
            L.append(f"| {label}{arrow} | " + " | ".join(cells) + " |")
        L.append("")
        L.append(f"## Speed by road curvature{gt} (km/h) — fair, route-agnostic")
        sbc = speed_by_curvature(grun)
        L.append("| segment | " + " | ".join(display_name(c) for c in cs) + " |")
        L.append("|---|" + "---|" * len(cs))
        for i, lab in enumerate(CURV_LABELS):
            L.append(f"| {lab} | " + " | ".join(f"{sbc[c][i]:.2f}" if not np.isnan(sbc[c][i]) else "—"
                                                for c in cs) + " |")
        L.append("")
    return "\n".join(L) + "\n"
