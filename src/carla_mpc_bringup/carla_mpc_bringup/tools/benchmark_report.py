"""Aggregate per-run logs into a controller-vs-controller comparison report.

Example invocations::

    ros2 run carla_mpc_bringup benchmark_report                 # all of ./logs/runs/*
    ros2 run carla_mpc_bringup benchmark_report logs/runs/2026* --out logs/benchmarks/my
    ros2 run carla_mpc_bringup benchmark_report --controllers pid,mpc,mpcc

Write a fresh timestamped directory under logs/benchmarks/ holding report.html,
report.md, summary.csv, and a plots/ directory. Lower-is-better metrics are drawn
hatched and the best controller per metric is highlighted. Read the run logs
(metrics.json + series.csv) rather than rosbags; aggregation lives in eval/benchmark.py.
"""
from __future__ import annotations

import argparse
import csv
import glob
import re
import sys
import time
from pathlib import Path

from carla_mpc_bringup.eval import benchmark as bench


def _safe(name: str) -> str:
    """Convert a scenario-group name to a filesystem-safe plot-filename suffix."""
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "group"

# Controller palette echoing the Foxglove path colours: green (driven), orange (predicted), blue (reference).
PALETTE = {"pid": "#00e676", "mpc": "#ff9100", "mpcc": "#2780ff"}
_EXTRA = ["#ff4081", "#b388ff", "#00e5ff"]
# Lane-invasion markers: each controller is drawn with a distinct shape *and* colour so an invasion is
# identifiable even where two paths overlap. Shapes cycle through this list; with more controllers than
# shapes the cycle repeats one size-tier larger (see _invasion_style), so a reused shape still reads as
# a different controller — i.e. size is the fallback differentiator once the shapes run out.
_INVASION_MARKERS = ["o", "s", "*", "^", "D", "P"]   # circle, square, star, triangle, diamond, plus
BG, FG, GRID, PANEL = "#16181c", "#e6e6e6", "#2c2f36", "#1d2026"
_disp = bench.display_name        # uppercase controller label (pidpp -> PID++); data keys stay lowercase

# Relative spread (hi-lo)/max(|lo|,|hi|) at which a metric's cells reach full red->green. Below it,
# colours compress toward amber so a narrow range (e.g. a ~10% spread) isn't painted as a big
# difference. Caveat: when the best value is ~0 any nonzero worst saturates to full colour, intended
# for "0 is ideal" metrics (lane_invasions) but a tiny relatively-spread metric saturates too.
_COLOR_FULL_SPREAD = 0.5
# Floor for higher-is-better metrics (speeds) so the slowest controller still reads amber-green, never
# red — slow is an achievement, not a failure. Ranking among controllers stays relative; this only
# biases the whole speed scale optimistically green.
_OPTIMISTIC_FLOOR = 0.55

# Shown under the Speed-by-curvature table; explains the curvature buckets.
SPEED_NOTE = ('<p class="note">R is the road\'s curve radius — larger R is straighter (R&gt;250 m ≈ '
              "straight, R&lt;33 m = a tight turn). Bucketing speed by R compares each controller on the "
              "same kind of road, independent of how much straight vs. curve a given route contains.</p>")

# One-line explanation per metric, shown under the aggregate table; keyed by the TABLE_METRICS label.
_METRIC_HELP = {
    "avg_speed_kmh": "Average speed while ACTUALLY driving — excludes the initial warm-up ramp, the route-end run-down, and any mid-route stop. Higher is better.",
    "distance_m": "Distance driven before the run ended. Context only — not ranked or coloured.",
    "lane_departures": "Number of excursions where the car strayed more than 1.5 m off the reference path (corner-cuts / big drifts). Lower is better.",
    "cte_rms_m": "Root-mean-square cross-track error — the typical lateral distance from the path. Lower is better.",
    "cte_max_m": "Worst-case lateral distance from the path over the run. Lower is better.",
    "heading_rms_deg": "RMS heading error — how far the car's heading deviates from the path tangent. Lower is better.",
    "lat_accel_max": "Peak lateral acceleration — the hardest cornering load felt by occupants. Lower is better.",
    "lat_jerk_rms": "RMS lateral jerk (rate of change of lateral acceleration) — a ride-smoothness / comfort measure. Lower is better.",
    "steer_rev_per_s": "Steering reversals per second — how often the wheel changes direction (chatter). Lower is better.",
    "solve_p99_ms": "99th-percentile controller solve time. A real-time sanity check; not ranked since all run comfortably in real time.",
    "lane_invasions": "Number of times the car crossed a lane marking (CARLA's lane sensor, with normal on-path crossings masked out). Lower is better.",
}


def _color(c, i):
    return PALETTE.get(c, _EXTRA[i % len(_EXTRA)])


def _invasion_style(i):
    """Return the (marker, point-size) a controller's lane invasions are drawn with.

    The shape cycles through _INVASION_MARKERS; each full wrap-around grows the size tier, so
    controllers that share a shape stay distinguishable by size. Star/plus glyphs read smaller than a
    circle at equal point-area, so they get a compensating size bump.
    """
    marker = _INVASION_MARKERS[i % len(_INVASION_MARKERS)]
    tier = i // len(_INVASION_MARKERS)
    base = 135.0 if marker in ("*", "P") else 90.0   # big enough that overlapping marks still read as distinct shapes
    return marker, base * (1.0 + 0.7 * tier)


def _grad_cmap(base):
    """Build a per-controller colormap (dim -> base -> bright).

    Lets a driven path be coloured by distance from start (like the route_lab plots) while
    still reading as that controller's colour.
    """
    import matplotlib.colors as mcolors
    r, g, b = mcolors.to_rgb(base)
    dim = (r * 0.30, g * 0.30, b * 0.30)
    bright = (r + (1 - r) * 0.65, g + (1 - g) * 0.65, b + (1 - b) * 0.65)
    return mcolors.LinearSegmentedColormap.from_list("grad", [dim, base, bright])


def _resolve(patterns) -> list[str]:
    if not patterns:
        patterns = ["logs/runs/*"]
    dirs = []
    for p in patterns:
        for d in sorted(glob.glob(p)):
            if (Path(d) / "metrics.json").exists():
                dirs.append(d)
    return dirs


def _write_csv(path, runs):
    tab = bench.summary_table(runs)
    labels = [lbl for _, lbl, _ in bench.TABLE_METRICS]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["controller", "n_runs"] + labels)
        for c, agg in tab.items():
            n = max((agg[l][2] for l in labels), default=0)
            w.writerow([_disp(c), n] + [f"{agg[l][0]:.2f}" if agg[l][0] is not None else "" for l in labels])


def _write_scenario_csv(path, runs):
    """Write the per-scenario x controller breakdown (mean +/- std).

    One row per (scenario, controller); written only when more than one scenario is present.
    """
    by = bench.summary_by_scenario(runs)
    labels = [lbl for _, lbl, _ in bench.TABLE_METRICS]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "controller", "n_runs"]
                   + [f"{l}_mean" for l in labels] + [f"{l}_std" for l in labels])
        for scen, tab in by.items():
            for c, agg in tab.items():
                n = max((agg[l][2] for l in labels), default=0)
                means = [f"{agg[l][0]:.2f}" if agg[l][0] is not None else "" for l in labels]
                stds = [f"{agg[l][1]:.2f}" if agg[l][1] is not None else "" for l in labels]
                w.writerow([scen, _disp(c), n] + means + stds)


def _finite(v):
    return v is not None and v == v          # v == v is False for NaN, no numpy needed


def _best(values, lower):
    """Return the index of the best (min if lower-is-better, else max) among finite values."""
    idx = [i for i, v in enumerate(values) if _finite(v)]
    if not idx:
        return None
    return min(idx, key=lambda i: values[i]) if lower else max(idx, key=lambda i: values[i])


def _best_set(values, lower):
    """Return all indices tied for best, so co-winners each get a star (e.g. MPC and MPCC both at 0)."""
    bi = _best(values, lower)
    if bi is None:
        return set()
    bv = values[bi]
    return {i for i, v in enumerate(values) if _finite(v) and abs(v - bv) <= 1e-9}


def _grad(t: float) -> str:
    """Map goodness t in [0, 1] (1 = best) to a hex colour, red -> amber -> green."""
    t = max(0.0, min(1.0, t))
    stops = [(0.0, (0x9a, 0x26, 0x26)), (0.5, (0x8a, 0x73, 0x1f)), (1.0, (0x1f, 0x84, 0x3e))]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1 or t1 == 1.0:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            r, g, b = (int(a + (d - a) * f) for a, d in zip(c0, c1))
            return f"#{r:02x}{g:02x}{b:02x}"
    return "#1f843e"


# --------------------------------------------------------------------- plots
def _theme():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib as mpl
    mpl.rcParams.update({
        "figure.facecolor": BG, "axes.facecolor": PANEL, "savefig.facecolor": BG,
        "text.color": FG, "axes.labelcolor": FG, "xtick.color": FG, "ytick.color": FG,
        "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.alpha": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "font.size": 11,
        "axes.titleweight": "bold", "figure.titleweight": "bold",
    })


def _bars(ax, labels, values, colors, lower=False, fmt="{:.2f}"):
    import numpy as np
    xs = np.arange(len(labels))
    for x, v, c in zip(xs, values, colors):
        if v is None:
            continue
        ax.bar(x, v, 0.68, color=c, edgecolor=FG, linewidth=0.6,
               hatch="////" if lower else None, alpha=0.95)
        ax.annotate(fmt.format(v), (x, v), textcoords="offset points", xytext=(0, 3),
                    ha="center", fontsize=7, color=FG, fontweight="bold")
    for bi in _best_set(values, lower):      # star every winner, including tied co-winners
        ax.annotate("★", (xs[bi], values[bi]), textcoords="offset points", xytext=(0, 13),
                    ha="center", fontsize=11, color="#ffd600")
    ax.set_xticks(xs); ax.set_xticklabels(labels, fontsize=7)
    ax.margins(y=0.18)


def _metrics_panel(out, runs, suffix, title_tag):
    """Render metrics.png<suffix>: one bar panel per metric for the given run subset (a scenario group)."""
    import matplotlib.pyplot as plt
    cs = bench.controllers(runs)
    cols = [_color(c, i) for i, c in enumerate(cs)]
    tab = bench.summary_table(runs)
    # Skip non-comparable metrics (distance, solve-time) and lane_departures (a coarse subset of
    # lane_invasions, nearly always 0). Lower-is-better panels are flagged.
    _NO_GRAPH = {"distance_m", "solve_p99_ms", "lane_departures"}
    keys = [(lbl, lower) for _, lbl, lower in bench.TABLE_METRICS if lbl not in _NO_GRAPH]
    ncol = 4
    nrow = (len(keys) + ncol - 1) // ncol
    fig, axs = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 3.5 * nrow), squeeze=False)
    fig.suptitle("Controller Metrics" + title_tag, fontsize=15)
    for ax, (k, lower) in zip(axs.ravel(), keys):
        vals = [tab[c][k][0] for c in cs]
        _bars(ax, [_disp(c) for c in cs], vals, cols, lower=lower)
        ax.set_title(("▼ " if lower else "▲ ") + k, fontsize=11,
                     color=("#ff8a80" if lower else "#b9f6ca"))
    for ax in axs.ravel()[len(keys):]:
        ax.axis("off")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out / "plots" / f"metrics{suffix}.png", dpi=120); plt.close(fig)


def _speed_curve_panel(out, runs, suffix, title_tag):
    """Render speed_by_curvature.png<suffix>: grouped speed-by-curvature bars for the given run subset."""
    import matplotlib.pyplot as plt
    import numpy as np
    cs = bench.controllers(runs)
    cols = [_color(c, i) for i, c in enumerate(cs)]
    sbc = bench.speed_by_curvature(runs)          # higher is better everywhere
    fig, ax = plt.subplots(figsize=(11, 4.5))
    x = np.arange(len(bench.CURV_LABELS)); w = 0.8 / max(len(cs), 1)
    for j, c in enumerate(cs):
        bars = ax.bar(x + j * w, sbc[c], w, label=_disp(c), color=cols[j], edgecolor=FG, linewidth=0.5)
        for b, v in zip(bars, sbc[c]):
            if not np.isnan(v):
                ax.annotate(f"{v:.2f}", (b.get_x() + b.get_width() / 2, v),
                            textcoords="offset points", xytext=(0, 2), ha="center",
                            fontsize=6, color=FG)
    ax.set_xticks(x + w * (len(cs) - 1) / 2); ax.set_xticklabels(bench.CURV_LABELS, fontsize=9)
    ax.set_ylabel("mean speed (km/h)")
    ax.set_title("Speed by Road Curvature" + title_tag + "  (▲ higher is better)")
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=FG)
    fig.tight_layout(); fig.savefig(out / "plots" / f"speed_by_curvature{suffix}.png", dpi=120); plt.close(fig)


def _plots(out, runs):
    _theme()
    import numpy as np
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    import matplotlib.pyplot as plt
    cs = bench.controllers(runs)
    cols = [_color(c, i) for i, c in enumerate(cs)]

    # Metrics + speed-by-curvature, one set per scenario group (named by its scenarios.yaml list).
    # A single group keeps the bare filenames (metrics.png / speed_by_curvature.png).
    grps = bench.groups(runs)
    single = grps == ["main"]
    for g in grps:
        grun = [r for r in runs if bench.group_of(r) == g]
        suffix = "" if single else f"_{_safe(g)}"
        tag = "" if single else f" — {g}"
        _metrics_panel(out, grun, suffix, tag)
        _speed_curve_panel(out, grun, suffix, tag)

    # Driven-path overlays per scenario: one panel per scenario shares a map, whereas a single global
    # overlay across different maps would be meaningless. Lane invasions are marked per controller with
    # a distinct shape + colour (see _invasion_style); the HTML caption spells out what the marks mean.
    scens = bench.scenarios(runs)
    if scens:
        ncol = min(3, len(scens)); nrow = (len(scens) + ncol - 1) // ncol
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.7 * ncol, 4.4 * nrow), squeeze=False)
        for si, scen in enumerate(scens):
            ax = axs[si // ncol][si % ncol]
            xs_all, ys_all = [], []
            for j, c in enumerate(cs):
                r = next((r for r in runs if r["controller"] == c and bench.scenario_of(r) == scen), None)
                if not r:
                    continue
                s = bench.read_series(r["dir"])
                if "x" not in s:
                    continue
                x = np.asarray(s["x"], float); y = np.asarray(s["y"], float)
                if x.size < 2:
                    continue
                xs_all.append(x); ys_all.append(y)
                d = np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]    # distance from start
                pts = np.column_stack([x, y]).reshape(-1, 1, 2)
                segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
                lc = LineCollection(segs, cmap=_grad_cmap(cols[j]), lw=1.6, alpha=0.9, zorder=3 + j)
                lc.set_array(d[:-1])                                           # colour the path by distance from start
                ax.add_collection(lc)
                lanes = np.asarray(s.get("lane_invasions_total", []), float)
                liv = (np.where(np.diff(lanes) > 0)[0] + 1) if lanes.size > 1 else []
                if len(liv):
                    # Distinct open shape + colour per controller, so invasions at the same spot (e.g.
                    # PID under PID++) stay legible instead of hiding one another — the shape
                    # disambiguates even where colours repeat.
                    marker, size = _invasion_style(j)
                    ax.scatter(x[liv], y[liv], marker=marker, facecolors="none",
                               edgecolors=cols[j], s=size, linewidths=1.5, zorder=8)
            if xs_all:                                                        # LineCollection needs limits set explicitly
                ax.set_xlim(min(a.min() for a in xs_all) - 5, max(a.max() for a in xs_all) + 5)
                ax.set_ylim(min(a.min() for a in ys_all) - 5, max(a.max() for a in ys_all) + 5)
            ax.set_aspect("equal", "datalim"); ax.invert_yaxis()   # CARLA y is down; flip so north is up (map convention)
            ax.set_title(scen, color=FG, fontsize=10)
            ax.tick_params(colors=FG, labelsize=7)
        for k in range(len(scens), nrow * ncol):
            axs[k // ncol][k % ncol].axis("off")
        # Each legend row shows the controller's path colour and its lane-invasion mark together.
        handles = []
        for j, c in enumerate(cs):
            mk = _invasion_style(j)[0]
            ms = 9 + 3 * (j // len(_INVASION_MARKERS))      # echo the on-plot size tier for reused shapes
            handles.append(Line2D([0], [0], color=cols[j], lw=2.5, marker=mk, markerfacecolor="none",
                                  markeredgecolor=cols[j], markersize=ms, label=_disp(c)))
        axs[0][0].legend(handles=handles, facecolor=PANEL, edgecolor=GRID, labelcolor=FG,
                         fontsize=8, title="open mark = lane invasion", title_fontsize=7)
        fig.tight_layout()
        fig.savefig(out / "plots" / "paths_by_scenario.png", dpi=120); plt.close(fig)


# --------------------------------------------------------------------- html
def _html(runs) -> str:
    def row(label, vals, lower, fmt="{:.2f}"):
        fin = [v for v in vals if _finite(v)]
        lo, hi = (min(fin), max(fin)) if len(fin) >= 2 else (None, None)
        tds = []
        for v in vals:
            if not _finite(v):
                tds.append("<td>—</td>")
                continue
            style = ""
            if lo is not None and hi > lo and lower is not None:
                t = (hi - v) / (hi - lo) if lower else (v - lo) / (hi - lo)   # 1 = best
                # Compress toward amber (0.5) when the spread is small relative to the magnitude, so a
                # narrow range stays muted instead of spanning the full red->green (see _COLOR_FULL_SPREAD).
                k = min(1.0, (hi - lo) / max(abs(lo), abs(hi), 1e-9) / _COLOR_FULL_SPREAD)
                t = 0.5 + (t - 0.5) * k
                if lower is False:        # higher-is-better (speeds): lean green, never red
                    t = _OPTIMISTIC_FLOOR + (1.0 - _OPTIMISTIC_FLOOR) * t
                style = f' style="background:{_grad(t)};color:#fff;font-weight:600"'
            tds.append(f"<td{style}>{fmt.format(v)}</td>")
        arrow = "" if lower is None else ('<span class="lo">▼ lower</span>' if lower
                                          else '<span class="hi">▲ higher</span>')
        return f"<tr><td class='m'>{label} {arrow}</td>{''.join(tds)}</tr>"

    # Metrics + Speed-by-curvature, rendered once per scenario group (named by its scenarios.yaml list).
    # Each group is a separate operating regime (e.g. full-speed flat 2D vs speed-capped elevated 3D), so
    # pooling them into one number is misleading; keep them apart and label each aggregation.
    grps = bench.groups(runs)
    single = grps == ["main"]

    def group_section(g):
        grun = [r for r in runs if bench.group_of(r) == g]
        tab = bench.summary_table(grun)
        sbc = bench.speed_by_curvature(grun)
        cs = list(tab.keys())
        nrun = {c: sum(1 for r in grun if r["controller"] == c) for c in cs}
        head = "".join(f"<th style='color:{_color(c,i)}'>{_disp(c)}<br><small>×{nrun[c]}</small></th>"
                       for i, c in enumerate(cs))
        mrows = "".join(row(lbl, [tab[c][lbl][0] for c in cs], lower)
                        for _, lbl, lower in bench.TABLE_METRICS)
        srows = "".join(row(lab, [sbc[c][i] for c in cs], False, "{:.2f}")
                        for i, lab in enumerate(bench.CURV_LABELS))
        suffix = "" if single else f"_{_safe(g)}"
        tag = "" if single else f" <small>· {g}</small>"
        return (f"<h2>Metrics{tag}</h2>"
                f"<table><tr><th>metric</th>{head}</tr>{mrows}</table>"
                + (glossary if single else "")
                + f'<img src="plots/metrics{suffix}.png">'
                + f"<h2>Speed by Road Curvature{tag}</h2>"
                + (SPEED_NOTE if single else "")
                + f"<table><tr><th>segment</th>{head}</tr>{srows}</table>"
                + f'<img src="plots/speed_by_curvature{suffix}.png">')

    # Per-scenario (per-route) breakdown: the same metrics split by route, under the aggregate.
    cs = bench.controllers(runs)
    by = bench.summary_by_scenario(runs)
    per_scen = ""
    if len(by) > 1:
        def sval(stab, c, lbl):
            v = stab.get(c, {}).get(lbl)
            return v[0] if v else None
        blocks = []
        for scen, stab in by.items():
            sc = [c for c in cs if c in stab] or cs
            sh = "".join(f"<th style='color:{_color(c,i)}'>{_disp(c)}</th>" for i, c in enumerate(sc))
            rws = "".join(row(lbl, [sval(stab, c, lbl) for c in sc], lower)
                          for _, lbl, lower in bench.TABLE_METRICS)
            blocks.append(f"<h3>{scen}</h3><table><tr><th>metric</th>{sh}</tr>{rws}</table>")
        per_scen = "<h2>Per-Scenario Metrics</h2>" + "".join(blocks)
    glossary = ("<div class='gloss'>" + "".join(
        f"<div><b>{lbl}</b> — {_METRIC_HELP[lbl]}</div>"
        for _, lbl, _ in bench.TABLE_METRICS if lbl in _METRIC_HELP)
        + "</div>")
    refs_by = {}
    for r in runs:
        refs_by.setdefault(r["controller"], []).append(Path(r["dir"]).name)
    references = ("<h2>References</h2><p class='note'>Every individual run folder under "
                  "<code>logs/runs/</code> that fed this report.</p>" + "".join(
        f"<h3>{_disp(c)} <small>×{len(v)}</small></h3><ul class='refs'>"
        + "".join(f"<li>{name}</li>" for name in sorted(v)) + "</ul>"
        for c, v in refs_by.items()))
    css = """
    body{background:#101216;color:#e6e6e6;font:14px/1.5 system-ui,Segoe UI,Roboto,sans-serif;margin:0;padding:28px}
    h1{font-weight:700;letter-spacing:.3px} h2{color:#9fb3c8;margin-top:30px;border-bottom:1px solid #2c2f36;padding-bottom:6px}
    h3{color:#cfd8e3;margin:18px 0 2px;font-size:15px}
    table{border-collapse:collapse;margin:14px 0;background:#1d2026;border-radius:10px;overflow:hidden;box-shadow:0 4px 18px #0008}
    th,td{padding:9px 16px;text-align:right} th{background:#23272e;font-weight:600}
    td.m{text-align:left;color:#cfd8e3}
    .lo{color:#ff8a80;font-size:11px} .hi{color:#b9f6ca;font-size:11px}
    img{max-width:100%;border-radius:10px;margin:10px 0;box-shadow:0 4px 18px #0008}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:18px} small{color:#8a93a0}
    .gloss{columns:2;column-gap:30px;font-size:12px;color:#aeb6c2;margin:10px 0 2px}
    .gloss div{break-inside:avoid;margin:3px 0} .gloss b{color:#cfd8e3}
    .note{color:#9fb3c8;font-size:12.5px;margin:4px 0 0;max-width:78ch}
    .refs{columns:3;font-size:11.5px;color:#aeb6c2;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;margin:2px 0 10px;list-style:none;padding-left:0}
    .refs li{margin:1px 0;break-inside:avoid} code{background:#23272e;padding:1px 5px;border-radius:4px}
    """
    # One Metrics + Speed block per scenario group. A single (unnamed) group renders the plain layout;
    # with multiple groups each aggregation is titled with its scenarios.yaml list name, and the shared
    # metric glossary + curvature note are shown once up front instead of inside each.
    sections = "".join(group_section(g) for g in grps)
    preamble = "" if single else (f"<p class='note'>{len(grps)} scenario groups, aggregated separately: "
                                  + ", ".join(f"<b>{g}</b>" for g in grps) + f".</p>{glossary}{SPEED_NOTE}")
    groups_note = "" if single else "<small> · grouped</small>"
    return f"""<!doctype html><html><head><meta charset=utf-8><title>Controller benchmark</title>
<style>{css}</style></head><body>
<h1>🏁 Controller benchmark <small>· {len(runs)} runs{groups_note} · {time.strftime('%Y-%m-%d %H:%M')}</small></h1>
{preamble}
{sections}
{per_scen}
<h2>Driven Paths Per Scenario</h2>
<p class='note'>Each controller's path is shaded along its length (dim at the route start, bright at the
end). The <b>open marks flag lane invasions</b>.</p>
<img src="plots/paths_by_scenario.png" style="max-width:100%">
{references}
</body></html>"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run dirs or globs (default: logs/runs/*)")
    ap.add_argument("--out", default=None, help="report dir (default: logs/benchmarks/<timestamp>)")
    ap.add_argument("--controllers", default="",
                    help="comma-separated controllers to include, IN THE ORDER they should appear in "
                         "every table and plot, e.g. pid,pidpp,mpc,mpcc,mpc@100,mpcc@100")
    args, _ = ap.parse_known_args()

    runs = bench.load_runs(_resolve(args.runs))
    if args.controllers:
        order = [c.strip() for c in args.controllers.split(",") if c.strip()]
        keep = set(order)
        runs = [r for r in runs if r["controller"] in keep]
        # Fix the controller order to the --controllers list. Every table/plot keys off the
        # controllers' first-appearance in `runs` (bench.controllers), so a STABLE sort by the
        # requested rank makes that order match the command line exactly; stability keeps each
        # controller's runs in their original scenario order, so scenario ordering is unchanged.
        rank = {c: i for i, c in enumerate(order)}
        runs.sort(key=lambda r: rank[r["controller"]])
    if not runs:
        print(f"No usable runs found in: {args.runs or ['logs/runs/*']}", file=sys.stderr)
        return 1

    out = Path(args.out or f"logs/benchmarks/{time.strftime('%Y%m%d-%H%M%S')}")
    (out / "plots").mkdir(parents=True, exist_ok=True)
    (out / "report.md").write_text(bench.to_markdown(runs))
    (out / "report.html").write_text(_html(runs))
    _write_csv(out / "summary.csv", runs)
    if len(bench.scenarios(runs)) > 1:                 # per-scenario breakdown only when multiple scenarios
        _write_scenario_csv(out / "summary_by_scenario.csv", runs)
    try:
        _plots(out, runs)
    except Exception as e:  # noqa: BLE001
        (out / "plots" / "ERROR.txt").write_text(f"plotting failed: {e}\n")
    print(bench.to_markdown(runs))
    print(f"\nReport -> {out}/  (open report.html)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
