"""Accumulate per-step run samples, then write a human-readable log directory.

The output is a plain directory (not a ROS bag): ``metrics.json`` + ``series.csv`` +
``plots/*.png`` + ``summary.txt`` + ``replay.gif``. ``record(**sample)`` is called every
control tick and only appends a row, so it stays cheap in the hot loop. ``write()`` runs
once at shutdown and does the numpy/matplotlib work.

The output feeds the benchmark aggregator: ``scenario`` carries the replicable
inputs (map, seed, controller, model, condition, target speed, dt) and
``metrics.json`` carries the comparable outputs.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

from carla_mpc_bringup.eval.metrics import compute_metrics, derived_series, summary_text

FIELDS = ["t", "x", "y", "yaw", "speed_kmh", "target_speed_kmh",
          "throttle", "brake", "steer", "cross_track_m", "heading_error_deg",
          "solve_time_ms", "lat_accel_ms2", "lane_invasions_total"]


class RunLogger:
    """Collect per-tick samples and emit metrics, CSV, plots, and a replay GIF."""

    def __init__(self, fields=FIELDS):
        self.fields = list(fields)
        self._rows: list[dict] = []
        self._series: dict | None = None   # cached by write_core, reused by write_artifacts

    def record(self, **sample) -> None:
        self._rows.append(sample)

    def __len__(self) -> int:
        return len(self._rows)

    def series(self) -> dict:
        return {k: [r.get(k, float("nan")) for r in self._rows] for k in self.fields}

    @staticmethod
    def _drop_backward_time(rows):
        """Drop the tail once sim-time jumps backward, keeping the monotonic prefix.

        On world teardown CARLA resets the clock, so the final cleanup sample
        lands at t~0 with speed 0 and a huge position jump. That one garbage row
        makes trip.duration negative (avg_speed -> 0) and spikes cte_max.
        """
        out, t_prev = [], None
        for r in rows:
            t = r.get("t")
            if t is not None and t_prev is not None and t < t_prev - 0.5:
                break
            if t is not None:
                t_prev = t
            out.append(r)
        return out

    def write(self, out_dir, scenario: dict) -> dict:
        """Write metrics, CSV, plots, and replay GIF for the run to ``out_dir``.

        Convenience wrapper around ``write_core`` (the fast, must-keep outputs) followed
        by ``write_artifacts`` (the slow, expendable figures) — identical behaviour to
        before the split. A caller that tears down a live simulator between persisting
        data and rendering figures should call the two halves directly instead; see
        ``write_core``.

        Parameters
        ----------
        out_dir : path-like
            Output directory; created along with a ``plots/`` subdirectory.
        scenario : dict
            Replicable run inputs (map, seed, controller, model, condition,
            target speed, dt) passed through to metrics and plot titles.

        Returns
        -------
        dict
            The computed metrics, also written to ``metrics.json``.
        """
        metrics = self.write_core(out_dir, scenario)
        self.write_artifacts(out_dir, scenario)
        return metrics

    def write_core(self, out_dir, scenario: dict) -> dict:
        """Write the fast, must-keep outputs: ``metrics.json``, ``summary.txt``, ``series.csv``.

        Split from the slow plot/GIF rendering so a caller can persist a run's data
        *before* a risky teardown. CARLA actor ``destroy()`` can hit a server time-out
        that surfaces as a C++ exception calling ``std::terminate`` — an uncatchable
        process abort — so writing the core first means such an abort can no longer
        discard the run. The benchmark aggregator reads only ``metrics.json`` +
        ``series.csv``, both written here. Returns the metrics and caches the computed
        series for a subsequent ``write_artifacts``.
        """
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._rows = self._drop_backward_time(self._rows)
        self._series = self.series()
        metrics = compute_metrics(self._series, scenario)
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2))
        (out / "summary.txt").write_text(summary_text(metrics))
        self._write_csv(out / "series.csv", self._series)
        return metrics

    def write_artifacts(self, out_dir, scenario: dict) -> None:
        """Write the slow, expendable outputs: the ``plots/`` PNGs and ``replay.gif``.

        Safe to skip, fail, or lose to a kill without discarding the run — its core data
        is already on disk via ``write_core``. Reuses the series cached by ``write_core``,
        recomputing it only if called standalone.
        """
        out = Path(out_dir)
        (out / "plots").mkdir(parents=True, exist_ok=True)
        series = self._series if self._series is not None else self.series()
        try:
            self._write_plots(out / "plots", series, scenario)
        except Exception as e:  # noqa: BLE001  plots must never sink a run's metrics
            (out / "plots" / "ERROR.txt").write_text(f"plotting failed: {e}\n")
        try:
            self._write_video(out / "replay.gif", series, scenario)
        except Exception as e:  # noqa: BLE001  video must never sink a run's metrics
            (out / "plots" / "VIDEO_ERROR.txt").write_text(f"video failed: {e}\n")

    def _write_csv(self, path, series) -> None:
        n = len(self._rows)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.fields)
            for i in range(n):
                w.writerow([series[k][i] for k in self.fields])

    def _write_video(self, path, series, scenario) -> None:
        """Write a low-res top-down replay of the run as a GIF.

        Renders the car driving its path (with trail), lane-violation markers,
        and a speed-vs-time cursor. Uses PillowWriter so no ffmpeg is needed.
        """
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        import numpy as np

        def col(k):
            return np.asarray(series.get(k, []), dtype=float)
        x, y, spd, t = col("x"), col("y"), col("speed_kmh"), col("t")
        if x.size < 5:
            return
        t0 = t - t[0]
        lanes = col("lane_invasions_total")
        li = (np.where(np.diff(lanes) > 0)[0] + 1) if lanes.size > 1 else np.array([], dtype=int)
        # Decimate to ~6 fps, cap ~100 frames: keeps the GIF small and the encode
        # quick enough to finish inside `ros2 launch`'s SIGINT->SIGKILL grace
        # (~15 s) when an extended run is stopped with Ctrl+C.
        nframes = int(min(100, max(10, min(t0[-1], 40.0) * 6)))
        idx = np.linspace(0, x.size - 1, nframes).astype(int)

        fig, (axp, axs) = plt.subplots(1, 2, figsize=(9.0, 3.6), dpi=60)
        tag = scenario.get("controller", "?")
        axp.plot(x, y, color="#555", lw=1.0); axp.set_aspect("equal", "datalim"); axp.invert_yaxis()
        if li.size:
            axp.scatter(x[li], y[li], facecolors="none", edgecolors="red", s=70, linewidths=1.6, zorder=4)
        trail, = axp.plot([], [], "-", color="#00e5ff", lw=2.2, alpha=0.85)
        car, = axp.plot([], [], "o", color="#00e5ff", ms=8, zorder=6)
        axp.set_title(f"path  [{tag}]"); axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
        txt = axp.text(0.02, 0.98, "", transform=axp.transAxes, va="top", fontsize=9,
                       color="#111", bbox=dict(fc="white", alpha=0.6, ec="none"))
        axs.plot(t0, spd, color="#00b050", lw=1.2); cur = axs.axvline(0, color="#d00", lw=1.2)
        axs.set_xlabel("t (s)"); axs.set_ylabel("speed (km/h)"); axs.set_title("speed")
        fig.tight_layout()

        def upd(k):
            i = idx[k]
            j = max(0, i - 25)
            trail.set_data(x[j:i + 1], y[j:i + 1]); car.set_data([x[i]], [y[i]])
            cur.set_xdata([t0[i], t0[i]]); txt.set_text(f"t={t0[i]:.1f}s  v={spd[i]:.0f} km/h")
            return trail, car, cur, txt

        anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
        import os
        # Encode to a temp path then os.replace: a Ctrl+C mid-encode leaves no
        # corrupt replay.gif. The .gif suffix lets PillowWriter/PIL infer format.
        tmp = str(path) + ".part.gif"
        try:
            anim.save(tmp, writer=PillowWriter(fps=8))
            os.replace(tmp, str(path))
        finally:
            plt.close(fig)
            if os.path.exists(tmp):
                try:
                    os.remove(tmp)
                except OSError:
                    pass

    def _write_plots(self, plot_dir, series, scenario) -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        def col(k):
            return np.asarray(series.get(k, []), dtype=float)

        t = col("t")
        t0 = t - t[0] if t.size else t
        d = derived_series(series)
        tag = f"{scenario.get('controller','?')}" + (
            f"/{scenario.get('model')}" if scenario.get("model") else "")

        # Driven path coloured by speed, with lane-violation markers.
        x, y, spd = col("x"), col("y"), col("speed_kmh")
        if x.size > 1:
            fig, ax = plt.subplots(figsize=(7, 7))
            sc = ax.scatter(x, y, c=spd, cmap="turbo", s=6)
            fig.colorbar(sc, ax=ax, label="speed (km/h)")
            lanes = col("lane_invasions_total")          # cumulative count: mark indices where it rises
            li = (np.where(np.diff(lanes) > 0)[0] + 1) if lanes.size > 1 else np.array([], dtype=int)
            if li.size:
                ax.scatter(x[li], y[li], facecolors="none", edgecolors="red", s=110, linewidths=2.0,
                           zorder=5, label=f"lane violation ({li.size})")
                ax.legend(loc="best")
            ax.scatter([x[0]], [y[0]], c="white", edgecolors="k", s=70, zorder=6)
            ax.set_aspect("equal", "datalim"); ax.invert_yaxis()   # CARLA y is down; flip so north is up
            ax.set_title(f"Driven path — colour = speed  [{tag}]")
            ax.set_xlabel("x (m, CARLA)"); ax.set_ylabel("y (m, CARLA)")
            fig.tight_layout(); fig.savefig(plot_dir / "trajectory.png", dpi=110); plt.close(fig)

        self._line(plt, plot_dir / "speed.png", t0,
                   [(col("speed_kmh"), "speed", "#00b050"),
                    (col("target_speed_kmh"), "target", "#ffab00")],
                   "Speed (km/h)", f"Speed — actual vs target  [{tag}]")

        self._line(plt, plot_dir / "tracking.png", t0,
                   [(col("cross_track_m"), "cross-track (m)", "#ff4081"),
                    (np.radians(col("heading_error_deg")), "heading (rad)", "#b388ff")],
                   "error", f"Tracking error  [{tag}]")

        self._line(plt, plot_dir / "control.png", t0,
                   [(col("throttle"), "throttle", "#00b050"),
                    (col("brake"), "brake", "#ff5252"),
                    (col("steer"), "steer", "#40c4ff")],
                   "command [-1,1]", f"Control commands  [{tag}]")

        # Lateral + longitudinal acceleration; dashed lines mark the lateral grip limit.
        a_lat_max = scenario.get("a_lat_max")
        fig, ax = plt.subplots(figsize=(10, 3.2))
        ax.plot(t0, d["lat_acc"], label="lateral", color="#ff7043")
        ax.plot(t0, d["long_acc"], label="longitudinal", color="#42a5f5", alpha=0.8)
        if a_lat_max:
            ax.axhline(a_lat_max, ls="--", c="#888"); ax.axhline(-a_lat_max, ls="--", c="#888")
        ax.set_xlabel("t (s)"); ax.set_ylabel("accel (m/s²)")
        ax.set_title(f"Acceleration  [{tag}]"); ax.legend(loc="upper right"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(plot_dir / "accel.png", dpi=110); plt.close(fig)

        # MPC solve-time histogram (positive, finite samples only).
        solve = col("solve_time_ms"); solve = solve[np.isfinite(solve) & (solve > 0)]
        if solve.size > 5:
            fig, ax = plt.subplots(figsize=(6, 3.2))
            ax.hist(solve, bins=40, color="#00e5ff")
            ax.axvline(np.percentile(solve, 99), ls="--", c="#ff5252", label="p99")
            ax.set_xlabel("solve time (ms)"); ax.set_ylabel("count")
            ax.set_title(f"MPC solve time  [{tag}]"); ax.legend()
            fig.tight_layout(); fig.savefig(plot_dir / "solve_time.png", dpi=110); plt.close(fig)

    @staticmethod
    def _line(plt, path, t, series_colors, ylabel, title):
        if t.size < 2:
            return
        fig, ax = plt.subplots(figsize=(10, 3.2))
        for v, label, color in series_colors:
            if v.size == t.size:
                ax.plot(t, v, label=label, color=color)
        ax.set_xlabel("t (s)"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(loc="upper right"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)
