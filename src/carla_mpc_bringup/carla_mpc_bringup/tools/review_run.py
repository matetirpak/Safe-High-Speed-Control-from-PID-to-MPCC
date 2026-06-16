"""Review a recorded run: read its rosbag2 and report control metrics.

Companion to record_run. Reads the scalar control telemetry from the bag and
prints a summary, also written as summary.json in the run directory. Metrics:

  * speed: mean / max / final vs target (tracking)
  * cross-track: mean / p95 / max; heading error: mean / max (accuracy)
  * steering: mean |steer|, jitter RMS, and reversals/s (micro left-right
    chatter; high = twitchy)
  * MPC solve time: p50 / p99 / max
  * duration

Usage:
    ros2 run carla_mpc_bringup review_run runs/<name>          # prints + summary.json
    ros2 run carla_mpc_bringup review_run runs/<name> --plot   # + review.png

Accepts a run directory (locating <run>/bag) or a bag directory directly.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SCALARS = [
    "speed_kmh", "target_speed_kmh", "throttle", "brake", "steer",
    "cross_track_error", "heading_error_deg", "solve_time_ms",
]


def _find_bag(arg: Path) -> Path:
    """Resolve a run or bag directory to the rosbag2 directory holding metadata.yaml."""
    if (arg / "metadata.yaml").exists():
        return arg
    if (arg / "bag" / "metadata.yaml").exists():
        return arg / "bag"
    # Treat arg as a runs/ parent: take the newest child that has a bag.
    cands = sorted(arg.glob("*/bag/metadata.yaml"))
    if cands:
        return cands[-1].parent
    raise FileNotFoundError(f"no rosbag2 (metadata.yaml) under {arg}")


def _read_scalars(bag_dir: Path) -> dict[str, np.ndarray]:
    """Read the /controller/* Float32 telemetry from the bag into per-scalar arrays.

    Returns a dict mapping each scalar name to its value array and ``<name>_t`` to
    the matching timestamps in seconds, zeroed to the first sample across scalars.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    # Prefer the storage id declared in metadata; fall back to the common default.
    storage_id = "sqlite3"
    try:
        import yaml
        meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
        storage_id = meta["rosbag2_bagfile_information"]["storage_identifier"]
    except Exception:
        pass

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {f"/controller/{s}": s for s in SCALARS}
    data: dict[str, list] = {s: [] for s in SCALARS}
    tdata: dict[str, list] = {s: [] for s in SCALARS}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in want and types.get(topic) == "std_msgs/msg/Float32":
            msg = deserialize_message(raw, get_message(types[topic]))
            key = want[topic]
            data[key].append(float(msg.data))
            tdata[key].append(t_ns * 1e-9)
        # record_run excludes the camera topics, so this pass stays light.
    out = {}
    t0 = min((tdata[s][0] for s in SCALARS if tdata[s]), default=0.0)
    for s in SCALARS:
        out[s] = np.array(data[s], dtype=float)
        out[s + "_t"] = np.array(tdata[s], dtype=float) - t0
    return out


def _pct(a, q):
    """Return the q-th percentile of array a, or NaN when it is empty."""
    return float(np.percentile(a, q)) if a.size else float("nan")


def _summary(d: dict) -> dict:
    """Reduce the per-scalar telemetry into the summary metrics dict."""
    steer = d["steer"]
    diffs = np.diff(steer) if steer.size > 1 else np.array([0.0])
    # Reversals are sign flips of the steering increment; ignore sub-1e-4 noise.
    nz = diffs[np.abs(diffs) > 1e-4]
    reversals = int(np.sum(np.sign(nz[1:]) != np.sign(nz[:-1]))) if nz.size > 1 else 0
    dur = float(d["steer_t"][-1] - d["steer_t"][0]) if d["steer_t"].size > 1 else 0.0
    spd, tgt = d["speed_kmh"], d["target_speed_kmh"]
    cte, head, solve = d["cross_track_error"], d["heading_error_deg"], d["solve_time_ms"]
    s = {
        "duration_s": round(dur, 1),
        "speed_kmh": {"mean": round(float(spd.mean()), 1) if spd.size else None,
                       "max": round(float(spd.max()), 1) if spd.size else None,
                       "final": round(float(spd[-1]), 1) if spd.size else None},
        "target_speed_kmh_mean": round(float(tgt.mean()), 1) if tgt.size else None,
        "cross_track_m": {"mean": round(float(np.abs(cte).mean()), 3) if cte.size else None,
                           "p95": round(_pct(np.abs(cte), 95), 3) if cte.size else None,
                           "max": round(float(np.abs(cte).max()), 3) if cte.size else None},
        "heading_err_deg": {"mean_abs": round(float(np.abs(head).mean()), 2) if head.size else None,
                             "max_abs": round(float(np.abs(head).max()), 2) if head.size else None},
        "steer": {"mean_abs": round(float(np.abs(steer).mean()), 3) if steer.size else None,
                   "jitter_rms": round(float(np.sqrt(np.mean(diffs**2))), 4),
                   "reversals_per_s": round(reversals / dur, 2) if dur > 0 else None},
        "solve_ms": ({"p50": round(_pct(solve, 50), 2), "p99": round(_pct(solve, 99), 2),
                      "max": round(float(solve.max()), 2)} if solve.size else None),
    }
    return s


def _plot(d: dict, out_png: Path):
    """Render speed, steering, cross-track, and solve-time traces to out_png."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(d["speed_kmh_t"], d["speed_kmh"], label="speed", color="#00b050")
    ax[0].plot(d["target_speed_kmh_t"], d["target_speed_kmh"], label="target", color="#ffab00")
    ax[0].set_ylabel("km/h"); ax[0].legend(loc="upper right"); ax[0].set_title("Speed")
    ax[1].plot(d["steer_t"], d["steer"], color="#1f77b4")
    ax[1].set_ylabel("steer [-1,1]"); ax[1].set_title("Steering (look for micro left/right chatter)")
    ax[2].plot(d["cross_track_error_t"], d["cross_track_error"], color="#d62728")
    ax[2].set_ylabel("cross-track [m]"); ax[2].set_title("Cross-track error")
    if d["solve_time_ms"].size:
        ax[3].plot(d["solve_time_ms_t"], d["solve_time_ms"], color="#17becf")
    ax[3].set_ylabel("solve [ms]"); ax[3].set_title("MPC solve time"); ax[3].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out_png, dpi=110); plt.close(fig)


def main() -> int:
    """Parse arguments, summarize the run, and write summary.json (and optional plot)."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="run directory (runs/<name>) or a bag directory")
    ap.add_argument("--plot", action="store_true", help="also write review.png")
    args, _ = ap.parse_known_args()

    run_path = Path(args.run).expanduser().resolve()
    bag_dir = _find_bag(run_path)
    out_dir = bag_dir.parent
    d = _read_scalars(bag_dir)
    if all(d[s].size == 0 for s in SCALARS):
        print(f"[review_run] no /controller/* scalars in {bag_dir} -- was the controller running?",
              file=sys.stderr)
        return 1
    summary = _summary(d)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"[review_run] {out_dir.name}")
    print(json.dumps(summary, indent=2))
    if args.plot:
        png = out_dir / "review.png"
        _plot(d, png)
        print(f"[review_run] wrote {png}")
    print(f"[review_run] summary -> {out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
