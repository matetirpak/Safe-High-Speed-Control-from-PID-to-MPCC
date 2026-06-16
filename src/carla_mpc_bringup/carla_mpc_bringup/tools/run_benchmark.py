"""Run a replicable controller benchmark and aggregate it into a comparison report.

Each controller drives the same bounded scenarios so they can be compared fairly.
Requires CARLA running with `--ros2`; this drives `control_node` directly (the run
log is written by control_node itself, so no Foxglove/ground-truth is needed).

    # default: pid/mpc/mpcc on Town05 seed 42 spawn 0, 800 m each
    ros2 run carla_mpc_bringup run_benchmark
    ros2 run carla_mpc_bringup run_benchmark --controllers mpc,mpcc \
        --seeds 42,7 --spawns 0 --distance 1000 --map Town05 --repeats 1

Every (scenario x controller x repeat) is one `control_node` run that auto-stops and
writes logs/runs/<run>/. Runs share the same seeded road, so the report compares speed
at matched curvature. The final step calls benchmark_report on exactly the runs produced.
"""
from __future__ import annotations

import argparse
import glob
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import yaml


def _newest_log(before: set[str]) -> str | None:
    """Return the newest logs/runs/ dir with a metrics.json not present in ``before``."""
    after = set(glob.glob("logs/runs/*/"))
    new = [d for d in (after - before) if (Path(d) / "metrics.json").exists()]
    return max(new, key=lambda d: Path(d).stat().st_mtime) if new else None


def _carla_up(host: str = "localhost", port: int | None = None, timeout: float = 2.0) -> bool:
    """True if CARLA's RPC port accepts a TCP connection.

    A pure-socket probe (no carla wheel needed), matching carla_auto_reboot.sh's check, so
    the benchmark can tell a live server from one that has segfaulted (the known
    CARLA-0.9.16-on-Blackwell PhysX crash) and is being relaunched.
    """
    port = port or int(os.environ.get("CARLA_PORT", "2000"))
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def _wait_for_carla(max_wait: float, poll: float = 3.0) -> bool:
    """Block until CARLA's RPC port responds, up to ``max_wait`` s. Returns True if up.

    Instant when the server is already up (the normal case). When it is down — e.g. after a
    crash, while carla_auto_reboot.sh relaunches it — this lets the suite pause and resume
    rather than burning the remaining runs against a dead server. ``max_wait <= 0`` disables
    the gate (probe once and return immediately).
    """
    if _carla_up() or max_wait <= 0:
        return _carla_up()
    print(f"  ... CARLA down; waiting up to {max_wait:.0f}s for it to come back (auto-reboot?)", flush=True)
    waited = 0.0
    while waited < max_wait:
        time.sleep(poll)
        waited += poll
        if _carla_up():
            print(f"  ... CARLA back up after {waited:.0f}s", flush=True)
            return True
    print(f"  !! CARLA still down after {max_wait:.0f}s", file=sys.stderr)
    return False


def _run_one(cmd: list[str], timeout: float) -> str | None:
    """Run one control_node drive and return its new run dir (with metrics.json), or None.

    Snapshots logs/runs/ before launching, runs the subprocess under a hard wall-clock cap,
    then polls briefly for the run dir to finalize (metrics.json is written just before
    control_node exits, but the dir can settle a moment after the process returns).
    """
    before = set(glob.glob("logs/runs/*/"))
    try:
        subprocess.run(cmd, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        print(f"  (timeout {timeout:.0f}s -- killed; log still written on exit)")
    d = None
    for _ in range(10):
        d = _newest_log(before)
        if d:
            break
        time.sleep(1.0)
    return d


def _load_scenarios(path: str, only_groups: list[str] | None = None) -> list[dict]:
    """Expand a scenario suite (scenarios.yaml) into a list of benchmark jobs.

    Each scenario is a deterministic bounded drive (map + spawn + seed) curated to
    traverse a road feature; its `condition` is the scenario name, which tags the run
    so the report groups by it. Every top-level YAML list is a scenario group named by
    its key, aggregated separately in the report. `only_groups` restricts the run to
    those groups (default: all); an unknown group name aborts before any run.

    Parameters
    ----------
    path : str
        Path to the scenarios.yaml suite.
    only_groups : list of str, optional
        Top-level group keys to run; ``None`` runs every group.

    Returns
    -------
    list of dict
        One job per scenario, with map/seed/spawn/distance/route/condition fields.

    Raises
    ------
    SystemExit
        If a requested group is unknown, or no scenarios are found.
    """
    raw = yaml.safe_load(Path(path).read_text()) or {}
    declared = [k for k, v in raw.items() if isinstance(v, list)]
    if only_groups:
        unknown = [g for g in only_groups if g not in declared]
        if unknown:
            raise SystemExit(f"Unknown scenario group(s): {', '.join(unknown)}. "
                             f"Available in {path}: {', '.join(declared) or '(none)'}")
    jobs = []
    for group, scs in raw.items():
        if not isinstance(scs, list) or (only_groups and group not in only_groups):
            continue
        for sc in scs:
            if not isinstance(sc, dict) or "name" not in sc:
                continue
            jobs.append({"group": group, "condition": sc["name"], "map": sc.get("map", "Town05"),
                         "seed": int(sc.get("seed", 42)), "spawn": int(sc.get("spawn", 0)),
                         "distance": float(sc.get("distance", 0.0 if sc.get("route") else 800.0)),
                         "route": sc.get("route", ""), "route_id": sc.get("route_id", "")})
    if not jobs:
        raise SystemExit(f"No scenarios found in {path}")
    return jobs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controllers", default="pid,mpc,mpcc",
                    help="comma list; append @<kmh> for an mpc/mpcc speed-cap VARIANT, e.g. "
                         "pid,mpc,mpcc,mpc@130,mpcc@130 (capped runs show as their own report column)")
    ap.add_argument("--scenarios", default=None,
                    help="scenario suite YAML (each = map+spawn+seed+distance, tagged by name). "
                         "Overrides --seeds/--spawns/--map; --repeats still applies for stats.")
    ap.add_argument("--groups", default="",
                    help="comma list of scenario GROUPS (scenarios.yaml top-level keys) to run; "
                         "default = all. An unknown name aborts before any run.")
    ap.add_argument("--seeds", default="42")
    ap.add_argument("--spawns", default="0")
    ap.add_argument("--map", default="Town05")
    ap.add_argument("--condition", default="")
    ap.add_argument("--distance", default=800.0, type=float, help="max distance per run (m)")
    ap.add_argument("--repeats", default=1, type=int)
    ap.add_argument("--timeout", default=300.0, type=float, help="hard wall-clock cap per run (s)")
    ap.add_argument("--carla-wait", dest="carla_wait", default=180.0, type=float,
                    help="if CARLA's RPC port is down (e.g. it crashed and is being relaunched by "
                         "carla_auto_reboot.sh), wait up to this many seconds for it to return before "
                         "each run, and retry a run once after recovery. 0 disables the gate.")
    ap.add_argument("--out", default=None, help="report dir (default: logs/benchmarks/<timestamp>)")
    ap.add_argument("--report-only", action="store_true", help="skip running; just report logs/runs/*")
    ap.add_argument("--record-camera", action="store_true",
                    help="save each run's driver-view camera.mp4 (forces rendering ON; slower + more storage)")
    args, _ = ap.parse_known_args()

    controllers = [c.strip() for c in args.controllers.split(",") if c.strip()]
    produced: list[str] = []

    # A job is one bounded drive (condition tag + map + seed + spawn + distance), from
    # either a curated scenario suite (--scenarios) or a seeds-x-spawns grid on one map.
    if args.scenarios:
        only_groups = [g.strip() for g in args.groups.split(",") if g.strip()]
        jobs = _load_scenarios(args.scenarios, only_groups or None)
    else:
        seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
        spawns = [int(s) for s in args.spawns.split(",") if s.strip()]
        jobs = [{"condition": args.condition, "map": args.map, "seed": s, "spawn": p,
                 "distance": args.distance} for s in seeds for p in spawns]

    if not args.report_only:
        total = len(jobs) * len(controllers) * args.repeats
        i = 0
        for rep in range(args.repeats):
            for job in jobs:
                name = job["condition"] or f"seed{job['seed']}_spawn{job['spawn']}"
                for ctrl in controllers:       # ctrl may carry an "@<kmh>" speed cap; control_node parses it
                    i += 1
                    span = (f"route={Path(job['route']).stem}" if job.get("route")
                            else f"dist={job['distance']:.0f}m")
                    print(f"\n=== [{i}/{total}] {ctrl}  {name} map={job['map']} {span} rep={rep} ===",
                          flush=True)
                    cmd = ["ros2", "run", "carla_mpc_bringup", "control_node",
                           "--controller", ctrl, "--map", job["map"],
                           "--seed", str(job["seed"]), "--spawn-index", str(job["spawn"]),
                           "--max-distance", str(job["distance"]),
                           "--no-rendering-mode", "on"]
                    if job.get("group"):        # group tag so the report aggregates per scenario group
                        cmd += ["--scenario-group", job["group"]]
                    if args.scenarios:          # curated routes are deterministic; no random extension
                        cmd += ["--route-extend", "false"]
                    if job.get("route"):        # explicit waypoint route XML overrides spawn+seed
                        cmd += ["--route", job["route"]]
                        if job.get("route_id"):
                            cmd += ["--route-id", job["route_id"]]
                    if job["condition"]:
                        cmd += ["--condition", job["condition"]]
                    if args.record_camera:      # control_node forces rendering on for the camera
                        cmd += ["--record-camera"]
                    # Don't launch into a dead/rebooting server: after a CARLA crash this blocks until
                    # the RPC port returns (carla_auto_reboot.sh relaunches it), so the suite resumes
                    # instead of failing every remaining run. No-op when CARLA is already up.
                    _wait_for_carla(args.carla_wait)
                    d = _run_one(cmd, args.timeout)
                    if d is None and args.carla_wait > 0 and _wait_for_carla(args.carla_wait):
                        # No log usually means CARLA crashed before control_node could save one. It's
                        # back now -> retry once so this controller isn't missing from the scenario.
                        print("  retrying run once after CARLA recovery ...", flush=True)
                        d = _run_one(cmd, args.timeout)
                    if d:
                        produced.append(d)
                        print(f"  -> {d}")
                    else:
                        print("  !! no log produced (CARLA up? control_node errored?)", file=sys.stderr)

    # Co-locate the per-run logs inside the benchmark folder so each report is self-contained.
    # Only the runs this invocation produced are relocated; pre-existing logs/runs/* are left
    # untouched.
    if args.out and produced:
        import shutil
        runs_dir = Path(args.out) / "runs"
        runs_dir.mkdir(parents=True, exist_ok=True)
        relocated = []
        for d in produced:
            try:
                dst = runs_dir / Path(d.rstrip("/")).name
                shutil.move(d.rstrip("/"), str(dst))
                relocated.append(str(dst))
            except Exception as e:  # noqa: BLE001
                print(f"  (could not relocate {d}: {e})", file=sys.stderr)
                relocated.append(d)
        produced = relocated
        print(f"  runs co-located in {runs_dir}")

    # Aggregate into a timestamped report (the runs just made, or all of logs/runs/*).
    # Pass --controllers so the report's tables and plots use the exact order entered on the command
    # line (here it only orders -- the produced runs are already this controller set). Omitted for
    # --report-only so that mode keeps reporting every controller found in logs/runs/*.
    from carla_mpc_bringup.tools.benchmark_report import main as report_main
    argv = (["benchmark_report"]
            + (["--out", args.out] if args.out else [])
            + ([] if args.report_only else ["--controllers", args.controllers])
            + (produced or []))
    sys.argv = argv
    print(f"\n=== aggregating {len(produced) or 'all logs/runs/'} runs into a report ===")
    return report_main()


if __name__ == "__main__":
    raise SystemExit(main())
