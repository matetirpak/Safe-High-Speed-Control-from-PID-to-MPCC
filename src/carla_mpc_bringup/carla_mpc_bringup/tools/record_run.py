"""Record and archive a run as a timestamped rosbag2 plus a reproducibility manifest.

Saves whatever is running so it can be reviewed later (debug a controller without
re-driving, or compare PID vs MPC). Each run is written to:

    <record-dir>/<timestamp>_<map>_<controller>[_<condition>]/
        bag/            rosbag2 -- control telemetry (/controller/*), ground truth
                        (/carla/hero/gt/*), /tf, /clock, and the ego model.
                        With --inputs, also the camera streams (larger).
        manifest.yaml   timestamp, controller, map/route, condition, host, ROS distro,
                        git (if any), what was recorded.
        config/         snapshot of vehicle/carla/controller/conditions YAML, so the
                        run stays self-contained even if those files are later edited.

Usable from the launch (record:=true) or standalone:
    ros2 run carla_mpc_bringup record_run --controller mpc --condition rainy_night

Review a saved run with:
    ros2 run carla_mpc_bringup review_run runs/<name>

Write the manifest and config snapshot, then run ros2 bag record as a child,
forwarding Ctrl-C / launch-shutdown signals so the bag always finalizes cleanly.
"""
from __future__ import annotations

import argparse
import datetime
import os
import shutil
import signal
import socket
import subprocess
import sys
from pathlib import Path

# Always dropped (noise). Heavy sensor streams are additionally dropped unless --inputs.
_DROP_ALWAYS = "rosout|parameter_events"
_DROP_SENSORS = "image|camera_info"


def _default_config(name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory("carla_mpc_bringup"))
    except Exception:
        base = Path(__file__).resolve().parent.parent
    return str(base / "config" / name)


def _ws_root() -> Path:
    return Path(os.environ.get("MPC_WS", os.getcwd()))


def _git(ws: Path, *args: str) -> str:
    try:
        r = subprocess.run(["git", "-C", str(ws), *args],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def _scenario(carla_cfg: str) -> str:
    """Derive a run name from the route file (if any) or the map in carla.yaml."""
    try:
        import yaml
        d = yaml.safe_load(open(carla_cfg)) or {}
        route = d.get("route") or {}
        if str(route.get("mode", "")) == "file" and route.get("file"):
            return Path(route["file"]).stem
        return str((d.get("world") or {}).get("map", "run"))
    except Exception:
        return "run"


def _snapshot_configs(dst: Path, *explicit: str) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    share_cfg = Path(_default_config("vehicle.yaml")).parent
    if share_cfg.is_dir():
        shutil.copytree(share_cfg, dst, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("__pycache__"))
    for f in explicit:
        try:
            if f and Path(f).is_file():
                shutil.copy2(f, dst / Path(f).name)
        except Exception:
            pass


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--record-dir",
                   default=os.environ.get("MPC_RUNS_DIR", str(_ws_root() / "runs")),
                   help="where run folders are saved (default: ./runs or $MPC_RUNS_DIR)")
    p.add_argument("--controller", default="", help="controller name (folder + manifest)")
    p.add_argument("--inputs", action="store_true",
                   help="ALSO record camera streams (larger bags)")
    p.add_argument("--condition", default="", help="weather condition (folder + manifest)")
    p.add_argument("--scenario", default="", help="override the auto scenario name")
    p.add_argument("--note", default="", help="free-text note stored in the manifest")
    p.add_argument("--name", default="", help="override the whole run-folder name")
    p.add_argument("--compress", action="store_true", help="zstd file compression")
    p.add_argument("--vehicle-config", default=_default_config("vehicle.yaml"))
    p.add_argument("--carla-config", default=_default_config("carla.yaml"))
    p.add_argument("--controller-config", default=_default_config("controller.yaml"))
    args, _ = p.parse_known_args()

    ws = _ws_root()
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    scenario = args.scenario or _scenario(args.carla_config)
    controller = args.controller or "ctrl"
    parts = [ts, scenario, controller] + ([args.condition] if args.condition else [])
    name = args.name or "_".join(parts)
    run_dir = Path(args.record_dir).expanduser() / name
    run_dir.mkdir(parents=True, exist_ok=True)

    _snapshot_configs(run_dir / "config", args.vehicle_config, args.carla_config,
                      args.controller_config)

    commit = _git(ws, "rev-parse", "--short", "HEAD")
    git_info = "not-a-git-repo" if not commit else {
        "commit": commit,
        "branch": _git(ws, "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git(ws, "status", "--porcelain")),
    }
    exclude = _DROP_ALWAYS if args.inputs else f"{_DROP_ALWAYS}|{_DROP_SENSORS}"
    manifest = {
        "name": name,
        "timestamp": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "controller": controller,
        "scenario": scenario,
        "condition": args.condition or "default",
        "note": args.note,
        "git": git_info,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "hostname": socket.gethostname(),
        "user": os.environ.get("USER", ""),
        "record": {"inputs": args.inputs, "exclude_regex": exclude, "compressed": args.compress},
        "bag": "bag",
        "config_snapshot": "config",
    }
    try:
        import yaml
        (run_dir / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    except Exception as e:  # never let manifest issues stop the recording
        (run_dir / "manifest.yaml").write_text(f"# manifest dump failed: {e}\n{manifest}\n")

    cmd = ["ros2", "bag", "record", "-a", "-x", exclude, "-o", str(run_dir / "bag")]
    if args.compress:
        cmd += ["--compression-mode", "file", "--compression-format", "zstd"]
    print(f"[record_run] saving run -> {run_dir}", flush=True)
    print(f"[record_run] {'inputs+telemetry' if args.inputs else 'telemetry only'}; "
          f"excluding /{exclude}/", flush=True)
    sys.stdout.flush()

    # Run the recorder as a child and translate ANY stop signal into a clean SIGINT
    # for it: rosbag2 writes metadata.yaml on SIGINT (Ctrl-C) but not on the SIGTERM
    # the launch escalates to on auto-shutdown -- forwarding keeps the bag valid either way.
    proc = subprocess.Popen(cmd)

    def _finalize(_signum, _frame):
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, _finalize)
    signal.signal(signal.SIGTERM, _finalize)
    proc.wait()
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
