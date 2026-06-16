"""Trace and measure a candidate waypoint route through CARLA.

Run the route through the same pipeline control_node uses
(``RoutePlanner.file_route``), then measure its geometry -- length, turning,
junctions, lane changes, curve radius, loop-back -- so scenario routes can be
designed empirically instead of by eye::

    ros2 run carla_mpc_bringup route_lab --json design.json
    ros2 run carla_mpc_bringup route_lab --route config/routes/sharp_turns.xml

Reports length, turn events (>=45 deg maneuvers) and total turning, junctions
entered, lane changes, whether the route loops back near the start, and min
curve radius, and writes ``logs/route_lab/<name>.png`` showing the traced route
over the road network. Loading the town changes the server map.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import carla
import numpy as np

from carla_mpc_bringup.sim.route_loader import Route, load_route
from carla_mpc_bringup.sim.route_planner import RoutePlanner


def _turn_events(xy, straight_deg=4.0, turn_deg=45.0):
    """Summarize turning along a polyline.

    Parameters
    ----------
    xy : ndarray, shape (N, 2)
        Route points in CARLA world frame (m).
    straight_deg : float
        Heading change below which a step counts as straight and closes any
        open turn.
    turn_deg : float
        Accumulated heading change at which a maneuver counts as one turn event.

    Returns
    -------
    tuple
        (number of maneuvers >=45 deg, total absolute turning in deg, max
        single-step heading change in deg).

    Notes
    -----
    Near-duplicate segments are dropped so a zero-length step's undefined
    heading cannot fake or hide a turn.
    """
    d = np.diff(xy, axis=0)
    d = d[np.hypot(d[:, 0], d[:, 1]) > 0.5]
    if len(d) < 2:
        return 0, 0.0, 0.0
    ang = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    dpsi = (np.diff(ang) + 180.0) % 360.0 - 180.0
    events, acc = 0, 0.0
    for x in dpsi:
        if abs(x) < straight_deg:                       # a straight closes any open turn
            if abs(acc) >= turn_deg:
                events += 1
            acc = 0.0
        else:
            if acc != 0.0 and (x > 0) != (acc > 0):     # direction reversal closes the previous turn
                if abs(acc) >= turn_deg:
                    events += 1
                acc = 0.0
            acc += x
    if abs(acc) >= turn_deg:
        events += 1
    return events, float(np.sum(np.abs(dpsi))), float(np.max(np.abs(dpsi)))


def _lane_changes(route):
    """Count lane-change transitions in a traced route.

    A transition is the single step where the lane shifts: an adjacent
    ``lane_id`` step on the same road and travel direction, or an explicit
    CHANGELANE road option.
    """
    try:
        from agents.navigation.local_planner import RoadOption
        change = {RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT}
    except Exception:                                   # noqa: BLE001
        change = set()
    n = 0
    for k in range(1, len(route)):
        wa, wb, opt = route[k - 1][0], route[k][0], route[k][1]
        if opt in change:
            n += 1
        elif (wa.road_id == wb.road_id and wa.lane_id * wb.lane_id > 0
              and abs(wa.lane_id - wb.lane_id) == 1 and not wa.is_junction and not wb.is_junction):
            n += 1
    return n


def _junctions_entered(route):
    """Count distinct junctions the route enters (rising edges of is_junction)."""
    n, prev = 0, False
    for wp, _ in route:
        j = wp.is_junction
        if j and not prev:
            n += 1
        prev = j
    return n


def _min_radius(xy, step=2.0):
    """Return the tightest curve radius (m) along the route, inf if straight."""
    if len(xy) < 3:
        return float("inf")
    d = np.diff(xy, axis=0)
    ang = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    dpsi = np.abs(np.diff(ang))
    seg = np.hypot(d[1:, 0], d[1:, 1])
    with np.errstate(divide="ignore"):
        r = np.where(dpsi > 1e-3, seg / dpsi, np.inf)
    return float(np.min(r)) if r.size else float("inf")


def _plot(town, road_xy, xy, route, out_png):
    """Render the traced route over the road network and save it to out_png.

    The route polyline is colored by arc length from the start; the axes are
    flipped so CARLA's y-down world frame renders with north up.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(13, 13))
    ax.scatter([p[0] for p in road_xy], [p[1] for p in road_xy], s=1, c="#5a6472", alpha=0.4, linewidths=0)
    dist = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    pts = xy.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", lw=2.6, zorder=4)
    lc.set_array(dist[:-1])
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, label="distance from start (m)", shrink=0.6)
    ax.scatter([xy[0, 0]], [xy[0, 1]], s=80, c="#00e676", zorder=6, label="start")
    ax.scatter([xy[-1, 0]], [xy[-1, 1]], s=80, c="#ff5252", zorder=6, label="end")
    ax.set_aspect("equal"); ax.invert_yaxis(); ax.legend(); ax.grid(True, alpha=0.2)   # CARLA y is down; flip so north is up
    ax.set_title(f"{town}: traced route ({len(route)} lane-wps)")
    fig.tight_layout(); fig.savefig(out_png, dpi=120); plt.close(fig)


def main() -> int:
    """Parse CLI args, trace the route in CARLA, print metrics, and write artifacts."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default=None, help="design json: {town, waypoints:[{x,y,z}]}")
    ap.add_argument("--route", default=None, help="route XML (Leaderboard format)")
    ap.add_argument("--town", default=None, help="town (required with --spawns)")
    ap.add_argument("--spawns", default=None,
                    help="comma-separated spawn-point indices (from map_atlas) -> route through them via the GRP")
    ap.add_argument("--save", default=None, help="also write the route to config/routes/<save>.xml")
    ap.add_argument("--name", default=None, help="output name (default: derived)")
    ap.add_argument("--out", default="logs/route_lab")
    ap.add_argument("--host", default="localhost"); ap.add_argument("--port", default=2000, type=int)
    args = ap.parse_args()

    spawn_idx = None
    if args.spawns:
        if not args.town:
            ap.error("--spawns needs --town")
        town, wps, name = args.town, None, args.name or args.save or "spawns_route"
        spawn_idx = [int(s) for s in args.spawns.replace(" ", "").split(",") if s]
    elif args.json:
        d = json.loads(Path(args.json).read_text())
        town = d["town"]
        wps = [(float(p["x"]), float(p["y"]), float(p.get("z", 0.3))) for p in d["waypoints"]]
        name = args.name or Path(args.json).stem
    elif args.route:
        r = load_route(args.route)
        town, wps, name = r.town, r.waypoints, args.name or Path(args.route).stem
    else:
        ap.error("need --json, --route, or --town+--spawns")

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port); client.set_timeout(120.0)
    print(f"[{name}] loading {town} ...", flush=True)
    world = client.load_world(town)
    if spawn_idx is not None:                           # resolve spawn-point indices on the loaded map
        sp = world.get_map().get_spawn_points()
        wps = [(sp[i].location.x, sp[i].location.y, max(0.3, sp[i].location.z)) for i in spawn_idx]
        print(f"  spawns {spawn_idx} -> {len(wps)} waypoints (GRP fills between them)", flush=True)
    planner = RoutePlanner(world, resolution=2.0)
    route = planner.file_route(Route(id=name, town=town, waypoints=wps))
    xy = np.array([[wp.transform.location.x, wp.transform.location.y] for wp, _ in route])

    length = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    n_turns, total_turn, max_step = _turn_events(xy)
    start = xy[0]
    tail = xy[int(len(xy) * 0.6):]
    loop_gap = float(np.min(np.linalg.norm(tail - start, axis=1))) if len(tail) else 9e9
    road_xy = [(w.transform.location.x, w.transform.location.y)
               for w in world.get_map().generate_waypoints(4.0)]
    _plot(town, road_xy, xy, route, out / f"{name}.png")

    print(f"\n=== {name} ({town}) ===")
    print(f"  length            : {length:6.1f} m   ({len(route)} lane waypoints)")
    print(f"  sharp turns (>=45): {n_turns:3d}        total turning {total_turn:6.0f} deg, max step {max_step:.0f}")
    print(f"  junctions entered : {_junctions_entered(route):3d}")
    print(f"  lane changes      : {_lane_changes(route):3d}")
    print(f"  min curve radius  : {_min_radius(xy):5.1f} m")
    print(f"  loops back to start: {'YES' if loop_gap < 30 else 'no'} (closest tail->start {loop_gap:.1f} m)")
    print(f"  plot -> {out / (name + '.png')}")

    if len(wps) > 2:                                   # per-leg breakdown: which leg detours / cusps
        print("  per-leg (anchor -> anchor : trace / straight = detour):")
        tags = spawn_idx if spawn_idx is not None else list(range(len(wps)))
        for k in range(len(wps) - 1):
            seg = planner.file_route(Route(id="leg", town=town, waypoints=[wps[k], wps[k + 1]]))
            sxy = np.array([[w.transform.location.x, w.transform.location.y] for w, _ in seg])
            sl = float(np.hypot(wps[k + 1][0] - wps[k][0], wps[k + 1][1] - wps[k][1]))
            tl = float(np.sum(np.linalg.norm(np.diff(sxy, axis=0), axis=1))) if len(sxy) > 1 else 0.0
            ms = _turn_events(sxy)[2]
            flag = "  <== CUSP (U-turn)" if ms >= 170 else ("  <== detour" if tl > 3 * max(sl, 1.0) else "")
            print(f"     {tags[k]:>4} -> {tags[k + 1]:<4}: {tl:6.1f} m / {sl:5.1f} m  x{tl / max(sl, 1.0):.1f}{flag}")

    if args.save:                                       # write the route XML (the user wires it into scenarios.yaml)
        xml = Path("src/carla_mpc_bringup/config/routes") / f"{args.save}.xml"
        ids = spawn_idx if spawn_idx is not None else [None] * len(wps)
        hdr = (f"{town} spawn-point indices (in order): {','.join(str(i) for i in ids)}"
               if spawn_idx is not None else f"{town} explicit waypoints")
        L = ['<?xml version="1.0" encoding="UTF-8"?>',
             f'<!-- {args.save}: {hdr}',
             '     EDIT BY THESE NUMBERS (atlas spawn indices), not the raw coords below. GRP-traced.',
             f'     Wire with:  route: routes/{args.save}.xml  in config/scenarios.yaml -->',
             '<routes>', f'    <route id="{args.save}" town="{town}">', '        <weathers>',
             '            <weather route_percentage="0" cloudiness="10" sun_altitude_angle="60"/>',
             '        </weathers>', '        <waypoints>']
        for (x, y, z), i in zip(wps, ids):
            tag = f'  <!-- spawn {i} -->' if i is not None else ''
            # Keep full precision: rounding to cm can snap a point onto a different
            # lane at junctions, so the loaded route traces a different (longer) path.
            L.append(f'            <position x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>{tag}')
        L += ['        </waypoints>', '        <scenarios/>', '    </route>', '</routes>']
        xml.write_text("\n".join(L) + "\n")
        print(f"  saved route -> {xml}  (spawns: {args.spawns})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
