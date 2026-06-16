"""Generate clean, feature-hitting scenario routes by walking Carla's lane graph.

Far-apart waypoints make the GlobalRoutePlanner detour onto shortcut lanes, so it can
miss the intended feature. A forward lane walk via Waypoint.next cannot detour: at each
junction it picks the branch that best serves the feature -- most-turning
for sharp_turns, stay-on-circle for the roundabout, straightest for the highway. The walk
is dumped as a dense Leaderboard route XML so control_node's file_route (which snaps
dense waypoints straight to lanes, no re-planning) reproduces it, then verified with
the route_lab metrics.

Run example::

    ros2 run carla_mpc_bringup build_routes --scenario sharp_turns
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import carla
import numpy as np

from carla_mpc_bringup.sim.route_loader import Route
from carla_mpc_bringup.sim.route_planner import RoutePlanner
from carla_mpc_bringup.tools.route_lab import (_junctions_entered, _lane_changes,
                                               _min_radius, _turn_events)

DRIVING = carla.LaneType.Driving


def _hc(a_yaw, b_yaw):
    """Return the signed heading change from a_yaw to b_yaw, wrapped to (-180, 180] deg."""
    return (b_yaw - a_yaw + 180.0) % 360.0 - 180.0


def walk_turns(m, start, target_len, step=2.0):
    """Walk forward from start, taking the most-turning branch at every junction.

    Parameters
    ----------
    m : carla.Map
        Map whose lane graph is walked.
    start : carla.Location
        Approximate start point, projected onto the nearest driving lane.
    target_len : float
        Walk length target in meters.
    step : float
        Forward step between successive waypoints, in meters.

    Returns
    -------
    list of carla.Waypoint
        Waypoints along the walk, starting at the projected start.

    Notes
    -----
    Branches turning more than ~90 deg are kept only if no gentler branch exists,
    rejecting near-U-turn cusps.
    """
    wp = m.get_waypoint(start, project_to_road=True, lane_type=DRIVING)
    pts, total = [wp], 0.0
    while total < target_len:
        nxts = wp.next(step)
        if not nxts:
            break
        cyaw = wp.transform.rotation.yaw
        opts = [(n, abs(_hc(cyaw, n.transform.rotation.yaw))) for n in nxts]
        opts = [o for o in opts if o[1] < 110.0] or opts   # reject near-U-turn cusps unless no gentler branch exists
        nw = max(opts, key=lambda o: o[1])[0]
        total += wp.transform.location.distance(nw.transform.location)
        pts.append(nw)
        wp = nw
    return pts


def find_roundabout(m):
    """Find the junction whose internal lanes form a ring.

    A roundabout junction has its waypoints spread all the way around the center
    (no large angular gap), a sizeable mean radius, low radius variation (a clean
    ring), and many entry/exit pairs.

    Parameters
    ----------
    m : carla.Map
        Map to scan for junctions.

    Returns
    -------
    tuple or None
        ``(1.0, junction, cx, cy, radius, full, npairs)`` for the best ring, where
        ``cx, cy`` is the center in meters, ``radius`` the mean radius in meters,
        ``full`` whether points span the whole circle, and ``npairs`` the entry/exit
        pair count. None if no junctions are found.
    """
    juncs = {}
    for w in m.generate_waypoints(3.0):
        if w.is_junction:
            j = w.get_junction()
            if j is not None and j.id not in juncs:
                juncs[j.id] = j
    cands = []
    for j in juncs.values():
        try:
            pairs = j.get_waypoints(DRIVING)
        except Exception:                              # noqa: BLE001
            continue
        pts = np.array([[w.transform.location.x, w.transform.location.y] for pr in pairs for w in pr])
        if len(pts) < 10:
            continue
        c = pts.mean(0)
        r = np.linalg.norm(pts - c, axis=1)
        mr = float(r.mean())
        if not (8.0 < mr < 45.0):
            continue
        ang = np.sort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
        gap = float(np.max(np.diff(np.concatenate([ang, [ang[0] + 2 * np.pi]]))))
        cv = float(r.std() / mr)                        # radius variation: LOW = a clean ring
        full = gap < math.radians(100.0)               # points span the whole circle
        cands.append((cv, mr, gap, len(pairs), j, float(c[0]), float(c[1]), full))
    if not cands:
        return None
    for c in sorted(cands, key=lambda x: (not x[7], x[0]))[:4]:
        print(f"    cand jct{c[4].id} c=({c[5]:.0f},{c[6]:.0f}) r={c[1]:.1f} cv={c[0]:.2f} "
              f"gap={math.degrees(c[2]):.0f} full={c[7]} pairs={c[3]}")
    rings = [c for c in cands if c[7] and c[0] < 0.30]  # full + tight = a real roundabout
    rings.sort(key=lambda c: (c[0], -c[1]))             # tightest, then biggest
    cv, mr, gap, npairs, j, cx, cy, full = (rings or sorted(cands, key=lambda c: c[0]))[0]
    return (1.0, j, cx, cy, mr, full, npairs)


def walk_roundabout(m, target_len, laps=2):
    """Build a clean N-lap roundabout route via the GlobalRoutePlanner.

    The ring is one-way, so a GRP trace from a spoke's inbound lane to the same
    spoke's outbound lane must go the full 360 deg around. GRP follows the real
    ring lane and produces no cusps, unlike geometric next()-walking which flips
    lanes and exits early. The in-ring arc is then duplicated for `laps` laps; the
    seam coincides at the entry spoke.

    Parameters
    ----------
    m : carla.Map
        Map to search for a roundabout.
    target_len : float
        Nominal route length target in meters (informational).
    laps : int
        Number of laps around the ring.

    Returns
    -------
    list of carla.Waypoint
        Approach, lead-in, N-lap ring, and exit waypoints.

    Raises
    ------
    SystemExit
        If no roundabout is found, or no spoke has both inbound and outbound lanes.
    """
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    info = find_roundabout(m)
    if info is None:
        raise SystemExit("no roundabout found")
    _, j, cx, cy, rad, circular, npairs = info
    print(f"  roundabout jct{j.id} center=({cx:.1f},{cy:.1f}) r={rad:.1f} circular={circular} paths={npairs}")
    center = carla.Location(x=cx, y=cy, z=0.3)

    inbound, outbound = {}, {}                         # spoke road_id -> approach point (~18 m outside)
    for entry, exit in j.get_waypoints(DRIVING):
        pv = entry.previous(18.0)
        if pv:
            inbound.setdefault(pv[0].road_id, pv[0])
        nx = exit.next(18.0)
        if nx:
            outbound.setdefault(nx[0].road_id, nx[0])
    common = [r for r in inbound if r in outbound]
    if not common:
        raise SystemExit("no spoke has both an inbound and outbound lane for a full lap")
    rd = common[0]
    grp = GlobalRoutePlanner(m, 2.0)
    a, b = inbound[rd].transform.location, outbound[rd].transform.location
    lap = [w for w, _ in grp.trace_route(a, b)]        # full 360 deg lap (one-way ring forces it around)
    lead = inbound[rd].previous(45.0)                  # extra lead-in so the ego is settled before the ring
    approach = [w for w, _ in grp.trace_route(lead[0].transform.location, a)] if lead else []

    in_ring = [w.transform.location.distance(center) < rad * 1.5 for w in lap]
    if True in in_ring:
        fr = in_ring.index(True)
        lr = len(in_ring) - 1 - in_ring[::-1].index(True)
    else:
        fr, lr = 0, len(lap) - 1
    pre, ring, post = lap[:fr], lap[fr:lr + 1], lap[lr + 1:]
    pts = approach + pre + ring * max(1, int(laps)) + post
    print(f"  GRP lap via spoke road {rd}: {len(lap)} wp ({len(ring)} in-ring) -> {int(laps)} laps, {len(pts)} pts")
    return pts


def walk_highway(m, start, target_len, exit_frac=0.30):
    """Walk a highway, exiting via an off-ramp then re-entering via an on-ramp.

    Town04's highway is elevated, so an off-ramp is a branch that turns off and
    descends and an on-ramp one that ascends; otherwise the straightest (mainline)
    branch is taken. The exit is attempted only after `exit_frac` of the target
    length, and re-entry only after a surface stretch of at least ~70 m.

    Parameters
    ----------
    m : carla.Map
        Map whose lane graph is walked.
    start : carla.Location
        Approximate start point, projected onto the nearest driving lane.
    target_len : float
        Walk length target in meters.
    exit_frac : float
        Fraction of target_len to cruise the mainline before seeking an off-ramp.

    Returns
    -------
    list of carla.Waypoint
        Waypoints along the walk.
    """
    wp = m.get_waypoint(start, project_to_road=True, lane_type=DRIVING)
    pts, total, phase, surf = [wp], 0.0, "cruise1", 0.0
    while total < target_len:
        nx = wp.next(2.0)
        if not nx:
            break
        cz = wp.transform.location.z
        cyaw = wp.transform.rotation.yaw
        straight = min(nx, key=lambda n: abs(_hc(cyaw, n.transform.rotation.yaw)))
        nw = straight
        if phase == "cruise1" and total > target_len * exit_frac and len(nx) > 1:
            ramps = [n for n in nx if n.transform.location.z < cz - 0.15
                     and abs(_hc(cyaw, n.transform.rotation.yaw)) > 6]
            if ramps:
                nw, phase = min(ramps, key=lambda n: n.transform.location.z), "surface"
        elif phase == "surface":
            if surf > 70.0 and len(nx) > 1:
                ramps = [n for n in nx if n.transform.location.z > cz + 0.15
                         and abs(_hc(cyaw, n.transform.rotation.yaw)) > 6]
                if ramps:
                    nw, phase = max(ramps, key=lambda n: n.transform.location.z), "cruise2"
        d = wp.transform.location.distance(nw.transform.location)
        total += d
        if phase == "surface":
            surf += d
        pts.append(nw)
        wp = nw
    print(f"  highway phases ended in '{phase}' (surface stretch {surf:.0f} m)")
    return pts


def _sample(pts, every_m=3.0):
    """Down-sample a dense walk to ~every_m spacing, returning (x, y, z) tuples in meters.

    file_route later snaps these dense points back onto the lane graph, so the
    spacing only needs to be fine enough to pin the intended path.
    """
    out = [pts[0]]
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        acc += a.transform.location.distance(b.transform.location)
        if acc >= every_m:
            out.append(b)
            acc = 0.0
    if out[-1] is not pts[-1]:
        out.append(pts[-1])
    return [(w.transform.location.x, w.transform.location.y, max(0.3, w.transform.location.z))
            for w in out]


def write_xml(path, route_id, town, waypoints):
    """Write the sampled (x, y, z) waypoints as a Leaderboard route XML at path."""
    L = ['<?xml version="1.0" encoding="UTF-8"?>',
         f'<!-- Auto-generated by build_routes (lane-graph walk). Feature route for {route_id}. -->',
         '<routes>', f'    <route id="{route_id}" town="{town}">',
         '        <weathers>',
         '            <weather route_percentage="0" cloudiness="10" sun_altitude_angle="60"/>',
         '        </weathers>', '        <waypoints>']
    L += [f'            <position x="{x:.2f}" y="{y:.2f}" z="{z:.2f}"/>' for x, y, z in waypoints]
    L += ['        </waypoints>', '        <scenarios/>', '    </route>', '</routes>']
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(L) + "\n")


def measure(planner, town, waypoints, name):
    """Re-trace the route through file_route and print route_lab metrics.

    Parameters
    ----------
    planner : RoutePlanner
        Planner used to reproduce the route from the sampled waypoints.
    town : str
        Carla town name.
    waypoints : list of tuple
        Sampled (x, y, z) waypoints in meters.
    name : str
        Route id, used for the Route and the printed label.

    Returns
    -------
    route : list
        The traced route (waypoint, road-option) pairs.
    xy : numpy.ndarray
        Traced (x, y) positions in meters, shape (N, 2).
    """
    route = planner.file_route(Route(id=name, town=town, waypoints=waypoints))
    xy = np.array([[w.transform.location.x, w.transform.location.y] for w, _ in route])
    length = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    nt, tot, mx = _turn_events(xy)
    print(f"  TRACED: {length:6.1f} m  turns(>=45)={nt}  total_turn={tot:.0f}  max_step={mx:.0f}  "
          f"junctions={_junctions_entered(route)}  lane_changes={_lane_changes(route)}  "
          f"min_R={_min_radius(xy):.1f}")
    return route, xy


SCEN = {  # scenario -> (town, start_xy or None, target_len)
    "sharp_turns": ("Town07", (-87.5, -65.4), 1000.0),
    "roundabout": ("Town03", None, 340.0),
    "highway": ("Town04", (386.0, -228.0), 1000.0),
}


def main() -> int:
    """Walk, write, and verify the route XML for the requested scenario."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True, choices=list(SCEN))
    ap.add_argument("--out-dir", default="src/carla_mpc_bringup/config/routes")
    ap.add_argument("--host", default="localhost"); ap.add_argument("--port", default=2000, type=int)
    args = ap.parse_args()

    town, start_xy, tlen = SCEN[args.scenario]
    client = carla.Client(args.host, args.port); client.set_timeout(120.0)
    print(f"[{args.scenario}] loading {town} ...", flush=True)
    world = client.load_world(town)
    m = world.get_map()

    if args.scenario == "sharp_turns":
        walk = walk_turns(m, carla.Location(x=start_xy[0], y=start_xy[1], z=0.3), tlen)
    elif args.scenario == "roundabout":
        walk = walk_roundabout(m, tlen)
    elif args.scenario == "highway":
        walk = walk_highway(m, carla.Location(x=start_xy[0], y=start_xy[1], z=0.3), tlen)
    else:
        raise SystemExit("not yet implemented")

    wps = _sample(walk)
    print(f"  walk: {len(walk)} steps -> {len(wps)} sampled waypoints")
    out = Path(args.out_dir) / f"{args.scenario}.xml"
    write_xml(out, args.scenario, town, wps)
    planner = RoutePlanner(world, resolution=2.0)
    measure(planner, town, wps, args.scenario)
    print(f"  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
