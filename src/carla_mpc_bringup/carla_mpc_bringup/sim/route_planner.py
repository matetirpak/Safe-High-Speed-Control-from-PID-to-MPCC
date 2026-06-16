"""Generate routes for the controller to track.

Two sources, selected by ``carla.yaml route.mode``:

* "random" -- CARLA's GlobalRoutePlanner traces a lane-level route from the
  ego's current waypoint to a random reachable goal (a map spawn point). The
  route is continuously extended well before the end is reached, so the car
  flows through targets instead of decelerating to a stop at them, mirroring
  continuous town driving.
* "file" -- a fixed CARLA Leaderboard route XML for reproducible benchmarking,
  parsed by route_loader and traced into a lane-level route.

Routes are returned in GlobalRoutePlanner format, a list of
(carla.Waypoint, RoadOption), which every controller (PID, PID++, MPC, MPCC)
consumes -- each tracks the waypoint positions.
"""
from __future__ import annotations

import logging
import random

import carla
import numpy as np
from agents.navigation.global_route_planner import GlobalRoutePlanner

logger = logging.getLogger("route_planner")


class RoutePlanner:
    """Trace and continuously extend lane-level routes from a CARLA map."""

    def __init__(self, world, resolution: float = 2.0, seed: int = 42):
        self.world = world
        self.map = world.get_map()
        self.grp = GlobalRoutePlanner(self.map, sampling_resolution=resolution)
        self.rng = random.Random(seed)
        self._spawn_points = self.map.get_spawn_points()
        if not self._spawn_points:
            raise RuntimeError("Map has no spawn points to use as route goals.")
        self.goal: carla.Location | None = None
        self.route: list = []
        self._xy = np.zeros((0, 2))
        self._i = 0          # tracked progress index, forward-local like the controller

    def trace(self, origin: carla.Location, destination: carla.Location):
        """Trace a lane-level route origin->destination as [(Waypoint, RoadOption), ...]."""
        return self.grp.trace_route(origin, destination)

    def _store(self, route: list, progress: int = 0):
        self.route = route
        self._xy = np.array([[wp.transform.location.x, wp.transform.location.y]
                             for wp, _ in route]) if route else np.zeros((0, 2))
        if self._xy.shape[0] >= 2:
            self._s = np.concatenate([[0.0],
                np.cumsum(np.linalg.norm(np.diff(self._xy, axis=0), axis=1))])
        else:
            self._s = np.zeros(self._xy.shape[0])
        self._i = max(0, progress)
        return route

    def in_lane_change(self, location: carla.Location, margin: float = 14.0) -> bool:
        """Report whether the route changes lanes within `margin` arc-metres of the ego.

        Used to mask the lane-marking crossing the reference makes during a lane change,
        so it is not counted or blinked as an unintended crossing.

        A lane change is detected two ways, because GlobalRoutePlanner does not always tag
        one: (1) an explicit RoadOption.CHANGELANE* waypoint, or (2) a same-road,
        same-direction, adjacent-lane_id step between non-junction waypoints (an implicit
        shift tagged LANEFOLLOW, e.g. moving over before a turn). Scanning road_options and
        lane_id around the projection rather than a pre-baked range is robust to a single
        zero-width tag, lane-change smoothing widening the maneuver, and projection slop.

        A turn crosses markings at the junction; a lane change happens on the road just
        before or after it. Never mask when the ego's own nearest waypoint is a junction
        (it is mid-turn), but still mask a lane change on the road approaching a junction.

        Parameters
        ----------
        location : carla.Location
            Ego position used to project onto the route.
        margin : float
            Half-window in arc-length metres scanned around the projection.

        Returns
        -------
        bool
            True if a lane change falls within `margin` of the ego.
        """
        if not self.route or self._s.size == 0:
            return False
        try:
            from agents.navigation.local_planner import RoadOption
            change = {RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT}
        except Exception:  # noqa: BLE001
            change = set()
        i = self._nearest_index(location)
        if self.route[min(i, len(self.route) - 1)][0].is_junction:
            return False                                 # ego in the junction is turning, not changing lanes
        s0 = float(self._s[min(i, self._s.size - 1)])
        lo = i
        while lo > 0 and s0 - self._s[lo - 1] <= margin:
            lo -= 1
        hi = i
        while hi + 1 < len(self.route) and self._s[hi + 1] - s0 <= margin:
            hi += 1
        for k in range(lo, hi + 1):
            if self.route[k][1] in change:               # (1) explicit CHANGELANE tag
                return True
        for k in range(max(lo, 1), hi + 1):              # (2) implicit adjacent-lane_id shift
            wa, wb = self.route[k - 1][0], self.route[k][0]
            if (wa.road_id == wb.road_id and wa.lane_id * wb.lane_id > 0
                    and abs(wa.lane_id - wb.lane_id) == 1
                    and not wa.is_junction and not wb.is_junction):
                return True
        return False

    def _random_goal(self, origin: carla.Location, min_length_m: float):
        for _ in range(40):
            goal = self.rng.choice(self._spawn_points).location
            if origin.distance(goal) < min_length_m:
                continue
            route = self.trace(origin, goal)
            if route and len(route) > 5:
                return goal, route
        goal = self.rng.choice(self._spawn_points).location
        return goal, self.trace(origin, goal)

    def _forward_stub(self, location: carla.Location, dist: float = 30.0, step: float = 2.0):
        """Follow the lane forward from `location` for ~`dist` metres.

        The continuation follows the lane's driving direction (carla Waypoint.next),
        bridging a route extension so the join cannot U-turn. Returns
        [(Waypoint, None), ...].
        """
        try:
            wp = self.map.get_waypoint(location, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
        except Exception:  # noqa: BLE001
            return []
        stub, acc = [], 0.0
        while acc < dist:
            nxts = wp.next(step)
            if not nxts:
                break
            wp = nxts[0]                  # first branch stays in the lane, going forward
            stub.append((wp, None))
            acc += step
        return stub

    @staticmethod
    def _max_turn_deg(route) -> float:
        """Compute the largest heading change between consecutive route segments (deg).

        Near-duplicate waypoints are dropped first: a zero-length segment has an
        undefined heading (atan2(0, 0) = 0) that otherwise masks a real cusp, the
        U-turn that produces an infeasible R~2 m hairpin the car brakes to a
        near-stop for.
        """
        if len(route) < 3:
            return 0.0
        P = np.array([[wp.transform.location.x, wp.transform.location.y] for wp, _ in route])
        d = np.diff(P, axis=0)
        d = d[np.hypot(d[:, 0], d[:, 1]) > 0.5]      # undefined heading of zero-length segments masks real cusps
        if len(d) < 2:
            return 0.0
        ang = np.arctan2(d[:, 1], d[:, 0])
        dang = np.abs((np.diff(ang) + np.pi) % (2 * np.pi) - np.pi)
        return float(np.degrees(dang.max())) if dang.size else 0.0

    def random_route(self, origin: carla.Location, min_length_m: float = 80.0):
        """Trace a route from origin to a random, sufficiently far goal."""
        goal, route = self._random_goal(origin, min_length_m)
        self.goal = goal
        logger.info("Random route: %d waypoints to (%.1f, %.1f).", len(route), goal.x, goal.y)
        return self._store(route)

    def file_route(self, route_obj):
        """Build a lane-level route from a Leaderboard waypoint list.

        Sparse waypoints (classic Leaderboard, tens of metres apart) are traced
        pair-by-pair with the GlobalRoutePlanner, which fills the drivable lane path
        between them. Dense waypoints (a pre-computed lane-graph walk from build_routes,
        a few metres apart) are snapped to lanes and used as-is: that path is already a
        valid lane route, and re-running the GRP over it only re-introduces detours and
        180-deg cusps it cannot shortcut (consecutive points straddle junctions on
        opposing lanes).
        """
        wps = route_obj.waypoints
        spacing = (float(np.median(np.linalg.norm(np.diff(np.asarray(
            [(w[0], w[1]) for w in wps]), axis=0), axis=1))) if len(wps) > 2 else 1e9)

        if spacing <= 8.0:                              # dense pre-traced walk: snap, do not re-GRP
            full = []
            for (x, y, z) in wps:
                w = self.map.get_waypoint(carla.Location(x=x, y=y, z=z),
                                          project_to_road=True, lane_type=carla.LaneType.Driving)
                if w is not None:
                    full.append((w, None))
            if not full:
                raise RuntimeError("file_route produced an empty route.")
            self.goal = carla.Location(x=wps[-1][0], y=wps[-1][1], z=wps[-1][2])
            logger.info("File route (dense walk, snapped): %d lane waypoints, spacing %.1f m.",
                        len(full), spacing)
            return self._store(full)

        full = []
        for a, b in zip(wps[:-1], wps[1:]):
            la = carla.Location(x=a[0], y=a[1], z=a[2])
            lb = carla.Location(x=b[0], y=b[1], z=b[2])
            seg = self.trace(la, lb)
            full.extend(seg if not full else seg[1:])
        if not full:
            raise RuntimeError("file_route produced an empty route.")
        self.goal = carla.Location(x=wps[-1][0], y=wps[-1][1], z=wps[-1][2])
        logger.info("File route: %d lane-level waypoints.", len(full))
        return self._store(full)

    def _nearest_index(self, location: carla.Location, window: int = 80) -> int:
        """Find the forward-local nearest route index, advancing from tracked progress only.

        Scanning forward from the last index keeps a self-approaching or freshly-appended
        route from jumping progress to a spatially-near but route-distant point.
        """
        if self._xy.shape[0] == 0:
            return 0
        lo = max(0, self._i - 2)
        hi = min(self._xy.shape[0], self._i + window)
        seg = self._xy[lo:hi]
        d2 = (seg[:, 0] - location.x) ** 2 + (seg[:, 1] - location.y) ** 2
        self._i = lo + int(np.argmin(d2))
        return self._i

    def remaining_distance(self, location: carla.Location) -> float:
        """Return the arc length of route still ahead of `location` in metres (0 if no route)."""
        if self._xy.shape[0] < 2:
            return 0.0
        i = self._nearest_index(location)
        ahead = self._xy[i:]
        if ahead.shape[0] < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(ahead, axis=0), axis=1)))

    def extend_forward(self, min_length_m: float = 120.0) -> list:
        """Append a forward lane-following stub past the current goal as reference-only road.

        The stub is deterministic (Waypoint.next, staying in the lane's driving direction),
        so unlike the random extension a U-turn or cusp is impossible. The goal is left
        untouched: callers use this to give the controller reference room past the endpoint
        without changing where the route ends or what counts as arrival.
        """
        stub = self._forward_stub(self.goal, dist=min_length_m)
        if not stub:
            logger.warning("extend_forward: no lane continuation past the goal; "
                           "the reference ends at the goal.")
            return self.route
        logger.info("Appended ~%.0f m forward reference buffer past the goal (%d wpts).",
                    min_length_m, len(stub))
        return self._store(self.route + stub, progress=self._i)

    def extend_random(self, ego_location: carla.Location, min_length_m: float = 80.0,
                      keep_behind: int = 5) -> list:
        """Continue the route forward through the old endpoint to a new random goal.

        A forward lane stub (Waypoint.next) bridges the old endpoint so the join cannot
        U-turn, which would otherwise let the new GlobalRoutePlanner segment route backward
        out of the endpoint into a 180 in the reference. The random goal is traced from the
        stub's forward end; a few goals are retried and the one whose join turns the least is
        kept. The ego index is taken on the old route before appending, so a segment looping
        near the car cannot discard the leftover.
        """
        i_ego = self._nearest_index(ego_location)
        stub = self._forward_stub(self.goal, dist=20.0)
        origin = stub[-1][0].transform.location if stub else self.goal

        best = None
        for _ in range(5):                           # bounded (each trace is a graph search)
            new_goal, seg = self._random_goal(origin, min_length_m)
            full = self.route + stub + seg[1:]       # old route, forward stub, then new segment
            # Worst turn across the join and the whole new segment only; the leftover's
            # existing curves are excluded so legitimate sharp intersection turns are not
            # rejected. Catches a U-turn/cusp anywhere the GlobalRoutePlanner doubled back
            # to reach the goal (an R~2 m hairpin no car can follow), prompting a retry.
            new_turn = self._max_turn_deg(self.route[-2:] + stub + seg[1:])
            if best is None or new_turn < best[2]:
                best = (new_goal, full, new_turn)
            if new_turn < 120.0:                     # no U-turn/cusp; accept
                break
        new_goal, full, turn = best
        if turn >= 120.0:
            logger.warning("Route extension: best of 5 goals still turns %.0f deg "
                           "(possible tight hairpin ahead).", turn)
        self.goal = new_goal
        start = max(0, i_ego - keep_behind)
        trimmed = full[start:]
        logger.info("Extended route forward through endpoint -> goal (%.1f, %.1f); "
                    "%d wpts, max join turn=%.0f deg.", new_goal.x, new_goal.y,
                    len(trimmed), turn)
        return self._store(trimmed, progress=i_ego - start)
