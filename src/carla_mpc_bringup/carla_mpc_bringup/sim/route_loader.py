"""Parse CARLA Leaderboard 2.0 route XML files.

The benchmark convention (scenario_runner / Leaderboard) defines the ego start
and path as an ordered list of <position> waypoints in CARLA world coordinates
(left-handed; x, y, z in metres). The FIRST waypoint is the spawn pose; the rest
define the route to drive.

Pure Python (xml.etree) -- no carla, no rclpy -- so it is testable offline.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET


@dataclass
class Route:
    """One parsed Leaderboard route: spawn pose, path, and weather presets."""

    id: str
    town: str
    waypoints: list[tuple[float, float, float]]   # CARLA world coords (x, y, z), metres
    weathers: list[dict] = field(default_factory=list)

    @property
    def start(self) -> tuple[float, float, float]:
        """First waypoint, used as the ego spawn pose."""
        return self.waypoints[0]


def parse_routes_file(path: str | Path) -> list[Route]:
    """Parse every <route> in a Leaderboard route XML file.

    Parameters
    ----------
    path : str or Path
        Path to the route XML file.

    Returns
    -------
    list of Route
        One entry per ``<route>`` element; routes with no ``<waypoints>`` or
        no ``<position>`` children are skipped.
    """
    tree = ET.parse(str(path))
    root = tree.getroot()
    routes: list[Route] = []
    for r in root.findall("route"):
        wps_node = r.find("waypoints")
        if wps_node is None:
            continue
        wps = [
            (float(p.get("x")), float(p.get("y")), float(p.get("z", 0.0)))
            for p in wps_node.findall("position")
        ]
        if not wps:
            continue
        weathers = []
        w_node = r.find("weathers")
        if w_node is not None:
            for w in w_node.findall("weather"):
                weathers.append({k: v for k, v in w.attrib.items()})
        routes.append(Route(
            id=r.get("id", Path(path).stem),
            town=r.get("town", ""),
            waypoints=wps,
            weathers=weathers,
        ))
    return routes


def load_route(path: str | Path, route_id: str | None = None) -> Route:
    """Load a single route from a Leaderboard route XML file.

    Parameters
    ----------
    path : str or Path
        Path to the route XML file.
    route_id : str or None, optional
        Route id to select; when None or empty, the first route is returned.

    Returns
    -------
    Route
        The matching route.

    Raises
    ------
    ValueError
        If the file contains no routes, or no route matches ``route_id``.
    """
    routes = parse_routes_file(path)
    if not routes:
        raise ValueError(f"No <route> found in {path}")
    if not route_id:
        return routes[0]
    for r in routes:
        if r.id == route_id:
            return r
    raise ValueError(
        f"route id '{route_id}' not in {path} (available: {[r.id for r in routes]})")


if __name__ == "__main__":
    import sys
    rs = parse_routes_file(sys.argv[1])
    for r in rs:
        print(f"{r.id}  town={r.town}  {len(r.waypoints)} waypoints  start={r.start}")
