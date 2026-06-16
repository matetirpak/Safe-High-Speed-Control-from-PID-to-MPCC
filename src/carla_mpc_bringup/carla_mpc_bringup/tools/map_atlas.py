"""Render a top-down map atlas for CARLA towns to aid scenario route design.

For each town, render the road network plus every spawn point (labelled with
its index) and junction, and dump the raw positions as JSON, so routes can be
authored against real coordinates instead of guessing spawn+seed.

Per town it writes ``<out>/<Town>.png`` and ``<out>/<Town>.json``, the latter
holding ``spawn_points`` (route XMLs use these CARLA coords directly) and
``junctions`` (a roundabout shows up as a large, high-``n_paths`` junction).

Loading a world changes the server's current map, so run this deliberately and
reload your scenario map afterwards. Coordinates are raw CARLA (left-handed);
the plot's y-axis is inverted so north is roughly up while the printed (x, y)
still match the JSON 1:1.

Run example::

    ros2 run carla_mpc_bringup map_atlas --towns Town03,Town07
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import carla

DEFAULT_TOWNS = ["Town03", "Town04", "Town06", "Town07", "Town10HD"]
SCATTER_TOWNS = {"Town11", "Town12", "Town15"}     # big maps: spread (de-overlap) the spawn labels


def _extract(carla_map):
    """Extract spawn points, junctions, and road segments from a loaded map.

    Returns
    -------
    tuple
        ``(spawns, junctions, road_seg)``, all in raw CARLA coordinates. Roads
        are coarsened on huge maps so the result stays plottable.
    """
    spawns = []
    for i, tf in enumerate(carla_map.get_spawn_points()):
        loc, rot = tf.location, tf.rotation
        spawns.append({"i": i, "x": round(loc.x, 2), "y": round(loc.y, 2),
                       "z": round(loc.z, 2), "yaw": round(rot.yaw, 1)})

    res = 2.0                                           # auto-coarsen on huge maps (Town11/12) to stay plottable
    wps = carla_map.generate_waypoints(res)
    while len(wps) > 120000 and res < 24.0:
        res *= 2.0
        wps = carla_map.generate_waypoints(res)
    road_seg = []
    for w in wps:
        nx = w.next(res * 1.3)
        if nx:
            a, b = w.transform.location, nx[0].transform.location
            road_seg.append(((round(a.x, 1), round(a.y, 1)), (round(b.x, 1), round(b.y, 1))))

    junctions: dict[int, dict] = {}
    for w in wps:
        if not w.is_junction:
            continue
        j = w.get_junction()
        if j is None or j.id in junctions:
            continue
        bb = j.bounding_box
        try:
            n_paths = len(j.get_waypoints(carla.LaneType.Driving))
        except Exception:                              # noqa: BLE001
            n_paths = 0
        junctions[j.id] = {"id": int(j.id),
                           "x": round(bb.location.x, 1), "y": round(bb.location.y, 1),
                           "ext_x": round(bb.extent.x, 1), "ext_y": round(bb.extent.y, 1),
                           "n_paths": n_paths}
    return spawns, list(junctions.values()), road_seg


def _plot(town, spawns, junctions, road_seg, out_png, scatter=False):
    """Render the town's road network, junctions, and labelled spawn points to PNG.

    Parameters
    ----------
    scatter : bool
        On big maps, de-overlap spawn labels onto a grid with leader lines back
        to each dot; otherwise place plain labels at the dot.

    Notes
    -----
    Figures wider than ~9000 px are split into a downscaled overview plus four
    quadrant PNGs, since most viewers cannot open the full image.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    xs = [p[0] for seg in road_seg for p in seg] or [0.0]
    ys = [p[1] for seg in road_seg for p in seg] or [0.0]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    side = max(10.0, min(150.0, span / 40.0))          # figure side in inches; scales up with map span
    dpi = 140 if side < 45.0 else 100                  # big maps -> ~15000 px

    fig, ax = plt.subplots(figsize=(side, side))
    ax.add_collection(LineCollection(road_seg, colors="#384150", linewidths=1.0, alpha=0.95))
    for j in junctions:
        r = max(j["ext_x"], j["ext_y"])
        ax.add_patch(plt.Circle((j["x"], j["y"]), r, fill=False, ec="#ff7a18", lw=1.6, alpha=0.95))
        ax.text(j["x"], j["y"], str(j["id"]), color="#cc5500", fontsize=5, ha="center", va="center")
    sx = [p["x"] for p in spawns]; sy = [p["y"] for p in spawns]
    ax.scatter(sx, sy, s=16, c="#1565ff", zorder=5)

    fpt = 6
    if scatter:
        # Bucket each label on a grid sized to one label; if a cell is taken, spiral
        # to the nearest free one and draw a leader line back, so dense-city numbers
        # stay readable.
        ch = (fpt / 72.0) * (span / side) * 2.6                              # label height in data units + gap
        cw = ch * 2.8                                                        # ~4-digit label width + gap
        occ: set = set()
        for p in spawns:
            x, y = p["x"], p["y"]
            gx, gy = round(x / cw), round(y / ch)
            tgt = (gx, gy)
            if tgt in occ:
                tgt = None
                for rad in range(1, 12):
                    ring = [(gx + dx, gy + dy) for dx in range(-rad, rad + 1) for dy in range(-rad, rad + 1)
                            if max(abs(dx), abs(dy)) == rad and (gx + dx, gy + dy) not in occ]
                    if ring:
                        tgt = min(ring, key=lambda g: (g[0] - gx) ** 2 + (g[1] - gy) ** 2)
                        break
                if tgt is None:
                    tgt = (gx, gy)
            occ.add(tgt)
            ax.annotate(str(p["i"]), (x, y), xytext=(tgt[0] * cw, tgt[1] * ch), textcoords="data",
                        fontsize=fpt, color="#0b3d91", ha="center", va="center",
                        arrowprops=dict(arrowstyle="-", color="#0b3d91", lw=0.7, alpha=0.9))
    else:
        for p in spawns:
            ax.annotate(str(p["i"]), (p["x"], p["y"]), fontsize=fpt, color="#0b3d91",
                        xytext=(2, 2), textcoords="offset points")

    ax.set_xlim(min(xs) - 8, max(xs) + 8); ax.set_ylim(min(ys) - 8, max(ys) + 8)
    # CARLA y points down: invert so north is up, the convention shared by all
    # map/path/report plots.
    ax.set_aspect("equal"); ax.invert_yaxis()
    ax.set_title(f"{town}  —  {len(spawns)} spawn points (blue), {len(junctions)} junctions (orange)")
    ax.set_xlabel("x (m, CARLA)"); ax.set_ylabel("y (m, CARLA)")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    if int(side * dpi) > 9000:                          # too big to open whole -> overview + 4 quadrant PNGs
        import io
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        buf = io.BytesIO(); fig.savefig(buf, dpi=dpi, format="png"); plt.close(fig); buf.seek(0)
        full = Image.open(buf); W, H = full.size
        full.resize((W // 4, H // 4)).save(out_png)     # small overview for navigation
        stem = str(out_png)[:-4]
        # Quadrants are named by image position, not compass direction.
        for qx, qy, tag in [(0, 0, "top_left"), (1, 0, "top_right"),
                            (0, 1, "bottom_left"), (1, 1, "bottom_right")]:
            full.crop((qx * W // 2, qy * H // 2, (qx + 1) * W // 2, (qy + 1) * H // 2)).save(f"{stem}_{tag}.png")
    else:
        fig.savefig(out_png, dpi=dpi); plt.close(fig)


def main() -> int:
    """Load each requested town, extract its layout, and write the PNG + JSON atlas."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--towns", default=",".join(DEFAULT_TOWNS))
    ap.add_argument("--out", default="logs/map_atlas")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", default=2000, type=int)
    args = ap.parse_args()

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port); client.set_timeout(300.0)  # big maps (Town11/12) load slowly
    towns = [t.strip() for t in args.towns.split(",") if t.strip()]

    for town in towns:
        try:
            print(f"[{town}] loading world ...", flush=True)
            world = client.load_world(town)
            carla_map = world.get_map()
            spawns, junctions, road_seg = _extract(carla_map)
            (out / f"{town}.json").write_text(json.dumps(
                {"town": town, "spawn_points": spawns, "junctions": junctions}, indent=1))
            _plot(town, spawns, junctions, road_seg, out / f"{town}.png", town in SCATTER_TOWNS)
            big = sorted(junctions, key=lambda j: max(j["ext_x"], j["ext_y"]), reverse=True)[:3]
            print(f"[{town}] {len(spawns)} spawns, {len(junctions)} junctions"
                  + ("  [labels scattered]" if town in SCATTER_TOWNS else "") + ". Largest: "
                  + ", ".join(f"id{j['id']}@({j['x']:.0f},{j['y']:.0f})" for j in big), flush=True)
        except Exception as e:                         # noqa: BLE001
            print(f"[{town}] FAILED: {e}", flush=True)
    print(f"\nAtlas -> {out}/  (<Town>.png + <Town>.json)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
