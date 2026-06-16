"""Measure CARLA's lateral-grip limit and cornering stiffness via steady-state circles.

Run a textbook constant-radius test so ``a_lat_max`` (and the dynamic model's Cf/Cr)
can be set from a measured number instead of a guess. For a few fixed steering angles,
ramp throttle so the car circles faster and faster; at each steady speed record body-frame
(vx, vy, r), the measured lateral accel a_y, and the front-wheel angle delta.

  * Grip limit  = peak sustained |a_y| (the friction circle's lateral reach -> a_lat_max).
  * Cf, Cr      = least-squares fit of F_y = -C*alpha per axle in the linear regime,
                  with the per-axle force from the steady-state moment balance
                  F_yf = (l_r/L)*m*a_y,  F_yr = (l_f/L)*m*a_y.

The measured grip is the vehicle-level SUSTAINED limit (~1.5-1.6 g for the Model 3), far
below CARLA's nominal ``wheel.tire_friction`` (~3.5) -- that is a PhysX parameter, not the
achievable lateral acceleration. The tool also prints tire_friction for reference.

Pick a map with open space (Town06's wide highway works well), then::

    ros2 run carla_mpc_bringup identify_grip --map Town06 --spawn-index 0

Prints the measured grip + Cf/Cr and a recommended a_lat_max; --out writes a YAML snippet.
Needs ~40-80 m of clear flat ground around the spawn; off-ground samples are filtered by
a vx floor, so on-ground steady samples are still usable if the car runs off the road.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

try:
    import carla
except ImportError:  # pragma: no cover
    print("carla not importable -- add $CARLA_ROOT/PythonAPI/carla to PYTHONPATH", file=sys.stderr)
    raise


def _body_xy(v, yaw):
    """Rotate a world velocity/accel vector into body-frame (longitudinal, lateral); CARLA is left-handed."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * v.x + s * v.y, -s * v.x + c * v.y


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", default=2000, type=int)
    ap.add_argument("--map", default=None, help="load this town first (e.g. Town06); default keeps current")
    ap.add_argument("--spawn-index", dest="spawn_index", default=0, type=int)
    ap.add_argument("--steers", default="0.15,0.25,0.35",
                    help="steer fractions of max_steer to sweep (smaller = larger radius; max_steer can be ~70 deg)")
    ap.add_argument("--out", default=None, help="write the identified Cf/Cr/a_lat_max to this YAML")
    args, _ = ap.parse_known_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    if args.map:
        client.load_world(args.map)
    world = client.get_world()
    s0 = world.get_settings()
    restore = (s0.synchronous_mode, s0.fixed_delta_seconds)
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.02          # 50 Hz physics for clean steady-state samples
    world.apply_settings(s)
    dt = 0.02

    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    sps = world.get_map().get_spawn_points()
    spawn = sps[args.spawn_index % len(sps)]
    ego = world.spawn_actor(bp, spawn)

    try:
        # Reuse the controller's geometry/mass extraction so identified params match the model.
        from ament_index_python.packages import get_package_share_directory
        from carla_mpc_bringup.control.mpc_controller import extract_bicycle_params
        from carla_mpc_bringup.core.config_loader import load_controller_config
        import os
        cfg = load_controller_config(
            os.path.join(get_package_share_directory("carla_mpc_bringup"), "config", "controller.yaml")).mpc
        for _ in range(5):
            world.tick()
        p, max_steer = extract_bicycle_params(ego, cfg)
        L, l_r, l_f, m = p.L, p.l_r, p.l_f, p.m
        print(f"[grip-id] vehicle: L={L:.3f} l_f={l_f:.3f} l_r={l_r:.3f} m={m:.0f} kg "
              f"max_steer={math.degrees(max_steer):.1f} deg")
        # CARLA's wheel.tire_friction is a PhysX parameter (~3.5 by default), NOT the achievable
        # lateral g: the steady-state limit measured below is far lower. Print it so the two aren't confused.
        mu_nominal = float(ego.get_physics_control().wheels[0].tire_friction)
        print(f"[grip-id] CARLA tire_friction (nominal mu) = {mu_nominal:.2f}  -- a PhysX parameter, not "
              f"the grip limit; the sustained lateral g measured below is what a_lat_max should track.")

        def hold(ticks):
            for _ in range(ticks):
                world.tick()

        def reset():
            ego.set_transform(spawn)
            ego.set_target_velocity(carla.Vector3D(0, 0, 0))
            ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            hold(int(1.5 / dt))

        RMAX = 2.0   # rad/s; above ~0.85*this the car is spinning, not steady-cornering
        def step_to(v_tgt, sf):
            """Hold a fixed steer with feedback throttle, tick once, and return (vx, vy, r, a_y).

            r is finite-differenced from yaw rather than read from get_angular_velocity, whose
            sign convention is unreliable: the raw value flips the slip-angle sign and yields a
            negative Cr. a_y = vx*r is the steady centripetal accel, robust to slide spikes.
            """
            y0 = math.radians(ego.get_transform().rotation.yaw)
            vx, _ = _body_xy(ego.get_velocity(), y0)
            thr = float(np.clip(0.6 * (v_tgt - vx), 0.0, 1.0))
            brk = float(np.clip(0.4 * (vx - v_tgt - 1.5), 0.0, 1.0))
            ego.apply_control(carla.VehicleControl(throttle=thr, steer=float(sf), brake=brk, hand_brake=False))
            world.tick()
            y1 = math.radians(ego.get_transform().rotation.yaw)
            r = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)) / dt
            vx, vy = _body_xy(ego.get_velocity(), y1)
            return vx, vy, r, vx * r

        samples = []  # rows: [vx, vy, r, a_y, delta]
        for sf in [float(x) for x in args.steers.split(",") if x.strip()]:
            reset()
            delta = sf * max_steer
            peak = 0.0
            for v_tgt in np.arange(4.0, 33.0, 3.0):
                for _ in range(int(3.5 / dt)):           # settle toward v_tgt before sampling
                    step_to(v_tgt, sf)
                rec = []
                for _ in range(int(1.0 / dt)):           # ~1 s window of steady samples
                    vx, vy, r, ay = step_to(v_tgt, sf)
                    if vx > 2.0:                          # vx floor: drop off-ground/stalled samples
                        rec.append([vx, vy, r, ay, delta])
                if not rec:
                    continue                              # not rolling yet -> try a higher target
                arr = np.array(rec)
                r_mean = float(np.abs(arr[:, 2]).mean())
                vx_mean = float(arr[:, 0].mean())
                ay_mean = float(np.abs(arr[:, 3]).mean())
                # Saturated yaw rate (spun out) or can't hold target speed (understeered wide)
                # both mean grip is exceeded; stop here -- these unsteady samples over-read it.
                if r_mean > 0.85 * RMAX or vx_mean < 0.6 * v_tgt:
                    print(f"[grip-id] steer={sf:.2f} ({math.degrees(delta):.0f}deg) v_tgt={v_tgt:4.1f} "
                          f"-> lost grip (r={r_mean:.2f} rad/s, vx={vx_mean*3.6:.0f}km/h); limit ~{peak:.1f} m/s^2")
                    break
                samples.extend(rec)
                peak = max(peak, ay_mean)
                print(f"[grip-id] steer={sf:.2f} ({math.degrees(delta):.0f}deg) v_tgt={v_tgt:4.1f} "
                      f"vx={vx_mean*3.6:5.1f}km/h a_y={ay_mean:5.2f} m/s^2 ({ay_mean/9.81:.2f}g)")
                if v_tgt > 8.0 and ay_mean < 0.90 * peak:
                    print(f"[grip-id]   -> a_y rolled off; grip ~{peak:.1f} m/s^2")
                    break

        reset()
        if len(samples) < 10:
            print("[grip-id] too few samples -- need more open space / a different spawn", file=sys.stderr)
            return 1
        A = np.array(samples)
        vx, vy, r, a_y, delta = A.T
        a_abs = np.abs(a_y)
        grip = float(np.percentile(a_abs, 95))           # robust peak of the stable samples (spins excluded)

        # Cornering stiffness: least-squares slope (through origin) of F_y = -C*alpha in the linear
        # regime. Tight low-speed circles barely slip the rear (alpha_r ~ 0, the kinematic
        # vy ~ l_r*r condition), so Cr is usually not identifiable from this geometry. Proper
        # Cf/Cr ID needs gentler, faster circles where both axles slip measurably.
        alpha_f = np.arctan2(vy + l_f * r, vx) - delta
        alpha_r = np.arctan2(vy - l_r * r, vx)
        Fyf = (l_r / L) * m * a_y
        Fyr = (l_f / L) * m * a_y
        lin = (a_abs > 1.0) & (a_abs < 0.7 * grip) & \
              (np.abs(np.degrees(alpha_f)) <= 5.0) & (np.abs(np.degrees(alpha_r)) <= 5.0)

        def _fit_C(F, al):
            a = al[lin]
            if lin.sum() < 8 or float(np.degrees(np.abs(a)).mean()) < 0.5:   # slip too small -> ill-conditioned
                return None
            C = float(-np.sum(F[lin] * a) / np.sum(a * a))                   # slope: F = -C*alpha
            return C if 1e4 < C < 5e5 else None                             # physical range only
        Cf = _fit_C(Fyf, alpha_f)
        Cr = _fit_C(Fyr, alpha_r)

        rec_alat = round(grip, 1)
        cf_s = f"{Cf:.0f} N/rad" if Cf else "not identifiable here (front slip small/nonlinear)"
        cr_s = (f"{Cr:.0f} N/rad" if Cr else
                "not identifiable here (rear barely slips on tight circles -- needs gentler/faster circles)")
        print("\n================ IDENTIFIED ================")
        print(f"  measured grip limit : {grip:.2f} m/s^2  ({grip/9.81:.2f} g)   [{len(samples)} samples]")
        print(f"  CARLA tire_friction : {mu_nominal:.2f} (nominal PhysX mu) -- the achievable lateral g above")
        print(f"                        is far below mu*g; tire_friction is a parameter, not the grip limit.")
        print(f"  Cf (front stiffness): {cf_s}")
        print(f"  Cr (rear  stiffness): {cr_s}")
        print(f"  -> measured grip = a_lat_max ceiling = {rec_alat}  (curve_speed_factor sets the margin below it)")
        print("     NOTE: the GRIP is the reliable output; Cf/Cr from this tight-circle test are rough")
        print("           (front saturates, rear barely slips). For accurate Cf/Cr run gentle big circles.")
        print("           This is the SUSTAINED steady-state limit; brief transient/peak grip can be slightly higher.")
        print("============================================")
        if args.out:
            import csv as _csv
            import yaml
            with open(args.out, "w") as f:
                yaml.safe_dump({"identified": {"grip_ms2": round(grip, 3), "grip_g": round(grip / 9.81, 3),
                                               "tire_friction_nominal": round(mu_nominal, 3),
                                               "Cf": (round(Cf, 1) if Cf else None),
                                               "Cr": (round(Cr, 1) if Cr else None),
                                               "recommend_a_lat_max": rec_alat}}, f)
            raw = args.out.rsplit(".", 1)[0] + "_raw.csv"   # raw rows for offline re-analysis
            with open(raw, "w", newline="") as f:
                w = _csv.writer(f); w.writerow(["vx", "vy", "r", "a_y", "delta"]); w.writerows(A.tolist())
            print(f"[grip-id] wrote {args.out} + {raw}")
        return 0
    finally:
        try:
            ego.destroy()
        except Exception:  # noqa: BLE001
            pass
        s2 = world.get_settings()
        s2.synchronous_mode, s2.fixed_delta_seconds = restore
        world.apply_settings(s2)


if __name__ == "__main__":
    raise SystemExit(main())
