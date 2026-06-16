"""Load and validate the structured vehicle / Carla / controller YAML configs.

Pure Python (yaml + frame_conventions only); no carla or rclpy imports, so it can
be exercised offline and unit-tested. The dataclasses here are the single source
of truth consumed by control_node, sensor_factory, and the PID/MPC/MPCC controllers.

Three configs:
  * vehicle.yaml    -> VehicleConfig    (ego blueprint + sensor rig)
  * carla.yaml      -> CarlaConfig      (world, sync rate, weather, route source)
  * controller.yaml -> ControllerConfig (PID gains + speed policy + MPC params)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .frame_conventions import Mount


@dataclass
class SensorConfig:
    name: str
    type: str                       # e.g. "camera.rgb" -> blueprint "sensor.camera.rgb"
    enabled: bool = True
    ros: bool = True                # call enable_for_ros() after spawn
    mount: Mount = field(default_factory=Mount)
    attributes: dict = field(default_factory=dict)

    @property
    def blueprint(self) -> str:
        return f"sensor.{self.type}"

    @property
    def is_camera(self) -> bool:
        return self.type.startswith("camera.")

    @classmethod
    def from_dict(cls, d: dict) -> "SensorConfig":
        return cls(
            name=d["name"],
            type=d["type"],
            enabled=bool(d.get("enabled", True)),
            ros=bool(d.get("ros", True)),
            mount=Mount.from_dict(d.get("mount")),
            attributes=dict(d.get("attributes", {})),
        )


@dataclass
class VehicleConfig:
    blueprint: str
    role_name: str
    ros_name: str
    base_frame: str
    frame_prefix: str
    sensors: list[SensorConfig]

    def enabled_sensors(self) -> list[SensorConfig]:
        return [s for s in self.sensors if s.enabled]

    def frame_id(self, name: str) -> str:
        return f"{self.frame_prefix}{name}"

    def topic_base(self, sensor: SensorConfig) -> str:
        return f"/carla/{self.ros_name}/{sensor.name}"

    def find(self, name: str) -> SensorConfig | None:
        return next((s for s in self.sensors if s.name == name), None)


@dataclass
class CarlaConfig:
    host: str
    port: int
    timeout: float
    map: str
    synchronous_mode: bool
    fixed_delta_seconds: float
    realtime_factor: float
    weather: str
    autopilot: bool
    num_vehicles: int
    num_walkers: int
    ignore_lights: bool
    ignore_signs: bool
    seed: int
    spawn_index: int
    warmup_seconds: float
    # Route source for the CONTROLLER to track:
    #   route_mode = "random" -> GlobalRoutePlanner from spawn to random goals,
    #                            regenerated on arrival (continuous town driving).
    #   route_mode = "file"   -> a CARLA Leaderboard route XML (route_file/route_id),
    #                            deterministic + reproducible benchmarking.
    route_mode: str
    route_loop: bool             # random mode: keep generating new goals forever
    route_resolution: float      # GlobalRoutePlanner sampling resolution (m)
    # Benchmark route (used when route_mode == "file").
    route_file: str
    route_id: str
    target_speed_kmh: float

    @property
    def use_route_file(self) -> bool:
        return self.route_mode == "file" and bool(self.route_file)


def _load_yaml(path: str | Path) -> dict:
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def load_vehicle_config(path: str | Path) -> VehicleConfig:
    raw = _load_yaml(path)
    veh = raw["vehicle"]
    defaults = raw.get("defaults", {})
    sensors = [SensorConfig.from_dict(s) for s in raw.get("sensors", [])]

    names = [s.name for s in sensors]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        raise ValueError(f"Duplicate sensor names in vehicle.yaml: {sorted(dupes)}")

    cfg = VehicleConfig(
        blueprint=veh["blueprint"],
        role_name=veh.get("role_name", "ego"),
        ros_name=veh.get("ros_name", veh.get("role_name", "ego")),
        base_frame=veh.get("base_frame", "base_link"),
        frame_prefix=defaults.get("frame_prefix", ""),
        sensors=sensors,
    )
    return cfg


def load_carla_config(path: str | Path) -> CarlaConfig:
    raw = _load_yaml(path)
    client = raw.get("client", {})
    world = raw.get("world", {})
    traffic = raw.get("traffic", {})
    spawn = raw.get("spawn", {})
    route = raw.get("route", {})

    # Resolve a relative route file against the config file's directory.
    route_file = route.get("file", "") or ""
    if route_file and not Path(route_file).is_absolute():
        route_file = str((Path(path).resolve().parent / route_file))

    return CarlaConfig(
        host=client.get("host", "localhost"),
        port=int(client.get("port", 2000)),
        timeout=float(client.get("timeout", 20.0)),
        map=world.get("map", "Town10HD"),
        synchronous_mode=bool(world.get("synchronous_mode", True)),
        fixed_delta_seconds=float(world.get("fixed_delta_seconds", 0.005)),
        realtime_factor=float(world.get("realtime_factor", 1.0)),
        weather=raw.get("weather", {}).get("preset", "ClearNoon"),
        autopilot=bool(traffic.get("autopilot", True)),
        num_vehicles=int(traffic.get("number_of_vehicles", 0)),
        num_walkers=int(traffic.get("number_of_walkers", 0)),
        ignore_lights=bool(traffic.get("ignore_lights", True)),
        ignore_signs=bool(traffic.get("ignore_signs", True)),
        seed=int(spawn.get("seed", 42)),
        spawn_index=int(spawn.get("spawn_index", 0)),
        warmup_seconds=float(spawn.get("warmup_seconds", 3.0)),
        route_mode=str(route.get("mode", "random")),
        route_loop=bool(route.get("loop", True)),
        route_resolution=float(route.get("resolution", 2.0)),
        route_file=route_file,
        route_id=route.get("id", "") or "",
        target_speed_kmh=float(route.get("target_speed_kmh", 0.0)),
    )


# ============================================================================
#  Controller config (controller.yaml)
# ============================================================================
@dataclass
class PidConfig:
    # Longitudinal speed PID (output: throttle/brake in [-1, 1]).
    long_kp: float = 0.10
    long_ki: float = 0.15
    long_kd: float = 0.25
    # Longitudinal feed-forward: command += long_kff * d(target)/tick. Pure feedback lags a ramping
    # target during decel (the car coasts instead of braking); the FF anticipates the ramp (brake on a
    # dropping target, throttle on a rising one) so the PID only trims the residual. 0 = off.
    long_kff: float = 0.0
    # Lateral heading PID (output: steer in [-1, 1]).
    lat_kp: float = 1.20
    lat_ki: float = 0.00
    lat_kd: float = 0.10
    max_throttle: float = 1.0
    max_brake: float = 1.0
    max_steer: float = 1.0
    # Curvature-aware speed policy (see docs/configuration.md and the PID controller).
    default_target_speed_kmh: float = 120.0
    min_speed_kmh: float = 10.0
    no_slowdown_below_deg: float = 15.0
    max_turn_angle_deg: float = 90.0
    angle_sliding_window_n: int = 10
    angle_to_speed_exponent: float = 1.4
    curve_safety_margin_m: float = 20.0
    decel_ms2: float = 3.0
    long_lookahead_sec: float = 3.0
    min_long_lookahead_dist: float = 20.0
    lat_lookahead_sec: float = 0.5
    # High-steer throttle clip (the reference's spin-out guard).
    high_steer_threshold: float = 0.2
    high_steer_throttle_cap: float = 0.3
    # PID++ only (ignored by the baseline PID): lateral curvature feedforward gain (Ackermann
    # delta_ff = lat_kff * atan(L*kappa)) and a steer-rate limit (normalized steer / s; 0 = off).
    lat_kff: float = 0.0
    steer_rate_max: float = 0.0
    # PID++ STANLEY lateral (cross-track gain k_e + low-speed softening k_soft; the
    # term is atan(k_e*e_cte/(k_soft+v))) and a derivative LOW-PASS (d_filter_alpha; 0=no filter,
    # ~0.7 typical). k_e=0 disables the Stanley cross-track term (falls back to heading+FF only).
    stanley_k_e: float = 0.0
    stanley_k_soft: float = 1.0
    d_filter_alpha: float = 0.7
    long_d_max: float = 1.0          # clamp on the longitudinal D contribution (D_max)
    # PID++ longitudinal: forward-backward velocity PROFILE from path curvature (replaces the
    # angle-ahead + braking-margin heuristic). 0 = use the inherited heuristic instead.
    a_lat_max: float = 0.0           # lateral-accel limit for the curvature speed cap (m/s^2)
    curve_speed_factor: float = 0.85 # corner at this fraction of the lateral limit (margin)
    a_accel: float = 3.0             # forward-pass acceleration bound for the profile (m/s^2)
    # Actuation smoothing: exponentially-weighted average of the last N (accel, steer) commands
    # (newest weighted most) -> non-chattering output. cmd_smooth_n = window length (<=1 disables);
    # cmd_smooth_decay in (0,1) = per-step weight ratio (lower = less smoothing/lag, higher = smoother).
    # Retune with the tick rate.
    cmd_smooth_n: int = 0
    cmd_smooth_decay: float = 0.6

    @classmethod
    def from_dict(cls, d: dict) -> "PidConfig":
        d = d or {}
        out = cls()
        for f in cls.__dataclass_fields__:               # noqa: SLF001
            if f in d:
                setattr(out, f, type(getattr(out, f))(d[f]))
        return out


@dataclass
class MpcConfig:
    # Model + horizon.
    model: str = "dyn"              # dyn (dynamic bicycle, default) | kin_cg | kin_rear | blended
    N: int = 25                     # horizon steps (config/controller.yaml ships N=50 -> 3.0 s)
    dt: float = 0.06               # horizon step (s) -> N*dt = 1.5 s lookahead.
    # Dynamic-bicycle tyre/inertia (the dyn model). Defaults are Model-3 estimates;
    # identify from CARLA for fidelity (steady-state circular tests -> C_f, C_r).
    Cf: float = 120000.0            # front cornering stiffness (N/rad)
    Cr: float = 150000.0            # rear cornering stiffness (N/rad)
    Iz: float = 3745.0              # yaw inertia (kg m^2); ~ m*l_f*l_r
    # Dynamic-model state bounds (keep the linear-tyre QP well-conditioned).
    vx_min: float = 1.0             # vx floor (m/s) -- the model is singular at vx=0
    vy_max: float = 12.0            # |lateral velocity| bound (m/s)
    r_max: float = 2.0              # |yaw rate| bound (rad/s)
    # Cost: a contouring decomposition of the position error (see docs/mpc.md).
    #   q_lat  -- CROSS-TRACK error (stay on the path): weighted HIGH.
    #   q_lon  -- ALONG-TRACK error (longitudinal pacing): weighted LOW, so the MPC
    #             is free to choose its own speed instead of being paced by the
    #             reference. Speed is driven by q_v toward the cruise cap and bounded
    #             by the a_lat constraint -- the MPC decides curve speed itself.
    q_lat: float = 20.0           # cross-track (path-normal) error
    q_lon: float = 1.0            # along-track (path-tangent) error -- LOW on purpose
    q_psi: float = 8.0            # heading error
    q_v: float = 2.0             # speed error (pursue the cruise cap)
    r_a: float = 0.3             # accel input
    r_delta: float = 1.5         # steer input
    r_ddelta: float = 12.0        # penalty on (delta - last applied delta): kills micro-steer chatter
    q_terminal_scale: float = 5.0  # terminal cost = scale * stage state cost
    # Input bounds.
    a_max: float = 6.0
    a_min: float = -7.0
    delta_max: float = 0.7         # rad (~40 deg); 0 -> use the vehicle's physical max
    steer_rate_max: float = 10.0   # rad/s; rate-limit on the commanded front-wheel angle
    rti_iters: int = 3             # SQP-RTI iterations per solve (a few ms each, well under the 50 ms tick; >1 = robust, warm-started)
    # Speed: a constant cruise cap (not a per-curve target) plus a lateral-accel constraint
    # (|a_lat| <= a_lat_max) -- the MPC slows for curves on its own (physics, not a prescribed
    # profile). a_lat_max is the curve-aggressiveness knob; CARLA's Model-3 tyres grip hard, so it
    # can sit well above 1 g. Reference positions are still placed at the friction-limited pace so
    # braking is anticipated and the OCP stays well-conditioned.
    target_speed_kmh: float = 150.0   # cruise cap on straights
    a_lat_max: float = 10.0           # lateral accel limit (m/s^2) -> curve speed the MPC picks
    curve_speed_factor: float = 0.85  # fraction of grip-limited curve speed (1.0 = full; <1 adds lateral margin)
    reference_smooth_m: float = 2.5   # path xy smoothing (eases implicit lane changes)
    lane_change_smooth_m: float = 6.0 # EXTRA xy smoothing on lane-change arcs (gentle S -> no brake/jerk)
    a_lat_slack: float = 100.0        # soft-constraint penalty on a_lat violation
    v_min: float = 3.0                # speed floor (m/s)
    a_long_acc: float = 4.0           # forward accel shaping the reachable reference pace
    a_long_dec: float = 7.0           # braking shaping the reference pace (pre-brake before curves)
    # Analytic actuation map (a [m/s^2] -> throttle/brake).
    a_max_throttle: float = 6.0
    a_max_brake: float = 8.0
    v_max: float = 60.0            # OCP/actuation speed ceiling (m/s)

    @classmethod
    def from_dict(cls, d: dict) -> "MpcConfig":
        d = d or {}
        out = cls()
        for f in cls.__dataclass_fields__:               # noqa: SLF001
            if f in d:
                setattr(out, f, type(getattr(out, f))(d[f]))
        return out


@dataclass
class MpccConfig:
    """Model Predictive Contouring Control parameters (controller:=mpcc).

    Blended dynamic bicycle plus a path-progress state theta; the car maximises
    progress while a per-stage v_theta target (the friction-limited curve speed)
    makes it slow for curves -- a pragmatic feed-forward, since emergent grip-limited
    speed needs a Pacejka tyre. Reuses ReferencePath for the geometry spline and
    v-profile, so it carries the same speed-profile fields.
    """
    N: int = 30                     # horizon steps (config/controller.yaml ships N=50 -> 2.5 s)
    dt: float = 0.05                  # N*dt = 1.5 s lookahead
    Cf: float = 120000.0              # tyre/inertia (same as the dyn MPC)
    Cr: float = 150000.0
    Iz: float = 3745.0
    vx_min: float = 1.0               # state bounds (keep the linear-tyre QP conditioned)
    vy_max: float = 12.0
    r_max: float = 2.0
    q_c: float = 20.0                 # contouring (cross-track) -- keep ON the path
    q_l: float = 2.0                  # lag (theta <-> real arc length)
    r_a: float = 0.3
    r_delta: float = 4.0              # raised vs the MPC -- damps MPCC progress chatter
    r_ddelta: float = 40.0            # steer-change penalty -- the main anti-wobble knob
    q_vtheta: float = 1.5             # track the progress-speed target (gentle -> less jitter)
    q_vx: float = 1.5                 # track actual speed to the same target
    a_max: float = 6.0
    a_min: float = -7.0
    delta_max: float = 0.7
    v_theta_max: float = 60.0         # progress-rate bound (m/s)
    steer_rate_max: float = 10.0
    # Shallow actuation smoothing: exponentially-weighted average over the last N (accel, steer)
    # commands (newest weighted most). MPCC is already smooth, so keep n small. n<=1 = off.
    cmd_smooth_n: int = 0
    cmd_smooth_decay: float = 0.4
    rti_iters: int = 3
    a_grip: float = 16.0              # friction-circle radius (m/s^2), SOFT safety backstop (> a_lat_max)
    grip_slack: float = 100.0
    # v_theta target = sqrt(a_lat_max/|kappa|) capped at target_speed_kmh (friction-limited).
    # a_lat_max is the "corner this hard" knob, a budget kept just under CARLA's Model-3
    # measured grip (~1.6 g) so MPCC carries principled curve speed; Pacejka + tyre ID
    # would set it exactly (then the curve speed is fully emergent).
    target_speed_kmh: float = 175.0   # cruise cap on straights
    a_lat_max: float = 14.0           # grip budget (~1.4 g), under CARLA M3's ~1.6 g grip
    curve_speed_factor: float = 0.85  # fraction of grip-limited curve speed (1.0 = full; <1 adds lateral margin)
    reference_smooth_m: float = 2.5
    lane_change_smooth_m: float = 6.0 # EXTRA xy smoothing on lane-change arcs (gentle S -> no brake/jerk)
    v_min: float = 3.0
    a_long_acc: float = 4.0
    a_long_dec: float = 7.0
    v_max: float = 60.0
    a_max_throttle: float = 6.0
    a_max_brake: float = 8.0
    v_blend: float = 4.0              # kinematic<->dynamic blend speed (low-speed launch stability)

    @classmethod
    def from_dict(cls, d: dict) -> "MpccConfig":
        d = d or {}
        out = cls()
        for f in cls.__dataclass_fields__:               # noqa: SLF001
            if f in d:
                setattr(out, f, type(getattr(out, f))(d[f]))
        return out


@dataclass
class ControllerConfig:
    pid: PidConfig
    pidpp: PidConfig
    mpc: MpcConfig
    mpcc: MpccConfig

    @classmethod
    def from_dict(cls, d: dict) -> "ControllerConfig":
        d = d or {}
        # PID++ inherits the baseline PID gains/policy, then the `pidpp` section overrides
        # (e.g. lat_kff, steer_rate_max) -- so the two share a baseline and differ only by the add-ons.
        return cls(pid=PidConfig.from_dict(d.get("pid", {})),
                   pidpp=PidConfig.from_dict({**d.get("pid", {}), **d.get("pidpp", {})}),
                   mpc=MpcConfig.from_dict(d.get("mpc", {})),
                   mpcc=MpccConfig.from_dict(d.get("mpcc", {})))


def load_controller_config(path: str | Path) -> ControllerConfig:
    return ControllerConfig.from_dict(_load_yaml(path) or {})


def _summary(vehicle_yaml: str, carla_yaml: str) -> int:
    """Print a human-readable summary -- used as a quick offline sanity check."""
    v = load_vehicle_config(vehicle_yaml)
    c = load_carla_config(carla_yaml)
    print(f"Vehicle : {v.blueprint}  role_name={v.role_name}  base_frame={v.base_frame}")
    print(f"Carla   : map={c.map}  sync={c.synchronous_mode}  dt={c.fixed_delta_seconds}s "
          f"({1.0 / c.fixed_delta_seconds:.0f} Hz)")
    print(f"Route   : mode={c.route_mode}  loop={c.route_loop}  "
          f"file={c.route_file or '-'}  target_speed_kmh={c.target_speed_kmh or 'auto'}")
    print(f"Sensors ({len(v.sensors)} total, {len(v.enabled_sensors())} enabled):")
    for s in v.sensors:
        flag = "on " if s.enabled else "off"
        ros = "ros" if s.ros else "   "
        print(f"  [{flag}|{ros}] {s.name:<16} {s.blueprint:<32} "
              f"@({s.mount.x:+.2f},{s.mount.y:+.2f},{s.mount.z:+.2f})")
    return 0


if __name__ == "__main__":
    import sys

    # this module lives at <pkg_root>/carla_mpc_bringup/core/config_loader.py;
    # config/ is at <pkg_root>/config -> three parents up.
    here = Path(__file__).resolve().parent.parent.parent / "config"
    vy = sys.argv[1] if len(sys.argv) > 1 else str(here / "vehicle.yaml")
    cy = sys.argv[2] if len(sys.argv) > 2 else str(here / "carla.yaml")
    raise SystemExit(_summary(vy, cy))
