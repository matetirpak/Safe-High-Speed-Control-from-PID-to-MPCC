"""Named environmental conditions: load `config/conditions.yaml` and apply them.

Selected by the launch `condition:=` flag, which control_node applies at startup. A
"condition" is a set of explicit ``carla.WeatherParameters`` fields (weather +
time-of-day). Applying a condition is just ``world.set_weather(...)`` — it takes
effect on the next world tick, so the running ticker shows it immediately and
nothing needs restarting.

Pure + testable: `carla` is imported lazily, so `load_conditions()` works without a
running sim (e.g. for tests or listing).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import yaml

logger = logging.getLogger(__name__)

# Numeric WeatherParameters attributes we drive from YAML. Applied via setattr
# guarded by hasattr, so version differences (dust_storm, scattering_*, ...) on
# older/newer Carla never raise.
WEATHER_FIELDS = (
    "cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
    "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
    "fog_falloff", "wetness", "scattering_intensity", "mie_scattering_scale",
    "rayleigh_scattering_scale", "dust_storm",
)


@dataclass
class Condition:
    """A named weather + time-of-day preset loaded from conditions.yaml."""

    name: str
    label: str
    params: dict = field(default_factory=dict)
    note: str = ""


def conditions_path(explicit: str | None = None) -> Path:
    """Resolve the path to conditions.yaml (install share dir, else source tree)."""
    if explicit:
        return Path(explicit)
    try:
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory("carla_mpc_bringup"))
    except Exception:
        base = Path(__file__).resolve().parent.parent
    return base / "config" / "conditions.yaml"


def load_conditions(path: str | None = None) -> list[Condition]:
    """Load conditions from conditions.yaml in file order (the switcher cycle order)."""
    p = conditions_path(path)
    data = yaml.safe_load(p.read_text()) or {}
    out: list[Condition] = []
    for entry in data.get("conditions", []):
        name = entry["name"]
        params = {k: float(v) for k, v in entry.items() if k in WEATHER_FIELDS}
        out.append(Condition(name, entry.get("label", name), params,
                             entry.get("note", "")))
    if not out:
        logger.warning("No conditions found in %s", p)
    return out


def find(conditions: list[Condition], name: str) -> Condition | None:
    """Return the condition with the given name, or None if absent."""
    return next((c for c in conditions if c.name == name), None)


def build_weather(params: dict):
    """Build a carla.WeatherParameters from a dict of field values.

    Fields unknown to the running Carla version are skipped with a warning, so
    newer YAML keys stay safe on older Carla.
    """
    import carla
    w = carla.WeatherParameters()
    for k, v in params.items():
        if hasattr(w, k):
            setattr(w, k, float(v))
        else:
            logger.warning("WeatherParameters has no '%s' (Carla version) -- skipped.", k)
    return w


def apply_condition(world, condition: Condition) -> None:
    """Set the world weather to the given condition (takes effect next tick)."""
    world.set_weather(build_weather(condition.params))
    logger.info("Condition: %s (%s)", condition.name, condition.label)


def apply_named(world, name: str, path: str | None = None) -> bool:
    """Apply a condition by name from conditions.yaml; fall back to a Carla preset.

    Empty/None name is a no-op (returns False). Returns True if a weather was set.
    """
    if not name:
        return False
    c = find(load_conditions(path), name)
    if c is not None:
        apply_condition(world, c)
        return True
    import carla
    preset = getattr(carla.WeatherParameters, name, None)
    if preset is not None:
        world.set_weather(preset)
        logger.info("Weather preset: %s", name)
        return True
    logger.warning("Unknown condition/preset '%s' -- leaving current weather.", name)
    return False
