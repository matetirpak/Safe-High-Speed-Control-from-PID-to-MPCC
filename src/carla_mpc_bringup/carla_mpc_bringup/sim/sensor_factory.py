"""Build Carla sensor blueprints and transforms from a SensorConfig.

Import ``carla`` lazily (guarded) so the rest of the package -- config loading,
frame math -- works without the Carla wheel installed.

Exposes helpers to construct a spawn-ready blueprint, derive the mount
transform, opt a spawned sensor into Carla's native ROS2 publisher, and spawn a
sensor attached to a vehicle.
"""
from __future__ import annotations

import logging

from carla_mpc_bringup.core.config_loader import SensorConfig
from carla_mpc_bringup.core.frame_conventions import ros_mount_to_carla

logger = logging.getLogger(__name__)

try:
    import carla
except ImportError:  # pragma: no cover - only needed at spawn time
    carla = None


def _attr_str(value) -> str:
    """Render a Python value in Carla's ``set_attribute`` string spelling."""
    if isinstance(value, bool):
        return "True" if value else "False"
    return str(value)


def build_transform(sensor: SensorConfig):
    """Convert a ROS-body mount into a left-handed ``carla.Transform``."""
    c = ros_mount_to_carla(sensor.mount)
    return carla.Transform(
        carla.Location(x=c["x"], y=c["y"], z=c["z"]),
        carla.Rotation(roll=c["roll"], pitch=c["pitch"], yaw=c["yaw"]),
    )


def build_blueprint(bp_lib, sensor: SensorConfig):
    """Find the blueprint, set role/ROS names and attributes, return it ready to spawn.

    Returns
    -------
    carla.ActorBlueprint
        Spawn-ready blueprint.
    """
    bp = bp_lib.find(sensor.blueprint)
    # role_name + ros_name must be set BEFORE spawn -> topic path /carla/<veh>/<sensor.name>/...
    # ros_name is a special native-publisher attribute (has_attribute() may be
    # False); set it unconditionally, matching the vehicle, to avoid // in topics.
    bp.set_attribute("role_name", sensor.name)
    bp.set_attribute("ros_name", sensor.name)

    for key, value in sensor.attributes.items():
        if not bp.has_attribute(key):
            logger.warning("Sensor %s: blueprint %s has no attribute '%s' -- skipping",
                           sensor.name, sensor.blueprint, key)
            continue
        bp.set_attribute(key, _attr_str(value))

    return bp


def enable_sensor_ros(actor, sensor: SensorConfig) -> bool:
    """Opt a spawned sensor into Carla's native ROS2 publisher.

    Returns
    -------
    bool
        True if publishing was enabled, False if disabled by config or
        unsupported by this sensor.
    """
    if not sensor.ros:
        return False
    # Some event sensors (e.g. lane_invasion, obstacle) don't support the native ROS2
    # publisher in 0.9.16 -- guard rather than crash the rig.
    if hasattr(actor, "enable_for_ros"):
        actor.enable_for_ros()
        return True
    logger.warning("%s (%s) has no enable_for_ros(); spawned without ROS publishing.",
                   sensor.name, sensor.blueprint)
    return False


def spawn_sensor(world, vehicle, sensor: SensorConfig, enable_ros: bool = True):
    """Build and spawn a sensor attached to the vehicle, opting it into ROS unless deferred.

    Parameters
    ----------
    enable_ros : bool
        When False, spawn WITHOUT publishing so the caller can let the spawn
        transient settle first -- Carla drops the ego onto the road, and the
        caller re-enables publishing via ``enable_sensor_ros()`` once the car has
        settled, so the first published frame is clean.

    Returns
    -------
    carla.Actor
        The spawned sensor actor.
    """
    bp = build_blueprint(world.get_blueprint_library(), sensor)
    transform = build_transform(sensor)
    actor = world.spawn_actor(bp, transform, attach_to=vehicle)
    ros_ok = enable_sensor_ros(actor, sensor) if enable_ros else "deferred"
    logger.info(
        "Spawned %-16s %-32s ros=%s  mount=(%.2f,%.2f,%.2f)",
        sensor.name, sensor.blueprint, ros_ok,
        sensor.mount.x, sensor.mount.y, sensor.mount.z,
    )
    return actor
