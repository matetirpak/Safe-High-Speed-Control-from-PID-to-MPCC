"""Coordinate-frame conversions: Carla <-> ROS body (REP-103) <-> camera optical.

Dependency-light (numpy only, no carla / no rclpy) so it can be unit-tested and
run offline without a live simulator.

Three frames are in play:

    Carla world/actor : left-handed,  X fwd,   Y RIGHT, Z up,  rotations in degrees
    ROS body          : right-handed, X fwd,   Y left,  Z up,  radians (REP-103)
    Camera optical    : right-handed, X right, Y down,  Z FWD, radians

Sensor mounts in ``vehicle.yaml`` are authored in the ROS body convention
(X forward, Y left, Z up, angles in degrees for readability) and converted to
Carla's left-handed frame at spawn time.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Fixed rotation from a REP-103 body frame to a camera-optical frame.
#   optical_X = -body_Y, optical_Y = -body_Z, optical_Z = body_X
R_BODY_TO_OPTICAL = np.array(
    [[0.0, -1.0, 0.0],
     [0.0, 0.0, -1.0],
     [1.0, 0.0, 0.0]],
    dtype=float,
)


@dataclass
class Mount:
    """Sensor pose relative to the vehicle origin, in ROS body convention.

    Translation in metres, rotation in degrees (roll about X, pitch about Y,
    yaw about Z), applied in the ZYX intrinsic order used throughout ROS.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_dict(cls, d: dict | None) -> "Mount":
        d = d or {}
        return cls(
            x=float(d.get("x", 0.0)),
            y=float(d.get("y", 0.0)),
            z=float(d.get("z", 0.0)),
            roll=float(d.get("roll", 0.0)),
            pitch=float(d.get("pitch", 0.0)),
            yaw=float(d.get("yaw", 0.0)),
        )


def euler_deg_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert ROS RPY (degrees) to a 3x3 rotation, ZYX intrinsic (R = Rz @ Ry @ Rx)."""
    r, p, y = (math.radians(roll), math.radians(pitch), math.radians(yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def make_transform(translation: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Assemble a 4x4 homogeneous transform from a translation 3-vector and a 3x3 rotation."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def mount_to_matrix(mount: Mount) -> np.ndarray:
    """Convert a ROS-body mount to the 4x4 transform of the sensor frame in the vehicle frame."""
    return make_transform(
        np.array([mount.x, mount.y, mount.z]),
        euler_deg_to_matrix(mount.roll, mount.pitch, mount.yaw),
    )


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a quaternion (x, y, z, w) to a 3x3 rotation."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def ros_mount_to_carla(mount: Mount) -> dict:
    """Convert a ROS-body mount to Carla left-handed spawn parameters.

    Carla is left-handed (Y points right), so Y and pitch/yaw are negated.
    Angles stay in degrees because ``carla.Rotation`` is authored in degrees.
    """
    return {
        "x": mount.x,
        "y": -mount.y,
        "z": mount.z,
        "roll": mount.roll,
        "pitch": -mount.pitch,
        "yaw": -mount.yaw,
    }
