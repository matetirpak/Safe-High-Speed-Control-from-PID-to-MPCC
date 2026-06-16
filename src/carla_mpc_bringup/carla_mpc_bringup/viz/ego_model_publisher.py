"""Publish the ego vehicle as a MarkerArray in the `hero` frame.

Foxglove/RViz render MarkerArray natively and reliably (unlike the URDF layer,
which needs /robot_description bridging). Every marker is in frame `hero`, which
gt_pose_publisher anchors to the ground-truth pose via `gt_map -> hero` -- so the car
stands at the current ego pose and drives along the driven trajectory.

Latched (transient_local), republished slowly so late joiners (Foxglove
reconnects) always get it.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

BODY = (0.10, 0.34, 0.66, 1.0)
CABIN = (0.20, 0.55, 0.92, 0.55)
TIRE = (0.05, 0.05, 0.05, 1.0)
SENSOR = (0.95, 0.62, 0.07, 1.0)

# 90 deg about X so a CYLINDER's axis (local Z) points along the vehicle Y (axle).
WHEEL_Q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))


def _color(rgba):
    """Build a ColorRGBA from an (r, g, b, a) tuple of floats in [0, 1]."""
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = (float(v) for v in rgba)
    return c


def _marker(frame, ns, mid, mtype, pos, scale, rgba, quat=(0.0, 0.0, 0.0, 1.0)):
    """Assemble a single frame-locked Marker.

    Parameters
    ----------
    frame : str
        Frame the marker pose is resolved against (here, ``hero``).
    pos, scale : tuple of float
        Position (m) and per-axis scale (m) in the marker frame.
    rgba : tuple of float
        Color as (r, g, b, a), each in [0, 1].
    quat : tuple of float
        Orientation as (x, y, z, w); defaults to identity.

    Returns
    -------
    visualization_msgs.msg.Marker
        Marker with ``frame_locked`` set so it tracks the latest transform.
    """
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    # frame_locked: continuously re-resolve the marker pose to the LATEST `hero`
    # transform. Without this the car is drawn once at the frame's t=0 pose
    # (i.e. stuck at the origin) instead of following the moving ego pose.
    m.frame_locked = True
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
    m.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                    z=float(quat[2]), w=float(quat[3]))
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
    m.color = _color(rgba)
    return m


def build_car(frame: str) -> MarkerArray:
    """Build the ego car as a MarkerArray: body, cabin, four tires, a sensor box.

    All markers share namespace ``ego`` and live in ``frame`` so the assembled
    car follows the ground-truth ego pose anchored by that transform.
    """
    arr = MarkerArray()
    C, B = Marker.CUBE, Marker.CYLINDER
    arr.markers = [
        _marker(frame, "ego", 0, C, (0.0, 0.0, 0.55), (4.9, 1.9, 0.55), BODY),
        _marker(frame, "ego", 1, C, (-0.35, 0.0, 1.05), (2.4, 1.78, 0.55), CABIN),
        _marker(frame, "ego", 2, B, (1.4, 0.86, 0.34), (0.68, 0.68, 0.25), TIRE, WHEEL_Q),
        _marker(frame, "ego", 3, B, (1.4, -0.86, 0.34), (0.68, 0.68, 0.25), TIRE, WHEEL_Q),
        _marker(frame, "ego", 4, B, (-1.4, 0.86, 0.34), (0.68, 0.68, 0.25), TIRE, WHEEL_Q),
        _marker(frame, "ego", 5, B, (-1.4, -0.86, 0.34), (0.68, 0.68, 0.25), TIRE, WHEEL_Q),
        _marker(frame, "ego", 6, C, (1.6, 0.0, 1.5), (0.10, 0.22, 0.08), SENSOR),
    ]
    return arr


def main() -> int:
    """Run the node: publish the latched ego MarkerArray and spin until stopped."""
    rclpy.init()
    node = Node("ego_model_publisher")
    node.declare_parameter("frame_id", "hero")
    node.declare_parameter("topic", "/ego_model")
    frame = node.get_parameter("frame_id").get_parameter_value().string_value
    topic = node.get_parameter("topic").get_parameter_value().string_value

    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # latched for late joiners
    pub = node.create_publisher(MarkerArray, topic, qos)
    car = build_car(frame)
    pub.publish(car)
    node.create_timer(2.0, lambda: pub.publish(car))   # re-latch periodically
    node.get_logger().info(f"Publishing ego car model ({len(car.markers)} markers) "
                           f"on {topic} in frame '{frame}'.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
