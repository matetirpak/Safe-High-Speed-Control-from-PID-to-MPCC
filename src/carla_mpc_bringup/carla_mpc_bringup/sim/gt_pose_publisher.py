#!/usr/bin/env python3
"""Publish the ego's ground-truth pose/trajectory and the TF anchor (read-only).

Passive counterpart to control_node, which owns the world tick. Samples
``vehicle.get_transform()`` and publishes, in the ROS world frame ``gt_map``
(CARLA world with Y negated, REP-103):

    /carla/<ego>/gt/path   nav_msgs/Path        driven trajectory
    /carla/<ego>/gt/odom   nav_msgs/Odometry    instantaneous pose + body twist
    TF  gt_map -> hero                           anchors CARLA's hero->{sensors}
                                                 tree so live cloud/images render

Pose is reported at the vehicle origin -- the point the controller tracks against
the lane-centre reference. ``ego_model_publisher`` draws the car in the ``hero``
frame, so it rides this anchor.

Never ticks the world. Launch with ``use_sim_time:=true`` so stamps align with
the /clock published by CARLA's native ROS2 publisher (the server started with
``--ros2``).
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time

try:
    import carla
except ImportError:
    sys.stderr.write("ERROR: 'carla' not importable (install the Carla wheel).\n")
    sys.exit(2)

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

MAP_FRAME = "gt_map"


def euler_to_quat(roll, pitch, yaw) -> Quaternion:
    """Convert roll/pitch/yaw (rad, ZYX intrinsic) to a ROS quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def sim_time_to_rostime(t_sec: float) -> RosTime:
    """Convert a float simulation time in seconds to a builtin_interfaces Time."""
    sec = int(t_sec)
    return RosTime(sec=sec, nanosec=int((t_sec - sec) * 1e9))


class GtPublisher(Node):
    """ROS2 node that publishes the ego ground-truth path, odometry, and TF anchor."""

    def __init__(self, ros_name: str, max_poses: int = 200000):
        super().__init__("gt_pose_publisher")
        self.set_parameters([rclpy.Parameter("use_sim_time", value=True)])
        self.body_frame = ros_name        # CARLA vehicle frame ("hero")
        self.max_poses = max_poses
        ns = f"/carla/{ros_name}/gt"
        self.path_pub = self.create_publisher(PathMsg, f"{ns}/path", 10)
        self.odom_pub = self.create_publisher(Odometry, f"{ns}/odom", qos_profile_sensor_data)
        self.tf_bcaster = TransformBroadcaster(self)
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = MAP_FRAME

    def publish(self, t_sec, location, rotation_deg, velocity):
        """Append the sample to the path and publish path, odometry, and TF.

        Parameters
        ----------
        t_sec : float
            Simulation timestamp in seconds (from the CARLA snapshot).
        location : carla.Location
            Vehicle origin in CARLA world coordinates (left-handed, metres).
        rotation_deg : carla.Rotation
            Vehicle orientation in CARLA convention (degrees).
        velocity : carla.Vector3D
            World-frame linear velocity in CARLA coordinates (m/s).
        """
        stamp = sim_time_to_rostime(t_sec)
        # CARLA (left-handed, Y right, deg) -> ROS (REP-103, Y left, rad).
        px, py, pz = location.x, -location.y, location.z
        q = euler_to_quat(math.radians(rotation_deg.roll),
                          math.radians(-rotation_deg.pitch),
                          math.radians(-rotation_deg.yaw))

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = MAP_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, pz
        ps.pose.orientation = q
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses = self.path_msg.poses[-(self.max_poses // 2):]
        self.path_msg.header.stamp = stamp
        self.path_pub.publish(self.path_msg)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = MAP_FRAME
        od.child_frame_id = self.body_frame
        od.pose.pose = ps.pose
        od.twist.twist.linear.x = velocity.x
        od.twist.twist.linear.y = -velocity.y
        od.twist.twist.linear.z = velocity.z
        self.odom_pub.publish(od)

        # Anchor CARLA's rigid sensor tree (hero -> {sensors}) to the true pose so
        # the live camera data renders in 3D.
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = MAP_FRAME
        tf.child_frame_id = self.body_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = px, py, pz
        tf.transform.rotation = q
        self.tf_bcaster.sendTransform(tf)


def find_vehicle(world, role_name: str):
    """Return the live vehicle actor whose role_name matches, or None."""
    for actor in world.get_actors():
        if "vehicle." in actor.type_id and actor.attributes.get("role_name") == role_name:
            return actor
    return None


def main() -> int:
    """Connect to CARLA, wait for the ego, and stream GT until it is destroyed."""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", default=2000, type=int)
    p.add_argument("--role-name", default="hero")
    p.add_argument("--rate", default=100.0, type=float, help="GT sample rate (Hz)")
    args, _ = p.parse_known_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    rclpy.init()
    node = GtPublisher(args.role_name)

    vehicle = None
    while rclpy.ok() and vehicle is None:
        # Re-fetch the world every retry: control_node load_world()s a new episode after we
        # start, and an actor found through the stale pre-load episode handle reports
        # is_alive=False, which would exit instantly with no trajectory on a town's first run.
        world = client.get_world()
        vehicle = find_vehicle(world, args.role_name)
        if vehicle is None:
            node.get_logger().info(f"Waiting for ego role_name='{args.role_name}'...")
            time.sleep(0.5)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    interval = 1.0 / args.rate
    node.get_logger().info(f"Publishing GT for '{args.role_name}' at {args.rate:.0f} Hz.")
    try:
        ticks = 0
        while rclpy.ok():
            ticks += 1
            # Stop once the ego is destroyed so nothing moves the displayed trajectory
            # afterwards. A handle cached by another client keeps `is_alive` stale in both
            # directions, so exit only on server truth (get_actor by id == None), re-checked
            # every ~2 s; the cached `is_alive` merely triggers that server re-check.
            if (not vehicle.is_alive or ticks % 200 == 0) and world.get_actor(vehicle.id) is None:
                node.get_logger().info("Ego destroyed (run complete) -> exiting; the published "
                                       "trajectory stays where it ended.")
                break
            snap = world.get_snapshot()
            tf = vehicle.get_transform()
            # Teleport guard: after control_node destroys the ego, this client's stale handle
            # can return a garbage far-away transform while is_alive and get_actor both stay
            # stale, so the checks above never fire. Drop any implausible jump (>100 m between
            # consecutive samples) so the trajectory freezes at the route end instead of
            # flinging across the map.
            if node.path_msg.poses:
                lp = node.path_msg.poses[-1].pose.position
                if math.hypot(tf.location.x - lp.x, (-tf.location.y) - lp.y) > 100.0:
                    if not getattr(node, "_frozen_logged", False):
                        node._frozen_logged = True
                        node.get_logger().info("Implausible ego jump (destroyed handle?) -> "
                                               "suppressing; trajectory frozen at the route end.")
                    time.sleep(interval)
                    continue
            node.publish(snap.timestamp.elapsed_seconds, tf.location, tf.rotation,
                         vehicle.get_velocity())
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
