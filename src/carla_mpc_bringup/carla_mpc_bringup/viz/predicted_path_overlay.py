"""Project the MPC predicted and reference paths into the front-camera image.

Subscribe to the driver-view camera, the ego odometry, and the MPC horizon /
reference paths; draw the paths onto each frame and republish the annotated
image for Foxglove.

Subscribes
    /carla/<ego>/<cam>/image          sensor_msgs/Image   driver-view camera
    /carla/<ego>/gt/odom              nav_msgs/Odometry   ego pose in gt_map
    /controller/predicted_path        nav_msgs/Path       MPC horizon (orange)
    /controller/reference_path_3d     nav_msgs/Path       route with terrain z (blue)
Publishes
    /controller/camera_overlay/image  sensor_msgs/Image   rgb8 with paths drawn

Projection uses explicit conventions rather than cv_bridge/TF optical guesswork:
world(gt_map) -> ego body (from gt/odom) -> camera link (from the camera mount in
vehicle.yaml) -> camera optical (REP-103 link -> optical) -> pixels via K. This
matches how gt_pose_publisher reports the ego pose (at the vehicle origin) and how
the rig is authored, so the overlay lines up.

Intrinsics K come from the camera `fov` + resolution in vehicle.yaml (CARLA's exact
pinhole: fx = fy = W / (2 tan(fov/2)), cx = W/2, cy = H/2), so there is no
/camera_info dependency that QoS/latching could silently break.

The camera image is heavier than odom and arrives later, so projecting world paths
with the latest ego pose would draw them through a camera pose from a different
time, making the overlay slide and jitter. Odom is buffered by timestamp and the
ego pose interpolated to the image stamp (and the predicted path nearest that
stamp chosen). All stamps are sim time (use_sim_time), so they are comparable.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from carla_mpc_bringup.core.config_loader import load_vehicle_config
from carla_mpc_bringup.core.frame_conventions import (
    R_BODY_TO_OPTICAL,
    mount_to_matrix,
    quaternion_to_matrix,
)

# Predicted orange (#ff9100) matches the 3D-scene horizon; reference blue (#2780ff) is the overlay's own convention.
COL_PRED = (255, 145, 0)     # orange  (#ff9100)
COL_REF = (39, 128, 255)     # blue    (#2780ff)


AHEAD_M = 200.0          # how much of the reference path to project ahead of the vehicle


def _window_ahead(pts, ego_xy, ahead=AHEAD_M):
    """Trim a path to the window from just behind the ego forward ~`ahead` metres.

    Keep one already-passed point (so the line into the current position stays),
    drop everything earlier, and cut the front at `ahead` metres ahead. `ego_xy`
    is the ego (x, y) in the same frame as ``pts[:, :2]``.
    """
    if pts is None or len(pts) < 2:
        return pts
    pts = np.asarray(pts, dtype=float)
    i0 = int(np.argmin(np.sum((pts[:, :2] - np.asarray(ego_xy)) ** 2, axis=1)))   # nearest = current spot
    start = max(0, i0 - 1)                                                         # keep one passed point
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts[start:, :2], axis=0), axis=1))]
    end = start + int(np.searchsorted(seg, ahead)) + 1
    return pts[start:end]


def _densify_path(pts, step=0.5):
    """Interpolate a 3D path to <= `step`-metre spacing (the OCP horizon goes sparse at speed)."""
    if pts is None or len(pts) < 2:
        return pts
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = float(np.linalg.norm(b - a))
        n = max(1, int(d / step))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.asarray(out)


def draw_paths_on_frame(rgb, ego_R, ego_t, ref_xyz, pred_xy, R_bc, t_bc, K,
                        col_ref=COL_REF, col_pred=COL_PRED):
    """Project and draw the reference (blue) and predicted (orange) paths onto an RGB frame, in place.

    The in-process twin of PredictedPathOverlay's projection, so control_node can
    overlay the benchmark camera.mp4 without the separate overlay node. A no-op on
    cv2-less builds.

    Parameters
    ----------
    rgb : np.ndarray
        HxWx3 RGB frame, drawn on in place.
    ego_R, ego_t : np.ndarray
        ROS world<-body rotation and ego position.
    ref_xyz : array_like
        Reference path (Nx3) in CARLA coords.
    pred_xy : array_like
        Predicted OCP horizon as a list of (x, y) in CARLA coords.
    R_bc, t_bc, K : np.ndarray
        Camera rig: link-in-body rotation/translation and pinhole intrinsics.
    """
    if cv2 is None or rgb is None:
        return rgb
    h, w = rgb.shape[:2]

    def _project(pts_world):                         # ROS-world pts -> Nx2 px, only in front of cam
        body = (pts_world - ego_t) @ ego_R
        cam = (body - t_bc) @ R_bc
        opt = cam @ R_BODY_TO_OPTICAL.T
        z = opt[:, 2]
        keep = z > 0.15
        if not np.any(keep):
            return np.zeros((0, 2), dtype=np.int32)
        opt, z = opt[keep], z[keep]
        u = K[0, 0] * opt[:, 0] / z + K[0, 2]
        v = K[1, 1] * opt[:, 1] / z + K[1, 2]
        return np.stack([u, v], axis=1).astype(np.int32)

    def _draw(pts_carla, color, thickness):
        if pts_carla is None or len(pts_carla) < 2:
            return
        pts = _densify_path(np.asarray(pts_carla, dtype=float)[:, :3])
        pts[:, 1] = -pts[:, 1]                        # CARLA (Y right) -> ROS (Y left)
        px = _project(pts)
        if px.shape[0] >= 2:
            cv2.polylines(rgb, [px.reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)
            for (u, v) in px:
                if 0 <= u < w and 0 <= v < h:
                    cv2.circle(rgb, (int(u), int(v)), max(2, thickness), color, -1)

    ref_full = np.asarray(ref_xyz, float) if (ref_xyz is not None and len(ref_xyz)) else None
    if ref_full is not None:
        # Next ~200 m only, dropping passed points (ego_t is ROS -> CARLA y = -ros y).
        _draw(_window_ahead(ref_full, (ego_t[0], -ego_t[1])), col_ref, 3)
    if pred_xy is not None and len(pred_xy):
        pr = np.asarray(pred_xy, dtype=float)
        # Lift the 2D OCP horizon to the terrain via the nearest reference waypoint (not z=0) so the
        # predicted path follows the ground; z=0 sinks it below/off the road on sloped terrain.
        if ref_full is not None:
            d = (pr[:, None, 0] - ref_full[None, :, 0]) ** 2 + (pr[:, None, 1] - ref_full[None, :, 1]) ** 2
            zs = ref_full[d.argmin(axis=1), 2]
        else:
            zs = np.full(len(pr), 0.3)
        _draw(np.column_stack([pr[:, 0], pr[:, 1], zs]), col_pred, 4)
    return rgb


def _default_vehicle_config() -> str:
    from pathlib import Path
    try:
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory("carla_mpc_bringup"))
    except Exception:
        base = Path(__file__).resolve().parent.parent
    return str(base / "config" / "vehicle.yaml")


def _image_to_rgb(msg: Image) -> np.ndarray:
    """Decode a sensor_msgs/Image to an HxWx3 uint8 RGB array (CARLA's common encodings)."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc, 3)
    img = buf.reshape(msg.height, msg.step // 1)[:, : msg.width * ch].reshape(
        msg.height, msg.width, ch)
    if enc in ("bgr8", "bgra8"):
        img = img[:, :, [2, 1, 0]]      # BGR(A) -> RGB
    elif enc in ("rgba8",):
        img = img[:, :, :3]
    elif enc == "mono8":
        img = np.repeat(img, 3, axis=2)
    out = np.ascontiguousarray(img[:, :, :3])
    # frombuffer(bytes) is read-only and ascontiguousarray won't copy an already-
    # contiguous array (image with no row padding), so cv2 can't draw on it; force
    # a writable buffer.
    return out if out.flags.writeable else out.copy()


def _rgb_to_image(rgb: np.ndarray, header) -> Image:
    """Wrap an HxWx3 RGB array in an rgb8 sensor_msgs/Image with the given header."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


class PredictedPathOverlay(Node):
    """ROS2 node that overlays the MPC predicted and reference paths on the camera feed."""

    def __init__(self):
        super().__init__("predicted_path_overlay")
        self.declare_parameter("camera", "cam_front")
        self.declare_parameter("ego", "hero")
        self.declare_parameter("vehicle_config", _default_vehicle_config())
        cam = self.get_parameter("camera").get_parameter_value().string_value
        ego = self.get_parameter("ego").get_parameter_value().string_value
        veh_cfg_path = self.get_parameter("vehicle_config").get_parameter_value().string_value

        if cv2 is None:
            self.get_logger().error("opencv (cv2) not importable; overlay disabled.")

        # Camera mount (vehicle origin -> camera link), from vehicle.yaml.
        veh = load_vehicle_config(veh_cfg_path)
        sensor = veh.find(cam)
        if sensor is None:
            self.get_logger().warn(f"camera '{cam}' not in vehicle.yaml; using x=1.2,z=1.5.")
            from carla_mpc_bringup.core.frame_conventions import Mount
            mount = Mount(x=1.2, z=1.5)
        else:
            mount = sensor.mount
        T_b_cam = mount_to_matrix(mount)          # 4x4 body<-? : camera link in body
        self._R_bc = T_b_cam[:3, :3]
        self._t_bc = T_b_cam[:3, 3]

        # Intrinsics from the camera config (CARLA pinhole) -- no /camera_info needed.
        attrs = sensor.attributes if sensor is not None else {}
        W = int(attrs.get("image_size_x", 960))
        H = int(attrs.get("image_size_y", 540))
        fov = float(attrs.get("fov", 90.0))
        fx = W / (2.0 * math.tan(math.radians(fov) / 2.0))
        self._K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
        self.get_logger().info(f"intrinsics from config: {W}x{H} fov={fov} -> fx={fx:.1f}")

        # Time-stamped buffers so the ego pose / prediction can be matched to the
        # image's capture time rather than the latest sample.
        self._odom = deque(maxlen=400)     # (t, quat[x,y,z,w], tvec[3])
        self._preds = deque(maxlen=60)     # (t, Nx3 world points)
        self._ref = None                   # reference is world-fixed, so latest is fine
        self._lag_log = 0.0

        self.create_subscription(Odometry, f"/carla/{ego}/gt/odom",
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PathMsg, "/controller/predicted_path",
                                 self._on_pred, 10)
        self.create_subscription(PathMsg, "/controller/reference_path_3d",   # 3D (terrain) for projection
                                 lambda m: setattr(self, "_ref", self._path_xyz(m)), 10)
        self.create_subscription(Image, f"/carla/{ego}/{cam}/image",
                                 self._on_image, qos_profile_sensor_data)
        self._pub = self.create_publisher(Image, "/controller/camera_overlay/image", 10)
        self.get_logger().info(
            f"predicted_path_overlay: /carla/{ego}/{cam}/image -> /controller/camera_overlay/image")

    @staticmethod
    def _stamp(header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    @staticmethod
    def _path_xyz(msg: PathMsg) -> np.ndarray:
        if not msg.poses:
            return np.zeros((0, 3))
        return np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                         for p in msg.poses])

    def _on_odom(self, msg: Odometry):
        o, p = msg.pose.pose.orientation, msg.pose.pose.position
        self._odom.append((self._stamp(msg.header),
                           np.array([o.x, o.y, o.z, o.w]),
                           np.array([p.x, p.y, p.z])))

    def _on_pred(self, msg: PathMsg):
        self._preds.append((self._stamp(msg.header), self._path_xyz(msg)))

    def _ego_pose_at(self, t: float):
        """Interpolate (R world<-body, t) to time `t` from the odom buffer."""
        if not self._odom:
            return None
        if t <= self._odom[0][0]:
            q, tv = self._odom[0][1], self._odom[0][2]
        elif t >= self._odom[-1][0]:
            q, tv = self._odom[-1][1], self._odom[-1][2]
        else:
            i = 0
            while i + 1 < len(self._odom) and self._odom[i + 1][0] < t:
                i += 1
            t0, q0, p0 = self._odom[i]
            t1, q1, p1 = self._odom[i + 1]
            a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            tv = (1 - a) * p0 + a * p1
            if float(np.dot(q0, q1)) < 0.0:        # nearest-arc nlerp
                q1 = -q1
            q = (1 - a) * q0 + a * q1
            q /= (np.linalg.norm(q) + 1e-12)
        return quaternion_to_matrix(q[0], q[1], q[2], q[3]), tv

    def _pred_at(self, t: float) -> np.ndarray | None:
        """Predicted path whose stamp is nearest `t` (the plan at the image time)."""
        if not self._preds:
            return None
        return min(self._preds, key=lambda e: abs(e[0] - t))[1]

    def _project(self, pts_world, ego_R, ego_t):
        """Project world (gt_map) points to Nx2 pixel coords, keeping only those in front of the cam."""
        body = (pts_world - ego_t) @ ego_R                      # = R^T (p - t)
        cam = (body - self._t_bc) @ self._R_bc                  # = R_bc^T (body - t_bc)
        opt = cam @ R_BODY_TO_OPTICAL.T                          # link -> optical
        z = opt[:, 2]
        keep = z > 0.15           # cull only points behind/at the lens; 0.5 culls the nearest
                                  # in-front waypoint, dropping the path one waypoint too early
        if not np.any(keep):
            return np.zeros((0, 2), dtype=int)
        opt, z = opt[keep], z[keep]
        u = self._K[0, 0] * opt[:, 0] / z + self._K[0, 2]
        v = self._K[1, 1] * opt[:, 1] / z + self._K[1, 2]
        return np.stack([u, v], axis=1).astype(np.int32)

    @staticmethod
    def _densify(pts, step=0.5):
        """Interpolate the 3D path to <= `step`-metre spacing to fill the near field and smooth the line.

        The OCP horizon points are ~v*dt apart (~1.1 m at 80 km/h), so at lane-change
        speed the projected dots go sparse and the nearest ones fall behind the camera,
        leaving the path gappy and seeming to start late.
        """
        if pts is None or len(pts) < 2:
            return pts
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            d = float(np.linalg.norm(b - a))
            n = max(1, int(d / step))
            for k in range(1, n + 1):
                out.append(a + (b - a) * (k / n))
        return np.asarray(out)

    def _draw(self, img, pts_world, ego_R, ego_t, color, thickness):
        if cv2 is None or pts_world is None or pts_world.shape[0] < 2:
            return
        h, w = img.shape[:2]
        pts_world = self._densify(pts_world)       # fill the near field (sparse at lane-change speed)
        px = self._project(pts_world, ego_R, ego_t)
        if px.shape[0] >= 2:
            cv2.polylines(img, [px.reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)
            for (u, v) in px:
                if 0 <= u < w and 0 <= v < h:
                    cv2.circle(img, (int(u), int(v)), max(2, thickness), color, -1)

    def _on_image(self, msg: Image):
        rgb = _image_to_rgb(msg)
        t_img = self._stamp(msg.header)
        pose = self._ego_pose_at(t_img)             # ego pose at the image time
        if pose is not None:
            ego_R, ego_t = pose
            ref_win = _window_ahead(self._ref, ego_t[:2]) if self._ref is not None else None
            self._draw(rgb, ref_win, ego_R, ego_t, COL_REF, 3)          # reference (blue, next ~200 m)
            self._draw(rgb, self._pred_at(t_img), ego_R, ego_t, COL_PRED, 4)  # prediction (orange)
            # Periodically report the image-vs-odom lag just compensated for.
            if self._odom:
                lag = (self._odom[-1][0] - t_img) * 1e3
                if abs(lag - self._lag_log) > 5.0:
                    if abs(lag) > 500.0:   # implausible -> stamps likely on different clocks
                        self.get_logger().warn(
                            f"image-vs-odom lag = {lag:.0f} ms is implausible; image and "
                            "odom stamps may be on different clocks (check use_sim_time / "
                            "CARLA sensor stamping) -- overlay falls back to nearest pose.")
                    else:
                        self.get_logger().info(f"image lag vs latest odom = {lag:.0f} ms (compensated)")
                    self._lag_log = lag
        self._pub.publish(_rgb_to_image(rgb, msg.header))


def main() -> int:
    rclpy.init()
    node = PredictedPathOverlay()
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
