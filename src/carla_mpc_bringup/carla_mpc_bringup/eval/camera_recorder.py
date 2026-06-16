"""Opt-in driver-view recorder for a run (camera.mp4).

Encode frames incrementally with a cv2 VideoWriter (frame-by-frame to disk), so a run stopped
with Ctrl-C keeps the video up to the last written frame; only a quick ``.release()`` runs at
shutdown.

Record the frames seen in Foxglove: prefer the overlay stream
``/controller/camera_overlay/image`` (the driver camera with the reference and predicted
trajectories drawn on it) and fall back to the raw camera only when no overlay is published
(e.g. a headless benchmark with no overlay node). Once any overlay frame is seen, raw frames
are ignored.

cv2 (OpenCV) supplies the codec (mp4v, MJPG fallback), so no ffmpeg binary is needed. Degrades
to a no-op when cv2 is unavailable. Opt in via control_node --record-camera, which forces
rendering on.
"""
from __future__ import annotations

import threading

import numpy as np

try:
    import cv2
except Exception:        # pragma: no cover - cv2 should be present (overlay node uses it)
    cv2 = None


def carla_image_to_rgb(image) -> np.ndarray:
    """Convert a carla.Image (BGRA bytes) to an HxWx3 RGB array."""
    buf = np.frombuffer(image.raw_data, dtype=np.uint8)
    return buf.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]


def ros_image_to_rgb(msg) -> np.ndarray:
    """Convert a sensor_msgs/Image to an HxWx3 RGB array, handling CARLA's common encodings."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc, 3)
    img = buf.reshape(msg.height, msg.step)[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
    if enc in ("bgr8", "bgra8"):
        img = img[:, :, [2, 1, 0]]
    elif enc == "mono8":
        img = np.repeat(img, 3, axis=2)
    return np.ascontiguousarray(img[:, :, :3])


class CameraRecorder:
    """Incremental driver-view video writer.

    Decimate incoming overlay/raw frames to a target rate and write them straight to a cv2
    container, preferring overlay frames once any are seen. All public methods are thread-safe.
    """

    def __init__(self, path, fps: int = 12, downsample: int = 1):
        self.path = str(path)
        self.fps = int(max(1, fps))
        self.ds = int(max(1, downsample))
        self._w = None
        self._last_t = -1.0e9
        self._seen_overlay = False
        self._n = 0
        self._closed = False
        self._lock = threading.Lock()

    def _open(self, h: int, w: int) -> bool:
        import os
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)   # run dir may not exist yet
        for fourcc, ext in (("mp4v", ".mp4"), ("MJPG", ".avi")):
            path = self.path if self.path.endswith(ext) else self.path.rsplit(".", 1)[0] + ext
            wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), float(self.fps), (w, h))
            if wr.isOpened():
                self._w, self.path = wr, path
                return True
            wr.release()
        return False

    def add(self, rgb, source: str, t: float) -> None:
        """Feed one RGB frame to the writer, decimating to the target fps.

        Thread-safe: called from both the ROS spin thread (overlay) and the CARLA sensor
        thread (raw).

        Parameters
        ----------
        rgb : np.ndarray
            HxWx3 RGB frame.
        source : str
            'overlay' (preferred) or 'raw'. Raw frames are dropped once any overlay frame
            has been seen.
        t : float
            Sim timestamp in seconds, used to decimate to roughly fps.
        """
        if cv2 is None:
            return
        try:
            if source == "raw" and self._seen_overlay:
                return
            with self._lock:
                if self._closed:
                    return
                if source == "overlay":
                    self._seen_overlay = True
                if t - self._last_t < 1.0 / self.fps:
                    return
                self._last_t = t
                if self.ds > 1:
                    rgb = rgb[:: self.ds, :: self.ds]
                rgb = np.ascontiguousarray(rgb)
                if self._w is None and not self._open(rgb.shape[0], rgb.shape[1]):
                    return
                self._w.write(rgb[:, :, ::-1])          # cv2 expects BGR
                self._n += 1
        except Exception:                               # never let a callback crash the run
            pass

    @property
    def n_frames(self) -> int:
        return self._n

    def close(self) -> None:
        """Finalize the container; fast (no re-encode), so it is safe in a Ctrl-C teardown."""
        with self._lock:
            self._closed = True
            if self._w is not None:
                try:
                    self._w.release()
                except Exception:
                    pass
                self._w = None
