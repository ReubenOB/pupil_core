"""
eye_tracking_node.py
--------------------
ROS 2 node that subscribes to the raw eye-camera Image streams published by
``pupil_camera_node`` and runs a lightweight online pupil-detection + gaze-
estimation pipeline entirely within ROS 2 — without requiring a running Pupil
Capture instance.

This node is designed to be used *instead of* ``pupil_gaze_node`` when you
want a fully standalone ROS 2 pipeline.  If you already have Pupil Capture
running you should use ``pupil_gaze_node`` instead (it receives the
high-quality tracking results directly from Pupil's optimised C++ detector).

Algorithm overview
------------------
1.  Subscribe to ``/pupil/eye0/image_raw`` and ``/pupil/eye1/image_raw``.
2.  For each eye frame, detect the pupil using a simple 2-D detector:
      - Convert to greyscale.
      - Apply Canny edge detection.
      - Fit an ellipse to the largest blob / contour.
3.  From the two detected pupil positions estimate a gaze direction using a
    pre-calibrated simple linear model (default: identity mapping of the
    mean normalised position).
4.  Publish:
      ``/pupil/eye_tracking/pupil/eye0``  (pupil_ros2_msgs/PupilDatum)
      ``/pupil/eye_tracking/pupil/eye1``  (pupil_ros2_msgs/PupilDatum)
      ``/pupil/eye_tracking/gaze``        (pupil_ros2_msgs/GazeStamped)
      ``/pupil/eye_tracking/gaze/point``  (geometry_msgs/PointStamped)

Parameters
----------
  frame_id_world        (string, default "pupil_world")
  min_blob_area         (int,    default 100)   — minimum pupil contour area
  max_blob_area         (int,    default 10000) — maximum pupil contour area
  canny_threshold1      (int,    default 20)
  canny_threshold2      (int,    default 60)
  use_binocular_gaze    (bool,   default true)  — average both eyes; false = eye0 only
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from pupil_ros2_msgs.msg import GazeStamped, PupilDatum


class EyeTrackingNode(Node):
    """Standalone online gaze estimation from raw eye-camera Image streams."""

    def __init__(self) -> None:
        super().__init__("pupil_eye_tracking_node")

        # ---- parameters -----------------------------------------------
        self.declare_parameter("frame_id_world", "pupil_world")
        self.declare_parameter("min_blob_area", 100)
        self.declare_parameter("max_blob_area", 10000)
        self.declare_parameter("canny_threshold1", 20)
        self.declare_parameter("canny_threshold2", 60)
        self.declare_parameter("use_binocular_gaze", True)

        self._frame_id = self.get_parameter("frame_id_world").value
        self._min_area = self.get_parameter("min_blob_area").value
        self._max_area = self.get_parameter("max_blob_area").value
        self._canny_t1 = self.get_parameter("canny_threshold1").value
        self._canny_t2 = self.get_parameter("canny_threshold2").value
        self._binocular = self.get_parameter("use_binocular_gaze").value

        # ---- bridge ---------------------------------------------------
        self._bridge = CvBridge()

        # ---- state: latest pupil detections ---------------------------
        self._pupil0: Optional[PupilDatum] = None
        self._pupil1: Optional[PupilDatum] = None

        # ---- subscribers ----------------------------------------------
        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Image, "/pupil/eye0/image_raw", self._eye0_cb, qos)
        self.create_subscription(Image, "/pupil/eye1/image_raw", self._eye1_cb, qos)

        # ---- publishers -----------------------------------------------
        self._pub_pd0 = self.create_publisher(PupilDatum, "/pupil/eye_tracking/pupil/eye0", qos)
        self._pub_pd1 = self.create_publisher(PupilDatum, "/pupil/eye_tracking/pupil/eye1", qos)
        self._pub_gaze = self.create_publisher(GazeStamped, "/pupil/eye_tracking/gaze", qos)
        self._pub_point = self.create_publisher(PointStamped, "/pupil/eye_tracking/gaze/point", qos)

        self.get_logger().info("EyeTrackingNode ready — waiting for eye images")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _eye0_cb(self, msg: Image) -> None:
        datum = self._detect_pupil(msg, eye_id=0)
        if datum is not None:
            self._pupil0 = datum
            self._pub_pd0.publish(datum)
            if not self._binocular:
                self._publish_gaze_from_single(datum)
            else:
                self._try_publish_binocular_gaze()

    def _eye1_cb(self, msg: Image) -> None:
        datum = self._detect_pupil(msg, eye_id=1)
        if datum is not None:
            self._pupil1 = datum
            self._pub_pd1.publish(datum)
            if self._binocular:
                self._try_publish_binocular_gaze()

    # ------------------------------------------------------------------
    # Pupil detection
    # ------------------------------------------------------------------

    def _detect_pupil(self, msg: Image, eye_id: int) -> Optional[PupilDatum]:
        """
        Detect the pupil in an eye-camera frame.

        Returns a :class:`PupilDatum` on success or ``None`` if the pupil
        could not be reliably located.
        """
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"CvBridge error (eye {eye_id}): {exc}")
            return None

        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # Edge detection
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blurred, self._canny_t1, self._canny_t2)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Pick the contour whose area is within [min, max] and is the largest
        best = None
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self._min_area <= area <= self._max_area and area > best_area:
                best = cnt
                best_area = area

        if best is None or len(best) < 5:
            return None

        # Fit ellipse
        ellipse = cv2.fitEllipse(best)
        (cx, cy), (minor, major), _ = ellipse

        # Normalise to [0,1] with origin at bottom-left (Pupil convention)
        norm_x = cx / w
        norm_y = 1.0 - (cy / h)
        diameter = (minor + major) / 2.0

        # Confidence heuristic: how ellipse-like is the contour?
        hull_area = cv2.contourArea(cv2.convexHull(best))
        confidence = min(1.0, best_area / hull_area) if hull_area > 0 else 0.0

        datum = PupilDatum()
        datum.header.stamp = msg.header.stamp
        datum.header.frame_id = msg.header.frame_id
        datum.eye_id = eye_id
        datum.confidence = float(confidence)
        datum.norm_pos_x = float(norm_x)
        datum.norm_pos_y = float(norm_y)
        datum.diameter = float(diameter)
        datum.method = "2d ros"
        datum.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        return datum

    # ------------------------------------------------------------------
    # Gaze estimation
    # ------------------------------------------------------------------

    def _try_publish_binocular_gaze(self) -> None:
        """Publish a gaze point averaged from both eyes when both are fresh."""
        if self._pupil0 is None or self._pupil1 is None:
            return
        now = self.get_clock().now().to_msg()
        confidence = (self._pupil0.confidence + self._pupil1.confidence) / 2.0
        nx = (self._pupil0.norm_pos_x + self._pupil1.norm_pos_x) / 2.0
        ny = (self._pupil0.norm_pos_y + self._pupil1.norm_pos_y) / 2.0
        self._publish_gaze(now, nx, ny, confidence, [self._pupil0, self._pupil1])

    def _publish_gaze_from_single(self, datum: PupilDatum) -> None:
        now = self.get_clock().now().to_msg()
        self._publish_gaze(now, datum.norm_pos_x, datum.norm_pos_y, datum.confidence, [datum])

    def _publish_gaze(self, stamp, nx: float, ny: float, confidence: float, base_data) -> None:
        msg = GazeStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.confidence = confidence
        msg.norm_pos_x = nx
        msg.norm_pos_y = ny
        msg.has_3d_gaze_point = False
        msg.timestamp = stamp.sec + stamp.nanosec * 1e-9
        msg.base_data.extend(base_data)
        self._pub_gaze.publish(msg)

        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = self._frame_id
        pt.point.x = nx
        pt.point.y = ny
        pt.point.z = 0.0
        self._pub_point.publish(pt)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EyeTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
