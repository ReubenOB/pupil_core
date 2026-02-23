"""
test_eye_tracking.py
--------------------
Unit tests for the standalone eye-tracking detector logic — no live ROS
instance required.

All ROS 2 packages (rclpy, sensor_msgs, cv_bridge, pupil_ros2_msgs) are
stubbed out with lightweight mocks so the tests run without a ROS 2
installation.

The pupil-detection algorithm in EyeTrackingNode._detect_pupil is exercised
by injecting synthetic OpenCV images that contain a clearly visible dark
ellipse (simulating a pupil).
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest


# ---------------------------------------------------------------------------
# ROS 2 / cv_bridge stubs
# (installed before any import of the nodes-under-test)
# ---------------------------------------------------------------------------

def _install_ros_stubs():
    """Register lightweight module stubs for all ROS 2 dependencies."""

    # ---- std_msgs -------------------------------------------------------
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg = std_msgs_msg

    class _Header:
        def __init__(self):
            self.stamp = MagicMock(sec=0, nanosec=0)
            self.frame_id = ""

    std_msgs_msg.Header = _Header
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    # ---- sensor_msgs ----------------------------------------------------
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg = sensor_msgs_msg

    class _Image:
        def __init__(self):
            self.header = _Header()
            self.data = b""
            self.width = 0
            self.height = 0
            self.encoding = ""
            self.step = 0
            self.is_bigendian = 0

    sensor_msgs_msg.Image = _Image
    sensor_msgs_msg.CameraInfo = MagicMock
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    # ---- geometry_msgs --------------------------------------------------
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg = geometry_msgs_msg
    geometry_msgs_msg.PointStamped = MagicMock
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    # ---- pupil_ros2_msgs ------------------------------------------------
    pupil_msgs = types.ModuleType("pupil_ros2_msgs")
    pupil_msgs_msg = types.ModuleType("pupil_ros2_msgs.msg")
    pupil_msgs.msg = pupil_msgs_msg

    class _PupilDatum:
        def __init__(self):
            self.header = _Header()
            self.eye_id = 0
            self.confidence = 0.0
            self.norm_pos_x = 0.0
            self.norm_pos_y = 0.0
            self.diameter = 0.0
            self.sphere_center_x = 0.0
            self.sphere_center_y = 0.0
            self.sphere_center_z = 0.0
            self.sphere_radius = 0.0
            self.timestamp = 0.0
            self.method = ""

    class _GazeStamped:
        def __init__(self):
            self.header = _Header()
            self.confidence = 0.0
            self.norm_pos_x = 0.0
            self.norm_pos_y = 0.0
            self.has_3d_gaze_point = False
            self.gaze_point_3d_x = 0.0
            self.gaze_point_3d_y = 0.0
            self.gaze_point_3d_z = 0.0
            self.base_data = []
            self.timestamp = 0.0

    pupil_msgs_msg.PupilDatum = _PupilDatum
    pupil_msgs_msg.GazeStamped = _GazeStamped
    sys.modules.setdefault("pupil_ros2_msgs", pupil_msgs)
    sys.modules.setdefault("pupil_ros2_msgs.msg", pupil_msgs_msg)

    # ---- cv_bridge ------------------------------------------------------
    class _CvBridge:
        """Minimal CvBridge using the real cv2 encode/decode path."""

        def cv2_to_imgmsg(self, img: np.ndarray, encoding: str = "bgr8"):
            msg = _Image()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = encoding
            msg.step = img.shape[1] * img.shape[2]
            # Encode as PNG bytes stored in .data
            _, buf = cv2.imencode(".png", img)
            msg.data = buf.tobytes()
            return msg

        def imgmsg_to_cv2(self, msg, desired_encoding: str = "bgr8") -> np.ndarray:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            return img

    cv_bridge_mod = types.ModuleType("cv_bridge")
    cv_bridge_mod.CvBridge = _CvBridge
    sys.modules.setdefault("cv_bridge", cv_bridge_mod)

    # ---- rclpy ----------------------------------------------------------
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = MagicMock()
    rclpy_mod.spin = MagicMock()
    rclpy_mod.shutdown = MagicMock()

    # QoS stubs
    qos_mod = types.ModuleType("rclpy.qos")
    preset = MagicMock()
    preset.value = 10  # depth
    qos_profiles = MagicMock()
    qos_profiles.SENSOR_DATA = preset
    qos_mod.QoSPresetProfiles = qos_profiles
    rclpy_mod.qos = qos_profiles

    # Node base class stub
    node_mod = types.ModuleType("rclpy.node")

    class _Node:
        def __init__(self, *a, **kw):
            pass

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value.to_msg.return_value = MagicMock(sec=0, nanosec=0)
            return clk

        def declare_parameter(self, name, default=None):
            return MagicMock()

        def get_parameter(self, name):
            defaults = {
                "frame_id_world": "pupil_world",
                "min_blob_area": 100,
                "max_blob_area": 10000,
                "canny_threshold1": 20,
                "canny_threshold2": 60,
                "use_binocular_gaze": True,
                "pupil_host": "localhost",
                "pupil_req_port": 50020,
                "publish_pupil": True,
                "publish_gaze": True,
                "min_confidence": 0.0,
                "publish_world": True,
                "publish_eye0": True,
                "publish_eye1": True,
                "frame_encoding": "jpeg",
                "frame_id_eye0": "pupil_eye0",
                "frame_id_eye1": "pupil_eye1",
            }
            m = MagicMock()
            m.value = defaults.get(name, "")
            return m

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def destroy_node(self):
            pass

    node_mod.Node = _Node
    rclpy_mod.node = node_mod

    sys.modules.setdefault("rclpy", rclpy_mod)
    sys.modules.setdefault("rclpy.node", node_mod)
    sys.modules.setdefault("rclpy.qos", qos_mod)


_install_ros_stubs()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_eye_image(width: int = 192, height: int = 192) -> np.ndarray:
    """Create a synthetic eye image with a dark ellipse on a bright background."""
    img = np.full((height, width, 3), 200, dtype=np.uint8)
    center = (width // 2, height // 2)
    axes = (30, 20)
    cv2.ellipse(img, center, axes, 0, 0, 360, (20, 20, 20), -1)
    return img


def _make_node():
    """Construct an EyeTrackingNode without a running ROS master."""
    from pupil_ros2_driver.eye_tracking_node import EyeTrackingNode
    return EyeTrackingNode()


def _to_ros_image(node, bgr: np.ndarray, eye_id: int = 0):
    """Convert a numpy image to a fake ROS Image using the node's bridge."""
    ros_img = node._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
    ros_img.header.stamp.sec = 1
    ros_img.header.stamp.nanosec = 0
    ros_img.header.frame_id = f"pupil_eye{eye_id}"
    return ros_img


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestEyeDetectionLogic:

    def test_detect_pupil_returns_datum_for_valid_image(self):
        node = _make_node()
        bgr = _make_eye_image()
        ros_img = _to_ros_image(node, bgr, eye_id=0)

        datum = node._detect_pupil(ros_img, eye_id=0)

        assert datum is not None, "Expected a PupilDatum for a valid eye image"
        assert datum.eye_id == 0
        assert 0.0 <= datum.norm_pos_x <= 1.0
        assert 0.0 <= datum.norm_pos_y <= 1.0
        assert datum.confidence > 0.0
        assert datum.diameter > 0.0
        assert datum.method == "2d ros"

    def test_detect_pupil_returns_none_for_blank_image(self):
        node = _make_node()
        bgr = np.full((192, 192, 3), 128, dtype=np.uint8)
        ros_img = _to_ros_image(node, bgr, eye_id=0)

        datum = node._detect_pupil(ros_img, eye_id=0)
        assert datum is None

    def test_norm_pos_centred_for_centred_pupil(self):
        node = _make_node()
        bgr = _make_eye_image(width=192, height=192)
        ros_img = _to_ros_image(node, bgr, eye_id=1)

        datum = node._detect_pupil(ros_img, eye_id=1)

        assert datum is not None
        assert abs(datum.norm_pos_x - 0.5) < 0.1, f"norm_pos_x={datum.norm_pos_x} expected ~0.5"
        assert abs(datum.norm_pos_y - 0.5) < 0.1, f"norm_pos_y={datum.norm_pos_y} expected ~0.5"

    def test_detect_eye1_sets_correct_id(self):
        node = _make_node()
        bgr = _make_eye_image()
        ros_img = _to_ros_image(node, bgr, eye_id=1)

        datum = node._detect_pupil(ros_img, eye_id=1)
        assert datum is not None
        assert datum.eye_id == 1

    def test_binocular_gaze_averaged(self):
        """When both eye data are present the gaze point should be their mean."""
        node = _make_node()
        bgr = _make_eye_image()

        ros0 = _to_ros_image(node, bgr, eye_id=0)
        ros1 = _to_ros_image(node, bgr, eye_id=1)

        pd0 = node._detect_pupil(ros0, eye_id=0)
        pd1 = node._detect_pupil(ros1, eye_id=1)

        assert pd0 is not None and pd1 is not None

        # Manually set the node's cached detections and trigger gaze
        node._pupil0 = pd0
        node._pupil1 = pd1

        published = []
        node._pub_gaze = MagicMock(publish=lambda m: published.append(m))
        node._pub_point = MagicMock()
        node._try_publish_binocular_gaze()

        assert published, "Expected gaze to be published"
        gaze = published[0]
        expected_x = (pd0.norm_pos_x + pd1.norm_pos_x) / 2.0
        expected_y = (pd0.norm_pos_y + pd1.norm_pos_y) / 2.0
        assert abs(gaze.norm_pos_x - expected_x) < 1e-9
        assert abs(gaze.norm_pos_y - expected_y) < 1e-9

