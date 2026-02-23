"""
camera_node.py
--------------
ROS 2 node that connects to Pupil Capture / Service and publishes the three
camera streams as ``sensor_msgs/msg/Image`` messages.

Published topics
----------------
  /pupil/world/image_raw          (sensor_msgs/msg/Image)
  /pupil/world/camera_info        (sensor_msgs/msg/CameraInfo)  [empty]
  /pupil/eye0/image_raw           (sensor_msgs/msg/Image)
  /pupil/eye0/camera_info         (sensor_msgs/msg/CameraInfo)  [empty]
  /pupil/eye1/image_raw           (sensor_msgs/msg/Image)
  /pupil/eye1/camera_info         (sensor_msgs/msg/CameraInfo)  [empty]

Parameters
----------
  pupil_host      (string,  default "localhost")
  pupil_req_port  (int,     default 50020)
  frame_encoding  (string,  default "jpeg") — "jpeg" or "yuv422" or "bgr"
  publish_eye0    (bool,    default true)
  publish_eye1    (bool,    default true)
  publish_world   (bool,    default true)
  frame_id_world  (string,  default "pupil_world")
  frame_id_eye0   (string,  default "pupil_eye0")
  frame_id_eye1   (string,  default "pupil_eye1")
"""

from __future__ import annotations

import threading
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from pupil_ros2_driver.pupil_driver import PupilDriver

# Pupil topic strings
TOPIC_WORLD = "frame.world"
TOPIC_EYE0 = "frame.eye.0"
TOPIC_EYE1 = "frame.eye.1"


class CameraNode(Node):
    """Streams the Pupil Core world + two eye cameras into ROS 2."""

    def __init__(self) -> None:
        super().__init__("pupil_camera_node")

        # ---- parameters -----------------------------------------------
        self.declare_parameter("pupil_host", "localhost")
        self.declare_parameter("pupil_req_port", 50020)
        self.declare_parameter("frame_encoding", "jpeg")
        self.declare_parameter("publish_world", True)
        self.declare_parameter("publish_eye0", True)
        self.declare_parameter("publish_eye1", True)
        self.declare_parameter("frame_id_world", "pupil_world")
        self.declare_parameter("frame_id_eye0", "pupil_eye0")
        self.declare_parameter("frame_id_eye1", "pupil_eye1")

        self._host = self.get_parameter("pupil_host").value
        self._req_port = self.get_parameter("pupil_req_port").value
        self._encoding = self.get_parameter("frame_encoding").value

        self._pub_world = self.get_parameter("publish_world").value
        self._pub_eye0 = self.get_parameter("publish_eye0").value
        self._pub_eye1 = self.get_parameter("publish_eye1").value

        self._fid_world = self.get_parameter("frame_id_world").value
        self._fid_eye0 = self.get_parameter("frame_id_eye0").value
        self._fid_eye1 = self.get_parameter("frame_id_eye1").value

        # ---- publishers -----------------------------------------------
        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value

        self._world_img_pub = self.create_publisher(Image, "/pupil/world/image_raw", qos)
        self._world_info_pub = self.create_publisher(CameraInfo, "/pupil/world/camera_info", qos)
        self._eye0_img_pub = self.create_publisher(Image, "/pupil/eye0/image_raw", qos)
        self._eye0_info_pub = self.create_publisher(CameraInfo, "/pupil/eye0/camera_info", qos)
        self._eye1_img_pub = self.create_publisher(Image, "/pupil/eye1/image_raw", qos)
        self._eye1_info_pub = self.create_publisher(CameraInfo, "/pupil/eye1/camera_info", qos)

        # ---- driver ---------------------------------------------------
        self._bridge = CvBridge()
        self._driver = PupilDriver(host=self._host, req_port=self._req_port)

        # ---- receive thread -------------------------------------------
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        topics = []
        if self._pub_world:
            topics.append(TOPIC_WORLD)
        if self._pub_eye0:
            topics.append(TOPIC_EYE0)
        if self._pub_eye1:
            topics.append(TOPIC_EYE1)

        self._driver.connect()
        self._driver.subscribe(topics)

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "CameraNode started — connected to %s:%s", self._host, self._req_port
        )

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._driver.disconnect()
        self.get_logger().info("CameraNode stopped")

    # ------------------------------------------------------------------
    # Receive loop (runs in background thread)
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        while self._running:
            result = self._driver.recv()
            if result is None:
                continue
            topic, payload = result
            try:
                if topic == TOPIC_WORLD and self._pub_world:
                    self._publish_frame(payload, self._world_img_pub, self._world_info_pub, self._fid_world)
                elif topic == TOPIC_EYE0 and self._pub_eye0:
                    self._publish_frame(payload, self._eye0_img_pub, self._eye0_info_pub, self._fid_eye0)
                elif topic == TOPIC_EYE1 and self._pub_eye1:
                    self._publish_frame(payload, self._eye1_img_pub, self._eye1_info_pub, self._fid_eye1)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to publish frame [{topic}]: {exc}")

    # ------------------------------------------------------------------
    # Frame publishing helper
    # ------------------------------------------------------------------

    def _publish_frame(
        self,
        payload: Dict,
        img_pub,
        info_pub,
        frame_id: str,
    ) -> None:
        """Decode a Pupil frame payload and publish an Image + CameraInfo pair."""
        now = self.get_clock().now().to_msg()

        img = self._decode_frame(payload, frame_id)
        if img is None:
            return

        ros_img = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_img.header.stamp = now
        ros_img.header.frame_id = frame_id
        img_pub.publish(ros_img)

        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = frame_id
        info.width = img.shape[1]
        info.height = img.shape[0]
        info_pub.publish(info)

    def _decode_frame(self, payload: Dict, frame_id: str = "") -> Optional[np.ndarray]:
        """
        Decode a raw Pupil frame payload into an OpenCV BGR image.

        Pupil sends frames in one of several encodings:
          - "jpeg"    : JPEG bytes in payload["__raw_data__"][0]
          - "yuv422"  : YUV422-packed bytes
          - "bgr"     : raw BGR bytes
          - "gray"    : 8-bit grayscale bytes
        """
        fmt = payload.get("format", "jpeg")
        raw_data_list = payload.get("__raw_data__", [])
        if not raw_data_list:
            return None
        raw: bytes = raw_data_list[0]

        width = payload.get("width", 0)
        height = payload.get("height", 0)

        if fmt == "jpeg":
            buf = np.frombuffer(raw, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            return img

        if fmt == "yuv422":
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 2))
            img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
            return img

        if fmt == "bgr":
            img = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
            return img.copy()

        if fmt == "gray":
            gray = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
            img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            return img

        self.get_logger().warning(f"Unknown frame format '{fmt}' on [{frame_id}]")
        return None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
