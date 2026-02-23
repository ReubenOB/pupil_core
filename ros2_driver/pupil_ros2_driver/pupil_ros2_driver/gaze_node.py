"""
gaze_node.py
------------
ROS 2 node that connects to Pupil Capture / Service and publishes raw pupil
detections and mapped gaze data.

Published topics
----------------
  /pupil/gaze          (pupil_ros2_msgs/msg/GazeStamped)
  /pupil/pupil/eye0    (pupil_ros2_msgs/msg/PupilDatum)
  /pupil/pupil/eye1    (pupil_ros2_msgs/msg/PupilDatum)
  /pupil/gaze/point    (geometry_msgs/msg/PointStamped)  — 3-D gaze point when
                        available, otherwise 2-D in the z=0 plane

Parameters
----------
  pupil_host      (string, default "localhost")
  pupil_req_port  (int,    default 50020)
  publish_pupil   (bool,   default true)  — publish raw pupil data
  publish_gaze    (bool,   default true)  — publish gaze data
  frame_id_world  (string, default "pupil_world")
  min_confidence  (float,  default 0.0)   — drop samples below this threshold
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from pupil_ros2_msgs.msg import GazeStamped, PupilDatum
from pupil_ros2_driver.pupil_driver import PupilDriver

TOPIC_GAZE = "gaze"
TOPIC_PUPIL0 = "pupil.0"
TOPIC_PUPIL1 = "pupil.1"


class GazeNode(Node):
    """Publishes Pupil Core gaze and raw pupil data to ROS 2."""

    def __init__(self) -> None:
        super().__init__("pupil_gaze_node")

        # ---- parameters -----------------------------------------------
        self.declare_parameter("pupil_host", "localhost")
        self.declare_parameter("pupil_req_port", 50020)
        self.declare_parameter("publish_pupil", True)
        self.declare_parameter("publish_gaze", True)
        self.declare_parameter("frame_id_world", "pupil_world")
        self.declare_parameter("min_confidence", 0.0)

        self._host = self.get_parameter("pupil_host").value
        self._req_port = self.get_parameter("pupil_req_port").value
        self._pub_pupil = self.get_parameter("publish_pupil").value
        self._pub_gaze = self.get_parameter("publish_gaze").value
        self._frame_id = self.get_parameter("frame_id_world").value
        self._min_conf = self.get_parameter("min_confidence").value

        # ---- publishers -----------------------------------------------
        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value

        self._gaze_pub = self.create_publisher(GazeStamped, "/pupil/gaze", qos)
        self._gaze_point_pub = self.create_publisher(PointStamped, "/pupil/gaze/point", qos)
        self._pupil0_pub = self.create_publisher(PupilDatum, "/pupil/pupil/eye0", qos)
        self._pupil1_pub = self.create_publisher(PupilDatum, "/pupil/pupil/eye1", qos)

        # ---- driver ---------------------------------------------------
        self._driver = PupilDriver(host=self._host, req_port=self._req_port)

        # ---- receive thread -------------------------------------------
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        topics: List[str] = []
        if self._pub_gaze:
            topics.append(TOPIC_GAZE)
        if self._pub_pupil:
            topics.extend([TOPIC_PUPIL0, TOPIC_PUPIL1])

        self._driver.connect()
        self._driver.subscribe(topics)

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "GazeNode started — connected to %s:%s", self._host, self._req_port
        )

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._driver.disconnect()
        self.get_logger().info("GazeNode stopped")

    # ------------------------------------------------------------------
    # Receive loop
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        while self._running:
            result = self._driver.recv()
            if result is None:
                continue
            topic, payload = result
            try:
                if topic == TOPIC_GAZE:
                    self._handle_gaze(payload)
                elif topic == TOPIC_PUPIL0:
                    self._handle_pupil(payload, eye_id=0)
                elif topic == TOPIC_PUPIL1:
                    self._handle_pupil(payload, eye_id=1)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to publish [{topic}]: {exc}")

    # ------------------------------------------------------------------
    # Message converters
    # ------------------------------------------------------------------

    def _handle_gaze(self, payload: Dict) -> None:
        confidence = float(payload.get("confidence", 0.0))
        if confidence < self._min_conf:
            return

        now = self.get_clock().now().to_msg()
        norm_pos = payload.get("norm_pos", [0.0, 0.0])

        # ---- GazeStamped -----------------------------------------------
        msg = GazeStamped()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.confidence = confidence
        msg.norm_pos_x = float(norm_pos[0])
        msg.norm_pos_y = float(norm_pos[1])
        msg.timestamp = float(payload.get("timestamp", 0.0))

        gaze_3d = payload.get("gaze_point_3d")
        if gaze_3d is not None:
            msg.has_3d_gaze_point = True
            msg.gaze_point_3d_x = float(gaze_3d[0])
            msg.gaze_point_3d_y = float(gaze_3d[1])
            msg.gaze_point_3d_z = float(gaze_3d[2])

        # Embed contributing pupil data
        for pd in payload.get("base_data", []):
            msg.base_data.append(self._make_pupil_datum(pd, now))

        self._gaze_pub.publish(msg)

        # ---- PointStamped (convenience topic) -------------------------
        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = self._frame_id
        if msg.has_3d_gaze_point:
            pt.point.x = msg.gaze_point_3d_x
            pt.point.y = msg.gaze_point_3d_y
            pt.point.z = msg.gaze_point_3d_z
        else:
            pt.point.x = msg.norm_pos_x
            pt.point.y = msg.norm_pos_y
            pt.point.z = 0.0
        self._gaze_point_pub.publish(pt)

    def _handle_pupil(self, payload: Dict, eye_id: int) -> None:
        confidence = float(payload.get("confidence", 0.0))
        if confidence < self._min_conf:
            return

        now = self.get_clock().now().to_msg()
        datum = self._make_pupil_datum(payload, now, eye_id=eye_id)

        if eye_id == 0:
            self._pupil0_pub.publish(datum)
        else:
            self._pupil1_pub.publish(datum)

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------

    @staticmethod
    def _make_pupil_datum(payload: Dict, stamp, eye_id: Optional[int] = None) -> PupilDatum:
        datum = PupilDatum()
        datum.header.stamp = stamp
        datum.eye_id = int(payload.get("id", eye_id or 0))
        datum.confidence = float(payload.get("confidence", 0.0))
        datum.timestamp = float(payload.get("timestamp", 0.0))
        datum.method = str(payload.get("method", ""))

        norm_pos = payload.get("norm_pos", [0.0, 0.0])
        datum.norm_pos_x = float(norm_pos[0])
        datum.norm_pos_y = float(norm_pos[1])
        datum.diameter = float(payload.get("diameter", 0.0))

        sphere = payload.get("sphere", {})
        if sphere:
            center = sphere.get("center", [0.0, 0.0, 0.0])
            datum.sphere_center_x = float(center[0])
            datum.sphere_center_y = float(center[1])
            datum.sphere_center_z = float(center[2])
            datum.sphere_radius = float(sphere.get("radius", 0.0))

        return datum


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeNode()
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
