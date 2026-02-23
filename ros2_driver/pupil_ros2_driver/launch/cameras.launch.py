"""
cameras.launch.py
-----------------
Launch only the Pupil Core camera streaming node.

Publishes:
  /pupil/world/image_raw     (sensor_msgs/Image)
  /pupil/world/camera_info   (sensor_msgs/CameraInfo)
  /pupil/eye0/image_raw      (sensor_msgs/Image)
  /pupil/eye0/camera_info    (sensor_msgs/CameraInfo)
  /pupil/eye1/image_raw      (sensor_msgs/Image)
  /pupil/eye1/camera_info    (sensor_msgs/CameraInfo)

Usage:
  ros2 launch pupil_ros2_driver cameras.launch.py
  ros2 launch pupil_ros2_driver cameras.launch.py pupil_host:=192.168.1.10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # ---- arguments --------------------------------------------
            DeclareLaunchArgument(
                "pupil_host",
                default_value="localhost",
                description="Hostname or IP of the machine running Pupil Capture / Service",
            ),
            DeclareLaunchArgument(
                "pupil_req_port",
                default_value="50020",
                description="Pupil Capture REQ/REP port (Network API → IPC Backend port)",
            ),
            DeclareLaunchArgument(
                "publish_world",
                default_value="true",
                description="Publish world camera stream",
            ),
            DeclareLaunchArgument(
                "publish_eye0",
                default_value="true",
                description="Publish eye-0 (left) camera stream",
            ),
            DeclareLaunchArgument(
                "publish_eye1",
                default_value="true",
                description="Publish eye-1 (right) camera stream",
            ),
            # ---- nodes ------------------------------------------------
            Node(
                package="pupil_ros2_driver",
                executable="camera_node",
                name="pupil_camera_node",
                output="screen",
                parameters=[
                    {
                        "pupil_host": LaunchConfiguration("pupil_host"),
                        "pupil_req_port": LaunchConfiguration("pupil_req_port"),
                        "publish_world": LaunchConfiguration("publish_world"),
                        "publish_eye0": LaunchConfiguration("publish_eye0"),
                        "publish_eye1": LaunchConfiguration("publish_eye1"),
                    }
                ],
            ),
        ]
    )
