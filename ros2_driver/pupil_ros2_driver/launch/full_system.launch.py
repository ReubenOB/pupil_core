"""
full_system.launch.py
---------------------
Launch the complete Pupil Core ROS 2 driver:

  pupil_camera_node      — streams world + eye0 + eye1 cameras
  pupil_gaze_node        — publishes gaze and raw pupil data from Pupil Capture
  pupil_eye_tracking_node — optional standalone eye tracker on the raw images
                            (use when Pupil Capture gaze is not available or
                             when you want an independent ROS 2 pipeline)

By default only camera_node and gaze_node are launched.
Set ``standalone_tracking:=true`` to also launch eye_tracking_node.
Set ``use_pupil_capture_gaze:=false`` to skip gaze_node.

Usage:
  ros2 launch pupil_ros2_driver full_system.launch.py
  ros2 launch pupil_ros2_driver full_system.launch.py pupil_host:=192.168.1.10
  ros2 launch pupil_ros2_driver full_system.launch.py standalone_tracking:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # ---- shared arguments -------------------------------------
            DeclareLaunchArgument(
                "pupil_host",
                default_value="localhost",
                description="Hostname / IP of the Pupil Capture / Service machine",
            ),
            DeclareLaunchArgument(
                "pupil_req_port",
                default_value="50020",
                description="Pupil Capture Network API REQ port",
            ),
            DeclareLaunchArgument(
                "use_pupil_capture_gaze",
                default_value="true",
                description="Launch gaze_node to receive gaze from Pupil Capture",
            ),
            DeclareLaunchArgument(
                "standalone_tracking",
                default_value="false",
                description="Launch the standalone eye_tracking_node (ROS-only pipeline)",
            ),
            DeclareLaunchArgument(
                "min_confidence",
                default_value="0.0",
                description="Drop gaze/pupil samples below this confidence threshold",
            ),

            # ---- camera node ------------------------------------------
            Node(
                package="pupil_ros2_driver",
                executable="camera_node",
                name="pupil_camera_node",
                output="screen",
                parameters=[
                    {
                        "pupil_host": LaunchConfiguration("pupil_host"),
                        "pupil_req_port": LaunchConfiguration("pupil_req_port"),
                    }
                ],
            ),

            # ---- gaze node (requires Pupil Capture running) -----------
            Node(
                package="pupil_ros2_driver",
                executable="gaze_node",
                name="pupil_gaze_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_pupil_capture_gaze")),
                parameters=[
                    {
                        "pupil_host": LaunchConfiguration("pupil_host"),
                        "pupil_req_port": LaunchConfiguration("pupil_req_port"),
                        "min_confidence": LaunchConfiguration("min_confidence"),
                    }
                ],
            ),

            # ---- standalone eye-tracking node (ROS-only pipeline) -----
            Node(
                package="pupil_ros2_driver",
                executable="eye_tracking_node",
                name="pupil_eye_tracking_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("standalone_tracking")),
            ),
        ]
    )
