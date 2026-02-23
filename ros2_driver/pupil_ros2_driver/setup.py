from setuptools import find_packages, setup
import os
from glob import glob

package_name = "pupil_ros2_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
        "pyzmq",
        "msgpack",
        "numpy",
        "opencv-python",
    ],
    zip_safe=True,
    maintainer="Pupil ROS2 Driver Maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 driver for the Pupil Core eye-tracking platform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"camera_node = {package_name}.camera_node:main",
            f"gaze_node = {package_name}.gaze_node:main",
            f"eye_tracking_node = {package_name}.eye_tracking_node:main",
        ],
    },
)
