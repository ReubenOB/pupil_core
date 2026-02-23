# Pupil Core ROS 2 Driver

ROS 2 driver for the [Pupil Core](https://docs.pupil-labs.com/core/) open-source
eye-tracking platform.  Connects to **Pupil Capture** or **Pupil Service** over
the ZeroMQ Network API and publishes the three camera streams (world camera + two
eye cameras) and gaze data as standard ROS 2 topics.

---

## Repository layout

```
ros2_driver/
├── pupil_ros2_msgs/          # Custom ROS 2 message definitions
│   ├── msg/
│   │   ├── PupilDatum.msg    # Raw eye-camera detection result
│   │   └── GazeStamped.msg   # Mapped gaze point
│   ├── CMakeLists.txt
│   └── package.xml
│
└── pupil_ros2_driver/        # Python driver package
    ├── pupil_ros2_driver/
    │   ├── pupil_driver.py       # Low-level ZMQ Network API wrapper
    │   ├── camera_node.py        # Streams world + eye cameras
    │   ├── gaze_node.py          # Publishes gaze / pupil data
    │   └── eye_tracking_node.py  # Standalone ROS-only eye tracker
    ├── launch/
    │   ├── cameras.launch.py     # Launch camera streams only
    │   └── full_system.launch.py # Launch full pipeline
    ├── test/
    │   ├── test_pupil_driver.py  # Tests for the ZMQ wrapper
    │   └── test_eye_tracking.py  # Tests for the detector logic
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS 2 (Humble / Iron / Jazzy) | Any tier-1 distribution |
| Pupil Capture ≥ 3.5 **or** Pupil Service | Running on the same machine or reachable via LAN |
| Python ≥ 3.10 | Provided by the ROS 2 installation |
| `pyzmq` | `pip install pyzmq` |
| `msgpack` | `pip install msgpack` |
| `opencv-python` | `pip install opencv-python` or ROS `cv_bridge` |

---

## Building

Place (or symlink) both packages inside your colcon workspace `src/` directory:

```bash
# From your colcon workspace root
ln -s /path/to/pupil_core/ros2_driver/pupil_ros2_msgs   src/
ln -s /path/to/pupil_core/ros2_driver/pupil_ros2_driver src/

colcon build --packages-select pupil_ros2_msgs pupil_ros2_driver
source install/setup.bash
```

---

## Nodes

### `camera_node`

Connects to Pupil Capture / Service and publishes the three raw camera streams.

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/pupil/world/image_raw` | `sensor_msgs/Image` | World (scene) camera |
| `/pupil/world/camera_info` | `sensor_msgs/CameraInfo` | World camera info |
| `/pupil/eye0/image_raw` | `sensor_msgs/Image` | Left eye camera |
| `/pupil/eye0/camera_info` | `sensor_msgs/CameraInfo` | Left eye camera info |
| `/pupil/eye1/image_raw` | `sensor_msgs/Image` | Right eye camera |
| `/pupil/eye1/camera_info` | `sensor_msgs/CameraInfo` | Right eye camera info |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `pupil_host` | `"localhost"` | Hostname / IP of the Pupil machine |
| `pupil_req_port` | `50020` | Network API REQ port |
| `publish_world` | `true` | Enable world camera |
| `publish_eye0` | `true` | Enable eye-0 camera |
| `publish_eye1` | `true` | Enable eye-1 camera |
| `frame_id_world` | `"pupil_world"` | TF frame for the world camera |
| `frame_id_eye0` | `"pupil_eye0"` | TF frame for eye-0 |
| `frame_id_eye1` | `"pupil_eye1"` | TF frame for eye-1 |

---

### `gaze_node`

Connects to Pupil Capture / Service and publishes the high-quality gaze and raw
pupil data produced by Pupil's built-in C++ detectors.

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/pupil/gaze` | `pupil_ros2_msgs/GazeStamped` | Mapped gaze datum |
| `/pupil/gaze/point` | `geometry_msgs/PointStamped` | Convenience 3-D gaze point |
| `/pupil/pupil/eye0` | `pupil_ros2_msgs/PupilDatum` | Raw eye-0 detection |
| `/pupil/pupil/eye1` | `pupil_ros2_msgs/PupilDatum` | Raw eye-1 detection |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `pupil_host` | `"localhost"` | Hostname / IP of the Pupil machine |
| `pupil_req_port` | `50020` | Network API REQ port |
| `publish_pupil` | `true` | Publish raw pupil data |
| `publish_gaze` | `true` | Publish gaze data |
| `min_confidence` | `0.0` | Drop samples below this confidence [0–1] |
| `frame_id_world` | `"pupil_world"` | TF frame for gaze messages |

---

### `eye_tracking_node`

Fully standalone ROS 2 eye-tracking node.  Subscribes to the raw eye-camera
`Image` topics published by `camera_node` and runs an on-board pupil detector
(Canny edge + ellipse fitting) followed by a simple gaze estimator.

Use this node when:
- You do **not** have Pupil Capture running, or
- You want a completely self-contained ROS 2 pipeline.

**Subscribed topics**

| Topic | Type |
|---|---|
| `/pupil/eye0/image_raw` | `sensor_msgs/Image` |
| `/pupil/eye1/image_raw` | `sensor_msgs/Image` |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/pupil/eye_tracking/pupil/eye0` | `pupil_ros2_msgs/PupilDatum` | Detected pupil (eye 0) |
| `/pupil/eye_tracking/pupil/eye1` | `pupil_ros2_msgs/PupilDatum` | Detected pupil (eye 1) |
| `/pupil/eye_tracking/gaze` | `pupil_ros2_msgs/GazeStamped` | Estimated gaze |
| `/pupil/eye_tracking/gaze/point` | `geometry_msgs/PointStamped` | Gaze point |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `frame_id_world` | `"pupil_world"` | TF frame for output messages |
| `min_blob_area` | `100` | Minimum pupil contour area (pixels²) |
| `max_blob_area` | `10000` | Maximum pupil contour area (pixels²) |
| `canny_threshold1` | `20` | Lower Canny edge threshold |
| `canny_threshold2` | `60` | Upper Canny edge threshold |
| `use_binocular_gaze` | `true` | Average both eyes; `false` = eye-0 only |

---

## Launch files

### Camera streams only

```bash
ros2 launch pupil_ros2_driver cameras.launch.py
# Remote machine:
ros2 launch pupil_ros2_driver cameras.launch.py pupil_host:=192.168.1.10
```

### Full pipeline (cameras + Pupil Capture gaze)

```bash
ros2 launch pupil_ros2_driver full_system.launch.py
```

### Full pipeline with standalone ROS eye tracker

```bash
ros2 launch pupil_ros2_driver full_system.launch.py \
  use_pupil_capture_gaze:=false \
  standalone_tracking:=true
```

---

## Integration into a colcon workspace (vigil)

Add both packages to your workspace `src/` directory and include them as
dependencies in the packages that consume them:

```xml
<!-- your_package/package.xml -->
<depend>pupil_ros2_msgs</depend>
<depend>pupil_ros2_driver</depend>
```

Subscribe to camera streams from any other ROS 2 node:

```python
from sensor_msgs.msg import Image
from pupil_ros2_msgs.msg import GazeStamped

self.create_subscription(Image,        '/pupil/world/image_raw', self.world_cb, 10)
self.create_subscription(Image,        '/pupil/eye0/image_raw',  self.eye0_cb,  10)
self.create_subscription(Image,        '/pupil/eye1/image_raw',  self.eye1_cb,  10)
self.create_subscription(GazeStamped,  '/pupil/gaze',            self.gaze_cb,  10)
```

---

## Running the tests

```bash
# From workspace root (after sourcing install/setup.bash)
colcon test --packages-select pupil_ros2_driver
colcon test-result --verbose

# Or directly with pytest (no ROS master needed):
cd ros2_driver/pupil_ros2_driver
PYTHONPATH=. python -m pytest test/ -v
```

---

## Architecture

```
┌─────────────────────────────┐
│      Pupil Capture /        │
│       Pupil Service         │
│                             │
│  ZMQ PUB  ←── frame.world  │
│            ←── frame.eye.0  │
│            ←── frame.eye.1  │
│            ←── gaze         │
│            ←── pupil.0/1    │
└────────────┬────────────────┘
             │ ZeroMQ
             ▼
┌────────────────────────────────────────────────┐
│              ROS 2 Driver                      │
│                                                │
│  pupil_camera_node                             │
│    /pupil/world/image_raw  ──►                 │
│    /pupil/eye0/image_raw   ──►                 │
│    /pupil/eye1/image_raw   ──►                 │
│                                                │
│  pupil_gaze_node                               │
│    /pupil/gaze             ──►                 │
│    /pupil/gaze/point       ──►                 │
│    /pupil/pupil/eye0       ──►                 │
│    /pupil/pupil/eye1       ──►                 │
│                                                │
│  pupil_eye_tracking_node (optional)            │
│    subscribes: eye0 + eye1 images              │
│    /pupil/eye_tracking/gaze ──►                │
└────────────────────────────────────────────────┘
             │ ROS 2 topics
             ▼
┌────────────────────────────────────────────────┐
│         Your downstream nodes                  │
│  (object detection, SLAM, gaze mapping, …)     │
└────────────────────────────────────────────────┘
```

---

## License

MIT