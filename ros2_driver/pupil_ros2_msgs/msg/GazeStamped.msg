# Gaze datum published by Pupil Capture/Service after applying the gaze-mapping model
std_msgs/Header header

float64 confidence      # Mapping confidence [0.0, 1.0]

# 2-D gaze position in normalised world-camera coordinates
float64 norm_pos_x      # [0.0, 1.0], origin bottom-left
float64 norm_pos_y      # [0.0, 1.0], origin bottom-left

# 3-D gaze point in world-camera reference frame (mm), available for 3-D mappings
bool    has_3d_gaze_point
float64 gaze_point_3d_x
float64 gaze_point_3d_y
float64 gaze_point_3d_z

# Contributing pupil data (one or two per binocular source, one per monocular source)
pupil_ros2_msgs/PupilDatum[] base_data

float64 timestamp       # Pupil Capture hardware timestamp (seconds)
