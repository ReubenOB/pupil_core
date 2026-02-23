# Pupil datum from one eye camera (raw detection result from Pupil Capture/Service)
std_msgs/Header header

uint8 eye_id           # 0 = left eye, 1 = right eye
float64 confidence     # Detection confidence [0.0, 1.0]

# 2-D pupil ellipse in normalised image coordinates
float64 norm_pos_x     # [0.0, 1.0], origin bottom-left
float64 norm_pos_y     # [0.0, 1.0], origin bottom-left
float64 diameter       # Diameter in pixels (in eye-camera image)

# 3-D sphere model (available when 3-D detector is active)
float64 sphere_center_x
float64 sphere_center_y
float64 sphere_center_z
float64 sphere_radius

float64 timestamp      # Pupil Capture hardware timestamp (seconds)
string  method         # Detection method, e.g. "2d c++" or "3d c++"
