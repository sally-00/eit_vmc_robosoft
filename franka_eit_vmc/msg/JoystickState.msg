# Host receipt timestamp from the ESP32 recorder, not time of CSV replay/publication.
std_msgs/Header header
uint64 sequence
# Base X tip angle (rotation about Base +Y) and Base Y tip angle.
float64 angle_rad
float64 angular_velocity_rad_s
float64 angle_y_rad
float64 angular_velocity_y_rad_s
bool measurement_valid
bool clipped
