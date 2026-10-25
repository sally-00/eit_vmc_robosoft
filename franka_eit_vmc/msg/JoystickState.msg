# Host receipt timestamp from the ESP32 recorder, not time of CSV replay/publication.
std_msgs/Header header
uint64 sequence
# One-axis shaft rotation about robot Base +Y; positive tip displacement is Base +X.
float64 angle_rad
float64 angular_velocity_rad_s
bool measurement_valid
bool clipped
