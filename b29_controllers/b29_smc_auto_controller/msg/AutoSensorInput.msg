uint8 OBSTACLE_UNKNOWN=0
uint8 OBSTACLE_LINE_CLAMP=1
uint8 OBSTACLE_DAMPER=2

std_msgs/Header header
bool lower_alive
bool imu_ready
bool grip_confirmed
bool joint_fault
bool grip_fault
bool obstacle_detected
uint8 obstacle_type
bool classification_stable
float64 range_to_obstacle
bool at_crossing_position
bool post_check_passed
bool post_check_failed
