uint8 CROSSING_NONE=0
uint8 CROSSING_LINE_CLAMP=1
uint8 CROSSING_DAMPER=2

std_msgs/Header header
string current_state
string previous_state
string last_event
string transition_reason
string command_reason
string output_mode
bool stop_all
bool freeze_joints
float64 left_wheel_speed
float64 right_wheel_speed
uint8 crossing_strategy
