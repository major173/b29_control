# 正式 PlannerControl 四关节命令。
# positions 顺序固定为：left_first, left_second, right_first, right_second。
std_msgs/Header header
uint32 session_id
uint32 sequence
float64[4] positions
