# PlannerControl 侧别。
uint8 CROSSING_SIDE_NONE=0
uint8 CROSSING_SIDE_LEFT=1
uint8 CROSSING_SIDE_RIGHT=2

# 最近一次命令拒绝原因。
uint8 REJECT_NONE=0
uint8 REJECT_NOT_ACTIVE=1
uint8 REJECT_NOT_ACCEPTING=2
uint8 REJECT_SESSION_MISMATCH=3
uint8 REJECT_INVALID_SEQUENCE=4
uint8 REJECT_SEQUENCE_GAP=5
uint8 REJECT_DUPLICATE_SEQUENCE_MISMATCH=6
uint8 REJECT_COMMAND_STALE=7
uint8 REJECT_NON_FINITE=8
uint8 REJECT_DELTA_EXCEEDED=9

# 最近一次 PlannerControl 会话退出原因。
uint8 EXIT_NONE=0
uint8 EXIT_COMPLETED=1
uint8 EXIT_SAFETY_REVOKED=2
uint8 EXIT_TOTAL_WATCHDOG_TIMEOUT=3
uint8 EXIT_CONTROLLER_STOPPED=4
uint8 EXIT_MANUAL_INTERVENTION=5
uint8 EXIT_COMMAND_STREAM_TIMEOUT=6

std_msgs/Header header
uint32 session_id
bool active
bool accepting_commands
uint8 crossing_side
bool has_accepted_command
uint32 last_accepted_sequence
uint32 last_rejected_sequence
uint8 reject_reason
uint32 rejection_count
uint32 last_completed_session_id
uint8 exit_reason
float64 max_delta_per_command
float64 command_timeout

# First-command safety reference. While this session is active and has not
# accepted sequence 1, the controller refreshes this from live encoders.
float64[4] reference_positions
time reference_stamp
