# A transition or notable safety event produced by one component.
std_msgs/Header header

# Shared safety-level mapping. Keep identical in SafetyStatus.msg.
uint8 UNKNOWN=0
uint8 SAFE=1
uint8 WARNING=2
uint8 STOP=3
uint8 ERROR=4

string source
uint8 previous_level
uint8 current_level
string reason
bool data_valid

# Optional Phase 9 correlation information. A value of zero means that this
# event is not part of a latency experiment. The steady-clock values are
# monotonic nanoseconds from the host's CLOCK_MONOTONIC domain.
uint64 event_id
string fault_type
int64 fault_time_steady_ns
