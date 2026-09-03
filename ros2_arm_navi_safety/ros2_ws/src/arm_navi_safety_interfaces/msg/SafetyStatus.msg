# Current snapshot produced by one safety-related component.
std_msgs/Header header

# Shared safety-level mapping. Keep identical in SafetyEvent.msg.
uint8 UNKNOWN=0
uint8 SAFE=1
uint8 WARNING=2
uint8 STOP=3
uint8 ERROR=4

uint8 level
string source
string reason
bool data_valid

# Optional Phase 9 latency correlation. These fields are populated only for a
# fault-injection event; zero means no correlated latency event is available.
uint64 latency_event_id
string latency_fault_type
int64 fault_time_steady_ns
int64 detection_time_steady_ns

# Input producers leave this at zero. The Safety Supervisor fills it when it
# publishes an aggregate decision so consumers can measure reaction latency.
int64 decision_time_steady_ns
