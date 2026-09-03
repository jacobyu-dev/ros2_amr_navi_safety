# Observable mission-manager snapshot.  The C++ state machine owns the
# transition policy; these constants make its state available to ROS clients.
std_msgs/Header header

uint8 IDLE=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 COMPLETED=3
uint8 FAILED=4
uint8 CANCELED=5

uint8 NAVIGATION_IDLE=0
uint8 WAITING_FOR_GOAL=1
uint8 NAVIGATING=2
uint8 NAVIGATION_PAUSED=3
uint8 ARRIVED=4
uint8 NAVIGATION_FAILED=5
uint8 NAVIGATION_CANCELED=6

uint64 mission_id
uint8 mission_state
uint8 navigation_state
uint32 current_goal_index
uint32 total_goals
# Latest NavigateToPose feedback. A negative distance means feedback has not
# been received for the current navigation goal.
float32 distance_remaining
float32 navigation_time_sec
int16 number_of_recoveries
string reason
