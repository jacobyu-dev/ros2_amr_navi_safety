# arm_navi_safety_interfaces

## Purpose

This package defines the shared ROS 2 messages used to report AMR safety state.
It contains schemas only: no sensor processing, TF lookup, safety decision
algorithm, ROS runtime node, navigation logic, or robot-stop command belongs
here. A producer reports its own observation with `SafetyStatus`; a state
transition or notable occurrence is reported with `SafetyEvent`.

## Package Structure

```text
arm_navi_safety_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── SafetyStatus.msg
│   └── SafetyEvent.msg
├── test/
│   └── test_safety_messages.cpp
└── README.md
```

## SafetyStatus

`SafetyStatus` is the current snapshot produced by one component. `header`
provides timestamp and frame context; `source` identifies the producer (for
example, `lidar_safety`, `tf_monitor`, or `safety_supervisor`). `reason` is
human-readable logging/debugging context only. Safety logic must use `level`
and `data_valid`, never parse `reason`.

```text
std_msgs/Header header
uint8 UNKNOWN=0
uint8 SAFE=1
uint8 WARNING=2
uint8 STOP=3
uint8 ERROR=4
uint8 level
string source
string reason
bool data_valid
```

## SafetyEvent

`SafetyEvent` records a safety-level transition or a specific significant
event. Its constants intentionally duplicate `SafetyStatus` because ROS 2
message constants cannot be imported from another `.msg` file. Both messages
use the same fixed mapping.

```text
std_msgs/Header header
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
```

`data_valid` is included on events so consumers can preserve the validity
context that caused the transition (for example, valid obstacle data versus a
lost TF lookup).

## Safety Level Definition

| Level | Value | Meaning |
| --- | ---: | --- |
| `UNKNOWN` | 0 | There is not yet enough information to make a safety judgment. Typical cases are node startup, before the first sensor message, or before initial TF verification. |
| `SAFE` | 1 | The currently available, valid inputs support normal operation. |
| `WARNING` | 2 | A possible hazard exists but does not by itself require immediate motion stop, such as an obstacle in a warning zone or increased TF latency. |
| `STOP` | 3 | The safety judgment requires robot motion to stop, such as an obstacle in the stop zone or a required transform being unavailable past its policy timeout. This package does not issue a stop command. |
| `ERROR` | 4 | The safety subsystem itself cannot operate correctly, such as invalid configuration, an internal processing error, or a required dependency failure. |

`UNKNOWN`, `ERROR`, and `STOP` are intentionally distinct:

- `UNKNOWN` means insufficient information, not a confirmed hazard or a broken subsystem.
- `ERROR` means the safety subsystem is malfunctioning.
- `STOP` means the component reached a safety judgment that motion must stop.

### `data_valid` Semantics

`data_valid=true` means the inputs used for that reported judgment were valid.
`data_valid=false` means the component cannot rely on its input, regardless of
the reported level. For example, startup can be `UNKNOWN` with invalid data, a
LiDAR timeout can be `UNKNOWN` with invalid data, and a failed TF lookup can be
`ERROR` with invalid data. A future Safety Supervisor may conservatively
escalate any invalid input to final `STOP` under a fail-safe policy; that policy
is deliberately not implemented in this Phase.

## Example Messages

```yaml
# /safety/lidar/status
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: laser_frame
level: 3
source: lidar_safety
reason: obstacle inside stop distance
data_valid: true
```

```yaml
# /safety/events
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: base_link
source: lidar_safety
previous_level: 1
current_level: 2
reason: obstacle entered warning zone
data_valid: true
```

## Topic Convention

These are recommended integration defaults, not hard-coded topics in this
interface package. Producers and consumers should permit parameterization or
ROS remapping.

| Topic | Producer / use |
| --- | --- |
| `/safety/lidar/status` | Individual LiDAR Safety Node `SafetyStatus` |
| `/safety/tf/status` | Individual TF Monitor `SafetyStatus` |
| `/safety/events` | State-transition `SafetyEvent` stream |
| `/safety/status` | Safety Supervisor final `SafetyStatus` |

## Build

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arm_navi_safety_interfaces
source install/setup.bash
ros2 interface show arm_navi_safety_interfaces/msg/SafetyStatus
ros2 interface show arm_navi_safety_interfaces/msg/SafetyEvent
```

The two `ros2 interface show` commands must display the schemas above,
including the shared level mapping (`UNKNOWN=0` through `ERROR=4`).

## Test

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select arm_navi_safety_interfaces
colcon test-result --verbose
```

`test_safety_messages.cpp` verifies default construction, constant mapping,
field assignment, invalid-data representations, and in-process ROS 2
serialization/deserialization for both message types. The transport checks
`/test/safety_status` and `/test/safety_event`, including the `SAFE -> WARNING
-> STOP` event sequence.

## Replay Test

**Not Required for Phase 4.** This phase validates message schema and ROS 2
serialization rather than raw sensor processing, so a rosbag does not add
coverage beyond the automated transport tests. After integration, replay can
exercise the end-to-end pipeline:

```text
rosbag
  |
  v
LiDAR Safety Node
  |
  | SafetyStatus
  v
Safety Supervisor
```

## Future Integration

```text
/LiDAR
   |
   v
LiDAR Safety Node
   |
   | SafetyStatus
   v
                    +-------------------+
TF Monitor -------->| Safety Supervisor |
  SafetyStatus      +-------------------+
                             |
                             | Final Safety State
                             v
                     Navigation / Robot
```

Phase 5 integrates this common interface: `lidar_safety` publishes
`/safety/lidar/status`, `tf_monitor` publishes `/safety/tf/status`, and
`safety_supervisor` aggregates them to `/safety/status` without changing the
meaning of any level.
