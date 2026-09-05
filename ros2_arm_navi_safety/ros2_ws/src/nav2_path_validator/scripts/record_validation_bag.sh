#!/usr/bin/env bash
set -euo pipefail

output="${1:-nav2_path_validation_bag}"
ros2 bag record -o "${output}" \
  /plan /received_global_plan /odom /tf /tf_static /scan /cmd_vel \
  /mission/active_goal /mission/status /navigate_to_pose/_action/status \
  /global_costmap/costmap /local_costmap/costmap \
  /nav2_path_validation/actual_trajectory \
  /nav2_path_validation/state /nav2_path_validation/result
