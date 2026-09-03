#!/usr/bin/env bash
set -euo pipefail

bag_dir="${1:-tf_monitor_synthetic_bag}"
if [[ -e "$bag_dir" ]]; then
  echo "Refusing to overwrite existing bag: $bag_dir" >&2
  exit 1
fi

package_share="$(ros2 pkg prefix tf_monitor)/share/tf_monitor"
(sleep 2; ros2 run tf_monitor publish_tf_sequence.py) &
publisher_pid=$!
set +e
timeout --signal=INT 14s ros2 bag record --output "$bag_dir" --topics /tf /clock \
  --qos-profile-overrides-path "$package_share/config/tf_bag_qos.yaml"
record_status=$?
set -e
wait "$publisher_pid"
if [[ "$record_status" -ne 0 && "$record_status" -ne 124 ]]; then
  exit "$record_status"
fi
ros2 bag info "$bag_dir"
