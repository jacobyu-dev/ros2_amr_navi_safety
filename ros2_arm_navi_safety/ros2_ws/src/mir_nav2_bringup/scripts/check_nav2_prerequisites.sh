#!/usr/bin/env bash
set -euo pipefail

required_packages=(
  nav2_amcl
  nav2_bringup
  nav2_bt_navigator
  nav2_controller
  nav2_map_server
  nav2_planner
  nav2_velocity_smoother
  nav2_msgs
)

missing=()
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    missing+=("${package}")
  fi
done

if ((${#missing[@]})); then
  echo "Missing ROS packages: ${missing[*]}" >&2
  echo "Install with:" >&2
  echo "  sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup" >&2
  exit 1
fi

nav2_msgs_prefix="$(ros2 pkg prefix nav2_msgs)"
if [[ "${nav2_msgs_prefix}" != /opt/ros/jazzy* ]]; then
  echo "nav2_msgs resolves to ${nav2_msgs_prefix}, not the complete Jazzy system package." >&2
  echo "Remove stale build/install/log directories and rebuild after installing Nav2." >&2
  exit 1
fi

echo "Nav2 prerequisites are available from ROS 2 Jazzy."
