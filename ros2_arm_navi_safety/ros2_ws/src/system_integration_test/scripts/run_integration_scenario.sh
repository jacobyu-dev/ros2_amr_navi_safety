#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
workspace_dir="$(cd "${script_dir}/../../.." && pwd)"
run_stamp="$(date +%Y%m%d_%H%M%S)"
run_log="${workspace_dir}/log/integration_test_${run_stamp}.log"
mkdir -p "${workspace_dir}/log"

source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"
export ROS_DOMAIN_ID="${INTEGRATION_TEST_DOMAIN_ID:-$((100 + ($$ % 100)))}"
set -u

cleanup() {
  if [[ -n "${launch_pid:-}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Integration test log: ${run_log}"
echo "ROS domain: ${ROS_DOMAIN_ID}"
ros2 launch system_integration_test integrated_safety_scenario.launch.py \
  headless:=true record_bag:="${RECORD_BAG:-false}" >"${run_log}" 2>&1 &
launch_pid=$!

set +e
result="$(timeout "${INTEGRATION_TEST_TIMEOUT_SEC:-180}" ros2 topic echo --once \
  /integration_test/result std_msgs/msg/String --qos-durability transient_local 2>&1)"
status=$?
set -e

if [[ ${status} -ne 0 ]]; then
  echo "Integration test result was not received before timeout."
  tail -80 "${run_log}"
  exit 1
fi

echo "${result}"
if [[ "${result}" == *"data: PASS"* ]]; then
  echo "Safety System Integration Scenario: PASS"
  exit 0
fi

echo "Safety System Integration Scenario: FAIL"
tail -80 "${run_log}"
exit 1
