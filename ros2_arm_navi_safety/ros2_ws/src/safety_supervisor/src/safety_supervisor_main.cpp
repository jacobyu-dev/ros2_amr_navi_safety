#include <memory>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_supervisor/safety_supervisor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<safety_supervisor::SafetySupervisorNode>();
  // The node validates executor_threads before this executor is constructed.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executor_threads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
