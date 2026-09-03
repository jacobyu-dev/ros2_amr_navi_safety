#include <memory>

#include "mission_manager/mission_manager_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<mission_manager::MissionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executorThreads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
