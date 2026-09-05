#include <memory>

#include "nav2_path_validator/node_factory.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(nav2_path_validator::makePathValidatorNode());
  rclcpp::shutdown();
  return 0;
}
