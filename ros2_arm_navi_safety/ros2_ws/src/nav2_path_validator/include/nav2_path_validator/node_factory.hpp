#ifndef NAV2_PATH_VALIDATOR__NODE_FACTORY_HPP_
#define NAV2_PATH_VALIDATOR__NODE_FACTORY_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace nav2_path_validator
{

std::shared_ptr<rclcpp::Node> makePathValidatorNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace nav2_path_validator

#endif  // NAV2_PATH_VALIDATOR__NODE_FACTORY_HPP_
