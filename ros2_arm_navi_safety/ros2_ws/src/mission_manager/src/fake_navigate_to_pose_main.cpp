#include <memory>

#include "mission_manager/fake_navigate_to_pose_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::FakeNavigateToPoseServer>());
  rclcpp::shutdown();
  return 0;
}
