#include <memory>

#include "navigator/NavigatorNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<navigator::NavigatorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
