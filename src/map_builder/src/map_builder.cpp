#include <rclcpp/rclcpp.hpp>
#include "node.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<MapBuilderNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}