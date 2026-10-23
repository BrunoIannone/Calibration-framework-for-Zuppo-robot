#include <rclcpp/rclcpp.hpp>
#include "node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPublisherNode>());
  rclcpp::shutdown();
  return 0;
}