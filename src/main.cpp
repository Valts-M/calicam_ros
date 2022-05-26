#include <rclcpp/rclcpp.hpp>
#include "calicam.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<calicam::CaliCam>());
  rclcpp::shutdown();
  return 0;
}