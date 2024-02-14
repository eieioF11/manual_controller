#include <rclcpp/rclcpp.hpp>
#include "manual_controller/manual_controller_component.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualController>());
  rclcpp::shutdown();
  return 0;
}