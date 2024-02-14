#pragma once
#include <iostream>
#include <rclcpp/clock.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
//ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ManualController : public rclcpp::Node
{
public:
  ManualController(const rclcpp::NodeOptions & options) : ManualController("", options) {}
  ManualController(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("manual_controller_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start manual_controller_node");
    CMD_VEL_TOPIC =
      param<std::string>("manual_controller.topic_name.cmd_vel", "manual/cmd_vel");
    AUTO_CMD_VEL_TOPIC =
      param<std::string>("manual_controller.topic_name.auto_cmd_vel", "auto/cmd_vel");
    std::string JOY_TOPIC = param<std::string>("manual_controller.topic_name.joy", "/joy");
    MAX_VEL = param<double>("manual_controller.max.vel", 0.5);
    MAX_ANGULAR = param<double>("manual_controller.max.angular", 0.2);
    // publisher
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    switch_pub_ =
      this->create_publisher<std_msgs::msg::String>("twist_bridge/switch", rclcpp::QoS(10));
    // subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = msg->axes[axes_list_["LY"]] * MAX_VEL;
        cmd_vel.linear.y = msg->axes[axes_list_["LX"]] * MAX_VEL;
        cmd_vel.angular.z = msg->axes[axes_list_["RX"]] * MAX_ANGULAR;
        if(msg->buttons[button_list_["A"]]){
          publish_switch(AUTO_CMD_VEL_TOPIC);
        }
        if(msg->buttons[button_list_["B"]]){
          publish_switch(CMD_VEL_TOPIC);
        }
        if(msg->buttons[button_list_["X"]]){
          ems(true);
        }
        if(msg->buttons[button_list_["Y"]]){
          ems(false);
        }
        if(msg->buttons[button_list_["L1"]]){
          cmd_vel.angular.z = MAX_ANGULAR;
        }
        else if(msg->buttons[button_list_["R1"]]){
          cmd_vel.angular.z = -MAX_ANGULAR;
        }
        if(!msg->axes[axes_list_["LY"]])
          cmd_vel.linear.x = msg->axes[axes_list_["UD"]] * MAX_VEL;
        if(!msg->axes[axes_list_["LX"]])
          cmd_vel.linear.y = msg->axes[axes_list_["LR"]] * MAX_VEL;
        cmd_vel_pub_->publish(cmd_vel);
      });
    // client
    ems_srv_ = this->create_client<std_srvs::srv::SetBool>("twist_bridge/ems");
    while(!ems_srv_->wait_for_service(1s)){
      if(!rclcpp::ok()){
        RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service...");
    }
  }

private:
  double MAX_VEL;
  double MAX_ANGULAR;
  std::string AUTO_CMD_VEL_TOPIC;
  std::string CMD_VEL_TOPIC;
  std::unordered_map<std::string, int> axes_list_ = {
    {"LX", 0},
    {"LY", 1},
    {"RX", 2},
    {"RY", 3},
    {"LR", 4},//Left Right
    {"UD", 5},//Up Down
  };
  std::unordered_map<std::string, int> button_list_ = {
    {"X", 0},  {"A", 1},  {"B", 2},      {"Y", 3},     {"L1", 4},  {"R1", 5},
    {"L2", 6}, {"R2", 7}, {"SELECT", 8}, {"START", 9}, {"L3", 10}, {"L4", 11},
  };
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_pub_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  // client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ems_srv_;
  // twists
  std::vector<geometry_msgs::msg::Twist> twists_;

  void publish_switch(const std::string & name)
  {
    std_msgs::msg::String msg;
    msg.data = name;
    switch_pub_->publish(msg);
  }

  void ems(const bool & data)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;
    auto result = ems_srv_->async_send_request(request,[&](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      RCLCPP_INFO(this->get_logger(), "%s",future.get()->message.c_str());
    });
  }

  template <class T>
  T param(const std::string & name, const T & def)
  {
    T value;
    declare_parameter(name, def);
    get_parameter(name, value);
    return value;
  }

  geometry_msgs::msg::Twist stop()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
  }
};