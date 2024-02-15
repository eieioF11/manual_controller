#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
//ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "controller/controller.hpp"
#include "controller/logi_xbox.hpp"
#include "controller/steamdeck.hpp"
#include "controller/ps4.hpp"

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
    CMD_VEL_TOPIC = param<std::string>("manual_controller.topic_name.cmd_vel", "manual/cmd_vel");
    AUTO_CMD_VEL_TOPIC =
      param<std::string>("manual_controller.topic_name.auto_cmd_vel", "auto/cmd_vel");
    std::string JOY_TOPIC = param<std::string>("manual_controller.topic_name.joy", "/joy");
    std::string RESET_SERVICE = param<std::string>("manual_controller.service_name.reset", "/reset");
    STEP_VEL = param<double>("manual_controller.step.vel", 0.1);
    STEP_ANGULAR = param<double>("manual_controller.step.angular", 0.1);
    MAX_VEL = param<double>("manual_controller.max.vel", 5.0);
    MAX_ANGULAR = param<double>("manual_controller.max.angular", 2.0);
    MIN_VEL = param<double>("manual_controller.min.vel", 0.5);
    MIN_ANGULAR = param<double>("manual_controller.min.angular", 0.2);
    double threshold = param<double>("manual_controller.trigger_threshold", 0.1);
    std::string controller_type = param<std::string>("manual_controller.type", "logi_xbox");
    if (controller_type == "logi_xbox")
      controller_ = std::make_shared<LogiXboxController>(threshold);
    else if (controller_type == "steamdeck")
      controller_ = std::make_shared<SteamDeckController>(threshold);
    else if (controller_type == "ps4")
      controller_ = std::make_shared<PS4Controller>(threshold);
    else
      controller_ = std::make_shared<LogiXboxController>(threshold);
    RCLCPP_INFO(this->get_logger(), "%s", controller_type.c_str());
    vel_ = MIN_VEL;
    angular_ = MIN_ANGULAR;
    // publisher
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    switch_pub_ =
      this->create_publisher<std_msgs::msg::String>("twist_bridge/switch", rclcpp::QoS(10));
    // subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
        controller_->update(*msg);
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = controller_->get_axis(Controller::Axis::LEFT_Y) * MAX_VEL;
        cmd_vel.linear.y = controller_->get_axis(Controller::Axis::LEFT_X) * MAX_VEL;
        cmd_vel.angular.z = controller_->get_axis(Controller::Axis::RIGHT_X) * MAX_ANGULAR;

        if (controller_->get_key_down(Controller::Key::A)) {
          publish_switch(AUTO_CMD_VEL_TOPIC);
        }
        if (controller_->get_key_down(Controller::Key::B)) {
          publish_switch(CMD_VEL_TOPIC);
        }
        if (controller_->get_key_down(Controller::Key::X)) {
          ems(true);
        }
        if (controller_->get_key_down(Controller::Key::Y)) {
          ems(false);
        }
        if (controller_->get_key_down(Controller::Key::START)) {
          reset();
        }
        if (controller_->get_key(Controller::Key::L1)) {
          cmd_vel.angular.z = angular_;
        } else if (controller_->get_key(Controller::Key::R1)) {
          cmd_vel.angular.z = -angular_;
        }
        if (controller_->get_key(Controller::Key::L2)) {
          if (controller_->get_key_down(Controller::Key::UP))
            vel_+=STEP_VEL;
          else if (controller_->get_key_down(Controller::Key::DOWN))
            vel_-=STEP_VEL;
          else if (controller_->get_key_down(Controller::Key::LEFT))
            angular_+=STEP_ANGULAR;
          if (controller_->get_key_down(Controller::Key::RIGHT))
            angular_-=STEP_ANGULAR;
          if(vel_ > MAX_VEL) vel_ = MAX_VEL;
          if(vel_ < MIN_VEL) vel_ = MIN_VEL;
          if(angular_ > MAX_ANGULAR) angular_ = MAX_ANGULAR;
          if(angular_ < MIN_ANGULAR) angular_ = MIN_ANGULAR;
          RCLCPP_INFO(this->get_logger(), "vel: %f angular: %f", vel_, angular_);
        } else {
          if (!controller_->get_axis(Controller::Axis::LEFT_Y))
            cmd_vel.linear.x = controller_->get_axis(Controller::Axis::UP_DOWN) * vel_;
          if (!controller_->get_axis(Controller::Axis::LEFT_X))
            cmd_vel.linear.y = controller_->get_axis(Controller::Axis::LEFT_RIGHT) * vel_;
        }
        cmd_vel_pub_->publish(cmd_vel);
      });
    // client
    ems_srv_ = this->create_client<std_srvs::srv::SetBool>("twist_bridge/ems");
    reset_srv_ = this->create_client<std_srvs::srv::SetBool>(RESET_SERVICE);
    while (!ems_srv_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service...");
    }
  }

private:
  double STEP_VEL;
  double STEP_ANGULAR;
  double MAX_VEL;
  double MAX_ANGULAR;
  double MIN_VEL;
  double MIN_ANGULAR;
  double vel_;
  double angular_;
  std::string AUTO_CMD_VEL_TOPIC;
  std::string CMD_VEL_TOPIC;
  // Controller controller_;
  std::shared_ptr<Controller> controller_;
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_pub_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  // client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ems_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr reset_srv_;
  // twists
  std::vector<geometry_msgs::msg::Twist> twists_;
  bool push_button() {}

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
    auto result = ems_srv_->async_send_request(
      request, [&](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "%s", future.get()->message.c_str());
      });
  }

  void reset()
  {
    if (!reset_srv_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "waiting for service...");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto result = reset_srv_->async_send_request(
      request, [&](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "%s", future.get()->message.c_str());
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

  // std::unordered_map<std::string, int> axes_list_ = {
  //   {"LX", 0}, {"LY", 1}, {"RX", 2}, {"RY", 3}, {"LR", 4},  //Left Right
  //   {"UD", 5},                                              //Up Down
  // };
  // std::unordered_map<std::string, int> button_list_ = {
  //   {"X", 0},  {"A", 1},  {"B", 2},      {"Y", 3},     {"L1", 4},  {"R1", 5},
  //   {"L2", 6}, {"R2", 7}, {"SELECT", 8}, {"START", 9}, {"L3", 10}, {"L4", 11},
  // };
};