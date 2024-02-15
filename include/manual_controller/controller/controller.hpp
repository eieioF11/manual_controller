#pragma once
#include <sensor_msgs/msg/joy.hpp>
#include <unordered_map>
class Controller
{
protected:
  double threshold_;
  sensor_msgs::msg::Joy joy_msg_;
  sensor_msgs::msg::Joy pre_joy_msg_;
  double get_axis_f(size_t i) const { return joy_msg_.axes[i]; }

  bool get_key_f(size_t i) const { return joy_msg_.buttons[i]; }

  bool get_trigger_down_f(size_t i, double threshold = 0.5) const
  {
    if (threshold < 0.0) {
      bool ax = joy_msg_.axes[i] < threshold;
      bool pre = pre_joy_msg_.axes[i] < threshold;
      return (ax ^ pre) & ax;
    }
    bool ax = joy_msg_.axes[i] > threshold;
    bool pre = pre_joy_msg_.axes[i] > threshold;
    return (ax ^ pre) & ax;
  }

  bool get_trigger_up_f(size_t i, double threshold = 0.5) const
  {
    if (threshold < 0.0) {
      bool ax = joy_msg_.axes[i] < threshold;
      bool pre = pre_joy_msg_.axes[i] < threshold;
      return (ax ^ pre) & pre;
    }
    bool ax = joy_msg_.axes[i] > threshold;
    bool pre = pre_joy_msg_.axes[i] > threshold;
    return (ax ^ pre) & pre;
  }

  bool get_key_down_f(size_t i) const
  {
    return (joy_msg_.buttons[i] ^ pre_joy_msg_.buttons[i]) & joy_msg_.buttons[i];
  }

  bool get_key_up_f(size_t i) const
  {
    return (joy_msg_.buttons[i] ^ pre_joy_msg_.buttons[i]) & pre_joy_msg_.buttons[i];
  }

private:
public:
  enum class Axis {
    LEFT_X,
    LEFT_Y,
    LEFT_TRIGGER,
    RIGHT_X,
    RIGHT_Y,
    RIGHT_TRIGGER,
    LEFT_RIGHT,
    UP_DOWN,
  };
  enum class Key {
    // button
    A,
    B,
    X,
    Y,
    L1,
    R1,
    L2,
    R2,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    SELECT,
    START,
    MODE,
    L_STICK,
    R_STICK,
    L3,
    R3,
    L4,
    R4,
    // stick
    LX_LEFT,
    LX_RIGHT,
    LY_UP,
    LY_DOWN,
    RX_LEFT,
    RX_RIGHT,
    RY_UP,
    RY_DOWN,
  };

  Controller() : threshold_(0.5)
  {
    joy_msg_.axes.resize(2, 0);
    joy_msg_.buttons.resize(11, 0);
  }
  Controller(double threshold) : threshold_(threshold)
  {
    joy_msg_.axes.resize(2, 0);
    joy_msg_.buttons.resize(11, 0);
  }

  void update(sensor_msgs::msg::Joy joy_msg)
  {
    pre_joy_msg_ = joy_msg_;
    joy_msg_ = joy_msg;
  }

  virtual double get_axis(Axis axis) const { return get_axis_f(static_cast<size_t>(axis)); }
  virtual bool get_key(Key key) const { return get_key_f(static_cast<size_t>(key)); }
  virtual bool get_key_down(Key key) const { return get_key_down_f(static_cast<size_t>(key)); }
  virtual bool get_key_up(Key key) const { return get_key_up_f(static_cast<size_t>(key)); }
};