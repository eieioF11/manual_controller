#pragma once
#include <sensor_msgs/msg/joy.hpp>
class Controller
{
protected:
  sensor_msgs::msg::Joy joy_msg_;
  sensor_msgs::msg::Joy pre_joy_msg_;

private:
public:
  enum class Axis {
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y,
    LEFT_RIGHT,
    UP_DOWN,
  };
  enum class Key {
    X,
    A,
    B,
    Y,
    L1,
    R1,
    L2,
    R2,
    SELECT,
    START,
    L_STICK,
    R_STICK,
  };

  Controller()
  {
    joy_msg_.axes.resize(2, 0);
    joy_msg_.buttons.resize(11, 0);
  }
  void update(sensor_msgs::msg::Joy joy_msg)
  {
    pre_joy_msg_ = joy_msg_;
    joy_msg_ = joy_msg;
  }
  double get_axis(Axis axis) const
  {
    return joy_msg_.axes[static_cast<std::underlying_type_t<Axis>>(axis)];
  }
  bool get_key(Key key) const
  {
    return joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)];
  }
  bool get_key_down(Key key) const
  {
    return (joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)] ^
            pre_joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)]) &
           joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)];
  }
  bool get_key_up(Key key) const
  {
    return (joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)] ^
            pre_joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)]) &
           pre_joy_msg_.buttons[static_cast<std::underlying_type_t<Key>>(key)];
  }
};