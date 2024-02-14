#pragma once
#include <sensor_msgs/msg/joy.hpp>
class Controller
{
protected:
  sensor_msgs::msg::Joy joy_msg_;
  sensor_msgs::msg::Joy pre_joy_msg_;
  virtual double get_axis_f(size_t i) const
  {
    return joy_msg_.axes[i];
  }

  virtual bool get_key_f(size_t i) const
  {
    return joy_msg_.buttons[i];
  }

  virtual bool get_key_down_f(size_t i) const
  {
    return (joy_msg_.buttons[i] ^
            pre_joy_msg_.buttons[i]) &
           joy_msg_.buttons[i];
  }

  virtual bool get_key_up_f(size_t i) const
  {
    return (joy_msg_.buttons[i] ^
            pre_joy_msg_.buttons[i]) &
           pre_joy_msg_.buttons[i];
  }

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

  virtual double get_axis(Axis axis) const
  {
    return  get_axis_f(static_cast<size_t>(axis));
  }
  virtual bool get_key(Key key) const
  {
    return get_key_f(static_cast<size_t>(key));
  }
  virtual bool get_key_down(Key key) const
  {
    return get_key_down_f(static_cast<size_t>(key));
  }
  virtual bool get_key_up(Key key) const
  {
    return get_key_up_f(static_cast<size_t>(key));
  }
};