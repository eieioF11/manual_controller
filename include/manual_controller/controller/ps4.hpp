#pragma once
#include "controller.hpp"
class PS4Controller : public Controller
{
public:
  PS4Controller() : Controller() {}
  PS4Controller(double threshold) : Controller(threshold) {}
  double get_axis(Axis axis) const
  {
    Axis_Map axis_map = convert_axis_map(axis);
    if (axis_map == Axis_Map::NONE || axis_map == Axis_Map::KEY) return 0;
    return get_axis_f(static_cast<size_t>(axis_map));
  }
  bool get_key(Key key) const
  {
    Key_Map key_map = convert_key_map(key);
    if (key_map == Key_Map::NONE) return false;
    if (key_map == Key_Map::AXIS) {
      switch (key) {
        case Key::UP:
          return get_axis(Axis::UP_DOWN) > threshold_;
        case Key::DOWN:
          return get_axis(Axis::UP_DOWN) < -threshold_;
        case Key::LEFT:
          return get_axis(Axis::LEFT_RIGHT) < -threshold_;
        case Key::RIGHT:
          return get_axis(Axis::LEFT_RIGHT) > threshold_;
        //stick
        case Key::LX_LEFT:
          return get_axis(Axis::LEFT_X) > threshold_;
        case Key::LY_UP:
          return get_axis(Axis::LEFT_Y) > threshold_;
        case Key::LX_RIGHT:
          return get_axis(Axis::LEFT_X) < -threshold_;
        case Key::LY_DOWN:
          return get_axis(Axis::LEFT_Y) < -threshold_;
        case Key::RX_LEFT:
          return get_axis(Axis::RIGHT_X) > threshold_;
        case Key::RY_UP:
          return get_axis(Axis::RIGHT_Y) > threshold_;
        case Key::RX_RIGHT:
          return get_axis(Axis::RIGHT_X) < -threshold_;
        case Key::RY_DOWN:
          return get_axis(Axis::RIGHT_Y) < -threshold_;
        default:
          return false;
      }
    }
    return get_key_f(static_cast<size_t>(key_map));
  }
  bool get_key_down(Key key) const {
    Key_Map key_map = convert_key_map(key);
    if (key_map == Key_Map::NONE) return false;
    if (key_map == Key_Map::AXIS) {
      switch (key) {
        case Key::UP:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::UP_DOWN),threshold_);
        case Key::DOWN:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::UP_DOWN),-threshold_);
        case Key::LEFT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_RIGHT),-threshold_);
        case Key::RIGHT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_RIGHT),threshold_);
        //stick
        case Key::LX_LEFT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_X),threshold_);
        case Key::LX_RIGHT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_X),-threshold_);
        case Key::LY_UP:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_Y),threshold_);
        case Key::LY_DOWN:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::LEFT_Y),-threshold_);
        case Key::RX_LEFT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::RIGHT_X),threshold_);
        case Key::RX_RIGHT:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::RIGHT_X),-threshold_);
        case Key::RY_UP:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::RIGHT_Y),threshold_);
        case Key::RY_DOWN:
          return get_trigger_down_f(static_cast<size_t>(Axis_Map::RIGHT_Y),-threshold_);
        default:
          return false;
      }
    }
    return get_key_down_f(static_cast<size_t>(key_map));
  }
  bool get_key_up(Key key) const {
    Key_Map key_map = convert_key_map(key);
    if (key_map == Key_Map::NONE) return false;
    if (key_map == Key_Map::AXIS) {
      switch (key) {
        case Key::UP:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::UP_DOWN),threshold_);
        case Key::DOWN:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::UP_DOWN),-threshold_);
        case Key::LEFT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_RIGHT),-threshold_);
        case Key::RIGHT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_RIGHT),threshold_);
        //stick
        case Key::LX_LEFT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_X),threshold_);
        case Key::LX_RIGHT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_X),-threshold_);
        case Key::LY_UP:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_Y),threshold_);
        case Key::LY_DOWN:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::LEFT_Y),-threshold_);
        case Key::RX_LEFT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::RIGHT_X),threshold_);
        case Key::RX_RIGHT:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::RIGHT_X),-threshold_);
        case Key::RY_UP:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::RIGHT_Y),threshold_);
        case Key::RY_DOWN:
          return get_trigger_up_f(static_cast<size_t>(Axis_Map::RIGHT_Y),-threshold_);
        default:
          return false;
      }
    }
    return get_key_up_f(static_cast<size_t>(key_map));
  }

private:
  enum class Axis_Map {
    LEFT_X,
    LEFT_Y, 
    RIGHT_X,
    LEFT_TRIGGER,
    RIGHT_TRIGGER,
    RIGHT_Y,
    LEFT_RIGHT,
    UP_DOWN,
    NONE,
    KEY,
  };
  enum class Key_Map {
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
    MODE,
    NONE,
    AXIS,
  };
  std::unordered_map<Axis, Axis_Map> axis_map_ = {
    {Axis::LEFT_X, Axis_Map::LEFT_X},
    {Axis::LEFT_Y, Axis_Map::LEFT_Y},
    {Axis::RIGHT_X, Axis_Map::RIGHT_X},
    {Axis::RIGHT_Y, Axis_Map::RIGHT_Y},
    {Axis::LEFT_RIGHT, Axis_Map::LEFT_RIGHT},
    {Axis::UP_DOWN, Axis_Map::UP_DOWN},
    {Axis::LEFT_TRIGGER, Axis_Map::LEFT_TRIGGER},
    {Axis::RIGHT_TRIGGER, Axis_Map::RIGHT_TRIGGER},
  };
  std::unordered_map<Key, Key_Map> key_map_ = {
    {Key::X, Key_Map::X},
    {Key::A, Key_Map::A},
    {Key::B, Key_Map::B},
    {Key::Y, Key_Map::Y},
    {Key::L1, Key_Map::L1},
    {Key::R1, Key_Map::R1},
    {Key::L2, Key_Map::L2},
    {Key::R2, Key_Map::R2},
    {Key::SELECT, Key_Map::SELECT},
    {Key::START, Key_Map::START},
    {Key::L_STICK, Key_Map::L_STICK},
    {Key::R_STICK, Key_Map::R_STICK},
    {Key::MODE, Key_Map::MODE},
    // axis
    {Key::UP, Key_Map::AXIS},
    {Key::DOWN, Key_Map::AXIS},
    {Key::LEFT, Key_Map::AXIS},
    {Key::RIGHT, Key_Map::AXIS},
    {Key::LX_LEFT, Key_Map::AXIS},
    {Key::LX_RIGHT, Key_Map::AXIS},
    {Key::LY_UP, Key_Map::AXIS},
    {Key::LY_DOWN, Key_Map::AXIS},
    {Key::RX_LEFT, Key_Map::AXIS},
    {Key::RX_RIGHT, Key_Map::AXIS},
    {Key::RY_UP, Key_Map::AXIS},
    {Key::RY_DOWN, Key_Map::AXIS},
    // none
    {Key::MODE, Key_Map::NONE},
    {Key::L3, Key_Map::NONE},
    {Key::R3, Key_Map::NONE},
    {Key::L4, Key_Map::NONE},
    {Key::R4, Key_Map::NONE},
  };
  Axis_Map convert_axis_map(Axis axis) const
  {
    if (axis_map_.find(axis) != axis_map_.end()) return axis_map_.at(axis);
    return Axis_Map::NONE;
  }
  Key_Map convert_key_map(Key key) const
  {
    if (key_map_.find(key) != key_map_.end()) return key_map_.at(key);
    return Key_Map::NONE;
  }
};