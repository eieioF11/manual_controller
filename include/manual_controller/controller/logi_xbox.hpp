#pragma once
#include "controller.hpp"
class LogiXboxController : public Controller
{
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

  LogiXboxController() : Controller()
  {
  }
};