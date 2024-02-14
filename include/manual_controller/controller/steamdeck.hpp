#pragma once
#include "controller.hpp"
class SteamDeckController : public Controller
{
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
    A,
    B,
    X,
    Y,
    L1,
    R1,
    SELECT,
    START,
    L_STICK,
    R_STICK,
  };

  SteamDeckController() : Controller()
  {
  }
};