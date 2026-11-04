#pragma once
#include <stdint.h>

enum class TrafficLightState : uint8_t {
  FLASHING = 0,
  GREEN = 1,
  YELLOW = 2,
  RED = 3
};
