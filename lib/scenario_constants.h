#pragma once
#include <stdint.h>
#include <string>

enum class TrafficLightState : uint8_t {
  FLASHING = 0,
  GREEN = 1,
  YELLOW = 2,
  RED = 3
};

constexpr const std::string to_string(TrafficLightState state) {
  switch (state) {
    case TrafficLightState::FLASHING:
      return "FLASHING";
    case TrafficLightState::GREEN:
      return "GREEN";
    case TrafficLightState::YELLOW:
      return "YELLOW";
    case TrafficLightState::RED:
      return "RED";
  }

  return "Unknown";
}

constexpr const bool can_pass(TrafficLightState state) {
  return state == TrafficLightState::GREEN;
}
