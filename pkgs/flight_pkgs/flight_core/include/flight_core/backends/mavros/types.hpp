#pragma once
#include <cstdint>
#include <string>
#include "flight_core/common/types.hpp"

namespace flight_core {
struct Config {
  double prestream_sec{1.5};
  double pose_timeout_sec{0.5};
  double state_timeout_sec{3.0};
  double extended_timeout_sec{3.0};
  double future_tolerance_sec{0.1};
  double request_timeout_sec{2.0};
  double max_target_distance_m{10.0};
  double max_publish_gap_sec{0.25};
  unsigned max_request_attempts{3};
};
struct Telemetry {
  bool connected{}, state_fresh{}, pose_fresh{}, extended_fresh{}, armed{};
  std::string mode;
  uint8_t landed_state{};  // MAVROS ExtendedState: 1 = on ground.
  Pose pose;
};
enum class Action {Offboard, Arm, Land, Disarm, GroundMode};
struct Request {uint64_t id; Action action;};
}  // namespace flight_core
