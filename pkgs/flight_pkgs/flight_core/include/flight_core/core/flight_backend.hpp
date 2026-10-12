#pragma once
#include <cstdint>
#include <string>
#include "flight_core/common/types.hpp"

namespace flight_core::control {
enum class Operation { Arm, Disarm, Takeoff, Land, Hold };
enum class Ground { Unknown, OnGround, InAir };
enum class Progress { Running, Accepted, Rejected };
enum class Code { Ok, Busy, Unsupported, InvalidGoal, TelemetryLost, BackendRejected,
                  Timeout, ControlLost, Preempted, InternalError };
struct BackendState {
  bool connected{}, state_fresh{}, pose_fresh{}, armed{}, control_active{};
  Ground ground{Ground::Unknown};
  Pose pose;
  bool healthy() const {return connected && state_fresh;}
  bool positioned() const {return healthy() && pose_fresh && valid_pose(pose);}
};
struct BackendRequest {uint64_t id; Operation operation; Pose target;};
struct BackendReply {uint64_t id; Progress progress; std::string detail;};
// Implementations own protocol, preparation/setpoint streaming and mode confirmation.
// begin/poll must be nonblocking. Replies are correlated by operation ID.
class FlightBackend {
public:
  virtual ~FlightBackend() = default;
  virtual bool supports(Operation operation) const = 0;
  virtual BackendState state(double now) const = 0;
  virtual void begin(const BackendRequest & request, double now) = 0;
  virtual BackendReply poll(double now) = 0;
  // Detach local completion tracking only; never implies remote command rollback.
  virtual void detach(uint64_t id) = 0;
};
}  // namespace flight_core::control
