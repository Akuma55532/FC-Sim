#pragma once
#include "flight_core/core/flight_backend.hpp"

namespace flight_core::control {
enum class Phase { Idle, Executing, Settling, Succeeded, Failed, Canceling, Canceled };
struct ActionState {
  uint64_t id{};
  Phase phase{Phase::Idle};
  Code code{Code::Ok};
  std::string detail;
};
// Protocol-independent basic action coordinator.
class BasicActionMachine {
public:
  explicit BasicActionMachine(FlightBackend & backend) : backend_(backend) {}
  bool start(Operation operation, double now, double height = 0.0);
  bool cancel(double now);
  bool land(double now);
  Operation operation() const {return operation_;}
  const Pose & target() const {return target_;}
  void tick(double now);
  const ActionState & status() const {return status_;}
  bool busy() const;
private:
  void finish(Phase phase, Code code, const std::string & detail);
  FlightBackend & backend_;
  ActionState status_;
  Operation operation_{Operation::Arm};
  Pose target_;
  uint64_t sequence_{};
  double started_{}, within_since_{-1.0}, last_tick_{};
  bool owned_arm_{false};
  bool control_seen_{false};
};
}  // namespace flight_core::control
