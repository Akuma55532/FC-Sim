#pragma once
#include <vector>
#include "flight_core/core/basic_action_machine.hpp"
namespace flight_core::control {
struct MissionGoal {
  std::string frame{"map"};
  std::vector<Pose> points;
  double position_tolerance{0.1}, yaw_tolerance{0.1745329252}, dwell{0.5}, timeout{60};
  bool check_yaw{true}, loop{false};
};
class WaypointMission {
public:
  bool start(const MissionGoal &, const BackendState &, double now);
  void tick(const BackendState &, double now);
  bool cancel(const BackendState &);
  void fail(Code code,const std::string & reason);
  bool busy() const {return phase_==Phase::Executing || phase_==Phase::Settling;}
  Phase phase() const {return phase_;}
  Code code() const {return code_;}
  const std::string & detail() const {return detail_;}
  const Pose & target() const {return target_;}
  uint32_t index() const {return index_;}
  uint32_t completed() const {return completed_;}
  uint32_t cycles() const {return cycles_;}
  double dwell_elapsed() const {return dwell_elapsed_;}
private:
  MissionGoal goal_;
  Phase phase_{Phase::Idle};
  Code code_{Code::Ok};
  std::string detail_;
  Pose target_;
  uint32_t index_{},completed_{},cycles_{};
  double activated_{},within_{-1},last_{},dwell_elapsed_{};
};
}
