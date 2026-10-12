#include "flight_core/core/basic_action_machine.hpp"
#include <cmath>
namespace flight_core::control {
bool BasicActionMachine::busy() const {
  return status_.phase==Phase::Executing || status_.phase==Phase::Settling ||
         status_.phase==Phase::Canceling;
}
void BasicActionMachine::finish(Phase phase, Code code, const std::string & detail) {
  backend_.detach(status_.id);
  status_.phase=phase; status_.code=code; status_.detail=detail;
}
bool BasicActionMachine::start(Operation operation, double now, double height) {
  if (busy()) {return false;}  // Preserve the active operation's observable status.
  status_={++sequence_,Phase::Idle,Code::Ok,""};
  const auto reject=[this](Code code,const char * detail) {
    status_.phase=Phase::Failed; status_.code=code; status_.detail=detail; return false;
  };
  if (!std::isfinite(now)) {return reject(Code::InvalidGoal,"invalid monotonic time");}
  if (operation==Operation::Hold) {return reject(Code::Unsupported,"hold is backend-internal");}
  if (!backend_.supports(operation) || (operation==Operation::Takeoff &&
      (!backend_.supports(Operation::Land) || !backend_.supports(Operation::Hold)))) {
    return reject(Code::Unsupported,"backend lacks required capability");
  }
  const auto t=backend_.state(now);
  if (!t.healthy()) {return reject(Code::TelemetryLost,"fresh telemetry required");}
  if (operation!=Operation::Land && t.ground!=Ground::OnGround) {
    return reject(Code::InvalidGoal,"confirmed ground state required");
  }
  if (operation==Operation::Takeoff) {
    if (!std::isfinite(height) || height<=0 || height>10) {
      return reject(Code::InvalidGoal,"height must be in (0, 10] metres");
    }
    if (!t.positioned()) {return reject(Code::TelemetryLost,"fresh valid pose required");}
    if (t.armed && (!owned_arm_ || t.control_active)) {
      return reject(Code::ControlLost,"external takeover is not supported");
    }
  }
  operation_=operation; started_=last_tick_=now; within_since_=-1; control_seen_=false;
  status_.phase=Phase::Executing;
  target_=t.positioned() ? yaw_only(t.pose) : Pose{};
  if (operation==Operation::Takeoff) {
    target_.z+=height;
    if (!valid_pose(target_)) {return reject(Code::InvalidGoal,"invalid takeoff target");}
  }
  // A no-op arm request must not establish ownership of externally armed hardware.
  if ((operation==Operation::Arm && t.armed) ||
      (operation==Operation::Disarm && !t.armed) ||
      (operation==Operation::Land && t.ground==Ground::OnGround && !t.armed)) {
    status_.phase=Phase::Succeeded;
    if (!t.armed) {owned_arm_=false;}
    return true;
  }
  backend_.begin({status_.id,operation,target_},now);
  return true;
}
bool BasicActionMachine::cancel(double now) {
  if (!busy() || operation_!=Operation::Takeoff || status_.phase==Phase::Canceling ||
      !std::isfinite(now) || now<last_tick_) {return false;}
  // begin(Takeoff) may already have emitted an arm command. Always run land cleanup.
  backend_.detach(status_.id);
  status_.id=++sequence_; status_.phase=Phase::Canceling;
  operation_=Operation::Land; started_=last_tick_=now;
  backend_.begin({status_.id,Operation::Land,{}},now);
  return true;
}
bool BasicActionMachine::land(double now) {
  if (!std::isfinite(now) || !backend_.state(now).healthy()) return false;
  if (busy() && operation_==Operation::Land) return true;
  if (!busy()) return start(Operation::Land,now);
  if (!backend_.supports(Operation::Land)) return false;
  backend_.detach(status_.id);
  status_={++sequence_,Phase::Executing,Code::Ok,"landing preemption"};
  operation_=Operation::Land; started_=last_tick_=now; within_since_=-1;
  backend_.begin({status_.id,Operation::Land,{}},now);
  return true;
}
void BasicActionMachine::tick(double now) {
  if (!busy()) {return;}
  if (!std::isfinite(now) || now<last_tick_) {
    finish(Phase::Failed,Code::InternalError,"invalid monotonic clock"); return;
  }
  last_tick_=now;
  const auto t=backend_.state(now);
  if (!t.healthy() || (operation_==Operation::Takeoff && !t.positioned())) {
    finish(Phase::Failed,Code::TelemetryLost,"required telemetry lost"); return;
  }
  if (operation_==Operation::Takeoff) {
    if (control_seen_ && !t.control_active) {
      finish(Phase::Failed,Code::ControlLost,"control lost during takeoff"); return;
    }
    control_seen_=control_seen_ || (t.armed && t.control_active);
  }
  if (now-started_>60) {finish(Phase::Failed,Code::Timeout,"action deadline exceeded"); return;}
  const auto reply=backend_.poll(now);
  if (reply.id==status_.id && reply.progress==Progress::Rejected) {
    finish(Phase::Failed,Code::BackendRejected,reply.detail); return;
  }
  bool done=false;
  switch(operation_) {
    case Operation::Arm:
      done=t.armed;
      if (done) {owned_arm_=true;}
      break;
    case Operation::Disarm: done=!t.armed && t.ground==Ground::OnGround; break;
    case Operation::Land: done=!t.armed && t.ground==Ground::OnGround &&
      reply.id==status_.id && reply.progress==Progress::Accepted; break;
    case Operation::Takeoff: {
      const auto yaw=[](const Pose & p) {return std::atan2(2*(p.qw*p.qz+p.qx*p.qy),
        1-2*(p.qy*p.qy+p.qz*p.qz));};
      const double delta=yaw(t.pose)-yaw(target_);
      const bool arrived=t.armed && t.control_active && t.ground==Ground::InAir &&
        distance(t.pose,target_)<=0.1 && std::abs(std::atan2(std::sin(delta),std::cos(delta)))<=0.1745329252;
      if (!arrived) {within_since_=-1; status_.phase=Phase::Executing;}
      else {
        status_.phase=Phase::Settling;
        if (within_since_<0) {within_since_=now;}
        done=now-within_since_>=0.5;
      }
      break;
    }
    case Operation::Hold: break;
  }
  if (done) {
    const bool canceled=status_.phase==Phase::Canceling;
    if (!t.armed) {owned_arm_=false;}
    finish(canceled?Phase::Canceled:Phase::Succeeded,
      canceled?Code::Preempted:Code::Ok,canceled?"cancellation cleanup confirmed":"telemetry confirmed");
    // Hold belongs to the backend and remains active after action completion.
    if (operation_==Operation::Takeoff) {backend_.begin({++sequence_,Operation::Hold,target_},now);}
  }
}
}  // namespace flight_core::control
