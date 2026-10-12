#include "flight_core/core/waypoint_mission.hpp"
#include <limits>
namespace flight_core::control {
void WaypointMission::fail(Code code,const std::string & reason) {
  phase_=Phase::Failed; code_=code; detail_=reason; within_=-1; dwell_elapsed_=0;
}
bool WaypointMission::start(const MissionGoal & g,const BackendState & t,double now) {
  if(busy()) return false;
  completed_=cycles_=index_=0; dwell_elapsed_=0; within_=-1;
  const auto reject=[this](Code code,const char * text){fail(code,text);return false;};
  if(!std::isfinite(now) || g.frame!="map" || g.points.empty() || g.points.size()>500 ||
      !std::isfinite(g.position_tolerance) || g.position_tolerance<=0 ||
      !std::isfinite(g.yaw_tolerance) || g.yaw_tolerance<=0 || g.yaw_tolerance>3.1415926536 ||
      !std::isfinite(g.dwell) || g.dwell<0 || !std::isfinite(g.timeout) ||
      g.timeout<=0 || g.dwell>g.timeout) return reject(Code::InvalidGoal,"invalid mission options");
  if(!t.positioned()) return reject(Code::TelemetryLost,"fresh position required");
  if(!t.armed || !t.control_active || t.ground!=Ground::InAir)
    return reject(Code::ControlLost,"take off and establish control before mission");
  Pose previous=t.pose;
  for(const auto & p:g.points) {
    if(!valid_pose(p) || distance(previous,p)>10) return reject(Code::InvalidGoal,"invalid waypoint or segment exceeds 10 m");
    previous=p;
  }
  if(g.loop && distance(g.points.back(),g.points.front())>10)
    return reject(Code::InvalidGoal,"loop closing segment exceeds 10 m");
  goal_=g; target_=yaw_only(g.points.front()); activated_=last_=now;
  phase_=Phase::Executing; code_=Code::Ok; detail_="executing waypoint"; return true;
}
void WaypointMission::tick(const BackendState & t,double now) {
  if(!busy()) return;
  if(!std::isfinite(now) || now<last_ || now-last_>0.25) {fail(Code::InternalError,"mission timer gap exceeded");return;}
  last_=now;
  if(!t.positioned()) {fail(Code::TelemetryLost,"mission position lost");return;}
  if(!t.armed || !t.control_active || t.ground!=Ground::InAir) {fail(Code::ControlLost,"mission control lost");return;}
  if(distance(t.pose,target_)>10) {fail(Code::InvalidGoal,"active waypoint outside distance bound");return;}
  if(now-activated_>goal_.timeout) {fail(Code::Timeout,"waypoint deadline exceeded");return;}
  const auto yaw=[](const Pose & p){return std::atan2(2*(p.qw*p.qz+p.qx*p.qy),1-2*(p.qy*p.qy+p.qz*p.qz));};
  const double d=yaw(t.pose)-yaw(target_);
  const bool arrived=distance(t.pose,target_)<=goal_.position_tolerance &&
    (!goal_.check_yaw || std::abs(std::atan2(std::sin(d),std::cos(d)))<=goal_.yaw_tolerance);
  if(!arrived) {within_=-1; dwell_elapsed_=0; phase_=Phase::Executing;return;}
  if(within_<0) within_=now;
  dwell_elapsed_=now-within_; phase_=Phase::Settling;
  if(dwell_elapsed_<goal_.dwell) return;
  if(completed_==std::numeric_limits<uint32_t>::max()) {fail(Code::InternalError,"mission counter exhausted");return;}
  ++completed_;
  if(index_+1==goal_.points.size()) {
    if(!goal_.loop) {phase_=Phase::Succeeded;detail_="all waypoints reached; holding final target";return;}
    if(cycles_==std::numeric_limits<uint32_t>::max()) {fail(Code::InternalError,"cycle counter exhausted");return;}
    ++cycles_; index_=0;
  } else ++index_;
  target_=yaw_only(goal_.points[index_]); activated_=now;within_=-1;dwell_elapsed_=0;phase_=Phase::Executing;
}
bool WaypointMission::cancel(const BackendState & t) {
  if(!busy()) return false;
  if(!t.positioned() || !t.armed || !t.control_active) {fail(Code::TelemetryLost,"cannot establish stop hold");return false;}
  target_=yaw_only(t.pose);phase_=Phase::Canceled;code_=Code::Preempted;detail_="mission stopped; holding current pose";
  return true;
}
}
