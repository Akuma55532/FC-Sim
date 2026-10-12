#include "flight_core/backends/mavros/mavros_backend.hpp"
namespace flight_core::control {
BackendState MavrosBackend::state(double now) const {
  const auto t=adapter_.telemetry(now);
  Ground ground=Ground::Unknown;
  if (t.extended_fresh) {
    if (t.landed_state==1) ground=Ground::OnGround;
    else if (t.landed_state==2 || t.landed_state==3 || t.landed_state==4) ground=Ground::InAir;
  }
  return {t.connected,t.state_fresh && t.extended_fresh,t.pose_fresh,t.armed,
    t.mode=="OFFBOARD",ground,t.pose};
}
void MavrosBackend::stage(Stage value,double now) {
  adapter_.cancel_pending(); pending_=0; stage_=value; stage_at_=now; sent_at_=-1; attempts_=0;
}
void MavrosBackend::fail(const std::string & reason) {
  adapter_.cancel_pending(); pending_=0; active_=false; stage_=Stage::Idle;
  reply_={request_.id,Progress::Rejected,reason};
}
void MavrosBackend::detach(uint64_t id) {
  if (request_.id!=id) return;
  adapter_.cancel_pending(); pending_=0; active_=false; stage_=Stage::Idle;
}
void MavrosBackend::begin(const BackendRequest & r,double now) {
  adapter_.cancel_pending(); pending_=0; request_=r; active_=true;
  reply_={r.id,Progress::Running,"preparing"}; seen_tick_=false;
  const auto t=adapter_.telemetry(now); const auto s=state(now);
  initial_mode_=t.mode; initial_armed_=t.armed; land_confirmed_=false;
  anchor_=s.positioned()?yaw_only(t.pose):Pose{};
  if (!s.healthy()) {fail("fresh state and landed telemetry required"); return;}
  switch(r.operation) {
    case Operation::Takeoff:
      if (!s.positioned() || s.ground!=Ground::OnGround || t.mode.empty() ||
          (t.armed && (t.mode=="OFFBOARD" || t.mode=="AUTO.LAND"))) {fail("takeoff startup conditions not met"); return;}
      stage(!t.armed && (t.mode=="OFFBOARD" || t.mode=="AUTO.LAND") ?
        Stage::GroundReset : Stage::Prestream,now); break;
    case Operation::Hold:
      if (!s.positioned() || !t.armed || t.mode!="OFFBOARD") {fail("cannot establish hold"); return;}
      stage(Stage::Fly,now); break;
    case Operation::Arm:
      stage(!t.armed && (t.mode=="OFFBOARD" || t.mode=="AUTO.LAND") ?
        Stage::GroundReset : Stage::GroundArm,now); break;
    case Operation::Disarm: stage(Stage::GroundDisarm,now); break;
    case Operation::Land:
      // Do not finish cleanup on one pre-command disarmed sample. Allow outstanding
      // requests to settle while requesting AUTO.LAND, then confirm grounded/disarmed.
      cleanup_until_=now+config_.request_timeout_sec;
      stage(Stage::Land,now); break;
  }
}
void MavrosBackend::command(Action action,double now) {
  if (sent_at_>=0 && now-sent_at_<config_.request_timeout_sec) return;
  if (attempts_>=config_.max_request_attempts) {fail("backend request retry limit reached: "+reply_.detail); return;}
  ++attempts_; sent_at_=now; pending_=++wire_id_;
  const auto operation_id=request_.id;
  adapter_.request({pending_,action},[this,operation_id](uint64_t id,bool ok,const std::string & detail) {
    if (!active_ || operation_id!=request_.id || id!=pending_) return;
    reply_.detail=ok?"request accepted; waiting for telemetry":detail;
  });
}
BackendReply MavrosBackend::poll(double now) {
  if (!active_) return reply_;
  if (seen_tick_ && (now<last_tick_ || now-last_tick_>config_.max_publish_gap_sec)) {
    fail("backend timer gap exceeded"); return reply_;
  }
  seen_tick_=true; last_tick_=now;
  const auto t=adapter_.telemetry(now); const auto s=state(now);
  if (!s.healthy()) {fail("telemetry lost"); return reply_;}
  if (stage_==Stage::GroundReset) {
    if (s.ground!=Ground::OnGround || t.armed) {fail("ground reset requires landed and disarmed telemetry"); return reply_;}
    if (t.mode=="AUTO.LOITER") {
      initial_mode_=t.mode;
      stage(request_.operation==Operation::Takeoff?Stage::Prestream:Stage::GroundArm,now);
      reply_.detail="ground mode confirmed; preparing requested action";
    } else if (t.mode!=initial_mode_) {
      fail("mode changed externally during ground reset");
    } else {
      reply_.detail="waiting for ground AUTO.LOITER confirmation";
      command(Action::GroundMode,now);
    }
    return reply_;
  }
  if (stage_==Stage::GroundArm || stage_==Stage::GroundDisarm) {
    if (s.ground!=Ground::OnGround) {fail("ground confirmation lost"); return reply_;}
    const bool arm=stage_==Stage::GroundArm;
    if (t.armed==arm) {reply_.progress=Progress::Accepted; return reply_;}
    command(arm?Action::Arm:Action::Disarm,now); return reply_;
  }
  if (stage_==Stage::Land) {
    // A completed landing does not require AUTO.LAND to remain selected.
    // Keep the cleanup window for late requests, but do not reclaim ground mode.
    if (s.ground==Ground::OnGround && !t.armed) {
      adapter_.cancel_pending(); pending_=0;
      reply_.detail="grounded and disarmed; waiting for cleanup confirmation";
      if (now>=cleanup_until_) reply_.progress=Progress::Accepted;
      return reply_;
    }
    if (t.mode=="AUTO.LAND") {land_confirmed_=true; adapter_.cancel_pending(); pending_=0;}
    else if (land_confirmed_) {fail("landing mode changed externally"); return reply_;}
    else if (t.mode!=initial_mode_) {fail("mode changed during landing request"); return reply_;}
    if (!land_confirmed_) {
      if (t.mode=="OFFBOARD" && t.armed && s.positioned()) adapter_.publish(anchor_);
      command(Action::Land,now);
    }
    return reply_;
  }
  if (!s.positioned()) {fail("position lost"); return reply_;}
  if (stage_==Stage::Prestream || stage_==Stage::Mode) {
    if (s.ground!=Ground::OnGround || t.armed!=initial_armed_ ||
        (t.mode!=initial_mode_ && !(stage_==Stage::Mode && t.mode=="OFFBOARD"))) {
      fail("startup state changed externally"); return reply_;
    }
    adapter_.publish(anchor_);
    if (stage_==Stage::Prestream && now-stage_at_>=config_.prestream_sec) stage(Stage::Mode,now);
    if (stage_==Stage::Mode) {
      if (t.mode=="OFFBOARD") stage(Stage::Arm,now);
      else command(Action::Offboard,now);
    }
    return reply_;
  }
  if (t.mode!="OFFBOARD") {fail("OFFBOARD control lost"); return reply_;}
  if (stage_==Stage::Arm) {
    if (!t.armed) {
      if (s.ground!=Ground::OnGround) {fail("ground confirmation lost before arm"); return reply_;}
      adapter_.publish(anchor_); command(Action::Arm,now); return reply_;
    }
    stage(Stage::Fly,now);
  }
  if (!t.armed) {fail("arm state lost"); return reply_;}
  if (distance(request_.target,t.pose)>config_.max_target_distance_m) {fail("target outside distance bound"); return reply_;}
  adapter_.publish(request_.target); reply_.progress=Progress::Accepted;
  reply_.detail=request_.operation==Operation::Hold?"holding":"tracking takeoff target";
  return reply_;
}
}  // namespace flight_core::control
