#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <flight_interfaces/action/arm.hpp>
#include <flight_interfaces/action/disarm.hpp>
#include <flight_interfaces/action/takeoff.hpp>
#include <flight_interfaces/action/land.hpp>
#include <flight_interfaces/action/execute_waypoints.hpp>
#include "flight_core/core/waypoint_mission.hpp"
#include <flight_interfaces/msg/control_status.hpp>
#include "flight_core/core/basic_action_machine.hpp"
#include "flight_core/backends/mavros/mavros_backend.hpp"
#include "flight_core/nodes/control_lock.hpp"

namespace flight_core::control {
using Arm=flight_interfaces::action::Arm;
using Disarm=flight_interfaces::action::Disarm;
using Takeoff=flight_interfaces::action::Takeoff;
using Land=flight_interfaces::action::Land;
using Waypoints=flight_interfaces::action::ExecuteWaypoints;
static std::string phase_name(Phase p) {
  switch(p) {
    case Phase::Idle:return "idle"; case Phase::Executing:return "executing";
    case Phase::Settling:return "settling"; case Phase::Succeeded:return "succeeded";
    case Phase::Failed:return "failed"; case Phase::Canceling:return "canceling";
    case Phase::Canceled:return "canceled";
  } return "unknown";
}
static std::string operation_name(Operation op) {
  switch(op) {
    case Operation::Arm:return "arm"; case Operation::Disarm:return "disarm";
    case Operation::Takeoff:return "takeoff"; case Operation::Land:return "land";
    case Operation::Hold:return "hold";
  } return "unknown";
}
static geometry_msgs::msg::Pose pose_msg(const Pose & p) {
  geometry_msgs::msg::Pose msg;
  msg.position.x=p.x; msg.position.y=p.y; msg.position.z=p.z;
  msg.orientation.x=p.qx; msg.orientation.y=p.qy; msg.orientation.z=p.qz; msg.orientation.w=p.qw;
  return msg;
}
class FlightActionsNode : public rclcpp::Node {
public:
  FlightActionsNode():Node("flight_core") {
    if(get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("use_sim_time must be false");
    auto ns=declare_parameter<std::string>("mavros_namespace","/mavros");
    if(ns.empty() || ns.front()!='/') throw std::invalid_argument("absolute MAVROS namespace required");
    while(!ns.empty() && ns.back()=='/') ns.pop_back();
    lock_=std::make_unique<ControlLock>(ns);
    backend_=std::make_unique<MavrosBackend>(*this,Config{},ns);
    machine_=std::make_unique<BasicActionMachine>(*backend_);
    status_pub_=create_publisher<flight_interfaces::msg::ControlStatus>("~/control_status",rclcpp::QoS(1).reliable().transient_local());
    arm_=server<Arm>("~/arm",Operation::Arm);
    disarm_=server<Disarm>("~/disarm",Operation::Disarm);
    takeoff_=server<Takeoff>("~/takeoff",Operation::Takeoff);
    land_=server<Land>("~/land_action",Operation::Land);
    create_mission_server();
    land_service_=create_service<std_srvs::srv::Trigger>("~/land",[this](
      const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r) {
      r->success=request_land(); r->message=r->success?"landing accepted; observe control_status":"landing rejected";
    });
    stop_=create_service<std_srvs::srv::Trigger>("~/stop",[this](
      const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r) {
      if(recovery_at_>=0) {
        r->success=false;r->message="failure landing cleanup cannot be stopped";
      } else if(mission_handle_) {
        r->success=stop_mission(); r->message=r->success?"mission stop accepted; observe result":"cannot stop mission safely";
      } else if(machine_->busy() && machine_->operation()==Operation::Takeoff) {
        r->success=machine_->cancel(steady_seconds());
        r->message="takeoff stop requests landing cleanup; observe action result";
      } else {r->success=backend_->holding(); r->message=r->success?"already holding":"no stoppable operation";}
    });
    guard_=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult r; r.successful=false; r.reason="startup-only parameters"; return r;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
private:
  struct Client {
    std::function<void(Phase,Code,const std::string &)> finish;
    std::function<void(const flight_interfaces::msg::ActionFeedback &)> feedback;
  };
  void flush_completed() {
    if(!machine_->busy() && !clients_.empty()) {
      const auto s=machine_->status(); finish_clients(s.phase,s.code,s.detail);
    }
  }
  bool request_land(double now=steady_seconds()) {
    if(reserved_) return false;
    flush_completed();
    const bool preempt=machine_->busy() && machine_->operation()!=Operation::Land;
    if(!machine_->land(now)) return false;
    if(mission_handle_) {
      mission_.fail(Code::Preempted,"mission preempted by landing"); finish_mission();
    }
    recovery_at_=-1; mission_visible_=false;
    if(preempt) finish_clients(Phase::Failed,Code::Preempted,"preempted by landing");
    return true;
  }
  void finish_clients(Phase phase,Code code,const std::string & detail) {
    auto clients=std::move(clients_); clients_.clear();
    for(auto & client:clients) client.finish(phase,code,detail);
  }
  template<class A> typename rclcpp_action::Server<A>::SharedPtr server(const std::string & name,Operation op) {
    return rclcpp_action::create_server<A>(this,name,
      [this,op](const rclcpp_action::GoalUUID &,std::shared_ptr<const typename A::Goal>) {
        flush_completed();
        // Only reserve here. Sending commands occurs after the goal is accepted.
        if(reserved_ || ((machine_->busy() || mission_handle_ || recovery_at_>=0) && op!=Operation::Land)) return rclcpp_action::GoalResponse::REJECT;
        reserved_=true; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this,op](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>> handle) {
        if(op!=Operation::Takeoff || !handle->is_active() || !machine_->cancel(steady_seconds()))
          return rclcpp_action::CancelResponse::REJECT;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this,op](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>> handle) {
        reserved_=false;
        Client client;
        client.finish=[handle,op](Phase phase,Code code,const std::string & detail) {
          if(op==Operation::Land && phase==Phase::Canceled) {phase=Phase::Succeeded;code=Code::Ok;}
          auto result=std::make_shared<typename A::Result>();
          result->result.outcome_code=static_cast<uint16_t>(code); result->result.message=detail;
          if(phase==Phase::Succeeded) handle->succeed(result);
          else if(phase==Phase::Canceled && handle->is_canceling()) handle->canceled(result);
          else handle->abort(result);
        };
        client.feedback=[handle](const flight_interfaces::msg::ActionFeedback & f) {
          auto feedback=std::make_shared<typename A::Feedback>(); feedback->feedback=f;
          if(handle->is_active()) handle->publish_feedback(feedback);
        };
        double height=0;
        if constexpr(std::is_same_v<A,Takeoff>) height=handle->get_goal()->height_m;
        const bool accepted=op==Operation::Land?request_land():machine_->start(op,steady_seconds(),height);
        if(!accepted) {
          const auto & s=machine_->status();
          client.finish(Phase::Failed,machine_->busy()?Code::Busy:s.code,
            machine_->busy()?"another action is executing":s.detail); return;
        }
        mission_visible_=false;
        clients_.push_back(std::move(client));
      });
  }
  void tick() {
    const double now=steady_seconds();
    if(recovery_at_>=0 && now>=recovery_at_) {
      recovery_at_=-1;
      if(!request_land(now)) backend_->detach(mission_wire_id_);
    }
    if(machine_->busy()) machine_->tick(now);
    else backend_->poll(now);  // Keep normal hold streaming and monitor its health.
    update_mission(now);
    const auto t=backend_->state(now); const auto & s=machine_->status();
    const auto reply=backend_->last_reply();
    if(recovery_at_>=0 && (!t.positioned() || !t.armed || !t.control_active || reply.progress==Progress::Rejected)) {
      recovery_at_=-1;backend_->detach(mission_wire_id_);
      mission_.fail(Code::ControlLost,"control lost during failure hold; no automatic takeover");
    }
    flight_interfaces::msg::ActionFeedback f;
    const Pose target=mission_visible_?mission_.target():machine_->target();
    f.phase=mission_visible_?phase_name(mission_.phase()):phase_name(s.phase); f.pose_valid=t.positioned();
    f.current_pose.header.stamp=this->now(); f.current_pose.header.frame_id="map";
    f.target_pose.header=f.current_pose.header;
    f.current_pose.pose=pose_msg(t.pose); f.target_pose.pose=pose_msg(target);
    f.position_error_m=t.positioned()?distance(t.pose,target):-1;
    const auto yaw=[](const Pose & p){return std::atan2(2*(p.qw*p.qz+p.qx*p.qy),1-2*(p.qy*p.qy+p.qz*p.qz));};
    const double d=yaw(t.pose)-yaw(target);
    f.yaw_error_rad=t.positioned()?std::abs(std::atan2(std::sin(d),std::cos(d))):-1;
    if(mission_handle_) {
      auto feedback=std::make_shared<Waypoints::Feedback>(); feedback->feedback=f;
      feedback->current_index=mission_.index();feedback->completed_count=mission_.completed();
      feedback->cycle_count=mission_.cycles();feedback->dwell_elapsed_sec=mission_.dwell_elapsed();
      mission_handle_->publish_feedback(feedback);
    }
    for(auto & client:clients_) client.feedback(f);
    if(!machine_->busy() && !clients_.empty()) finish_clients(s.phase,s.code,s.detail);
    flight_interfaces::msg::ControlStatus msg;
    msg.header=f.current_pose.header; msg.operation=operation_name(machine_->operation());
    msg.phase=f.phase; msg.operation_id=s.id; msg.outcome_code=static_cast<uint16_t>(s.code); msg.detail=s.detail;
    msg.connected=t.connected; msg.telemetry_valid=t.healthy(); msg.pose_valid=t.positioned();
    msg.armed=t.armed; msg.control_active=t.control_active; msg.holding=backend_->holding() && !mission_.busy() && recovery_at_<0;
    msg.ground_state=static_cast<uint8_t>(t.ground); msg.backend_mode=backend_->mode(now);
    msg.current_pose=f.current_pose.pose; msg.target_pose=f.target_pose.pose;
    if(msg.holding) {msg.operation="hold";msg.phase="holding";}
    if(mission_visible_) {
      msg.operation="waypoints";msg.phase=phase_name(mission_.phase());msg.operation_id=mission_wire_id_;
      msg.outcome_code=static_cast<uint16_t>(mission_.code());msg.detail=mission_.detail();
      msg.mission_active=mission_.busy();msg.waypoint_index=mission_.index();
      msg.completed_count=mission_.completed();msg.cycle_count=mission_.cycles();msg.dwell_elapsed_sec=mission_.dwell_elapsed();
    }
    if(recovery_at_>=0) {msg.phase="failure_hold";msg.detail+="; landing after 2 s";}
    if(reply.progress==Progress::Rejected) {msg.phase="failed"; msg.detail=reply.detail; msg.outcome_code=static_cast<uint16_t>(Code::BackendRejected);}
    status_pub_->publish(msg);
  }
  void create_mission_server() {
    mission_server_=rclcpp_action::create_server<Waypoints>(this,"~/execute_waypoints",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Waypoints::Goal>) {
        flush_completed();
        if(reserved_ || machine_->busy() || mission_handle_ || recovery_at_>=0)
          return rclcpp_action::GoalResponse::REJECT;
        reserved_=true;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Waypoints>> handle) {
        if(handle!=mission_handle_ || !stop_mission()) return rclcpp_action::CancelResponse::REJECT;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Waypoints>> handle) {
        reserved_=false;
        const auto g=handle->get_goal(); MissionGoal goal;
        goal.frame=g->frame_id;goal.position_tolerance=g->position_tolerance_m;
        goal.check_yaw=g->check_yaw;goal.yaw_tolerance=g->yaw_tolerance_rad;
        goal.dwell=g->dwell_sec;goal.timeout=g->waypoint_timeout_sec;goal.loop=g->loop;
        for(const auto & w:g->waypoints) {
          goal.points.push_back({w.x,w.y,w.z,0,0,std::sin(w.yaw_rad/2),std::cos(w.yaw_rad/2)});
        }
        const auto now=steady_seconds();
        // Raw OFFBOARD alone is not proof that this process owns control.
        if(!backend_->holding()) {
          auto result=std::make_shared<Waypoints::Result>();
          result->result.outcome_code=static_cast<uint16_t>(Code::ControlLost);
          result->result.message="this core must hold control before executing waypoints";
          handle->abort(result);return;
        }
        WaypointMission candidate;
        if(!candidate.start(goal,backend_->state(now),now)) {
          auto result=std::make_shared<Waypoints::Result>();
          result->result.outcome_code=static_cast<uint16_t>(candidate.code());result->result.message=candidate.detail();
          handle->abort(result);return;
        }
        mission_=std::move(candidate);mission_handle_=handle;mission_visible_=true;
        command_mission_target(now);
      });
  }
  void command_mission_target(double now) {
    backend_->begin({++mission_wire_id_,Operation::Hold,mission_.target()},now);
  }
  bool stop_mission() {
    if(!mission_handle_ || !mission_.busy()) return false;
    const auto now=steady_seconds();
    const bool accepted=mission_.cancel(backend_->state(now));
    if(accepted) command_mission_target(now);
    else backend_->detach(mission_wire_id_);
    return accepted;
  }
  void finish_mission() {
    if(!mission_handle_) return;
    auto result=std::make_shared<Waypoints::Result>();
    result->result.outcome_code=static_cast<uint16_t>(mission_.code());result->result.message=mission_.detail();
    result->completed_count=mission_.completed();
    if(mission_.phase()==Phase::Succeeded) mission_handle_->succeed(result);
    else if(mission_.phase()==Phase::Canceled && mission_handle_->is_canceling()) mission_handle_->canceled(result);
    else mission_handle_->abort(result);
    mission_handle_.reset();
  }
  void update_mission(double now) {
    if(!mission_handle_) return;
    const auto t=backend_->state(now); const auto reply=backend_->last_reply();
    const auto before=mission_.completed();
    if(reply.id==mission_wire_id_ && reply.progress==Progress::Rejected)
      mission_.fail(Code::BackendRejected,reply.detail);
    else mission_.tick(t,now);
    if(mission_.busy()) {
      if(mission_.completed()!=before) command_mission_target(now);
      return;
    }
    if(mission_.phase()==Phase::Canceled) {
      if(reply.id!=mission_wire_id_ || reply.progress!=Progress::Accepted) return;
    }
    if(mission_.phase()==Phase::Failed) {
      if((mission_.code()==Code::Timeout || mission_.code()==Code::InvalidGoal) &&
          t.positioned() && t.armed && t.control_active) {
        backend_->begin({++mission_wire_id_,Operation::Hold,yaw_only(t.pose)},now);
        recovery_at_=now+2.0;
      } else backend_->detach(mission_wire_id_);
    }
    finish_mission();
  }
  WaypointMission mission_;
  bool mission_visible_{};
  double recovery_at_{-1};
  uint64_t mission_wire_id_{uint64_t{1}<<63};
  rclcpp_action::Server<Waypoints>::SharedPtr mission_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Waypoints>> mission_handle_;
  std::unique_ptr<ControlLock> lock_;
  std::unique_ptr<MavrosBackend> backend_;
  std::unique_ptr<BasicActionMachine> machine_;
  bool reserved_{};
  std::vector<Client> clients_;
  rclcpp_action::Server<Arm>::SharedPtr arm_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_;
  rclcpp_action::Server<Land>::SharedPtr land_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_,stop_;
  rclcpp::Publisher<flight_interfaces::msg::ControlStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<flight_core::control::FlightActionsNode>());}
  catch(const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("flight_actions"),"%s",e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
