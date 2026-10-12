#include "flight_core/backends/mavros/mavros_adapter.hpp"
#include <chrono>
#include <limits>
namespace flight_core {
double stamp_seconds(const builtin_interfaces::msg::Time & stamp) {
  if (stamp.sec<0 || stamp.nanosec>=1000000000u) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(stamp.sec)+static_cast<double>(stamp.nanosec)*1e-9;
}
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Pose from_ros(const geometry_msgs::msg::PoseStamped & msg) {
  const auto & p=msg.pose.position; const auto & q=msg.pose.orientation;
  return {p.x,p.y,p.z,q.x,q.y,q.z,q.w};
}
bool MavrosAdapter::stamp_valid(const builtin_interfaces::msg::Time & stamp, double age) const {
  double s=stamp_seconds(stamp);
  double diff=node_.now().seconds()-s;
  return std::isfinite(s) && s>0 && diff>=-config_.future_tolerance_sec && diff<=age;
}
MavrosAdapter::MavrosAdapter(rclcpp::Node & node, Config config, const std::string & prefix)
: node_(node), config_(config) {
  pose_pub_=node_.create_publisher<geometry_msgs::msg::PoseStamped>("~/pose",rclcpp::SensorDataQoS());
  setpoint_pub_=node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    prefix+"/setpoint_position/local",rclcpp::QoS(1).reliable());
  const auto state_qos=rclcpp::QoS(10).reliable().transient_local();
  state_sub_=node_.create_subscription<mavros_msgs::msg::State>(prefix+"/state",state_qos,
    [this](mavros_msgs::msg::State::ConstSharedPtr m) {
      const double stamp=stamp_seconds(m->header.stamp);
      if (stamp<=state_stamp_ || !stamp_valid(m->header.stamp,config_.state_timeout_sec)) {return;}
      state_stamp_=stamp;
      latest_.connected=m->connected; latest_.armed=m->armed; latest_.mode=m->mode;
      state_at_=steady_seconds();
    });
  extended_sub_=node_.create_subscription<mavros_msgs::msg::ExtendedState>(
    prefix+"/extended_state",state_qos,[this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m) {
      const double stamp=stamp_seconds(m->header.stamp);
      if (stamp<=extended_stamp_ || !stamp_valid(m->header.stamp,config_.extended_timeout_sec)) {return;}
      extended_stamp_=stamp;
      latest_.landed_state=m->landed_state; extended_at_=steady_seconds();
    });
  pose_sub_=node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    prefix+"/local_position/pose",rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
      const double stamp=stamp_seconds(m->header.stamp);
      if (m->header.frame_id!="map" || !valid_pose(from_ros(*m)) || stamp<=pose_stamp_ ||
        !stamp_valid(m->header.stamp,config_.pose_timeout_sec)) {return;}
      latest_.pose=from_ros(*m); pose_stamp_=stamp; pose_at_=steady_seconds();
      pose_pub_->publish(*m);  // Preserve the observation timestamp.
    });
  mode_client_=node_.create_client<mavros_msgs::srv::SetMode>(prefix+"/set_mode");
  arm_client_=node_.create_client<mavros_msgs::srv::CommandBool>(prefix+"/cmd/arming");
}
Telemetry MavrosAdapter::telemetry(double now) const {
  Telemetry t=latest_;
  t.state_fresh=state_at_>=0 && now-state_at_<=config_.state_timeout_sec;
  t.pose_fresh=pose_at_>=0 && now-pose_at_<=config_.pose_timeout_sec;
  t.extended_fresh=extended_at_>=0 && now-extended_at_<=config_.extended_timeout_sec;
  t.connected=t.connected && t.state_fresh;
  return t;
}
void MavrosAdapter::publish(const Pose & p) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp=node_.now(); msg.header.frame_id="map";
  msg.pose.position.x=p.x; msg.pose.position.y=p.y; msg.pose.position.z=p.z;
  msg.pose.orientation.x=p.qx; msg.pose.orientation.y=p.qy;
  msg.pose.orientation.z=p.qz; msg.pose.orientation.w=p.qw;
  setpoint_pub_->publish(msg);
}
void MavrosAdapter::cancel_pending() {
  if (mode_pending_) {mode_client_->remove_pending_request(*mode_pending_); mode_pending_.reset();}
  if (arm_pending_) {arm_client_->remove_pending_request(*arm_pending_); arm_pending_.reset();}
  current_request_=0;
}
void MavrosAdapter::request(const Request & request, Reply reply) {
  cancel_pending(); current_request_=request.id;
  const auto id=request.id;
  if (request.action==Action::Arm || request.action==Action::Disarm) {
    if (!arm_client_->service_is_ready()) {reply(id,false,"arming service unavailable"); return;}
    auto req=std::make_shared<mavros_msgs::srv::CommandBool::Request>(); req->value=request.action==Action::Arm;
    auto future=arm_client_->async_send_request(req,
      [this,id,reply](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
        if (id!=current_request_) {return;}
        arm_pending_.reset();
        try {auto r=f.get(); reply(id,r->success,"MAV_RESULT="+std::to_string(r->result));}
        catch (const std::exception & e) {reply(id,false,e.what());}
      });
    arm_pending_=future.request_id;
  } else {
    if (!mode_client_->service_is_ready()) {reply(id,false,"mode service unavailable"); return;}
    auto req=std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode=0; req->custom_mode=request.action==Action::Land ? "AUTO.LAND" :
      (request.action==Action::GroundMode ? "AUTO.LOITER" : "OFFBOARD");
    auto future=mode_client_->async_send_request(req,
      [this,id,reply](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f) {
        if (id!=current_request_) {return;}
        mode_pending_.reset();
        try {reply(id,f.get()->mode_sent,"mode request not sent");}
        catch (const std::exception & e) {reply(id,false,e.what());}
      });
    mode_pending_=future.request_id;
  }
}
}  // namespace flight_core
