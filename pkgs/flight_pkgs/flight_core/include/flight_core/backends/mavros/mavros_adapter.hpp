#pragma once
#include <functional>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include "flight_core/backends/mavros/types.hpp"
namespace flight_core {
double steady_seconds();
double stamp_seconds(const builtin_interfaces::msg::Time &);
Pose from_ros(const geometry_msgs::msg::PoseStamped &);
class MavrosAdapter {
public:
  using Reply = std::function<void(uint64_t,bool,const std::string &)>;
  MavrosAdapter(rclcpp::Node &, Config, const std::string & mavros_namespace);
  Telemetry telemetry(double now) const;
  void publish(const Pose &);
  void request(const Request &, Reply);
  void cancel_pending();
private:
  bool stamp_valid(const builtin_interfaces::msg::Time &, double max_age) const;
  rclcpp::Node & node_;
  Config config_;
  Telemetry latest_;
  double state_at_{-1}, pose_at_{-1}, extended_at_{-1}, pose_stamp_{-1};
  double state_stamp_{-1}, extended_stamp_{-1};
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_, pose_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  std::optional<int64_t> mode_pending_, arm_pending_;
  uint64_t current_request_{};
};
}  // namespace flight_core
