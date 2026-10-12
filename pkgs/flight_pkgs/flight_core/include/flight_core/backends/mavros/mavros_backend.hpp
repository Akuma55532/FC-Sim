#pragma once
#include "flight_core/core/flight_backend.hpp"
#include "flight_core/backends/mavros/mavros_adapter.hpp"
namespace flight_core::control {
class MavrosBackend final : public FlightBackend {
public:
  MavrosBackend(rclcpp::Node & node, Config config, const std::string & ns)
  : adapter_(node,config,ns), config_(config) {}
  bool supports(Operation) const override {return true;}
  BackendState state(double now) const override;
  void begin(const BackendRequest &, double now) override;
  BackendReply poll(double now) override;
  void detach(uint64_t id) override;
  const std::string & mode(double now) {mode_=adapter_.telemetry(now).mode; return mode_;}
  BackendReply last_reply() const {return reply_;}
  bool holding() const {return active_ && request_.operation==Operation::Hold;}
private:
  enum class Stage { Idle, GroundReset, Prestream, Mode, Arm, Fly, Land, GroundArm, GroundDisarm };
  void fail(const std::string & reason);
  void command(Action action, double now);
  void stage(Stage value, double now);
  MavrosAdapter adapter_;
  Config config_;
  BackendRequest request_{0,Operation::Arm,{}};
  BackendReply reply_{0,Progress::Running,"idle"};
  Pose anchor_;
  Stage stage_{Stage::Idle};
  std::string initial_mode_, mode_;
  bool active_{false}, seen_tick_{false}, land_confirmed_{false}, initial_armed_{false};
  double stage_at_{}, last_tick_{}, sent_at_{-1}, cleanup_until_{};
  unsigned attempts_{};
  uint64_t wire_id_{}, pending_{};
};
}  // namespace flight_core::control
