#include <gtest/gtest.h>
#include "flight_core/core/basic_action_machine.hpp"
using namespace flight_core;
using namespace flight_core::control;
struct FakeBackend : FlightBackend {
  BackendState t{true,true,true,false,false,Ground::OnGround,{}};
  BackendRequest request{0,Operation::Arm,{}};
  BackendReply reply{0,Progress::Running,""};
  int sends{}; bool capable{true};
  bool supports(Operation) const override {return capable;}
  BackendState state(double) const override {return t;}
  void begin(const BackendRequest & r,double) override {request=r; ++sends;}
  BackendReply poll(double) override {return reply;}
  void detach(uint64_t) override {}
};
TEST(BasicActions, AckDoesNotCompleteArm) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  b.reply={m.status().id,Progress::Accepted,"ok"}; m.tick(1);
  EXPECT_TRUE(m.busy()); b.t.armed=true; m.tick(2); EXPECT_EQ(m.status().phase,Phase::Succeeded);
}
TEST(BasicActions, AirborneDisarmRejected) {
  FakeBackend b; b.t.armed=true; b.t.ground=Ground::InAir; BasicActionMachine m(b);
  EXPECT_FALSE(m.start(Operation::Disarm,0)); EXPECT_EQ(b.sends,0);
}
TEST(BasicActions, BusyPreservesOperation) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  auto id=m.status().id; EXPECT_FALSE(m.start(Operation::Disarm,1));
  EXPECT_EQ(m.status().id,id); EXPECT_EQ(b.sends,1);
}
TEST(BasicActions, CancelIgnoresStaleReply) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Takeoff,0,2));
  auto old=m.status().id; ASSERT_TRUE(m.cancel(1)); b.t.armed=true;
  b.reply={old,Progress::Rejected,"old failure"}; m.tick(2);
  EXPECT_EQ(m.status().phase,Phase::Canceling);
  b.t.armed=false; b.reply={m.status().id,Progress::Accepted,"landed"}; m.tick(3); EXPECT_EQ(m.status().phase,Phase::Canceled);
}
TEST(BasicActions, LandRequiresGroundAndDisarmed) {
  FakeBackend b; b.t.armed=true; b.t.ground=Ground::InAir; BasicActionMachine m(b);
  ASSERT_TRUE(m.start(Operation::Land,0)); b.t.ground=Ground::OnGround; m.tick(1);
  EXPECT_TRUE(m.busy()); EXPECT_FALSE(m.cancel(1));
  b.t.armed=false; b.reply={m.status().id,Progress::Accepted,"landed"}; m.tick(2); EXPECT_EQ(m.status().phase,Phase::Succeeded);
}
TEST(BasicActions, TakeoffChecksXYZYawDwell) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Takeoff,0,2));
  b.t.armed=true; b.t.control_active=true; b.t.ground=Ground::InAir;
  b.t.pose.z=2; b.t.pose.x=1; m.tick(1); EXPECT_EQ(m.status().phase,Phase::Executing);
  b.t.pose.x=0; b.t.pose.qz=std::sin(0.5); b.t.pose.qw=std::cos(0.5);
  m.tick(2); EXPECT_EQ(m.status().phase,Phase::Executing);
  b.t.pose.qz=0; b.t.pose.qw=1; m.tick(3); m.tick(3.4); EXPECT_TRUE(m.busy());
  b.t.pose.z=1; m.tick(3.5); b.t.pose.z=2; m.tick(4); m.tick(4.5);
  EXPECT_EQ(m.status().phase,Phase::Succeeded); EXPECT_EQ(b.request.operation,Operation::Hold);
}
TEST(BasicActions, StaleTelemetryFails) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  b.t.state_fresh=false; m.tick(1); EXPECT_EQ(m.status().code,Code::TelemetryLost);
}
TEST(BasicActions, DeadlineAndCapabilities) {
  FakeBackend b; BasicActionMachine m(b); b.capable=false;
  EXPECT_FALSE(m.start(Operation::Arm,0)); EXPECT_EQ(b.sends,0);
  b.capable=true; ASSERT_TRUE(m.start(Operation::Arm,1)); m.tick(62);
  EXPECT_EQ(m.status().code,Code::Timeout);
}
TEST(BasicActions, ExternalArmDoesNotGrantOwnership) {
  FakeBackend b; b.t.armed=true; BasicActionMachine m(b);
  ASSERT_TRUE(m.start(Operation::Arm,0)); EXPECT_EQ(b.sends,0);
  EXPECT_FALSE(m.start(Operation::Takeoff,1,2)); EXPECT_EQ(m.status().code,Code::ControlLost);
}
TEST(BasicActions, OwnArmAllowsTakeoff) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  b.t.armed=true; m.tick(1); ASSERT_TRUE(m.start(Operation::Takeoff,2,2));
  EXPECT_EQ(b.request.operation,Operation::Takeoff); EXPECT_DOUBLE_EQ(b.request.target.z,2);
}
TEST(BasicActions, InvalidHeightNeverSends) {
  FakeBackend b; BasicActionMachine m(b);
  EXPECT_FALSE(m.start(Operation::Takeoff,0,0));
  EXPECT_FALSE(m.start(Operation::Takeoff,0,std::nan(""))); EXPECT_EQ(b.sends,0);
}

TEST(BasicActions, TakeoffLosingControlFails) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Takeoff,0,2));
  b.t.armed=true; b.t.control_active=true; m.tick(1);
  b.t.control_active=false; m.tick(2); EXPECT_EQ(m.status().code,Code::ControlLost);
}
TEST(BasicActions, BackendRejectionIsReported) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  b.reply={m.status().id,Progress::Rejected,"health check failed"}; m.tick(1);
  EXPECT_EQ(m.status().code,Code::BackendRejected);
  EXPECT_EQ(m.status().detail,"health check failed");
}

TEST(BasicActions, LandPreemptsTakeoffAndRequiresCleanupConfirmation) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Takeoff,0,2));
  const auto old=m.status().id; ASSERT_TRUE(m.land(1)); EXPECT_NE(m.status().id,old);
  m.tick(2); EXPECT_TRUE(m.busy());
  b.reply={m.status().id,Progress::Accepted,"cleanup confirmed"}; m.tick(3);
  EXPECT_EQ(m.status().phase,Phase::Succeeded);
}

TEST(BasicActions, DisarmedResidualControlMayPrepareTakeoff) {
  FakeBackend b; b.t.control_active=true; BasicActionMachine m(b);
  ASSERT_TRUE(m.start(Operation::Takeoff,0,2)); m.tick(0.1);
  b.t.control_active=false; m.tick(0.2);
  EXPECT_TRUE(m.busy());
}
TEST(BasicActions, ArmedResidualControlStillRejectsTakeover) {
  FakeBackend b; BasicActionMachine m(b); ASSERT_TRUE(m.start(Operation::Arm,0));
  b.t.armed=true; m.tick(0.1); b.t.control_active=true;
  EXPECT_FALSE(m.start(Operation::Takeoff,0.2,2));
  EXPECT_EQ(m.status().code,Code::ControlLost);
}
