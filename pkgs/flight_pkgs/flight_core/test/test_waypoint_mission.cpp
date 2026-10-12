#include <gtest/gtest.h>
#include "flight_core/core/waypoint_mission.hpp"
using namespace flight_core;
using namespace flight_core::control;
static BackendState air() {return {true,true,true,true,true,Ground::InAir,{0,0,2,0,0,0,1}};}
static MissionGoal goal() {MissionGoal g;g.points={{0,0,1,0,0,0,1}};g.dwell=0;return g;}
TEST(Mission, RequiresAirborneOwnedTelemetry) {
  WaypointMission m;auto t=air();t.ground=Ground::OnGround;
  EXPECT_FALSE(m.start(goal(),t,0));t=air();t.pose_fresh=false;EXPECT_FALSE(m.start(goal(),t,0));
}
TEST(Mission, RejectsWholeInvalidList) {
  WaypointMission m;auto g=goal();g.points.push_back({30,0,1,0,0,0,1});
  EXPECT_FALSE(m.start(g,air(),0));g=goal();g.points[0].z=std::nan("");EXPECT_FALSE(m.start(g,air(),0));
  g=goal();g.frame="odom";EXPECT_FALSE(m.start(g,air(),0));
}
TEST(Mission, ThreeDimensionalArrivalAndSuccessIndex) {
  WaypointMission m;auto t=air();ASSERT_TRUE(m.start(goal(),t,0));m.tick(t,0.1);
  EXPECT_TRUE(m.busy());t.pose.z=1;m.tick(t,0.2);EXPECT_EQ(m.phase(),Phase::Succeeded);
  EXPECT_EQ(m.completed(),1u);EXPECT_EQ(m.index(),0u);
}
TEST(Mission, YawOptionalButTargetUnchanged) {
  auto t=air();t.pose.z=1;auto g=goal();g.points[0].qz=1;g.points[0].qw=0;
  WaypointMission m;ASSERT_TRUE(m.start(g,t,0));m.tick(t,0.1);EXPECT_TRUE(m.busy());
  g.check_yaw=false;WaypointMission n;ASSERT_TRUE(n.start(g,t,0));n.tick(t,0.1);
  EXPECT_EQ(n.phase(),Phase::Succeeded);EXPECT_NEAR(std::abs(n.target().qz),1,1e-8);
}
TEST(Mission, YawWrapUsesShortestAngle) {
  auto t=air();t.pose.z=1;auto g=goal();const double pi=std::acos(-1.0);
  t.pose.qz=std::sin((pi-0.01)/2);t.pose.qw=std::cos((pi-0.01)/2);
  g.points[0].qz=std::sin((-pi+0.01)/2);g.points[0].qw=std::cos((-pi+0.01)/2);
  WaypointMission m;ASSERT_TRUE(m.start(g,t,0));m.tick(t,0.1);EXPECT_EQ(m.phase(),Phase::Succeeded);
}
TEST(Mission, DwellResetsOnLeavingTolerance) {
  auto t=air();t.pose.z=1;auto g=goal();g.dwell=0.3;WaypointMission m;ASSERT_TRUE(m.start(g,t,0));
  m.tick(t,0.1);m.tick(t,0.3);t.pose.z=2;m.tick(t,0.4);t.pose.z=1;m.tick(t,0.5);
  m.tick(t,0.7);EXPECT_TRUE(m.busy());m.tick(t,0.9);EXPECT_EQ(m.phase(),Phase::Succeeded);
}
TEST(Mission, LoopAndCancelSnapshot) {
  auto t=air();t.pose.z=1;auto g=goal();g.loop=true;WaypointMission m;ASSERT_TRUE(m.start(g,t,0));
  m.tick(t,0.1);m.tick(t,0.2);EXPECT_EQ(m.cycles(),2u);EXPECT_TRUE(m.busy());
  t.pose.x=0.3;ASSERT_TRUE(m.cancel(t));EXPECT_EQ(m.phase(),Phase::Canceled);EXPECT_DOUBLE_EQ(m.target().x,0.3);
}
TEST(Mission, TimeoutTelemetryLossAndClockGap) {
  auto g=goal();g.timeout=0.15;WaypointMission m;ASSERT_TRUE(m.start(g,air(),0));m.tick(air(),0.2);
  EXPECT_EQ(m.code(),Code::Timeout);WaypointMission n;ASSERT_TRUE(n.start(goal(),air(),0));
  auto t=air();t.pose_fresh=false;n.tick(t,0.1);EXPECT_EQ(n.code(),Code::TelemetryLost);
  WaypointMission k;ASSERT_TRUE(k.start(goal(),air(),0));k.tick(air(),0.3);EXPECT_EQ(k.code(),Code::InternalError);
}
TEST(Mission, MultiPointAndBusyRejection) {
  auto t=air();t.pose.z=1;auto g=goal();g.points.push_back({0,0,2,0,0,0,1});
  WaypointMission m;ASSERT_TRUE(m.start(g,t,0));EXPECT_FALSE(m.start(goal(),t,0.01));
  m.tick(t,0.1);EXPECT_EQ(m.index(),1u);EXPECT_EQ(m.completed(),1u);
  t.pose.z=2;m.tick(t,0.2);EXPECT_EQ(m.completed(),2u);EXPECT_EQ(m.phase(),Phase::Succeeded);
}

TEST(Mission, LoopClosingSegmentValidated) {
  auto t=air();auto g=goal();g.points={{-6,0,2,0,0,0,1},{0,0,2,0,0,0,1},{6,0,2,0,0,0,1}};
  g.loop=true;WaypointMission m;EXPECT_FALSE(m.start(g,t,0));
  g.loop=false;EXPECT_TRUE(m.start(g,t,0));
}
TEST(Mission, CancelWithoutFreshPoseDoesNotClaimHold) {
  auto t=air();WaypointMission m;ASSERT_TRUE(m.start(goal(),t,0));
  t.pose_fresh=false;EXPECT_FALSE(m.cancel(t));EXPECT_EQ(m.phase(),Phase::Failed);
}
TEST(Mission, OptionsMustBeFiniteAndConsistent) {
  auto g=goal();WaypointMission m;
  g.dwell=2;g.timeout=1;EXPECT_FALSE(m.start(g,air(),0));
  g=goal();g.position_tolerance=std::nan("");EXPECT_FALSE(m.start(g,air(),0));
  g=goal();g.points.clear();EXPECT_FALSE(m.start(g,air(),0));
}
