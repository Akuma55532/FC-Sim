#include <gtest/gtest.h>
#include <limits>
#include "flight_rviz_plugins/quadrotor_model.hpp"
using flight_rviz_plugins::QuadrotorModel;
TEST(QuadrotorModel, FullAttitudeRotatesBodyOffsetsAndForwardArrow) {
  QuadrotorModel model;geometry_msgs::msg::Pose p;
  p.position.x=1;p.position.y=2;p.position.z=3;
  p.orientation.y=std::sqrt(0.5);p.orientation.w=std::sqrt(0.5);
  model.update(p,true);std_msgs::msg::Header h;h.frame_id="map";
  auto m=model.markers(h,"uav1",true);ASSERT_EQ(m.markers.size(),8u);
  EXPECT_DOUBLE_EQ(m.markers[0].pose.position.z,3);
  // Positive pitch rotates body +X toward world -Z and body +Z toward +X.
  EXPECT_NEAR(m.markers[2].pose.position.x,1.035,1e-9);
  EXPECT_NEAR(m.markers[2].pose.position.z,3.22,1e-9);
  EXPECT_NEAR(m.markers[6].pose.position.x,1.07,1e-9);
  EXPECT_NEAR(m.markers[6].pose.orientation.y,p.orientation.y,1e-9);
  EXPECT_EQ(m.markers.back().text,"uav1");
}
TEST(QuadrotorModel, InvalidOrStalePosePreservesOnlyLastValidPosition) {
  QuadrotorModel model;std_msgs::msg::Header h;geometry_msgs::msg::Pose p;
  p.orientation.w=0;model.update(p,true);EXPECT_TRUE(model.markers(h,"uav",true).markers.empty());
  p.orientation.w=1;p.position.x=2;model.update(p,true);
  p.position.x=99;model.update(p,false);
  auto stale=model.markers(h,"uav",true);ASSERT_EQ(stale.markers.size(),8u);
  EXPECT_DOUBLE_EQ(stale.markers[0].pose.position.x,2);
  EXPECT_FLOAT_EQ(stale.markers[6].color.r,stale.markers[6].color.g);
  EXPECT_NE(stale.markers.back().text.find("位姿失效"),std::string::npos);
  p.position.x=std::numeric_limits<double>::quiet_NaN();model.update(p,true);
  EXPECT_DOUBLE_EQ(model.markers(h,"uav",true).markers[0].pose.position.x,2);
  p.position.x=3;model.update(p,true);
  EXPECT_NE(model.markers(h,"uav",false).markers.back().text.find("位姿失效"),std::string::npos);
  EXPECT_EQ(model.markers(h,"uav",true).markers.back().text,"uav");
  model.clear();EXPECT_TRUE(model.markers(h,"uav",true).markers.empty());
}
