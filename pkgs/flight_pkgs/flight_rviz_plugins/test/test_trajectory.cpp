#include <gtest/gtest.h>
#include <limits>
#include "flight_rviz_plugins/trajectory.hpp"
using flight_rviz_plugins::Trajectory;
static geometry_msgs::msg::Point point(double x) {geometry_msgs::msg::Point p;p.x=x;return p;}
TEST(Trajectory, RecordsMeasuredPositionsAndFiltersJitter) {
  Trajectory t;t.sample(point(0),1,true);t.sample(point(0.01),1.1,true);
  EXPECT_TRUE(t.points().empty());t.sample(point(0.2),1.2,true);
  ASSERT_EQ(t.points().size(),2u);EXPECT_DOUBLE_EQ(t.points()[0].x,0);EXPECT_DOUBLE_EQ(t.points()[1].x,0.2);
}
TEST(Trajectory, InvalidStaleAndReorderedSamplesBreakSegments) {
  Trajectory t;t.sample(point(0),1,true);t.sample(point(1),1.1,true);
  t.sample(point(2),1.2,false);t.sample(point(3),1.3,true);t.sample(point(4),1.4,true);
  t.sample(point(8),3,true);t.sample(point(9),3.1,true);
  auto points=t.points();ASSERT_EQ(points.size(),6u);
  EXPECT_DOUBLE_EQ(points[2].x,3);EXPECT_DOUBLE_EQ(points[4].x,8);
  t.sample(point(10),3.0,true);t.sample(point(11),3.2,true);EXPECT_EQ(t.points().size(),6u);
  t.sample(point(std::numeric_limits<double>::quiet_NaN()),3.3,true);
  t.sample(point(12),3.4,true);EXPECT_EQ(t.points().size(),6u);
  t.break_segment();t.sample(point(13),3.5,true);EXPECT_EQ(t.points().size(),6u);
}
TEST(Trajectory, BoundedHistoryAndClear) {
  Trajectory t;
  for(int i=0;i<2102;++i)t.sample(point(i),1+i*0.1,true);
  ASSERT_EQ(t.points().size(),4000u);EXPECT_DOUBLE_EQ(t.points().front().x,101);
  t.clear();EXPECT_TRUE(t.points().empty());t.sample(point(0),1,true);EXPECT_TRUE(t.points().empty());
}
