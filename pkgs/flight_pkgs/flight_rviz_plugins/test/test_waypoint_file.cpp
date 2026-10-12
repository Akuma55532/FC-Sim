#include <gtest/gtest.h>
#include "flight_rviz_plugins/waypoint_file.hpp"
#include <limits>
using namespace flight_rviz_plugins;
TEST(WaypointFile, RoundTripPreservesMultipleVehiclesAndYawOption) {
  Document d;d["/flight_core"].points={{1,2,3,-179.5},{0,0,1,0}};
  d["/flight_core"].options.check_yaw=false;d["/uav2/flight_core"].points={{4,5,6,90}};
  const auto decoded=decode(encode(d));ASSERT_EQ(decoded.size(),2u);
  EXPECT_DOUBLE_EQ(decoded.at("/flight_core").points[0].yaw_deg,-179.5);
  EXPECT_FALSE(decoded.at("/flight_core").options.check_yaw);
  EXPECT_DOUBLE_EQ(decoded.at("/uav2/flight_core").points[0].z,6);
}
TEST(WaypointFile, RejectsInvalidValuesBeforeReplacingDocument) {
  Document d;d["/flight_core"].points={{0,0,1,0}};
  auto invalid=d;invalid.begin()->second.points[0].x=std::numeric_limits<double>::infinity();
  EXPECT_THROW(encode(invalid),std::invalid_argument);
  EXPECT_THROW(decode("schema_version: 2\nframe_id: map\nvehicles: []"),std::invalid_argument);
  auto v=d.begin()->second;v.options.dwell=61;EXPECT_THROW(validate(v),std::invalid_argument);
  v=d.begin()->second;v.points.resize(501);EXPECT_THROW(validate(v),std::invalid_argument);
  EXPECT_EQ(d.begin()->second.points.size(),1u);
}
TEST(WaypointFile, EmptyDraftAllowedAndDuplicatesRejected) {
  Document d;d["/flight_core"]={};EXPECT_TRUE(decode(encode(d)).at("/flight_core").points.empty());
  const std::string entry="  - namespace: /flight_core\n    options: {position_tolerance_m: 0.1, yaw_tolerance_deg: 10, dwell_sec: 0.5, waypoint_timeout_sec: 60, check_yaw: true, loop: false}\n    waypoints: []\n";
  EXPECT_THROW(decode("schema_version: 1\nframe_id: map\nvehicles:\n"+entry+entry),std::invalid_argument);
}
