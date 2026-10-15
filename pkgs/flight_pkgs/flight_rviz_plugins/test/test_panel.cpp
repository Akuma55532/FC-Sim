#include <gtest/gtest.h>
#include <QApplication>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QLineEdit>
#include <rclcpp/rclcpp.hpp>
#include <flight_interfaces/msg/control_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "flight_rviz_plugins/flight_panel.hpp"
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <flight_interfaces/action/takeoff.hpp>
#include <flight_interfaces/action/execute_waypoints.hpp>
#include <QLabel>
#include <pluginlib/class_loader.hpp>
namespace {
struct Environment : testing::Environment {
  std::unique_ptr<QApplication> app;
  int argc{1}; char name[16]="panel_test"; char *argv[2]{name,nullptr};
  void SetUp() override {qputenv("QT_QPA_PLATFORM","offscreen");setenv("ROS_DOMAIN_ID","185",1);setenv("ROS_LOCALHOST_ONLY","1",1);app=std::make_unique<QApplication>(argc,argv);rclcpp::init(argc,argv);}
  void TearDown() override {rclcpp::shutdown();app.reset();}
};
auto *environment=testing::AddGlobalTestEnvironment(new Environment);
void pump(double seconds){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);while(std::chrono::steady_clock::now()<end){QApplication::processEvents();std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
}
TEST(Panel, EditingAndYawDefaultDoNotSendCommands) {
  flight_rviz_plugins::FlightPanel panel;
  auto *table=panel.findChild<QTableWidget*>("waypoints");ASSERT_NE(table,nullptr);
  EXPECT_TRUE(panel.findChild<QCheckBox*>("check_yaw")->isChecked());
  EXPECT_FALSE(panel.findChild<QPushButton*>("arm")->isEnabled());
  panel.findChild<QPushButton*>("add")->click();EXPECT_EQ(table->rowCount(),1);
  table->item(0,0)->setText("2.5");panel.findChild<QPushButton*>("insert")->click();
  EXPECT_EQ(table->rowCount(),2);panel.findChild<QPushButton*>("delete")->click();EXPECT_EQ(table->rowCount(),1);
  EXPECT_EQ(table->item(0,0)->text(),"2.5");
  rviz_common::Config config;panel.save(config);QString ns;ASSERT_TRUE(config.mapGetString("CoreNamespace",&ns));EXPECT_EQ(ns,"/flight_core");
}
TEST(Panel, StaleStatusDisablesControls) {
  flight_rviz_plugins::FlightPanel panel;
  auto *endpoint=panel.findChild<QLineEdit*>("core_namespace");endpoint->setText("/panel_test_core");panel.findChild<QPushButton*>("connect")->click();
  auto node=std::make_shared<rclcpp::Node>("panel_status_test");
  auto pub=node->create_publisher<flight_interfaces::msg::ControlStatus>("/panel_test_core/control_status",rclcpp::QoS(1).reliable().transient_local());
  flight_interfaces::msg::ControlStatus s;s.header.stamp=node->now();s.telemetry_valid=true;s.pose_valid=true;s.connected=true;s.ground_state=1;
  pub->publish(s);pump(0.5);pump(1.1);
  EXPECT_FALSE(panel.findChild<QPushButton*>("takeoff")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton*>("land")->isEnabled());
}

TEST(Panel, PluginDiscoveryAndAsyncTakeoffResult) {
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common","rviz_common::Panel");
  auto panel=loader.createSharedInstance("flight_rviz_plugins/FlightPanel");
  auto *endpoint=panel->findChild<QLineEdit*>("core_namespace");endpoint->setText("/panel_action_test");panel->findChild<QPushButton*>("connect")->click();
  using A=flight_interfaces::action::Takeoff;
  auto node=std::make_shared<rclcpp::Node>("panel_action_mock");
  auto pub=node->create_publisher<flight_interfaces::msg::ControlStatus>("/panel_action_test/control_status",rclcpp::QoS(1).reliable().transient_local());
  int goals=0;
  auto server=rclcpp_action::create_server<A>(node,"/panel_action_test/takeoff",
    [&goals](const rclcpp_action::GoalUUID &,std::shared_ptr<const A::Goal> g){++goals;EXPECT_DOUBLE_EQ(g->height_m,2.0);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>>){return rclcpp_action::CancelResponse::REJECT;},
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>> h){auto r=std::make_shared<A::Result>();r->result.outcome_code=5;r->result.message="mock refusal";h->abort(r);});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);
  auto step=[&]{flight_interfaces::msg::ControlStatus s;s.header.stamp=node->now();s.telemetry_valid=true;s.pose_valid=true;s.connected=true;s.ground_state=1;s.phase="idle";pub->publish(s);ex.spin_some();pump(0.03);};
  auto *takeoff=panel->findChild<QPushButton*>("takeoff");
  for(int i=0;i<100 && !takeoff->isEnabled();++i)step();
  ASSERT_TRUE(takeoff->isEnabled());EXPECT_EQ(goals,0);takeoff->click();
  auto *result=panel->findChild<QLabel*>("result");
  for(int i=0;i<100 && !result->text().contains("mock refusal");++i)step();
  EXPECT_EQ(goals,1);EXPECT_TRUE(result->text().contains("mock refusal"));
  pump(1.2);EXPECT_FALSE(takeoff->isEnabled());
  panel->resize(660,900);panel->show();pump(0.1);panel->grab().save("/tmp/flight_panel_preview.png");
  ex.remove_node(node);panel.reset();
}
TEST(Panel, SingleStepUsesSnapshotYawOptionAndAdvancesOnlyOnSuccess) {
  flight_rviz_plugins::FlightPanel panel;
  panel.findChild<QLineEdit*>("core_namespace")->setText("/panel_mission_test");panel.findChild<QPushButton*>("connect")->click();
  panel.findChild<QPushButton*>("add")->click();panel.findChild<QPushButton*>("add")->click();
  auto *table=panel.findChild<QTableWidget*>("waypoints");table->item(0,3)->setText("90");table->selectRow(0);
  panel.findChild<QCheckBox*>("check_yaw")->setChecked(false);
  using A=flight_interfaces::action::ExecuteWaypoints;
  auto node=std::make_shared<rclcpp::Node>("panel_mission_mock");
  auto pub=node->create_publisher<flight_interfaces::msg::ControlStatus>("/panel_mission_test/control_status",rclcpp::QoS(1).reliable().transient_local());
  std::shared_ptr<rclcpp_action::ServerGoalHandle<A>> handle;
  auto server=rclcpp_action::create_server<A>(node,"/panel_mission_test/execute_waypoints",
    [](const rclcpp_action::GoalUUID &,std::shared_ptr<const A::Goal> g){EXPECT_EQ(g->waypoints.size(),1u);EXPECT_FALSE(g->check_yaw);EXPECT_FALSE(g->loop);EXPECT_NEAR(g->waypoints[0].yaw_rad,1.57079632679,1e-9);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>>){return rclcpp_action::CancelResponse::REJECT;},
    [&handle](std::shared_ptr<rclcpp_action::ServerGoalHandle<A>> h){handle=h;});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);
  auto spin=[&]{flight_interfaces::msg::ControlStatus s;s.header.stamp=node->now();s.telemetry_valid=true;s.pose_valid=true;s.connected=true;s.armed=true;s.control_active=true;s.holding=true;s.ground_state=2;s.phase="holding";pub->publish(s);ex.spin_some();pump(0.03);};
  auto *step=panel.findChild<QPushButton*>("step");for(int i=0;i<100 && !step->isEnabled();++i)spin();ASSERT_TRUE(step->isEnabled());step->click();
  for(int i=0;i<100 && !handle;++i){spin();}
  ASSERT_NE(handle,nullptr);EXPECT_EQ(table->currentRow(),0);
  auto result=std::make_shared<A::Result>();result->result.outcome_code=0;result->completed_count=1;handle->succeed(result);
  for(int i=0;i<100 && table->currentRow()!=1;++i){spin();}
  EXPECT_EQ(table->currentRow(),1);
  ex.remove_node(node);
}

TEST(Panel, ActualTrajectoryPublishesHidesAndClears) {
  flight_rviz_plugins::FlightPanel panel;
  panel.findChild<QLineEdit*>("core_namespace")->setText("/panel_trail_test");
  panel.findChild<QPushButton*>("connect")->click();
  auto node=std::make_shared<rclcpp::Node>("panel_trail_mock");
  auto qos=rclcpp::QoS(1).reliable().transient_local();
  auto pub=node->create_publisher<flight_interfaces::msg::ControlStatus>("/panel_trail_test/control_status",qos);
  std::vector<geometry_msgs::msg::Point> points;
  auto sub=node->create_subscription<visualization_msgs::msg::MarkerArray>("/panel_trail_test/panel_markers",qos,
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr m){points.clear();for(const auto &v:m->markers)if(v.ns=="actual_trajectory")points=v.points;});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);
  auto sample=[&](double x){
    flight_interfaces::msg::ControlStatus s;s.header.stamp=node->now();s.header.frame_id="map";
    s.connected=true;s.telemetry_valid=true;s.pose_valid=true;s.current_pose.position.x=x;
    s.target_pose.position.x=99;pub->publish(s);pump(0.08);ex.spin_some();
  };
  for(int i=0;i<20;++i)sample(0);
  for(int i=0;i<20 && points.empty();++i)sample(1);
  ASSERT_EQ(points.size(),2u);EXPECT_DOUBLE_EQ(points.back().x,1);
  panel.findChild<QCheckBox*>("show_trajectory")->setChecked(false);
  sample(2);EXPECT_TRUE(points.empty());
  panel.findChild<QCheckBox*>("show_trajectory")->setChecked(true);
  sample(2);ASSERT_EQ(points.size(),4u);EXPECT_DOUBLE_EQ(points.back().x,2);
  panel.findChild<QPushButton*>("clear_trajectory")->click();sample(2);EXPECT_TRUE(points.empty());
  sample(3);ASSERT_EQ(points.size(),2u);EXPECT_DOUBLE_EQ(points.front().x,2);
  panel.findChild<QPushButton*>("connect")->click();sample(3);EXPECT_TRUE(points.empty());
  ex.remove_node(node);
}

TEST(Panel, VehicleModelToggleStalenessAndReconnect) {
  flight_rviz_plugins::FlightPanel panel;
  panel.findChild<QLineEdit*>("core_namespace")->setText("/panel_vehicle_test");
  panel.findChild<QPushButton*>("connect")->click();
  auto node=std::make_shared<rclcpp::Node>("panel_vehicle_mock");
  auto qos=rclcpp::QoS(1).reliable().transient_local();
  auto pub=node->create_publisher<flight_interfaces::msg::ControlStatus>("/panel_vehicle_test/control_status",qos);
  std::vector<visualization_msgs::msg::Marker> model;
  auto sub=node->create_subscription<visualization_msgs::msg::MarkerArray>("/panel_vehicle_test/panel_markers",qos,
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr m){model.clear();for(const auto &v:m->markers)if(v.ns=="vehicle_model")model.push_back(v);});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);
  auto step=[&](bool valid){flight_interfaces::msg::ControlStatus s;s.header.stamp=node->now();s.header.frame_id="map";
    s.connected=s.telemetry_valid=s.pose_valid=valid;s.current_pose.position.z=2;s.current_pose.orientation.w=1;
    pub->publish(s);pump(0.1);ex.spin_some();};
  for(int i=0;i<25 && model.empty();++i)step(true);
  ASSERT_EQ(model.size(),8u);EXPECT_DOUBLE_EQ(model[0].pose.position.z,2);
  panel.findChild<QCheckBox*>("show_vehicle")->setChecked(false);step(true);EXPECT_TRUE(model.empty());
  panel.findChild<QCheckBox*>("show_vehicle")->setChecked(true);step(true);ASSERT_EQ(model.size(),8u);
  pump(1.2);ex.spin_some();ASSERT_EQ(model.size(),8u);
  EXPECT_NE(model.back().text.find("位姿失效"),std::string::npos);
  step(true);ASSERT_EQ(model.size(),8u);EXPECT_EQ(model.back().text,"/panel_vehicle_test");
  step(false);panel.findChild<QPushButton*>("connect")->click();step(false);EXPECT_TRUE(model.empty());
  ex.remove_node(node);
}
