#pragma once
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>

namespace flight_rviz_plugins {
// Visualization only: body FLU (+X forward, +Y left, +Z up) in map/ENU.
class QuadrotorModel {
public:
  void clear() {seen_=live_=false;}
  void update(geometry_msgs::msg::Pose p,bool valid) {
    const auto &v=p.position;auto &q=p.orientation;
    const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    live_=valid && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
      std::isfinite(n) && std::abs(n-1.0)<=0.02;
    if(!live_)return;
    const double length=std::sqrt(n);q.x/=length;q.y/=length;q.z/=length;q.w/=length;
    pose_=p;seen_=true;
  }
  visualization_msgs::msg::MarkerArray markers(const std_msgs::msg::Header &header,
      const std::string &name,bool fresh) const {
    visualization_msgs::msg::MarkerArray out;if(!seen_)return out;
    const bool live=live_ && fresh;
    using Marker=visualization_msgs::msg::Marker;
    auto add=[&](int type,double x,double y,double z,double r,double g,double b)->Marker& {
      Marker m;m.header=header;m.ns="vehicle_model";m.id=static_cast<int>(out.markers.size());
      m.type=type;m.action=Marker::ADD;m.pose=pose_;
      // Rotate body-local offsets by the complete measured attitude.
      const auto &q=pose_.orientation;
      const double tx=2*(q.y*z-q.z*y),ty=2*(q.z*x-q.x*z),tz=2*(q.x*y-q.y*x);
      m.pose.position.x+=q.w*tx+q.y*tz-q.z*ty+x;
      m.pose.position.y+=q.w*ty+q.z*tx-q.x*tz+y;
      m.pose.position.z+=q.w*tz+q.x*ty-q.y*tx+z;
      m.color.r=live?r:0.55;m.color.g=live?g:0.55;m.color.b=live?b:0.55;m.color.a=live?1.0:0.65;
      out.markers.push_back(m);return out.markers.back();
    };
    auto &body=add(Marker::CUBE,0,0,0,0.85,0.88,0.94);
    body.scale.x=0.24;body.scale.y=0.16;body.scale.z=0.08;
    auto &arms=add(Marker::LINE_LIST,0,0,0,0.35,0.45,0.6);arms.scale.x=0.035;
    for(double y:{-0.22,0.22}) {
      geometry_msgs::msg::Point a,b;a.x=-0.22;a.y=y;b.x=0.22;b.y=-y;
      arms.points.push_back(a);arms.points.push_back(b);
    }
    for(double x:{-0.22,0.22})for(double y:{-0.22,0.22}) {
      auto &rotor=add(Marker::CYLINDER,x,y,0.035,0.2,0.65,0.85);
      rotor.scale.x=rotor.scale.y=0.20;rotor.scale.z=0.012;
    }
    auto &nose=add(Marker::ARROW,0,0,0.07,1.0,0.15,0.12);
    nose.scale.x=0.42;nose.scale.y=0.035;nose.scale.z=0.065;
    auto &label=add(Marker::TEXT_VIEW_FACING,0,0,0,0.95,0.95,0.95);
    label.pose.position=pose_.position;label.pose.position.z+=0.5;
    label.pose.orientation.x=label.pose.orientation.y=label.pose.orientation.z=0;label.pose.orientation.w=1;
    label.scale.z=0.15;label.text=name+(live?"":" [位姿失效 / 最后位置]");
    return out;
  }
private:
  geometry_msgs::msg::Pose pose_;
  bool seen_{false},live_{false};
};
}
