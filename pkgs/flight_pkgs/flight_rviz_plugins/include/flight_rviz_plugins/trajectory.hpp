#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <deque>
#include <utility>
#include <vector>

namespace flight_rviz_plugins {
// Bounded actual-position history; gaps are never joined into fictitious motion.
class Trajectory {
public:
  void clear() {segments_.clear();break_segment();last_stamp_=0;}
  void break_segment() {anchored_=false;}
  void sample(const geometry_msgs::msg::Point &p,double stamp,bool valid) {
    if(!valid || !std::isfinite(stamp) || stamp<=0 ||
       !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      break_segment();return;
    }
    if(stamp<=last_stamp_) {break_segment();return;}
    if(stamp-last_stamp_>1.0)break_segment();
    last_stamp_=stamp;
    if(!anchored_) {anchor_=p;anchored_=true;return;}
    const double distance=std::hypot(std::hypot(p.x-anchor_.x,p.y-anchor_.y),p.z-anchor_.z);
    if(distance<0.05)return;
    segments_.emplace_back(anchor_,p);anchor_=p;
    if(segments_.size()>2000)segments_.pop_front();
  }
  std::vector<geometry_msgs::msg::Point> points() const {
    std::vector<geometry_msgs::msg::Point> out;out.reserve(segments_.size()*2);
    for(const auto &s:segments_){out.push_back(s.first);out.push_back(s.second);}
    return out;
  }
private:
  bool anchored_{false};
  double last_stamp_{0};
  geometry_msgs::msg::Point anchor_;
  std::deque<std::pair<geometry_msgs::msg::Point,geometry_msgs::msg::Point>> segments_;
};
}
