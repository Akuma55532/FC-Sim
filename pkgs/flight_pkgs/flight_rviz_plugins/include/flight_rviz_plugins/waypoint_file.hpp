#pragma once
#include <map>
#include <string>
#include <vector>
namespace flight_rviz_plugins {
struct Point {double x{}, y{}, z{1}, yaw_deg{};};
struct Options {double position{0.1}, yaw_deg{10}, dwell{0.5}, timeout{60};bool check_yaw{true}, loop{false};};
struct Vehicle {std::vector<Point> points; Options options;};
using Document=std::map<std::string,Vehicle>;
void validate(const Vehicle &);
Document decode(const std::string & yaml);
std::string encode(const Document &);
}
