#pragma once
#include <cmath>

namespace flight_core {
struct Pose {
  double x{}, y{}, z{}, qx{}, qy{}, qz{}, qw{1.0};
};
inline bool valid_pose(const Pose & p) {
  const double values[]{p.x,p.y,p.z,p.qx,p.qy,p.qz,p.qw};
  for (double v : values) {if (!std::isfinite(v)) {return false;}}
  const double norm = p.qx*p.qx+p.qy*p.qy+p.qz*p.qz+p.qw*p.qw;
  return std::abs(norm-1.0) <= 0.02;
}
inline double distance(const Pose & a, const Pose & b) {
  return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
inline Pose yaw_only(Pose p) {
  const double n=std::sqrt(p.qx*p.qx+p.qy*p.qy+p.qz*p.qz+p.qw*p.qw);
  p.qx/=n; p.qy/=n; p.qz/=n; p.qw/=n;
  const double yaw = std::atan2(2*(p.qw*p.qz+p.qx*p.qy),
    1-2*(p.qy*p.qy+p.qz*p.qz));
  p.qx=0; p.qy=0; p.qz=std::sin(yaw/2); p.qw=std::cos(yaw/2);
  return p;
}
}  // namespace flight_core
