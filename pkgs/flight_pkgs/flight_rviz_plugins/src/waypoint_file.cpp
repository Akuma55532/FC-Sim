#include "flight_rviz_plugins/waypoint_file.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
namespace flight_rviz_plugins {
void validate(const Vehicle & v) {
  if(v.points.size()>500) throw std::invalid_argument("最多 500 个航点");
  for(const auto & p:v.points) for(double x:{p.x,p.y,p.z,p.yaw_deg})
    if(!std::isfinite(x)) throw std::invalid_argument("航点必须为有限数值");
  const auto & o=v.options;
  for(double x:{o.position,o.yaw_deg,o.dwell,o.timeout})
    if(!std::isfinite(x)) throw std::invalid_argument("任务选项必须为有限数值");
  if(o.position<=0 || o.position>10 || o.yaw_deg<=0 || o.yaw_deg>180 || o.dwell<0 ||
     o.timeout<=0 || o.dwell>o.timeout) throw std::invalid_argument("到点/停留/超时选项无效");
}
Document decode(const std::string & text) {
  auto root=YAML::Load(text);
  if(root["schema_version"].as<int>()!=1 || root["frame_id"].as<std::string>()!="map")
    throw std::invalid_argument("文件版本或坐标系不支持（需要版本 1 / map）");
  if(!root["vehicles"].IsSequence() || root["vehicles"].size()>100)
    throw std::invalid_argument("vehicles 必须是最多 100 项的列表");
  Document result;
  for(const auto & entry:root["vehicles"]) {
    const auto ns=entry["namespace"].as<std::string>();
    if(ns.empty() || ns.front()!='/' || result.count(ns)) throw std::invalid_argument("车辆命名空间为空或重复");
    Vehicle v;auto options=entry["options"];
    v.options={options["position_tolerance_m"].as<double>(),options["yaw_tolerance_deg"].as<double>(),
      options["dwell_sec"].as<double>(),options["waypoint_timeout_sec"].as<double>(),
      options["check_yaw"].as<bool>(),options["loop"].as<bool>()};
    auto points=entry["waypoints"];
    if(!points.IsSequence() || points.size()>500) throw std::invalid_argument("航点列表无效或超过 500 点");
    for(const auto & p:points) v.points.push_back({p["x"].as<double>(),p["y"].as<double>(),p["z"].as<double>(),p["yaw_deg"].as<double>()});
    validate(v);result.emplace(ns,v);
  }
  return result;
}
std::string encode(const Document & d) {
  if(d.size()>100) throw std::invalid_argument("车辆条目过多");
  YAML::Node root;root["schema_version"]=1;root["frame_id"]="map";
  root["vehicles"]=YAML::Node(YAML::NodeType::Sequence);
  for(const auto & pair:d) {
    if(pair.first.empty() || pair.first.front()!='/') throw std::invalid_argument("车辆命名空间无效");
    validate(pair.second);YAML::Node v;v["namespace"]=pair.first;
    const auto & o=pair.second.options;auto opt=v["options"];
    opt["position_tolerance_m"]=o.position;opt["yaw_tolerance_deg"]=o.yaw_deg;
    opt["dwell_sec"]=o.dwell;opt["waypoint_timeout_sec"]=o.timeout;
    opt["check_yaw"]=o.check_yaw;opt["loop"]=o.loop;
    v["waypoints"]=YAML::Node(YAML::NodeType::Sequence);
    for(const auto & p:pair.second.points) {YAML::Node w;w["x"]=p.x;w["y"]=p.y;w["z"]=p.z;w["yaw_deg"]=p.yaw_deg;v["waypoints"].push_back(w);}
    root["vehicles"].push_back(v);
  }
  YAML::Emitter out;out.SetDoublePrecision(17);out<<root;
  if(!out.good()) throw std::runtime_error("YAML 编码失败");
  return out.c_str();
}
}
