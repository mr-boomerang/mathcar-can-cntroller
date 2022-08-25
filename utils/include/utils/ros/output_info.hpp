#pragma once

#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/this_node.h>

#include <utils/params.hpp>

#include <utils/OutputInfo.h>

#include <utils/mode_printing.h>

DEFINE_PARAMS_ENUM(OutputType, Image, Disparity, PointCloud, Trajectory, Stream,
                   VisualizationMarkerArray, GPSObject, DataPlotter, OSM, Text);

#define OUT_INFO_DELIM ";"

#define OUT_INFO_SRV_NAME "output_info"
#define OUT_INFO_SRV(node_name) node_name + "/" + "output_info"

class output_info {
public:
  static output_info &instance() {
    static output_info oi;
    return oi;
  }

  void add(const std::string &name, const PENS::OutputType &type) {
    PRINT_FUNC_ENTER;
    std::string topic = nh.resolveName(name);
    if (topics_.empty()) {
      topics_ += topic;
      types_ += type.to_string();
    } else {
      topics_ += OUT_INFO_DELIM + topic;
      types_ += OUT_INFO_DELIM + type.to_string();
    }
    PI$ ct_vdblu(topics_) pcommas ct_vdblu(types_) pendl;
    PRINT_FUNC_EXIT;
  }

  bool attend_service_call(utils::OutputInfo::Request & /*req*/,
                           utils::OutputInfo::Response &res) {
    PRINT_FUNC_ENTER;
    res.topics = topics_;
    res.types = types_;
    PRINT_FUNC_EXIT;
    return true;
  }

private:
  output_info() { reinit(); }

  void reinit() {
    PRINT_FUNC_ENTER;
    topics_ = "";
    types_ = "";
    std::string nh_name = ros::this_node::getName();
    n_ = new ros::NodeHandle(nh_name);
    info_ = n_->advertiseService(OUT_INFO_SRV_NAME,
                                 &output_info::attend_service_call, this);
    PRINT_FUNC_EXIT;
  }

  /* data */
  std::string topics_;
  std::string types_;
  ros::NodeHandle *n_;
  ros::ServiceServer info_;
  ros::NodeHandle nh;
};
