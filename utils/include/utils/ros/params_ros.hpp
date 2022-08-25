#pragma once

//#include <stddef.h>
//#include <cstddef>
// hack to compile ow error for ptrdiff_t does not name a type keeps popping up,
// internet says that including stddef.h or cstddef should work, but it doesn't
// hence we declare it ourselves, should be removed in future for some other
// version of g++, boost, ros or something.
typedef long int ptrdiff_t;
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <utils/ParamGet.h>
#include <utils/ParamInfo.h>
#include <utils/ParamSet.h>

#define PARAM_SERVER "params_server"

#define INFO_STR "info"
#define SET_STR "set"
#define GET_STR "get"

#define INFO_SRV_NAME(node_name) node_name + "/" + PARAM_SERVER + "/" + INFO_STR
#define SET_SRV_NAME(node_name) node_name + "/" + PARAM_SERVER + "/" + SET_STR
#define GET_SRV_NAME(node_name) node_name + "/" + PARAM_SERVER + "/" + GET_STR

//! ROS Service for parameters of the ROS Node.
/*!
 * If the utils library is incorporated in a ros node (executable), the library
 * will add services provided by the node related to param. These services are
 * optional as they need to be started by a function call after calling
 * ros::init(). Corresponding srv's have been also defined in seperate files.
 */
class ParamROS {
public:
  //! function call to provide services by the node.
  /*!
   * While writing a node, one might wish to make the parameters available to
   * other nodes for changing configurations. In that case calling this function
   * provides services /<node name>/params_server/list and
   * /<node name>/params_server/set.
   * \return True on success, else false.
   */
  static ParamROS &instance() {
    static ParamROS p;
    return p;
  }

  void advertise_value(const std::string &str) {
    std_msgs::String msg;
    msg.data = str;
    // For publishing parameters
    //     pub_.publish(msg);
  }

  // Change the parameter publish to true for publishing parameters updated in
  // swahana_master gui
  bool start_ros_service(bool publish = false) {
    try {
      if (publish)
        pub_ = nh.advertise<std_msgs::String>("param_update", 1);
      if (!ros::ok()) {
        std::cerr << "Call ros::init before starting param server" << std::endl;
        return false;
      }
      std::string nh_name = ros::this_node::getName() + "/" + PARAM_SERVER;
      n_ = new ros::NodeHandle(nh_name);
      info_ = n_->advertiseService(INFO_STR, &ParamROS::info_params, this);
      set_ = n_->advertiseService(SET_STR, &ParamROS::set_param, this);
      get_ = n_->advertiseService(GET_STR, &ParamROS::get_param, this);
    } catch (ros::Exception &e) {
      DP$ "line: " << __LINE__ pendl;
      ROS_ERROR("Except: %s", e.what());
      DP$ "file: " << __FILE__ pendl;
      return false;
    }
    return true;
  }

  //! function associated with /<>/params_server/info service.
  bool info_params(utils::ParamInfo::Request &req,
                   utils::ParamInfo::Response &res);

  //! function associated with /<>/params_server/set service.
  bool set_param(utils::ParamSet::Request &req, utils::ParamSet::Response &res);

  //! function associated with /<>/params_server/get service.
  bool get_param(utils::ParamGet::Request &req, utils::ParamGet::Response &res);

private:
  /* data */
  ros::NodeHandle *n_;
  ros::ServiceServer info_, set_, get_;
  ros::NodeHandle nh;
  ros::Publisher pub_;

  ParamROS() {}
};
