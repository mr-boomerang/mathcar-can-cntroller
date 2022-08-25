#pragma once

#include <utils/check_ret.h>
#include <utils/error_code.h>
#include <utils/gui/object_xd.hpp>
//#include <utils/color.hpp>

//#include <utils/gui/matrix_stream_object.hpp>

#include <pangolin/pangolin.h>

#include <utils/str_utils.h>

#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#ifdef BUILT_WITH_ROS
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#endif

class DataPlotterObject : public Objectxd {
public:
  DataPlotterObject(const std::vector<std::string> &labels)
      : Objectxd(TwoDObject) {
    PRINT_FUNC_ENTER;
    reinit(labels);
    PRINT_FUNC_EXIT;
  }
  int reinit(const std::vector<std::string> &labels);
  int update(const std::vector<float> &data);

#ifdef BUILT_WITH_ROS
  DataPlotterObject(const std::string &topic_sub, ros::NodeHandle &n,
                    uint32_t queue_sz = 5)
      : Objectxd(TwoDObject) {
    reinit(topic_sub, n, queue_sz);
  }
  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5);
  void callback(const std_msgs::Float32MultiArray &data_sub);
#endif

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    PRINT_FUNC_ENTER;
    transparency_ = t;
    PRINT_FUNC_EXIT;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

private:
  bool notAdded;
  std::vector<std::string> labels_;
  pangolin::DataLog log;
  pangolin::Plotter *plotter; //(&log);
  int sz_;

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
#endif
};
