#pragma once

#include <queue>

#include <utils/gui/object_xd.hpp>

#include <utils/gui/textView.hpp>

//#include <utils/gui/new/gui.hpp>

#include <utils/check_ret.h>
#include <utils/color.hpp>
#include <utils/error_code.h>

#ifdef BUILT_WITH_ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#endif

#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

#include <ctime>
#include <sstream>
#include <utils/fs_utils.h>
#include <utils/mode_printing.h>
#define VBO_ELEMS 3

class Object3d : public Objectxd {
public:
  Object3d() : Objectxd(ThreeDObject) { cam_ = NULL; }
  // virtual ~Object3d ();

  int set_cam(pangolin::OpenGlRenderState *cam) {
    cam_ = cam;
    return RET_SUCCESS_INT;
  }
  pangolin::OpenGlRenderState *get_cam() { return cam_; }

protected:
  /* data */
  pangolin::OpenGlRenderState *cam_;
};

class AxisObject : public Object3d {
public:
  AxisObject(float s, const Eigen::Matrix4f &p = Eigen::Matrix4f::Identity()) {
    reinit(s, p);
  }
  int reinit(float s, const Eigen::Matrix4f &p = Eigen::Matrix4f::Identity());

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }
  int set_line_width(const float width) {
    line_width_ = width;
    return RET_SUCCESS_INT;
  }

private:
  /* data */
  float scale_;
  Eigen::Matrix4f pose_;
  float line_width_;
};

class ZeroGridObject : public Object3d {
public:
  enum Alignment { XGrid, YGrid, ZGrid };
  ZeroGridObject(Alignment a = ZGrid, float scale = 1.0f, int num_grid = 10)
      : col_(utils::Color::grey) {
    reinit(a, scale, num_grid);
  }
  int reinit(Alignment a = ZGrid, float scale = 1.0f, int num_grid = 10);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  int set_color(const utils::Color &c) {
    col_ = c;
    return RET_SUCCESS_INT;
  }
  int set_line_width(const float width) {
    line_width_ = width;
    return RET_SUCCESS_INT;
  }

private:
  /* data */
  Alignment a_;
  float scale_;
  int n_grid_;
  float line_width_;
  utils::Color col_;
};

class TrajectoryObject : public Object3d {
public:
  TrajectoryObject(size_t keep = 50, bool follow = true)
      : traj_size_(2), traj_pts_col_(utils::Color::red),
        traj_line_col_(utils::Color::hblue) {
    reinit(keep, follow);
  }

  int reinit(size_t keep = 50, bool follow = true);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  int follow();
  int unfollow();

  int set_gl_cam_relative(float rel_x, float rel_y, float rel_z,
                          pangolin::AxisDirection up = pangolin::AxisNegY);

  int set_pose(const pangolin::OpenGlMatrix &pose);
  int set_pose(double x, double y, double z);

#ifdef UTILS_USE_OPENCV
  int set_pose(const cv::Mat &rot, const cv::Mat &trans);
#endif

  int set_traj_color(const utils::Color &c_p, const utils::Color &c_l) {
    traj_pts_col_ = c_p;
    traj_line_col_ = c_l;
    return RET_SUCCESS_INT;
  }

  int set_traj_size(int size) {
    traj_size_ = size;
    return RET_SUCCESS_INT;
  }
  int set_axis_size(float size) {
    axis_size_ = size;
    return RET_SUCCESS_INT;
  }

  int set_target_frame(std::string target_frame) {
    target_frame_ = target_frame;
    return RET_SUCCESS_INT;
  }
#ifdef BUILT_WITH_ROS
  // added 181016:1128
  TrajectoryObject(const std::string &topic_sub, ros::NodeHandle &n,
                   uint32_t queue_sz = 5, size_t keep = 50, bool follow = true)
      : traj_size_(2), traj_pts_col_(utils::Color::red),
        traj_line_col_(utils::Color::hblue) {
    reinit(topic_sub, n, queue_sz, keep, follow);
  }

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5, size_t keep = 50, bool follow = true);
#endif
private:
  /* data */
  size_t keep_;      // num of pose to show in trajectories. 0 is show all.
  size_t traj_size_; // size of points and line for trajectory.
  float axis_size_;  // size of points and line for trajectory.
  bool follow_, followed_;
  std::vector<float> traj_;
  pangolin::OpenGlMatrix pose_;
  pangolin::OpenGlRenderState cam__;
  utils::Color traj_pts_col_, traj_line_col_;

  // target_frame to transform odometery to. If empty, doesnot transform.
  std::string target_frame_;
  pangolin::OpenGlMatrix ref_frame_;

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
  /* callback function for the subsriber */
  void callback(const nav_msgs::Odometry::ConstPtr &odom);
#endif
};

class PTSObject : public Object3d {
public:
  PTSObject(const std::string &file,
            utils::Color default_col = utils::Color::green, double scale = 1.0,
            int skip = 0) {
    reinit(file, default_col, scale, skip);
  }
  int reinit(const std::string &file,
             utils::Color default_col = utils::Color::green, double scale = 1.0,
             int skip = 0);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  static int read_PTS_file(const std::string &file, double *&points,
                           unsigned char *&col,
                           utils::Color default_col = utils::Color::green,
                           double scale = 1.0, int skip = 0);

private:
  /* data */
  pangolin::GlBuffer ptsb_;
  pangolin::GlBuffer colb_;
  double *pts_;
  unsigned char *col_;
};

class PointCloudObject : public Object3d {
public:
  PointCloudObject(size_t num_of_frames = 1,
                   utils::Color col = utils::Color::green)
      : ptsb_(NULL), colb_(NULL), col_(col), pts_(NULL), c_pts_(NULL),
        pc_count_(0) {
    reinit(num_of_frames);
  }
  int reinit(size_t num_of_frames, utils::Color col = utils::Color::green);
  int resize(size_t num_frames);

  int update(size_t num_pts, double *pts, unsigned char *col = NULL);
  // added 310816:1503 - function overloading for float type
  int update(size_t num_pts, float *pts, unsigned char *col = NULL);
  int erase(int idx);
  int erase_last();

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  int save_pointCloud(const std::string &file_name = NULL);
  ObjectType type() { return type_; }

  inline int next_idx();
  inline int prev_idx();

#ifdef BUILT_WITH_ROS
  PointCloudObject(const std::string &topic_sub, ros::NodeHandle &n,
                   uint32_t queue_sz = 5, size_t num_of_frames = 1,
                   utils::Color col = utils::Color::green)
      : ptsb_(NULL), colb_(NULL), col_(col), pts_(NULL), c_pts_(NULL),
        pc_count_(0) {
    reinit(topic_sub, n, queue_sz, num_of_frames, col);
  }

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5, size_t num_of_frames = 1,
             utils::Color col = utils::Color::green);
  void callback(const sensor_msgs::PointCloud2 &pc_sub);
#endif

private:
  /* data */
  int curr_idx_; // so that we know when it goes below zero.
  size_t num_frames_;
  pangolin::GlBuffer *ptsb_;
  pangolin::GlBuffer *colb_;
  bool *show_;
  bool *cb_valid_; // color buffer valid

  utils::Color col_;
  double *pts_;
  unsigned char *c_pts_;
  int pc_count_;

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
  /* helper function for the subscriber callback */
  inline int get_pt_using_field_datatype(const unsigned char *pc_elem,
                                         const uint8_t &datatype, double *pts) {

    if (datatype == sensor_msgs::PointField::FLOAT32) {
      float *f_pc_elem = (float *)pc_elem;
      pts[0] = (double)f_pc_elem[0];
    } else if (datatype == sensor_msgs::PointField::FLOAT64) {
      double *d_pc_elem = (double *)pc_elem;
      pts[0] = d_pc_elem[0];
    } else {
      return 1;
    }

    return 0;
  }
#endif

  /* functions */
  int reinit_buf(int idx, size_t sz, bool color);
  /* Destructor */
  ~PointCloudObject() {
    if (NULL != pts_) {
      delete[] pts_;
      pts_ = NULL;
    }
    if (NULL != c_pts_) {
      delete[] c_pts_;
      c_pts_ = NULL;
    }
  }
};

// added 191016
class VisualizationMarkerArrayObject : public Object3d {
public:
  VisualizationMarkerArrayObject(uint32_t keep = 2) { reinit(keep); }
  int reinit(uint32_t keep = 2);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  int set_target_frame(std::string target_frame) {
    target_frame_ = target_frame;
    return RET_SUCCESS_INT;
  }
#ifdef BUILT_WITH_ROS
  int update(const visualization_msgs::MarkerArray &viz_marker_array);
  VisualizationMarkerArrayObject(const std::string &topic_sub,
                                 ros::NodeHandle &n, uint32_t queue_sz = 5,
                                 uint32_t keep = 2) {
    reinit(topic_sub, n, queue_sz, keep);
  }
  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5, uint32_t keep = 2);

#endif
private:
  uint32_t n_keep;
  uint32_t cur_id;
  float offset_x;
  float offset_y;
  float offset_z;
  pangolin::OpenGlMatrix ref_frame_;
  std::string target_frame_;
#ifdef BUILT_WITH_ROS
  std::vector<visualization_msgs::MarkerArray> marker_array_vec;
  ros::Subscriber sub_;
  /* callback function for the subsriber */
  void callback(const visualization_msgs::MarkerArray &viz_marker_array);
  uint32_t get_next_id() {
    cur_id = (cur_id + 1) % n_keep;
    return (cur_id);
  }

#endif
};

class GPSObject : public TrajectoryObject {
public:
  GPSObject(size_t keep = 50, bool follow = true)
      : traj_size_(2), traj_pts_col_(utils::Color::red),
        traj_line_col_(utils::Color::hblue) {
    reinit(keep, follow);
  }
  VisualizationMarkerArrayObject VMAObject;
  int reinit(size_t keep = 50, bool follow = true);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  //  void follow();
  //  void unfollow();

  int set_gl_cam_relative(float rel_x, float rel_y, float rel_z,
                          pangolin::AxisDirection up = pangolin::AxisNegY);

  int set_pose(double x, double y, double z);

// #ifdef UTILS_USE_OPENCV
//  void set_pose(const cv::Mat &rot, const cv::Mat &trans);
// #endif

/*
  void set_traj_color(const utils::Color &c_p, const utils::Color &c_l) {
    traj_pts_col_ = c_p;
    traj_line_col_ = c_l;
  }

  void set_traj_size(int size) { traj_size_ = size; }
  void set_axis_size(float size) { axis_size_ = size; }
*/
#ifdef BUILT_WITH_ROS
  // added 181016:1128
  GPSObject(const std::string &topic_sub, ros::NodeHandle &n,
            uint32_t queue_sz = 5, size_t keep = 50, bool follow = true)
      : traj_size_(2), traj_pts_col_(utils::Color::red),
        traj_line_col_(utils::Color::hblue) {
    reinit(topic_sub, n, queue_sz, keep, follow);
  }

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5, size_t keep = 50, bool follow = true);
#endif
private:
  /* data */
  size_t keep_;      // num of pose to show in trajectories. 0 is show all.
  size_t traj_size_; // size of points and line for trajectory.
  float axis_size_;  // size of points and line for trajectory.
  bool follow_, followed_;
  std::vector<float> traj_;
  pangolin::OpenGlMatrix pose_;
  pangolin::OpenGlRenderState cam__;
  utils::Color traj_pts_col_, traj_line_col_;

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
  /* callback function for the subsriber */
  void callback(const sensor_msgs::NavSatFixConstPtr &fix);

#endif
};

class OSMObject : public Object3d {

private:
  int msg_flag;

public:
  TrajectoryObject traj_obj;

  double northing, easting, min_northing, min_easting, max_northing,
      max_easting, first_gps_x, first_gps_y;
  const double RADIANS_PER_DEGREE = M_PI / 180.0;

  // WGS84 Parameters
  const double WGS84_A = 6378137.0;    // major axis
  const double WGS84_E = 0.0818191908; // first eccentricity

  // UTM Parameters
  const double UTM_K0 = 0.9996;              // scale factor
  const double UTM_E2 = (WGS84_E * WGS84_E); // e^2
  const double UTM_E4 = (UTM_E2 * UTM_E2);   // e^4

  typedef struct {
    unsigned int final_way_id;
    std::vector<double> utm_x;
    std::vector<double> utm_y;
    std::vector<unsigned int> final_node_id;
    std::string final_way_tag;
    std::string final_way_tag_value;
  } final_way;
  std::vector<final_way> final_way_vect;

  char prev[100];
  int flag = 0;

  OSMObject()

  {}
  int reinit(char *sample1);
  int load_osm(char *sample);

#ifdef BUILT_WITH_ROS

  OSMObject(const std::string &topic_sub, ros::NodeHandle &n,
            uint32_t queue_sz = 5)

  {

    std::string msg_type =
        "rostopic info " + topic_sub + " | grep Type | grep -q NavSatFix";

    /*
        if(topic_sub == topic)
             {
               reinit_fix(topic_sub, n, queue_sz );
             }

        else {
              reinit(topic_sub, n, queue_sz);
             }
    */

    msg_flag = system(msg_type.c_str());
    if (!msg_flag) {
      reinit_fix(topic_sub, n, queue_sz);
    }
    /* else
       {
            reinit(topic_sub, n, queue_sz);
       }
      */
  }

  int reinit_fix(const std::string &topic_sub, ros::NodeHandle &n,
                 uint32_t queue_sz = 5);

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5);

  ros::Subscriber sub_;
  /* callback function for the subsriber */
  void fix_callback(const sensor_msgs::NavSatFix &fix_msg);

#endif

  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  // char UTMLetterDesignator(double Lat);   // to compute the zone of the
  // latitude.

  int LLtoUTM(const double Lat, const double Long, double &UTMNorthing,
              double &UTMEasting, std::string &UTMZone);

  int LLtoUTM(const double Lat, const double Long, double &UTMNorthing,
              double &UTMEasting, char *UTMZone);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
};
