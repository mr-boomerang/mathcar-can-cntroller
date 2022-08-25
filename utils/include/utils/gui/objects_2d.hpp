#pragma once

#include <mutex>

#include <utils/check_ret.h>
#include <utils/color.hpp>
#include <utils/error_code.h>
#include <utils/gui/common.h>
#include <utils/gui/object_xd.hpp>

#include <utils/gui/matrix_stream_object.hpp>

#include <pangolin/pangolin.h>

#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#ifdef BUILT_WITH_ROS
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
//#include <sensor_msgs/NavSatFix.h>
#endif

// Load an image and show it in a 2D view.
class ImageObject : public Objectxd {
public:
  ImageObject(unsigned char *img, int w, int h, GLint format)
      : Objectxd(TwoDObject) {
    reinit(img, w, h, format);
  }
  int reinit(unsigned char *img, int w, int h, GLint format);
#ifdef UTILS_USE_OPENCV

  ImageObject(const std::string &img_path, int cv_imread_flag = -1)
      : Objectxd(TwoDObject) {
    reinit(cv::imread(img_path, cv_imread_flag));
  }

  ImageObject(const cv::Mat &img) : Objectxd(TwoDObject) { reinit(img); }

  int reinit(const cv::Mat &img);
#endif

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

private:
  /* data */
  pangolin::GlTexture img_tex_;
};

class ImageStreamObject : public Objectxd {
public:
  ImageStreamObject(int w, int h) : Objectxd(TwoDObject) { reinit(w, h); }

  int reinit(int w, int h);
  int update_img(const unsigned char *img, int w, int h);

  int draw_obj(pangolin::View & /*v*/); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

#ifdef UTILS_USE_OPENCV

  ImageStreamObject(const std::string &vid_path);

  ImageStreamObject(const cv::VideoCapture &vid) : Objectxd(TwoDObject) {
    reinit(vid);
  }

  int reinit(const cv::VideoCapture &vid);
  int update_img(const cv::Mat &img);
#endif

#ifdef BUILT_WITH_ROS
  ImageStreamObject(const std::string &topic_sub, ros::NodeHandle &n,
                    uint32_t queue_sz = 5)
      : Objectxd(TwoDObject) {
    reinit(topic_sub, n, queue_sz);
  }

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5);
  void callback(const sensor_msgs::ImageConstPtr &img_sub);
#endif

private:
  /* data */
  pangolin::GlTexture img_tex_;
  std::mutex mtx_;
  GLenum format_;

#ifdef UTILS_USE_OPENCV
  cv::VideoCapture vc_;
#endif

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
/* callback function for the subsriber */
#endif
};
typedef ImageStreamObject VideoObject;

class TextObject : public Objectxd {
public:
  ros::Subscriber sub_;

#if 0
  struct str_color_stat {
    int str_start_ind;
    int str_end_ind;
    int text_prop;
    int fg_color;
    int bg_color;

    str_color_stat() {
      str_start_ind = 0;
      str_end_ind = 0;

      text_prop = 0;
      fg_color = 0;
      bg_color = 0;
    }
  };

  std::vector<str_color_stat> color_stats;

  struct str_stat {
    int start_ind;
    int end_ind;
    int n_chars;
  };
  std::vector<str_stat> str_line_stats;
#endif

  TextObject(const utils::Color &font_col = utils::Color::white,
             const utils::Color &bg_col = utils::Color::black,
             const std::string &text = "")
      : Objectxd(TwoDObject), fnt_(pangolin::GlFont::I()) {

    PI$ ct_vred(&fc_) pcommas ct_vred(&bgc_) pendl;
    reinit(font_col, bg_col, text);
  }

  int reinit(const utils::Color &fc = utils::Color::white,
             const utils::Color &bgc = utils::Color::black,
             const std::string &text = "");

  int update(const std::string &text, bool append = true);
  int do_str_stats();
  // std::string removeColorEsc(const std::string& str);

  int draw_obj(pangolin::View &v); // opaque by default

  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  int set_borders(const int &left = 5, const int &bottom = 5) {
    left_border_ = left;
    bottom_border_ = bottom;
    return RET_SUCCESS_INT;
  }

  int set_line_spacing(const int &gap_in_pixels = 2) {
    line_spacing_ = gap_in_pixels;
    return RET_SUCCESS_INT;
  }

  int set_fg_color(const utils::Color &c = utils::Color::white) {
    fc_ = c;
    return RET_SUCCESS_INT;
  }

  int set_bg_color(const utils::Color &c = utils::Color::black) {
    bgc_ = c;
    return RET_SUCCESS_INT;
  }

#ifdef BUILT_WITH_ROS
  TextObject(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5)
      : Objectxd(TwoDObject), fnt_(pangolin::GlFont::I()) {
    reinit(topic_sub, n, queue_sz);
  }

  int reinit(const std::string &topic_sub, ros::NodeHandle &n,
             uint32_t queue_sz = 5);

  void callback(const std_msgs::String &sub_msg);
// void callback(const sensor_msgs::ImageConstPtr &img_sub);
#endif

  int draw_text(pangolin::View &v);

  int increment_start_idx() {
    if (start_idx_ == ((int)draw_text_.size() - 1))
      return RET_SUCCESS_INT;
    start_idx_++;
    return RET_SUCCESS_INT;
  }
  int decrement_start_idx() {

    if (start_idx_ <= 0) {
      start_idx_ = 0;
      return RET_SUCCESS_INT;
    }

    start_idx_--;
    return RET_SUCCESS_INT;
  }

private:
  /* data */
  std::vector<std::string> text_;
  std::vector<pangolin::GlText> draw_text_;

  pangolin::GlFont &fnt_;

  int view_w_, view_h_; // view width and height
  int text_top_limit_;  // this will help avoid margins draw by ViewNode
  float char_w_, char_h_, total_text_h_; // width and height in font.
  int left_border_, bottom_border_, line_spacing_;
  int start_idx_;
  int end_idx_;

  utils::Color fc_;  // font color
  utils::Color bgc_; // back ground color
};

/* Handler class */
class TextScrollHandler : public pangolin::Handler {
public:
  TextScrollHandler(TextObject *tobj) : tobj_(tobj) {}
  void Mouse(pangolin::View &, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state);

private:
  TextObject *tobj_;
};
