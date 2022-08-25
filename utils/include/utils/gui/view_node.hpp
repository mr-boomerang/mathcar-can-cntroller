#pragma once

#include <iostream>
#include <memory>
#include <pangolin/image/pixel_format.h>
#include <pangolin/pangolin.h>
#include <utils/TicToc.h>
#include <utils/check_ret.h>
#include <utils/error_code.h>
#include <utils/gui/object_xd.hpp>
#include <utils/gui/objects_2d.hpp>
#include <utils/str_utils.h>
using namespace utils;

class ViewNode {
public:
  // ViewNode ();
  ViewNode(const std::string &name, ViewType type, ViewNode *parent);
  int init_viewnode(const std::string &name, ViewType type, ViewNode *parent);

  int bounds(pangolin::Attach bot, pangolin::Attach top, pangolin::Attach left,
             pangolin::Attach right);
  int layout(pangolin::Layout layout);

  int add_object(const std::string &name, Objectxd *obj);
  int remove_object(const std::string &name);
  Objectxd *get_object(const std::string &name);

  int draw_margins(pangolin::View &v);
  int draw(pangolin::View &v);

  int set_parent(ViewNode *new_parent);
  ViewNode *get_parent();
  std::string name() { return name_; }
  std::string parent_name() {
    if (parent_ == NULL)
      return "";
    return parent_->name();
  }

  ViewType type() { return type_; }

  pangolin::View &view() { return v_; }

  bool point_in_head(int x, int y) { return vp_head_wc_.Contains(x, y); }

  bool point_in_close(int x, int y) { return vp_close_wc_.Contains(x, y); }

  bool point_in_record(int x, int y) { return vp_record_wc_.Contains(x, y); }

  int set_viewcam_projection(pangolin::OpenGlMatrix m) {
    cam_.SetProjectionMatrix(m);
    return RET_SUCCESS_INT;
  }

  int set_viewcam_pose(pangolin::OpenGlMatrix m) {
    cam_.SetModelViewMatrix(m);
    return RET_SUCCESS_INT;
  }

  int set_3d_handler() {
    if (ThreeD == type_)
      v_.SetHandler(new pangolin::Handler3D(cam_));
    return RET_SUCCESS_INT;
  }

  int set_close_cb(const std::function<void(ViewNode *)> &cb) {
    close_cb_ = cb;
    return RET_SUCCESS_INT;
  }

  int call_close_cb() {
    if (close_cb_)
      close_cb_(this);
    return RET_SUCCESS_INT;
  }
  int toggleRecord() {
    const int min_w = 640;
    if (name_.at(0) == '/')
      name_.erase(name_.begin());
    if (recorder_.IsOpen()) {
      PN$ "Stopped recording view " << ct_pur(rec_name_[rec_name_.size() - 1])
                                    << " at " << rec_sz_ pendl;
      recorder_.Close();
    } else {
      split_str(name_, '/', &rec_name_);
      std::string video_file_ = "ffmpeg:[fps=15]//" +
                                rec_name_[rec_name_.size() - 1] + "_" +
                                date_time_now() + ".avi";
      recorder_.Open(video_file_);
      const pangolin::PixelFormat fmt =
          pangolin::PixelFormatFromString("RGB24");
      rec_sz_ = cv::Size(v_.vp.w, v_.vp.h);
      rec_resize_ = false;
      if (rec_sz_.width < min_w) {
        rec_sz_.height = min_w / (rec_sz_.width / rec_sz_.height);
        rec_sz_.width = min_w;
        rec_resize_ = true;
      }
      rec_buffer_ = cv::Mat::zeros(v_.vp.h, v_.vp.w, CV_8UC3);
      streams_.push_back(pangolin::StreamInfo(
          fmt, rec_sz_.width, rec_sz_.height, rec_sz_.width * fmt.bpp));
      recorder_.SetStreams(streams_);
      PN$ "Started recording view " << ct_pur(rec_name_[rec_name_.size() - 1])
                                    << " at " << rec_sz_ pendl;
    }
    return RET_SUCCESS_INT;
  }

  pangolin::OpenGlRenderState &get_ogl_render_state() { return cam_; }

protected:
  std::map<std::string, Objectxd *> objects_;
  std::vector<std::string> insertion_order_;

private:
  /* data */
  bool init_;
  ViewType type_;
  ViewNode *parent_; // This will be null if parent is base view.
  std::string name_;
  std::vector<std::string> rec_name_;

  pangolin::View &v_; // for quick access.
  pangolin::GlText txt_;
  pangolin::Viewport vp_head_;
  pangolin::Viewport vp_head_wc_;   // window coordinates.
  pangolin::Viewport vp_close_wc_;  // window coordinates.
  pangolin::Viewport vp_record_wc_; // window coordinates.

  pangolin::VideoOutput recorder_; // for recording views
  cv::Mat rec_buffer_;
  cv::Size rec_sz_;
  bool rec_resize_;
  std::vector<pangolin::StreamInfo> streams_;
  const int radius_ = 7;

  // this is useful only for 3D views, check if it can be disabled for 2D.
  pangolin::OpenGlRenderState cam_;

  std::function<void(ViewNode *)> close_cb_;

  /* functions */
  int update_head_vp(pangolin::Viewport &vp);
};
