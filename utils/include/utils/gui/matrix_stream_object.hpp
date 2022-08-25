#pragma once

#include <mutex>

#include <utils/error_code.h>
#include <utils/gui/object_xd.hpp>
#include <utils/mode_printing.h>
#include <utils/str_utils.h> //for demangling in CHECK_GL_ERROR.
//#include <utils/fs_utils.h>

#include <pangolin/pangolin.h>

#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#ifdef BUILT_WITH_ROS
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <stereo_msgs/DisparityImage.h>
#endif

template <typename T> class MatrixStreamObject : public Objectxd {
public:
  MatrixStreamObject(int w, int h, bool grayscale = false)
      : Objectxd(TwoDObject) {
    reinit(w, h, grayscale);
  }

  int reinit(int w, int h, bool grayscale = false, T min = 0, T max = 0) {
    static_assert(std::is_arithmetic<T>::value,
                  "MatrixStreamObject is supposed"
                  " to be only for types float, double, int.");
    try {
      if (!w || !h)
        return RET_INVALID_MATRIX_STREAM_OBJECT;
      if (w != img_tex_.width || h != img_tex_.height) {
        if (grayscale) {
          img_tex_.Reinitialise(w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE);
          img_ = new unsigned char[w * h];
        } else {
          img_tex_.Reinitialise(w, h, GL_RGB, GL_UNSIGNED_BYTE);
          img_ = new unsigned char[3 * w * h];
        }

        CHECK_GL_ERROR_CERR(__FILE__ << ":" << __LINE__);
        grayscale_ = grayscale;
        min_ = min;
        max_ = max;
      }
    } catch (std::exception e) {
      DP$ "Exception(" << __FILE__ << ":" << __LINE__ << ") :- "
                       << e.what() pendl;
      DP$ pendo;
    }
    return RET_SUCCESS_INT;
  }

  // This is when it's color.
  int update_jetmap(const T *img, size_t size) {
    if (grayscale_) {
      return 1;
    }
    T min = min_ ? min_ : *std::min_element(img, img + size);
    T max = max_ ? max_ : *std::max_element(img, img + size);
    T range = max - min;

    for (size_t i = 0; i < size; ++i) {
      T elem = img[i];
      if (elem < min) {
        elem = min;
      }
      if (elem > max) {
        elem = max;
      }

      size_t idx = 3 * i;
      if (elem < (min + 0.25 * range)) {
        img_[idx] = (unsigned char)0;
        img_[idx + 1] = (unsigned char)((4 * (elem - min) / range) * 255);
        img_[idx + 2] = (unsigned char)255;
      } else if (elem < (min + 0.5 * range)) {
        img_[idx] = (unsigned char)0;
        img_[idx + 1] = (unsigned char)255;
        img_[idx + 2] =
            (unsigned char)((1 + 4 * (min + 0.25 * range - elem) / range) *
                            255);
      } else if (elem < (min + 0.75 * range)) {
        img_[idx] =
            (unsigned char)((4 * (elem - min - 0.5 * range) / range) * 255);
        img_[idx + 1] = (unsigned char)255;
        img_[idx + 2] = (unsigned char)0;
      } else {
        img_[idx] = (unsigned char)255;
        img_[idx + 1] =
            (unsigned char)((1 + 4 * (min + 0.75 * range - elem) / range) *
                            255);
        img_[idx + 2] = (unsigned char)0;
      }
#if 0
        img_[idx] = 255;
        img_[idx+1] = (unsigned char)((1+4*(min+0.75*range-elem)/range)*255);
        img_[idx+2] = 0;
#endif
    }
    return RET_SUCCESS_INT;
  }

  // This is when we want grayscale.
  int update_grayscale(const T *img, size_t size) {
    if (!grayscale_)
      return 1;
    T min = min_ ? min_ : *std::min_element(img, img + size);
    T max = max_ ? max_ : *std::max_element(img, img + size);
    T range = max - min;
    for (size_t i = 0; i < size; ++i) {
      T elem = img[i];
      img_[i] = (elem - min) / range;
    }
    return RET_SUCCESS_INT;
  }

  int update_img(const T *img, int w, int h) {
    if (img == NULL)
      return false;

    if (w != img_tex_.width || h != img_tex_.height)
      reinit(w, h);

    grayscale_ ? !(update_grayscale(img, (size_t)w * h))
               : !(update_jetmap(img, (size_t)w * h));

    mtx_.lock();
    img_tex_.Upload(img_, (grayscale_ ? GL_LUMINANCE : GL_RGB),
                    GL_UNSIGNED_BYTE);
    mtx_.unlock();
    return RET_SUCCESS_INT;
  }

  int draw_obj(pangolin::View & /*v*/) { // opaque by default
    glColor4f(1.0f, 1.0f, 1.0f, transparency_);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    if (img_tex_.tid) {
      mtx_.lock();
      img_tex_.RenderToViewportFlipY();
      mtx_.unlock();
      CHECK_GL_ERROR_CERR(__FILE__ << ":" << __LINE__);
    }
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    return RET_SUCCESS_INT;
  }

  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

#ifdef UTILS_USE_OPENCV
  int update_img(const cv::Mat &img);
#endif

#ifdef BUILT_WITH_ROS
  enum MessageType { Disparity };
  MatrixStreamObject(const MessageType &type, const std::string &topic_sub,
                     ros::NodeHandle &n, uint32_t queue_sz = 5,
                     bool grayscale = false)
      : Objectxd(TwoDObject) {
    reinit(type, topic_sub, n, queue_sz, grayscale);
  }

  int reinit(const MessageType &type, const std::string &topic_sub,
             ros::NodeHandle &n, uint32_t queue_sz = 5,
             bool grayscale = false) {
    static_assert(std::is_arithmetic<T>::value,
                  "MatrixStreamObject is supposed"
                  " to be only for types float, double, int.");

    switch (type) {
    case Disparity:
      if (!std::is_same<T, float>::value)
        return false;
      sub_ = n.subscribe(topic_sub, queue_sz,
                         &MatrixStreamObject::disparity_callback, this);
      break;
    default:
      return false;
    }

    grayscale_ = grayscale;

    return RET_SUCCESS_INT;
  }
  void disparity_callback(const stereo_msgs::DisparityImageConstPtr &img_sub) {
    if (!min_)
      min_ = img_sub->min_disparity;
    if (!max_)
      max_ = img_sub->max_disparity;
    update_img((T *)img_sub->image.data.data(), img_sub->image.width,
               img_sub->image.height);
  }
#endif

private:
  /* data */
  bool grayscale_;
  unsigned char *img_;
  pangolin::GlTexture img_tex_;
  std::mutex mtx_;

  T min_, max_;

#ifdef UTILS_USE_OPENCV
  cv::VideoCapture vc_;
#endif

#ifdef BUILT_WITH_ROS
  ros::Subscriber sub_;
/* callback function for the subsriber */
#endif
};
typedef MatrixStreamObject<float> FloatVideoObject;
typedef MatrixStreamObject<double> DoubleVideoObject;
typedef MatrixStreamObject<int> IntVideoObject;
