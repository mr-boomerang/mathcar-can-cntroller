#pragma once

#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

#include <algorithm>
#include <cstdlib>

#include <utils/mode_printing.h>
#include <utils/str_utils.h>
namespace utils {

struct Color {

  enum Type {
    invalid,
    generic,
    random,
    black,
    white,
    grey,
    red,
    green,
    blue,
    hred,
    hgreen,
    hblue,
    cyan,
    yellow,
    magenta,
    hcyan,
    hyellow,
    hmagenta,
    orange // orange is red + half green
  };

  Color(Type c = black) : col_(c) {
    raw_col_ = new float[4];
    getColForenum(col_, raw_col_);

#ifdef UTILS_USE_OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = cv::Scalar(int(raw_col_[2] * 255), int(raw_col_[1] * 255),
                           int(raw_col_[0] * 255), int(raw_col_[3] * 255));
#endif
  }

  Color(float r, float g, float b) {
    raw_col_ = new float[4];
    raw_col_[0] = r;
    raw_col_[1] = g;
    raw_col_[2] = b;
    raw_col_[3] = 1.0f;
    col_ = generic;
#ifdef UTILS_USE_OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = cv::Scalar(int(raw_col_[2] * 255), int(raw_col_[1] * 255),
                           int(raw_col_[0] * 255), int(raw_col_[3] * 255));
#endif
  }

  Color(const std::string &col) {
    raw_col_ = new float[4];
    std::string col_name = utils::to_lower(col);
    if ("red" == col_name) {
      raw_col_[0] = 1.0f, raw_col_[1] = 0.0f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = red;
    } else if ("green" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 1.0f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = green;
    } else if ("blue" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 0.0f;
      raw_col_[2] = 1.0f, raw_col_[3] = 1.0f;
      col_ = blue;
    } else if ("yellow" == col_name) {
      raw_col_[0] = 1.0f, raw_col_[1] = 1.0f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = yellow;
    } else if ("magenta" == col_name) {
      raw_col_[0] = 1.0f, raw_col_[1] = 0.0f;
      raw_col_[2] = 1.0f, raw_col_[3] = 1.0f;
      col_ = magenta;
    } else if ("cyan" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 1.0f;
      raw_col_[2] = 1.0f, raw_col_[3] = 1.0f;
      col_ = cyan;
    } else if ("hyellow" == col_name) {
      raw_col_[0] = 0.5f, raw_col_[1] = 0.5f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = hyellow;
    } else if ("hmagenta" == col_name) {
      raw_col_[0] = 0.5f, raw_col_[1] = 0.0f;
      raw_col_[2] = 0.5f, raw_col_[3] = 1.0f;
      col_ = hmagenta;
    } else if ("hcyan" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 0.5f;
      raw_col_[2] = 0.5f, raw_col_[3] = 1.0f;
      col_ = hcyan;
    } else if ("hred" == col_name) {
      raw_col_[0] = 0.5f, raw_col_[1] = 0.0f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = hred;
    } else if ("hgreen" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 0.5f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = hgreen;
    } else if ("hblue" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 0.0f;
      raw_col_[2] = 0.5f, raw_col_[3] = 1.0f;
      col_ = hblue;
    } else if ("black" == col_name) {
      raw_col_[0] = 0.0f, raw_col_[1] = 0.0f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = black;
    } else if ("white" == col_name) {
      raw_col_[0] = 1.0f, raw_col_[1] = 1.0f;
      raw_col_[2] = 1.0f, raw_col_[3] = 1.0f;
      col_ = white;
    } else if ("grey" == col_name) {
      raw_col_[0] = 0.5f, raw_col_[1] = 0.5f;
      raw_col_[2] = 0.5f, raw_col_[3] = 1.0f;
      col_ = grey;
    } else if ("orange" == col_name) {
      raw_col_[0] = 1.0f, raw_col_[1] = 0.5f;
      raw_col_[2] = 0.0f, raw_col_[3] = 1.0f;
      col_ = orange;
    } else if ("random" == col_name) {
      raw_col_[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[1] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[2] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[3] = 1.0f;
      col_ = random;
    } else if ("generic" == col_name) {
      raw_col_[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[1] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[2] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      raw_col_[3] = 1.0f;
      col_ = generic;
    } else {
      raw_col_[0] = -1.0f, raw_col_[1] = -1.0f;
      raw_col_[2] = -1.0f, raw_col_[3] = 1.0f;
      col_ = invalid;
      PE$ "Error: Check the spelling or color is not documented in "
          "this class" pendl;
    }
#ifdef UTILS_USE_OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = cv::Scalar(int(raw_col_[2] * 255), int(raw_col_[1] * 255),
                           int(raw_col_[0] * 255), int(raw_col_[3] * 255));
#endif
  }

  Color(const Color &obj) : col_(obj.col_) {
    raw_col_ = new float[4];
    raw_col_[0] = obj.raw_col_[0];
    raw_col_[1] = obj.raw_col_[1];
    raw_col_[2] = obj.raw_col_[2];
    raw_col_[3] = obj.raw_col_[3];

#ifdef UTILS_USE_OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = obj.cv_color_;
#endif
  }

  const Color &operator()() const { return *this; }

  void operator=(const Color &rhs) {
    raw_col_[0] = rhs.raw_col_[0];
    raw_col_[1] = rhs.raw_col_[1];
    raw_col_[2] = rhs.raw_col_[2];
    raw_col_[3] = rhs.raw_col_[3];

    col_ = rhs.col_;

#ifdef UTILS_USE_OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = rhs.cv_color_;
#endif
  }

  void operator=(Type c) {
    getColForenum(c, raw_col_);

#ifdef OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = cv::Scalar(int(raw_col_[2] * 255), int(raw_col_[1] * 255),
                           int(raw_col_[0] * 255), int(raw_col_[3] * 255));
#endif
  }

  bool operator==(const Color::Type &rhs) const { return (this->col_ == rhs); }

  bool operator==(const Color &rhs) const { return (this->col_ == rhs.col_); }

#ifdef UTILS_USE_OPENCV
  operator cv::Scalar() const { return cv_color_; }
#endif

  operator float *() const { return raw_col_; }

  Color operator/(float denom) const {
    return Color(raw_col_[0] / denom, raw_col_[1] / denom, raw_col_[2] / denom);
  }

  float operator[](int idx) const { return get_colf(idx); }

  void operator+=(Type c) {
    float cur[4];
    getColForenum(c, cur);

    raw_col_[0] = std::min(1.0f, cur[0] + raw_col_[0]);
    raw_col_[1] = std::min(1.0f, cur[1] + raw_col_[1]);
    raw_col_[2] = std::min(1.0f, cur[2] + raw_col_[2]);
    raw_col_[3] = std::min(1.0f, cur[3] + raw_col_[3]);

#ifdef OPENCV
    // scalar order is BGR and rgb in float*
    cv_color_ = cv::Scalar(int(raw_col_[2] * 255), int(raw_col_[1] * 255),
                           int(raw_col_[0] * 255), int(raw_col_[3] * 255));
#endif
  }

  Type col_type() const { return col_; }

  float get_colf(int idx) const {
    if (idx < 0 && idx > 3)
      return -1.0f;
    return raw_col_[idx];
  }

  void set_alpha(float alpha) {
    if (alpha >= 0.0f && alpha <= 1.0f)
      raw_col_[3] = alpha;
  }

  float get_alpha() { return raw_col_[3]; }

private:
  Type col_;
#ifdef UTILS_USE_OPENCV
  cv::Scalar cv_color_;
#endif
  float *raw_col_;

  void getColForenum(Type c, float *ret) {
    switch (c) {
    case red:
      ret[0] = 1.0f, ret[1] = 0.0f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case green:
      ret[0] = 0.0f, ret[1] = 1.0f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case blue:
      ret[0] = 0.0f, ret[1] = 0.0f, ret[2] = 1.0f, ret[3] = 1.0f;
      break;
    case yellow:
      ret[0] = 1.0f, ret[1] = 1.0f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case magenta:
      ret[0] = 1.0f, ret[1] = 0.0f, ret[2] = 1.0f, ret[3] = 1.0f;
      break;
    case cyan:
      ret[0] = 0.0f, ret[1] = 1.0f, ret[2] = 1.0f, ret[3] = 1.0f;
      break;
    case hyellow:
      ret[0] = 0.5f, ret[1] = 0.5f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case hmagenta:
      ret[0] = 0.5f, ret[1] = 0.0f, ret[2] = 0.5f, ret[3] = 1.0f;
      break;
    case hcyan:
      ret[0] = 0.0f, ret[1] = 0.5f, ret[2] = 0.5f, ret[3] = 1.0f;
      break;
    case hred:
      ret[0] = 0.5f, ret[1] = 0.0f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case hgreen:
      ret[0] = 0.0f, ret[1] = 0.5f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case hblue:
      ret[0] = 0.0f, ret[1] = 0.0f, ret[2] = 0.5f, ret[3] = 1.0f;
      break;
    case black:
      ret[0] = 0.0f, ret[1] = 0.0f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case white:
      ret[0] = 1.0f, ret[1] = 1.0f, ret[2] = 1.0f, ret[3] = 1.0f;
      break;
    case grey:
      ret[0] = 0.5f, ret[1] = 0.5f, ret[2] = 0.5f, ret[3] = 1.0f;
      break;
    case orange:
      ret[0] = 1.0f, ret[1] = 0.5f, ret[2] = 0.0f, ret[3] = 1.0f;
      break;
    case generic:
      ret[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[1] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[2] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[3] = 1.0f;
      break;
    case random:
      ret[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[1] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[2] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      ret[3] = 1.0f;
      break;
    case invalid:
      ret[0] = -1.0f, ret[1] = -1.0f, ret[2] = -1.0f, ret[3] = -1.0f;
      break;
    }
  }
};

inline void color_as_ostream(std::ostream &os, const utils::Color &col) {
  switch (col.col_type()) {
  case Color::red:
    os << "red";
    break;
  case Color::green:
    os << "green";
    break;
  case Color::blue:
    os << "blue";
    break;
  case Color::yellow:
    os << "yellow";
    break;
  case Color::magenta:
    os << "magenta";
    break;
  case Color::cyan:
    os << "cyan";
    break;
  case Color::hyellow:
    os << "half yellow";
    break;
  case Color::hmagenta:
    os << "half magenta";
    break;
  case Color::hcyan:
    os << "half cyan";
    break;
  case Color::hred:
    os << "half red";
    break;
  case Color::hgreen:
    os << "half green";
    break;
  case Color::hblue:
    os << "half blue";
    break;
  case Color::black:
    os << "black";
    break;
  case Color::white:
    os << "white";
    break;
  case Color::grey:
    os << "grey";
    break;
  case Color::orange:
    os << "orange";
    break;
  case Color::invalid:
    os << "Invalid color";
    break;
  case Color::random:
    os << "Random color(" << col[0] << "," << col[1] << "," << col[2] << ")";
    break;
  case Color::generic:
    os << "Generic color(" << col[0] << "," << col[1] << "," << col[2] << ")";
    break;
  default:
    os << "No color";
    break;
  }
}

inline std::ostream &operator<<(std::ostream &os, utils::Color &col) {
  color_as_ostream(os, col);
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const utils::Color &col) {
  color_as_ostream(os, col);
  return os;
}

inline Color operator+(Color::Type c1, Color::Type c2) {
  Color col(c1);
  col += c2;
  return col;
}

inline Color operator/(Color::Type c, float denom) {
  Color col(c);
  return col / denom;
}

} /* utils */
