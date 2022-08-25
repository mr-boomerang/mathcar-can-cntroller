#pragma once

#include <pangolin/pangolin.h>
#include <utils/error_code.h>
#include <utils/mode_printing.h>
// should include utils/str_utils.hpp for CLS_DEMANGLES_NAME to work.
#define CLS_DEMANGLED_NAME utils::demangle_type(typeid(decltype(*this)).name())

#define CHECK_GL_ERROR(x)                                                      \
  if (gl_error()) {                                                            \
    PE$ "error in class: " << CLS_DEMANGLED_NAME << ", function: " << __func__ \
                           << ", extra info = " << x << pendo;                 \
    return false;                                                              \
  }

#define CHECK_GL_ERROR_CERR(x)                                                 \
  if (gl_error()) {                                                            \
    std::cerr << "error in class: " << CLS_DEMANGLED_NAME                      \
              << ", function: " << __func__ << ", extra info = " << x          \
              << std::endl;                                                    \
    return false;                                                              \
  }

inline bool gl_error() {
  GLenum err = glGetError();
  if (GL_NO_ERROR != err) {
    PE$ "Opengl error code = " << err pendl;
    return RET_SUCCESS_BOOL;
  }
  return false;
}

enum ObjectType { TwoDObject, ThreeDObject };

class Objectxd {
public:
  Objectxd(ObjectType type, float transparency = 1.0f)
      : type_(type), transparency_(transparency), v_(NULL) {}
  virtual ~Objectxd() {}

  virtual ObjectType type() = 0;
  virtual int draw_obj(pangolin::View &v) = 0; // opaque by default
  virtual int set_transparency(float t) = 0;

  virtual int set_view_ptr(pangolin::View *v) { v_ = v; }

  virtual int set_name(const std::string &name) { name_ = name; }

protected:
  /* data */
  ObjectType type_;
  float transparency_;
  pangolin::View *v_; // access to view properties befre draw_obj function.
  std::string name_;
};
