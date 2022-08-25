#pragma once

#include <typeinfo>
#include <utils/mode_printing.h>

#define CHECK_RET(func_call)                                                   \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
      return r;                                                                \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
      return r;                                                                \
    }                                                                          \
  }
#define CHECK_RET_VOID(func_call)                                              \
  {                                                                            \
    int r = func_call;                                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (r) {                                                                   \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
      return;                                                                  \
    }                                                                          \
  }

#define CHECK_RET_TO_VAR(func_call, var)                                       \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
      return r;                                                                \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
      return r;                                                                \
    }                                                                          \
    var = r;                                                                   \
  }

#define CHECK_RET_VAL(func_call, ret)                                          \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", returning " << ct_red(ret) pendl;                 \
      return ret;                                                              \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", returning " << ct_red(ret) pendl;                 \
      return ret;                                                              \
    }                                                                          \
  }

#define CHECK_RET_INFO(func_call, append)                                      \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", addendum: " << append pendl;                      \
      return r;                                                                \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", addendum: " << append pendl;                      \
      return r;                                                                \
    }                                                                          \
  }

#define CHECK_RET_VAL_INFO(func_call, ret, append)                             \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", returning " << ct_red(ret)                        \
          << ", addendum: " << append pendl;                                   \
      return ret;                                                              \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PE$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) << ", returning " << ct_red(ret)                        \
          << ", addendum: " << append pendl;                                   \
      return ret;                                                              \
    }                                                                          \
  }
#define CHECK_WARN(func_call)                                                   \
  {                                                                            \
    decltype(func_call) r = func_call;                                         \
    PI$ #func_call " return value is " << ct_vred(r) pendl;                    \
    if (typeid(r) == typeid(int) && r) {                                       \
      PW$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
    }                                                                          \
    if (typeid(r) == typeid(bool) && !r) {                                     \
      PW$ #func_call " was unsuccessful with return value "                    \
          << ct_red(r) pendl;                                                  \
    }                                                                          \
  }
