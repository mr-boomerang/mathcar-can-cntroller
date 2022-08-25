#pragma once

// strings and co
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>

// for time.
#include <ctime>
#include <iomanip>

// for INT_MAX
#include <limits.h>

// stl's
// If one wishes to include stl headers, before including this file define
// UTILS_USE_STL. Like in the line below.
//#define UTILS_USE_STL
#ifdef UTILS_USE_STL
#include "stl_utils.h"
#define STL_DEFINED_STATE std::cout << "STL defined" << std::endl;
#else
#define STL_DEFINED_STATE std::cout << "STL not defined" << std::endl;
#endif

#ifndef STR
#define STR(x) std::string(x)
#endif

// to tackle c++11 errors
#ifdef BOOST_NOT_CPP11
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#else
#include <memory>
#endif
#include <utils/boostd.h>

// other cpp standards
#include <algorithm>
#include <exception>

#ifdef _WIN32
#define PATH_SEPERATOR '\\' // i.e. backslash
#elif defined __linux__ || defined __APPLE__
#define PATH_SEPERATOR '/' // i.e. forwardslash
#else
#define PATH_SEPERATOR '/' // i.e. forwardslash
#endif
// If one doesn't wish to use gflags lib, before including this file define
// UTILS_AVOID_GFLAGS. Like in the line below.
//#define UTILS_AVOID_GFLAGS
#ifndef UTILS_AVOID_GFLAGS
#include <gflags/gflags.h>
#else
#include <utils/miniflags.h>
#endif

#include <utils/colored_text.h>

#define pendo "\n"
#define pcommas << ", " <<
#define pspace << " " <<
#ifndef COMMON_PRINT_NO_DEF
#define pendl << "\n"
#define pmspace(n) << std::string(n, ' ') <<
#define ptab << "\t" <<
#define ptabo << "\t"
#define pmtab(n) << std::string(n, '\t') <<
#define pcomma << "," <<
#define pcommao << ","
#define pcommaso << ", "
#define pcolon << "::" <<
#define pcolons << " :: " <<
#define pcolono << "::"
#define pcolonso << " :: "
#define pequal << "=" <<
#define pequals << " = " <<
#define pequalo << "="
#define pequalso << " = "
#endif

namespace sl {
const int INFO = 0;
const int WARN = 1;
const int ERROR = 2;
const int FATAL = INT_MAX;
const int CALM = 3; // sets same verbosity level as FATAL but doesn't exit.
const int SILENCE = 4;
} /* sl */

// bool splitVerbose(const char* flagname, const std::string& val) {
// std::cout << "validating " << flagname << " for " << val << std::endl;
// return true;
//}

// declare flags defined in corresponding cpp file.
DECLARE_bool(logtocerr);
DECLARE_bool(logtofile);
DECLARE_bool(alsologtofile);
DECLARE_string(logprefix);
DECLARE_string(logdir);

DECLARE_int32(sl);
DECLARE_string(sections);

namespace print {

extern std::string prog_name;

#define PREFIXED_STR(x, c, w) std::setfill(c) << std::setw(w) << x
// Check Level
#define CHK_LVL(x) if (FLAGS_sl <= x)
//[Check] [Level] then provide [Stream]
#define CLS(x)                                                                 \
  if (FLAGS_sl <= x)                                                           \
  print::Logger().stream() <<

#ifndef NDEBUG
#define DBG 1
#else
#define DBG 0
#endif

// Print in various forms
// Predefined Values for printing
#define PRINT_INFO ct::DBU << "I: " << DDL
#define PRINT_TEMP ct::DBU << "Delete this print statement: " << DDL
#define PRINT_SUCCESS ct::DGN << "Success: " << DDL
#define PRINT_WARN ct::DYW << "W: " << DDL
#define PRINT_ERROR ct::RED << "E: " << DDL
#define PRINT_FATAL ct::DRD << "F: " << DDL
#define PRINT_DEBUG ct::DCN << "D: " << DDL
#define PRINT_SECTION(x) ct::DPR << "S(" << x << "): " << DDL

#define PRINT_FUNC_ENTER                                                       \
  if (DBG)                                                                     \
    CLS(sl::INFO) PRINT_DEBUG << __func__ << "::::enter" << pendo;
#define PRINT_FUNC_EXIT                                                        \
  if (DBG)                                                                     \
    CLS(sl::INFO) PRINT_DEBUG << __func__ << "::::exit" << pendo;

#define PRINT_SEPERATOR                                                        \
  CLS(sl::CALM)                                                                \
  ct::DPR << "----------------------------" << ct::DCN                         \
          << "----------------------------" << ct::RST << pendo
#define PRINT_SEPERATORI                                                       \
  CLS(sl::INFO)                                                                \
  ct::DPR << "----------------------------" << ct::DCN                         \
          << "----------------------------" << ct::RST << pendo
#define PRINT_SEPERATOR_SEC(sec)                                               \
  if (print::section_map != NULL && SEC_COND(sec)) {                           \
    PRINT_SEPERATOR;                                                           \
  }

inline std::string bfn(const char *fullname) { // basefilename
  // const char* name = strrchr(fullname, '/');
  // return (name==NULL)?fullname:(name+1);
  std::string str(fullname);
  size_t p = str.rfind('/') + 1;
  return (p == std::string::npos) ? fullname : str.substr(p, std::string::npos);
}

#define LOCATION                                                               \
  << print::bfn(__FILE__) << ":" << __LINE__ << "(" << __func__ << ")" <<
#define PRINT_LOC(c) c << "[" LOCATION "] " << ct::RST

#define PXLE(c) CLS(sl::INFO) PRINT_LOC(c) << std::endl
#define PDLE PXLE(ct::GRY)
#define PRLE PXLE(ct::RED)
#define PGLE PXLE(ct::GRN)
#define PBLE PXLE(ct::BLU)
#define PYLE PXLE(ct::YLW)
#define PPLE PXLE(ct::PUR)
#define PCLE PXLE(ct::CYN)

#define PXL CLS(sl::INFO) PRINT_LOC(c)
#define PDL PXL(ct::GRY)
#define PRL PXL(ct::RED)
#define PGL PXL(ct::GRN)
#define PBL PXL(ct::BLU)
#define PYL PXL(ct::YLW)
#define PPL PXL(ct::PUR)
#define PCL PXL(ct::CYN)

#ifndef NDEBUG
#define DPXLE(c) CLS(sl::INFO) PRINT_LOC(c) << std::endl
#define DPDLE DPXLE(ct::GRY)
#define DPRLE PXLE(ct::RED)
#define DPGLE PXLE(ct::GRN)
#define DPBLE PXLE(ct::BLU)
#define DPYLE PXLE(ct::YLW)
#define DPPLE PXLE(ct::PUR)
#define DPCLE PXLE(ct::CYN)

#define DDL                                                                    \
  PRINT_LOC(ct::GRY) // i know naming here is cheating, but this is used a lot.

#else
#define DPXLE(c)
#define DPDLE DPXLE(ct::GRY)
#define DPRLE DPXLE(ct::RED)
#define DPGLE DPXLE(ct::GRN)
#define DPBLE DPXLE(ct::BLU)
#define DPYLE DPXLE(ct::YLW)
#define DPPLE DPXLE(ct::PUR)
#define DPCLE DPXLE(ct::CYN)

#define DDL ct::RST

#endif

// Print modes
// Normal print, no forced format, should print message in default scenario,
// hence level is sl::ERROR
#define PN$ CLS(sl::ERROR) DDL <<
// Normal print, no forced format, should print message in verbose mode, hence
// level is sl::INFO
#define PV$ CLS(sl::INFO) DDL <<
#define PI$ CLS(sl::INFO) PRINT_INFO <<   // Info print
#define PT$ CLS(sl::CALM) PRINT_TEMP <<   // Info print
#define PW$ CLS(sl::WARN) PRINT_WARN <<   // Warning print
#define PE$ CLS(sl::ERROR) PRINT_ERROR << // Error print
// Fatal print, code will exit when this is printed.
#define DP$                                                                    \
  if (DBG)                                                                     \
  CLS(sl::CALM) PRINT_DEBUG <<
#define PF$(ec) CHK_LVL(sl::FATAL) print::Logger(ec).stream() << PRINT_FATAL <<

// Continue the prev mode, i.e. no header printing. Useful in loops.
#define PIC$ CLS(sl::INFO)
#define PWC$ CLS(sl::WARN)
#define PEC$ CLS(sl::ERROR)
#define DPC$                                                                   \
  if (DBG)                                                                     \
  CLS(sl::CALM)
#define PFC$(ec) CHK_LVL(sl::FATAL) print::Logger(ec).stream() <<

// PS is for print string, which can be a variable too, but it's name won't be
// printed, just the value. PV is for print variable, that is print variable
// name then it's value.
#define PSEC(x, col) CLS(sl::INFO) DDL col << x << ct::RST pend
#define PVEC(x, col) CLS(sl::INFO) DDL col << #x << ct::RST pequal x pend

// Print Matrix.
#define PMC(x, col) CLS(sl::INFO) DDL col << #x << ct::RST pend << x pend
#define PMLC(x, col)                                                           \
  CLS(sl::INFO) PRINT_LOC(ct::GRY) << std::endl;                               \
  PMC(x, col)

// Print conditionally
// CP
#define cPN$(cond)                                                             \
  if (cond)                                                                    \
  PN$
#define cPV$(cond)                                                             \
  if (cond)                                                                    \
  PV$
#define cPI$(cond)                                                             \
  if (cond)                                                                    \
  PI$
#define cPW$(cond)                                                             \
  if (cond)                                                                    \
  PW$
#define cPE$(cond)                                                             \
  if (cond)                                                                    \
  PE$
#define cDP$(cond)                                                             \
  if (cond)                                                                    \
  DP$
#define cPF$(cond, ec)                                                         \
  if (cond)                                                                    \
  PF$(ec)
// Continue mode.
#define cPNC$(cond)                                                            \
  if (cond)                                                                    \
  PNC$
#define cPVC$(cond)                                                            \
  if (cond)                                                                    \
  PVC$
#define cPIC$(cond)                                                            \
  if (cond)                                                                    \
  PIC$
#define cPWC$(cond)                                                            \
  if (cond)                                                                    \
  PWC$
#define cPEC$(cond)                                                            \
  if (cond)                                                                    \
  PEC$
#define cDPC$(cond)                                                            \
  if (cond)                                                                    \
  DPC$
#define cPFC$(cond, ec)                                                        \
  if (cond)                                                                    \
  PFC$(ec)

// Section printing, i.e. only print if the section number is mentioned in
// command line flag. For e.g. if the code contains the follwing line
//  PS$(10) "Object created" pend;
// A normal ./prog_name won't print it, but following command will print it
//  ./prog_name --sections=10 or ./prog_name --sections=1,3,10
// if section_size is non-zero, section map would be not null, this condition
// to be paranoid and make sure segfault doesn't happen.
#define SEC_COND(x) x > 0 && x < print::section_size &&print::section_map[x]
#define SP$(x)                                                                 \
  if (print::section_map != NULL && SEC_COND(x))                               \
  CLS(sl::CALM) PRINT_SECTION(x) <<
#define SPC$(x)                                                                \
  if (print::section_map != NULL && SEC_COND(x))                               \
  CLS(sl::CALM)

#define Pcond(cond, x)                                                         \
  if (cond)                                                                    \
    CLS(sl::CALM) x;
#define PCOND(cond, x, y)                                                      \
  if (cond) {                                                                  \
    CLS(sl::CALM) x;                                                           \
  } else {                                                                     \
    CLS(sl::CALM) y;                                                           \
  }
#define PCONDI(cond, x, y)                                                     \
  if (cond) {                                                                  \
    CLS(sl::INFO) x;                                                           \
  } else {                                                                     \
    CLS(sl::INFO) y;                                                           \
  }

// Print Vector.
#define PRINT_VECTOR(x)                                                        \
  DPDLE;                                                                       \
  CLS(sl::INFO) ct::GRN << #x << "(" << x.size() << "):";                      \
  PCONDI(!x.size(), ct_bred("Empty Vec\n"), ct_rst("\n" << x[0]));             \
  for (size_t ii = 1; ii < x.size(); ii++)                                     \
    CLS(sl::INFO) ", " << x[ii];                                               \
  CLS(sl::INFO) pendo;

#define DPRINT_VECTOR(x)                                                       \
  if (DBG)                                                                     \
  PRINT_VECTOR(x)

#define PRINT_VECTOR_SEC(sec, x)                                               \
  if (print::section_map != NULL && SEC_COND(sec)) {                           \
    int flag_sl = FLAGS_sl;                                                    \
    FLAGS_sl = 0;                                                              \
    PRINT_VECTOR(x);                                                           \
    FLAGS_sl = flag_sl;                                                        \
  }

// Shared pointer declares
#define S_V(x) *x.get()

struct LogFile {

  LogFile();

  ~LogFile() {
    log_file_.close();
    std::cout << "Output logged to " << log_fn_ << std::endl;
  }

  inline std::ofstream &operator()() { return log_file_; }

  inline bool is_open() { return log_file_.is_open(); }

private:
  std::ofstream log_file_;
  std::string log_fn_;
};

extern utils_boostd::shared_ptr<LogFile> lf;
extern bool *section_map;
extern int section_size;

class Logger {
public:
  Logger(int exit_code = 0) : exit_code_(exit_code) {}

  ~Logger() {
    just_print_it();
    if (exit_code_) {
      // Print it anyhow.
      std::cout << ct_bred("Fatal Exit") << "\n";
      exit(exit_code_);
    }
  }

  std::stringstream &stream() { return ss_; }

private:
  /* data */
  std::stringstream ss_;
  int exit_code_;

  void just_print_it();
};

void init(int *argc, char ***argv, bool remove_flags = false);

} /* print */

#ifdef STL_DEF
// operators for STL's

#define PRINT_STL(x)                                                           \
  DPDLE;                                                                       \
  CLS(sl::INFO) ct::GRN << #x << "(" << x.size() << "): " << ct::RST;          \
  CLS(sl::INFO)(x.size() ? "\n" : "");                                         \
  CLS(sl::INFO) x pendl;

#define PRINT_STL_SEC(sec, x)                                                  \
  if (print::section_map != NULL && SEC_COND(sec)) {                           \
    PRINT_STL(x);                                                              \
  }
#define DPRINT_STL(x)                                                          \
  if (DBG)                                                                     \
  PRINT_STL(x)

#else

#define PRINT_STL(x)
#define PRINT_STL_SEC(sec, x)
#define DPRINT_STL(x)

#endif /* STL_DEF */
