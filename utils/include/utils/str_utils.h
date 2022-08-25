#ifndef STR_UTILS_H_MU7OQJD1
#define STR_UTILS_H_MU7OQJD1

// string headers
#include <iostream> //to print message and get input in get_*input funcs.
#include <sstream>
#include <string>

#if defined(__GLIBCXX__) || defined(__GLIBCPP__)
#include <cstdlib>
#include <cxxabi.h>
#endif

// to tackle c++11 errors
#ifdef BOOST_NOT_CPP11
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#else
#include <memory>
#endif
#include <utils/boostd.h>
#include <utils/error_code.h>

// stl's
// If one doesn't wish to include stl headers, before including this file define
// USE_STL_UTIL. Like in the line below.
//#define USE_STL_UTIL
// stl headers
#include <vector>
#ifdef USE_STL_UTIL
#include "stl_utils.h"
#endif
// std useful functions
#include <algorithm>

#ifndef STR
#define STR(x) std::string(x)
#endif

// These functions are kept simple, no error checking or validation, it's
// assumed
// user will take care of it.

typedef std::vector<std::string> vec_str;

#ifndef BOOST_NOT_CPP11
inline std::string operator"" _s(const char *ch, size_t) {
  return std::string(ch);
}
#endif

namespace utils {
/*------------------------------------------------------
 * taken from SO
 * link:
 * http://stackoverflow.com/questions/281818/unmangling-the-result-of-stdtype-infoname
 */

#if defined(__GLIBCXX__) || defined(__GLIBCPP__)

#ifdef BOOST_NOT_CPP11
struct Unique_Ptr_Deleter {
  void operator()(char *p) { std::free(p); }
};
#endif

inline std::string demangle_type(const char *name) {

  int status = -4; // some arbitrary value to eliminate the compiler warning

#ifdef BOOST_NOT_CPP11
  boost::interprocess::unique_ptr<char, Unique_Ptr_Deleter> res(
      abi::__cxa_demangle(name, NULL, NULL, &status));
#else
  // enable c++11 by passing the flag -std=c++11 to g++
  std::unique_ptr<char, void (*)(void *)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
#endif
  std::string ret;
  if (status == 0) {
    ret = res.get();
    // TODO: This might need to change with versions, think of a better
    // solution.
    if (ret.find("basic_string<char, std::char_traits<char>, " // with " "
                 "std::allocator<char> >") != std::string::npos ||
        ret.find("basic_string<char,std::char_traits<char>," // without " "
                 "std::allocator<char>>") != std::string::npos) {
      ret = "std::string";
    }
  } else
    ret = name;

  return ret;
}

#else

// does nothing if not g++
inline std::string demangle_type(const char *name) { return name; }

#endif

inline std::string demangle_type(const std::string &type) {
  return demangle_type(type.c_str());
}

inline std::string to_lower(const std::string &orig_str) {
  std::string str = orig_str;
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return str;
}

inline std::string to_upper(const std::string &orig_str) {
  std::string str = orig_str;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  return str;
}

inline bool isSpace(char c) {
  // char for some locale e` or beta etc might result in -ve value, but is space
  // expects +ve values, hence the cast and a function defined to enable the
  // cast.
  return std::isspace(static_cast<unsigned char>(c));
}

inline std::string &ltrim(std::string &str) {
  // c++11 version.
  // str.erase(str.begin(), std::find_if_not(str.begin(), str.end(), isSpace));
  // back compatible
  while (!str.empty() && isSpace(str[0])) {
    str.erase(0, 1);
  }
  return str;
}

inline std::string &rtrim(std::string &str) {
  // c++11 version.
  // don't know why base() needs to be called :S
  // str.erase(std::find_if_not(str.rbegin(), str.rend(), isSpace).base(),
  // str.end());
  // back compatible
  size_t lc = str.length() - 1; // last char
  while (!str.empty() && lc < str.length() && isSpace(str[lc])) {
    str.erase(lc, 1);
    lc--;
  }
  return str;
}

inline std::string &trim(std::string &str) { return ltrim(rtrim(str)); }

inline char last_char(const std::string &str) {
  return str.at(str.length() - 1);
}

inline std::string erase_last_nchar(std::string &str, int n = 1) {
  return str.erase(str.length() - 1, n);
}

inline std::string erase_first_nchar(std::string &str, int n = 1) {
  return str.erase(0, n);
}

// conversion utils.
template <typename T> inline std::string val_to_str(const T &val) {
  std::ostringstream oss;
  oss << val;
  return oss.str();
}

template <typename T> inline int val_to_str(const T &val, std::string *str) {
  std::ostringstream oss;
  oss << val;
  *str = oss.str();
  return RET_SUCCESS_INT;
}

template <typename T>
inline int val_to_str(const T &val,
                      utils_boostd::shared_ptr<std::string> &str) {
  std::ostringstream oss;
  oss << val;
  if (str)
    *str = oss.str();
  else
    str = utils_boostd::make_shared<std::string>(oss.str());
  return RET_SUCCESS_INT;
}

template <typename T> inline T str_to_val(const std::string &str) {
  T val;
  std::istringstream iss(str);
  iss >> val;
  return val;
}

template <typename T> inline void str_to_val(const std::string &str, T *val) {
  std::istringstream iss(str);
  iss >> *val;
}

template <> inline std::string str_to_val(const std::string &str) {
  return str;
}

template <> inline void str_to_val(const std::string &str, std::string *val) {
  *val = str;
}

// replace occurances of a sub-string with another substring. n_replace
// specifies
// how many occurances to replace, -1 means all occurances.
std::string replace(const std::string &str, const std::string &replace,
                    const std::string &with, int n_replace = -1,
                    int *count_replace = NULL);

// returns number of replacements.
int replace_inplace(std::string &str, const std::string &replace,
                    const std::string &with, int n_replace = -1);

// split string in two at nth(idx from 1) occurance of delim, if n_occurance is
// greater than number of delims actually present then str is left as it is
//-ve n_occurance will search from back
std::string split_str_in_two(std::string &str, const std::string &delim,
                             int n_occurance = 1);

int split_str_in_two(const std::string &str, const std::string &delim,
                     std::string *first, std::string *second,
                     int n_occurance = 1);

int split_str_in_two(const std::string &str, const std::string &delim,
                     utils_boostd::shared_ptr<std::string> &first,
                     utils_boostd::shared_ptr<std::string> &second,
                     int n_occurance = 1);

// string to list and vice versa.
// string to list and vice versa.
template <typename T>
inline int split_str(const std::string &str, char delim,
                     std::vector<T> *elems) {
  std::string token;
  std::stringstream ss(str);
  // on delim not found, elems will have one element.
  while (std::getline(ss, token, delim)) {
    elems->push_back(str_to_val<T>(token));
  }
  return RET_SUCCESS_INT;
}

template <typename T>
inline std::vector<T> split_str(const std::string &str, char delim) {
  std::vector<T> elems;
  split_str(str, delim, &elems);
  return elems;
}

template <typename T>
inline int split_str(const std::string &str, const std::string &delim,
                     std::vector<T> *elems) {
  std::string token;
  size_t epos, spos = 0;

  while ((epos = str.find(delim, spos)) != std::string::npos) {
    token = str.substr(spos, epos - spos);
    elems->push_back(str_to_val<T>(token));
    spos = epos + delim.length();
  }
  elems->push_back(str_to_val<T>(str.substr(spos)));
  return RET_SUCCESS_INT;
}

template <typename T>
inline std::vector<T> split_str(const std::string &str, std::string delim) {
  std::vector<T> elems;
  split_str(str, delim, &elems);
  return elems;
}

#ifndef PAIR_OPERATOR_DEF
#define PAIR_OPERATOR_DEF
template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os,
                                const std::pair<key, val> &p) {
  os << "[" << p.first << ";" << p.second << "]";
  return os;
}
#endif

template <typename T>
inline std::string join_stl(const T &stl, const std::string &delim = ",") {
  std::stringstream ss;
#ifdef STL_DEF
  if (delim == ",")
    return val_to_str(stl);
#endif
  typename T::const_iterator penultimate = stl.end();
  penultimate--;
  typename T::const_iterator i = stl.begin();
  for (; i != penultimate; ++i) {
    ss << *i << delim;
  }
  ss << *i;
  return ss.str();
}

bool get_eval_input(const std::string &msg, const std::string &option = "");

std::string get_input(const std::string &msg);

} /* utils */

#endif /* end of include guard: STR_UTILS_H_MU7OQJD1 */
