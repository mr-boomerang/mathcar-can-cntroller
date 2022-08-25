#pragma once

// Utils to check files/dir on file system
// It's aimed to be agnostic to OS, but have not been tested with others.

#ifdef _WIN32
// TODO: which one to use?
#include <direct.h>
#include <windows.h>

#define PATH_SEPERATOR '\\' // i.e. backslash

#elif defined __linux__ || defined __APPLE__

#include <glob.h>     //for files from regex.
#include <stdlib.h>   //for getenv in tilde expansion.
#include <sys/stat.h> //stat for dir.
#include <unistd.h>   //for access method in file_exist.

#define PATH_SEPERATOR '/' // i.e. forwardslash
#else
#define PATH_SEPERATOR '/' // i.e. forwardslash
#endif

#include "error_code.h"
#include "mode_printing.h"
#include "str_utils.h"
#include <string>
#include <vector>

namespace utils {

enum FileAccessType {
  Exist = F_OK, // 0
  Exec = X_OK,  // 1
  Write = W_OK, // 2
  Read = R_OK,  // 4
};

inline std::string tilde_expansion(const std::string &path) {
  std::string p = path;
#ifdef __linux__
  if (p.length() && p[0] == '~') {
    p.replace(0, 1, getenv("HOME"));
  }
#endif
  return p;
}

inline std::string tilde_expansion(std::string &path) {
#ifdef __linux__
  if (path.length() && path[0] == '~') {
    path.replace(0, 1, getenv("HOME"));
  }
#endif
  return path;
}

// tilde won't be interpreted.
inline bool file_exist(const char *fn) {
#ifdef __linux__
  if (fn[0] == '~') {
    PE$ "First char is tilde, might not give correct result. " << pendo;
  }
  return (access(fn, F_OK) != -1);
#elif defined _WIN32
  // TODO: someday check it on windows.
  if (INVALID_FILE_ATTRIBUTES == GetFileAttributes(fn) &&
      GetLastError() == ERROR_FILE_NOT_FOUND)
    return RET_SUCCESS_BOOL;
  else
    return false;
#else
  return false;
#endif
}

inline bool file_exist(const std::string &fn) {
  return file_exist(tilde_expansion(fn).c_str());
}

inline bool file_accessible(const char *fn, FileAccessType t) {
#ifdef __linux__
  // struct stat buf;
  // return (stat(fn, &buf) == 0);
  if (fn[0] == '~') {
    PE$ "First char is tilde, might not give correct result. " << pendo;
  }
  return (access(fn, t) == 0);
#else
  // TODO: someday check it on windows.
  return (0xffffffff != GetFileAttributes(fn));
#endif
}

inline bool file_accessible(const std::string &fn, FileAccessType t) {
  return file_accessible(tilde_expansion(fn).c_str(), t);
}

inline bool is_dir(const char *fn) {
#ifdef __linux__
  if (fn[0] == '~') {
    PE$ "First char is tilde, might not give correct result. " << pendo;
  }
  struct stat buf;
  return (stat(fn, &buf) == 0) ? S_ISDIR(buf.st_mode) : false;
#else
  // TODO: someday check it on windows.
  DWORD ret = GetFileAttributes(fn); // from winbase.h
  if (ret & FILE_ATTRIBUTE_DIRECTORY)
    return RET_SUCCESS_BOOL;
  else
    return false;
#endif
}

inline bool is_dir(const std::string &fn) {
  return is_dir(tilde_expansion(fn).c_str());
}

inline std::string remove_trailing_slash(const std::string &path) {
  if (path.empty())
    return path;
  size_t len = path.length() - 1;
  return (path.at(len) == PATH_SEPERATOR) ? path.substr(0, len) : path;
}

inline std::string filename_without_extension(const std::string &file_name) {
  return file_name.substr(0, file_name.rfind("."));
}

inline std::string basename(const std::string &path) {
  if (path.empty())
    return path;
  std::string p = remove_trailing_slash(tilde_expansion(path));
  return p.substr(p.rfind(PATH_SEPERATOR) + 1, std::string::npos);
}

inline std::string basename(const char *path) {
  std::string s(path);
  return basename(s);
}

inline std::string dir_path(const std::string &path) {
  if (path.empty())
    return path;
  std::string p = remove_trailing_slash(tilde_expansion(path));
  std::size_t pos = p.rfind(PATH_SEPERATOR);
  if (pos == std::string::npos) // no path seperator, that means current dir.
    return ".";
  else
    return p.substr(0, pos);
}

std::string get_overwrite_confirmation(const std::string &filename,
                                       bool alternate = false);

vec_str filelist_from_regex(const std::string &str, size_t start = 0,
                            size_t end = 0, bool only_dir = false,
                            bool period = false);

vec_str read_file(const std::string &file_name);

} /* utils */
