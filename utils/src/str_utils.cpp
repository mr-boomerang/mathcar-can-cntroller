#include <utils/mode_printing.h>
#include <utils/str_utils.h>
// These functions are kept simple, no error checking or validation, it's
// assumed
// user will take care of it.

typedef std::vector<std::string> vec_str;

namespace utils {

// * split string in two at nth(idx from 1) occurance of delim, if n_occur is
//   greater than number of delims actually present then str is left as it is
// * -ve n_occur will search from back
// * if delim is not found then returned string will be same as input
//   string(str)
std::string split_str_in_two(std::string &str, const std::string &delim,
                             int n_occur) {
  int delim_len = delim.length();
  size_t pos, p_pos = (n_occur > 0) ? -delim_len : std::string::npos;
  std::string ret;

  for (int i = 0; i < std::abs(n_occur); ++i) {
    pos = (n_occur > 0) ? str.find(delim, p_pos + delim_len)
                        : str.rfind(delim, p_pos - delim_len);
    p_pos = pos;
    if (pos == std::string::npos) break;
  }
  if (pos != std::string::npos) {
    ret = str.substr(0, pos);
    str.erase(0, pos + delim_len);
  } else {
    ret = str;
  }
  return ret;
}

int split_str_in_two(const std::string &str, const std::string &delim,
                     std::string *first, std::string *second, int n_occur) {
  int delim_len = delim.length();
  size_t pos, p_pos = (n_occur > 0) ? -delim_len : std::string::npos;

  for (int i = 0; i < std::abs(n_occur); ++i) {
    pos = (n_occur > 0) ? str.find(delim, p_pos + delim_len)
                        : str.rfind(delim, p_pos - delim_len);
    p_pos = pos;
    if (pos == std::string::npos) break;
  }
  if (pos != std::string::npos) {
    *first = str.substr(0, pos);
    *second = str.substr(pos + delim_len);
    return RET_SUCCESS_INT;
  }
  return RET_ERROR_SPLITTING_STRING;
}

int split_str_in_two(const std::string &str, const std::string &delim,
                     utils_boostd::shared_ptr<std::string> &first,
                     utils_boostd::shared_ptr<std::string> &second,
                     int n_occur) {
  int delim_len = delim.length();
  size_t pos, p_pos = (n_occur > 0) ? -delim_len : std::string::npos;

  for (int i = 0; i < std::abs(n_occur); ++i) {
    pos = (n_occur > 0) ? str.find(delim, p_pos + delim_len)
                        : str.rfind(delim, p_pos - delim_len);
    p_pos = pos;
    if (pos == std::string::npos) break;
  }
  if (pos != std::string::npos) {
    if (first)
      *first.get() = str.substr(0, pos);
    else {
      first = utils_boostd::make_shared<std::string>(str.substr(0, pos));
    }
    if (second)
      *second.get() = str.substr(pos + delim_len);
    else
      second =
          utils_boostd::make_shared<std::string>(str.substr(pos + delim_len));
    return RET_SUCCESS_INT;
  }
  return RET_ERROR_SPLITTING_STRING;
}

int replace_inplace(std::string &str, const std::string &find,
                    const std::string &with, int n_replace) {
  if (!n_replace) return RET_SUCCESS_INT;
  int count;
  str = replace(str, find, with, n_replace, &count);
  return count;
}

std::string replace(const std::string &str, const std::string &find,
                    const std::string &with, int n_replace,
                    int *count_replace) {
  if (!n_replace) return "";
  std::string ret = str;
  int count = 0, find_len = find.length(), with_len = with.length();
  std::size_t pos = 0;
  while ((pos = ret.find(find, pos)) != std::string::npos &&
         (n_replace == -1 || count < n_replace)) {
    ret.replace(pos, find_len, with);
    pos = pos + with_len;
    count++;
  }
  if (count_replace != NULL) *count_replace = count;
  return ret;
}

bool get_eval_input(const std::string &msg, const std::string &option) {
  std::string ans;

  PN$ msg << " " << std::flush;
  std::cin >> ans;

  trim(ans);
  if (ans == option) return RET_SUCCESS_BOOL;
  return false;
}

std::string get_input(const std::string &msg) {
  std::string ans;

  PN$ msg << " " << std::flush;
  std::cin >> ans;

  return trim(ans);
}

} /* utils */
