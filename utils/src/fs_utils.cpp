#include <utils/fs_utils.h>

#include <fstream>

namespace utils {

std::string get_overwrite_confirmation(const std::string &filename,
                                       bool alternate) {
  PRINT_FUNC_ENTER;
  std::string ret;
  std::string fn = tilde_expansion(filename);
  if (file_exist(fn)) {
    bool not_overwrite;
    std::string alt = fn;
    int count = 0;
    std::stringstream msg;
    if (!alternate)
      msg << "answering " << ct_red("no") << " would " << ct_red("quit")
          << " the program. ";
    msg << "Overwrite (" << ct_dred("yes") << "/" << ct_grn("no") << ")?";
    // Run the loop until the user enters useable file name or asks to quit.
    do {
      // PN$ "File " << ct_dblu(alt) << (count?" also":"") << " exists, ";
      not_overwrite = !get_eval_input(msg.str(), "yes");
      if (not_overwrite) {
        if (alternate) {
          cPN$(!count) "\nWhile providing alt. name, if a "
              << ct_dblu("'" << PATH_SEPERATOR << "' exists") << " then ans"
              << " is considered a " << ct_dblu("path")
              << "(relative/absolute)."
                 " If " << ct_dylw("no '" << PATH_SEPERATOR << "'")
              << " exists, it's assumed only " << ct_dylw("basename")
              << " is being changed, i.e. new file is created in "
              << ct_dcyn(dir_path(fn)) << pendo;
          alt = get_input("Provide alternate file name(quit to exit): ");
          trim(alt);
          if (alt == "quit") break;
          if (alt.find(PATH_SEPERATOR) ==
              std::string::npos) {  // same path, diff file
            alt = dir_path(fn) + PATH_SEPERATOR + alt;
          } else {
            tilde_expansion(alt);
          }
        } else {
          PN$ ct_grn("Not overwriting existing file and exiting.") << pendo;
          exit(1);
        }
      } else {
        PN$ ct_red("Overwriting " << alt) << pendo;
      }
      count++;
    } while (not_overwrite && file_accessible(alt, Write));

    if (not_overwrite) {
      if (alt == "quit")
        return std::string("");
      else {
        PN$ "Alternately writing to " << ct_dcyn(alt) << pendo;
        return alt;
      }
    }
  }
  PRINT_FUNC_EXIT;
  return filename;
}

vec_str filelist_from_regex(const std::string &str, size_t start, size_t end,
                            bool only_dir, bool period) {
  PRINT_FUNC_ENTER;
  vec_str file_list;
  glob_t dont_know_why_its_called_glob;
  int glob_ret;

  std::string glob_ip_path = remove_trailing_slash(str);
  if (is_dir(glob_ip_path)) glob_ip_path += "/*";

  SP$(950) ct_vylw(glob_ip_path) << pendo;
  int flag = GLOB_BRACE | GLOB_TILDE;
  if (only_dir) flag |= GLOB_ONLYDIR;
  if (period) flag |= GLOB_PERIOD;
  glob_ret =
      glob(glob_ip_path.c_str(), flag, 0, &dont_know_why_its_called_glob);
  SP$(950) "num of files = " << dont_know_why_its_called_glob.gl_pathc << pendo;
  if (glob_ret == GLOB_NOMATCH) {
    SP$(950) ct_red("No matches found.") << pendo;
    return file_list;  // file_list will be empty
  } else if (glob_ret == GLOB_ABORTED) {
    SP$(950) "Read error." << pendo;
    return file_list;  // file_list will be empty
  }  // One more error type GLOB_NOSPACE, but i doubt it will ever happen.
  else {
    unsigned int i;
    end = !end ? (dont_know_why_its_called_glob.gl_pathc)
               : (std::min(end, dont_know_why_its_called_glob.gl_pathc));
    for (i = start; i < end; ++i) {
      std::string bn = basename(dont_know_why_its_called_glob.gl_pathv[i]);
      if (bn == "." || bn == "..") continue;
      file_list.push_back(dont_know_why_its_called_glob.gl_pathv[i]);
    }
    globfree(&dont_know_why_its_called_glob);

    // This check is there in cvd(cvd has || we have &&), cvd probably uses ||
    // to handle the case where if dir is passed, but we take care of it by
    // chekcing if input is_dir, so using && doesn't seem to hurt,
    // but don't know why the condition is there in the first place,
    // hopefully will catch some illegal cases.
    if (file_list.size() == 1 && file_list[0] == "") {
      SP$(950) "Blank or only one filename." << pendo;
      return file_list;  // file_list will be empty
    }
    PI$ PRINT_SUCCESS << file_list.size() << " files located for "
                      << glob_ip_path << pendo;
    PRINT_FUNC_EXIT;
    return file_list;  // Success would return from here.
  }
  PRINT_FUNC_EXIT;
  return file_list;  // Code should never reach here, just for safety.
}

vec_str read_file(const std::string &file_name) {
  vec_str retval;
  try {
    std::ifstream ifs(file_name.c_str());
    std::string line;

    if (!ifs.is_open()) {
      PF$(100) "Couldn't open the file \"" << ct_red(file_name) << "\"." pendl;
      return vec_str();  // Empty vector.
    }

    while (std::getline(ifs, line)) {
      retval.push_back(line);
    }
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return vec_str();
  }
  return retval;
}

} /* utils */
