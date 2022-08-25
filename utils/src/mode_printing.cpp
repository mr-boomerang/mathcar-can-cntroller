#include <utils/mode_printing.h>

// define flags.
DEFINE_bool(logtocerr, false, "Output should to redirected to std::cerr");
DEFINE_bool(logtofile, false,
            "Output should to redirected to file "
            "<prog name>_<YYYY>_<MM>_<DD>_<HH>:<MM>");
DEFINE_bool(alsologtofile, false,
            "Output printed on scree and also redirected "
            "to file <prog name>_<YYYY>_<MM>_<DD>_<HH>:<MM>. This supersedes"
            " logtofile option.");
DEFINE_string(logprefix, "", "Prefix string for log file name.");
DEFINE_string(logdir, "/tmp", "Dir where log file should be created.");

DEFINE_int32(sl, sl::ERROR, "This flags sets the severity level");
DEFINE_string(sections, "", "Print statements for the specified sections.");

namespace print {

std::string prog_name = "log";
utils_boostd::shared_ptr<LogFile> lf;
bool *section_map = NULL;
int section_size = 0;

LogFile::LogFile() {
  PRINT_FUNC_ENTER;
  std::stringstream log_file_name;
  std::time_t t = std::time(0);
  std::tm tm = *std::localtime(&t);
  char lc = FLAGS_logdir[FLAGS_logdir.length() - 1];
  log_file_name << FLAGS_logdir
                << ((lc == PATH_SEPERATOR) ? ((char)0) : PATH_SEPERATOR)
                << (FLAGS_logprefix.empty() ? STR("")
                                            : (FLAGS_logprefix + STR("_")))
                << prog_name << "_" << (tm.tm_year + 1900)
                << PREFIXED_STR(tm.tm_mon + 1, '0', 2)
                << PREFIXED_STR(tm.tm_mday, '0', 2) << "_"
                << PREFIXED_STR(tm.tm_hour, '0', 2)
                << PREFIXED_STR(tm.tm_min, '0', 2)
                << PREFIXED_STR(tm.tm_sec, '0', 2) << ".log";
  log_fn_ = log_file_name.str();
  log_file_.open(log_fn_.c_str(), std::ofstream::out);
  PRINT_FUNC_EXIT;
}

void Logger::just_print_it() {
  bool to_screen = (FLAGS_alsologtofile || !FLAGS_logtofile);
  bool to_file = (FLAGS_logtofile || FLAGS_alsologtofile);

  if (to_screen) {
    if (FLAGS_logtocerr)
      std::cerr << ss_.str();
    else
      std::cout << ss_.str();
  }

  if (to_file) {
    std::string ss_str = ss_.str();
    if (ss_str.length()) {
      // TODO: other m's should not be deleted.
      size_t s_pos = 0, len;
      while (true) {
        s_pos = ss_str.find("\E[");
        if (s_pos == std::string::npos) break;
        len = ss_str.substr(s_pos).find("m") + 1;  // gives before m.
        if (len == std::string::npos) break;
        ss_str.erase(s_pos, len);
      }
      (*lf)() << ss_str;
    }
  }
}

void init(int *argc, char ***argv, bool remove_flags) {
  prog_name = *argv[0];
  prog_name =
      prog_name.substr(prog_name.rfind(PATH_SEPERATOR) + 1, std::string::npos);
#ifndef UTILS_AVOID_GFLAGS
  // Just use gflags, see how easy it is then, with more functionalities
  gflags::ParseCommandLineFlags(argc, argv, remove_flags);
#else
  miniflags::miniflags_init(*argc, *argv);
#endif
  if (!lf && (FLAGS_logtofile || FLAGS_alsologtofile))
    lf = utils_boostd::make_shared<LogFile>();
  if (!FLAGS_sections.empty() && section_map == NULL) {
    try {
      PI$ "Looks like section is there." pendl;
      std::string sec;
      std::stringstream ss(FLAGS_sections);
      std::vector<int> secs;
      while (std::getline(ss, sec, ',')) {
        secs.push_back(atoi(sec.c_str()));
      }
      std::sort(secs.begin(), secs.end());
      section_size = secs.back() + 1;
      section_map = new bool[section_size];
      std::fill(section_map, section_map + section_size, false);
      // for (auto i : secs) {}
      for (std::vector<int>::iterator i = secs.begin(); i != secs.end(); ++i) {
        section_map[*i] = true;
      }
    } catch (std::exception &e) {
      DP$ ct_vpur(e.what()) pendl;
    }
  }
  PRINT_FUNC_EXIT;
}

} /* print */
