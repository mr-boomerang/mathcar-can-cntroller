#include <stdlib.h>
#include <utils/miniflags.h>

namespace miniflags {

std::map<std::string, Flag *> flag_map;

bool Flag::set(std::string val) {
  // TODO: rest number type other than int
  if ('i' == type_) {  // int32
    // int val_i = utils::str_to_val<int>(val);
    int val_i = atoi(val.c_str());
    SET_VAL(int, val_i);
  } else if ('I' == type_) {  // int64
    // int val_i = utils::str_to_val<int>(val);
    int val_i = atoi(val.c_str());
    SET_VAL(int, val_i);
  } else if ('U' == type_) {  // uint64
    // int val_i = utils::str_to_val<int>(val);
    int val_i = atoi(val.c_str());
    SET_VAL(int, val_i);
  } else if ('d' == type_) {  // double
    // double val_d = utils::str_to_val<double>(val);
    double val_d = atof(val.c_str());
    SET_VAL(double, val_d);
  } else if ('s' == type_) {  // string
    SET_VAL(std::string, val);
  } else if ('b' == type_) {  // bool
    // if no was added then user probably wants it to be false
    bool val_b;
    if (val == "1" || val == "t" || val == "true" || val == "y" || val == "yes")
      val_b = true;
    else if (val == "0" || val == "f" || val == "false" || val == "n" ||
             val == "no")
      val_b = false;
    else {
      // i know the above else if is not required, it coud be just else, but
      // this it's easier to read and understand the idea.
      val_b = false;
    }
    SET_VAL(bool, val_b);
  } else {
    // invalid type.
    return false;
  }
  return true;
}

void miniflags_init(int argc, char **argv) {
  // No optimized
  for (int i = 1; i < argc; ++i) {
    if (argv[i][0] != '-')  // flags format should be --flag or -flag
      continue;

    std::string arg = argv[i];
    // if --flag then arg+2 o.w.(i.e. -flags) then arg+1
    arg.erase(0, (arg[1] == '-') ? 2 : 1);
    size_t pos = arg.find("=");
    std::string name = arg.substr(0, pos);
    std::string val;
    if (pos == std::string::npos) {  //"=" not found

      if (!flag_map[name]) {  // flag not found
        if (name[0] == 'n' && name[1] == 'o') {
          // since flag not found, check for bool case --noflag
          name.erase(0, 2);
          if (flag_map[name] && flag_map[name]->type() == 'b') {
            val =
                "0";  // since --noflag was used, we assume it should be false.
          }
        }
      }       // flag not found end
      else {  //"=" not found but flag found, case --flag val or --flag
        if (i == argc - 1 || argv[i + 1][0] == '-') {
          // If it's the last argument, i.e. i == argc-1, then check if the flag
          // is boolean, set true if boolean otherwise do nothing.
          // next arg starts with '-', chances are no value was specified for
          // the
          // flag, if it's bool set val to true or else let flag have default
          // val.
          if (flag_map[name]->type() ==
              'b') {    // only --flag was used, check if bool.
            val = "1";  // since --flag was used, we assume it should be false.
          }
          // else: do nothing, let flag have it's default value.
        } else {  // flag val
          val = argv[++i];
        }
      }
    } else {  // flag=val or flag= val, latter case is not considered.
      if (flag_map[name]) val = arg.substr(pos + 1, std::string::npos);
    }
    if (flag_map[name]) flag_map[name]->set(val);
  }
}

} /* miniflags */
