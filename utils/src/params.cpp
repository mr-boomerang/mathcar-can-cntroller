#include <stdio.h>
#include <utils/params.hpp>
#include <utils/stl_utils.h>

bool get_bool_val(const std::string &val_s) {
  if (val_s == "true" || val_s == "True" || val_s == "TRUE" || val_s == "t" ||
      val_s == "T" || val_s == "on" || val_s == "On" || val_s == "ON" ||
      val_s == "1") {
    return true;
  } else if (val_s == "false" || val_s == "False" || val_s == "FALSE" ||
             val_s == "f" || val_s == "F" || val_s == "off" || val_s == "Off" ||
             val_s == "OFF" || val_s == "0") {
    return false;
  }
  return false;
}

namespace PENS {
std::map<std::string, EnumBase *> _type_enum_map;
} // namespace PENS

#define ELSE_IF_ARITH(name, T)                                              \
  else if (type == name) {                                                  \
    std::vector<T> vals = utils::split_str<T>(s_arr_val, kParamSeperator);  \
    SP$(sn) ct_vblu(vals) pendl;                                            \
    CHECK_RET_VAL_INFO(set_array(var_name, vals.data(), vals.size()), 1,    \
                       ct_vblu(var_name));                                  \
    if (sprops.size() == 3) {                                               \
      T min = utils::str_to_val<T>(sprops[0]);                              \
      T max = utils::str_to_val<T>(sprops[1]);                              \
      T interval = utils::str_to_val<T>(sprops[2]);                         \
      CHECK_RET_VAL_INFO(set_properties(var_name, min, max, interval), 2,   \
                         ct_vblu(var_name));                                \
    } else if (sprops.size()) {                                             \
      DP$ ct_vred(var_name) << " has properties, but has " << sprops.size() \
                            << " properties, not 3 properties" pendl;       \
    }                                                                       \
  }

// Can dump all type which have << operator defined for ostream or stringstream.
void Params::dump(const std::string &fn) {
  std::string file_name = utils::get_overwrite_confirmation(fn, true);
  std::ofstream ofs(file_name.c_str());
  // for (auto param : param_map) { }
  for (std::map<std::string, ParamT *>::iterator param = param_map.begin();
       param != param_map.end(); ++param) {
    PI$ ct_vcyn(param->first) pcommas ct_vcyn(param->second->has_ostream_op)
        pendl;
    if (param->second->has_ostream_op) {
      ofs << param->first << ": ";
      param->second->to_stream(param->second, ofs);
      ofs << "\n";
    }
  }
}

// Can load some type of parameters, as of now supported types are:
// int, short, long, long long, unsigned, unsigned short, unsigned long,
// unsigned long long, char, signed char, unsigned char, float, double,
// long double, std::string, bool
// TODO: extend support to custom type.
int Params::load(const std::string &file_name) {
  PI$ "Loading params from " << file_name << std::endl;
  std::ifstream ifs(file_name.c_str());
  std::string line;

  if (!ifs.is_open()) {
    PE$ "Couldn't open the file " << ct_red(file_name) << std::endl;
    return RET_CANNOT_OPEN_FILE_INT;
  }

  while (std::getline(ifs, line)) {
    std::string var_name, var_type, etc = "";
    utils::trim(line);
    if (line.empty() || line[0] == '#') continue;
    var_name = utils::split_str_in_two(line, ":");

    utils::trim(line);
    utils::erase_first_nchar(line);  //'['
    if (utils::last_char(line) == kParamEnumDelim[1]) {
      utils::erase_last_nchar(line);                  //')'
      std::string val_enum_break(2, kParamDelim[1]);  //"]]"
      val_enum_break[1] = kParamEnumDelim[0];         //"]("
      etc = utils::split_str_in_two(line, val_enum_break);
      std::swap(line, etc);
    } else if (utils::last_char(line) == kParamDelim[1]) {
      utils::erase_last_nchar(line);  //']'
    } else {
      PE$ "Error at line \"" << line << "\" while loading " << file_name pendl;
      return RET_ERROR_LOADING_FILE;
    }

    var_type = utils::split_str_in_two(line, kParamSeperator);
    utils::trim(line);
    utils::trim(var_type);
    CHECK_RET(set_param_from_str(var_name, var_type,
                                 line)); // etc not useful to set var.
  }
  DP$ ct_dylw(file_name) << " loaded" << std::endl;
  return RET_SUCCESS_INT;
}

std::string Params::list_full(const std::string &delim) {
  std::stringstream ret;
  std::string dlm = delim.empty() ? ";" : delim;
  for (std::map<std::string, ParamT *>::iterator param = param_map.begin();
       param != param_map.end();) {
    ret << param->first << ":";
    param->second->to_stream(param->second, ret);
    if (++param != param_map.end()) ret << dlm;
  }
  return ret.str();
}

std::string Params::list_names(const std::string &delim) {
  std::string ret;
  std::string dlm = delim.empty() ? ";" : delim;
  for (std::map<std::string, ParamT *>::iterator param = param_map.begin();
       param != param_map.end();) {
    ret = ret + param->first;
    if (++param != param_map.end()) ret += dlm;
  }
  return ret;
}

std::string Params::param_info_full(const std::string &name) {
  if (param_map.find(name) == param_map.end()) {
    return "";
  }
  std::stringstream ret;
  ret << name << ":";
  param_map[name]->to_stream(param_map[name], ret);
  return ret.str();
}

std::string Params::get_string(const std::string &var_name) {
  if (param_map.find(var_name) == param_map.end()) {
    return "";
  }
  std::string val;
  param_map[var_name]->get_string(param_map[var_name], &val);
  return val;
}

bool Params::has_changed(const std::string &var_name) {
  bool check_var;
  // return param_map[var_name]->has_changed();
  CHECK_RET_TO_VAR(param_map[var_name]->has_changed(), check_var);
  return check_var;
}

int Params::set_param_from_str(const std::string &var_name,
                               const std::string &type,
                               const std::string &sval) {
  const int sn = 110;  // section number.

  PRINT_SEPERATOR_SEC(sn);
  SP$(sn) ct_vcyn(var_name) pendl;
  SP$(sn) ct_vcyn(type) pendl;
  SP$(sn) ct_vcyn(sval) pendl;

  // check if array or not.
  size_t pc = sval.find(kParamArrayDelim[1]);
  bool array = (sval[0] == kParamArrayDelim[0] && pc != std::string::npos);
  SP$(sn) ct_vblu(array) pendl;
  std::string s_arr_val, s_aux;
  std::vector<std::string> sprops;
  if (array) {
    CHECK_RET(utils::split_str_in_two(sval, std::string(1, kParamArrayDelim[1]),
                                      &s_arr_val, &s_aux));
    utils::trim(s_arr_val);
    utils::erase_first_nchar(s_arr_val);  // remove '('
    utils::trim(s_arr_val);
    SP$(sn) ct_vblu(s_arr_val) pendl;
    SP$(sn) ct_vblu(s_aux) pendl;

    if (s_aux.size()) {
      utils::trim(s_aux);
      utils::erase_first_nchar(s_aux);  // remove ","
      utils::trim(s_aux);
      sprops = utils::split_str<std::string>(s_aux, kParamSeperator);
    }
  } else {
    CHECK_WARN(
        utils::split_str_in_two(sval, kParamSeperator, &s_arr_val, &s_aux));
    utils::trim(s_arr_val);
    utils::trim(s_aux);
    if (s_arr_val.empty()) s_arr_val = sval;
    if (s_aux.size())
      sprops = utils::split_str<std::string>(s_aux, kParamSeperator);
    SP$(sn) ct_vblu(s_arr_val) pendl;
    SP$(sn) ct_vblu(s_aux) pendl;
  }
  SP$(sn) ct_vblu(sprops) pendl;

  for (std::string elem : sprops) {
    utils::trim(elem);
  }

  if (type == "bool") {
    std::vector<std::string> svals =
        utils::split_str<std::string>(s_arr_val, kParamSeperator);
    SP$(sn) ct_vblu(svals) pendl;
    bool *vals = new bool[svals.size()];
    for (unsigned int i = 0; i < svals.size(); ++i) {
      vals[i] = get_bool_val(svals[i]);
    }
    bool cb_btn =
        (sprops.size() == 1) ? utils::str_to_val<bool>(sprops[0]) : false;
    CHECK_RET_VAL_INFO(set_array(var_name, vals, svals.size(), cb_btn), 1,
                       ct_vblu(var_name));
    // TODO: set props
  } else if (type == "std::string") {
    if (sprops.size()) {
      PE$ "properties found for " << ct_vylw(var_name) pcommas ct_vylw(type)
                                  << " which shouldn't be the case." pendl;
      return RET_INVALID_TYPE;
    }
    std::vector<std::string> svals =
        utils::split_str<std::string>(s_arr_val, kParamSeperator);
    CHECK_RET_VAL_INFO(set_array(var_name, svals.data(), svals.size()), 1,
                       ct_vblu(var_name));
  } else if (type == "char const*") {  // type for value in "" is char const*
    if (sprops.size()) {
      PE$ "properties found for " << ct_vylw(var_name) pcommas ct_vylw(type)
                                  << " which shouldn't be the case." pendl;
      return RET_INVALID_TYPE;
    }
    std::vector<std::string> svals =
        utils::split_str<std::string>(s_arr_val, kParamSeperator);
    SP$(sn) ct_vblu(svals) pendl;
    std::vector<const char *> vals;
    SP$(sn) ct_vblu(vals) pendl;
    for (unsigned int i = 0; i < vals.size(); ++i) {
      vals.push_back(svals[i].c_str());
    }
    CHECK_RET_VAL_INFO(set_array(var_name, vals.data(), vals.size()), 1,
                       ct_vblu(var_name));
  }
  ELSE_IF_ARITH("int", int)
  ELSE_IF_ARITH("short", short)
  ELSE_IF_ARITH("long", long)
  ELSE_IF_ARITH("long long", long long)
  ELSE_IF_ARITH("unsigned", unsigned)
  ELSE_IF_ARITH("unsigned short", unsigned short)
  ELSE_IF_ARITH("unsigned long", unsigned long)
  ELSE_IF_ARITH("unsigned long long", unsigned long long)
  ELSE_IF_ARITH("char", char)
  ELSE_IF_ARITH("signed char", signed char)
  ELSE_IF_ARITH("unsigned char", unsigned char)
  ELSE_IF_ARITH("float", float)
  ELSE_IF_ARITH("double", double)
  ELSE_IF_ARITH("long double", long double)
  else if (type.find(PENS_STR) != std::string::npos) {
    if (PENS::_type_enum_map.find(type) != PENS::_type_enum_map.end()) {
      CHECK_RET_INFO(PENS::_type_enum_map[type]->add_derived_as_param(
                         var_name, sval, params_utils),
                     ct_vblu(var_name));
    } else {
      PE$ "type " << ct_dylw(type) << " is not present in _type_enum_map."
                  << pendo;
      return RET_UNDEFINED_TYPE;
    }
  }
  else {
    PE$ "Extra support is needed for " << ct_vylw(type)
                                       << ", which is the requested type for "
                                       << ct_vylw(var_name) pendl;
    return RET_NEEDED_EXTRA_SUPPORT;
  }

  PRINT_SEPERATOR_SEC(sn);
  return RET_SUCCESS_INT;
}

void Params::dummy() {}

#ifdef BUILT_WITH_ROS

bool Params::start_ros_service() {
  return ParamROS::instance().start_ros_service();
}

bool ParamROS::info_params(utils::ParamInfo::Request &req,
                           utils::ParamInfo::Response &res) {
  try {
    if (req.name == "") {
      res.info = req.full_info ? params_utils.list_full(req.delim)
                               : params_utils.list_names(req.delim);
    } else
      res.info = params_utils.param_info_full(req.name);
  } catch (std::exception &e) {
    DP$ ct_ylw("Except: ") << ct_vpur(e.what()) pendl;
    DP$ ct_vpur(req.name) pcommas ct_vpur(req.delim)
        pcommas req.full_info pendl;
    return RET_EXCEPTION_BOOL;
  }
  return RET_SUCCESS_BOOL;
}

bool ParamROS::set_param(utils::ParamSet::Request &req,
                         utils::ParamSet::Response &res) {
  std::string type = utils::demangle_type(params_utils.get_type(req.name));
  if (type.empty()) {
    PE$ "No type found." << std::endl;
    res.success = false;
    return RET_NO_TYPE_FOUND;
  } else {
    int ret = params_utils.set_param_from_str(req.name, type, req.value);
    res.success = !ret;
    cPI$(!ret) ct_vpur(params_utils.get_string(req.name)) << std::endl;
    // temp
    res.value = params_utils.get_string(req.name);
    if (ret) {
      PE$ "setting of param from str failed with value = " << ct_dblu(ret)
                                                           << std::endl;
      res.value = params_utils.get_string(req.name);
    }
  }
  return RET_SUCCESS_BOOL;
}

bool ParamROS::get_param(utils::ParamGet::Request &req,
                         utils::ParamGet::Response &res) {
  std::string type = utils::demangle_type(params_utils.get_type(req.name));
  if (type.empty()) {
    PE$ "No type found." << std::endl;
    return RET_NO_TYPE_FOUND;
  } else {
    res.value = params_utils.get_string(req.name);
    res.type = type;
    PN$ "res.value:	 " << res.value << "\n"
                           << "res.type:		" << res.type << pendo;
  }
  return RET_SUCCESS_BOOL;
}

#endif
