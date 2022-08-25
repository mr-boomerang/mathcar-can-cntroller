#pragma once

#include <pangolin/pangolin.h>

#include <utils/str_utils.h>

#include <utils/error_code.h>

#include <utils/mode_printing.h>

class RadioButton {
public:
  std::string selected() const { return selected_; }
  std::string grp_name() const { return grp_name_; }
  // bool init_done() const { return init_done_; }
  // void init_done(bool id) { init_done_ = id; }

  int select(std::string name);

  bool exists(const std::string &name) {
    return (values_.find(name) != values_.end());
  }

  int operator=(const RadioButton &rb) {
    grp_name_ = rb.grp_name_;
    values_ = rb.values_;
    init_done_ = rb.init_done_;
    select(rb.selected());
    return RET_SUCCESS_INT;
  }

  int operator=(const std::string &sel) {
    select(sel);
    return RET_SUCCESS_INT;
  }

  std::string type() { return type_; }

  static int add(const std::string &group_name, const std::string type,
                 const std::string &vals, const std::string &def_val);

  static RadioButton *get(const std::string &group_name);

  static bool is_existing(const std::string &group_name);

  static int remove(const std::string &group_name);

private:
  /* data */
  bool init_done_;
  std::map<std::string, pangolin::Var<bool> *> values_;
  std::string grp_name_;
  std::string selected_;
  std::string type_;

  /* static data */
  static std::map<std::string, RadioButton *> rb_map_;

  /* constructor */
  RadioButton(const std::string &grp_name, const std::string &typ,
              const std::string &values_csv,
              const std::string &default_sel = "")
      : init_done_(false), grp_name_(grp_name) {
    init(typ, values_csv, default_sel);
  type_=typ;
  }

  int init(const std::string &typ, const std::string &values_csv,
           const std::string &default_sel = "");
};

#if 0
std::ostream& operator<< (std::ostream& os, const RadioButton&){
  return os;
}

std::istream& operator>> (std::istream& is, RadioButton& rb){
  std::string val;
  is >> val;
  std::cout << "val = " << val << std::endl;
  if(val.size()) {
    rb.set(val);
  }
  return is;
}
#endif
