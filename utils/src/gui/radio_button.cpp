#include <utils/gui/radio_button.hpp>

std::map<std::string, RadioButton *> RadioButton::rb_map_;

int RadioButton::select(std::string name) {
  PI$ ct_vred(name) pcommas ct_vred(grp_name()) << ", alright" << pendo;
  try {
    if (!init_done_) {
      PF$(1)
      "Something weird happended, init_done_ should not be false." << pendo;
    }
    if (name.empty()) {
      return RET_RADIO_BUTTON_NAME_EMPTY;
    } else {
      if (name.find(grp_name()) == std::string::npos)
        name = grp_name() + "." + name;
      if (!exists(name)) {
        PE$ "Non-existent value " << ct_ylw(name) << std::endl;
        return RET_NON_EXISTENT_RADIO_BUTTON_NAME;
      }
      PI$ ct_vred(name) pcommas ct_vred(selected_)
              pcommas ct_vred(selected_.size())
          << pendo;
      if (selected_.size()) { // Something is selected.
        *values_[selected_] = false;
      }
      *values_[name] = true;
      selected_ = name;
    }
  } catch (std::exception &e) {
    PE$ "Couldn't set the requested radio button." << std::endl;
    PN$ ct_ylw("except: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int RadioButton::init(const std::string &typ, const std::string &values_csv,
                      const std::string &default_sel) {
  try {
    pangolin::Var<std::string> heading_btn(grp_name_, "");
    vec_str vals = utils::split_str<std::string>(values_csv, ",");
    if (vals.size() == 1) {
      PE$ "No ',' in comma seperated values for enum values. "
          << ct_vdylw(values_csv) << pendo;
      return RET_CHECKBOXES_CANNOT_BE_ADDED;
    }
    PI$ "Adding " << vals.size() << " checkboxes." << std::endl;
    for (size_t i = 0; i < vals.size(); i++) {
      utils::trim(vals[i]);
      PI$ "adding " << ct_grn(vals[i]) << std::endl;
      std::string cb_name = grp_name_ + "." + vals[i];
      if (exists(cb_name)) {
        PE$ "Checkbox with name " << ct_dylw(cb_name) << " already exists"
                                  << pendo;
      }
      if (vals[i] == default_sel) {
        values_[cb_name] = new pangolin::Var<bool>(cb_name, true, true);
        selected_ = cb_name;
      } else
        values_[cb_name] = new pangolin::Var<bool>(cb_name, false, true);
    }
    init_done_ = true;
  } catch (std::exception e) {
    PE$ "Couldn't initialize the radio button." << std::endl;
    PN$ ct_ylw("except: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int RadioButton::add(const std::string &group_name, const std::string type,
                     const std::string &vals, const std::string &def_val) {
  try {
    if (is_existing(group_name)) {
      PE$ "RadioButton with name " << ct_dylw(group_name) << " already exist."
                                   << pendo;
      return RET_RADIO_BUTTON_ALREADY_EXISTS;
    }
    rb_map_[group_name] = new RadioButton(group_name, type, vals, def_val);
  } catch (std::exception e) {
    PE$ "Couldn't add the radio button." << std::endl;
    PN$ ct_ylw("except: ") << e.what() << std::endl;
    return RET_EXCEPTION_BOOL;
  }
}

RadioButton *RadioButton::get(const std::string &group_name) {
  try {
    if (!is_existing(group_name)) {
      PE$ "RadioButton with name " << ct_dylw(group_name) << " doesn't exist."
                                   << pendo;
      return NULL;
    }
    return rb_map_[group_name];
  } catch (std::exception e) {
    PE$ "Can't retrieve the radio button." << std::endl;
    PN$ ct_ylw("except: ") << e.what() << std::endl;
    return NULL;
  }
}

bool RadioButton::is_existing(const std::string &group_name) {
  return (rb_map_.find(group_name) != rb_map_.end());
}

int RadioButton::remove(const std::string &group_name) {
  try {
    rb_map_.erase(rb_map_.find(group_name));
  } catch (std::exception e) {
    PE$ "Can't remove the radio button." << std::endl;
    PN$ ct_ylw("except: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}
