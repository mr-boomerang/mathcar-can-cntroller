#include <utils/gui/view_graph.hpp>

#include <utils/fs_utils.h>

int ViewGraph::init(const std::string &name, int w, int h, int *argc,
                    char ***argv) {

  if (argc != NULL && argv != NULL)
    print::init(argc, argv);

  pangolin::CreateWindowAndBind(name, w, h);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  gh_ = new GuiHandler(" ");
  pangolin::DisplayBase().SetHandler(gh_);

  Window.info[0] = -2.0f;
  Window.info[1] = -2.0f;
  Window.info[2] = -2.0f;
  Window.info[3] = -2.0f;

  Window.pos[0] = -1;
  Window.pos[1] = -1;

  // vn_[""] = &pangolin::DisplayBase();
  return RET_SUCCESS_INT;
}

int ViewGraph::add_2d_view(const std::string &name, const std::string &parent) {
  try {
    if (view_exists(name)) {
      PE$ "View " << ct_dred(name) << " already exists." pendl;
      return RET_VIEW_ALREADY_EXISTS;
    }
    if (vn_[parent]->type() != Container)
      return RET_INVALID_CONTAINER;
    vn_[name] = new ViewNode(name, TwoD, vn_[parent]);
    vv_[&vn_[name]->view()] = vn_[name];
    parent_name_[name] = parent;
    gh_->move_float_view_last();
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int ViewGraph::add_3d_view(const std::string &name, const std::string &parent) {
  try {
    if (view_exists(name)) {
      PE$ "View " << ct_dred(name) << " already exists." pendl;
      return RET_VIEW_ALREADY_EXISTS;
    }
    if (vn_[parent]->type() != Container)
      return RET_INVALID_CONTAINER;
    vn_[name] = new ViewNode(name, ThreeD, vn_[parent]);
    vv_[&vn_[name]->view()] = vn_[name];
    parent_name_[name] = parent;
    gh_->move_float_view_last();
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int ViewGraph::add_container(const std::string &name, pangolin::Layout layout,
                             const std::string &parent) {
  try {
    if (view_exists(name)) {
      PE$ "View " << ct_dred(name) << " already exists." pendl;
      return RET_VIEW_ALREADY_EXISTS;
    }
    vn_[name] = new ViewNode(name, Container, vn_[parent]);
    vv_[&vn_[name]->view()] = vn_[name];
    vn_[name]->layout(layout);
    parent_name_[name] = parent;
    gh_->move_float_view_last();
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }

  return RET_SUCCESS_INT;
}

int ViewGraph::add_container(const std::string &name, pangolin::Attach bot,
                             pangolin::Attach top, pangolin::Attach left,
                             pangolin::Attach right, pangolin::Layout layout,
                             const std::string &parent) {
  // bool retval = true;
  int retval = 0;
  try {
    retval = add_container(name, layout, parent);
    vn_[name]->bounds(bot, top, left, right);
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }

  return RET_SUCCESS_INT;
}

int ViewGraph::view_exists(const std::string &name) {
  // return (vn_.find(name) != vn_.end());
  if (vn_.find(name) != vn_.end())
    return RET_VIEW_DOES_NOT_EXIST;
  else
    return RET_SUCCESS_INT;
}

#if 1
pangolin::View &ViewGraph::get_view(const std::string &name) {
  if (!view_exists(name))
    PF$(1) "View " << name << " doesn't exist" << pendo;
  return vn_[name]->view();
}
#endif

ViewNode *ViewGraph::get_view_node(const std::string &name, std::string fn,
                                   int line) {
  int view_exist = view_exists(name);
  if (!view_exist) {
    DP$ "View " << ct_dred(name) << " doesn't exist" << pendo;
    cDP$(line != -1) "(Caller: " << utils::basename(fn) pcolon line << ")"
                                 << pendo;
    return NULL;
  }
  return vn_[name];
}

ViewNode *ViewGraph::get_view_node(pangolin::View *v, std::string fn,
                                   int line) {
  if (vv_.find(v) == vv_.end()) {
    PW$ "View " << ct_dred(v) << " doesn't exist" << pendo;
    cPW$(line != -1) "(Caller: " << utils::basename(fn) pcolon line << ")"
                                 << pendo;
    return NULL;
  }
  return vv_[v];
}

int ViewGraph::read_layout(const std::string &layout_fn) {
  try {
    vec_str lines = utils::read_file(layout_fn);
    std::string view_name, view_name_etc, view_type;
    std::string parent = "";
    std::string layout, bounds, lay_bou;
    pangolin::Layout lay;
    pangolin::Attach b[4];

    for (std::string line : lines) {
      utils::trim(line);

      if (line.empty() || line[0] == '#')
        continue;

      parent = "";
      layout = "";
      bounds = "";
      lay_bou = "";
      lay = pangolin::LayoutEqual;

      if (utils::split_str_in_two(line, ":", &view_name, &view_name_etc)) {
        PF$(1) "Invalid format (missing ':' in line = " << line << ")" << pendo;
      }
      utils::trim(view_name);
      utils::trim(view_name_etc);
      SP$(70) ct_vred(view_name) pcommas ct_vred(view_name_etc) pendl;
      if (view_name.empty())
        PF$(1) "Empty view name in line = \"" << line << "\"" << pendo;

      if (view_name_etc.empty()) {
        PF$(1)
        "No information provided for view \"" << view_name << "\"" << pendo;
      }

      view_type = utils::split_str_in_two(view_name_etc, " ");
      utils::trim(view_type);
      utils::trim(view_name_etc);
      view_type = utils::to_lower(view_type);
      SP$(70) ct_vred(view_type) pcommas ct_vred(view_name_etc) pendl;

      if (view_name_etc.empty()) {
        PF$(1)
        "No information provided for view \"" << view_name << "\"" << pendo;
      }

      lay_bou = utils::split_str_in_two(view_name_etc, "[");
      parent = utils::split_str_in_two(view_name_etc, "]");
      SP$(70) ct_vred(lay_bou) pcommas ct_vred(parent) pendl;
      if (parent == view_name_etc)
        parent = "";

      utils::trim(parent);
      utils::trim(lay_bou);

      if (parent.size() && !view_exists(parent)) {
        PF$(1)
        "Inexistent parent \"" << parent << "\" for view \"" << view_name
                               << "\"" << pendo;
      }
      if (parent == "" && lay_bou[0] != '(') {
        PF$(1)
        "No information provided for top level view \"" << view_name << "\""
                                                        << pendo;
      }

      // bounds mentioned
      if (lay_bou[0] == '(') {
        bounds = utils::split_str_in_two(lay_bou, ")");
        utils::erase_first_nchar(bounds); // erase leadin '('
        vec_str bnds = utils::split_str<std::string>(bounds, ',');
        if (bnds.size() != 4) {
          PF$(1)
          "Information provided on bounds for view \""
              << view_name << "\" in invalid." << pendo;
        }
        for (int i = 0; i < 4; ++i) {
          int val = utils::str_to_val<int>(utils::trim(bnds[i]));
          if (val < 0) {
            PF$(1)
            "Information provided on bounds for view \""
                << view_name << "\" in invalid." << pendo;
          }
          if (val > 2) { // val might be 0.0 to 1.0
            b[i] = pangolin::Attach::Pix(val);
          } else {
            float valf = utils::str_to_val<float>(utils::trim(bnds[i]));
            b[i] = pangolin::Attach::Frac(valf);
          }
        }
      } else {
        b[0] = pangolin::Attach(0.0f);
        b[1] = pangolin::Attach(1.0f);
        b[2] = pangolin::Attach(0.0f);
        b[3] = pangolin::Attach(1.0f);
      }

      utils::trim(lay_bou);
      // layout mentioned
      if (lay_bou[0] == '{' && utils::last_char(lay_bou) == '}') {
        layout = utils::erase_last_nchar(lay_bou);
        utils::erase_first_nchar(layout);
        utils::to_lower(layout);
        lay_bou = "";
        if (layout == "overlay")
          lay = pangolin::LayoutOverlay;
        else if (layout == "equal")
          lay = pangolin::LayoutEqual;
        else if (layout == "equalhoriz")
          lay = pangolin::LayoutEqualHorizontal;
        else if (layout == "equalvert")
          lay = pangolin::LayoutEqualVertical;
        else if (layout == "horiz")
          lay = pangolin::LayoutHorizontal;
        else if (layout == "vert")
          lay = pangolin::LayoutVertical;
        else {
          PF$(1) "Invalid layout for view \"" << view_name << "\"" << pendo;
        }
      }
      if (lay_bou.size()) {
        PF$(1)
        "Invalid/Extra format/information provided for level view \""
            << view_name << "\"" << pendo;
      }

      SP$(70)
      ct_vred(view_name) pcommas ct_vred(view_type) pcommas ct_vred(layout)
              pcommas ct_vred(bounds) pcommas ct_vred(parent)
          << pendo;
      if (view_type == "container") {
        if (bounds.size()) {
          if (layout.empty())
            lay = pangolin::Layout::LayoutEqual;
          add_container(view_name, b[0], b[1], b[2], b[3], lay, parent);
        } else if (layout.size())
          add_container(view_name, lay, parent);
        else
          add_container(view_name, pangolin::LayoutEqual, parent);
        SP$(70) "Container added" << pendo;
      } else if (view_type == "2d") {
        if (vn_[parent]->type() == Container)
          add_2d_view(view_name, parent);
        else {
          PF$(1)
          "Trying to add 2d view \"" << view_name << "\" to a"
                                                     " non-Container type view."
                                     << pendo;
        }
        if (View(parent).layout == pangolin::LayoutOverlay && bounds.size())
          vn_[view_name]->bounds(b[0], b[1], b[2], b[3]);
        SP$(70) "2D added" << pendo;
      } else if (view_type == "3d") {
        if (vn_[parent]->type() == Container)
          add_3d_view(view_name, parent);
        else {
          PF$(1)
          "Trying to add 3d view \"" << view_name << "\" to a"
                                                     " non-Container type view."
                                     << pendo;
        }
        if (View(parent).layout == pangolin::LayoutOverlay && bounds.size())
          vn_[view_name]->bounds(b[0], b[1], b[2], b[3]);
        SP$(70) "3D added" << pendo;
      }
    }
  } catch (std::exception e) {
    PE$ ct_ylw("Exception: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}
