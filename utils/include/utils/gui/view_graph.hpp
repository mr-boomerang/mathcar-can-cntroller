#pragma once

#include <chrono>
#include <thread>

#include <utils/check_ret.h>
#include <utils/error_code.h>
#include <utils/gui/gui_handler.hpp>
#include <utils/gui/view_node.hpp>

#include <pangolin/pangolin.h>

#define Window ViewGraph::I()
#define View(arg) ViewGraph::I().get_view(arg)
#define Viewnode(arg) ViewGraph::I().get_view_node(arg, __FILE__, __LINE__)

class ViewGraph {
public:
  static ViewGraph &I() {
    static ViewGraph vg;
    return vg;
  }

  int simple_idle_loop() {
    try {
      while (!pangolin::ShouldQuit()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::FinishFrame();
      }
    } catch (std::exception e) {
      return RET_EXCEPTION_INT;
    }
    return RET_SUCCESS_INT;
  }

  int init(const std::string &name, int w, int h, int *argc = NULL,
           char ***argv = NULL);
  int add_2d_view(const std::string &name, const std::string &parent);

  int add_3d_view(const std::string &name, const std::string &parent);

  int add_container(const std::string &name,
                    pangolin::Layout layout = pangolin::LayoutEqual,
                    const std::string &parent = "");
  int add_container(const std::string &name, pangolin::Attach bot,
                    pangolin::Attach top, pangolin::Attach left,
                    pangolin::Attach right,
                    pangolin::Layout layout = pangolin::LayoutEqual,
                    const std::string &parent = "");

  int read_layout(const std::string &layout_fn);

  pangolin::View &get_view(const std::string &name);
  ViewNode *get_view_node(const std::string &name, std::string fn = "",
                          int line = -1);
  ViewNode *get_view_node(pangolin::View *v, std::string fn = "",
                          int line = -1);

  int view_exists(const std::string &name);

  // To display information of window point.
  int pos[2];
  float info[4]; // it will contain r,g,b,intensity or X,Y,Z,Distance

  pangolin::View *sel_view_;
  // added 260916:1530 to extract list of views and their show status
  int get_list_of_views(std::vector<std::string> &views_names,
                        std::vector<bool> &views_isshown) {
    views_names.clear();
    views_isshown.clear();

    for (std::map<std::string, ViewNode *>::iterator it = vn_.begin();
         it != vn_.end(); ++it) {
      ViewNode *viewNode = (it->second);
      if (viewNode != NULL) {
        views_names.push_back(it->first);
        views_isshown.push_back((viewNode->view().IsShown()));
      }
    }
    return RET_SUCCESS_INT;
  }

private:
  ViewGraph() : init_info_avail_(false) {}

  /* data */
  bool init_info_avail_;

  std::map<std::string, ViewNode *> vn_;
  std::map<pangolin::View *, ViewNode *> vv_;
  std::map<std::string, std::string> parent_name_;

  GuiHandler *gh_;
};
