#include <utils/gui/data_plotter_object.hpp>

#include <utils/mode_printing.h>

int DataPlotterObject::reinit(const std::vector<std::string>& labels) {
  PRINT_FUNC_ENTER;
  const int sn=36;
  
  labels_ = labels;
  log.SetLabels(labels_);
  SP$(sn) ct_vpur(labels_.size()) pendl;
  PRINT_STL_SEC(sn, labels_);

  plotter = new pangolin::Plotter(&log);
  plotter->SetBounds(0.1, 0.9, 0.1, 0.9);
  plotter->Track("$i");
  notAdded = true;
  
  PRINT_FUNC_EXIT;
  return 0;
}

int DataPlotterObject::update(const std::vector<float>& data){
  PRINT_FUNC_ENTER;

  if (labels_.size() < data.size()) {
    labels_.clear();
    for(int i=0; i < data.size(); i++) {
      labels_.push_back("labels" + utils::val_to_str(i));
    }
    reinit(labels_);
  }

  log.Log(data);
  PRINT_FUNC_EXIT;
  return 0;
}

#ifdef BUILT_WITH_ROS

int DataPlotterObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                              uint32_t queue_sz) {
  PRINT_FUNC_ENTER;
  sub_ = n.subscribe(topic_sub, queue_sz, &DataPlotterObject::callback, this);
  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

void DataPlotterObject::callback(const std_msgs::Float32MultiArray &data_sub) {
  PRINT_FUNC_ENTER;
  update(data_sub.data);
  PRINT_FUNC_EXIT;
}
#endif

// This function is called when iterator in ViewNode draws the objects.
int DataPlotterObject::draw_obj(pangolin::View &v) {
  PRINT_FUNC_ENTER;
  glColor4f(1.0f, 1.0f, 1.0f, transparency_); // just to set transparency

  if (notAdded) {
    if (plotter) {
      v.AddDisplay(*plotter);
      notAdded = false;
      PI$ "Added plotter view: " << ct_vcyn(name_) pendl;
    }
  }

  glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // just to reset transparency
  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}
