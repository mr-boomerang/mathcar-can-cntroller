#include <utils/gui/graph_objects.hpp>

#include <utils/mode_printing.h>
#include <utils/str_utils.h>

#include <boost/graph/circle_layout.hpp>
#include <boost/graph/kamada_kawai_spring_layout.hpp>

#define MIN_EDGE_WEIGHT 100

/*--------------------------GraphObject Start---------------------------------*/

#if 1

int GraphObject::reinit(const utils::Color &bg_col, bool use_on_off) {
  try {
    vtxIdPropmap_ = boost::get(&VertexProperties::index, g_);
    positionMap_ = boost::get(&VertexProperties::point, g_);
    weightPropertyMap_ = boost::get(&EdgeProperty::weight, g_);
    idx_ = 0;
    bg_col_ = bg_col;
    use_on_off_ = use_on_off;

    // since this should be an invisible node, we are not calling add_node
    // funciton.
    if (use_on_off_) {
      dummy_desc_ = boost::add_vertex(g_);
      vtxIdPropmap_[dummy_desc_] = idx_;
      idx_++;
      // vd_nodes_[dummy_desc_] = new GraphNodeObject("DUMMMY", 0.5f, 0.5f,
      // 0.5f,
      // utils::Color::grey,
      // utils::Color::black, false);
      // name_vd_["DUMMMY"] = dummy_desc_;
    }
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int GraphObject::draw_me_like_one_of_your_french_girls() {
  PRINT_FUNC_ENTER;

  // bool retval = false;
  int retval = 1;

  int mec = 0;          // max edge count.
  VertexDescriptor mvd; // max vertex descriptor.
  for (std::pair<VertexDescriptor, GraphNodeObject *> node : vd_nodes_) {
    if (mec < node.second->edge_count()) {
      mec = node.second->edge_count();
      mvd = node.first;
    }
  }
  if (mec) {
    PI$ "Adding dummy node to " << ct_dblu(vd_nodes_[mvd]->name()) pendl;
    CHECK_RET(add_dummy_edge(mvd));
  }

  boost::circle_graph_layout(g_, positionMap_, LAYOUT_SIDE_LEN);
  PI$ "circle_graph_layout executed" pendl;

#if 0
  retval = boost::kamada_kawai_spring_layout(
      g_, positionMap_, weightPropertyMap_, boost::square_topology<>(),
      boost::side_length<double>(LAYOUT_SIDE_LEN), boost::layout_tolerance<>(),
      1.0, vtxIdPropmap_);
  PI$ "kamada_kawai_spring_layout executed" pendl;
  if (!retval) {
    PE$ "kamada_kawai_spring_layout returned false." pendl;
    return false;
  }
#endif

  boost::graph_traits<Graph>::vertex_iterator i, end;
  double x_min = DBL_MAX, y_min = DBL_MAX, x_max = 0.0, y_max = 0.0;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    double x = positionMap_[*i][0], y = positionMap_[*i][1];
    if (x < x_min)
      x_min = x;
    if (y < y_min)
      y_min = y;
    if (x > x_max)
      x_max = x;
    if (y > y_max)
      y_max = y;
  }
// dividing by 0.9 or such to keep some margin of 0.05 or such on both sides,
// 0.05 or such will be later added.
#define GRAPH_LAYOUT_TOTAL_WITHOUT_MARGIN 0.8
#define GRAPH_LAYOUT_LEFT_MARGIN (1 - GRAPH_LAYOUT_TOTAL_WITHOUT_MARGIN) / 2
  double x_span = (x_max - x_min) / GRAPH_LAYOUT_TOTAL_WITHOUT_MARGIN;
  double y_span = (y_max - y_min) / GRAPH_LAYOUT_TOTAL_WITHOUT_MARGIN;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    double x =
        (positionMap_[*i][0] - x_min) / x_span + GRAPH_LAYOUT_LEFT_MARGIN;
    double y =
        (positionMap_[*i][1] - y_min) / y_span + GRAPH_LAYOUT_LEFT_MARGIN;
    if (vd_nodes_.find(*i) != vd_nodes_.end())
      vd_nodes_[*i]->ratios((float)x, (float)y);
  }

  PRINT_FUNC_EXIT;
  return retval;
}

int GraphObject::draw_obj(pangolin::View &v) {

  try {
    glColor3fv(bg_col_);
    pangolin::glDrawRect(0.0f, 0.0f, v.vp.w, v.vp.h);
    glColor3f(1.1f, 1.0f, 1.0f);
    for (std::pair<VertexDescriptor, GraphNodeObject *> node : vd_nodes_) {
      if (node.second != NULL)
        CHECK_RET(node.second->draw_obj(v));
    }
    for (GraphEdgeObject *edge : edges_) {
      if (edge != NULL)
        CHECK_RET(edge->draw_obj(v));
    }
  } catch (std::exception e) {
    PE$ ct_ylw("Exception: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int GraphObject::add_edge(const std::string &src_node,
                          const std::string &end_node) {
  try {
    // check if the nodes exist, Fatal exit if they don't exist
    std::map<std::string, VertexDescriptor>::iterator name_vd_it_s;
    std::map<std::string, VertexDescriptor>::iterator name_vd_it_e;
    name_vd_it_s = name_vd_.find(src_node);
    name_vd_it_e = name_vd_.find(end_node);
    if (name_vd_it_s == name_vd_.end()) {
      PE$ "VertexDescriptor doesn't exist for src node = "
          << src_node << "( while end node was" << end_node << ")." pendl;
      return RET_VERTEX_DESCRIPTOR_DOES_NOT_EXIST_FOR_SRC_NODE;
    }
    if (name_vd_it_e == name_vd_.end()) {
      PE$ "VertexDescriptor doesn't exist for end node = "
          << end_node << "( while src node was" << src_node << ")." pendl;
      return RET_VERTEX_DESCRIPTOR_DOES_NOT_EXIST_FOR_END_NODE;
    }
    std::map<VertexDescriptor, GraphNodeObject *>::iterator vd_nodes_it_s;
    std::map<VertexDescriptor, GraphNodeObject *>::iterator vd_nodes_it_e;
    vd_nodes_it_s = vd_nodes_.find(name_vd_it_s->second);
    vd_nodes_it_e = vd_nodes_.find(name_vd_it_e->second);
    if (vd_nodes_it_s == vd_nodes_.end()) {
      PE$ "GraphNodeObject doesn't exist for src node = "
          << src_node << "( while end node was" << end_node << ")." pendl;
      return RET_GRAPH_NODE_OBJECT_DOES_NOT_EXIST_FOR_SRC_NODE;
    }
    if (vd_nodes_it_e == vd_nodes_.end()) {
      PE$ "GraphNodeObject doesn't exist for end node = "
          << end_node << "( while src node was" << src_node << ")." pendl;
      return RET_GRAPH_NODE_OBJECT_DOES_NOT_EXIST_FOR_END_NODE;
    }
    GraphNodeObject *src = vd_nodes_it_s->second;
    GraphNodeObject *end = vd_nodes_it_e->second;
    if (src == NULL || end == NULL) {
      PE$ "Trying to add edge with invalid nodes(" << src_node pcomma end_node
                                                   << ")." pendl;
      return RET_ADDING_EDGE_WITH_INVALID_NODES;
    }
    // check if the edge exists, return true if it does
    for (GraphEdgeObject *edge : edges_) {
      if ((edge->src() == src && edge->end() == end) ||
          (edge->src() == end && edge->end() == src)) {
        PW$ "Trying to add edge again (" << src_node pcomma end_node
                                         << ")." pendl;
        return RET_ADDING_EDGE_AGAIN;
      }
    }
    PI$ "Adding edge between " << ct_dblu(src->name()) << " and "
                               << ct_dblu(end->name()) pendl;
    int edge_wt = std::max(MIN_EDGE_WEIGHT, src->width() + end->width());
    boost::add_edge(name_vd_[src_node], name_vd_[end_node],
                    EdgeProperty(edge_wt), g_);
    edges_.insert(new GraphEdgeObject(src, end));
    CHECK_RET(src->inc_edge_count());
    CHECK_RET(end->inc_edge_count());
    DP$ "Added edge between " << ct_dblu(src->name()) << " and "
                              << ct_dblu(end->name()) pendl;
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_STD_EXCEPTION;
  }
  return RET_SUCCESS_INT;
}

int GraphObject::add_dummy_edge(const VertexDescriptor &vd) {
  SP$(33) "Adding dummy edge." pendl;
  boost::add_edge(vd, dummy_desc_, EdgeProperty(MIN_EDGE_WEIGHT), g_);
  vd_nodes_[vd]->dummy_edge_cnt_++;
  // GraphEdgeObject* ge = new GraphEdgeObject(vd_nodes_[dummy_desc_],
  // vd_nodes_[vd]);
  // vd_nodes_[vd]->dummy_ge_ = ge;
  // edges_.insert(ge);
  return RET_SUCCESS_INT;
}

int GraphObject::remove_dummy_edge(const VertexDescriptor &vd) {
  SP$(33) "Removing dummy edge." pendl;
  boost::remove_edge(vd, dummy_desc_, g_);
  vd_nodes_[vd]->dummy_edge_cnt_--;
  // edges_.erase(vd_nodes_[vd]->dummy_ge_);
  return RET_SUCCESS_INT;
}

int GraphObject::add_node(const std::string &node_name, const utils::Color &col,
                          const utils::Color &sec_col, bool on) {
  try {
    PI$ "Adding node " << ct_grn(node_name) pendl;

    if (name_vd_.find(node_name) != name_vd_.end()) {
      PE$ "Trying to add node (" << ct_vylw(node_name)
                                 << ") for second time." pendl;
      return RET_ADDING_NODE_AGAIN;
    }

    VertexDescriptor vd = boost::add_vertex(g_);
    vtxIdPropmap_[vd] = idx_;
    idx_++;
    vd_nodes_[vd] =
        new GraphNodeObject(node_name, 0.5f, 0.5f, 0.5f, col, sec_col, on);
    name_vd_[node_name] = vd;

    if (use_on_off_) {
      SP$(33) "add DUMMY edge to " << ct_dblu(node_name) pendl;
      CHECK_RET(add_dummy_edge(vd));
      vd_nodes_[vd]->go_ = this;
    }
  } catch (std::exception e) {
    PE$ ct_ylw("Exception: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

GraphNodeObject *GraphObject::node(const std::string &node_name) {
  if (name_vd_.find(node_name) == name_vd_.end()) {
    PF$(3)
    "Trying to access non-existant node \"" << ct_ylw(node_name) << "\"" pendl;
  }
  return vd_nodes_[name_vd_[node_name]];
}

std::string GraphObject::node_for_xy(float x, float y) const {
  for (std::pair<VertexDescriptor, GraphNodeObject *> node : vd_nodes_) {
    if (node.second->contains(x, y)) {
      return node.second->name();
    }
  }
  return "";
}

#endif

/*--------------------------GraphObject End-----------------------------------*/

/*--------------------------GraphNodeObject Start-----------------------------*/

// Embedded fonts:
extern "C" const unsigned char AnonymousPro_ttf[];

int GraphNodeObject::reinit(const std::string &node_name, float x_ratio,
                            float y_ratio, float resolution_degree,
                            const utils::Color &pc, const utils::Color &sc,
                            bool state) {
  state_ = state;
  edge_cnt_ = 0;
  pri_col_ = (pc == utils::Color::invalid) ? utils::Color::red : pc;
  sec_col_ = (sc == utils::Color::invalid) ? pri_col_ : sc;
  col_ = state_ ? &pri_col_ : &sec_col_;

  fnt_ = new pangolin::GlFont(AnonymousPro_ttf, TEXT_H);
  name(node_name);

  x_ratio_ = x_ratio;
  y_ratio_ = y_ratio;
  pt_[0] = 0.0;
  pt_[1] = 0.0;

  // added 300117:1600. draw differently for .launch nodes.
  if (node_name.find(".launch") != std::string::npos)
    resolution_degree = 60;
  resolution_ = resolution_degree;

  num_pts_ = (int)(360 / resolution_degree);

  pts_ = new GLfloat[2 * num_pts_];

  return RET_SUCCESS_INT;
}

int GraphNodeObject::draw_obj(pangolin::View &v) {
  SP$(71)
  "Drawing graph node " << ct_vpur(name_) << " in " << col_ << " color, at "
                        << pt_[0] pcomma pt_[1] pendl;

  if (v.vp.w * x_ratio_ != pt_[0] || v.vp.h * y_ratio_ != pt_[1]) {

    int major_radius = 0.8 * txt_.Width(); // 1.x extra for margin.
    int minor_radius = TEXT_H + 5;         // 0.3*major_radius;
    pt_[0] = v.vp.w * x_ratio_;
    pt_[1] = v.vp.h * y_ratio_;

    SP$(72) "Points of ellipse for " << name_ << ": " pendl;
    for (int i = 0; i < 2 * num_pts_; ++i) {
      float radian = i * (M_PI / 180) * resolution_;
      radian /= 2; // divide it by two beacuse 0 is x and 1 is y.
      pts_[i] = major_radius * cos(radian) + pt_[0];
      pts_[++i] = minor_radius * sin(radian) + pt_[1];
      SPC$(72)
      "pts_[" << i - 1 << "]=" << pts_[i - 1] << ", pts_[" << i << "]"
              << pts_[i] pcommaso;
    }
    SPC$(72) pendo;
  }

  glColor3fv(*col_);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  SP$(71) "Drawing ellipse." pendl;
  // Draw ellipse of the pre-computed points.
  glVertexPointer(2, GL_FLOAT, 0, pts_);
  glEnableClientState(GL_VERTEX_ARRAY);
#if 0
  glDrawArrays(GL_LINE_LOOP, 0, num_pts_);
  glDisableClientState(GL_VERTEX_ARRAY);
#else
  // added 080916:1648
  glDrawArrays(GL_POLYGON, 0, num_pts_);
  glDisableClientState(GL_VERTEX_ARRAY);
  // since filled ellipse color and text color are same earlier,
  // changing text color.
  glColor3f(0.0f, 0.0f, 0.0f);
#endif
  // to shift the height by half of text, we need to divide by 4.
  txt_.DrawWindow(v.vp.l + pt_[0] - txt_.Width() / 2,
                  v.vp.b + pt_[1] - (TEXT_H / 4));

  glColor3f(1.0f, 1.0f, 1.0f);

  return RET_SUCCESS_INT;
}

Point GraphNodeObject::point_at_angle(float angle) {
  Point retval;
  int idx = 2 * angle * resolution_;
  retval[0] = pts_[idx];
  retval[1] = pts_[idx + 1];
  return retval;
}

int GraphNodeObject::point_in_dir_of(const Point &p, GLfloat *&x, GLfloat *&y) {
  Point slope;
  slope[0] = p[0] - pt_[0];
  slope[1] = p[1] - pt_[1];
  float angle = atan2f((float)slope[1], (float)slope[0]) * (180.0f / M_PI);
  if (angle < 0.0f)
    angle = 360 + angle;
  int idx = 2 * (int)(angle / resolution_); // 2 because 0 is x and 1 is y.

  x = pts_ + idx;
  y = x + 1;
  return RET_SUCCESS_INT;
}

bool GraphNodeObject::contains(float x, float y) {
  bool retval = false;
  try {
    retval = (std::abs((float)pt_[0] - x) < width()) &&
             (std::abs((float)pt_[1] - y) < height());
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_BOOL;
  }
  return retval;
}

int GraphNodeObject::set_state_on() {
  PRINT_FUNC_ENTER;
  state_ = true;
  col_ = &pri_col_;
  PRINT_FUNC_EXIT;
  return !(go_->draw_me_like_one_of_your_french_girls());
}

int GraphNodeObject::set_state_off() {
  state_ = false;
  col_ = &sec_col_;
  return RET_SUCCESS_INT;
}

int GraphNodeObject::inc_edge_count() {
  // remove only if count is zero(is being increased to one).
  if (!edge_cnt_)
    CHECK_RET(FriendGnoGo::call_remove_dummy_edge(go_, this));
  return ++edge_cnt_;
}

int GraphNodeObject::dec_edge_count() {
  // add only if count is one(is being decreased to zero).
  if (edge_cnt_ == 1)
    CHECK_RET(FriendGnoGo::call_add_dummy_edge(go_, this));
  return --edge_cnt_;
}

/*--------------------------GraphNodeObject End-------------------------------*/

/*--------------------------GraphEdgeObject Start-----------------------------*/

int GraphEdgeObject::reinit(GraphNodeObject *starting_node,
                            GraphNodeObject *end_node) {
  s_ = starting_node;
  e_ = end_node;

  Point &sc = s_->center(), ec = e_->center();
  sc_ = sc;
  ec_ = ec;

  return RET_SUCCESS_INT;
}

int GraphEdgeObject::draw_obj(pangolin::View & /*v*/) {

  SP$(73)
  "Drawing graph edge object for " << ct_vpur(s_->name()) << " and "
                                   << ct_vpur(e_->name()) pendl;

  Point &sc = s_->center(), ec = e_->center();
  if (sc_[0] != sc[0] || sc_[1] != sc[1] || ec_[0] != ec[0] ||
      ec_[1] != ec[1]) {
    sc_ = sc;
    ec_ = ec;
    s_->point_in_dir_of(ec_, sx_, sy_);
    e_->point_in_dir_of(sc_, ex_, ey_);
  }

  utils::Color col_s = s_->color(), col_e = e_->color();

  SP$(73)
  "Edge between " << ct_dpur(s_->name()) << " and "
                  << ct_dpur(e_->name()) pendl;

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  glBegin(GL_LINES);
#if 0
  glColor3fv(col_e);
  glVertex2f(*sx_, *sy_);
  glColor3fv(col_s);
#else
  // added 080916:1642
  // the above code draws edge color changing
  // from end node to start node respectively,
  // where the reverse is(may be) required.
  glColor3fv(col_s);
  glVertex2f(*sx_, *sy_);
  glColor3fv(col_e);
#endif
  glVertex2f(*ex_, *ey_);
  glEnd();

#if 0
  glPointSize(10.0);
  glBegin(GL_POINTS);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex2f(*sx_, *sy_);
  glVertex2f(*ex_, *ey_);
  glEnd();
#endif

  return RET_SUCCESS_INT;
}

/*--------------------------GraphEdgeObject End-------------------------------*/
