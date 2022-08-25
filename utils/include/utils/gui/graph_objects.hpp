#pragma once

// NOTE: for error at ct::RED
// Always include this file before view_graph.hpp, or in general any file which
// includes colored_text.h, as it changes defines a macro "red()" which replaces
// a function called "red()" in boost/graph/property.hpp
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/topology.hpp>
#include <boost/property_map/property_map.hpp>

#include <utils/check_ret.h>
#include <utils/color.hpp>
#include <utils/error_code.h>
#include <utils/gui/object_xd.hpp>

#include <pangolin/pangolin.h>

#define TEXT_H 12
#define LAYOUT_SIDE_LEN 200.0

// Third party code inspiration: All boost related code taken from
// http://code.google.com/p/levelfour/source/browse/branches/dev/vd2/Components/D2M/sandratest/kamada_layout_example.cpp?r=207
// TODO: This needs to be cleaned up.
#if 1
typedef boost::square_topology<>::point_type Point;

struct VertexProperties {
  std::size_t index;
  Point point;
};

struct EdgeProperty {
  EdgeProperty(const std::size_t &w) : weight(w) {}
  double weight;
};

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS,
                              VertexProperties, EdgeProperty>
    Graph;
typedef boost::property_map<Graph, std::size_t VertexProperties::*>::type
    VertexIndexPropertyMap;
typedef boost::property_map<Graph, Point VertexProperties::*>::type PositionMap;
typedef boost::property_map<Graph, double EdgeProperty::*>::type
    WeightPropertyMap;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;
#endif

// Third party code ends.

class GraphNodeObject;
class GraphEdgeObject;
class FriendGnoGo;

class GraphObject : public Objectxd {
  friend FriendGnoGo;

public:
  GraphObject(const utils::Color &bg_col, bool use_on_off)
      : Objectxd(TwoDObject) {
    reinit(bg_col, use_on_off);
  }

  int reinit(const utils::Color &bg_col, bool use_on_off);
  int draw_me_like_one_of_your_french_girls();

  int draw_obj(pangolin::View &v); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  int add_node(const std::string &node_name, const utils::Color &col,
               const utils::Color &sec_col = utils::Color::invalid,
               bool on = true);
  int add_edge(const std::string &src_node, const std::string &end_node);
  GraphNodeObject *node(const std::string &node_name);
  // TODO: remove node and remove edge.

  // TODO: edge_for_xy();
  std::string node_for_xy(float x, float y) const;

private:
  /* data */
  utils::Color bg_col_;
  bool use_on_off_;

  /* graph data */
  PositionMap positionMap_;
  Graph g_;
  std::set<GraphEdgeObject *> edges_;
  std::map<VertexDescriptor, GraphNodeObject *> vd_nodes_;
  std::map<std::string, VertexDescriptor> name_vd_;
  VertexIndexPropertyMap vtxIdPropmap_;
  WeightPropertyMap weightPropertyMap_;
  int idx_;
  // jiska koi nahi hota uska dummy hota hai.
  VertexDescriptor dummy_desc_; // in case on/off of GraphNode is used.

  /* functions */
  int add_dummy_edge(const VertexDescriptor &vd);
  int remove_dummy_edge(const VertexDescriptor &vd);
};

class GraphNodeObject : public Objectxd {
  friend int GraphObject::add_node(const std::string &node_name,
                                   const utils::Color &col,
                                   const utils::Color &sec_col, bool on);

public:
  // resolution is in degrees.
  GraphNodeObject(const std::string &node_name, float x_ratio, float y_ratio,
                  float resolution_degree, const utils::Color &pc,
                  const utils::Color &sc = utils::Color::invalid,
                  bool state = true)
      : Objectxd(TwoDObject), go_(NULL) {
    reinit(node_name, x_ratio, y_ratio, resolution_degree, pc, sc, state);
  }

  int reinit(const std::string &node_name, float x_ratio, float y_ratio,
             float resolution_degree, const utils::Color &pc,
             const utils::Color &sc = utils::Color::invalid, bool state = true);

  int draw_obj(pangolin::View &v); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  utils::Color &color() { return *col_; }
  int pri_color(const utils::Color &col) {
    pri_col_ = col;
    return RET_SUCCESS_INT;
  }
  int sec_color(const utils::Color &col) {
    sec_col_ = col;
    return RET_SUCCESS_INT;
  }

  Point &center() { return pt_; }
  int ratios(float x_ratio, float y_ratio) {
    x_ratio_ = x_ratio;
    y_ratio_ = y_ratio;
    return RET_SUCCESS_INT;
  }

  Point point_at_angle(float angle); // angle in degrees.
  int point_in_dir_of(const Point &p, GLfloat *&x, GLfloat *&y);

  bool contains(float x, float y);

  //  int toggle_state() { return state_ ? set_state_off() : set_state_on(); }
  int toggle_state() { return state_ ? !(set_state_off()) : set_state_on(); }
  // This should be called last when turning node on, as it redraws the graph.
  int set_state_on();
  int set_state_off();
  bool state() const { return state_; }

  std::string name() const { return name_; }
  int name(const std::string &name) {
    utils::split_str_in_two(name, "/", &name_, &friendly_name_, -1);
    name_ = name;
    if (friendly_name_.empty())
      friendly_name_ = name_;
    txt_ = fnt_->Text(friendly_name_);
    return RET_SUCCESS_INT;
  }

  int width() { return 1.4 * txt_.Width(); }
  int height() { return TEXT_H + 5; }

  int inc_edge_count();
  int dec_edge_count();
  int edge_count() { return edge_cnt_; }

  int dummy_edge_cnt_;

private:
  /* data */
  utils::Color pri_col_;
  utils::Color sec_col_;
  utils::Color *col_;
  std::string name_;
  std::string friendly_name_;

  bool state_;
  int edge_cnt_;
  GraphObject *go_;

  pangolin::GlFont *fnt_;
  pangolin::GlText txt_;

  GLfloat *pts_;
  int num_pts_;
  Point pt_;
  float x_ratio_, y_ratio_;
  float resolution_;
};

class GraphEdgeObject : public Objectxd {
public:
  GraphEdgeObject(GraphNodeObject *starting_node, GraphNodeObject *end_node)
      : Objectxd(TwoDObject) {
    reinit(starting_node, end_node);
  }
  int reinit(GraphNodeObject *starting_node, GraphNodeObject *end_node);

  int draw_obj(pangolin::View &v); // opaque by default
  int set_transparency(float t) {
    transparency_ = t;
    return RET_SUCCESS_INT;
  }
  ObjectType type() { return type_; }

  GraphNodeObject *src() { return s_; }
  GraphNodeObject *end() { return e_; }

private:
  /* data */
  GraphNodeObject *s_;
  GraphNodeObject *e_;
  Point sc_, ec_; // src center and end center.
  GLfloat *sx_, *sy_, *ex_, *ey_;
};

class FriendGnoGo // Friend brigde between GraphNodeObject and GraphObject.
{
  friend int GraphNodeObject::inc_edge_count();
  friend int GraphNodeObject::dec_edge_count();

public:
private:
  /* data */
  FriendGnoGo();

  static int call_remove_dummy_edge(GraphObject *go,
                                    const GraphNodeObject *gno) {
    for (std::pair<VertexDescriptor, GraphNodeObject *> node : go->vd_nodes_) {
      if (node.second == gno) {
        return go->remove_dummy_edge(node.first);
      }
    }
    return RET_CANNOT_REMOVE_DUMMY_EDGE;
  }

  static int call_add_dummy_edge(GraphObject *go, const GraphNodeObject *gno) {
    for (std::pair<VertexDescriptor, GraphNodeObject *> node : go->vd_nodes_) {
      if (node.second == gno) {
        return go->add_dummy_edge(node.first);
      }
    }
    return RET_CANNOT_ADD_DUMMY_EDGE;
  }
};
