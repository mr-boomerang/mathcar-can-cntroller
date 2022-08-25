#include <utils/gui/objects_2d.hpp>
#include <utils/gui/objects_3d.hpp>
#include <utils/gui/view_graph.hpp>
#include <utils/gui/view_node.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <utils/mode_printing.h>

float rx = 0.0f; // red or X
float gy = 0.0f; // green or Y
float bz = 0.0f; // blue or Z
float dg = 0.0f; // distance or greyscale.
int mouse_x = -1, mouse_y = -1;

ViewNode::ViewNode(const std::string &name, ViewType type, ViewNode *parent)
    : v_(pangolin::Display(name)) {
  init_viewnode(name, type, parent);
}
// type_(type), parent_(parent), name_(name),
int ViewNode::init_viewnode(const std::string &name, ViewType type,
                            ViewNode *parent) {
  // reinit(pangolin::ProjectionMatrix(w,h,0.9*w,0.9*h,w/2,h/2,0.1,2000),
  // pangolin::ModelViewLookAt(0,-50,-50,0,0,10,pangolin::AxisNegY));
  try {
    type_ = type;
    parent_ = parent;
    name_ = name;
#if 1
    v_ = pangolin::Display(name_);
    if (parent == NULL)
      pangolin::DisplayBase().AddDisplay(v_);
    else
      parent_->view().AddDisplay(v_);

    if (ThreeD == type_) {
      v_.SetDrawFunction(
          std::bind(&ViewNode::draw, this, std::placeholders::_1));
      // v_.SetHandler(new pangolin::Handler3D(cam_3d_));
      txt_ = pangolin::GlFont::I().Text(name_.c_str());
      CHECK_RET(update_head_vp(v_.vp));
    } else if (TwoD == type_) {
      // TODO: 2D stuff
      v_.SetDrawFunction(
          std::bind(&ViewNode::draw, this, std::placeholders::_1));
      txt_ = pangolin::GlFont::I().Text(name_.c_str());
      CHECK_RET(update_head_vp(v_.vp));
    } else if (Panel == type_) {
      // TODO: Panel stuff
    } else {
      // Case: Container == type_
      v_.SetDrawFunction(
          std::bind(&ViewNode::draw_margins, this, std::placeholders::_1));
    }
#endif
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }

  init_ = true;
  return RET_SUCCESS_INT;
}

int ViewNode::bounds(pangolin::Attach bot, pangolin::Attach top,
                     pangolin::Attach left, pangolin::Attach right) {
  v_.SetBounds(bot, top, left, right);
  return RET_SUCCESS_INT;
}

int ViewNode::layout(pangolin::Layout layout) {
  try {
    v_.SetLayout(layout);
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int ViewNode::draw(pangolin::View &v) {

  if (TwoD == type_)
    v.ActivatePixelOrthographic();
  else
    v.Activate(cam_);

  for (std::string name : insertion_order_) {
    objects_[name]->draw_obj(v);
  }
  CHECK_RET(draw_margins(v));
  return RET_SUCCESS_INT;
}

int ViewNode::draw_margins(pangolin::View &v) {
  /* Margin design
   * -----------------------------
   * |_Viewname______x/           |
   * |                            |
   * |                            |
   * |                            |
   * |                            |
   * -----------------------------
   * */

  pangolin::VideoOutput recorder;
  try {

    /* Opengl settings */
    v.ActivatePixelOrthographic();

    pangolin::GlState s;
    s.glDisable(GL_DEPTH_TEST);
    s.glDisable(GL_LIGHTING);

    if (Container == type_) {
      glColor4f(0.564f, 0.831f, 0.709f, 0.5f);
      glLineWidth(2.0f);
      pangolin::glDrawLine(0, v.vp.h - 3, v.vp.w, v.vp.h - 3);
      glLineWidth(1.0f);
    } else {
      if (vp_head_wc_.l != v.vp.l || vp_head_wc_.b != v.vp.b)
        CHECK_RET(update_head_vp(v.vp));

      glColor4f(0.4f, 0.0f, 1.0f, 0.5f);
      /* Rectangle margin for whole view */
      pangolin::glDrawRectPerimeter(0, 0, v.vp.w, v.vp.h);

      /* Rectangle for name */
      // for width of rectangle get the text width.
      // pangolin::glDrawRect(head_left, head_top, head_right, head_bot);
      pangolin::glDrawRect(vp_head_.l, vp_head_.t(), vp_head_.r(), vp_head_.b);
      glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
      /* Record Rectangle */
      pangolin::glDrawRect(vp_head_.r() + 1, vp_head_.t() + 1,
                           vp_head_.r() + 20, vp_head_.b);
      glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
      pangolin::glDrawCircle((vp_head_.r() + 20 + vp_head_.r()) / 2,
                             (vp_head_.b + vp_head_.t() + 1) / 2, 3);
      /* Name */
      glColor3f(0.0f, 0.0f, 0.0f);
      txt_.DrawWindow(v.vp.l + VIEW_HEAD_BORDER,
                      v.vp.b + v.vp.h - 1.2 * VIEW_HEAD_HEIGHT);

#if 1
      /* Triangle next to record */
      glColor4f(0.7f, 0.0f, 0.0f, 1.0f);
      glBegin(GL_QUADS);
      // If you change any of the vertex2i line change vp_close_wc_ in
      // update_head_vp.
      glVertex2i(vp_head_.r() + 20, vp_head_.t());
      glVertex2i(vp_head_.r() + 20, vp_head_.b);
      // head_top + 1 shouldn't be required, o.w. triangle is a little tilted.
      glVertex2i(vp_head_.r() + 20 + VIEW_HEAD_BORDER, vp_head_.b);
      glVertex2i(vp_head_.r() + 20 + 2.5 * VIEW_HEAD_BORDER, vp_head_.t());
      glEnd();
#endif
      // close circle center
      glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
      float cx = (float)(vp_head_.r() + 20) + 0.7f * VIEW_HEAD_BORDER;
      float cy = (float)vp_head_.b + vp_head_.h / 2.0f;
      float radius = 0.5f * VIEW_HEAD_BORDER;
      pangolin::glDrawCirclePerimeter(cx, cy, radius);
      float radius_cos45 = 0.707 * radius; // also equal to roadisu*sin(45)
      pangolin::glDrawLine(cx - radius_cos45, cy - radius_cos45,
                           cx + radius_cos45, cy + radius_cos45);
      pangolin::glDrawLine(cx - radius_cos45, cy + radius_cos45,
                           cx + radius_cos45, cy - radius_cos45);
      /* Reset */
      glColor3f(1.0f, 1.0f, 1.0f);
#if 1
      if (recorder_.IsOpen()) {
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1); // TODO: Avoid this?
        glReadPixels(0, 0, 0, 0, GL_RGB, GL_UNSIGNED_BYTE, rec_buffer_.data);
        if (rec_resize_) {
          cv::Mat buf;
          glReadPixels(v.vp.l, v.vp.b, v.vp.w, v.vp.h, GL_RGB, GL_UNSIGNED_BYTE,
                       rec_buffer_.data);
          cv::resize(rec_buffer_, buf, rec_sz_);
          recorder_.WriteStreams(buf.data);
        } else {
          glReadPixels(v.vp.l, v.vp.b, v.vp.w, v.vp.h, GL_RGB, GL_UNSIGNED_BYTE,
                       rec_buffer_.data);
          recorder_.WriteStreams(rec_buffer_.data);
        }
        //  pangolin::RenderRecordGraphic(v.vp);
        v.vp.ActivatePixelOrthographic();
        pangolin::glRecordGraphic(v.vp.w - 2 * radius_, v.vp.h - 2 * radius_,
                                  radius_);
      }
#endif

      /* Info of the screen */
      if (v.vp.Contains(Window.pos[0], Window.pos[1])) {
        std::stringstream ss;
        const int gap = 5;
        const int txt_height = 10;
        // x,y are the top left of the text.
        int txtx = v.vp.l + gap, txty = v.vp.b + gap;
        pangolin::GlText txt;

        if (TwoD == type_) {
          int precision = 3;

          ss.str("");
          ss << "x=" << (Window.pos[0] - v.vp.l)
             << " y=" << (Window.pos[1] - v.vp.b);
          txt = pangolin::GlFont::I().Text(ss.str().c_str());
          glColor3f(0.0f, 1.0f, 1.0f);
          txt.DrawWindow(txtx, txty);

          if (Window.info[1] > 0.0f && Window.info[1] > 0.0f &&
              Window.info[2] > 0.0f) {
            ss.str("");
            ss << std::setprecision(precision) << Window.info[2];
            txt = pangolin::GlFont::I().Text(ss.str().c_str());
            glColor3f(0.0f, 0.0f, 1.0f);
            txt.DrawWindow(txtx, txty + gap + txt_height);

            ss.str("");
            ss << std::setprecision(precision) << Window.info[1];
            txt = pangolin::GlFont::I().Text(ss.str().c_str());
            glColor3f(0.0f, 1.0f, 0.0f);
            txt.DrawWindow(txtx, txty + 2 * (gap + txt_height));

            ss.str("");
            ss << std::setprecision(precision) << Window.info[0];
            txt = pangolin::GlFont::I().Text(ss.str().c_str());
            glColor3f(1.0f, 0.0f, 0.0f);
            txt.DrawWindow(txtx, txty + 3 * (gap + txt_height));
          } else if (Window.info[3] > 0.0f) {
            ss.str("");
            ss << std::setprecision(precision) << Window.info[3];
            txt = pangolin::GlFont::I().Text(ss.str().c_str());
            glColor3f(1.0f, 1.0f, 0.0f);
            txt.DrawWindow(txtx, txty + gap + txt_height);
          }

          glColor3f(1.0f, 1.0f, 1.0f); // Reset the color
        } else if (ThreeD == type_) {
          int precision = 2;
          ss.str("");
          ss << std::setprecision(precision) << Window.info[0];
          txt = pangolin::GlFont::I().Text(ss.str().c_str());
          glColor3f(1.0f, 0.0f, 0.0f);
          txt.DrawWindow(txtx, txty);

          ss.str("");
          ss << std::setprecision(precision) << Window.info[1];
          txt = pangolin::GlFont::I().Text(ss.str().c_str());
          glColor3f(0.0f, 1.0f, 0.0f);
          txt.DrawWindow(txtx, txty + gap + txt_height);

          ss.str("");
          ss << std::setprecision(precision) << Window.info[2];
          txt = pangolin::GlFont::I().Text(ss.str().c_str());
          glColor3f(0.0f, 0.0f, 1.0f);
          txt.DrawWindow(txtx, txty + 2 * (gap + txt_height));

          ss.str("");
          ss << std::setprecision(precision) << Window.info[3];
          txt = pangolin::GlFont::I().Text(ss.str().c_str());
          glColor3f(1.0f, 1.0f, 0.0f);
          txt.DrawWindow(txtx, txty + 3 * (gap + txt_height));

          glColor3f(1.0f, 1.0f, 1.0f); // Reset the color
        }
      }
    }
  } catch (std::exception e) {
    return RET_EXCEPTION_INT;
  }

  return RET_SUCCESS_INT;
}

int ViewNode::add_object(const std::string &name, Objectxd *obj) {
  try {
    if (obj == NULL) {
      PF$(2) "Passed object is NULL." << pendo;
    }
    if (objects_.find(name) != objects_.end()) {
      PF$(2) "Object with same name already exists" << pendo;
    }
    obj->set_view_ptr(&v_);
    obj->set_name(name);
    objects_[name] = obj;
    if (obj->type() == ThreeDObject) {
      Object3d *obj3d = static_cast<Object3d *>(obj);
      obj3d->set_cam(&cam_);
    }
    insertion_order_.push_back(name);
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int ViewNode::remove_object(const std::string &name) {
  try {
    std::map<std::string, Objectxd *>::iterator pos = objects_.find(name);
    if (pos == objects_.end()) {
      return RET_OBJECT_NOT_FOUND;
    }
    delete objects_[name];
    objects_.erase(pos);
    vec_str::iterator it =
        std::find(insertion_order_.begin(), insertion_order_.end(), name);
    insertion_order_.erase(it);
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

Objectxd *ViewNode::get_object(const std::string &name) {
  std::map<std::string, Objectxd *>::iterator pos = objects_.find(name);
  if (pos == objects_.end()) {
    return NULL;
  }
  return pos->second;
}

int ViewNode::update_head_vp(pangolin::Viewport &vp) {
  vp_head_.b = vp.h - 2.3 * VIEW_HEAD_HEIGHT + VIEW_HEAD_BORDER;
  // vp_head_.h = v.vp.h - 0.3*VIEW_HEAD_HEIGHT - vp_head_.b;
  vp_head_.h = 2 * VIEW_HEAD_HEIGHT - VIEW_HEAD_BORDER; // simplified form
  vp_head_.l = 0;
  vp_head_.w = txt_.Width() + 3 * VIEW_HEAD_BORDER;

  vp_head_wc_ = vp_head_;
  vp_head_wc_.l = vp.l;
  vp_head_wc_.b = vp.t() - vp_head_.h;
  vp_record_wc_ = vp_head_wc_;
  vp_record_wc_.l = vp_record_wc_.r();
  vp_record_wc_.w = 20;
  // vp_record_wc_.w+=VIEW_HEAD_BORDER;

  vp_record_wc_ = vp_head_wc_;
  vp_record_wc_.l = vp_record_wc_.r();
  vp_record_wc_.w = 20;
  // vp_record_wc_.w+=VIEW_HEAD_BORDER;

  // If you change any of the lines configuring vp_close_wc_ then change it
  // vertex after GL_QUADS in draw_margin.
  vp_close_wc_ = vp_head_wc_;
  vp_close_wc_.l = vp_close_wc_.r() + 20;
  vp_close_wc_.w = 20;
  return RET_SUCCESS_INT;
}

int ViewNode::set_parent(ViewNode *new_parent) {
  if (new_parent == NULL) {
    PF$(2) "Trying to see NULL as parent, shame on you." << pendo;
    return RET_PARENT_VIEWNODE_NULL;
  }
  parent_ = new_parent;
  return RET_SUCCESS_INT;
}

ViewNode *ViewNode::get_parent() { return parent_; }
