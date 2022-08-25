#include <algorithm>
#include <utils/TicToc.h>
#include <utils/error_code.h>
#include <utils/gui/gui_handler.hpp>
#include <utils/gui/view_graph.hpp>
using namespace utils;

#include <opencv2/imgproc/imgproc.hpp>
GuiHandler::GuiHandler(std::string flt_name) : float_(flt_name) {
  flt_view_ = &pangolin::Display(float_);
  flt_view_->Show(false);
  // flt_view_->SetBounds(0.0f, 0.0f, 0.0f, 0.0f);

  flt_draw_func_ =
      std::bind(&GuiHandler::EmptyDrawFunction, this, std::placeholders::_1);
  flt_view_->SetDrawFunction(flt_draw_func_);
}

void GuiHandler::SwitchViews(pangolin::View *it, pangolin::View *with,
                             pangolin::View *from, pangolin::View *to) {
  //  040816:1457
  //  issues with view/window draw/drop resolved
  if ((with)->NumVisibleChildren() > 1) {
    PE$ " sorry cannot add here " pendl;
    return;
  }
#if 1
  // added 190816:1040
  if (NULL == Viewnode(it)) {
    PE$ " Viewnode (source view) is NULL." pendl;
    return;
  }
  if (NULL == Viewnode(with)) {
    PE$ " Viewnode (destintion view) is NULL. Sorry cannot add here." pendl;
    return;
  }
#endif
  std::vector<pangolin::View *>::iterator pos_from;
  std::vector<pangolin::View *> &from_views = from->views;
  pos_from = std::find(from_views.begin(), from_views.end(), it);

  std::vector<pangolin::View *>::iterator pos_to;
  std::vector<pangolin::View *> &to_views = to->views;
  pos_to = std::find(to_views.begin(), to_views.end(), with);
  if (pos_from != from_views.end() && pos_to != to_views.end()) {

    if (from == to) {
      // If switching is happening within the view.
      std::iter_swap(pos_from, pos_to);
    } else {
      std::iter_swap(pos_from, pos_to);
      Viewnode(it)->set_parent(Viewnode(to));
      Viewnode(with)->set_parent(Viewnode(from));
    }
    pangolin::View *a = *pos_from;
    pangolin::View *b = *pos_to;
    pangolin::Attach al = a->left, ar = a->right, atop = a->top,
                     abtm = a->bottom;
    a->SetBounds(b->bottom, b->top, b->left, b->right);
    b->SetBounds(abtm, atop, al, ar);
  }

  to->ResizeChildren();
  from->ResizeChildren();
}

void GuiHandler::Mouse(pangolin::View &v, pangolin::MouseButton button, int x,
                       int y, bool pressed, int button_state) {

#if 1
  if (pressed && button == pangolin::MouseButtonLeft) {
    if (flt_view_->IsShown())
      return;
    pangolin::View *cv = v.FindChild(x, y);

    if (!cv)
      return;
    pangolin::View *ccv, *pv = &pangolin::DisplayBase();
    while ((ccv = cv->FindChild(x, y)) != NULL) {
      pv = cv;
      cv = ccv;
    }

    ViewNode *vn_cv = Viewnode(cv);
    // std::string view_name = vn_cv->name();
    if (vn_cv != NULL && vn_cv->point_in_close(x, y)) {
      vn_cv->view().Show(false);
      if (vn_cv->name() == "view2d_m") {
        View("container 2D").Show(false);
      }
      vn_cv->call_close_cb();
      return;
    } else if (vn_cv != NULL && vn_cv->point_in_record(x, y)) {
      vn_cv->toggleRecord();
    } else if (vn_cv != NULL && vn_cv->point_in_head(x, y)) {
      flt_view_->Show(true);
      w_ = cv->GetBounds().w / 2;
      h_ = cv->GetBounds().h / 2;
      setFloatViewBounds(x, y);

      from_view_ = pv;
      Window.sel_view_ = cv;
      pangolin::DisplayBase().SetFocus(); // so that left release call this
                                          // func
    } else {
      Handler::Mouse(v, button, x, y, pressed, button_state);
    }
  } else {
    // All events other than left click press
    if (from_view_) {
      // This block is to take care of stuff if dragging was initiated.

      // Events where any other button on mouse is pressed or released
      // deactivate the dragging, irrespective of view mouse is on.
      flt_view_->Show(false);
      // Note the info being reset.
      pangolin::View *it = Window.sel_view_; // it. (view from panel).
      pangolin::View *from = from_view_;     // from (panel).
      // Noted

      from_view_ = NULL;
      Window.sel_view_ = NULL;
      // Deactivated.

      if (!pressed && button == pangolin::MouseButtonLeft) {
        // Switch if left button is released.
        pangolin::View *with = v.FindChild(x, y); // add view to (main)
        if (!with)
          return;

        pangolin::View *cv, *to = &pangolin::DisplayBase();
        while ((cv = with->FindChild(x, y)) != NULL) {
          to = with;
          with = cv;
        }

        if (it != with) // Don't call function if not switching with itself.
          SwitchViews(it, with, from, to);
        // DP$ ct_vgrn(Viewnode(it)->name()) pcommas
        // ct_vgrn(Viewnode(from)->name())
        // pcommas ct_vgrn(Viewnode(with)->name()) pcommas
        // ct_vgrn(Viewnode(to)->name()) pend;
        // else: left release in some other window than "to_".
        // Deactivate dragging, no action taken
        // Deactivate float view.
        // flt_view_->SetDrawFunction(flt_draw_func_);
        flt_view_->Show(false);
      }
      // else: any other event than left release will deactivate dragging, and
      // releasing the left button after won't have any effect.
    } else {
      // If dragging was not started, pass the event to the child views, that's
      // what default handler event functions do.
      Handler::Mouse(v, button, x, y, pressed, button_state);
    }
  }
#else
  Handler::Mouse(v, button, x, y, pressed, button_state);
#endif
}

void GuiHandler::MouseMotion(pangolin::View &v, int x, int y,
                             int button_state) {

  if (from_view_) {
    // this means view has been selected for switching, selection is triggered
    // when left button is pressed, so button_state will have MouseButtonLeft
    // here.
    setFloatViewBounds(x, y);
  } else {
    Handler::MouseMotion(v, x, y, button_state);
  }
}

void GuiHandler::PassiveMouseMotion(pangolin::View &v, int x, int y,
                                    int button_state) {
  // glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, Window.col);
  // Window.pos[0] = x;
  // Window.pos[1] = y;

  pangolin::View *cv = v.FindChild(x, y);
  ViewNode *vn = NULL;
  if (cv != NULL) {
    while ((cv = cv->FindChild(x, y)) != NULL) {
      // added 160916:1427
      // View "address" doesnot exist..issue when mouse over the context_menu,
      // textview, parameters

      if (vn != NULL && ViewType::Container != vn->type())
        break;
      vn = Viewnode(cv);
      if (vn != NULL && ViewType::ThreeD == vn->type()) {
        glEnable(GL_DEPTH_TEST);
        pangolin::View &clv = vn->view(); // clicked view.
        GLfloat winz = clv.GetClosestDepth(x, y, 3);
        if (winz < 1.0f) {
          GLdouble *pos = new GLdouble[3];
          clv.GetObjectCoordinates(vn->get_ogl_render_state(), x, y, winz,
                                   pos[0], pos[1], pos[2]);
          // PT$ ct_vpur(winz) pcommas ct_vgrn(pos[0]) pcommas pos[1] pcommas
          // pos[2]
          // pendl;
          Window.info[0] = (float)pos[0];
          Window.info[1] = (float)pos[1];
          Window.info[2] = (float)pos[2];
          Window.info[3] =
              (float)sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
          Window.pos[0] = x;
          Window.pos[1] = y;
        } else {
          Window.pos[0] = -1;
          Window.pos[1] = -1;
        }
      } else if (vn != NULL && ViewType::TwoD == vn->type()) {
        unsigned char col[3];
        glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, col);
        if (col[0] == col[1] && col[0] == col[2]) {
          Window.info[0] = -1.0f;
          Window.info[1] = -1.0f;
          Window.info[2] = -1.0f;
          Window.info[3] = (float)col[0];
        } else {
          Window.info[0] = (float)col[0];
          Window.info[1] = (float)col[1];
          Window.info[2] = (float)col[2];
          Window.info[3] = -1.0f;
        }
        Window.pos[0] = x;
        Window.pos[1] = y;
      }
    }
  }

  Handler::PassiveMouseMotion(v, x, y, button_state);
}

bool GuiHandler::EmptyDrawFunction(pangolin::View &v) {
  v.Activate();
  // PV$ ct_vred(Window.sel_view_) pcommas ct_vred((Window.sel_view_ == NULL))
  // pend;
  if (Window.sel_view_ != NULL) {
    // PV$ "should be drawing in float" pend;
    Viewnode(Window.sel_view_)->draw(v);
  } else {
    // PV$ "float view shouldn't be drawn on" pend;
  }
  // Window.sel_view_->extern_draw_function(v);
  return RET_SUCCESS_BOOL;
}

void GuiHandler::move_float_view_last() {
  try {
    std::vector<pangolin::View *>::iterator flt_pos;
    flt_pos = std::find(pangolin::DisplayBase().views.begin(),
                        pangolin::DisplayBase().views.end(), flt_view_);
    if (std::next(flt_pos) != pangolin::DisplayBase().views.end())
      std::rotate(flt_pos, flt_pos + 1, pangolin::DisplayBase().views.end());
  } catch (std::exception e) {
    PE$ "Exception:Couldn't move float view to last. no biggie."
        << e.what() pendl;
  }
}
