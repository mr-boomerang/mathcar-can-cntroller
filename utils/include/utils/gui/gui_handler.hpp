#pragma once

#include <pangolin/pangolin.h>

//#include <utils/mode_printing.h>

#define VIEW_HEAD_HEIGHT 15
#define VIEW_HEAD_WIDTH_BORDER 10

#if 1
class GuiHandler : public pangolin::Handler {
public:
  GuiHandler(std::string flt_name);

  void Mouse(pangolin::View &v, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state);
  void MouseMotion(pangolin::View &v, int x, int y, int button_state);
  void PassiveMouseMotion(pangolin::View &v, int x, int y, int button_state);

  void SwitchViews(pangolin::View *it, pangolin::View *with,
                   pangolin::View *from, pangolin::View *to);
  // void Keyboard(pangolin::View&, unsigned char key, int x, int y,
  // bool pressed);
  bool EmptyDrawFunction(pangolin::View &v);

  pangolin::View *get_float_view() { return flt_view_; }

  void move_float_view_last();

private:
  /* data */
  std::string float_;
  int w_, h_;
  bool float_view_activated_;
  pangolin::View *from_view_;
  pangolin::View *flt_view_;

  std::function<void(pangolin::View &)> flt_draw_func_;

  /* functions */
  inline void setFloatViewBounds(int x, int y) {
    int l, b, r, t;
#if 0
    l = x - w_/2; r = l + w_;
    t = y + VIEW_HEAD_HEIGHT/3; b = t - h_;
#else
    // :haha: Solves issue when dragging near main window borders 040816:1444
    int float_window_width = 100;
    int float_window_height = 100;
    x = std::max(x, float_window_width / 2);
    y = std::max(y, float_window_height / 2);
    l = x - float_window_width / 2;
    r = l + float_window_width;
    t = y + float_window_height / 2;
    b = t - float_window_height;
#endif
    // PV$ ct_vblu(l) pcomma ct_vblu(r) pcomma ct_vblu(b) pcomma ct_vblu(t)
    // pend;
    // PV$ ct_vgrn(x) pcomma ct_vgrn(y) pend;
    flt_view_->SetBounds(pangolin::Attach::Pix(b), pangolin::Attach::Pix(t),
                         pangolin::Attach::Pix(l), pangolin::Attach::Pix(r));
  }
};
#endif
