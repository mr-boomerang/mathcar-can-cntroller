#pragma once

#include <pangolin/pangolin.h>
static int temp_x, temp_y, temp_w, temp_h, temp_l, temp_b;
class Handler2DMultiView : public pangolin::Handler {
public:
  Handler2DMultiView() {}

  void Mouse(pangolin::View &v, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state) {
    pn = Viewnode(&v)->get_parent()->name();
    if (pressed && button == pangolin::MouseButtonLeft) {
      left_x_ = x - v.vp.l;
      bot_y_ = y - v.vp.b;
      temp_x = left_x_;
      temp_y = bot_y_;
      temp_w = v.vp.w;
      temp_h = v.vp.h;
      temp_l = v.vp.l;
      temp_b = v.vp.b;
      active = true;
    } else if (!pressed && button == pangolin::MouseButtonLeft) {
      active = false;
    } else {
      Handler::Mouse(v, button, x, y, pressed, button_state);
    }
  }
  void MouseMotion(pangolin::View &v, int x, int y, int button_state) {
    if (active) {
      pangolin::Viewport new_vp = v.vp;
      l_ = x - left_x_;
      b_ = y - bot_y_;
      diff_x_ = 0;
      diff_y_ = 0;
      if (((float)temp_x > 0.15f * temp_w && (float)temp_x < 0.85f * temp_w) &&
          ((float)temp_y > 0.15f * temp_h && (float)temp_y < 0.85f * temp_h)) {
        if (l_ >= 0 && l_ <= View(pn).vp.w - v.vp.w && b_ >= 0 &&
            b_ <= View(pn).vp.h - v.vp.h) {
          l_ = l_ / View(pn).vp.w;
          b_ = b_ / View(pn).vp.h;
          v.SetBounds(
              pangolin::Attach::Frac(b_),
              pangolin::Attach::Frac(b_ + (float)v.vp.h / View(pn).vp.h),
              pangolin::Attach::Frac(l_),
              pangolin::Attach::Frac(l_ + (float)v.vp.w / View(pn).vp.w));
        }
      } else {
        if ((float)temp_x < 0.15f * temp_w) {
          diff_x_ = new_vp.l - x;
          new_vp.l = x;
        }
        if ((float)temp_x > 0.85f * temp_w) {
          diff_x_ = x - (new_vp.l + new_vp.w);
        }
        if ((float)temp_y < 0.15f * temp_h) {
          diff_y_ = new_vp.b - y;
          new_vp.b = y;
        }
        if ((float)temp_y > 0.85f * temp_h) {
          diff_y_ = y - (new_vp.b + new_vp.h);
        }
        if (l_ >= 0 && l_ <= View(pn).vp.w - v.vp.w && b_ >= 0 &&
            b_ <= View(pn).vp.h - v.vp.h)
          v.SetBounds(pangolin::Attach::Pix(new_vp.b),
                      pangolin::Attach::Pix(new_vp.b + new_vp.h + diff_y_),
                      pangolin::Attach::Pix(new_vp.l),
                      pangolin::Attach::Pix(new_vp.l + new_vp.w + diff_x_));
      }
    } else {
      Handler::MouseMotion(v, x, y, button_state);
    }
  }

private:
  /*data*/
  int left_x_, bot_y_;
  bool active = false;
  float l_, b_;
  int diff_x_ = 0, diff_y_ = 0;
  std::string pn;
};
