#include <iostream>
#include <thread>
#include <chrono>

#include <pangolin/pangolin.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define PAP pangolin::Attach::Pix
#define PAF pangolin::Attach::Frac

#define WIN_W 1024
#define WIN_H 768

#define VIEW_HEAD_HEIGHT 15
#define VIEW_HEAD_BORDER 10

pangolin::GlTexture img_tex_;

bool draw_margins(pangolin::View& v) {
  /* Margin design
   * -----------------------------
   * |_Viewname_______/           |
   * |                            |
   * |                            |
   * |                            |
   * |                            |
   * -----------------------------
   * */

  try {

    /* Opengl settings */
    v.ActivatePixelOrthographic();

    pangolin::GlState s;
    s.glDisable(GL_DEPTH_TEST);
    s.glDisable(GL_LIGHTING);

    //if(Container == type_) {
      //glColor4f(0.564f, 0.831f, 0.709f, 0.5f);
      //glLineWidth(2.0f);
      //pangolin::glDrawLine(0,v.vp.h-3,v.vp.w,v.vp.h - 3);
      //glLineWidth(1.0f);
    //}
    //else {
      glColor4f(0.4f, 0.0f, 1.0f, 0.5f);
      /* Rectangle margin for whole view */
      pangolin::glDrawRectPerimeter(0,0, v.vp.w, v.vp.h);

      /* Rectangle for name */
      // for width of rectangle get the text width.
      pangolin::GlText txt = pangolin::GlFont::I().Text("view");
      int w = txt.Width();
      float head_top = v.vp.h - 0.3*VIEW_HEAD_HEIGHT;
      float head_bot = v.vp.h - 2.3*VIEW_HEAD_HEIGHT+VIEW_HEAD_BORDER;
      float head_left = 0;
      float head_right = w + 2*VIEW_HEAD_BORDER;
      pangolin::glDrawRect(head_left, head_top, head_right, head_bot);

      /* Triangle next to name */
      glBegin(GL_TRIANGLES);
      glVertex2i(head_right, head_top+1);
      glVertex2i(head_right, head_bot);
      //head_top + 1 shouldn't be required, o.w. triangle is a little tilted.
      glVertex2i(head_right + 1.5*VIEW_HEAD_BORDER, head_top+1);
      glEnd();

      /* Name */
      glColor3f(0.0f, 0.0f, 0.0f);
      txt.DrawWindow(v.vp.l + VIEW_HEAD_BORDER, v.vp.b + v.vp.h - VIEW_HEAD_HEIGHT);

      /* Reset */
      glColor3f(1.0f, 1.0f, 1.0f);

    //}

  }
  catch(std::exception e) {
    return false;
  }

  return true;
}

bool draw(pangolin::View& v) {
  v.Activate();
  glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
  //glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  if(img_tex_.tid) {
    img_tex_.RenderToViewportFlipY();
  }
  return draw_margins(v);
}

int main(void)
{
  pangolin::CreateWindowAndBind("test", WIN_W, WIN_H);
  glEnable(GL_DEPTH_TEST);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  cv::Mat img = cv::imread("/home/laxit/data/fisheye/2928323892_06303c17a8_o.jpg");
  cv::cvtColor(img, img, CV_BGR2RGB);
  img_tex_.Reinitialise(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE);
  img_tex_.Upload(img.data, GL_RGB, GL_UNSIGNED_BYTE);

  pangolin::View& v_main = pangolin::Display("Main");
  v_main.SetBounds(PAF(0.0f), PAF(2/3.0f), PAF(0.0f), PAF(1.0f));
  v_main.SetLayout(pangolin::LayoutEqualHorizontal);

  pangolin::View& v_panel = pangolin::Display("view_panel");
  v_panel.SetBounds(PAF(2/3.0f), PAF(1.0f), PAF(0.0f), PAF(1.0f));
  v_panel.SetLayout(pangolin::LayoutEqualHorizontal);

  pangolin::View& v2d_1 = pangolin::Display("view2d_1");
  pangolin::View& v2d_2 = pangolin::Display("view2d_2");
  pangolin::View& v2d_3 = pangolin::Display("view2d_3");
  pangolin::View& v2d_4 = pangolin::Display("view2d_4");

  //v2d_1.SetDrawFunction(&draw_margins);
  //v2d_2.SetDrawFunction(&draw_margins);
  //v2d_3.SetDrawFunction(&draw_margins);
  //v2d_4.SetDrawFunction(&draw_margins);
  v2d_1.SetDrawFunction(&draw);
  v2d_2.SetDrawFunction(&draw);
  v2d_3.SetDrawFunction(&draw);
  v2d_4.SetDrawFunction(&draw);

  //v2d_1.SetBounds(0.0f, 1.0f, 0.0f, 0.5f);
  //v2d_2.SetBounds(0.0f, 1.0f, 0.5f, 1.0f);
  //v2d_3.SetBounds(0.0f, 1.0f, 0.0f, 0.5f);
  //v2d_4.SetBounds(0.0f, 1.0f, 0.5f, 1.0f);

  v_main.AddDisplay(v2d_1);
  v_main.AddDisplay(v2d_2);

  v_panel.AddDisplay(v2d_3);
  v_panel.AddDisplay(v2d_4);

  while(!pangolin::ShouldQuit()) {
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pangolin::FinishFrame();
  }

  return 0;
}
