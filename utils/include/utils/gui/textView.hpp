

#pragma once

#include <iostream>
#include <pangolin/pangolin.h>
#include <utils/check_ret.h>
#include <utils/error_code.h>
#include <vector>

/* floating..adjusting height with data...breaking into multiline...
 like wordwrap thing...
*/

class TextViewFloating : public pangolin::View {

private:
  std::string strVal;

public:
  int char_w; // = (font.Text("W")).Width();
  int char_h;
  int space_h;
  pangolin::Viewport parent_viewport;
  GLfloat colour_bg[4]; //= {0.9f, 0.9f, 0.9f, 1.0f};
  GLfloat colour_fg[4]; //= {1.0f, 1.0f, 1.0f, 1.0f};

  pangolin::GlFont &font = pangolin::GlFont::I();

  pangolin::GlText gltext;
  GLfloat raster[2];
  int end_b;
  int start_b;
  TextViewFloating(pangolin::Viewport &parent_vp) //: View(), strVal("")
  {
    TextViewFloating("", parent_vp);
  }

  TextViewFloating(std::string val, pangolin::Viewport &parent_vp);

  int setTextColor(float r, float g, float b, float a = 1.0);
  int setBGColor(float r, float g, float b, float a = 1.0);

  void glRect(pangolin::Viewport v) {
    return;
    // :haha: 300816:1513... issue drawing rectangle..with multiple views and so
    // disabled

    GLfloat vs[] = {(float)v.l,   (float)v.b,   (float)v.l,   (float)v.t(),
                    (float)v.r(), (float)v.t(), (float)v.r(), (float)v.b};

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, vs);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
  }

  void glRect(pangolin::Viewport v, int inset) { glRect(v.Inset(inset)); }

  // structure to hold start and end indices of strVal
  // for each line to be displayed..in case of multiline
  // spanning strVal.
  struct str_stat {
    int start_ind;
    int end_ind;
    int n_chars;
  };
  std::vector<str_stat> str_line_stats;

  // structure to hold color and text properties
  // when string contains terminal text and background
  // color representing escape sequences.

  struct str_color_stat {
    int str_start_ind;
    int str_end_ind;
    int text_prop;
    int fg_color;
    int bg_color;

    str_color_stat() {
      str_start_ind = 0;
      str_end_ind = 0;

      text_prop = 0;
      fg_color = 0;
      bg_color = 0;
    }
  };

  std::vector<str_color_stat> color_stats;

  // parse color representing escape sequences, update color_stats
  // and remove these characters from string to be rendered.

  std::string removeColorEsc(std::string str);

  // parse strVal ( for possible color escape chars) and
  // break string into multiline if possible.

  int do_str_stats();

  // Returns GLfloat[4] array pointer representing r,g,b,a values
  // possible colors returned are
  // black,red,green,yellow,blue,magenta,cyan,white
  // for id ranging from 0-black,1-red, and so on utpo 7-white
  GLfloat *getColor(int id, int bold = 0);

  // Render text on view
  int writeText(bool doWrite = true);

  // compute statistics without being text rendered..
  // useful when view width/height or some info required
  // may be for ajusting..but Rendering not required.
  int do_pre_Render_stats();

  // Draws text on view (Overridden from pangolin::View)
  void Render();

  // (Overridden from pangolin::View)
  void ResizeChildren();

  // Append string to existing strVal
  int appendStr(std::string &str) {
    strVal.append(str);
    return RET_SUCCESS_INT;
  }
  int appendStr(std::string str) {
    strVal.append(str);
    return RET_SUCCESS_INT;
  }
  int appendStr(const char *str) {
    strVal.append(str);
    return RET_SUCCESS_INT;
  }
};

//
class MultiLineText : public pangolin::View, public pangolin::Handler {

public:
  pangolin::Viewport prev_vp;
  std::vector<TextViewFloating *> textViews;
  TextViewFloating *tp;

  bool dragStarted = false;
  int prev_y_for_drag = 0;

  MultiLineText() //:View()
  {
    layout = pangolin::LayoutVertical;
    SetHandler(this);

    prev_vp.l = 0;
    prev_vp.b = 0;
    prev_vp.w = 0;
    prev_vp.h = 0;
    // top = 1.0; bottom = 0.0;
    // left = 0.0; right = 1.0;
    // hlock = LockLeft;
    // vlock = LockBottom;
  }

  void glRect(pangolin::Viewport v) {
    return;
    // :haha: 300816:1513... issue drawing rectangle..with multiple views and so
    // disabled
    GLfloat vs[] = {(float)v.l,   (float)v.b,   (float)v.l,   (float)v.t(),
                    (float)v.r(), (float)v.t(), (float)v.r(), (float)v.b};

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, vs);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
  }

  void Render() { ResizeChildren(); }
  void Resize() { ResizeChildren(); }
  void ResizeChildren();
  int addTextViewFloating(std::string strText) {
    addLine(strText);
    return RET_SUCCESS_INT;
  }
  int addLine(std::string strText);
  // Mouse Events
  void MouseMotion(View &d, int x, int y, int button_state);
  void Mouse(View &d, pangolin::MouseButton button, int x, int y, bool pressed,
             int button_state);
};
