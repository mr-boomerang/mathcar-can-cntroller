#include <utils/gui/textView.hpp>

using namespace std;
using namespace pangolin;

// TextViewFloating
TextViewFloating::TextViewFloating(std::string val,
                                   pangolin::Viewport &parent_vp)
    : View(), strVal(val) {
  parent_viewport = parent_vp;
  top = 1.0;
  bottom = 0.0; // Attach::Pix(-tab_h);
  left = 0.0;
  right = 1.0;
  v = parent_vp;
  hlock = LockLeft;
  vlock = LockBottom;
  gltext = font.Text(strVal);

  char_w = (font.Text("W")).Width();
  char_h = (font.Text("W")).Height();
  // space between two lines in multi-line case.
  space_h = 4;

  // white fore ground
  colour_fg[0] = 1.0;
  colour_fg[1] = 1.0;
  colour_fg[2] = 1.0;
  colour_fg[3] = 1.0;

  // black back ground
  colour_bg[0] = 0.0;
  colour_bg[1] = 1.0;
  colour_bg[2] = 0.0;
  colour_bg[3] = 1.0;

  raster[0] = v.l + 4;   // floor(v.l + (v.w-gltext.Width())/2.0f);
  raster[1] = v.b + v.h; // floor(v.b + (v.h-gltext.Height())/2.0f);

  start_b = v.b;
  end_b = v.b + v.h;
}
int TextViewFloating::setTextColor(float r, float g, float b, float a) {
  colour_fg[0] = r;
  colour_fg[1] = g;
  colour_fg[2] = b;
  colour_fg[3] = a;
  return RET_SUCCESS_INT;
}
int TextViewFloating::setBGColor(float r, float g, float b, float a) {
  colour_bg[0] = r;
  colour_bg[1] = g;
  colour_bg[2] = b;
  colour_bg[3] = a;
  return RET_SUCCESS_INT;
}

int TextViewFloating::do_pre_Render_stats() {
  CHECK_RET(do_str_stats());
  CHECK_RET(writeText(false));
  return RET_SUCCESS_INT;
}
void TextViewFloating::ResizeChildren() {
  v = parent_viewport;
  raster[0] = v.l + 4;
  raster[1] = v.b + v.h;
}

void TextViewFloating::Render() {
  v = parent_viewport;
  str_line_stats.clear();
  do_str_stats();
  glColor4fv(colour_fg);
  writeText();
}

GLfloat *TextViewFloating::getColor(int id, int bold) {
  static GLfloat colors[8][4] = {
      {0, 0, 0, 1}, // black
      {1, 0, 0, 1}, // red
      {0, 1, 0, 1}, // green
      {1, 1, 0, 1}, // yellow
      {0, 0, 1, 1}, // blue
      {1, 0, 1, 1}, // magenta
      {0, 1, 1, 1}, // cyan
      {1, 1, 1, 1}  // white

  };
  int col_id = 7;
  if (id >= 0 && id < 8) {
    col_id = id;
  }

  static GLfloat color[4];
  for (int k = 0; k < 4; k++)
    color[k] = colors[col_id][k];

  if (bold) {
    color[0] /= 2.0;
    color[1] /= 2.0;
    color[2] /= 2.0;
    // color[3]/=2.0;
  }
  if (col_id == 0) {
    color[0] = 0.5;
    color[1] = 0.5;
    color[2] = 0.5;
  }
  return color;

  // return colors[col_id];
}

int TextViewFloating::writeText(bool doWrite) {
  vector<str_stat> &stats = str_line_stats;
  int n_lines_possible_in_view = v.h / (char_h + space_h);
  GlText tempText;
  int start_y = v.b + v.h - char_h - space_h * 2;
  int n_lines = stats.size();
  end_b = v.b + v.h; //  start_y;

  for (int i = 0; i < stats.size(); i++) {
    // for text
    if (start_y <= parent_viewport.b)
      break;
    // for text background
    if (start_y - space_h <= parent_viewport.b)
      break;

    str_stat &t = stats[i];
    // tempText = font.Text( strVal.substr(t.start_ind, t.n_chars));
    if (doWrite) {
#if 0
			glColor4fv(colour_fg);
			tempText.DrawWindow(raster[0],start_y );
#else
      int tsi = t.start_ind;
      int fg_color = 7;
      int bg_color = 0;
      int text_prop = 0;

      int j = 0;
      string tmpstr = strVal.substr(t.start_ind, t.n_chars);
      int start_raster_x = raster[0];
      int nchars_drawn = 0;
      int nchars_to_draw = t.n_chars;
      while (1) {
        if (tsi > t.start_ind + t.n_chars)
          break;

        while (j < color_stats.size() && color_stats[j].str_start_ind <= tsi) {
          j++;
        }
        if (j == 0) //.. default color/ prev color
        {
          // use fg_color,bg_color , text_prop

        } else if (j > 0) {
          j = j - 1;
          fg_color = color_stats[j].fg_color;
          bg_color = color_stats[j].bg_color;
          text_prop = color_stats[j].text_prop;
        }
        j++;
        if (j < color_stats.size()) {

          if (false && color_stats[j].str_start_ind > t.start_ind + t.n_chars) {
            tmpstr = strVal.substr(tsi, (t.start_ind + t.n_chars -
                                         (color_stats[j].str_start_ind)));
          } else {
            int nk = color_stats[j].str_start_ind - tsi;
            if (nk + tsi > t.start_ind + t.n_chars) {
              nk = t.start_ind + t.n_chars - tsi;
            }
            tmpstr = strVal.substr(tsi, nk);
          }
          nchars_drawn += tmpstr.length();

          if (nchars_drawn > t.n_chars) {
            tmpstr = tmpstr.substr(0, t.n_chars - nchars_drawn);
          }
          tsi = color_stats[j].str_start_ind;
        } else {
          break;
        }

        GlText tempGlText = font.Text(tmpstr);

        // draw text-background
        // glColor4fv( getColor( bg_color) );
        GLfloat *bgcolor = getColor(bg_color, 1);
        glColor4fv(bgcolor);
        pangolin::Viewport text_vp;
        text_vp.l = start_raster_x;
        text_vp.w = tempGlText.Width();
        text_vp.b = start_y - space_h;
        text_vp.h = char_h + 2 * space_h;
        glRect(text_vp);

        // draw text
        GLfloat *color = getColor(fg_color, text_prop);
        glColor4fv(color);
        tempGlText.DrawWindow(start_raster_x, start_y);
        start_raster_x += tempGlText.Width();

        if (tsi > t.start_ind + t.n_chars)
          break;
      }
      // draw remaining string...if any
      if (nchars_drawn < t.n_chars) {
        tmpstr =
            strVal.substr(t.start_ind + nchars_drawn, t.n_chars - nchars_drawn);
        GlText tempGlText = font.Text(tmpstr);
        // draw text-background
        GLfloat *bgcolor = getColor(bg_color, 1);
        glColor4fv(bgcolor);
        pangolin::Viewport text_vp;
        text_vp.l = start_raster_x;
        text_vp.w = tempGlText.Width();
        text_vp.b = start_y - space_h;
        text_vp.h = char_h + 2 * space_h;
        glRect(text_vp);
        // draw text
        glColor4fv(getColor(fg_color, text_prop));
        tempGlText.DrawWindow(start_raster_x, start_y);
      }

#endif
    }
    start_y = start_y - char_h - space_h * 2;
  }
  // start_b = start_y;
  start_b = start_y + char_h + space_h;

  return RET_SUCCESS_INT;
}

std::string TextViewFloating::removeColorEsc(string str) {
  // color_stats.clear();
  string outStr = "";
  while (str.length() > 0) {
    int i = str.find("\033\[");
    int j = -1;

    if (i != string::npos) {
      while (i < str.length() && str.at(i) != '[')
        i++;
      i--;
      j = i;
      bool reachedM = false;
      int ncolons = 0;
      bool reachedSemiColon = false;
      int mi = -1;
      int coloni = -1;
      int text_prop = 0;
      int fg_color = 7;
      int bg_color = 0;
      for (int i1 = i; i1 < str.length() && (i1 - i) < 10; i1++) {
        if (str.at(i1) == 'm') {
          mi = i1;
          reachedM = true;
          break;
        } else if (str.at(i1) == ';') {
          ncolons++;
          reachedSemiColon = true;
          coloni = i1;
        }
      }
      if (reachedM) {
        if (!reachedSemiColon) // && (mi-i)==2)
        {
          j = mi;
        } else if (reachedSemiColon)
          j = mi;
        if (reachedSemiColon) {
          if (ncolons == 1) {
            // expecting format \033[0;34m
            if ((coloni - i) == 2)
              text_prop = str.at(coloni - 1) - '0';
            if ((mi - 1 - coloni) == 2) {
              if (str.at(coloni + 1) == '3') {
                fg_color = str.at(coloni + 2) - '0';

              } else if (str.at(coloni + 1) == '4') {
                bg_color = str.at(coloni + 2) - '0';
              }
            }

          } else if (ncolons == 2) {
            // expecting format \033[0;34;44m
            // TODO
            //
          }
        }

        outStr.append(str.substr(0, i));
        color_stats.push_back(str_color_stat());
        str_color_stat &temp_color_stat = color_stats[color_stats.size() - 1];
        temp_color_stat.str_start_ind = outStr.length();
        temp_color_stat.text_prop = text_prop;
        temp_color_stat.fg_color = fg_color;
        temp_color_stat.bg_color = bg_color;
      }
    } else
      break;

    str = str.substr(j + 1);
  }
  if (str.length() > 0)
    outStr.append(str);
#if 0

	cout << " color_stats : " << color_stats.size() << endl;
	for( int i=0;i<color_stats.size() ; i++)
	{
		str_color_stat &scs =color_stats[i];
		cout << " " << i << " startind " << scs.str_start_ind << " , fg : " << scs.fg_color << " bg : " << scs.bg_color << endl;
	}
#endif

  return outStr;
}

int TextViewFloating::do_str_stats() {
  std::vector<str_stat> &str_stats = str_line_stats;

  strVal = removeColorEsc(strVal);
  str_stats.clear();
  str_stat temp_stat;
  GlText tempText = font.Text(strVal);
  int str_len = strVal.length();
  temp_stat.start_ind = 0;
  temp_stat.end_ind = str_len - 1;
  temp_stat.n_chars = str_len;

  int new_line_pos = strVal.find('\n');

  if (new_line_pos == std::string::npos) {
    str_stats.push_back(temp_stat);
    return RET_SUCCESS_INT;
  }
  // has new line chars..
  std::string tempStr = strVal;
  int view_w = v.w - 4;
  int start_ind = 0;

  while (new_line_pos != std::string::npos) {
    std::string tempStr1 = tempStr.substr(0, new_line_pos);

    tempText = font.Text(tempStr1);
    int t_w = tempText.Width();

    int n_lines_for_this_str = 0;
    int text_w = char_w * (int)(tempStr1.length());

    n_lines_for_this_str = (text_w / view_w) + ((text_w % view_w) > 0);

    int strlen_w = view_w / char_w; // + ((view_w%char_w)>0);
    int strl_left = tempStr1.length();

    int start_ind_1 = start_ind;
    for (int i = 0; i < n_lines_for_this_str && start_ind < strVal.length();
         i++) {
      temp_stat.start_ind = start_ind;
      temp_stat.n_chars = std::min(strlen_w, strl_left);

      start_ind += temp_stat.n_chars;
      str_stats.push_back(temp_stat);

      strl_left -= strlen_w;
    }
    if (new_line_pos >= tempStr.length())
      break;
    tempStr = tempStr.substr(1 + new_line_pos);
    start_ind = start_ind_1 + new_line_pos + 1;
    new_line_pos = tempStr.find('\n');
    if (new_line_pos == std::string::npos && tempStr.length() > 0) {
      new_line_pos = tempStr.length();
    }
  }

  return RET_SUCCESS_INT;
}

// MultiLineText
void MultiLineText::ResizeChildren() {
  // background for this view
  static GLfloat bg_color[4] = {0.0f, 0.5f, 0.0f, 1.0f};
  glColor4fv(bg_color);
  glRect(v);
#if 0
	static int ii = 0;
	pangolin::Attach tempa;
	if(ii%100 == 0)
	{	
	cout << " viewport MultiLineText : " << v.b << "," << v.t() << "," << v.l << "," << v.r() << " w : " << v.w << " h : " << v.h  << endl;
	//tempa = pangolin::Attach(pangolin::Pixel,right.p);
	tempa = pangolin::Attach::Frac(right.p);
	pangolin::Attach tempb = pangolin::Attach::Pix(v.r());
	cout << " left : " << left.p << " , " << top.p << " " << bottom.p << " " << right.p << " : " << tempa.p << " ::: "<< tempb.p << endl;
	}
	ii++;
	pangolin::Viewport vt1;
	//36,324,704,1216
	vt1.l = v.l;
	vt1.b = v.b;
	vt1.w = v.w;
	vt1.h = v.h;
	vt1 = GetBounds();
	/*
	vt1.l = 0;
	vt1.b = 0;
	vt1.w = 1216-704;//v.w;
	vt1.h = 324-36;//v.h;
	*/

	//vt1.l = pangolin::Attach::Pix(left).p;
	//vt1.l = pangolin::Attach::Pix(left).p;
	//vt1.b = v.b;
	//vt1.w = v.w;
	//vt1.h = v.h;
	//cout << " left : " << left.p << " , " << top.p << " " << bottom.p << " " << right.p << endl;

	glRect(vt1);
	//prev_vp = v;
#endif

  int n_views_max = 0;
  int last_b;
  int tot_v_h = 0;
  pangolin::Viewport v1 = v;

  int i = 0;
  // make sure scroll_offset is within limits
  if (scroll_offset < 0)
    scroll_offset = 0;
  if (scroll_offset > textViews.size() - 1)
    scroll_offset = textViews.size() - 1;
  int so = scroll_offset;
  // from scroll_offset find number of textviews that can fit in this->v
  //
  for (i = so; i < textViews.size(); i++) {
    textViews[i]->parent_viewport = v1;
    textViews[i]->ResizeChildren();
    textViews[i]->do_pre_Render_stats();
    last_b = textViews[i]->start_b;
    tot_v_h += (textViews[i]->end_b - textViews[i]->start_b);
    n_views_max++;
    if (tot_v_h >= v.h)
      break;
  }

  // if selected text views do not fill this->v completely
  // ... look for textviews above the scroll offset..
  //
  if (tot_v_h < v.h && n_views_max < textViews.size()) {
    // means less views shown
    int j = so - 1;
    for (; j >= 0; j--) {
      textViews[j]->parent_viewport = v1;
      textViews[j]->ResizeChildren();
      textViews[j]->do_pre_Render_stats();
      last_b = textViews[j]->start_b;
      tot_v_h += (textViews[j]->end_b - textViews[j]->start_b);
      n_views_max++;
      if (tot_v_h >= v.h)
        break;
    }
    so = j + 1;
  }

  // once the optimum scroll offset is found...
  scroll_offset = so;
  int k = 0;
  // do not show the text views above scroll offset
  for (k = 0; k < so; k++) {
    textViews[k]->show = false;
  }
  v1 = v;
  // adjust viewport, render and show the textviews
  for (k = so; k < i; k++) {
    textViews[k]->show = true;
    if (v1.h <= 0) {
      textViews[k]->show = false;
      // continue;
      break;
    }
    textViews[k]->parent_viewport = v1;
    textViews[k]->ResizeChildren();
    textViews[k]->Render();
    // textViews[i]->do_pre_Render_stats();
    v1.h = textViews[k]->start_b - vp.b;
  }
  // do not show the text views below scroll offset
  for (int k = i; k < textViews.size(); k++) {
    textViews[k]->show = false;
  }
}

int MultiLineText::addLine(std::string strText) {
  tp = new TextViewFloating(strText, v);
  tp->SetBounds(0.0, 0.5, 0.0, 1.0);
  tp->show = true;
  tp->setTextColor(1, 1, 1, 1);
  tp->setBGColor(1, 0, 1, 1);

  if (textViews.size() > 0) {
    pangolin::Viewport &tp_vp = textViews[textViews.size() - 1]->vp;
    tp->parent_viewport.h = textViews[textViews.size() - 1]->start_b - vp.b;
  }
  textViews.push_back(tp);
  /* limit no. of text views in UI (limits the memory usage) */
  if (textViews.size() > 1000) {
    delete textViews[0];
    textViews.erase(textViews.begin());
  }
  scroll_offset = textViews.size() - 1;
  return RET_SUCCESS_INT;
}

void MultiLineText::MouseMotion(View &d, int x, int y, int button_state) {
  if (x < d.v.l || y < d.v.b || x > (d.v.r()) || y > d.v.t())
    return;

  if (dragStarted) {
    if (prev_y_for_drag - y > 0)
      d.scroll_offset += 1;
    else {
      d.scroll_offset -= 1;
    }
    if (d.scroll_offset < 0)
      d.scroll_offset = 0;
    if (d.scroll_offset >= textViews.size())
      d.scroll_offset = textViews.size() - 1;
    ResizeChildren();
  }
  prev_y_for_drag = y;
}
void MultiLineText::Mouse(View &d, MouseButton button, int x, int y,
                          bool pressed, int button_state) {

  if (!pressed && (button == MouseWheelUp || button == MouseWheelDown)) {
    if (button == MouseWheelUp) {
      d.scroll_offset -= 1;
    } else if (button == MouseWheelDown) {
      d.scroll_offset += 1;
    }
    if (d.scroll_offset < 0)
      d.scroll_offset = 0;
    if (d.scroll_offset >= textViews.size())
      d.scroll_offset = textViews.size() - 1;
    ResizeChildren();
  } else if (pressed && button == pangolin::MouseButtonLeft) {
    if (y < (v.b + v.h) / 2)
      d.scroll_offset += 1;
    else {
      d.scroll_offset -= 1;
    }
    if (d.scroll_offset < 0)
      d.scroll_offset = 0;
    if (d.scroll_offset >= textViews.size())
      d.scroll_offset = textViews.size() - 1;
    ResizeChildren();
    dragStarted = true;
  }

  else if (button == pangolin::MouseButtonLeft) {
    // Mouse UP
    dragStarted = false;
  } else {
    Handler::Mouse(d, button, x, y, pressed, button_state);
  }
}
