#include <utils/gui/objects_2d.hpp>

#include <utils/fs_utils.h>

#ifdef BUILT_WITH_ROS
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#endif

/*--------------------------ImageObject Start---------------------------------*/

#ifdef UTILS_USE_OPENCV
#include <opencv2/imgproc/imgproc.hpp>
int ImageObject::reinit(const cv::Mat &img) {
  GLint frmt = GL_LUMINANCE;
  if (!img.empty()) {
    if (img.channels() == 3) {
      cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
      frmt = GL_RGB;
    }
    return reinit(img.data, img.cols, img.rows, frmt);
  } else
    return RET_IMAGE_NULL;
}
#endif

int ImageObject::reinit(unsigned char *img, int w, int h, GLint format) {
  try {
    if (img == NULL || !w || !h) // Not full proof, but something.
      return RET_IMAGE_NULL;

    // first time
    // OR (already intialized AND w OR h are not same as before)
    // first time img_tex_.width and height would be 0.
    if (w != img_tex_.width || h != img_tex_.height) {
      img_tex_.Reinitialise(w, h, format, GL_UNSIGNED_BYTE);
      CHECK_GL_ERROR("while reinitialising.");
    }
    img_tex_.Upload(img, format, GL_UNSIGNED_BYTE);
    CHECK_GL_ERROR("while uploading.");
  } catch (std::exception e) {
    PE$ ct_ylw("Exception: ") << e.what() << std::endl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

// This function is called when iterator in ViewNode draws the objects.
int ImageObject::draw_obj(pangolin::View & /*v*/) {
  glColor4f(1.0f, 1.0f, 1.0f, transparency_);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  if (img_tex_.tid) {
    img_tex_.RenderToViewportFlipY();
  }
  CHECK_GL_ERROR("");
  return RET_SUCCESS_INT;
}
/*--------------------------ImageObject End-----------------------------------*/

/*--------------------------ImageStreamObject Start---------------------------*/

int ImageStreamObject::reinit(int w, int h) {
  try {
    if (!w || !h)
      return RET_INVALID_IMAGE;
    format_ = GL_RGB;
    if (w != img_tex_.width || h != img_tex_.height) {
      img_tex_.Reinitialise(w, h, format_, GL_UNSIGNED_BYTE);
      CHECK_GL_ERROR("");
    }
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
  }
  return RET_SUCCESS_INT;
}

// This function is called when iterator in ViewNode draws the objects.
int ImageStreamObject::draw_obj(pangolin::View & /*v*/) {
  glColor4f(1.0f, 1.0f, 1.0f, transparency_);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  if (img_tex_.tid) {
#ifdef UTILS_USE_OPENCV
    if (vc_.isOpened() && vc_.grab()) {
      cv::Mat img;
      vc_.retrieve(img);
      CHECK_RET(update_img(img));
    }
#endif
    mtx_.lock();
    img_tex_.RenderToViewportFlipY();
    mtx_.unlock();
    CHECK_GL_ERROR("");
  }
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  return RET_SUCCESS_INT;
}

int ImageStreamObject::update_img(const unsigned char *img, int w, int h) {
  if (img == NULL)
    return RET_IMAGE_NULL;
  if (w != img_tex_.width || h != img_tex_.height)
    CHECK_RET(reinit(w, h));
  mtx_.lock();
  img_tex_.Upload(img, format_, GL_UNSIGNED_BYTE);
  mtx_.unlock();
  return RET_SUCCESS_INT;
}

#ifdef UTILS_USE_OPENCV

ImageStreamObject::ImageStreamObject(const std::string &vid_path)
    : Objectxd(TwoDObject) {
  if (utils::file_accessible(vid_path, utils::FileAccessType::Read))
    reinit(cv::VideoCapture(vid_path));

  else
    PE$ "Cannot read the video file " << ct_bred(vid_path) << pendo;
}

int ImageStreamObject::update_img(const cv::Mat &img) {
  if (!img.empty() && img.type() == CV_8UC3) {
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    return update_img(img.data, img.cols, img.rows);
  } else
    return RET_IMAGE_NULL;
}

int ImageStreamObject::reinit(const cv::VideoCapture &vid) {
  if (!vid.isOpened()) {
    PE$ "cv::VideoCapture object passed hasn't opened an video." << pendo;
    return RET_VIDEO_CAPTURE_NOT_OPENED;
  }
  vc_ = vid;
  return reinit((int)vc_.get(cv::CAP_PROP_FRAME_WIDTH),
                (int)vc_.get(cv::CAP_PROP_FRAME_HEIGHT));
}
#endif

#ifdef BUILT_WITH_ROS

int ImageStreamObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                              uint32_t queue_sz) {
  sub_ = n.subscribe(topic_sub, queue_sz, &ImageStreamObject::callback, this);
  return RET_SUCCESS_INT;
}

void ImageStreamObject::callback(const sensor_msgs::ImageConstPtr &img_sub) {
  if (img_sub->encoding == sensor_msgs::image_encodings::BGR8) {
    format_ = GL_BGR;
  } else if (img_sub->encoding == sensor_msgs::image_encodings::RGB8) {
    format_ = GL_RGB;
  }
  update_img(img_sub->data.data(), img_sub->width, img_sub->height);
}

#endif

/*--------------------------ImageStreamObject End-----------------------------*/

/*--------------------------TextObject Start-----------------------------*/

int TextObject::reinit(const utils::Color &fc, const utils::Color &bgc,
                       const std::string &text) {
  PRINT_FUNC_ENTER;
  const int sn = 430;
  view_w_ = 0;
  view_h_ = 0;
  left_border_ = 5;
  bottom_border_ = 5;
  line_spacing_ = 2;
  start_idx_ = 0;
  end_idx_ = (int)draw_text_.size();
  PI$ ct_vred(&fc) pcommas ct_vred(&bgc) pendl;
  PI$ ct_vred(&fc_) pcommas ct_vred(&bgc_) pendl;
  fc_ = fc;
  bgc_ = bgc;

  char_w_ = fnt_.Text("W").Width();
  // line_spacing_ is a padding for vertical text seperation,
  // gaps between the line. It's added here as char_h_ is used only in draw_obj_
  // which is the function with highest call frequency.
  char_h_ = fnt_.Text("W").Height() + (float)line_spacing_;

  if (text.size())
    CHECK_RET(update(text));

  if (v_ != NULL)
    v_->SetHandler(new TextScrollHandler(this));

  SP$(sn) ct_vpur(fc_) pcommas ct_vpur(bgc_) pendl;
  SP$(sn)
  ct_vpur(view_w_) pcommas ct_vpur(view_h_) pcommas ct_vpur(char_w_)
      pcommas ct_vpur(char_h_) pendl;

  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

int TextObject::update(const std::string &text, bool append) {
  PRINT_FUNC_ENTER;
  text_.push_back(text);
  CHECK_RET(do_str_stats());
  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

#if 0
std::string TextObject::removeColorEsc(const std::string &str) {
  // color_stats.clear();
  std::string outStr = "";
  while (str.length() > 0) {
    int i = str.find("\033\[");
    int j = -1;

    if (i != std::string::npos) {
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

    //str = str.substr(j + 1);
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
#endif

int TextObject::do_str_stats() {
  PRINT_FUNC_ENTER;
  const int sn = 431;
  if (v_ == NULL) {
    PW$ "v_ shoudln't be null. Not updating stats. Did you add the object to a "
        "ViewNode?" pendl;
    return RET_TEXT_VIEW_NULL;
  }

  draw_text_.clear();
  view_w_ = v_->vp.w;
  view_h_ = v_->vp.h;
  // this is set to top, then come View head border down and then come text
  // border down.
  text_top_limit_ =
      view_h_ - VIEW_HEAD_HEIGHT - 2 * VIEW_HEAD_BORDER - bottom_border_;
  int line_num_chars = (int)((view_w_ - left_border_) / char_w_);

  for (unsigned int i = 0; i < text_.size(); ++i) {

    std::string &str = text_[i];
    if (str == "") { // if some one wants to add empty line in display, it
                     // should
      draw_text_.push_back(fnt_.Text(str));
    }

    std::vector<std::string> str_split_n =
        utils::split_str<std::string>(str, '\n');
    SP$(sn) ct_vpur(str) pcommas ct_vpur(str_split_n.size()) pendl;
    PRINT_VECTOR_SEC(sn, str_split_n);

    for (unsigned int i = 0; i < str_split_n.size(); ++i) {
      std::string str_break_n = utils::trim(str_split_n[i]); // string till "\n"
      SP$(sn)
      ct_vpur(line_num_chars) pcommas ct_vpur(str_break_n)
          pcommas ct_vpur(str_break_n.size()) pendl;

      int idx = 0;
      std::string line;
      while (idx < (int)str_break_n.size()) {
        if ((idx + line_num_chars) < (int)str_break_n.size()) {
          std::string sub_sbn = str_break_n.substr(idx, line_num_chars);
          int break_idx = sub_sbn.rfind(" ");
          line = sub_sbn.substr(0, break_idx);
          idx += (break_idx + 1);
        } else {
          line = str_break_n.substr(idx); // from idx to string end.
          idx = (int)str_break_n.size();
        }
        draw_text_.push_back(fnt_.Text(line));
      }
    }
  }

  total_text_h_ = (float)draw_text_.size() * char_h_;

  if ((int)total_text_h_ > text_top_limit_) {
    end_idx_ = (int)draw_text_.size();
    start_idx_ = end_idx_ - (int)((text_top_limit_) / char_h_);

  } else {
    start_idx_ = 0;
    end_idx_ = (int)draw_text_.size();
  }

  SP$(sn) ct_vpur(draw_text_.size()) pcommas ct_vpur(text_.size()) pendl;
  SP$(sn)
  ct_vpur(view_w_) pcommas ct_vpur(view_h_) pcommas ct_vpur(line_num_chars)
      pcommas ct_vpur(total_text_h_) pcommas ct_vpur(text_top_limit_) pendl;
  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

#ifdef BUILT_WITH_ROS
int TextObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                       uint32_t queue_sz) {

  sub_ = n.subscribe(topic_sub, queue_sz, &TextObject::callback, this);

  return RET_SUCCESS_INT;
}

void TextObject::callback(const std_msgs::String &sub_msg) {
  // std::cout <<  sub_msg.data << std::endl;
  const utils::Color &fc = utils::Color::white;
  const utils::Color &bgc = utils::Color::black;
  std::string dup = sub_msg.data;
  const std::string delimiter1 = ":";
  const std::string delimiter2 = ";";
  dup = dup.substr(dup.find(delimiter1, 0) + 1, dup.size() - 1);
  dup = dup.substr(0, dup.size() - 1);
  reinit(fc, bgc, dup);
}
#endif

int TextObject::draw_obj(pangolin::View &v) {
  PRINT_FUNC_ENTER;
  const int sn = 432;
  if (v_ == NULL) {
    // if v_ is NULL, there is a good chance stats have not been updated.
    PW$ "v_ shoudln't be null. Not Drawing. Did you add the object to a "
        "ViewNode?" pendl;
    return RET_TEXT_NOT_DRAWN;
  }

  if (v.vp.w != view_w_ || v.vp.h != view_h_)
    CHECK_RET(do_str_stats());

  SP$(sn) ct_vpur(start_idx_) pcommas ct_vpur(end_idx_) pendl;

  CHECK_RET(draw_text(v));

  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

int TextObject::draw_text(pangolin::View &v) {
  PRINT_FUNC_ENTER;
  glColor3fv(fc_);

  const int sn = 432;
  for (int i = start_idx_; i < end_idx_; ++i) {
    GLint x = v.vp.l + left_border_;
    // bottom border is essentially top border here.
    // GLint y = text_top_limit_ - (int)((i - start_idx_)*char_h_);
    GLint y = v.vp.b + v.vp.h -
              (VIEW_HEAD_HEIGHT + 2 * VIEW_HEAD_BORDER + bottom_border_) -
              (int)((i - start_idx_) * char_h_);
    SP$(sn) ct_vpur(x) pcommas ct_vpur(y) pendl;
    SP$(sn) ct_vpur(draw_text_[i].str) pendl;
    // SP$(sn) ct_vpur(draw_text_[i+scroll_idx_].str) pendl;
    draw_text_[i].DrawWindow(x, y);
  }
  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

void TextScrollHandler::Mouse(pangolin::View &, pangolin::MouseButton button,
                              int x, int y, bool pressed, int button_state) {

  (void)x;
  (void)y;
  (void)button_state;
  (void)pressed;
  if ((button == pangolin::MouseWheelUp ||
       button == pangolin::MouseWheelDown)) {
    if (button == pangolin::MouseWheelUp)
      tobj_->decrement_start_idx();
    if (button == pangolin::MouseWheelDown)
      tobj_->increment_start_idx();
  }
}
