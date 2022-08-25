/****************************************************************************
 **
 ** File: system.hpp
 ** **
 ** Copyright © 2015, Uurmi Systems                                         **
 ** All rights reserved.                                                    **
 ** http://www.uurmi.com **
 ** **
 ** All information contained herein is property of Uurmi Systems           **
 ** unless otherwise explicitly mentioned.                                  **
 ** **
 ** The intellectual and technical concepts in this file are proprietary    **
 ** to Uurmi Systems and may be covered by granted or in process national   **
 ** and international patents and are protect by trade secrets and          **
 ** copyright law.                                                          **
 ** **
 ** Redistribution and use in source and binary forms of the content in     **
 ** this file, with or without modification are not permitted unless        **
 ** permission is explicitly granted by Uurmi Systems.                      **
 ** **
 ****************************************************************************/

#pragma once

#include <ctime>
#include <memory>
#include <string>
#include <thread>

#include <pangolin/pangolin.h>

#include <opencv2/core/core.hpp>
#include <utils/error_code.h>

#define GuiCreate Gui::instance
#define GuiInstance Gui::instance()

#define image_view "Image"

// time in millisecond.
#define PAUSE_TIME 10
#define PLAY_TIME 30
#define SLOW_TIME 200

bool keyPressed = false;

enum WinSize { normal = 1, fullscreen = 2 };

class GuiHandler : public pangolin::Handler {
public:
  GuiHandler() {}

  void Keyboard(pangolin::View &, unsigned char key, int x, int y,
                bool pressed) {
    std::cout << "Key pressed" << std::endl;
    keyPressed = true;
  }

private:
  /* data */
};

class Gui {

public:
  static Gui &instance(std::string name = "Gl Window", WinSize sz = normal,
                       int wait_time = 30) {
    static Gui gui_singleton(name, sz, wait_time);
    return gui_singleton;
  }
  // virtual ~gui ();

  bool initDone() { return init_done_; }

  bool show(const std::string &view_name, cv::InputArray ip_img) {
    if (!init_done_) {
      std::cout << "Window not created. Call GuiCreate" << std::endl;
      return false;
    }

    cv::Mat img;
    img = ip_img.getMat();

    try {
      GLint format = GL_RGB;
      GLenum type = GL_UNSIGNED_BYTE;
      if (!tex_.tid) { // Need to initialize texture and buffers
        tex_.Reinitialise(img.cols, img.rows, format, true, 0, format, type);
      }

      if (!img.empty()) {
        cv::cvtColor(img, img, CV_BGR2RGB);
        tex_.Upload((unsigned char *)img.data, format, type);
      }
    } catch (std::exception e) {
      return false;
    }
    return RET_SUCCESS_BOOL;
  }
  bool waitKey(int time = 0) {
    double t1 = (double)cv::getTickCount();
    bool time_remain = true;
    while (!pangolin::ShouldQuit() && time_remain) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(wait_time_)); // msec

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      pangolin::FinishFrame();
      if (keyPressed)
        break;
      if (time) {
        double t2 = (double)cv::getTickCount();
        double time_elapsed = (t2 - t1) / cv::getTickFrequency();
        std::cout << time_elapsed << ", " << time << std::endl;
        time_remain = (time_elapsed * 1000 < time);
      }
    }
    return !pangolin::ShouldQuit();
  }
  bool quit() { return pangolin::ShouldQuit(); }
  void waitTime(int wait_time) { wait_time_ = wait_time; }

private:
  /* init function */
  Gui(std::string name, WinSize sz, int wait_time) {
    gui_init(name, sz, wait_time);
  }
  bool gui_init(std::string name, WinSize sz, int wait_time) {

    win_sz_scheme_ = sz;
    int width = 640, height = 480;
    pangolin::CreateWindowAndBind(name, width, height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View &view_img = pangolin::Display(image_view);
    view_img.SetDrawFunction(
        std::bind(&Gui::DrawImage, this, std::placeholders::_1));
    pangolin::DisplayBase().SetHandler(new GuiHandler());
    if (sz == fullscreen)
      pangolin::ToggleFullscreen();
    init_done_ = true;
  }

  /* data */
  bool init_done_;
  int wait_time_;
  WinSize win_sz_scheme_;

  volatile bool step_;

  pangolin::GlTexture tex_;

  GLint format_;
  GLenum type_;

  /* function */
  void DrawImage(pangolin::View &v) {
    v.Activate();

    glColor3f(1.0f, 1.0f, 1.0f);

    if (tex_.tid) {
      tex_.RenderToViewportFlipY();
    }
  }
};
