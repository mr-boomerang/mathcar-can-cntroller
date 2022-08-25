// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file keyboard_node.h
 * @brief Keyboard interface
 * @author Divyanshu Goel
 * @date 2018-02-7 (yyyy-mm-dd)
 **/

#ifndef INTERFACES_KEYBOARD_NODE_H
#define INTERFACES_KEYBOARD_NODE_H
#ifndef BUILD_WITHOUT_ROS
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#endif

#include <fcntl.h>
#include <math.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

/// * @brief Opens, reads from Keyboard events
class Keyboard
{
 private:
#ifndef BUILD_WITHOUT_ROS
  /// publisher for ros
  ros::Publisher pub_;
#endif
  /// state of Keyboard
  bool open_;
  /// total number of events occurance
  int event_count_;
  /// total number of publishes done
  int pub_count_;
  /// last time stamp
  double last_diag_time_;
  /// pointer for clean exit
  const int *exit_flag_ptr_;
  /// flag for keyboard if it is threaded
  bool keyboard_is_threaded_ = false;
  /// thread for keyboard
  std::thread keyboard0_;
  /// debug flag
  bool debug_enable_ = false;
  /// to contain Keyboard message
  char key_pressed;

 public:
  /// constructor
  Keyboard() {}
  /// destructor
  ~Keyboard();
  /**
   * @brief This function is to initialize the Keyboard Parameters
   * @param exit_flag_arg pointer to global exit flag
   * @param debug_enable to set debug enable option
   */
  void JoyInit(const int *exit_flag_arg, bool debug_enable = false);
  /**
   * @brief to set debug enable option
   * @param debug_enable option to set
   */
  void setDebugOption(bool debug_enable);
  /**
   * @brief to set the update in message
   * @param update_option option to set
   */
  void setMsgUpdated(bool update_option);
  /**
   * @brief to get debug enable option
   */
  bool getDebugOption();
  /**
   * @brief to get the update in message
   */
  bool getMsgUpdated();
  /// * @brief to get the Keyboard message
  char GetKeyPressed();
  /// * @brief to open Keyboard
  void KeyboardOpen();
  /**
   * @brief monitors keyboard
   */
  void KeyboardMonitor();
  /**
   * @brief monitors keyboard in a threaded manner
   */
  void KeyboardMonitorThd();
  /**
   * @brief to handle interrupts
   * @param signum interrupt code
   */
  void SignalHandler(int signum);
  /**
   * @brief to do a clean exit
   */
  void JoyExit();
};
#endif  // INTERFACES_KEYBOARD_NODE_H
