// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file joy_node.h
 * @brief joystick interface for ros.
 * @author Divyanshu Goel
 * @date 2018-01-25 (yyyy-mm-dd)
 **/

#ifndef INTERFACES_JOY_NODE_H
#define INTERFACES_JOY_NODE_H
#ifndef BUILD_WITHOUT_ROS
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#endif

#include <fcntl.h>
#include <linux/joystick.h>
#include <math.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "interfaces/joy_struct.h"

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

/// * @brief Opens, reads from and publishes joystick events
class Joystick
{
 private:
#ifndef BUILD_WITHOUT_ROS
  /// publisher for ros
  ros::Publisher pub_;
#endif
  /// state of joystick
  bool open_;
  /// path to joystick
  std::string joy_dev_;
  /// deadzone for the joystick
  double dead_zone_ = 0.05;
  /// refresh rate in Hz. 0 for no repeat.
  double auto_repeat_rate_ = 0;
  /// Defaults to 100 Hz rate limit.
  double coalesce_interval_ = 0.001;
  /// Parameter conversions
  double autorepeat_interval_;
  /// Parameter conversions
  double scale_;
  /// Parameter conversions
  double unscaled_dead_zone_;
  /// total number of events occurance
  int event_count_;
  /// total number of publishes done
  int pub_count_;
  /// last time stamp
  double last_diag_time_;
  /// pointer for clean exit
  const int *exit_flag_ptr_;
  /// joystick open link
  int joy_fd_;
  /// flag for joystick if it is threaded
  bool joy_is_threaded_ = false;
  /// thread for joystick
  std::thread joy_stick0_;
  /// count of axes
  __u8 axes0_;
  /// count of buttons
  __u8 buttons0_;
  /// debug flag
  bool debug_enable_ = false;
  /// joystick event
  /// js_event *event_;
  /// to expose joystick topic output to ros
  bool expose_topics_ = false;
  /// to contain joystick message
  JoystickMsg joy_msg_;

 public:
  /// constructor
  Joystick() {}
  /// destructor
  ~Joystick();
  /**
   * @brief This function is to initialize the joystick Parameters
   * @param port location of joystick
   * @param exit_flag_arg pointer to global exit flag
   * @param debug_enable to set debug enable option
   * @param dead_zone parameters for joystick
   * @param auto_repeat_rate parameters for joystick
   * @param coalesce_interval parameters for joystick
   */
  void JoyInit(std::string port, const int *exit_flag_arg,
               bool debug_enable = false, double dead_zone = 0.05,
               double auto_repeat_rate = 0, double coalesce_interval = 0.001);
  /**
   * @brief to set debug enable option
   * @param debug_enable option to set
   */
  void setDebugOption(bool debug_enable);
  /**
   * @brief to set the update in message
   * @param update_option option to set
   */
  void setJoyMsgUpdated(bool update_option);
  /**
   * @brief to set debug enable option
   * @param expose_topics_enable option to set
   */
  void setExposeTopicsOption(bool expose_topics_enable);
  /**
   * @brief to get debug enable option
   */
  bool getDebugOption();
  /**
   * @brief to get the update in message
   */
  bool getJoyMsgUpdated();
  /**
   * @brief to set debug enable option
   */
  bool getExposeTopicsOption();
  /// * @brief to get the joystick message
  JoystickMsg GetJoystickMsg();
  /// * @brief to open joystick
  void JoyOpen();
  /**
   * @brief to check up on events of joystick
   * @param event a joystick event that occurs
   * @return if a message was valid and to print mssage now or later
   */
  int JoyEventCheck(js_event event);
  /**
   * @brief Opens joystick port, reads from port and publishes while node is
   * active
   */
  void JoyMonitor();
  /**
   * @brief Opens joystick port, reads from port and publishes while node is
   * active in a threaded manner
   */
  void JoyMonitorThd();
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
#endif  // INTERFACES_JOY_NODE_H
