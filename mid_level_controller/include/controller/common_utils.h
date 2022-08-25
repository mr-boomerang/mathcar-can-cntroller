// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file common_utils.h
 * @brief provides some basic common functionalities
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/

#ifndef CONTROLLER_COMMON_UTILS_H
#define CONTROLLER_COMMON_UTILS_H

#ifndef BUILD_WITHOUT_ROS
#include "geometry_msgs/Twist.h"
#include "mid_level_controller/Callback_Msg.h"
#include "mid_level_controller/Can_Status_Message.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#endif

#include <signal.h>
#include <stdio.h>

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

#include "can_driver/can_struct.h"  /// for Header

/// path to config file
DECLARE_string(joy_cfg);
/// input mode selection
DECLARE_string(mode);
/// path to joystick
DECLARE_string(joy_path);
/// flag for printing help
DECLARE_bool(print_help);

/// allows allocation of maximum size of buffer for character input
#define BUFFSIZE 256

/**
 * This class will be used to implement some basic functionalities of joystick
 * commands required.
 */
class CommonUtils
{
 public:
  /// parameters for the vehicle
  CarState car_state_;
#ifndef BUILD_WITHOUT_ROS
  /// geometry msg for transmission
  geometry_msgs::Twist control_input_;
  /// object twist
  geometry_msgs::Twist teleop_car_velocity_;
#endif

 private:
  /// pointer to exit flag from signal handler
  const int *exit_flag_ptr_;
  /// to print help
  bool print_help_;
  /// to store path fo joystick
  std::string joy_path_;
  /// to have debug message enabled
  bool debug_enable_;
  /// to have debug message enabled
  bool slope_enable_;
  /// to store current value of slope
  double current_slope_ = 0;
  /// to store current topic of slope
  std::string slope_topic_ = "";
  /// throttle or velocity control mode. some joystick may only provide some
  /// functionality.
  std::string mode_;
  /// messages of the current state of car
  std::string data_[30];
  /// data updated flag
  bool data_updated_ = 0;
  /// to keep track of latest gear
  std::string gear_current = "neutral";

#ifndef BUILD_WITHOUT_ROS
  /// Ros Subscriber for current car state
  ros::Subscriber car_sub_;
  /// Ros Subscriber for slope feedback
  ros::Subscriber feedback_sub_slope;
  /// Publisher variable for ROS
  ros::Publisher control_pub_;
  /// Publisher variable for ROS
  ros::Publisher speed_control_;
#endif

 public:
  /// constructor
  CommonUtils() {}
  /// destructor~
  ~CommonUtils() {}
  // Setter and getter methods
  /**
   * @brief This method will be used to set debug_enable value
   * @param value to set desired value
   */
  void SetDebugOption(bool value);
  /**
   * @brief This method will be used to get the current debug_enable value
   */
  bool GetDebugOption();
  /**
   * @brief This method will be used to set help option
   * @param value to set desired value
   */
  void SetHelpOption(bool value);
  /// This method will be used to keep track is data is updated
  bool GetDataStateUpdated();
  /**
   * @brief This method will be used to set value of slope topic
   * @param slope_topic value of slope topic
   */
  void SetSlopeTopic(std::string slope_topic);
  /// This method will be used to let the code know that data is stale
  void SetDataStateUpdatedStale();
  /// This method will be used to keep track is data is updated
  double GetDataStateTimeUpdated();
  /// This method will be used to keep track is data is updated
  double GetSlopeValue();
  /**
   * @brief This method will be used to set mode of joystick
   * @param mode mode to be used
   */
  void SetMode(std::string mode);
  /// main methods
  /**
   * @brief This method will be used to initiate the class options
   * @param exit_flag_arg pointer to exit flag for safe exit
   * @param joy_path path to joystick
   * @param print_help print help
   * @param debug_enable to print debug info
   * @param slope_enable to enable slope input
   */
  void Init(const int *exit_flag_arg, std::string joy_path = "/dev/input/js0",
            std::string slope_topic = "slope", bool print_help = 0,
            bool debug_enable = 0, bool slope_enable = 0);
  /**
   * @brief This method will be used to execute a system command
   * @param command command to execute
   */
  int ExecCommand(std::string command);
  /// This method will be used to get current joy path after checking
  std::string GetJoyPath();
  /**
   * @brief This method will be used to start interface
   * @param target_gear gear to set
   */
  void SetGear(std::string target_gear);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * speed controller
   * @param speed target speed
   * @param steer target steer
   * @param lock_mode_speed enable disable steer lock
   * @param lock_mode_steer enable disable steer lock
   * @param brake_press target braking
   */
  void SendCmdToController(double speed, double steer,
                           double lock_mode_speed = 0,
                           double lock_mode_steer = 0,
                           double brake_press = 100);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void SendCmdToVehicle(std::string item_requested, std::string item_value);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void SendCmdToVehicle(std::string item_requested, int item_value);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void SendCmdToVehicle(std::string item_requested, double item_value);
  /**
   * @brief This method will be used to update current vehicle state
   * @param msg latest update messages
   */
  void CanCallback(std::string msg[]);
  /**
   * @brief This method will be used to check invalid messages in the array at
   * given index
   * @param msg latest update message
   * @param index index to check at
   */
  bool CheckmsgAtIndex(std::string msg[], int index);
  /**
   * @brief This method will be used to reset the current car state
   */
  void ResetCar();
  /**
   * @brief This method will be used to update current slope state
   * @param value latest update message
   */
  void SlopeUpdateRos(double value);

#ifndef BUILD_WITHOUT_ROS
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * speed controller
   * @param control_input combined twist message
   */
  void SendCmdToController(geometry_msgs::Twist control_input);
  /**
   * @brief This method will be used to setup ROS feedback/publisher connections
   */
  void CanRosSetup();
  /**
   * @brief This method will be used to update current vehicle state
   * @param msg latest update message
   */
  void SlopeUpdateRos(const std_msgs::Float64::ConstPtr &msg);
  /**
   * @brief This method will be used to update current vehicle state
   * @param msg latest update message
   */
  void CanCallback(
      const mid_level_controller::Can_Status_Message::ConstPtr &msg);
#endif
};
/**
   @brief fetches system timestamp from RTC
   @return current ns timestamp in double
*/
static double GetTimestampNow()
{
  auto start = std::chrono::high_resolution_clock::now();
  return (start.time_since_epoch().count() / 1000);
};
#endif  // CONTROLLER_COMMON_UTILS_H
