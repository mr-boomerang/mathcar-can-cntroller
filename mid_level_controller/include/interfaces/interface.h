// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file interface.h
 * @brief follows in inputs from a joystick
 * @author Divyanshu Goel
 * @date 2017-03-08 (yyyy-mm-dd)
 **/

#ifndef INTERFACES_INTERFACE_H
#define INTERFACES_INTERFACE_H

#ifndef BUILD_WITHOUT_ROS
#include "geometry_msgs/Twist.h"
#include "mid_level_controller/Callback_Msg.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#endif

#include <fcntl.h>
#include <linux/joystick.h>
#include <math.h>
#include <mid_level_controller/config.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "controller/common_utils.h"
#include "interfaces/joy_inputs.h"
#include "interfaces/joy_node.h"

#include "interfaces/peripherals/keyboard.h"
#include "interfaces/peripherals/redgear.h"
#include "interfaces/peripherals/sidewinder.h"
#include "interfaces/peripherals/xbox.h"

/// allows allocation of maximum size of buffer for character input
#define BUFFSIZE 256
/**
 * @brief This class will be used to send data over can after reading the yaml
 * settings file.
 */
class Interface
{
 private:
  /// for the purpose of sending and recieving
  CommonUtils cm0_;
  /// for the purpose of processing input
  ControllerBase *joystick;
  /// attribute parametes for keyboard control
  int kfd_ = 0;
  /// attribute parameters for keyboard control
  struct termios cooked_;
  /// attribute parameters for keyboard control
  struct termios raw_;
  /// to store old settings for clean exit
  struct termios old_settings_;
  /// velocity variable
  int velocity_ = 0;
  /// steering variable
  int steer_input_ = 0;
  /// variable for trajectory mode
  std::string trajectory_ = "sin";
  /// variable for the debug_enabled flag
  bool debug_enable_ = false;
  /// variable for enable trajectories
  bool trajectory_following_ = true;
  /// to print help
  bool print_help_ = false;
  /// @{varibles for set point update
  double steer_target_value0_ = 0, vel_target_value0_ = 0,
         last_steer_target_value_ = 0, brake_target_value0_ = 0,
         total_time_ = .3, frequency_ = 100;
  double step_size_speed_ = 1;
  double step_size_steer_ = 5;
  double steer_offset_ = 0;
  /// @}
  /// time stepp for the equation
  int t_equation_ = 0;
  /// flag if key is pressed
  bool dirty_ = false;
  /// value of key pressed
  std::string key_pressed_ = "None";
  /// limits for the steering angle
  double steer_degree_limit_ = 40;
  ///@{ settings for keyboard
  double step_size_ = 1, step_size_accl_ = 1, step_size_vel_ = 1;
  ///@}
  /// set maximum value of velocity in kmph
  double max_velocity_ = 18;
  /// to set direction of steerinf
  double steer_direction_multiplier_ = -1;
  ///@{ mode of input through controller or throttle
  std::string input_mode_, velocity_input_units_, steer_input_units_;
  /// @}
  /// @{time stamps variables
  double speed_time_last_, speed_time_diff_, speed_time_now_;
  double steer_time_last_, steer_time_diff_, steer_time_now_;
  double steer_value_time_last_, steer_value_time_diff_, steer_value_time_now_;
  /// @}
  /// threads for controlling keyboard or joystick
  std::thread interface0_;
  /// threads for processing joystick
  std::thread joy_process0_;
  /// flag to allow if threading is done
  bool interface0_is_threaded_ = false;
  /// flag to allow if threading is done
  bool joy_process0_is_threaded_ = false;
  /// pointer to exit flag from signal handler
  const int *exit_flag_ptr_;
  /// select type of joystick
  std::string joy_type_ = "xbox";
  /// joystick expose topics
  bool joy_expose_topics_ = false;
  /// joystick handler
  Joystick j0_;
  ///@{ settings for interface
  double acceleration_value_ = 0, steer_degree_ = 0;
  double velocity_value_ = 0.0, steer_degree_input_ = 0,
         velocity_value_input_ = 0.0;
  int updated_ = 5;
  int gear_status_ = 1;
  unsigned int brake_value_ = 0, horn_status_ = 0, wiper_status_ = 0,
               right_turn_lamp_ = 0, left_turn_lamp_ = 0, hazard_lamp_ = 0,
               head_lamp_ = 0, brake_light_ = 0, homing_active_ = 0,
               accl_limit_ = 60;
  /// @}
  /// limit for velocity
  double velocity_limit_ = 5;
  /// set direction of steer
  std::string steer_dir_ = "right";

#ifndef BUILD_WITHOUT_ROS
  /// Callback msg object
  mid_level_controller::Callback_Msg teleop_car_control_;
  /// object twist
  geometry_msgs::Twist teleop_car_velocity_;
  /// Ros Subscriber for joystick key
  ros::Subscriber sub_;
  /// Publisher variable for ROS
  ros::Publisher control_pub_;
  /// Publisher variable for ROS
  ros::Publisher speed_control_;
  /// geometry msg for transmission
  geometry_msgs::Twist control_input_;
#endif

 public:
  /// constructor
  Interface() {}
  /// destructor~
  ~Interface();
  /**
   * @brief This method will be used to execute a system command
   * @param command command to execute
   */
  int ExecCommand(std::string command);
  /**
   * @brief This method will be used to set mode of joystick
   * @param mode mode to be used
   */
  void SetMode(std::string mode);
  /**
   * @brief This method will be used to start interface
   * @param argc input argument count
   * @param argv input arguments
   * @param exit_flag_arg_ pointer to exit flag for safe exit
   */
  void StartInterface(int argc, char **argv, const int *exit_flag_arg_);
  /**
   * @brief This method will be used to start joystick presses node.
   * @param joy_dev  path to joystick
   */
  void JoystickMonitor(std::string joy_dev);

  /**
   * @brief This method will be used monitor joystick in a threaded manner.
   * @param joy_dev path to joystick
   */
  void JoystickMonitorThd(std::string joy_dev);
  /**
   * @brief This method will be used monitor joystick.
   */
  void JoystickProcessVelocity();
  /**
   * @brief This method will be used to process joystick presses.
   * @param msg  pointer to joystick message
   */
  void JoystickCallback(JoystickMsg msg);
  /**
   * @brief This method will be used to process keyboard presses.
   */
  void KeyboardMonitor();
  /**
   * @brief This method will be used to process keyboard presses in a threaded
   * manner.
   */
  void KeyboardMonitorThd();
  /**
   * @brief This method will be used to initialize the parameters for the
   * program.
   * @param controller_settings_file_path path to controller settings
   */
  void InitKeyboard(const std::string controller_settings_file_path);
  /**
   * @brief This method will be used to initialize the parameters for the
   * program for joystick control.
   * @param controller_settings_file_path path to controller settings
   */
  void InitJoystick(const std::string controller_settings_file_path);
  /// This method will be used to get current steering
  double CurrentSteering(int time_handle, double target,
                         double starting_position, double time_limit);
  /// This method will be used to get current speed
  double CurrentSpeed(int time_handle, double target, double starting_position,
                      double time_limit);
  /// This method will be used to cleanly exit from code
  void SignalHandler(int signum);
};
#endif  // INTERFACES_INTERFACE_H
