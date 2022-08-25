// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file mm_controller.h
 * @brief controller for speed and steering over CAN
 * @author Divyanshu Goel
 * @date 2017-05-08 (yyyy-mm-dd)
 */

#ifndef CONTROLLER_MM_CONTROLLER_H
#define CONTROLLER_MM_CONTROLLER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

#ifndef BUILD_WITHOUT_ROS
#include "controller/look_up_tables.h"
#include "geometry_msgs/Twist.h"
#include "mid_level_controller/Callback_Msg.h"
#include "mid_level_controller/Can_Status_Message.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#endif

#include <chrono>
#include <string>
#include <thread>

#include "can_driver/can_transceiver.h"
#include "controller/common_utils.h"

/**
 * @brief This class will be used to control speed of the car
 */
class MmController
{
 public:
  CommonUtils cm0_;
  /// check for valid initialization
  bool init_success = false;
  /// have an instance of transreceiver
  CanTransceiver txrx;
  /// log file object
  /// @{ variables for storing data
  volatile int steer_in = 0, steer_out = 0, steer_out_direction_multiplier = 1,
               steer_in_direction_multiplier = 1, speed = 0, accln_in = 0,
               accln_out = 0, brake_in = 0, brake_out = 0, speed_direction = 1;
  /// @}
  /// if config is loaded elsewhere
  bool params_loaded = false;
  /// file path for settings file
  std::string log_file_path;
  /// file path for settings file
  std::string settings_file_path;
  /// file path for settings file
  std::string control_yaml_file_path;
  /// file path for settings file
  std::string status_yaml_file_path;
  /// topic name for slope
  std::string slope_topic;
  /// update flag
  int updated = 0;
  /// @{values for pid_throttle
  double Kp = 6.0, Ki = 0, Kd = 1;
  /// @}
  /// @{values for pid_brake
  double Kp_brake = 12.0, Ki_brake = 0, Kd_brake = 2;
  /// @}
  /// @{output variables
  int accln_commanded = 0, brake_commanded = 0;
  /// @}
  /// values for proportional control
  double brake_press = 0;
  /// brake multiplier
  int brake_multiplier = 37;
  /// @{error variables
  double error_brake_sum = 0, error_brake_dt = 0, last_error_brake = 0,
         error_brake = 0;
  /// @}
  /// @{error variables
  double pedal_press = 0, error_sum = 0, dt = 0.01, error_dt = 0,
         last_error = 0, error = 0, C_1 = 0, K_2 = 0, theta = 0, C_2 = 0;
  /// @}
  /// steer direction which is +ve or -ve
  double steer_direction_multiplier = -1;
  /// @{ enable variable for lookup and slope
  int lookup_enable = 1, slope_enable = 0;
  /// @}
  /// @{ velocity set point that is to be achieved
  volatile double setpoint_speed = 0, last_speed = -1;
  /// @}
  /// steer set point that is to be achieved
  volatile double setpoint_steer = 0;
  /// size of lookup table
  int throttle_value_count = 27;
  /// size of lookup table
  int throttle_value_reverse_count = 27;
  /// size of lookup table
  int throttle_value_boost_count = 27;
  /// size of brake lookup table
  int brake_value_count = 27;
  /// brake target value
  double input_brake_target0 = 0;
  /// max throttle limit
  double throttle_limit = 60;
  /// max brake limit
  double brake_limit = 88;
  // @{ watch dog timers enable flags
  bool watch_dog_feedback_planner_enable = false,
       watch_dog_feedback_speed_enable = false,
       watch_dog_feedback_slope_enable = false;
  // @}
  /// lookup table for throttle fwddrive gear
  int *throttle_values;
  /// lookup table for throttle reverse gear
  int *throttle_reverse_values;
  /// lookup table for throttle boost gear
  int *throttle_boost_values;
  ///@{ watch dog time stamps variables
  double watch_dog_time_diff, watch_dog_time_now;
  /// @}
  /// @{time stamps variables
  double watch_dog_time_planner_feedback = 0, watch_dog_time_speed_feedback = 0,
         watch_dog_time_slope_feedback = 0;
  /// @}
  /// @{ debug enable variable
  bool debug_enable = false, raw_msg_enable = false;
  /// @}
  /// @{ unit variable for inputs
  std::string velocity_input_units = "mps", steer_input_units = "degree",
              controller_feedback_source = "gateway", brake_mode = "regular";
  /// @}
  /// current gear
  std::string gear_current = "neutral";
  /// @{ limits for input in mps and degrees
  double vel_limit = 5.55, steer_limit = 40;
  /// @}
  /// variable to store braking distance
  double braking_distance = 0;
  /// @{ enable calculation of braking distance and velocity achievement time
  bool calc_target_velocity_achievement_time = false,
       calc_braking_distance = false;
  /// @}
  /// allow update of flag
  bool enable_update_pid_flag = false;
  /// exit flag for clean exit
  int exit_flag = 0;
  /// pointer to exit flag from signal handler
  const int *exit_flag_pointer_;
  /// thread for controller
  std::thread controller_thread0;
  /// flag to allow spin in main thread
  bool controllerIsThreaded = false;
  /// constructor
  MmController() {}
  /// destructor~
  ~MmController();
  /// This method will be used to change gear
  void changeGear(std::string targetGear);
  /// This method will be used to calculate pedal press value.
  int calc_pedal_press_value();
  /// This method will be used to calculate brake press value.
  int calc_brake_press_value();
  /// This method will be used to calculate corresponding base brake press value
  double get_brake_lookup();
  /**
   * @brief This method will be used to calculate corresponding base pedal press
   * value
   * @param input_velocity input value for calculating corresponding lookup
   * value
   */
  int polynomial_func(double input_velocity);
  /**
   * @brief This method will be used to intialize the states
   * @param argc parameter count
   * @param argv parameter value
   * @param exit_flag_arg_ pointer to exit flag for safe exit
   */
  void init(int argc, char **argv, const int *exit_flag_arg_);
  /**
   * @brief This method will be used to update steer
   * @param slope_ang feedback of current slope angle
   */
  void slope_update(double slope_ang);
  /**
   * @brief This method will be used to update steer
   * @param setpoint_steer targer set point steer
   */
  void update_steer(int setpoint_steer);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void send_cmd_to_vehicle(std::string item_requested, std::string item_value);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void send_cmd_to_vehicle(std::string item_requested, int item_value);
  /**
   * @brief This method will be used to update command to be sent to vehicle
   * @param item_requested target command
   * @param item_value associated value
   */
  void send_cmd_to_vehicle(std::string item_requested, double item_value);
  /**
   * @brief This method will be used to update the velocity setpoint.
   * @param inp_speed target speed
   * @param inp_steer_angle target steering
   * @param inp_brake_target target braking
   */
  void update_setpoint(double inp_speed, double inp_steer_angle,
                       double inp_brake_target = 0);
  /// This method will be used to start controller as a thread.
  void start_controller_thd();
  /// This method will be used to start controller .
  void controller();
  /**
   * @brief This method will be used to handle clean exit.
   * @param signum signal value
   */
  void controller_exit(int signum);
  /**
   * @brief This method will be used to handle signals for clean exits.
   * @param signum signal value
   */
  void signalHandler(int signum);
#ifndef BUILD_WITHOUT_ROS
  /// ros publish variables
  ros::Publisher control_pub;
  /// to get sensor feedback
  ros::Subscriber feedback_sub;
  /// to get sensor feedback
  ros::Subscriber feedback_sub_slope;
  /// to get commanding value
  ros::Subscriber input_sub;
  /// geometry twist message
  geometry_msgs::Twist zero_speed;
  /// This method will be used to setup ros connections for the controller
  /// code.
  void setup_ros_connections();
  /**
   * @brief This method will be used to update the slope for compensation
   * @param msg pointer for feedback message
   */
  void slope_update_ros(const std_msgs::Float64::ConstPtr &msg);
  /**
   * @brief This method will be used to update the velocity setpoint.
   * @param msg pointer for feedback message
   */
  void update_setpoint(const geometry_msgs::Twist::ConstPtr &msg);
#endif
  /**
   * @brief This method will be used to read settings from file and act
   * accordingly
   * @param *testing_settings_file_path file location for settings
   */
  void read_settings(const std::string testing_settings_file_path);
};
/// This method will be used to check parameters
static bool IsNonEmptyMessage(const std::string flagname,
                              const std::string &value)
{
  return value[0] != '\0';
}
#endif  // CONTROLLER_MM_CONTROLLER_H
