// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file interface_ros.cpp
 * @brief this file provides for keyboard or joystick control of car depending
 * on config.
 * @author Divyanshu Goel
 * @date 2017-03-29 (yyyy-mm-dd)
 **/
#include <interfaces/interface.h>  // controller for speed and steering over CAN
/// for clean exit of code
int exit_flag_interface = 0;
/// signal handler to do a clean exit
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by interface.\n", signum);
  // terminate program
  exit_flag_interface = 1;
  ros::shutdown();
}
/// main function
int main(int argc, char** argv)
{
  /// initiate ros node
  ros::init(argc, argv, "teleop_node", ros::init_options::NoSigintHandler);
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  /// instance of keyboard
  Interface teleop_car;
  // start the keyboard control
  teleop_car.StartInterface(argc, argv, &exit_flag_interface);
  // to process callbacks
  ros::spin();
  // return for the function
  return (0);
}
