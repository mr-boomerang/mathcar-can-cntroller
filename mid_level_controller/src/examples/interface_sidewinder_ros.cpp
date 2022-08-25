// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file interface_sidewinder_ros.cpp
 * @brief this file provides for keyboard or joystick control of car depending
 * on config.
 * @author Divyanshu Goel
 * @date 2018-03-29 (yyyy-mm-dd)
 **/
#include <can_driver/can_transceiver.h>  // interface for mid level controller
#include <interfaces/interface.h>  // controller for speed and steering over CAN

/// for clean exit of code
int exit_flag_can_transceiver = 0;
/// signal handler to do a clean exit
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by interface.\n", signum);
  // terminate program
  exit_flag_can_transceiver = 1;
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
  /// instance of transreceiver
  CanTransceiver txrx0;
  /// instance of keyboard
  Interface teleop_car;
  // set mode as sidewinder
  teleop_car.SetMode("sidewinder");
  // initialize transreceiver
  txrx0.init(argc, argv, &exit_flag_can_transceiver);
  // start transreceiver
  txrx0.start_transreceiver();
  // start the keyboard control
  teleop_car.StartInterface(argc, argv, &exit_flag_can_transceiver);
  // to process callbacks
  ros::spin();
  // return for the function
  return (0);
}
