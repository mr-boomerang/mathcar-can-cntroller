// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file transceiver_ros.cpp
 * @brief transreceiver for can. This file can be used to send and recieve
 * messages over can given that it is setup already and store commands in a log
 * file
 * @author Divyanshu Goel
 * @date 2017-06-29 (yyyy-mm-dd)
 **/
// controller for speed and steering over CAN
#include <can_driver/can_transceiver.h>
/// for clean exit of code
int exit_flag_can_transceiver = 0;
// signal handler
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by transceiver.\n", signum);
  // terminate program
  exit_flag_can_transceiver = 1;
  ros::shutdown();
}
// main function
int main(int argc, char** argv)
{
  /// initiate ros node
  ros::init(argc, argv, "transceiver", ros::init_options::NoSigintHandler);
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  /// instance of transreceiver
  CanTransceiver txrx0;
  // initialize transreceiver
  txrx0.init(argc, argv, &exit_flag_can_transceiver);
  // start transreceiver
  txrx0.start_transreceiver();
  // to process callbacks
  ros::spin();
  // return for the function
  return 0;
}
