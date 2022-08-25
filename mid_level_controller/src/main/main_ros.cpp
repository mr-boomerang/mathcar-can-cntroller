// Copyright 2011-2018 The Mathworks, Inc.
// controller for speed and steering over CAN
#include <controller/mm_controller.h>
/// for clean exit of code
int exit_flag_mm_controller = 0;
// signal handler
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by controller.\n", signum);
  // terminate program
  exit_flag_mm_controller = 1;
  ros::shutdown();
}
// main function
int main(int argc, char** argv)
{
  // initialize ros node
  ros::init(argc, argv, "mid_level_controller",
            ros::init_options::NoSigintHandler);
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  // create an instance of the controller
  MmController controller_0;
  // start init parameters
  controller_0.init(argc, argv, &exit_flag_mm_controller);
  // start controller
  controller_0.start_controller_thd();
  // keep the code from quitting
  ros::spin();
  // return for the function
  return 0;
}
