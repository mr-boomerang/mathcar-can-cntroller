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
}
// main function
int main(int argc, char** argv)
{
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  // create an instance of the controller
  MmController controller_0;
  // start init parameters
  controller_0.init(argc, argv, &exit_flag_mm_controller);
  // start controller in a blocking manner
  controller_0.start_controller_thd();
  // at this point the feedback is present in output variable as string of
  // size 25 and input can be provided using the function send_cmd_to_vehicle.
  while (!exit_flag_mm_controller)
  {
  }
  return 0;
}
