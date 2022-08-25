// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_transceiver.cpp
 * @brief transmitter and eceiver for can. This file can be used to transmit
 * messages over can every 10 ms and recieve messages every 10 ms.
 * @author Divyanshu Goel
 * @date 2017-02-29 (yyyy-mm-dd)
 **/
#include <can_driver/can_transceiver.h>
#include <mid_level_controller/config.h>

#include <algorithm>
#include <string>
#include <vector>

#ifndef UTILS_AVOID_GFLAGS
/// flag for taking file name
DEFINE_string(cfg, "settings_default.cfg",
              "File to the configuration file for the package.");
/// flag for taking path to store log files
DEFINE_string(log_path, "./logs/", "File path to the logs folder.");
/// flag for taking name of control messages
DEFINE_string(control_yaml_file_path, "control_messages.yaml",
              "file for the control yaml file for CAN");
/// flag for taking name of status messages
DEFINE_string(status_yaml_file_path, "status_messages.yaml",
              "file for the status yaml  file for CAN");
/// to print help for the input and outputs for CAN
DEFINE_bool(print_help, false, "Print help for the inputs and outputs of CAN");
/// to enable communication over virtual can for testing
DEFINE_bool(virtual_can, false, "Start on Virtual CAN");
#else
std::string FLAGS_cfg = "can_settings.cfg";
std::string FLAGS_log_path = "./logs";
std::string FLAGS_control_yaml_file_path = "control_messages.yaml";
std::string FLAGS_status_yaml_file_path = "status_messages.yaml";
bool FLAGS_print_help = false;
bool FLAGS_virtual_can = false;
#endif
/// initialize parameters
void CanTransceiver::init(const std::string control_input_yaml_file_path,
                          const std::string status_input_yaml_file_path,
                          const std::string log_file_path,
                          const std::string settings_file_path,
                          const int *exit_flag_arg_)
{
  PRINT_FUNC_ENTER;
  // sanity check
  if (!flags_set)
  {
    PE$ ct_red("Pass the flags properly and call parse flags first.") pendl;
    return;
  }
  exit_flag_pointer_ = exit_flag_arg_;

  tx0.init(control_input_yaml_file_path, log_file_path, settings_file_path,
           exit_flag_pointer_, FLAGS_print_help, FLAGS_virtual_can);
  if (*exit_flag_pointer_) return;
  tx0.params_loaded = true;
  rx0.params_loaded = true;
  init_success = true;
  // to ensure that there are no overlap issues
  usleep(200000);
  rx0.init(status_input_yaml_file_path, log_file_path, settings_file_path,
           exit_flag_pointer_, FLAGS_print_help, FLAGS_virtual_can);
  // to ensure that there are no overlap issues
  usleep(200000);
#ifndef BUILD_WITHOUT_ROS
  create_data_publisher_handles();
#endif
  PRINT_FUNC_EXIT;
}
/// initialize parameters
void CanTransceiver::init(int argc, char **argv, const int *exit_flag_arg_)
{
  PRINT_FUNC_ENTER;
  // gflags or miniflags check
  init_parse_flags(argc, argv);
  init(control_yaml_file_path, status_yaml_file_path, log_file_path,
       settings_file_path, exit_flag_arg_);
  PRINT_FUNC_EXIT;
}
/// initialize parameters
void CanTransceiver::init_parse_flags(int argc, char **argv)
{
  PRINT_FUNC_ENTER;
// gflags or miniflags check
#ifndef UTILS_AVOID_GFLAGS
  // parse commands
  if (!master_class_data_read) print::init(&argc, &argv);
#else
  if (!master_class_data_read) miniflags::miniflags_init(argc, argv);
#endif
#if 0
  settings_file_path = FLAGS_cfg;
  log_file_path = FLAGS_log_path;
  control_yaml_file_path = FLAGS_control_yaml_file_path;
  status_yaml_file_path = FLAGS_status_yaml_file_path;
#else
#ifndef _DEPLOY_SWAHANA_SUITE_
  settings_file_path = PKG_SRC "/cfg/" + FLAGS_cfg;
  log_file_path = PKG_SRC "/logs/";
  control_yaml_file_path = PKG_SRC "/yaml/" + FLAGS_control_yaml_file_path;
  status_yaml_file_path = PKG_SRC "/yaml/" + FLAGS_status_yaml_file_path;
#else
  settings_file_path =
      CONTROLLER_PREFIX "/../cfg/mid_level_controller/" + FLAGS_cfg;
  log_file_path = CONTROLLER_PREFIX "/../cfg/mid_level_controller/logs/";
  control_yaml_file_path = CONTROLLER_PREFIX
                           "/../cfg/mid_level_controller/yaml/" +
                           FLAGS_control_yaml_file_path;
  status_yaml_file_path = CONTROLLER_PREFIX
                          "/../cfg/mid_level_controller/yaml/" +
                          FLAGS_status_yaml_file_path;
#endif
#endif
  flags_set = true;
  PRINT_FUNC_EXIT;
}
/// start transmitter receiver
void CanTransceiver::start_transreceiver()
{
  PRINT_FUNC_ENTER;
  if (*exit_flag_pointer_) return;
  if (!init_success)
  {
    PE$ ct_red(
        "CAN Transceiver. Error in intilization. Please Ensure proper "
        "initialization") pendl;
    return;
  }
#ifndef BUILD_WITHOUT_ROS
  start_data_publisher_thd();
#endif
  tx0.start_can_transmitter_thd();
  rx0.start_can_listener_thd();
  PRINT_FUNC_EXIT;
}
#ifndef BUILD_WITHOUT_ROS
/// This method will be used to setup ros handles
void CanTransceiver::create_data_publisher_handles()
{
  PRINT_FUNC_ENTER;
  /// initiate ros node handle
  ros::NodeHandle nh;
  /// create instance of a subscriber to take specific command based input
  data_subs =
      nh.subscribe("canControl", 15, &CanTransmitter::can_callback, &tx0);
  /// create instance of a subscriber to take specific command based input
  data_pubs =
      nh.advertise<mid_level_controller::Can_Status_Message>("canStatus", 10);
  PRINT_FUNC_EXIT;
}
// start publish to topic in a threaded manner
void CanTransceiver::start_data_publisher_thd()
{
  PRINT_FUNC_ENTER;
  if (*exit_flag_pointer_) return;
  dataPublisherIsThreaded = true;
  data_publisher_0 = std::thread(&CanTransceiver::start_data_publisher, this);
  PRINT_FUNC_EXIT;
}
// start publish to topic
void CanTransceiver::start_data_publisher()
{
  PRINT_FUNC_ENTER;
  /// we are going to publish data at 100 Hz to the can
  ros::Rate loop_rate(500);
  while (!exit_flag && ros::ok() && !*exit_flag_pointer_)
  {
    if (rx0.new_incoming_data)
    {
      data_pubs.publish(rx0.output1);
      rx0.new_incoming_data = 0;
    }
    if (dataPublisherIsThreaded) ros::spinOnce();
    loop_rate.sleep();
    exit_flag = *exit_flag_pointer_;
  }
  PRINT_FUNC_EXIT;
}
#endif
CanTransceiver::~CanTransceiver()
{
  PRINT_FUNC_ENTER;
  exit_flag = true;
  // closing other things
  // signalHandler(1);
  if (data_publisher_0.joinable()) data_publisher_0.join();
  PRINT_FUNC_EXIT;
}
// clean exit
void CanTransceiver::transreceiver_exit(int signum)
{
  PRINT_FUNC_ENTER;
  /// clean exit
  exit_flag = 1;
  tx0.signalHandler(signum);
  rx0.signalHandler(signum);
#ifndef UTILS_AVOID_GFLAGS
  gflags::ShutDownCommandLineFlags();
#endif
  PRINT_FUNC_EXIT;
}
/// cleanly exit from transmitter
void CanTransceiver::signalHandler(int signum)
{
  PRINT_FUNC_ENTER;
  PI$ "Closing transceiver instance" pendl;
  /// clean exit
  exit_flag = 1;
  PRINT_FUNC_EXIT;
  transreceiver_exit(signum);
}
