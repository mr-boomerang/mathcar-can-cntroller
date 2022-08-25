// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_transceiver.h
 * @brief transmitter and receiver for can. This file can be used to transmit
 * and receive messages over can every 10 ms
 * @author Divyanshu Goel
 * @date 2017-01-11 (yyyy-mm-dd)
 **/

#ifndef CAN_DRIVER_CAN_TRANSCEIVER_H
#define CAN_DRIVER_CAN_TRANSCEIVER_H

#include "can_driver/can_receiver.h"
#include "can_driver/can_transmitter.h"
#include <mid_level_controller/config.h>
#include <string>

/**
 * @brief This class will be used to send/receive data over can after reading
 * the yaml settings file.
 * @author Divyanshu Goel
 * @date 2017-02-28
 */
class CanTransceiver
{
 public:
  /// check for valid initialization
  bool init_success = false;
  ///@{ default file paths
  std::string control_yaml_file_path, status_yaml_file_path, log_file_path,
      settings_file_path;
  ///@}
  /// if data is initialized by master class
  bool master_class_data_read = false;
  /// if flags are read properly
  bool flags_set = false;
  /// instance of transmitter
  CanTransmitter tx0;
  /// instance of receiver
  CanReceiver rx0;
  /// thread for dedcated writer to the data
  std::thread data_publisher_0;
  /// flag to allow for spin processing
  bool dataPublisherIsThreaded = false;
  /// pointer to exit flag from signal handler
  const int* exit_flag_pointer_;

#ifndef BUILD_WITHOUT_ROS
  /// create instance of a subscriber to take specific command based input
  ros::Subscriber data_subs;
  /// create instance of a subscriber to take specific command based input
  ros::Publisher data_pubs;
#endif
  // destructor~
  /**
   * @brief This method as destructor
   * @author Divyanshu Goel
   * @date 05/11/2017
   */
  ~CanTransceiver();
  /**
   * @brief To initialiaze the values for the paths overloaded function
   * @author Divyanshu Goel
   * @param settings_file_path input settings file path
   * @param control_input_yaml_file_path input settings file path for control
   * @param status_input_yaml_file_path input settings file path for status
   * @param log_file_path path where log file will be stored(absolute and
   * contains / at end, make sure it exists)
   * @param settings_file_path settings file path
   * @param exit_flag_arg_ pointer to exit flag for safe exit
   * @date 2017-11-2
   */
  void init(const std::string control_input_yaml_file_path,
            const std::string status_input_yaml_file_path,
            const std::string log_file_path,
            const std::string settings_file_path, const int* exit_flag_arg_);
  /**
   * @brief to initialiaze the values for the paths overloaded function
   * @author Divyanshu Goel
   * @param argc parameter count
   * @param argv parameter value
   * @param exit_flag_arg_ pointer to exit flag for safe exit
   * @date 2017-11-2
   */
  void init(int argc, char** argv, const int* exit_flag_arg_);
  /**
   * @brief to initialiaze the values for the paths overloaded function
   * @author Divyanshu Goel
   * @param argc parameter count
   * @param argv parameter value
   * @date 2017-11-2
   */
  void init_parse_flags(int argc, char** argv);
  /**
   * @brief to start rx and tx over CAN
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void start_transreceiver();
  /**
   * @brief This method will be used to cleanly exit from the transreceiver.
   * @author Divyanshu Goel
   * @param signum signal value
   * @date 2017-11-11
   */
  void transreceiver_exit(int signum);
#ifndef BUILD_WITHOUT_ROS
  /**
   * @brief This method will be used to setup ros handles
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void create_data_publisher_handles();
  /**
   * @brief This method will be used to start publishing in a threaded manner.
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void start_data_publisher_thd();
  /**
   * @brief This method will be used to start publishing.
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void start_data_publisher();
#endif
  /**
   * @brief This method will be used to handle interrupts.
   * @author Divyanshu Goel
   * @param signum value what is the signal value that has terminated the
   * program.
   * @date 2017-02-09
   */
  void signalHandler(int signum);
  /// flag for clean exit
  bool exit_flag = 0;
};
/**
 * @brief This method will be used to handle interrupts.
 * @author Divyanshu Goel
 * @param signum value what is the signal value that has terminated the
 * program.
 * @date 2017-02-09
 */
void mysignalHandler(int signum);

#endif  // CAN_DRIVER_CAN_TRANSCEIVER_H
