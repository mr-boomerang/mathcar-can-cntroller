// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_transmitter.h
 * @brief transmitter for can. This file can be used to transmit
 * messages over can every 10 ms
 * @author Divyanshu Goel
 * @date 2018-02-28 (yyyy-mm-dd)
 **/

#ifndef CAN_DRIVER_CAN_TRANSMITTER_H
#define CAN_DRIVER_CAN_TRANSMITTER_H

#ifndef BUILD_WITHOUT_ROS
#include "mid_level_controller/Callback_Msg.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#endif

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

#include <algorithm>
#include <bitset>
#include <csignal>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include "can_driver/can_bus.h"
#include "can_driver/can_struct.h"
#include "can_driver/can_yaml_parser.h"
/// allows allocation of maximum size of buffer for character input
#define BUFSIZE 128

/**
 * @brief This class will be used to send data over can after reading the yaml
 * settings file.
 */
class CanTransmitter
{
 public:
  /// to print support options
  bool print_help;
  /// check for valid initialization
  bool init_success = false;
  /// object for data read from yaml parser
  CanTopicsDataMap control_map;
  ///@{ Instance of messages containing status and control messages
  CanYamlParser control, status;
  ///@}
  /// if data is initialized by master class
  bool params_loaded = false;
  /// Instance of can bus to send data
  CanBus transmitter;
  ///@{ settings for can
  bool log_enable = false, debug_enable = true, raw_msg_enable = false;
  int log_data_enable = 0;
  bool root_password_present = true;
  ///@}
  ///@{ paths to settingsfile,logfile
  char *yaml_file_path, *log_file_path, *can_settings_file_path;
  std::string can_interface = "vcan0", sudo_password = "";
  ///@}
  /// data file log object
  Logger data_log_file;
  /// thread for dedicatatedly sending data to bus
  std::thread can_bus_sender;
  /// flag to allow for spin processing
  bool canBusSenderIsThreaded = false;
  /// pointer to exit flag from signal handler
  const int* exit_flag_pointer_;
  // destructor~
  ~CanTransmitter();
  /**
   * @brief to initialiaze the values for the paths overloaded function
   * @author Divyanshu Goel
   * @param input_yaml_file input settings file path
   * @param logfilepath path where log file will be stored(absolute and
   * contains / at end, make sure it exists)
   * @param can_settings_file_path can settings file path
   * @param exit_flag_arg_ pointer to exit flag for safe exit
   * @param print_help_flag print I/O for input output functions
   * @param virtual_can_startup start virtual can
   */
  void init(const std::string input_yaml_file, const std::string logfilepath,
            const std::string can_settings_file_path, const int* exit_flag_arg_,
            bool print_help_flag = false, bool virtual_can_startup = false);
  /**
   * @brief to send data over CAN
   * @author Divyanshu Goel
   * @date 2017-11-02
   */
  void start_can_transmitter();
  /**
   * @brief to send data over CAN in a threaded manner
   * @author Divyanshu Goel
   * @date 2017-11-02
   */
  void start_can_transmitter_thd();
  /**
   * @brief to update data to be sent over CAN
   * @param item_requested input command
   * @param item_data value associated with the command
   */
  void update_transmitter_data(std::string item_requested,
                               std::string item_data);
  /**
   * @brief This method will be used to print can message array
   * @param can_message_array - contains all the message that need to be sent on
   * can
   */
  void print_can_message_vector(std::vector<canfd_frame> can_message_array);
  /**
   * @brief gives value message for can message transmission
   * @param start - start bit where message is to be shifted
   * @param length - no of zeroes to put in message after start bit
   */
  static inline uint64_t returns_reset_msg(uint64_t start, uint64_t length)
  {
    uint64_t number = -1, top = -1, bottom = 0;
    uint64_t top_remainder_value = 0;
    if (start + length <= 64)
    {
      top_remainder_value = pow(2, start + length);
      top_remainder_value = top_remainder_value - 1;
      bottom = pow(2, start);
      bottom = bottom - 1;
    }
    number = top - top_remainder_value + bottom;
    return (number);
  };
  /**
   * @brief execute a system command
   * @param command to be executed
   */
  int exec_command(std::string command);
  /**
   * @brief gives value message for can message transmission.
   * @param start - start bit where message is to be shifted
   * @param value - value to set
   */
  static inline uint64_t returns_value_msg(uint64_t start, uint64_t value)
  {
    uint64_t number = -1;
    number = value * pow(2, start);
    return (number);
  };
  /**
   * @brief This method will be used to print all the variables and data in the
   * map
   * @param input_map - contains all the parsed information
   */
  void print_structure_stored(CanTopicsDataMap input_map);
  /**
   * @brief open file handle for data
   */
  void open_log_file(const std::string logfilepath);
  /**
   * @brief write data to file from the reciever after parsing
   * @param item_requested - input command
   * @param item_data - value associated with the command
   */
  void write_to_data_log_file(std::string item_requested,
                              std::string item_data);
  /**
   * @brief clean exit of file writing
   */
  void close_data_log_file();
  /**
   * @brief setup can to start the connection
   * @author Divyanshu Goel
   * @date 2017-11-16
   */
  int can_setup();
  /**
   * @brief This method will be used to setup all the variables and data for the
   * program to work.
   * @param input_yaml_file input settings file path
   * @param logfilepath - path where log file will be stored(absolute and
   * contains / at end, make sure it exists)
   * @param can_settings_file_path can settings file path
   * @param print_help_flag print I/O for input output functions
   * @param virtual_can_startup start virtual can
   */
  void transmitter_setup(const std::string input_yaml_file,
                         const std::string logfilepath,
                         const std::string can_settings_file_path,
                         bool print_help_flag, bool virtual_can_startup);
  /**
   * @brief This method will be used to cleanly exit from the transmitter.
   * @param signum signal value
   */
  void transmitter_exit(int signum);
  /**
   * @brief This method will be used to handle interrupts.
   * @param signum value what is the signal value that has terminated the
   * program.
   */
  void signalHandler(int signum);
#ifndef BUILD_WITHOUT_ROS
  /**
   * @brief This method will be used to update message sent on can by the
   * publisher call to /can/control
   * @param msg pointer to ROS CAN_Callback Msg
   */
  void can_callback(const mid_level_controller::Callback_Msg::ConstPtr& msg);
#endif
 private:
  /// loop exit variable
  bool exit_flag = 0;
};
#endif  // CAN_DRIVER_CAN_TRANSMITTER_H
