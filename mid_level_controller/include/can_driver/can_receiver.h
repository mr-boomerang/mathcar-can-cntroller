// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_receiver.h
 * @brief receiver for can. This file can be used to recieve
 * messages over can and sort them out by the given dictionary and update on the
 * publisher
 * @author Divyanshu Goel
 * @date 2018-02-28 (yyyy-mm-dd)
 **/

#ifndef CAN_DRIVER_CAN_RECEIVER_H
#define CAN_DRIVER_CAN_RECEIVER_H
#ifndef BUILD_WITHOUT_ROS
#include "mid_level_controller/Can_Status_Message.h"
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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "can_driver/can_bus.h"
#include "can_driver/can_struct.h"
#include "can_driver/can_yaml_parser.h"
#include "linux/can.h"
/// allows allocation of maximum size of buffer for character input
#define BUFSIZE 128

/**
 * @brief This class will be used to parse the incoming can message and then
 * send data over ROS topic
 * @author Divyanshu Goel
 * @date 2017-02-28
 */
class CanReceiver
{
 public:
  ///@{ default settings for can bus
  char *yaml_file_path, *log_file_path, *can_settings_file_path;
  std::string can_interface = "vcan0";
  unsigned int incoming_queue_size = 14;
  ///@}
  /// to print support options
  bool print_help;
  /// check for valid initialization
  bool init_success = false;
  /// if data is initialized by master class
  bool params_loaded = false;
  /// if new incoming data is available or not
  bool new_incoming_data = 0;
  /// Instance of messages containing status and control messages
  CanYamlParser status;
  /// Instance of can bus to send data
  CanBus bus_receiver;
  /// output coming from receiver
  OutputMsg output0;
  /// data file log object
  Logger data_log_file;
  /// pointer to exit flag from signal handler
  const int* exit_flag_pointer_;
#ifndef BUILD_WITHOUT_ROS
  /// for output to ros
  mid_level_controller::Can_Status_Message output1;
#endif
  /// total count of unknown messages
  unsigned int unknown_count = 0;
  ///@{ default settings for can bus to enable log, to enable print
  bool log_enable = false, debug_enable = true, raw_msg_enable = false;
  int log_data_enable = 0;
  canfd_frame incoming_messages[20];
  canfd_frame single_incoming_message;
  ///@}

  ///@{ empty dictionary and sstream for unknowns
  DictionaryMap empty_dictionary_map;
  std::stringstream newunknownmessage;
  InputParticularsMap empty_particulars, data_particulars;
  CanTopicsDataMap empty_topics_data_map;
  ///@}
  /// instance contains data related to topics for use in callback function
  CanTopicsDataMap status_map;
  /// thread object for can bus receiver
  std::thread can_bus_listener;
  /// thread object for can data parser
  std::thread can_bus_parser0;
  /// flag to allow for spin processing
  bool canBusParserIsThreaded = false;
  /// flag to allow for spin processing
  bool canBusListenerIsThreaded = false;
  // destructor~
  /**
   * @brief This method as destructor
   * @author Divyanshu Goel
   * @date 05/11/2017
   */
  ~CanReceiver();
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
   * @date 2017-11-2
   */
  void init(const std::string input_yaml_file, const std::string logfilepath,
            const std::string can_settings_file_path, const int* exit_flag_arg_,
            bool print_help_flag, bool virtual_can_startup);
  /**
   * @brief get string associated with value in the dictionary map
   * @author Divyanshu Goel
   * @param input_value input value that is to be searched
   * @param input_map input dictionary that e used for searching
   * @date 2017-03-2
   */
  std::string get_key_dictionary(unsigned int input_value,
                                 DictionaryMap input_map);
  /**
   * @brief get string associated with value in the dictionary map
   * @author Divyanshu Goel
   * @param result value to be copied
   * @param topic_value topic for which data has to be changed
   * @date 2017-03-21
   */
  void insert_output_topic_data(std::string result, std::string topic_value);
  /**
   * @brief execute a system command
   * @author Divyanshu Goel
   * @param command to be executed
   * @date 2017-03-21
   */
  int exec_command(std::string command);
  /**
   * @brief prints output
   * @author Divyanshu Goel
   * @date 2017-03-21
   */
  void print_output();
  /**
   * @brief open file handle for data
   * @author Divyanshu Goel
   * @date 2017-11-16
   */
  void open_log_file(const std::string logfilepath);
  /**
   * @brief write data to file from the reciever after parsing
   * @author Divyanshu Goel
   * @date 2017-11-16
   */
  void write_to_data_log_file(OutputMsg output_temp);
  /**
   * @brief clean exit of file writing
   * @author Divyanshu Goel
   * @date 2017-11-16
   */
  void close_data_log_file();
  /**
   * @brief This method will be used to print can frame as string
   * @author Divyanshu Goel
   * @param message_temp input canfd_frame which is to be printed
   * @date 2017-02-28
   */
  std::string can_frametostr(canfd_frame message_temp);
  /**
   * @brief This method will be used to find an+d retu+rn index of given element
   * in can array
   * @author Divyanshu Goel
   * @param can_message input canfd_frame which is to be searched
   * @date 2017-02-28
   */
  int get_index(canfd_frame can_message);
  /**
   * @brief This method will be used to listen to raw can messages
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void start_can_listener_thd();
  /**
   * @brief This method will be used to collect and issue parse command for can
   * message
   * @author Divyanshu Goel
   * @date 2017-11-11
   */
  void start_can_listener();
  /**
   * @brief This method will be used to parse the incoming can message
   * @author Divyanshu Goel
   * @param can_message input canfd_frame which is to be parsed
   * @param incoming_data_len total number of can frames incoming
   * @date 2017-02-28
   */
  void msg_parse(canfd_frame can_message[], int incoming_data_len);
  /**
   * @brief This method will be used to print can data stored in the arrays.
   * @author Divyanshu Goel
   * @param input_map - input processed structure tailored to status messages
   * @date 2017-02-28
   */
  void print_processed_structure_stored(InputParticularsMap input_map);
  /**
   * @brief This method will be used to print can message array.
   * @author Divyanshu Goel
   * @param can_message_array - input canfd_frame
   * @date 2017-02-28
   */
  void print_can_message_vector(std::vector<canfd_frame> can_message_array);
  /**
   * @brief This method will be used to print various topics other different
   * values like name,dictionary, start bits in single array for each one
   * @author Divyanshu Goel
   * @param input_map struct topics data map that needs to be printed
   * @date 2017-02-22
   */
  void print_structure_stored(CanTopicsDataMap input_map);
  /**
   * This method will be used to setup all the variables and data for the
   * program to work.
   * @param input_yaml_file input settings file path
   * @param logfilepath path where log file will be stored(absolute and contains
   * / at end, make sure it exists)
   * @param can_settings_file_path can settings file path
   * @param print_help_flag print I/O for input output functions
   * @param virtual_can_startup start virtual can
   */
  void receiver_setup(const std::string input_yaml_file,
                      const std::string logfilepath,
                      const std::string can_settings_file_path,
                      bool print_help_flag, bool virtual_can_startup);
  /**
   * @brief This method will be used to exit from the reciever
   * @param signum value what is the signal value that has terminated the
   * program.
   */
  void receiver_exit(int signum);
  /**
   * @brief This method will be used to handle interrupts.
   * @param signum value what is the signal value that has terminated the
   * program.
   */
  void signalHandler(int signum);

 private:
  // to ensure clean exit
  bool exit_flag = 0;
};
#endif  // CAN_DRIVER_CAN_RECEIVER_H
