// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_receiver_vehicle.h
 * @brief receiver for CAN over vehicle CAN. This file can be used to recieve
 * messages over CAN and sort them out by the given dictionary and update on the
 * publisher
 * @author Divyanshu Goel
 * @date 2017-02-28 (yyyy-mm-dd)
 **/
#ifndef CAN_DRIVER_CAN_RECEIVER_VEHICLE_H
#define CAN_DRIVER_CAN_RECEIVER_VEHICLE_H

#ifndef BUILD_WITHOUT_ROS
#include "mid_level_controller/Can_Status_Message.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#endif

#include "linux/can.h"
#include <algorithm>
#include <bitset>
#include <csignal>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "can_driver/can_bus.h"
#include "can_driver/can_receiver.h"
#include "can_driver/can_yaml_parser.h"

/**
 * @brief This class will be used to parse the incoming can message and then
 * send data over ROS
 */
class CanReceiverVehicle
{
 public:
  /// pointer to exit flag from signal handler
  int* exit_flag_pointer_;
  /// check for valid initialization
  bool init_success = false;
  /// Instance of messages containing status and control messages
  CanYamlParser status;
  /// Instance of can bus to send data
  CanBus bus_receiver;
  /// output going to the ros publisher/ feedback object
  output_msg output0;
  /// incoming_messages are parsed and finally stored in this variable
  std::vector<output_msg> recieved_data;
  /// for output to ros
  mid_level_controller::Can_Status_Message output1;
  /// total count of unknown messages
  unsigned int unknown_count = 0;
  ///@{ default settings for can bus to enable log, to enable print
  bool log_enable = false, debug_enable = true, raw_msg_enable = false;
  canfd_frame incoming_messages[20];
  canfd_frame single_incoming_message;
  std::string can_interface = "vcan0", sudo_password = "";
  bool root_password_present = true;
  ///@}
  ///@{ default settings for can bus
  char *yaml_file_path, *log_file_path, *can_settings_file_path;
  unsigned int incoming_queue_size = 14;
  ///@}

  ///@{ empty dictionary and sstream for unknowns
  struct dictionary_map empty_dictionary_map;
  std::stringstream newunknownmessage;
  input_particulars_map empty_particulars, data_particulars;
  can_topics_data_map empty_topics_data_map;
  ///@}
  /// instance contains data related to topics for use in callback function
  can_topics_data_map status_map;
  /**
   * @brief get string associated with value in the dictionary map
   * @param input_value input value that is to be searched
   * @param input_map input dictionary that e used for searching
   */
  std::string get_key_dictionary(unsigned int input_value,
                                 dictionary_map input_map);
  /**
   * @brief get string associated with value in the dictionary map
   * @param result value to be copied
   * @param topic_value topic for which data has to be changed
   */
  void insert_output_topic_data(std::string result, std::string topic_value);
  /**
   * @brief prints output
   */
  void print_output();
  /**
   * @brief This method will be used to print can frame as string
   * @param message_temp input canfd_frame which is to be printed
   */
  std::string can_frametostr(canfd_frame message_temp);
  /**
   * @brief This method will be used to find and return the index of given
   * element in can array
   * @param can_message input canfd_frame which is to be searched
   */
  int get_index(canfd_frame can_message);
  /**
   * @brief This method will be used to listen to can connection at higher
   * speeds
   */
  void can_listener();
  /**
   * @brief This method will be used to parse the incoming can message
   * @param can_message input canfd_frame which is to be parsed
   * @param incoming_data_len total number of can frames incoming
   */
  void msg_parse(canfd_frame can_message[], int incoming_data_len);
  /**
   * @brief This method will be used to print can data stored in the arrays.
   * @param input_map - input processed structure tailored to status messages
   */
  void print_processed_structure_stored(input_particulars_map input_map);
  /**
   * @brief This method will be used to print can message array.
   * @param can_message_array - input canfd_frame
   */
  void print_can_message_vector(std::vector<canfd_frame> can_message_array);
  /**
   * @brief This method will be used to print various topics other different
   * values like name,dictionary, start bits in single array for each one
   * @param input_map struct topics data map that needs to be printed
   */
  void print_structure_stored(can_topics_data_map input_map);
  /**
   * @brief This method will be used to setup all the variables and data for the
   * program to work.
   * @param input_yaml_file input settings file path
   * @param logfilepath path where log file will be stored(absolute and
   * contains / at end, make sure it exists)
   * @param can_settings_file_path can settings file path
   */
  void receiver_setup(const std::string input_yaml_file,
                      const std::string logfilepath,
                      const std::string can_settings_file_path);
  /**
   * @brief This method will be used to exit from the reciever
   */
  void receiver_exit();
  /**
   * @brief This method will be used to handle interrupts.
   * @param signum value what is the signal value that has terminated the
   * program.
   */
  void signalHandler(int signum);
};
/**
 * @brief This method will be used to handle interrupts.
 * @param signum value what is the signal value that has terminated the
 * program.
 */
void mysignalHandler(int signum);
#endif  // CAN_DRIVER_CAN_RECEIVER_VEHICLE_H
