// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file logger.h
 * @brief print can messages to log file given location including the name of
 * file preferably absolute path.
 * @author Divyanshu Goel
 * @date 2018-02-28 (yyyy-mm-dd)
 */
#ifndef CAN_DRIVER_LOGGER_H
#define CAN_DRIVER_LOGGER_H

#include <chrono>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include "linux/can.h"
#include "utils/mode_printing.h"
/**
 * @brief This class will be used to write data to a file.
 * file.
 */
class Logger
{
 public:
  /// log file object
  std::ofstream myfile;
  /**
   * @brief This method will be used to create empty log file and enter in that
   * file that log has started.
   * @author Divyanshu Goel
   * @param *filepath The string which will contain the path to the log file
   * @date 02/28/2017
   */
  void open_file(const std::string filepath);
  /**
   * @brief This method will be used to flush stream to the file
   * @author Divyanshu Goel
   * @date 16/11/2017
   */
  void flush();
  /**
   * @brief This method will be used to print a single can fd frame to the log
   * file.
   * @author Divyanshu Goel
   * @param temp The canfd_frame which will be written to the log file.
   * @date 02/28/2017
   */
  void print_canfdframe_to_file(canfd_frame temp);
  /**
   * @brief This method will be used to print a single can frame to the log
   * file.
   * @author Divyanshu Goel
   * @param temp The can_frame which will be written to the log file.
   * @date 02/28/2017
   */
  void print_canframe_to_file(can_frame temp);
  /**
   * @brief This method will be used to print a single char array to the log
   * file.
   * @author Divyanshu Goel
   * @param *data - The character array which will be written to the log
   * file.
   * @note no new line at the end
   * @date 02/28/2017
   */
  void print_string_message_to_file(std::string data);
  /**
   * @brief This method will be used to print a string to the log file.
   * @author Divyanshu Goel
   * @param data - The string which will be written to the log
   * file.
   * @note no new line at the end
   * @date 02/28/2017
   */
  void print_strmessage_to_file(const std::string data);
  /**
   * @brief This method will be used to print a new line in the log file.
   * @author Divyanshu Goel
   * @date 02/28/2017
   */
  void print_newline_to_file();
  /**
   * @brief This method will be used to close the file safely after entering the
   * closing time and date.
   * @author Divyanshu Goel
   * @date 02/28/2017
   */
  void close_file();
};
#endif  // CAN_DRIVER_LOGGER_H
