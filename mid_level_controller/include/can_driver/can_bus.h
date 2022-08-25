// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_bus.h
 * @brief transreceiver for can. This file can be used to send and recieve
 * messages over can given that it is setup already and store commands in a log
 * file
 * @author Divyanshu Goel
 * @date 2017-02-06 (yyyy-mm-dd)
 **/
#ifndef CAN_DRIVER_CAN_BUS_H
#define CAN_DRIVER_CAN_BUS_H

#include <endian.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "can_driver/can_bus.h"
#include "can_driver/logger.h"

/**
 * @brief This class will be used for exception handling
 */
class readexception : public std::exception
{
  /// handle for read exception
  virtual const char* what() const throw() { return "Device read failed"; }
};
/**
 * @brief This class will be used for exception handling
 */
class writeexception : public std::exception
{
  /// handle for write exception
  virtual const char* what() const throw() { return "Device write failed"; }
};
/**
 * @brief This class will be used for exception handling
 */
class connectexception : public std::exception
{
  /// handle for connection exception
  virtual const char* what() const throw()
  {
    return "Device connection failed";
  }
};
/**
 * @brief This class will be used for exception handling
 */
class socketexception : public std::exception
{
  /// handle for socket exception
  virtual const char* what() const throw()
  {
    return "Device scoket operation failed";
  }
};
/**
 * @brief This class will be used for exception handling
 */
class IOCTLexception : public std::exception
{
  /// handle for IOCTL exception
  virtual const char* what() const throw()
  {
    return "Device connection failed";
  }
};
/**
 * @brief This class will be used for I/O to can bus and contains functions to
 * convert message to bits and viceversa.
 */
class CanBus
{
 public:
  /// exception handling for read
  readexception readex;
  /// exception handling for write
  writeexception writeex;
  /// exception handling for connect
  connectexception connectex;
  /// exception handling for socket
  socketexception socketex;
  /// exception handling for ioctl
  IOCTLexception IOCTLex;
  /// socket variable for can transmission
  int soc;
  /// create instance for log file
  Logger log_file;
  /// if log is enabled or not
  bool log_enabled = true;
  /// if new incoming data is available or not
  bool new_incoming_data = 0;
  /// incoming data variables
  std::vector<canfd_frame> incoming_message;
  /// incoming data variables
  struct canfd_frame frame_temp[20];
  /// incoming data variables
  int frame_count = 0;
  /// incoming data variables
  int max_no_frames = 6;
  /**
   * @brief This method will be used to print a single character to the lcd.
   * @param port The string which will contain the name of the can port namely
   * can0 or vcan0. Please Note that the program does not initiate connection
   * until bitrate is set and connection is up and running.
   * @param logfilepath path where file will be created and rx/tx shall be
   * written only location to folder needed. needs last /.
   * @param log_enabler if log is to be enabled or not
   * @param exit_flag_arg_  pointer to exit flag for safe exit
   */
  int open_port(const std::string port, const std::string logfilepath,
                bool log_enabler, const int* exit_flag_arg_);
  /**
   * @brief This method will be used to send data over can.
   * @param *frame can_frame that consists of message_id and data. As of now
   * only 8 bytes of data can be sent. Need to expand it more.
   */
  int send_port(struct canfd_frame* frame);
  /**
   * @brief This method will be used to send data over can.
   * @param *frame can_frame that consists of message_id and data. As of now
   * only 8 bytes of data can be sent. Need to expand it more.
   */
  int send_port(struct can_frame* frame);
  /**
   * @brief This method will be used to recieve data over can.
   * @note Please run in a different thread if possible. This listens to the
   * port continously. and would stop using of other functions.
   */
  void read_port();
  /**
   * @brief This method will be used to set filter for the can read .
   * @param *can_id - (int) Set filter can_id.
   * @param *can_filter - (int) set can mask.
   * @note only messages which pass can_id & can_filter == can_msg_id &
   * can_filter are in the resulting stream.
   */
  int set_filter(unsigned int can_id, unsigned int can_filter);
  /**
   * @brief This method will be used tocopy canfd_frame to can_frame.
   * @param *frame - canfd frame.
   */
  can_frame copy_fd_frame_to_frame(struct canfd_frame* frame);
  /**
   * @brief This method will be used to convert a 64 byte no in a corresponding
   * integer array of size 8 for sending on can transmission sytem
   * @param out_array output array unsigned int 8 bits of size 8
   * @param byte_no_input input 64 bit number
   */
  static inline void byte_to_array(uint8_t (&out_array)[8],
                                   uint64_t byte_no_input)
  {
    uint8_t temp;
    for (int i = 0; i < 8; ++i)
    {
      temp = byte_no_input & 0b11111111;
      out_array[i] = temp;
      byte_no_input = byte_no_input >> 8;
    }
  };
  /**
   * @brief This method will be used to convert two 64 byte no in a
   * corresponding integer array of size 16 for sending on can transmission
   * sytem
   * @param out_array output array unsigned int 8 bits of size 16
   * @param byte_no_input input 64 bit number array of size 2 [2]
   */
  static inline void byte_to_array(uint8_t (&out_array)[16],
                                   uint64_t byte_no_input[2])
  {
    uint8_t temp;
    for (int i = 0; i < 8; ++i)
    {
      temp = byte_no_input[0] & 0b11111111;
      out_array[i] = temp;
      byte_no_input[0] = byte_no_input[0] >> 8;
    }
    for (int i = 0; i < 8; ++i)
    {
      temp = byte_no_input[1] & 0b11111111;
      out_array[8 + i] = temp;
      byte_no_input[1] = byte_no_input[1] >> 8;
    }
  };
  /**
   * @brief This method will be used to convert two 64 byte no in a
   * corresponding integer array of size 16 for sending on can transmission
   * sytem
   * @param byte_no_output output 64 bit number
   * @param input_array output array unsigned int 8 bits of size 8
   */
  static inline void array_to_byte(uint64_t* byte_no_output,
                                   uint8_t input_array[8])
  {
    (*byte_no_output) = 0;
    uint64_t temp = 0;
    for (int i = 0; i <= 7; i++)
    {
      temp = input_array[i] * pow(2, 8 * i);
      (*byte_no_output) = (*byte_no_output) + temp;
    }
  };
  /**
   * @brief This method will be used to close can socket associated with the
   * class.
   */
  int close_port();

 private:
  /// var for condition to exit cleanly
  bool read_exit_flag = 0;
  /// pointer to exit flag from signal handler
  const int* exit_flag_pointer_;
};
/**
 * @brief This method will be used to update speed and other things from can
 * gateway
 * @param msg pointer for feedback message
 */
static double get_timestamp()
{
  auto start = std::chrono::high_resolution_clock::now();
  return (start.time_since_epoch().count() / 1000);
};
#endif  // CAN_DRIVER_CAN_BUS_H
