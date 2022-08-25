// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_struct.h
 * @brief strucutes for CAN
 * @author Divyanshu Goel
 * @date 2017-01-11 (yyyy-mm-dd)
 **/

#ifndef CAN_DRIVER_CAN_STRUCT_H
#define CAN_DRIVER_CAN_STRUCT_H

#include "linux/can.h"
#include <chrono>  /// for time keeping
#include <string>
#include <vector>

/// Standard metadata for higher-level stamped data types.
struct Header
{
  /// sequence ID: consecutively increasing ID
  unsigned int seq;
  /// timestamp
  double stamp;
  /// Frame this data is associated with
  std::string frame_id;
  Header()
  {
    /// set default values for Header on init
    Clear();
  }
  // to set the value back to zero
  void Clear()
  {
    seq = 0;
    /// set current time in us
    auto start = std::chrono::high_resolution_clock::now();
    stamp = start.time_since_epoch().count() / 1000;
    frame_id = "";
  }
  void SetTime()
  {
    /// set current time in us
    auto start = std::chrono::high_resolution_clock::now();
    stamp = start.time_since_epoch().count() / 1000;
  }
};

/// Structure of dictionary containing valid match keys and values
struct DictionaryMap
{
  /// contains keys for a given submenssage
  std::vector<std::string> key;
  /// contains corresponding value for a given submenssage
  std::vector<unsigned int> value;
};

/// Structure of can_messages status or control and what info to expect of them
struct CanMessage
{
  /// contains name for the message
  std::string message_name = "";
  /// contains arbitration id for the given message
  unsigned int message_id = 0;
  /// contains total no of message in the given message
  unsigned int message_count = 0;
  /// contains total length of the given message
  unsigned int total_length = 0;
  /// contains start bits for the submessages for the given message
  std::vector<unsigned int> start_bit;
  /// contains validity bits for the submessages for the given message
  std::vector<unsigned int> validity_bit;
  /// contains message lengths for all the submessages for the given message
  std::vector<unsigned int> message_length;
  /// contains names for the submessages in the given message
  std::vector<std::string> names;
  /// contains dictionary map for the submessage message
  std::vector<DictionaryMap> dictionary;
};
/**
 * @brief This struct forms the datastructure to contain all the data pertaining
 * to the transmission as read from the file.
 */
struct CanTopicsDataMap
{
  /// frames that will be sent go in here
  std::vector<canfd_frame> can_message_array_map;
  /// contains key and value pairs associated with the message
  std::vector<DictionaryMap> can_dictionary_map;
  /// contains topic names associated with the message
  std::vector<std::string> can_topics_map;
  /// contains total message length associated with the message
  std::vector<unsigned int> can_total_length_map;
  /// contains start bits in all the message
  std::vector<unsigned int> can_start_bit_map;
  /// contains message length or no of bits of message
  std::vector<unsigned int> can_message_length_map;
  /// contains validity bits
  std::vector<unsigned int> can_validity_bits_map;
  /// contains no of messages in a single transmission
  std::vector<unsigned int> can_message_count_map;
  /// contains message_id
  std::vector<unsigned int> can_arbitration_id_map;
};
/**
 * @brief This struct forms the data structure to contain all the data
 *pertaining to the recieving as read from the file.
 **/
struct InputParticularsMap
{
  /// frames that will be recieved go in here
  std::vector<canfd_frame> input_can_message_array_map;
  /// all data needed for parsing
  std::vector<CanTopicsDataMap> input_can_topics_data_map;
};
/**
 * @brief This struct forms the data structure to contain only output data
 */
struct OutputMsg
{
  /// @{ data that will be published go in here
  std::vector<std::string> topic;
  std::vector<std::string> value;
  std::vector<std::string> raw_data;
  std::vector<unsigned int> msg_count;
  ///@}
};
/// This struct forms the data structure to contain current state of car
struct CarState
{
  /// header for steer
  Header steer_header;
  /// header for speed
  Header speed_header;
  /// speed of car
  int speed_fdbk = 0;
  /// steer of car applied in degrees left positive and right is negative
  int steer_cmd = 0;
  /// steer of car read in degrees left positive and right is negative
  int steer_fdbk = 0;
  /// current gear possible values: neutral, reverse, boost, fwddrive
  std::string gear_cmd = "neutral";
  /// current gear possible values: neutral, reverse, boost, fwddrive
  std::string gear_fdbk = "neutral";
  /// current indicator possible values: off, left, right, hazard
  std::string indicator_cmd = "off";
  /// current indicator possible values: off, left, right, hazard
  std::string indicator_fdbk = "off";
  /// wiper state applied
  bool wiper_cmd = 0;
  /// wiper state feedback
  bool wiper_fdbk = 0;
  /// horn state applied
  bool horn_cmd = 0;
  /// horn state feedback
  bool horn_fdbk = 0;
  /// headlight state applied
  bool headlight_cmd = 0;
  /// headlight state feedback
  bool headlight_fdbk = 0;
  /// throttle applied in percentage
  int throttle_cmd = 0;
  /// throttle read in percentage
  int throttle_fdbk = 0;
  /// brake applied
  int brake_cmd = 0;
  /// brake read
  int brake_fdbk = 0;
  /// velocity target
  double velocity_target = 0;
  /// steer target
  double steer_target = 0;
  /// steer offset
  double steer_offset = 0;
  /// steer limit in degrees
  double steer_limit = 40;
  /// velocity limit in mps
  double velocity_limit = 5;
  /// throttle limit in percentage
  double throttle_limit = 60;
  /// auto mode count
  int auto_mode_count = 0;
  /// time_counter for steering
  int t_equation = 0;

  CarState()
  {
    // set values reset
    Clear();
  }
  void Clear()
  {
    /// header for steer
    steer_header.SetTime();
    /// header for speed
    speed_header.SetTime();
    /// speed of car
    speed_fdbk = 0;
    /// steer of car applied in degrees left positive and right is negative
    steer_cmd = 0;
    /// steer of car read in degrees left positive and right is negative
    steer_fdbk = 0;
    /// current gear possible values: neutral, reverse, boost, fwddrive
    gear_cmd = "neutral";
    /// current gear possible values: neutral, reverse, boost, fwddrive
    gear_fdbk = "neutral";
    /// current indicator possible values: off, left, right, hazard
    indicator_cmd = "off";
    /// current indicator possible values: off, left, right, hazard
    indicator_fdbk = "off";
    /// wiper state applied
    wiper_cmd = 0;
    /// wiper state feedback
    wiper_fdbk = 0;
    /// horn state applied
    horn_cmd = 0;
    /// horn state feedback
    horn_fdbk = 0;
    /// headlight state applied
    headlight_cmd = 0;
    /// headlight state feedback
    headlight_fdbk = 0;
    /// throttle applied in percentage
    throttle_cmd = 0;
    /// throttle read in percentage
    throttle_fdbk = 0;
    /// brake applied
    brake_cmd = 0;
    /// brake read
    brake_fdbk = 0;
    /// velocity target
    velocity_target = 0;
    /// steer target
    steer_target = 0;
    /// steer offset
    steer_offset = 0;
    /// steer limit in degrees
    steer_limit = 40;
    /// auto mode count
    auto_mode_count = 0;
    /// to set the value of steering equation to zero
    t_equation = 0;
  }
  void SteerSetTime()
  {
    /// set current time in us
    steer_header.SetTime();
  }
  void SpeedSetTime()
  {
    /// set current time in us
    speed_header.SetTime();
  }
};

#endif  // CAN_DRIVER_CAN_STRUCT_H
