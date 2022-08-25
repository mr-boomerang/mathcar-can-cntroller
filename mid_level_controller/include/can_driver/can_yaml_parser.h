// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_yaml_parser.h
 * @brief yaml parser for can status and control messages
 * @author Divyanshu Goel
 * @date 2018-02-28 (yyyy-mm-dd)
 */

#ifndef CAN_DRIVER_CAN_YAML_PARSER_H
#define CAN_DRIVER_CAN_YAML_PARSER_H

#include "can_driver/can_struct.h"

#include <linux/can.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <utils/mode_printing.h>

#include <algorithm>
#include <bitset>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief This class will be used to read the file of transmission settings and
 * convert it into a predefined structure that can be accessed as
 * objectname.can_io_message and can message array which can be accessed as
 * objectname.can_message_array_map
 * @author Divyanshu Goel
 * @date 2017-02-28
 */
class CanYamlParser
{
 public:
  /// object of input settings file
  std::ifstream myfile;
  /// contains cuurent no of messages
  int vector_count = -1;
  /// contains all the messages read from file
  std::vector<CanMessage> can_io_message;
  /// contains current canfd frames that will be sent
  std::vector<canfd_frame> can_message_array_map;
  /**
   * @brief trim from start (in place)
   * @author Divyanshu Goel
   * @param &sample - input string
   * @param char_to_erase - character to erase from start
   * @date 2017-02-09
   */
  static inline void ltrim(std::string* sample, char char_to_erase)
  {
    int len = (*sample).size();
    char element = (*sample)[0];
    int element_count = 0;
    while (element == char_to_erase && element_count < len)
    {
      element_count++;
      element = (*sample)[element_count];
    }
    (*sample).erase(0, element_count);
  };

  /**
   * @brief trim from end (in place)
   * @author Divyanshu Goel
   * @param *sample - input string
   * @param char_to_erase - character to erase from end
   * @date 2017-02-09
   */
  static inline void rtrim(std::string* sample, char char_to_erase)
  {
    int len = (*sample).size();
    char element = (*sample)[len];
    int element_count = len;
    if (element == '\0')
    {
      element = (*sample)[len - 1];
      element_count = len - 1;

      while (element == char_to_erase && element_count > 0)
      {
        element_count--;
        element = (*sample)[element_count];
      }
      (*sample).erase(element_count + 1, len - 1);
    }
    else
    {
      while (element == char_to_erase && element_count > 0)
      {
        element_count--;
        element = (*sample)[element_count];
      }
      (*sample).erase(element_count, len);
    }
  };
  /**
   * @brief trim from both ends (in place)
   * @author Divyanshu Goel
   * @param &sample - input string
   * @param char_to_erase - character to erase from both ends
   * @date 2017-02-09
   */
  static inline void trim(std::string* sample, char char_to_erase)
  {
    ltrim(sample, char_to_erase);
    rtrim(sample, char_to_erase);
  };
  /**
   * @brief print integer array from the valid input array
   * @author Divyanshu Goel
   * @param messages_array_temp - input unsigned int array vector for
   * printing
   * @date 2017-02-09
   */
  void print_u_int_vector(std::vector<unsigned int> messages_array_temp);
  /**
   * @brief print integer array from the valid input string array
   * @author Divyanshu Goel
   * @param messages_array_temp - input string array vector for printing
   * @date 2017-02-10
   */
  void print_names_vector(std::vector<std::string> messages_array_temp);
  /**
   * @brief print dictionary map from the valid input dictionary
   * @author Divyanshu Goel
   * @param messages_array_temp - input dictionary for printing
   * @date 2017-02-10
   */
  void print_dictionary_map(DictionaryMap messages_array_temp);
  /**
   * @brief print dictionary array from the valid input dictionary array
   * @author Divyanshu Goel
   * @param messages_array_temp - input dictionary vector for printing
   * @date 2017-02-10
   */
  void print_dictionary_vector(std::vector<DictionaryMap> messages_array_temp);
  /**
   * @brief Get integer array from the valid input string
   * @author Divyanshu Goel
   * @param *sample contains input sample
   * @param *messages_array contains output integer array
   * @date 2017-02-09
   */
  void get_array(std::string* sample,
                 std::vector<unsigned int>* messages_array);
  /**
   * @brief removes special charaters from the given string array by erasin them
   * and returns total count of elements present by checking string against
   * comma. charaters removed are ['"','[',']','{','}']
   * @author Divyanshu Goel
   * @param &sample contains input sample
   * @date 2017-02-10
   */
  int remove_special_chars(std::string* sample);
  /**
   * @brief Get names array from the valid input string
   * @author Divyanshu Goel
   * @param &sample contains input sample
   * @param &messages_array_data contains output topic name
   * @date 2017-02-09
   */
  void get_names_array(std::string* sample,
                       std::vector<std::string>* messages_array_data);
  /**
   * @brief Get single key and value pair from the given string to build up the
   * dictionary
   * @author Divyanshu Goel
   * @param &sample contains input sample
   * @param &key contains input sample
   * @param &value contains output for corresponding key
   * @date 2017-02-10
   */
  void fetch_key_value_pair(std::string* sample, std::string* key,
                            unsigned int* value);
  /**
   * @brief Get dictionary array from the valid input string
   * @author Divyanshu Goel
   * @param &sample contains input sample
   * @param &messages_array contains output in dicitonary key and value format
   * @date 2017-02-09
   */
  void get_dictionary_array(std::string* sample,
                            std::vector<DictionaryMap>* messages_array);

  /**
   * @brief Get dictionary map from the valid input string containing a single
   * dictionary
   * @author Divyanshu Goel
   * @param &sample -> dictionary to created from this string input
   * @date 2017-02-10
   */
  DictionaryMap get_dictionary_map(std::string* sample);
  /**
   * @brief To print all theparameters in the current vector/array of structure.
   * @author Divyanshu Goel
   * @date 2017-02-09
   */
  void param_print_all();
  /**
   * @brief To add parameters to the current vector/array of structure
   * containing all details of the can messages.
   * @author Divyanshu Goel
   * @param &parameter_name -> parameter name
   * @param &parameter_value -> pararmeter value to set
   * @date 2017-02-09
   */
  void add_param(std::string* parameter_name, std::string* parameter_value);
  /**
   * @brief To read parameters from a file and add it to the an array containing
   * all details of the can messages.
   * @author Divyanshu Goel
   * @param filepath - path to file
   * @date 2017-02-09
   */
  void read_file(const std::string filepath);
  /**
   * @brief This method will be used to close file object.
   * @author Divyanshu Goel
   * @date 2017-02-09
   */
  int close_file();
};
#endif  // CAN_DRIVER_CAN_YAML_PARSER_H
