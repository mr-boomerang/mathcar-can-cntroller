// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_receiver.cpp
 * @brief receiver for can. This file can be used to recieve
 * messages over can and sort them out by the given dictionary and update on the
 * publisher
 * @author Divyanshu Goel
 * @date 2017-02-29 (yyyy-mm-dd)
 **/
#include <can_driver/can_receiver.h>

#include <algorithm>
#include <string>
#include <vector>

/// initialize parameters
void CanReceiver::init(const std::string input_yaml_file,
                       const std::string logfilepath,
                       const std::string can_settings_file_path,
                       const int *exit_flag_arg, bool print_help_flag,
                       bool virtual_can_startup)
{
  PRINT_FUNC_ENTER;
  exit_flag_pointer_ = exit_flag_arg;
  if (*exit_flag_pointer_) return;
  // call receiver setup
  receiver_setup(input_yaml_file, logfilepath, can_settings_file_path,
                 print_help_flag, virtual_can_startup);
  // set validity flag
  if (!*exit_flag_arg) init_success = true;
  PRINT_FUNC_EXIT;
}
/// convert fd frame to string
std::string CanReceiver::can_frametostr(canfd_frame message_temp)
{
  PRINT_FUNC_ENTER;
  std::stringstream message_stream;
  std::string temp_message_string;
  message_stream << "[can_id : 0x" << std::hex << message_temp.can_id
                 << std::dec << " ,can_dlc : ";
  message_stream << std::to_string(message_temp.len) << ", can_data : [";
  for (int element_no = 0; element_no < message_temp.len - 1; element_no++)
  {
    message_stream << unsigned(message_temp.data[element_no]) << ", ";
  }
  message_stream << unsigned(message_temp.data[message_temp.len - 1]) << "]";
  temp_message_string = message_stream.str();
  PRINT_FUNC_EXIT;
  return (temp_message_string);
}
/// get index of message in can array
int CanReceiver::get_index(canfd_frame can_message)
{
  PRINT_FUNC_ENTER;
  int len = data_particulars.input_can_message_array_map.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    if (can_message.can_id ==
        data_particulars.input_can_message_array_map[element_no].can_id)
    {
      PRINT_FUNC_EXIT;
      return (element_no);
    }
  }
  PRINT_FUNC_EXIT;
  return (-1);
}
/// get string associated with value
std::string CanReceiver::get_key_dictionary(unsigned int input_value,
                                            DictionaryMap input_map)
{
  PRINT_FUNC_ENTER;
  int index = -1;
  std::string temp;
  int len = input_map.key.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    if (input_map.value[element_no] == input_value)
    {
      PRINT_FUNC_EXIT;
      return (input_map.key[element_no]);
    }
  }
  PRINT_FUNC_EXIT;
  return ("None");
}
/// print_output
void CanReceiver::print_output()
{
  PRINT_FUNC_ENTER;
  printf("--------------------------\n");
  int len = output0.topic.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    printf("%s : ", output0.topic[element_no].c_str());
    printf("%s ", output0.value[element_no].c_str());
    printf("%d\n", output0.msg_count[element_no]);
  }
  printf("--------------------------\n");
  PRINT_FUNC_EXIT;
}
///  insert data in output topic
void CanReceiver::insert_output_topic_data(std::string result,
                                           std::string topic_value)
{
  PRINT_FUNC_ENTER;
  int index = -1;
  std::string temp;
  int len = output0.topic.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    if (output0.topic[element_no].compare(topic_value) == 0)
    {
      output0.value[element_no] = result;
      output0.msg_count[element_no] = output0.msg_count[element_no] + 1;
      index = element_no;
    }
  }
  PRINT_FUNC_EXIT;
}
/// msg parser for can
void CanReceiver::msg_parse(canfd_frame can_message[], int incoming_data_len)
{
  PRINT_FUNC_ENTER;
  int len = incoming_data_len;
  // start loop for each data element recieved
  for (int element_no = 0; element_no < len; element_no++)
  {
    // copy the element to a temporary frame
    canfd_frame temp = can_message[element_no];
    uint64_t message_content;
    // convert data to byte string
    bus_receiver.array_to_byte(&message_content, temp.data);
    // search frame in current output array and get its location
    int location = get_index(temp);
    // if a valid location is found means message exists
    if (location != -1)
    {
      int messages_count = 0;
      // data map associated with the given message that is what the message
      // means
      CanTopicsDataMap parsing_for =
          data_particulars.input_can_topics_data_map[location];
      // search in the data map and update the feedback value accordingly
      if (parsing_for.can_message_count_map.size() != 0)
      {
        messages_count = parsing_for.can_message_count_map[0];

        for (int message_no = 0; message_no < messages_count; message_no++)
        {
          std::string topic_name_temp = parsing_for.can_topics_map[message_no];
          int message_shift_bit = parsing_for.can_start_bit_map[message_no];
          int message_validity_bit =
              parsing_for.can_validity_bits_map[message_no];
          int message_data_length =
              parsing_for.can_message_length_map[message_no];
          int message_temp_content = message_content >> message_shift_bit;
          DictionaryMap temp_dictionary =
              parsing_for.can_dictionary_map[message_no];
          uint64_t mask = pow(2, message_data_length);
          mask = mask - 1;
          message_temp_content = message_temp_content & mask;
          std::string result;
          // check for integer input values otherwise update the value as per
          // dictionary
          if (temp_dictionary.key[0].compare("max_value") == 0)
          {
            result = std::to_string(message_temp_content);
            if (message_temp_content > temp_dictionary.value[0])
              printf("Error!!! Value Exceeded for %s . Current value %s \n",
                     parsing_for.can_topics_map[message_no].c_str(),
                     result.c_str());
          }
          else
          {
            // now find in dictionary associated the
            result = get_key_dictionary(message_temp_content, temp_dictionary);
          }
          // insert data in the ouput map
          insert_output_topic_data(result, topic_name_temp);

          // get message_pertaining to the number
          // check validity_bit
          if (debug_enable)
          {
            printf("Message Recieved:");
            std::cout << std::bitset<8>(message_temp_content) << std::endl;
            status.print_dictionary_map(temp_dictionary);
            printf("\nTopic : %s", topic_name_temp.c_str());
            printf("\nResult : %s\n", result.c_str());
          }
        }
        if (debug_enable)
        {
          print_structure_stored(parsing_for);
        }
      }

      // copy to location
      data_particulars.input_can_message_array_map[location] = temp;
      //  output0.raw_data[location] = can_frametostr(temp);
      if (messages_count > 0)
      {
        // copy relevant info
        for (int topic_name_index = 0; topic_name_index < messages_count;
             topic_name_index++)
        {
          int start_bit = data_particulars.input_can_topics_data_map[location]
                              .can_start_bit_map[topic_name_index];
          int validity_bit =
              data_particulars.input_can_topics_data_map[location]
                  .can_validity_bits_map[topic_name_index];
          int message_length =
              data_particulars.input_can_topics_data_map[location]
                  .can_message_length_map[topic_name_index];
          int total_length = temp.len;
          std::string topic_name =
              data_particulars.input_can_topics_data_map[location]
                  .can_topics_map[topic_name_index];
          DictionaryMap current_dictionary =
              data_particulars.input_can_topics_data_map[location]
                  .can_dictionary_map[topic_name_index];
          if (message_length <= 64)
          {
            uint64_t message_temp, message_copy;
            bus_receiver.array_to_byte(&message_temp, temp.data);
            message_copy = message_temp >> start_bit;
            message_copy = message_temp & unsigned(pow(2, message_length) - 1);
            // output0.value[location] =
            // get_key_dictionary(message_copy, current_dictionary);
          }
        }
      }
    }
    else
    {
      // for unkwown message create a empty value in each of the vector
      unknown_count++;
      newunknownmessage << "Unknown_Message_" << unknown_count;
      status_map.can_message_array_map.push_back(temp);
      data_particulars.input_can_message_array_map.push_back(temp);
      empty_topics_data_map.can_dictionary_map.push_back(empty_dictionary_map);
      empty_topics_data_map.can_topics_map.push_back(newunknownmessage.str());
      empty_topics_data_map.can_total_length_map.push_back(temp.len);
      empty_topics_data_map.can_start_bit_map.push_back(0);
      empty_topics_data_map.can_message_length_map.push_back(0);
      empty_topics_data_map.can_validity_bits_map.push_back(0);
      empty_topics_data_map.can_message_count_map.push_back(0);
      empty_topics_data_map.can_arbitration_id_map.push_back(temp.can_id);
      data_particulars.input_can_topics_data_map.push_back(
          empty_topics_data_map);
      empty_topics_data_map.can_dictionary_map.clear();
      empty_topics_data_map.can_topics_map.clear();
      empty_topics_data_map.can_total_length_map.clear();
      empty_topics_data_map.can_start_bit_map.clear();
      empty_topics_data_map.can_message_length_map.clear();
      empty_topics_data_map.can_validity_bits_map.clear();
      empty_topics_data_map.can_message_count_map.clear();
      empty_topics_data_map.can_arbitration_id_map.clear();
      output0.topic.push_back(newunknownmessage.str());
      output0.value.push_back(newunknownmessage.str());
      // output0.raw_data.push_back(can_frametostr(temp));
      output0.msg_count.push_back(1);
      printf("New Message Encountered : 0x%03x \n", temp.can_id);
      newunknownmessage.str(std::string());
      if (debug_enable)
      {
        // print sorted data
      }
    }
  }
#ifndef BUILD_WITHOUT_ROS
  output1.topic = output0.topic;
  output1.value = output0.value;
  // output1.raw_data = output0.raw_data;
// output1.msg_count = output0.msg_count;
#endif
  PRINT_FUNC_EXIT;
}
/// to print can message array
void CanReceiver::print_can_message_vector(
    std::vector<canfd_frame> can_message_array)
{
  PRINT_FUNC_ENTER;
  int len = can_message_array.size();
  if (len > 0)
  {
    printf("Message ids : [");
    for (int i = 0; i < len; ++i)
    {
      printf("0x%3x", can_message_array[i].can_id);
      if (i < len - 1)
      {
        printf(", ");
      }
    }
    printf("]\n");
    printf("Message dlc : [");
    for (int i = 0; i < len; ++i)
    {
      printf("%u ", can_message_array[i].len);
      if (i < len - 1)
      {
        printf(", ");
      }
    }
    printf("]\n");
    printf("Message data : [");
    for (int i = 0; i < len; ++i)
    {
      for (int j = 0; j < can_message_array[i].len; ++j)
      {
        printf("%u", can_message_array[i].data[j]);

        if (j < can_message_array[i].len - 1)
        {
          printf(", ");
        }
      }
      if (i < len - 1)
      {
        printf("], [");
      }
      printf(" ");
    }
    printf("]\n");
  }
  PRINT_FUNC_EXIT;
}
// This method will be used to print all the variables and data in the map
void CanReceiver::print_structure_stored(CanTopicsDataMap input_map)
{
  PRINT_FUNC_ENTER;
  int len = input_map.can_topics_map.size();
  printf("Message Names : ");
  status.print_names_vector(input_map.can_topics_map);
  printf("\nTotal Length array : ");
  status.print_u_int_vector(input_map.can_total_length_map);
  printf("Start bit array : ");
  status.print_u_int_vector(input_map.can_start_bit_map);
  printf("Message Length array : ");
  status.print_u_int_vector(input_map.can_message_length_map);
  printf("Validity Bits array : ");
  status.print_u_int_vector(input_map.can_validity_bits_map);
  printf("Message Count array : ");
  status.print_u_int_vector(input_map.can_message_count_map);
  printf("Arbitration id array : ");
  status.print_u_int_vector(input_map.can_arbitration_id_map);
  print_can_message_vector(input_map.can_message_array_map);
  PRINT_FUNC_EXIT;
}
// This method will be used to print all the variables and data in the
// processed map
void CanReceiver::print_processed_structure_stored(
    InputParticularsMap input_map)
{
  PRINT_FUNC_ENTER;
  printf("********************\n");
  printf("Total can message arrays : \n");
  print_can_message_vector(input_map.input_can_message_array_map);
  printf("Total Stored processed data : \n");
  int len = input_map.input_can_topics_data_map.size();
  for (int i = 0; i < len; ++i)
  {
    printf("--\n");
    print_structure_stored(input_map.input_can_topics_data_map[i]);
  }
  printf("********************\n");
  PRINT_FUNC_EXIT;
}
// execute a system command
int CanReceiver::exec_command(std::string command)
{
  char buf[BUFSIZE];
  FILE *fp;

  if ((fp = popen(command.c_str(), "r")) == NULL)
  {
    printf("Error opening pipe!\n");
    PRINT_FUNC_EXIT;
    return -1;
  }
  if (print_help)
  {
    while (fgets(buf, BUFSIZE, fp) != NULL)
    {
      printf("OUTPUT: %s", buf);
    }
  }
  if (pclose(fp))
  {
    printf("Command not found or exited with error status\n");
    PRINT_FUNC_EXIT;
    return -1;
  }
  PRINT_FUNC_EXIT;
  return 0;
}
// This method will be used to setup all the variables and data for the
// program to work.

void CanReceiver::receiver_setup(const std::string input_yaml_file,
                                 const std::string logfilepath,
                                 const std::string can_settings_file_path,
                                 bool print_help_flag, bool virtual_can_startup)
{
  PRINT_FUNC_ENTER;
  // read the input file which contains info of dictionary

  status.read_file(input_yaml_file);
  status.close_file();
  if (!params_loaded)
  {
    // load parameters using utils
    params_utils.load(can_settings_file_path);
  }
  // store values in local variables
  can_interface = params_utils.get_val<std::string>("can_channel", "can0");
  raw_msg_enable = params_utils.get_val<bool>("raw_msg_rx_enable", false);
  log_enable = params_utils.get_val<bool>("log_enable", true);
  log_data_enable = params_utils.get_val<bool>("log_data_rx_enable", true);
  debug_enable = params_utils.get_val<bool>("debug_msg_rx_enable", false);
  print_help = print_help_flag;
  if (virtual_can_startup) can_interface = "vcan0";
  // create empty dictionary
  empty_dictionary_map.key.push_back("None");
  empty_dictionary_map.value.push_back(0);
  // adjusting the incoming frame array size accordingly
  if (can_interface.compare("vcan0") == 0)
  {
    bus_receiver.max_no_frames = 6;
  }
  else
  {
    bus_receiver.max_no_frames = 14;
  }
  int array_len = status.can_io_message.size();
  if (print_help) printf("Outputs for Status :\n");
  for (int i = 0; i < array_len; ++i)
  {
    /// create an empty frame with given arbitration id and add it to
    /// structure
    canfd_frame temp_can_frame;
    temp_can_frame.can_id = status.can_io_message[i].message_id;
    temp_can_frame.len = status.can_io_message[i].total_length / 8;
    if (temp_can_frame.len <= 8)
    {
      __u8 data[] = {0, 0, 0, 0, 0, 0, 0, 0};
      memcpy(temp_can_frame.data, data, 8);
      status_map.can_message_array_map.push_back(temp_can_frame);
    }
    else
    {
      canfd_frame temp_can_frame_fd;
      temp_can_frame_fd.can_id = status.can_io_message[i].message_id;
      temp_can_frame_fd.len = status.can_io_message[i].total_length / 8;

      __u8 data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      memcpy(temp_can_frame_fd.data, data,
             15 * sizeof(*data));  // temp_can_frame.can_dlc);
      status_map.can_message_array_map.push_back(temp_can_frame_fd);
    }
    status_map.can_message_count_map.push_back(
        status.can_io_message[i].message_count);
    int second_loop_limit = status.can_io_message[i].message_count;
    for (int j = 0; j < second_loop_limit; ++j)
    {
      /// get the parameters from the file in following order: start_bit,
      /// message_length, validity_bit, message_names, dictionary
      /// associated, arbitration id
      status_map.can_total_length_map.push_back(
          status.can_io_message[i].total_length);
      status_map.can_start_bit_map.push_back(
          status.can_io_message[i].start_bit[j]);

      status_map.can_message_length_map.push_back(
          status.can_io_message[i].message_length[j]);

      status_map.can_validity_bits_map.push_back(
          status.can_io_message[i].validity_bit[j]);
      std::transform(status.can_io_message[i].names[j].begin(),
                     status.can_io_message[i].names[j].end(),
                     status.can_io_message[i].names[j].begin(), ::tolower);
      status_map.can_topics_map.push_back(status.can_io_message[i].names[j]);

      status_map.can_dictionary_map.push_back(
          status.can_io_message[i].dictionary[j]);

      status_map.can_arbitration_id_map.push_back(
          status.can_io_message[i].message_id);

      if (print_help)
      {
        printf("   %s\n", status.can_io_message[i].message_name.c_str());
        /// Print input guide for the inputs
        printf("\ttopic_name : %s\n",
               status.can_io_message[i].names[j].c_str());
      }
      std::string key_match_copy =
          status.can_io_message[i].dictionary[j].key[0];
      /// make mathcxes case insensitive
      std::transform(key_match_copy.begin(), key_match_copy.end(),
                     key_match_copy.begin(), ::tolower);
      /// check if its a integer input
      if (print_help)
      {
        if (key_match_copy.compare("max_value") == 0)
        {
          std::cout << "\tInteger Output : ["
                    << status.can_io_message[i].dictionary[j].key[0] << "--> "
                    << status.can_io_message[i].dictionary[j].value[0] << " ]"
                    << std::endl;
        }
        else
        {
          /// otherwise print options
          printf("\tOutputs : [");
          int key_limits = status.can_io_message[i].dictionary[j].key.size();
          for (int each_key = 0; each_key < key_limits; ++each_key)
          {
            std::cout << "\'"
                      << status.can_io_message[i].dictionary[j].key[each_key];
            if (each_key < key_limits - 1) std::cout << "\', ";
          }
          /// default option to set validity bit to zero
          printf("']\n");
        }
      }
    }
  }
  output0.topic = status_map.can_topics_map;
#ifndef BUILD_WITHOUT_ROS
  output1.topic = output0.topic;
  output1.value = output0.value;
  // output1.raw_data = output0.raw_data;
// output1.msg_count = output0.msg_count;
#endif
  // standard outputs expected
  array_len = status_map.can_validity_bits_map.size();
  unsigned int search_arbit_id;
  search_arbit_id = status_map.can_arbitration_id_map[0];
  CanTopicsDataMap temp_map, empty_topics_map;
  data_particulars.input_can_message_array_map =
      status_map.can_message_array_map;
  int count_for_message_nos = 0;
  for (int i = 0; i < array_len; ++i)
  {
    output0.value.push_back(std::string());
    output0.raw_data.push_back(std::string());
    output0.msg_count.push_back(0);
    if (search_arbit_id == status_map.can_arbitration_id_map[i])
    {
      temp_map.can_dictionary_map.push_back(status_map.can_dictionary_map[i]);
      temp_map.can_topics_map.push_back(status_map.can_topics_map[i]);
      temp_map.can_total_length_map.push_back(
          status_map.can_total_length_map[i]);
      temp_map.can_start_bit_map.push_back(status_map.can_start_bit_map[i]);
      temp_map.can_message_length_map.push_back(
          status_map.can_message_length_map[i]);
      temp_map.can_validity_bits_map.push_back(
          status_map.can_validity_bits_map[i]);
      temp_map.can_message_count_map.push_back(
          status_map.can_message_count_map[count_for_message_nos]);
      temp_map.can_arbitration_id_map.push_back(
          status_map.can_arbitration_id_map[i]);
    }
    else
    {
      data_particulars.input_can_topics_data_map.push_back(temp_map);
      temp_map = empty_topics_map;
      count_for_message_nos++;
      temp_map.can_dictionary_map.push_back(status_map.can_dictionary_map[i]);
      temp_map.can_topics_map.push_back(status_map.can_topics_map[i]);
      temp_map.can_total_length_map.push_back(
          status_map.can_total_length_map[i]);
      temp_map.can_start_bit_map.push_back(status_map.can_start_bit_map[i]);
      temp_map.can_message_length_map.push_back(
          status_map.can_message_length_map[i]);
      temp_map.can_validity_bits_map.push_back(
          status_map.can_validity_bits_map[i]);
      temp_map.can_message_count_map.push_back(
          status_map.can_message_count_map[count_for_message_nos]);
      temp_map.can_arbitration_id_map.push_back(
          status_map.can_arbitration_id_map[i]);
      search_arbit_id = status_map.can_arbitration_id_map[i];
    }
  }
  if (debug_enable)
  {
    printf("Data read from Input file:\n");
    status.param_print_all();
    printf("Data read from can settings file:\n");
    std::string lst = params_utils.list_full("\n");
    // params with vales and properties.
    std::cout << lst << std::endl;
    printf("Data stored from file:\n");
    print_structure_stored(status_map);
    printf("Processed Data stored from file:\n");
    print_processed_structure_stored(data_particulars);
  }
  /// get time stamp
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  std::time_t time_now0 = std::chrono::system_clock::to_time_t(start);
  struct tm *now = std::localtime(&time_now0);
  /// open can port
  std::stringstream file_name_rx_stream;
  file_name_rx_stream << logfilepath << (now->tm_year + 1900) << '_'
                      << (now->tm_mon + 1) << '_' << now->tm_mday << "_"
                      << (now->tm_hour) << "_" << (now->tm_min) << "_"
                      << (now->tm_sec) << "_CAN_rx.log";
  std::string file_name_rx_string = file_name_rx_stream.str();

  int open_port_exception = 1;

  while (open_port_exception && !*exit_flag_pointer_)
  {
    try
    {
      bus_receiver.open_port(can_interface.c_str(), file_name_rx_string.c_str(),
                             log_enable, exit_flag_pointer_);
      open_port_exception = 0;
    }
    catch (std::exception &e)
    {
      open_port_exception = 1;
      printf("\n");
      usleep(2000000);
    }
  }

  /// file to save parsed information
  std::stringstream file_name_rx_data_stream;
  file_name_rx_data_stream << logfilepath << (now->tm_year + 1900) << '_'
                           << (now->tm_mon + 1) << '_' << now->tm_mday << "_"
                           << (now->tm_hour) << "_" << (now->tm_min) << "_"
                           << (now->tm_sec) << "_data_rx.log";
  std::string file_name_rx_data_string = file_name_rx_data_stream.str();
  //
  printf("*********************************************\n");
  printf(" Receiver data : \n");
  if (log_enable)
    printf(" Writing CAN Log file to : %s\n", file_name_rx_string.c_str());
  if (log_data_enable) open_log_file(file_name_rx_data_string.c_str());
  printf(" Listening to can bus at: %s\n", can_interface.c_str());
  printf("*********************************************\n");
}
// open file handle for data
void CanReceiver::open_log_file(const std::string logfilepath)
{
  PRINT_FUNC_ENTER;
  printf(" Writing DATA Log file to : %s\n", logfilepath.c_str());
  data_log_file.open_file(logfilepath);
  std::stringstream data, output_topic;
  int len = output0.topic.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    output_topic << output0.topic[element_no].c_str();
    if (element_no < len - 1) output_topic << ", ";
  }
  std::string output_topic_str = output_topic.str();
  data << "Timestamp, " << output_topic_str << std::endl;
  std::string data_str = data.str();
  data_log_file.print_string_message_to_file(data_str.c_str());
  PRINT_FUNC_EXIT;
}
// write data to file from the reciever after parsing
void CanReceiver::write_to_data_log_file(OutputMsg output_temp)
{
  PRINT_FUNC_ENTER;
  std::stringstream data, output_data;

  int len = output0.topic.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    output_data << output0.value[element_no].c_str();
    if (element_no < len - 1) output_data << ", ";
  }
  std::string output_data_str = output_data.str();
  double timestamp_log = get_timestamp() / 1000000L;
  data << std::to_string(timestamp_log) << ", " << output_data_str << std::endl;
  std::string data_str = data.str();
  data_log_file.print_string_message_to_file(data_str.c_str());
  PRINT_FUNC_EXIT;
}
// clean exit of file writing
void CanReceiver::close_data_log_file()
{
  PRINT_FUNC_ENTER;
  data_log_file.flush();
  data_log_file.close_file();
  PRINT_FUNC_EXIT;
}
// listener from the reciever
void CanReceiver::start_can_listener_thd()
{
  PRINT_FUNC_ENTER;
  // to allow for waiting for init to be done properly
  usleep(10000);
  canBusParserIsThreaded = true;
  can_bus_parser0 = std::thread(&CanReceiver::start_can_listener, this);
  PN$ PRINT_SUCCESS << ct_grn("Receiver Working") pendl;
  PRINT_FUNC_EXIT;
}
void CanReceiver::start_can_listener()
{
  PRINT_FUNC_ENTER;
  if (!init_success)
  {
    PE$ ct_red(
        "CAN Receiver. Error in intilization. Please Ensure proper "
        "initialization") pendl;
    PRINT_FUNC_EXIT;
    return;
  }
  /// listens to bus blocking call so started as a thread
  canBusListenerIsThreaded = true;
  can_bus_listener = std::thread(&CanBus::read_port, &bus_receiver);
  std::string temp_raw_stream_string;
  while (!exit_flag && !*exit_flag_pointer_)
  {
    auto time_start = std::chrono::high_resolution_clock::now();
    if (bus_receiver.new_incoming_data)
    {
      int incoming_data_len = bus_receiver.max_no_frames;
      if (incoming_data_len > 0)
      {
        msg_parse(bus_receiver.frame_temp, incoming_data_len);
        // at this point output is present in output1, output0 variable of
        // the class;
        if (log_data_enable)
        {
          write_to_data_log_file(output0);
        }
      }
      if (raw_msg_enable)
      {
        for (int temp_var = 0; temp_var < incoming_data_len; temp_var++)
        {
          temp_raw_stream_string =
              can_frametostr(bus_receiver.frame_temp[temp_var]);
          double timestamp_now = get_timestamp() / 1000000L;
          printf("[%f] RX %s\n", timestamp_now, temp_raw_stream_string.c_str());
        }
        printf("\n");
      }
      if (debug_enable)
      {
        print_output();
      }

      bus_receiver.new_incoming_data = 0;
      new_incoming_data = 1;
      exit_flag = *exit_flag_pointer_;
      auto time_end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::micro> elapsed_time =
          time_end - time_start;
      usleep(10000 - static_cast<int>(elapsed_time.count()));
    }
  }
  //  signalHandler(1);
  PRINT_FUNC_EXIT;
}
CanReceiver::~CanReceiver()
{
  // join the threads
  if (can_bus_parser0.joinable()) can_bus_parser0.join();
  if (can_bus_listener.joinable()) can_bus_listener.join();
}
// exit from the reciever
void CanReceiver::receiver_exit(int signum)
{
  close_data_log_file();
  if (init_success) bus_receiver.close_port();
  exit_flag = 1;
  PRINT_FUNC_EXIT;
}
/// cleanly exit from transmitter
void CanReceiver::signalHandler(int signum)
{
  PI$ "Closing receiver instance." pendl;
  /// clean exit
  receiver_exit(signum);
}
