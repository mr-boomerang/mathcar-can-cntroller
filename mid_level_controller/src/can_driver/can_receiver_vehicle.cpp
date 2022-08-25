// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_receiver_vehicle.cpp
 * @brief receiver for can. This file can be used to recieve
 * messages over can and sort them out by the given dictionary and update on the
 * publisher
 * @author Divyanshu Goel
 * @date 2017-02-29 (yyyy-mm-dd)
 **/

#include <can_driver/can_receiver_vehicle.h>

#include <algorithm>
#include <string>
#include <vector>
/// global instance of reciever
CanReceiverVehicle receiver0;
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received.\n", signum);
  // cleanup and close up stuff here
  // terminate program
  receiver0.signalHandler(signum);
};
/// convert fd frame to string
std::string CanReceiverVehicle::can_frametostr(canfd_frame message_temp)
{
  std::stringstream message_stream;
  std::string temp_message_string;
  message_stream << "[can_id : 0x" << std::hex << message_temp.can_id
                 << std::dec << " ,can_dlc : " << message_temp.len
                 << ", can_data : [";
  for (int element_no = 0; element_no < message_temp.len - 1; element_no++)
  {
    message_stream << unsigned(message_temp.data[element_no]) << ", ";
  }
  message_stream << unsigned(message_temp.data[message_temp.len - 1]) << "]";
  temp_message_string = message_stream.str();
  return (temp_message_string);
}
/// get index of message in can array
int CanReceiverVehicle::get_index(canfd_frame can_message)
{
  int index = -1;
  int len = data_particulars.input_can_message_array_map.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    if (can_message.can_id ==
        data_particulars.input_can_message_array_map[element_no].can_id)
    {
      return (element_no);
    }
  }
  return (-1);
}
/// get string associated with value in the dictionary
std::string CanReceiverVehicle::get_key_dictionary(unsigned int input_value,
                                                   dictionary_map input_map)
{
  int index = -1;
  std::string temp;
  int len = input_map.key.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    if (input_map.value[element_no] == input_value)
    {
      return (input_map.key[element_no]);
    }
  }
  return ("None");
}
/// print_output
void CanReceiverVehicle::print_output()
{
  printf("--------------------------\n");
  int len = output0.topic.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    printf("%s : ", output0.topic[element_no].c_str());
    printf("%s ", output0.value[element_no].c_str());
    printf("%d\n", output0.msg_count[element_no]);
  }
  printf("--------------------------\n");
}
/// insert data in output topic
void CanReceiverVehicle::insert_output_topic_data(std::string result,
                                                  std::string topic_value)
{
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
}
/// msg parser for can
void CanReceiverVehicle::msg_parse(canfd_frame can_message[],
                                   int incoming_data_len)
{
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
      can_topics_data_map parsing_for =
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
          dictionary_map temp_dictionary =
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
      output0.raw_data[location] = can_frametostr(temp);
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
          dictionary_map current_dictionary =
              data_particulars.input_can_topics_data_map[location]
                  .can_dictionary_map[topic_name_index];
          if (message_length <= 64)
          {
            uint64_t message_temp, message_copy;
            bus_receiver.array_to_byte(&message_temp, temp.data);
            message_copy = message_temp >> start_bit;
            message_copy = message_temp & unsigned(pow(2, message_length) - 1);
            output0.value[location] =
                get_key_dictionary(message_copy, current_dictionary);
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
      output0.raw_data.push_back(can_frametostr(temp));
      output0.msg_count.push_back(1);
      printf("New Message Encountered : 0x%03x \n", temp.can_id);
      newunknownmessage.str(std::string());
      if (debug_enable)
      {
        // print sorted data
      }
    }
  }

  output1.topic = output0.topic;
  output1.value = output0.value;
  output1.raw_data = output0.raw_data;
  // output1.msg_count = output0.msg_count;
}
/// to print can message array in a set format
void CanReceiverVehicle::print_can_message_vector(
    std::vector<canfd_frame> can_message_array)
{
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
}

// This method will be used to print all the variables and data in the map
void CanReceiverVehicle::print_structure_stored(can_topics_data_map input_map)
{
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
}
// This method will be used to print all the variables and data in the
// processed map
void CanReceiverVehicle::print_processed_structure_stored(
    input_particulars_map input_map)
{
  printf("********************\n");
  printf("Total can message arrays : \n");
  print_can_message_vector(input_map.input_can_message_array_map);
  printf("Total Stored processed data : \n");
  int len = input_map.input_can_topics_data_map.size();
  for (int i = 0; i < len; ++i)
  {
    print_structure_stored(input_map.input_can_topics_data_map[i]);
    if (i < len - 1) printf("--\n");
  }
  printf("********************\n");
}
// This method will be used to setup all the variables and data for the
// program to work.

void CanReceiverVehicle::receiver_setup(
    const std::string input_yaml_file, const std::string logfilepath,
    const std::string can_settings_file_path)
{
  // read the input file which contains info of dictionary
  status.read_file(input_yaml_file);
  status.close_file();
  // load parameters using utils
  params_utils.load(can_settings_file_path);
  // store values in local variables
  can_interface = params_utils.get_val<std::string>("can_channel_2", "can1");
  raw_msg_enable = params_utils.get_val("raw_msg_enable", false);
  log_enable = params_utils.get_val("log_enable", false);
  debug_enable = params_utils.get_val("debug_msg_enable", false);
  // other parameters are not needed so they are not loaded. Print the list
  printf("Data read from can settings file:\n");
  std::string lst = params_utils.list_full("\n");
  // params with vales and properties.
  printf("%s\n", lst.c_str());
  printf("*********************************************\n");
  empty_dictionary_map.key.push_back("None");
  empty_dictionary_map.value.push_back(0);
  // assumption that user is added to group can_group and allowed to use sudo
  sudo_password = params_utils.get_val<std::string>("sudo_password", "");
  if (sudo_password == "")
  {
    PN$ "No sudo password present. Assuming command requires no password "
        "(added to sudoers list) and proceeding ahead." pendl;
    root_password_present = false;
  }
  if (can_interface.compare("vcan0") == 0)
  {
    bus_receiver.max_no_frames = 16;
  }
  else
  {
    bus_receiver.max_no_frames = 18;
  }

  int interface_enable;
  if (can_interface == "vcan0" && root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep vcan0 | grep \"qlen 1\"";
    interface_enable = system(command.c_str());
    // enable the interface and open the connection
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      std::string command =
          "echo " + sudo_password + " | sudo -kS modprobe vcan";
      interface_enable = system(command.c_str());
      command =
          "echo " + sudo_password + " | sudo -kS ip link add vcan0 type vcan";
      interface_enable = system(command.c_str());
      command = "echo " + sudo_password + " | sudo -kS ip link set vcan0 up";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }
  // if root password doesn't exists for vcan
  if (can_interface == "vcan0" && !root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep vcan0 | grep \"qlen 1\"";
    interface_enable = system(command.c_str());
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      command = "sudo modprobe vcan";
      interface_enable = system(command.c_str());
      command = "sudo ip link add vcan0 type vcan";
      interface_enable = system(command.c_str());
      command = "sudo ip link set vcan0 up";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }
  // if root password exists for can
  if (can_interface == "can0" && root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep can0 | grep \"qlen 10\"";
    interface_enable = system(command.c_str());
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      command = "echo " + sudo_password +
                " | sudo -kS ip link set can0 up type can bitrate 500000";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }
  // if root password doesn't exists for vcan
  if (can_interface == "can0" && !root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep can0 | grep \"qlen 10\"";
    interface_enable = system(command.c_str());
    // std::cout << interface_enable << std::endl;
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      command = "sudo ip link set can0 up type can bitrate 500000";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }

  // if root password exists for can
  if (can_interface == "can1" && root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep can1 | grep \"qlen 10\"";
    interface_enable = system(command.c_str());
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      command = "echo " + sudo_password +
                " | sudo -kS ip link set can1 up type can bitrate 500000";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }
  // if root password doesn't exists for vcan
  if (can_interface == "can1" && !root_password_present)
  {
    // check if module exists
    std::string command = "ip link | grep can1 | grep \"qlen 10\"";
    interface_enable = system(command.c_str());
    // std::cout << interface_enable << std::endl;
    if (!interface_enable)
    {
      printf("Enabling interface\n");
      command = "sudo ip link set can1 up type can bitrate 500000";
      interface_enable = system(command.c_str());
    }
    else
    {
      PE$ ct_red(
          "Could not start interface as it doesn't exist in ip link list. "
          "Please go through the installation docs on how to setup the "
          "drivers") pendl;
    }
  }
  int array_len = status.can_io_message.size();
  // standard outputs expected
  printf("Outputs for Status : \n");
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

      printf("   %s\n", status.can_io_message[i].message_name.c_str());
      /// Print input guide for the inputs
      printf("\ttopic_name : %s\n", status.can_io_message[i].names[j].c_str());
      std::string key_match_copy =
          status.can_io_message[i].dictionary[j].key[0];
      /// make matches case insensitive
      std::transform(key_match_copy.begin(), key_match_copy.end(),
                     key_match_copy.begin(), ::tolower);
      /// check if its a integer input
      if (key_match_copy.compare("max_value") == 0)
      {
        std::cout << "\tInteger Input : ["
                  << status.can_io_message[i].dictionary[j].key[0] << "--> "
                  << status.can_io_message[i].dictionary[j].value[0]
                  << ", 'invalid']" << std::endl;
      }
      else
      {
        /// otherwise print options
        printf("\tInputs : [");
        int key_limits = status.can_io_message[i].dictionary[j].key.size();
        for (int each_key = 0; each_key < key_limits; ++each_key)
        {
          std::cout << "\'"
                    << status.can_io_message[i].dictionary[j].key[each_key]
                    << "\', ";
        }
        /// default option to set validity bit to zero
        printf("\'invalid\']\n");
      }
    }
  }
  // initialise output variables
  output0.topic = status_map.can_topics_map;
  output1.topic = output0.topic;
  output1.value = output0.value;
  output1.raw_data = output0.raw_data;
  // output1.msg_count = output0.msg_count;
  array_len = status_map.can_validity_bits_map.size();
  unsigned int search_arbit_id;
  search_arbit_id = status_map.can_arbitration_id_map[0];
  can_topics_data_map temp_map, empty_topics_map;
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
    can_settings.param_print_all();
    printf("Data stored from file:\n");
    print_structure_stored(status_map);
    printf("Processed Data stored from file:\n");
    print_processed_structure_stored(data_particulars);
  }
  /// get time stamp
  time_t time_now = time(0);
  struct tm *now = localtime_r(&time_now);
  /// open can port
  std::stringstream file_name_rx_stream;
  file_name_rx_stream << logfilepath << (now->tm_year + 1900) << '_'
                      << (now->tm_mon + 1) << '_' << now->tm_mday << "_"
                      << (now->tm_hour) << "_" << (now->tm_min) << "_"
                      << (now->tm_sec) << "_rx_vehicle_can.log";
  std::string file_name_rx_string = file_name_rx_stream.str();
  printf("*********************************************\n");
  // write to log file
  if (log_enable)
    printf(" Wrtiting Log file to : %s\n", file_name_rx_string.c_str());
  printf(" Listening to can bus at: %s\n", can_interface.c_str());
  printf("*********************************************\n");
  bus_receiver.open_port(can_interface.c_str(), file_name_rx_string.c_str(),
                         log_enable);
}
// listener from the reciever
void CanReceiverVehicle::can_listener()
{
  /// listens to bus
  bus_receiver.read_port();
}
// exit from the reciever
void CanReceiverVehicle::receiver_exit()
{
  PI$ "Closing receiver instance." pendl;
  int interface_enable;
  // if root password doesn't exists for can
  if (can_interface == "can1" && !root_password_presente)
  {
    // check if module exists
    std::string command = "sudo ip link set can1 down";
    interface_enable = system(command.c_str());
  }
  if (can_interface == "can1" && root_password_present)
  {
    std::string command;
    printf("Disabling interface\n");
    command = "echo " + sudo_password + " | sudo -kS ip link set can1 down";
    interface_enable = system(command.c_str());
    bus_receiver.close_port();
  }
  // if root password doesn't exists for can
  if (can_interface == "can0" && !root_password_present)
  {
    // check if module exists
    std::string command = "sudo ip link set can0 down";
    interface_enable = system(command.c_str());
  }
  if (can_interface == "can0" && root_password_present)
  {
    std::string command;
    printf("Disabling interface\n");
    command = "echo " + sudo_password + " | sudo -kS ip link set can0 down";
    interface_enable = system(command.c_str());
    bus_receiver.close_port();
  }
  // if root password doesn't exists for can
  if (can_interface == "vcan0" && !root_password_present)
  {
    // check if module exists
    std::string command = "sudo ip link set vcan0 down";
    interface_enable = system(command.c_str());
  }
  if (can_interface == "vcan0" && root_password_present)
  {
    std::string command;
    printf("Disabling interface\n");
    command = "echo " + sudo_password + " | sudo -kS ip link set vcan0 down";
    interface_enable = system(command.c_str());
    bus_receiver.close_port();
  }
}
/// cleanly exit from transmitter
void CanReceiverVehicle::signalHandler(int signum)
{
  /// clean exit
  receiver_exit();
}
/// main module
int main(int argc, char **argv)
{
  receiver0.yaml_file_path = argv[1];
  receiver0.log_file_path = argv[2];
  receiver0.can_settings_file_path = argv[3];
  /// initiate ros node
  ros::init(argc, argv, "CanReceiverVehicle");
  /// initiate ros node handle
  ros::NodeHandle nh;
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  /// create instance of a subscriber to take specific command based input
  ros::Publisher chatter_pub =
      nh.advertise<mid_level_controller::Can_Status_Message>(
          "/canVehicleStatus", 1000);
  /// create a timer that writes to can bus periodically
  // bus_writer_Callback);
  /// since we are going to publish data at 100 Hz to the can
  ros::Rate loop_rate(10000);
  receiver0.receiver_setup(receiver0.yaml_file_path, receiver0.log_file_path,
                           receiver0.can_settings_file_path);
  int count = 0;
  std::string temp_raw_stream_string;
  std::thread t1(&CanReceiverVehicle::can_listener, &receiver0);
  while (ros::ok())
  {
    // receiver0.incoming_messages.push_back(message_temp);
    int incoming_data_len = receiver0.bus_receiver.frame_count;
    if (incoming_data_len > 0)
    {
      receiver0.msg_parse(receiver0.bus_receiver.frame_temp, incoming_data_len);
      chatter_pub.publish(receiver0.output1);
    }
    // print message in case raw msg is enabled
    if (receiver0.raw_msg_enable)
    {
      for (int temp_var = 0; temp_var < incoming_data_len; temp_var++)
      {
        temp_raw_stream_string = receiver0.can_frametostr(
            receiver0.bus_receiver.frame_temp[temp_var]);
        printf("%s", temp_raw_stream_string.c_str());
      }
    }
    // print output in case debug msg is enabled
    if (receiver0.debug_enable)
    {
      receiver0.print_output();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
