// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_transmitter.cpp
 * @brief receiver for can. This file can be used to transmit
 * messages over can every 10 ms
 * @author Divyanshu Goel
 * @date 2017-02-29 (yyyy-mm-dd)
 **/
#include <can_driver/can_transmitter.h>

#include <algorithm>
#include <string>
#include <vector>

/// initialize parameters
void CanTransmitter::init(const std::string input_yaml_file,
                          const std::string logfilepath,
                          const std::string can_settings_file_path,
                          const int *exit_flag_arg_, bool print_help_flag,
                          bool virtual_can_startup)
{
  PRINT_FUNC_ENTER;
  exit_flag_pointer_ = exit_flag_arg_;
  transmitter_setup(input_yaml_file, logfilepath, can_settings_file_path,
                    print_help_flag, virtual_can_startup);
  if (!*exit_flag_arg_) init_success = true;
  PRINT_FUNC_EXIT;
}
/// to print current can frames array
void CanTransmitter::print_can_message_vector(
    std::vector<canfd_frame> can_message_array)
{
  PRINT_FUNC_ENTER;
  int len = can_message_array.size();
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
  PRINT_FUNC_EXIT;
}
/// send actual revised data to can
void CanTransmitter::start_can_transmitter()
{
  PRINT_FUNC_ENTER;
  if (!init_success)
  {
    PE$ ct_red(
        "CAN Transmitter. Error in intilization. Please Ensure proper "
        "initialization") pendl;
    return;
  }
  int count = 0;
  int max_limit = control_map.can_message_array_map.size();
  while (!exit_flag && !*exit_flag_pointer_)
  {
    auto time_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < max_limit; ++i)
    {
      try
      {
        transmitter.send_port(&control_map.can_message_array_map[i]);
      }
      catch (std::exception &e)
      {
      }
      if (raw_msg_enable)
      {
        double time_is_now = get_timestamp() / 1000000L;
        printf(
            "[%f] TX [can_id : 0x%3x ,can_dlc : %u, can_data : [%u, %u, %u, "
            "%u, %u, %u, %u, %u]\n",
            time_is_now, control_map.can_message_array_map[i].can_id,
            control_map.can_message_array_map[i].len,
            control_map.can_message_array_map[i].data[0],
            control_map.can_message_array_map[i].data[1],
            control_map.can_message_array_map[i].data[2],
            control_map.can_message_array_map[i].data[3],
            control_map.can_message_array_map[i].data[4],
            control_map.can_message_array_map[i].data[5],
            control_map.can_message_array_map[i].data[6],
            control_map.can_message_array_map[i].data[7]);
      }
    }
    if (raw_msg_enable)
    {
      printf("\n");
    }
    exit_flag = *exit_flag_pointer_;
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed_time =
        time_end - time_start;
    usleep(10000 - static_cast<int>(elapsed_time.count()));
  }
  PRINT_FUNC_EXIT;
}
/// This method will be used to update message sent on can by the function call
/// of update
void CanTransmitter::update_transmitter_data(std::string item_requested,
                                             std::string item_data)
{
  PRINT_FUNC_ENTER;
  std::transform(item_requested.begin(), item_requested.end(),
                 item_requested.begin(), ::tolower);
  std::transform(item_data.begin(), item_data.end(), item_data.begin(),
                 ::tolower);

  if (item_requested.compare("driving_gear_control") == 0)
  {
    printf("%s\n", item_data.c_str());
  }
  int message_index = 0, topic_found_index = -1;
  std::vector<std::string> temp_topics_container = control_map.can_topics_map;
  int temp_topics_container_size = temp_topics_container.size();
  for (int i = 0; i < temp_topics_container_size; ++i)
  {
    if (temp_topics_container[i].compare(item_requested) == 0)
    {
      topic_found_index = i;
    }
  }
  if (topic_found_index == -1)
  {
    PE$ ct_red("Invalid Input") pendl;
  }
  if (topic_found_index != -1)
  {
    /// get a list of all the params
    unsigned int temp_can_total_length, temp_can_start_bit,
        temp_can_validity_bit, temp_can_message_length, temp_can_arbitration_id;
    /// get a list of params
    DictionaryMap temp_can_message_dictionary;
    temp_can_total_length = control_map.can_total_length_map[topic_found_index];
    temp_can_start_bit = control_map.can_start_bit_map[topic_found_index];
    temp_can_validity_bit =
        control_map.can_validity_bits_map[topic_found_index];
    temp_can_message_length =
        control_map.can_message_length_map[topic_found_index];
    temp_can_arbitration_id =
        control_map.can_arbitration_id_map[topic_found_index];
    temp_can_message_dictionary =
        control_map.can_dictionary_map[topic_found_index];
    /// find can_message_index on basis of arbitration_id
    int temp_can_message_index = -1;
    std::vector<canfd_frame> temp_can_frame = control_map.can_message_array_map;
    int temp_can_message_container_size =
        control_map.can_message_array_map.size();
    for (int i = 0; i < temp_can_message_container_size; ++i)
    {
      if (temp_can_arbitration_id == temp_can_frame[i].can_id)
      {
        temp_can_message_index = i;
      }
    }
    int validity_bitset = 0b1;
    int value_to_set = 0;
    if (item_data.compare("invalid") == 0)
    {
      validity_bitset = 0b0;
    }
    int temp_can_message_dictionary_container_size =
        temp_can_message_dictionary.key.size();
    for (int i = 0; i < temp_can_message_dictionary_container_size; ++i)
    {
      if (temp_can_message_dictionary.key[i].compare("max_value") == 0)
      {
        if (item_data.compare("invalid") != 0)
        {
          value_to_set = std::stoul(item_data, nullptr, 10);
        }
        else
        {
          value_to_set = 0;
        }
        if (value_to_set < 0)
        {
          value_to_set = 0;
        }
        if (value_to_set > temp_can_message_dictionary.value[i])
          value_to_set = temp_can_message_dictionary.value[i];
      }
      if (temp_can_message_dictionary.key[i].compare(item_data) == 0)
      {
        value_to_set = temp_can_message_dictionary.value[i];
      }
    }
    // at this point there I have can_id index , values and bits
    uint64_t temp_new_msg = 0, reset_msg = 0, reset_valid_msg = 0,
             can_msg_tmp = 0, temp_new_msg_validity_msg = 0;
    // convert message into revelant type
    temp_new_msg = returns_value_msg(temp_can_start_bit, value_to_set);
    temp_new_msg_validity_msg =
        returns_value_msg(temp_can_validity_bit, validity_bitset);
    temp_new_msg = temp_new_msg | temp_new_msg_validity_msg;
    reset_msg = returns_reset_msg(temp_can_start_bit, temp_can_message_length);
    reset_valid_msg = returns_reset_msg(temp_can_validity_bit, 1);
    reset_msg = reset_msg & reset_valid_msg;
    transmitter.array_to_byte(
        &can_msg_tmp,
        control_map.can_message_array_map[temp_can_message_index].data);
    can_msg_tmp = can_msg_tmp & reset_msg;
    can_msg_tmp = can_msg_tmp | temp_new_msg;
    uint8_t new_message[8];
    transmitter.byte_to_array(new_message, can_msg_tmp);
    memcpy(control_map.can_message_array_map[temp_can_message_index].data,
           new_message, 8);
    // print control exerted information
    if (debug_enable)
    {
      printf("Topic : %s\n", item_requested.c_str());
      printf("Dictionary for controlled topic: ");
      control.print_dictionary_map(temp_can_message_dictionary);
      printf("\nTotal length bit : %d\n", temp_can_total_length);
      printf("Start bit : %d\n", temp_can_start_bit);
      printf("Validity Bit : %d\n", temp_can_validity_bit);
      printf("Mesage Length is : %d\n", temp_can_message_length);
      printf("Message ID : 0x%3x\n", temp_can_arbitration_id);
      printf("Reset message is : ");
      std::cout << std::bitset<64>(reset_msg) << std::endl;
      printf("Relevant Message Generated is : ");
      std::cout << std::bitset<64>(temp_new_msg) << std::endl;
      printf("Can index Generated is : %d\n", temp_can_message_index);
      printf("Final Message Generated is : ");
      std::cout << std::bitset<64>(can_msg_tmp) << std::endl;
      printf("CAN message Generated is : \n");
      print_can_message_vector(control_map.can_message_array_map);
    }
    if (log_data_enable) write_to_data_log_file(item_requested, item_data);
  }
  PRINT_FUNC_EXIT;
}
#ifndef BUILD_WITHOUT_ROS
/// This method will be used to update message sent on can by the publisher call
/// to /canControl
void CanTransmitter::can_callback(
    const mid_level_controller::Callback_Msg::ConstPtr &msg)
{
  PRINT_FUNC_ENTER;
  // processes any command sent
  std::string item_requested, item_data;
  item_requested = msg->topic_name;
  item_data = msg->message_data;
  update_transmitter_data(item_requested, item_data);
  PRINT_FUNC_EXIT;
}
#endif
/// This method will be used to print all the variables and data in the map
void CanTransmitter::print_structure_stored(CanTopicsDataMap input_map)
{
  PRINT_FUNC_ENTER;
  int len = input_map.can_topics_map.size();
  printf("Message Names : ");
  control.print_names_vector(input_map.can_topics_map);
  printf("\nTotal Length array : ");
  control.print_u_int_vector(input_map.can_total_length_map);
  printf("Start bit array : ");
  control.print_u_int_vector(input_map.can_start_bit_map);
  printf("Message Length array : ");
  control.print_u_int_vector(input_map.can_message_length_map);
  printf("Validity Bits array : ");
  control.print_u_int_vector(input_map.can_validity_bits_map);
  printf("Message Count array : ");
  control.print_u_int_vector(input_map.can_message_count_map);
  printf("Arbitration id array : ");
  control.print_u_int_vector(input_map.can_arbitration_id_map);
  print_can_message_vector(input_map.can_message_array_map);
  PRINT_FUNC_EXIT;
}
// execute a system command
int CanTransmitter::exec_command(std::string command)
{
  PRINT_FUNC_ENTER;
  char buf[BUFSIZE];
  if (debug_enable)
    printf("Trying to execute system command : %s\n", command.c_str());
  if (command.compare("ip link") == 0)
  {
    FILE *fp;
    int inteface_found = 0;
    if ((fp = popen(command.c_str(), "r")) == NULL)
    {
      printf("Error opening pipe!\n");
      PRINT_FUNC_EXIT;
      return -1;
    }

    while (fgets(buf, BUFSIZE, fp) != NULL)
    {
      if (debug_enable)
      {
        printf("OUTPUT: %s", buf);
      }
      std::string temp = std::string(buf);
      int found = temp.find(':');
      if (found != std::string::npos)
      {
        temp = temp.substr(found + 1);
        found = temp.find(':');
        temp = temp.substr(1, found - 1);
        // printf("data: %s\n", temp.c_str());
        if (temp.compare(can_interface) == 0) inteface_found = 1;
      }
    }
    if (pclose(fp))
    {
      printf("Command not found or exited with error status\n");
      PRINT_FUNC_EXIT;
      return -1;
    }
    return inteface_found;
  }
  else
  {
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
}
// setup can to be able to open a socket to it later.
int CanTransmitter::can_setup()
{
  PRINT_FUNC_ENTER;
  int interface_enable = 0;
  // if root password exists for vcan
  if (can_interface == "vcan0" && root_password_present)
  {
    // check if module exists
    std::string command = "ip link";
    interface_enable = exec_command(command);
    if (!interface_enable)
    {
      PN$ ct_grn("CAN interface NOT found -_- Enabling CAN interface") pendl;
      std::string command =
          "echo " + sudo_password + " | sudo -kS modprobe vcan";
      interface_enable = exec_command(command);
      command =
          "echo " + sudo_password + " | sudo -kS ip link add vcan0 type vcan";
      interface_enable = exec_command(command);
      command = "ip link";
      interface_enable = exec_command(command);
      if (!interface_enable)
      {
        PE$ ct_red(
            "Could not start CAN interface vcan0 as it doesn't exist in "
            "ip link list.Please go through the installation docs on "
            "how to setup the drivers.") pendl;
        return -1;
      }
    }
    else
    {
      PN$ ct_grn("CAN Device found -_- Setting CAN interface UP on vcan0")
          pendl;
      command = "echo " + sudo_password + " | sudo -kS ip link set vcan0 up";
      interface_enable = exec_command(command);
    }
  }
  // if root password doesn't exists for vcan
  if (can_interface == "vcan0" && !root_password_present)
  {
    // check if module exists
    std::string command = "ip link";
    interface_enable = exec_command(command);
    if (!interface_enable)
    {
      PN$ ct_grn("CAN Device NOT found -_- Enabling CAN interface") pendl;
      command = "sudo ip link add vcan0 type vcan";
      interface_enable = exec_command(command);
      usleep(10000);
      command = "ip link";
      interface_enable = exec_command(command);
      if (!interface_enable)
      {
        PE$ ct_red(
            "Could not start CAN interface vcan0 as it doesn't exist in "
            "ip link list. Please go through the installation docs on "
            "how to setup the drivers.") pendl;
        return -1;
      }
    }
    else
    {
      PN$ ct_grn("CAN Device found -_- Setting CAN interface UP on vcan0")
          pendl;
      command = "sudo ip link set vcan0 up";
      interface_enable = exec_command(command);
    }
  }
  // if root password exists for can
  if (can_interface == "can0" && root_password_present)
  {
    // check if module exists
    std::string command = "ip link";
    interface_enable = exec_command(command);
    if (interface_enable == 1)
    {
      PN$ ct_grn("Setting CAN interface UP on can0") pendl;
      command = "echo " + sudo_password +
                " | sudo -kS ip link set can0 up type can bitrate 500000";
      interface_enable = exec_command(command);
    }
    else
    {
      PE$ ct_red(
          "Could not start CAN interface CAN0 as it doesn't exist in ip "
          "link list.Please go through the installation docs on how to "
          "setup the drivers") pendl;
      return -1;
    }
  }
  // if root password doesn't exists for vcan
  if (can_interface == "can0" && !root_password_present)
  {
    // check if module exists
    std::string command = "ip link";
    interface_enable = exec_command(command);
    // std::cout << interface_enable << std::endl;
    if (interface_enable == 1)
    {
      PN$ ct_grn("Setting CAN interface UP on can0") pendl;
      command = "sudo ip link set can0 up type can bitrate 500000";
      interface_enable = exec_command(command);
    }
    else
    {
      PE$ ct_red(
          "Could not start CAN interface CAN0 as it doesn't exist in ip "
          "link list. Please go through the installation docs on how to "
          "setup the drivers") pendl;
      PRINT_FUNC_EXIT;
      return (-1);
    }
  }
  PRINT_FUNC_EXIT;
  return 1;
}
/// This method will be used to setup all the variables and data for the program
/// to work.
void CanTransmitter::transmitter_setup(const std::string input_yaml_file,
                                       const std::string logfilepath,
                                       const std::string can_settings_file_path,
                                       bool print_help_flag,
                                       bool virtual_can_startup)
{
  PRINT_FUNC_ENTER;
  /// Load transmitter and receiver settings by reading the yaml and the final
  /// filtered outputs are contained in the can_io_message vector of type
  /// can_message
  control.read_file(input_yaml_file);
  control.close_file();
  if (!params_loaded)
  {
    // load parameters using utils
    params_utils.load(can_settings_file_path);
  }
  // store values in local variables
  can_interface = params_utils.get_val<std::string>("can_channel", "can0");
  raw_msg_enable = params_utils.get_val<bool>("raw_msg_tx_enable", false);
  log_enable = params_utils.get_val<bool>("log_enable", true);
  log_data_enable = params_utils.get_val<bool>("log_data_tx_enable", true);
  debug_enable = params_utils.get_val<bool>("debug_msg_tx_enable", false);
  print_help = print_help_flag;
  // assumption that user is added to group can_group and allowed to use sudo
  // if value is empty or use value directly if its present.
  sudo_password = params_utils.get_val<std::string>("sudo_passwd", "");
  if (virtual_can_startup) can_interface = "vcan0";
  // check for sudo password
  root_password_present = true;
  if (sudo_password == "")
  {
    PN$ ct_grn(
        "No sudo password present. Assuming command requires no password "
        "(added to sudoers list or can group) and proceeding ahead.") pendl;
    root_password_present = false;
  }
  if (sudo_password != "")
  {
    PN$ ct_grn("Sudo password present.") pendl;
    root_password_present = true;
  }
  // check for interface and start the interface
  int array_len = control.can_io_message.size();
  int can_setup_success = 0;
  int trial_count = 100;
  while (can_setup_success != 1 && trial_count > 0 && !*exit_flag_pointer_)
  {
    can_setup_success = can_setup();
    if (can_setup_success == -1)
    {
      // sleep for 2 secs and restart the connection
      usleep(2000000);
      if (trial_count > 0)
      {
        std::stringstream print_message;
        print_message << "Retrying Connection( " << trial_count << " )...";
        PN$ ct_grn(print_message.str()) pendl;
      }
    }
    trial_count = trial_count - 1;
  }
  if (trial_count <= 0 && can_setup_success != 1)
  {
    PF$(1)
    ct_red(
        "Could not start CAN interface as it doesn't exist in ip link list. "
        "Please go through the installation docs on how to setup the "
        "drivers or check connection") pendl;
  }
  if (*exit_flag_pointer_) return;
  // sleep for 2 secs and to ensure proper startup.
  usleep(2000000);
  // displays what controls can be exerted
  if (print_help) printf("Inputs for Control : \n");
  for (int i = 0; i < array_len; ++i)
  {
    /// create an empty frame with given arbitration id and add it to
    /// structure
    canfd_frame temp_can_frame;
    temp_can_frame.can_id = control.can_io_message[i].message_id;
    temp_can_frame.len = control.can_io_message[i].total_length / 8;
    __u8 data[] = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(temp_can_frame.data, data, 8);
    control_map.can_message_array_map.push_back(temp_can_frame);

    control_map.can_message_count_map.push_back(
        control.can_io_message[i].message_count);

    int second_loop_limit = control.can_io_message[i].message_count;
    for (int j = 0; j < second_loop_limit; ++j)
    {
      /// get the parameters from the file in following order: start_bit,
      /// message_length, validity_bit, message_names, dictionary
      /// associated, arbitration id
      control_map.can_total_length_map.push_back(
          control.can_io_message[i].total_length);
      control_map.can_start_bit_map.push_back(
          control.can_io_message[i].start_bit[j]);
      control_map.can_message_length_map.push_back(
          control.can_io_message[i].message_length[j]);
      control_map.can_validity_bits_map.push_back(
          control.can_io_message[i].validity_bit[j]);
      std::transform(control.can_io_message[i].names[j].begin(),
                     control.can_io_message[i].names[j].end(),
                     control.can_io_message[i].names[j].begin(), ::tolower);
      control_map.can_topics_map.push_back(control.can_io_message[i].names[j]);
      control_map.can_dictionary_map.push_back(
          control.can_io_message[i].dictionary[j]);

      control_map.can_arbitration_id_map.push_back(
          control.can_io_message[i].message_id);
      if (print_help)
      {
        printf("   %s\n", control.can_io_message[i].message_name.c_str());
        /// Print input guide for the inputs
        printf("\ttopic_name : %s\n",
               control.can_io_message[i].names[j].c_str());
      }
      std::string key_match_copy =
          control.can_io_message[i].dictionary[j].key[0];
      /// make mathcxes case insensitive
      std::transform(key_match_copy.begin(), key_match_copy.end(),
                     key_match_copy.begin(), ::tolower);
      /// check if its a integer input
      if (print_help)
      {
        if (key_match_copy.compare("max_value") == 0)
        {
          std::cout << "\tInteger Input : ["
                    << control.can_io_message[i].dictionary[j].key[0] << "--> "
                    << control.can_io_message[i].dictionary[j].value[0] << "]"
                    << std::endl;
        }
        else
        {
          /// otherwise print options
          printf("\tInputs : [");
          int key_limits = control.can_io_message[i].dictionary[j].key.size();
          for (int each_key = 0; each_key < key_limits; ++each_key)
          {
            std::cout << "\'"
                      << control.can_io_message[i].dictionary[j].key[each_key]
                      << "'";
            if (each_key < key_limits - 1) std::cout << ", ";
          }
          /// default option to set validity bit to zero
          printf("]\n");
        }
      }
    }
  }
  if (debug_enable)
  {
    printf("Data read from Input file:\n");
    status.param_print_all();
    printf("Data stored from file:\n");
    print_structure_stored(control_map);
  }
  // get time stamp now to create a log file
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  std::time_t time_now0 = std::chrono::system_clock::to_time_t(start);
  struct tm *now = std::localtime(&time_now0);
  /// open can port
  std::stringstream file_name_tx_stream;
  // log file name
  file_name_tx_stream << logfilepath << (now->tm_year + 1900) << '_'
                      << (now->tm_mon + 1) << '_' << now->tm_mday << "_"
                      << (now->tm_hour) << "_" << (now->tm_min) << "_"
                      << (now->tm_sec) << "_CAN_tx.log";
  std::string file_name_tx_string = file_name_tx_stream.str();

  /// file to save parsed information
  std::stringstream file_name_tx_data_stream;
  file_name_tx_data_stream << logfilepath << (now->tm_year + 1900) << '_'
                           << (now->tm_mon + 1) << '_' << now->tm_mday << "_"
                           << (now->tm_hour) << "_" << (now->tm_min) << "_"
                           << (now->tm_sec) << "_data_tx.log";

  std::string file_name_tx_data_string = file_name_tx_data_stream.str();

  printf("*********************************************\n");
  printf(" Transmitter data : \n");
  if (log_enable)
    printf(" Writing CAN Log file to : %s\n", file_name_tx_string.c_str());
  if (log_data_enable) open_log_file(file_name_tx_data_string.c_str());
  printf(" Writing to can bus at: %s\n", can_interface.c_str());
  printf("*********************************************\n");
  int open_port_exception = 1;

  while (open_port_exception && !*exit_flag_pointer_)
  {
    try
    {
      transmitter.open_port(can_interface.c_str(), file_name_tx_string.c_str(),
                            log_enable, exit_flag_pointer_);
      open_port_exception = 0;
    }
    catch (std::exception &e)
    {
      open_port_exception = 1;
      usleep(2000000);
    }
  }
  PRINT_FUNC_EXIT;
}

// open file handle for data
void CanTransmitter::open_log_file(const std::string logfilepath)
{
  PRINT_FUNC_ENTER;
  printf(" Writing DATA Log file to : %s\n", logfilepath.c_str());
  data_log_file.open_file(logfilepath);
  std::stringstream data;
  data << "Timestamp, Command, Value\n";
  std::string data_str = data.str();
  data_log_file.print_string_message_to_file(data_str.c_str());
  data_log_file.flush();
  PRINT_FUNC_EXIT;
}
// write data to file from the transmitter after parsing
void CanTransmitter::write_to_data_log_file(std::string item_requested,
                                            std::string item_data)
{
  PRINT_FUNC_ENTER;
  std::stringstream data;
  double timestamp_log = get_timestamp() / 1000000L;
  data << std::to_string(timestamp_log) << ", " << item_requested << ", "
       << item_data << std::endl;
  std::string data_str = data.str();
  data_log_file.print_string_message_to_file(data_str.c_str());
  PRINT_FUNC_EXIT;
}
// clean exit of file writing
void CanTransmitter::close_data_log_file()
{
  PRINT_FUNC_ENTER;
  data_log_file.flush();
  data_log_file.close_file();
  PRINT_FUNC_EXIT;
}
/// cleanly exit from transmitter
void CanTransmitter::transmitter_exit(int signum)
{
  PRINT_FUNC_ENTER;
  close_data_log_file();
  exit_flag = 1;

  if (init_success)
  {
    int interface_enable;
    transmitter.close_port();
    // if root password doesn't exists for can
    if (can_interface == "can0" && !root_password_present)
    {
      // check if module exists
      std::string command = "sudo ip link set can0 down";
      interface_enable = exec_command(command);
    }
    if (can_interface == "can0" && root_password_present)
    {
      std::string command;
      printf("Disabling interface\n");
      command = "echo " + sudo_password + " | sudo -kS ip link set can0 down";
      interface_enable = exec_command(command);
    }
    // if root password doesn't exists for can
    if (can_interface == "vcan0" && !root_password_present)
    {
      // check if module exists
      std::string command = "sudo ip link set vcan0 down";
      interface_enable = exec_command(command);
    }
    if (can_interface == "vcan0" && root_password_present)
    {
      std::string command;
      printf("Disabling interface\n");
      command = "echo " + sudo_password + " | sudo -kS ip link set vcan0 down";
      interface_enable = exec_command(command);
      exit_flag = 1;
    }
  }
  PRINT_FUNC_EXIT;
}
// listener from the reciever
void CanTransmitter::start_can_transmitter_thd()
{
  PRINT_FUNC_ENTER;
  canBusSenderIsThreaded = true;
  /// listens to bus blocking call so started as a thread
  can_bus_sender = std::thread(&CanTransmitter::start_can_transmitter, this);
  PN$ PRINT_SUCCESS << ct_grn("Transmitter Working") pendl;
  PRINT_FUNC_EXIT;
}
/// destructor
CanTransmitter::~CanTransmitter()
{
  PRINT_FUNC_ENTER;
  exit_flag = true;
  // closing other things
  if (can_bus_sender.joinable()) can_bus_sender.join();
  PRINT_FUNC_EXIT;
}
/// cleanly exit from transmitter
void CanTransmitter::signalHandler(int signum)
{
  PRINT_FUNC_ENTER;
  /// clean exit
  PI$ "Closing transmitter instance." pendl;
  transmitter_exit(signum);
  PRINT_FUNC_EXIT;
}
