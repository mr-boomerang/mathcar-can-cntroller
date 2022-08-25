// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file common_utils.cpp
 * @brief joystick utilities for changing commands and gears etc.
 * @author Divyanshu Goel
 * @date 2018-05-03 (yyyy-mm-dd)
 **/
#include <controller/common_utils.h>

#include <algorithm>  /// to use transform
#include <chrono>     /// to use timestamps
#include <string>     /// to use strings
#include <vector>     /// to use vectors

/// This method will be used to initiate the class options
void CommonUtils::Init(const int *exit_flag_arg, std::string joy_path,
                       std::string slope_topic, bool print_help,
                       bool debug_enable, bool slope_enable)
{
  PRINT_FUNC_ENTER;
  exit_flag_ptr_ = exit_flag_arg;
  joy_path_ = joy_path;
  print_help_ = print_help;
  debug_enable_ = debug_enable;
  slope_enable_ = slope_enable;

#ifndef BUILD_WITHOUT_ROS
  if (ros::isInitialized() && !(*exit_flag_ptr_))
  {
    CanRosSetup();
  }
#endif
  PRINT_FUNC_EXIT;
}
/// This method will be used to get debug_enable value
bool CommonUtils::GetDebugOption() { return (debug_enable_); }
/// This method will be used to set debug_enable value
void CommonUtils::SetDebugOption(bool value) { debug_enable_ = value; }
/// This method will be used to set help option
void CommonUtils::SetHelpOption(bool value) { print_help_ = value; }
/// This method will be used to return slope value
double CommonUtils::GetSlopeValue() { return (current_slope_); }
/// set value of slope topic
void CommonUtils::SetSlopeTopic(std::string slope_topic)
{
  slope_topic_ = slope_topic;
}
/// This method will be used to execute a system command
int CommonUtils::ExecCommand(std::string command)
{
  PRINT_FUNC_ENTER;
  std::vector<std::string> valid_ports;
  int valid_joy = -1;
  char buf[BUFFSIZE];
  if (debug_enable_)
    printf("Trying to execute system command : %s\n", command.c_str());
  FILE *fp;
  if ((fp = popen(command.c_str(), "r")) == NULL)
  {
    printf("Error opening pipe!\n");
    return -1;
  }
  while (fgets(buf, BUFFSIZE, fp) != NULL)
  {
    if (debug_enable_)
    {
      printf("OUTPUT: %s", buf);
    }
    std::string output0 = std::string(buf);
    if (command.compare("ls -l /dev/input") == 0)
    {
      int loc = output0.find("js");
      if (loc != std::string::npos)
      {
        int loc_end = output0.find("\n");
        int length = output0.length();
        std::string output_port = output0.substr(loc, (length + 2) - loc_end);
        valid_ports.push_back(output_port);
      }
    }
  }
  if (command.compare("ls -l /dev/input") == 0)
  {
    bool matched = false;
    if (print_help_) printf("Available Ports:\n");
    int length_port = valid_ports.size();
    for (int i = 0; i < length_port; i++)
    {
      valid_ports[i] = "/dev/input/" + valid_ports[i];
      if (debug_enable_)
        printf("\t%d. \033[1;32m%s\033[0m\n", i + 1, valid_ports[i].c_str());
      if (joy_path_.compare(valid_ports[i]) == 0) matched = true;
    }
    if (!matched)
    {
      printf("\033[1;31mInput Device Doesn't Exist.\033[0m\n");
      if (length_port > 0)
      {
        printf(
            "\033[1;31mInvalid Device Input!!! Device changed to %s \033[0m\n",
            valid_ports[0].c_str());
        joy_path_ = valid_ports[0];
        valid_joy = 1;
      }
      else
      {
        printf(
            "\033[1;31mInvalid Device Input!!! No valid device "
            "found.\033[0m\n");
      }
    }
    else
    {
      valid_joy = 1;
      if (debug_enable_) printf("[Default]: %s\n", joy_path_.c_str());
    }
  }
  if (pclose(fp))
  {
    printf("Command not found or exited with error status\n");
    return -1;
  }
  PRINT_FUNC_EXIT;
  return valid_joy;
}
/// return path to joystick
std::string CommonUtils::GetJoyPath()
{
  PRINT_FUNC_ENTER;
  PRINT_FUNC_EXIT;
  /// return the joystick path
  return joy_path_;
}
/// * This method will be used to set mode of joystick
void CommonUtils::SetMode(std::string mode)
{
  PRINT_FUNC_ENTER;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
  if (mode.compare("throttle") == 0)
    mode_ = "throttle";
  else
    mode_ = "controller";
  PRINT_FUNC_EXIT;
}
/// This method will be used to start interface
void CommonUtils::SetGear(std::string target_gear)
{
  PRINT_FUNC_ENTER;
  // printf("asdasd\n");
  // std::string gear_current = car_state_.gear_fdbk;
  // sanity check
  if (gear_current.compare(target_gear) == 0) return;
  printf("\n");
  // if current gear is reverse
  if (gear_current.compare("reverse") == 0)
  {
    if (target_gear.compare("fwddrive") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
    }
    if (target_gear.compare("neutral") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
    }

    if (target_gear.compare("boost") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "boost");
    }
  }
  // if current gear is neutral
  if (gear_current.compare("neutral") == 0)
  {
    if (target_gear.compare("fwddrive") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
    }
    if (target_gear.compare("reverse") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "reverse");
    }
    if (target_gear.compare("boost") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "boost");
    }
  }
  // if current gear is fwddrive
  if (gear_current.compare("fwddrive") == 0)
  {
    if (target_gear.compare("neutral") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
    }
    if (target_gear.compare("reverse") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "reverse");
    }
    if (target_gear.compare("boost") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "boost");
    }
  }
  // if current gear is fwddrive
  if (gear_current.compare("boost") == 0)
  {
    if (target_gear.compare("neutral") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
    }
    if (target_gear.compare("reverse") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "neutral");
      // sleep for 31000 usecs and switch gear
      usleep(31000);
      SendCmdToVehicle("Driving_Gear_Control", "reverse");
    }
    if (target_gear.compare("fwddrive") == 0)
    {
      SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
    }
  }
  gear_current = target_gear;
  PRINT_FUNC_EXIT;
}
/// This method will be used to update command to be sent to controller
void CommonUtils::SendCmdToController(double speed, double steer,
                                      double lock_mode_speed,
                                      double lock_mode_steer,
                                      double brake_press)
{
  PRINT_FUNC_ENTER;
#ifndef BUILD_WITHOUT_ROS
  geometry_msgs::Twist teleop_car_control;
  teleop_car_control.linear.x = speed;
  teleop_car_control.linear.y = brake_press;
  teleop_car_control.linear.z = lock_mode_speed;
  teleop_car_control.angular.x = lock_mode_steer;
  teleop_car_control.angular.z = steer;
  speed_control_.publish(teleop_car_control);
#else
  printf("Please substitute for the publish messages in absence of ROS\n");
#endif
  PRINT_FUNC_EXIT;
}
/// This method will be used to update command to be sent to vehicle
void CommonUtils::SendCmdToVehicle(std::string item_requested,
                                   std::string item_value)
{
  PRINT_FUNC_ENTER;
#ifndef BUILD_WITHOUT_ROS
  mid_level_controller::Callback_Msg teleop_car_control;
  teleop_car_control.topic_name = item_requested;
  teleop_car_control.message_data = item_value;
  control_pub_.publish(teleop_car_control);
#else
  printf("Please substitute for the publish messages in absence of ROS\n");
#endif
  PRINT_FUNC_EXIT;
}
/// This method will be used to update command to be sent to vehicle
void CommonUtils::SendCmdToVehicle(std::string item_requested, int item_value)
{
  PRINT_FUNC_ENTER;
  SendCmdToVehicle(item_requested, std::to_string(item_value));
  PRINT_FUNC_EXIT;
}
/// This method will be used to update command to be sent to vehicle
void CommonUtils::SendCmdToVehicle(std::string item_requested,
                                   double item_value)
{
  PRINT_FUNC_ENTER;
  SendCmdToVehicle(item_requested, std::to_string(item_value));
  PRINT_FUNC_EXIT;
}
/// This method will be used to keep track is data is updated
bool CommonUtils::GetDataStateUpdated()
{
  PRINT_FUNC_ENTER;
  PRINT_FUNC_EXIT;
  return (data_updated_);
}
/// This method will be used to let the code know that data is stale
void CommonUtils::SetDataStateUpdatedStale()
{
  PRINT_FUNC_ENTER;
  data_updated_ = false;
  PRINT_FUNC_EXIT;
}
/// This method will be used to keep track is data is updated
double CommonUtils::GetDataStateTimeUpdated()
{
  PRINT_FUNC_ENTER;
  PRINT_FUNC_EXIT;
  return (car_state_.speed_header.stamp);
}
/// This method will be used to update current vehicle state
bool CommonUtils::CheckmsgAtIndex(std::string msgs[], int index)
{
  PRINT_FUNC_ENTER;
  bool check_msg_value = msgs[index].compare("None") != 0 &&
                         msgs[index].compare("none") &&
                         msgs[index].compare("") != 0;
  PRINT_FUNC_EXIT;
  return (check_msg_value);
}
/// This method will be used to update current vehicle state
void CommonUtils::CanCallback(std::string msgs[])
{
  PRINT_FUNC_ENTER;
  int steer_out_direction_multiplier = 1;
  int steer_in_direction_multiplier = 1;
  try
  {
    // get proper value for various things
    // steer_direction commanded  positioned at 20th position in value matrix
    // of the data. Check if the value is not valid otherwise update it with
    // present value. similar is done for all the variables below.
    if (CheckmsgAtIndex(msgs, 20))
    {
      if (msgs[20].compare("right") == 0) steer_out_direction_multiplier = -1;
      if (msgs[20].compare("left") == 0) steer_out_direction_multiplier = 1;
    }
    // steer_direction feedback  positioned at 8th position in value matrix of
    // the data.
    if (CheckmsgAtIndex(msgs, 8))
    {
      if (msgs[8].compare("right") == 0) steer_in_direction_multiplier = -1;
      if (msgs[8].compare("left") == 0) steer_in_direction_multiplier = 1;
    }
    // gear value feedback  positioned at 4th position in value matrix of the
    // data.
    if (CheckmsgAtIndex(msgs, 3)) car_state_.gear_fdbk = msgs[3];
    // steer degree feedback  positioned at 7th position in value matrix of the
    // data.
    if (CheckmsgAtIndex(msgs, 7))
    {
      int steer_in = std::stoi(msgs[7], nullptr, 10);
      car_state_.steer_fdbk = steer_in * steer_in_direction_multiplier;
    }
    // steer degree commanded  positioned at 19th position in value matrix of
    // the data.
    if (CheckmsgAtIndex(msgs, 19))
    {
      int steer_out = std::stoi(msgs[19], nullptr, 10);
      car_state_.steer_cmd = steer_out * steer_out_direction_multiplier;
    }
    // accln commanded  positioned at 23th position in value matrix of the data.
    if (CheckmsgAtIndex(msgs, 23))

      car_state_.throttle_cmd = std::stoi(msgs[23], nullptr, 10);
    // accln feedback  positioned at 11th position in value matrix of the data.
    if (CheckmsgAtIndex(msgs, 11))
      car_state_.throttle_fdbk = std::stoi(msgs[11], nullptr, 10);

    // brake percentage commanded positioned at 22th position in value matrix of
    // the data.
    if (CheckmsgAtIndex(msgs, 22))
      car_state_.brake_cmd = std::stoi(msgs[22], nullptr, 10);
    // brake percentage feedback positioned at 10th position in value matrix of
    // the data.
    if (CheckmsgAtIndex(msgs, 10))
      car_state_.brake_fdbk = std::stoi(msgs[10], nullptr, 10);
    // speed feedback positioned at 14th position in value matrix of the data.
    if (CheckmsgAtIndex(msgs, 14))
      car_state_.speed_fdbk = std::stoi(msgs[14], nullptr, 10);
    // car state
    if (CheckmsgAtIndex(msgs, 2))
    {
      if (msgs[2].compare("autonomus") == 0)
        car_state_.auto_mode_count = 0;
      else
        car_state_.auto_mode_count++;
      if (car_state_.auto_mode_count >= 500) car_state_.auto_mode_count = 500;
    }

    car_state_.SpeedSetTime();
    data_updated_ = 1;

    if (debug_enable_)
      printf("Current Gateway Speed %i kmph\n", car_state_.speed_fdbk);
  }
  catch (std::invalid_argument e)
  {
  }
  PRINT_FUNC_EXIT;
}

/// This method will be used to update current vehicle state
void CommonUtils::SlopeUpdateRos(double value)
{
  // update current value
  current_slope_ = value;
}
#ifndef BUILD_WITHOUT_ROS

/// This method will be used to update command to be sent to vehicle speed
/// controller
void CommonUtils::SendCmdToController(geometry_msgs::Twist control_input)
{
  speed_control_.publish(control_input);
}
/// This method will be used to setup ROS feedback/publisher connections
void CommonUtils::CanRosSetup()
{
  PRINT_FUNC_ENTER;
  // instance of ros node handle
  ros::NodeHandle nh;
  // to get sensor feedback
  car_sub_ = nh.subscribe("canStatus", 10, &CommonUtils::CanCallback, this);
  // to publish the control message to vehicle
  control_pub_ =
      nh.advertise<mid_level_controller::Callback_Msg>("canControl", 10);
  // to publish the control message to controller
  speed_control_ =
      nh.advertise<geometry_msgs::Twist>("velocitySteeringAngle", 10);
  if (slope_enable_)
    // to get sensor feedback
    feedback_sub_slope =
        nh.subscribe(slope_topic_, 10, &CommonUtils::SlopeUpdateRos, this);
  PRINT_FUNC_EXIT;
}
/// This method will be used to update current vehicle state
void CommonUtils::SlopeUpdateRos(const std_msgs::Float64::ConstPtr &msg)
{
  SlopeUpdateRos(static_cast<double>(msg->data));
}
/// This method will be used to update current vehicle state
void CommonUtils::CanCallback(
    const mid_level_controller::Can_Status_Message::ConstPtr &msg)
{
  PRINT_FUNC_ENTER;
  // copy data to a valid matrix
  int len = msg->value.size();
  for (int element_no = 0; element_no < len; element_no++)
  {
    data_[element_no] = msg->value[element_no];
  }
  // process feedback
  CanCallback(data_);
  PRINT_FUNC_EXIT;
}
void CommonUtils::ResetCar()
{
  // bring the car to a rest state
  SendCmdToVehicle("Driving_Gear_Control", "fwddrive");
  SendCmdToVehicle("Accln_Control_Percntg_Control", 0);
  SendCmdToVehicle("Braking_Control_Percntg_Control", 90);
  SendCmdToVehicle("Steering_Homing_Control", "homing_active");
  SendCmdToVehicle("Steering_Direction_Control", "right");
  SendCmdToVehicle("Steering_Degree_Control", 0);
  SendCmdToVehicle("Driving_Lamp_Brake_Light_Control", "off");
  SendCmdToVehicle("Driving_Lamp_Head_Lamp_Control", "off");
  SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "off");
  SendCmdToVehicle("Braking_Control_Percntg_Control", 0);
  SendCmdToVehicle("Horn_Control", "off");
  SendCmdToVehicle("Wiper_Control", "off");
}
#endif
