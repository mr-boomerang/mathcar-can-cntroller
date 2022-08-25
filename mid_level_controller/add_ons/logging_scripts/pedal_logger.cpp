// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file pedal_logger.cpp
 * @brief logs speed in step sizes interface for can.
 * @author Divyanshu Goel
 * @date 04/28/2016
 **/

#include "mid_level_controller/Callback_Msg.h"
#include "mid_level_controller/Can_Status_Message.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

ros::Publisher control_pub;
// variables for stepsizes and switch and other variables.
int step_size = 1;
int speed = 0;
int pedal_input = 0;
int pedal_feedback = 0;
int speed_feedback = 0;
int steer_feedback = 0;
int pedal_max = 60;
int pedal = 60;
int steer_feedback_direction = 1;
int updated = 0;
std::string gear_str;
int pedal_init, pedal_limit, speed_limit, steer_val;
double frequency, time_limit;

/// prints the current can States of the car
void print_state()
{
  printf("Input Pedal: %d\n", pedal);
  printf("Feedback Pedal: %d\n", pedal_feedback);
  printf("Feedback Speed: %d\n", speed_feedback);
  printf("Feedback Steer: %d\n", steer_feedback);
  printf("********************************\n");
}
/// prints the current can States of the car
void speed_callback(
    const mid_level_controller::Can_Status_Message::ConstPtr& msgs)
{
  // steer_direction feedback  positioned at 8th position in value matrix of
  // the data.
  if (msgs->value[8].compare("None") != 0 &&
      msgs->value[8].compare("none") != 0)
  {
    if (msgs->value[8].compare("right") == 0) steer_feedback_direction = 1;
    if (msgs->value[8].compare("left") == 0) steer_feedback_direction = -1;
  }
  try
  {
    // speed feedback positioned at 14th position in value matrix of the data.
    if (msgs->value[14].compare("None") != 0 &&
        msgs->value[14].compare("none") != 0 &&
        msgs->value[14].compare("") != 0)
    {
      speed_feedback = std::stoi(msgs->value[14], nullptr, 10);
      updated = 1;
    }
    // steer degree feedback  positioned at 7th position in value matrix of the
    // data.
    if (msgs->value[7].compare("None") != 0 &&
        msgs->value[7].compare("none") != 0 && msgs->value[7].compare("") != 0)
    {
      steer_feedback = std::stoi(msgs->value[7], nullptr, 10);
      steer_feedback = steer_feedback * steer_feedback_direction;
      updated = 1;
    }
    // accln feedback  positioned at 11th position in value matrix of the data.
    if (msgs->value[11].compare("None") != 0 &&
        msgs->value[11].compare("none") != 0 &&
        msgs->value[11].compare("") != 0)
    {
      pedal_feedback = std::stoi(msgs->value[11], nullptr, 10);
      updated = 1;
    }
  }
  catch (std::invalid_argument& e)
  {
  }
}
// send command to vehicle functions start
void send_cmd_to_vehicle(std::string item_requested, double item_value)
{
  mid_level_controller::Callback_Msg teleop_car_control;
  teleop_car_control.topic_name = item_requested;
  teleop_car_control.message_data = std::to_string(item_value);
  control_pub.publish(teleop_car_control);
}
void send_cmd_to_vehicle(std::string item_requested, int item_value)
{
  mid_level_controller::Callback_Msg teleop_car_control;
  teleop_car_control.topic_name = item_requested;
  teleop_car_control.message_data = std::to_string(item_value);
  control_pub.publish(teleop_car_control);
}
void send_cmd_to_vehicle(std::string item_requested, std::string item_value)
{
  mid_level_controller::Callback_Msg teleop_car_control;
  teleop_car_control.topic_name = item_requested;
  teleop_car_control.message_data = item_value;
  control_pub.publish(teleop_car_control);
}

/// reset can settings for transmission
void reset_state()
{
  // bring the car to a rest state
  send_cmd_to_vehicle("Driving_Gear_Control", "neutral");
  send_cmd_to_vehicle("Accln_Control_Percntg_Control", 0);
  send_cmd_to_vehicle("Braking_Control_Percntg_Control", 0);
  send_cmd_to_vehicle("Driving_Lamp_Brake_Light_Control", "off");
  send_cmd_to_vehicle("Driving_Lamp_Head_Lamp_Control", "off");
  send_cmd_to_vehicle("Driving_Lamp_Turn_Indicator_Control", "off");
  send_cmd_to_vehicle("Braking_Control_Percntg_Control", 0);
  send_cmd_to_vehicle("Horn_Control", "off");
  send_cmd_to_vehicle("Wiper_Control", "off");
  usleep(30000);
}
int main(int argc, char** argv)
{
  /// create handles for sigtermination
  ros::init(argc, argv, "test_record_car_pedal");
  // nodehandle
  ros::NodeHandle nh0;
  // ros publisher
  ros::Subscriber speed_sub;
  // handle the inputs
  std::string pedal_init_str = argv[1];
  std::string pedal_limit_str = argv[2];
  std::string speed_limit_str = argv[3];
  std::string steer_val_str = argv[4];
  std::string time_limit_str = argv[5];
  gear_str = argv[6];
  try
  {
    pedal_init = std::stoi(pedal_init_str, nullptr, 10);
    pedal_limit = std::stoi(pedal_limit_str, nullptr, 10);
    speed_limit = std::stoi(speed_limit_str, nullptr, 10);
    steer_val = std::stoi(steer_val_str, nullptr, 10);
    time_limit = std::stod(time_limit_str);
  }
  catch (std::invalid_argument e)
  {
    printf(
        "Wrong argument given, please check the input and restart the "
        "code\n");
    exit(EXIT_FAILURE);
  }
  printf("Starting Conditions\n");
  printf(" Pedal Init: %d percent\n", pedal_init);
  printf(" Pedal Limit: %d percent\n", pedal_limit);
  printf(" Speed Linit: %d kmph\n", speed_limit);
  printf(" Steer val: %d degree\n", steer_val);
  printf(" Update Time: %.1f secs\n", time_limit);
  printf("********************************\n");
  frequency = 1 / time_limit;
  pedal = static_cast<int>(pedal_init);
  // feedback from the gateway
  speed_sub = nh0.subscribe("canStatus", 1000, speed_callback);
  control_pub =
      nh0.advertise<mid_level_controller::Callback_Msg>("canControl", 10);
  usleep(1000000);
  // reset state of car
  reset_state();
  // setting steer
  printf("Setting steer value: %d\n", static_cast<int>(steer_val));

  if (steer_val < 0)
  {
    send_cmd_to_vehicle("Steering_Direction_Control", "left");
    send_cmd_to_vehicle("Steering_Degree_Control", -1 * steer_val);
  }
  else
  {
    send_cmd_to_vehicle("Steering_Direction_Control", "right");
    send_cmd_to_vehicle("Steering_Degree_Control", steer_val);
  }
  // setting gear
  printf("Setting gear value: %s\n", gear_str.c_str());
  if (gear_str[0] == 'b')
  {
    send_cmd_to_vehicle("Driving_Gear_Control", "fwddrive");
    usleep(30000);
    send_cmd_to_vehicle("Driving_Gear_Control", "boost");
    usleep(30000);
  }
  if (gear_str[0] == 'r')
  {
    send_cmd_to_vehicle("Driving_Gear_Control", "reverse");
    usleep(30000);
  }
  if (gear_str[0] == 'f')
  {
    send_cmd_to_vehicle("Driving_Gear_Control", "fwddrive");
    usleep(30000);
  }
  printf("********************************\n");
  // loop to generate the value

  // looprate
  pedal = static_cast<int>(pedal_init);
  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    pedal = (pedal + step_size);
    if (pedal >= 0 && pedal <= pedal_max)
    {
      send_cmd_to_vehicle("Accln_Control_Percntg_Control", pedal);
      usleep(10000);
    }
    print_state();
    if (pedal > pedal_max)
    {
      printf("Reached end for the given pedal value. Exiting\n");
      break;
    }
    if (updated == 1)
    {
      updated = 0;
      if (speed_feedback >= 1.1 * speed_limit)
      {
        printf("Reached Max. speed Limit. Exiting\n");
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  reset_state();
  // for clean exit
  send_cmd_to_vehicle("Accln_Control_Percntg_Control", 0);
  send_cmd_to_vehicle("Braking_Control_Percntg_Control", 88);
  // setting gear
  printf("Setting gear value back to neutral.\n");
  if (gear_str[0] == 'b')
  {
    send_cmd_to_vehicle("Driving_Gear_Control", "fwddrive");
    usleep(30000);
    send_cmd_to_vehicle("Driving_Gear_Control", "neutral");
    usleep(30000);
  }
  if (gear_str[0] == 'r' || gear_str[0] == 'f')
  {
    send_cmd_to_vehicle("Driving_Gear_Control", "neutral");
    usleep(30000);
  }
  return 0;
}
