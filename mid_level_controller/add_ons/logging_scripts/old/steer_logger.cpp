// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file steer_logger.cpp
 * @brief logs steering in stepsizes interface for can. this file allows for
 *driving of car though keyboard.
 * @author Divyanshu Goel
 * @date 02/28/2016
 **/

#include "mid_level_controller/Callback_Msg.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <string>
#include <vector>
/// prints the current can State
mid_level_controller::Callback_Msg teleop_car_control;
ros::Publisher control_pub;

int step_size = 1;
int switch_0 = 0;
int count = 1;
int steer = 0;
/// reset can settings for transmission
void reset_state()
{
  teleop_car_control.topic_name = "Steering_Homing_Control";
  teleop_car_control.message_data = "homing_active";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Steering_Degree_Control";
  teleop_car_control.message_data = "0";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Steering_Direction_Control";
  teleop_car_control.message_data = "right";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Driving_Lamp_Brake_Light_Control";
  teleop_car_control.message_data = "off";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Driving_Lamp_Head_Lamp_Control";
  teleop_car_control.message_data = "off";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Driving_Lamp_Turn_Indicator_Control";
  teleop_car_control.message_data = "off";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Braking_Control_Percntg_Control";
  teleop_car_control.message_data = "0";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Horn_Control";
  teleop_car_control.message_data = "off";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Wiper_Control";
  teleop_car_control.message_data = "off";
  control_pub.publish(teleop_car_control);
  teleop_car_control.topic_name = "Accln_Control_Percntg_Control";
  teleop_car_control.message_data = "0";
  control_pub.publish(teleop_car_control);
}
/// prints the current can States of the car
void print_state() { printf("Steer: %d\n", steer); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_car_steer");
  ros::NodeHandle nh0;
  std::string time_val = argv[1];
  double frequency = std::stod(time_val);
  control_pub =
      nh0.advertise<mid_level_controller::Callback_Msg>("/canControl/", 10);
  ros::Rate loop_rate(frequency);
  reset_state();
  teleop_car_control.topic_name = "Steering_Degree_Control";
  teleop_car_control.message_data = std::to_string(steer);
  control_pub.publish(teleop_car_control);
  while (ros::ok())
  {
    steer = (steer + step_size);
    if (steer >= 0)
    {
      teleop_car_control.topic_name = "Steering_Direction_Control";
      teleop_car_control.message_data = "right";
      control_pub.publish(teleop_car_control);
      teleop_car_control.topic_name = "Steering_Degree_Control";
      teleop_car_control.message_data = std::to_string(steer);
      if (steer >= 40) teleop_car_control.message_data = "40";
      control_pub.publish(teleop_car_control);
    }
    if (steer < 0)
    {
      teleop_car_control.topic_name = "Steering_Direction_Control";
      teleop_car_control.message_data = "left";
      control_pub.publish(teleop_car_control);
      teleop_car_control.topic_name = "Steering_Degree_Control";
      teleop_car_control.message_data = std::to_string(-1 * steer);
      if (steer <= -40) teleop_car_control.message_data = "40";
      control_pub.publish(teleop_car_control);
    }
    if (steer >= 40 || steer <= -40)
    {
      step_size = -1 * step_size;
    }
    if (steer == 0)
    {
      switch_0++;
    }
    if (switch_0 >= 2)
    {
      step_size++;
      switch_0 = 0;
    }
    if (step_size >= 40) exit(EXIT_SUCCESS);
    print_state();
    loop_rate.sleep();
  }

  return (0);
}
