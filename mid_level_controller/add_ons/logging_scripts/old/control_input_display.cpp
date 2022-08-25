// Copyright 2011-2018 The Mathworks, Inc.
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
int value = 0;
ros::Publisher control_pub;
void chatterCallback(const std_msgs::Int64::ConstPtr &msg)
{
  value = msg->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_control_input_display");
  ros::NodeHandle n;
  // to get sensor feedback
  ros::Subscriber feedback_sub =
      n.subscribe("/can/control_velocity", 1000, chatterCallback);
  // to publish the control message
  ros::Publisher control_pub =
      n.advertise<std_msgs::Int64>("/can/control_velocity_2", 10);
  // update rate set to 100 Hz
  ros::Rate loop_rate(100);
  // variables to contain messages
  std_msgs::Int64 msg;
  while (n.ok())
  {
    msg.data = value;
    control_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}
