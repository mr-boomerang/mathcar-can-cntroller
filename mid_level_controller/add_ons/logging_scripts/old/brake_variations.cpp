// Copyright 2011-2018 The Mathworks, Inc.
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>

ros::Publisher speed_publisher;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_variation");
  ros::NodeHandle n;
  std::string time_val = argv[1];
  std::string mode = argv[2];
  double frequency = std::stod(time_val);
  double step_size_speed, speed_limit = 5 * 5 / 18;

  double speed = 0, steer = 15 * PI / 180, speed_max = 5.0;

  geometry_msgs::Twist message_speed;

  speed_publisher =
      n.advertise<geometry_msgs::Twist>("/velcitySteeringAngle", 1000);
  // to publish the control message
  ros::Rate loop_rate(frequency);
  double timer_counter = 0;
  double timer_total = 20;
  if (mode.compare("step") == 0 || mode.compare("ramp") == 0)
  {
    timer_total = 20;
  }
  step_size_speed = 0.278;
  while (n.ok())
  {
    if (mode.compare("sin") == 0)
    {
      speed = speed_max * sin(timer_counter * PI / timer_total);
    }
    if (mode.compare("step") == 0)
    {
      speed = speed_limit * timer_counter / timer_total;
    }
    if (mode.compare("ramp") == 0)
    {
      if (timer_counter <= timer_total / 2)
        speed = speed_limit * timer_counter / timer_total;
      else
        speed = speed_limit * (timer_total - timer_counter) / timer_total;
    }
    if (mode.compare("cycloidal") == 0)
    {
      speed = speed_max / 2 / PI / timer_total *
              (2 * PI * timer_counter -
               (timer_total)*sin(2 * timer_counter * PI / timer_total));
    }
    message_speed.linear.x = speed;
    message_speed.angular.z = steer;
    if (timer_counter > timer_total)
    {
      timer_counter = -1;
      if (mode.compare("sin") == 0 || mode.compare("cycloidal") == 0 ||
          (mode.compare("step") == 0 && speed_limit >= speed_max) ||
          (mode.compare("ramp") == 0 && speed_limit >= speed_max))
      {
        message_speed.linear.x = 0;
        message_speed.angular.z = steer;
        speed_publisher.publish(message_speed);
        exit(EXIT_SUCCESS);
      }
      if (mode.compare("step") == 0 || mode.compare("ramp") == 0)
      {
        printf("yaata\n");
        speed_limit = speed_limit + step_size_speed;
        printf("%f data\n", speed_limit);
      }
    }
    timer_counter = timer_counter + 1;
    if (message_speed.linear.x <= 0) message_speed.linear.x = 0;
    speed_publisher.publish(message_speed);
    loop_rate.sleep();
  }

  return 0;
}
