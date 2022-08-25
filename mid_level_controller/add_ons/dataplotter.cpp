// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file dataplotter.cpp
 * @brief plots the data online.
 * @author Vivek Jha
 * @date 02/06/2016
 **/

// #include <add_ons/dataplotter.hpp>
#include <csignal>
#include <iostream>
#ifdef PANGOLIN_FOUND
#include <chrono>  // NOLINT
#include <string>
#include <vector>

#include <pangolin/pangolin.h>
#include <utils/gui/data_plotter_object.hpp>
#include <utils/gui/graph_objects.hpp>
#include <utils/gui/objects_2d.hpp>
#include <utils/gui/objects_3d.hpp>
#include <utils/gui/view_graph.hpp>
#include <utils/str_utils.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

// Message headers
#include <mid_level_controller/Callback_Msg.h>
#include <mid_level_controller/Can_Status_Message.h>
// Synchroniser headers
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#define PAP pangolin::Attach::Pix
#define PAF pangolin::Attach::Frac
#define frequency 100
#define resolution .0081

// Height and Width of Plotter window
#define WIN_W 640
#define WIN_H 480
#define todegree 180.0 / M_PI

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

static double get_timestamp()
{
  auto start = std::chrono::high_resolution_clock::now();
  return (start.time_since_epoch().count() / 1000);
};

/// for clean exit of code
int exit_flag_dataplotter = 0;

/// signal handler to do a clean exit
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by dataplotter.\n", signum);
  // terminate program
  exit_flag_dataplotter = 1;
  // ros::shutdown();
  // teleop_car.exit_flag = 1;
  // teleop_car.signalHandler(signum);
  // exit(signum);
};
// Variables to fetch data into
double l_tick_count = 0, l_speed = 0, l_speed_pre = 0, lr_speed = 0,
       lr_speed_pre = 0, del_lr_speed = 0, Err_lr_speed = 0,
       Err_lr_speed_pre = 0, r_tick_count = 0, r_speed = 0, r_speed_pre = 0;
double cmd_vel_target_velocity = 0, cmd_target_velocity_pre = 0,
       cmd_vel_target_steer;
std::string gateway_velocity;
double gateway_velocity_fl = 0, del_gateway_velocity = 0,
       gateway_velocity_pre = 0, Err_gateway_velocity = 0,
       Err_gateway_velocity_pre = 0;
double gps_latitude, gps_latitude_pre, gps_latitude_del, gps_longitude,
    gps_longitude_pre, gps_longitude_del, gps_covariance_1, gps_covariance_2,
    gps_covariance_3;
double IMU_orient_x, IMU_orient_y, IMU_orient_z, IMU_orient_w, IMU_angular_x,
    IMU_angular_y, IMU_angular_z, IMU_linear_x, IMU_linear_y, IMU_linear_z,
    IMU_roll, IMU_pitch, IMU_yaw;
double IMU_orient_x2, IMU_orient_y2, IMU_orient_z2, IMU_orient_w2, IMU_yaw2;
std::string mode, mode_check[10];
std::string steering_angle, steering_direction;
double steering_angle_fl;
int steering_direction_multiplier = 1;

bool start_measuring_pid_parameters = true;
double start_time_pid_rise_time_measurement = get_timestamp();
double end_time_pid_rise_time_measurement = get_timestamp();
double start_time_pid_settling_time_measurement = get_timestamp();
double end_time_pid_settling_time_measurement = get_timestamp();
double start_settling_values_array[] = {-10, -10, -10, -10, -10,   // NOLINT
                                        -10, -10, -10, -10, -10};  // NOLINT
double diff_settling[] = {-10, -10, -10, -10, -10, -10, -10, -10, -10, -10};
int settling_value_array_counter = 0;
int settling_value_array_counter_max = 10;
double setpoint = 0;
double max_value_overshoot = -32000, min_value_overshoot = 32000,
       last_commanded_velocity = 0, overshoot_percentage_final = 0;
double rise_time = 0, instance_count = 0;
double settling_percentage = 5;
double settling_time = -1;
double overshoot_percentage = -1;
bool first_instance_update = false, rise_time_calc_flag = false;

std::string braking_percentage, accln_percentage, braking_ctrl_prcntg,
    accln_ctrl_prcntg;
double braking_percentage_fl, accln_percentage_fl;
double Odom_theta = 0, Odom_x = 0, Odom_y = 0, Odom_z = 0, Odom_w = 0,
       Odom_temp = 0;

int i = 0, mode_sum = 0;
bool PLOT = true;

std::ofstream myPlotterFile;

void cmdVel_callback(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_vel_target_velocity = msg->linear.x;
  cmd_vel_target_steer = msg->angular.z * 180 / 3.1415 * -1;
  last_commanded_velocity = setpoint;
  setpoint = cmd_vel_target_velocity;
  start_measuring_pid_parameters = true;
  start_time_pid_rise_time_measurement = get_timestamp();
  end_time_pid_rise_time_measurement = get_timestamp();
  start_time_pid_settling_time_measurement = get_timestamp();
  end_time_pid_settling_time_measurement = get_timestamp();
  for (int count = 0; count < settling_value_array_counter_max; count++)
  {
    start_settling_values_array[count] = -10;
    diff_settling[count] = -10;
  }
  settling_value_array_counter = 0;
  max_value_overshoot = -32000;
  min_value_overshoot = 32000;
  rise_time = 0;
  instance_count = 0;
  settling_percentage = 10;
  first_instance_update = false;
  rise_time_calc_flag = true;
  overshoot_percentage_final = overshoot_percentage;
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  gps_latitude = msg->latitude;
  gps_longitude = msg->longitude;
  gps_covariance_1 = msg->position_covariance[0];
  gps_covariance_2 = msg->position_covariance[4];
  gps_covariance_3 = msg->position_covariance[8];
}

void IMU_callback(const sensor_msgs::ImuConstPtr& msg)
{
  IMU_orient_x = msg->orientation.x;
  IMU_orient_y = msg->orientation.y;
  IMU_orient_z = msg->orientation.z;
  IMU_orient_w = msg->orientation.w;
  IMU_angular_x = msg->angular_velocity.x;
  IMU_angular_y = msg->angular_velocity.y;
  IMU_angular_z = msg->angular_velocity.z;
  IMU_linear_x = msg->linear_acceleration.x;
  IMU_linear_y = msg->linear_acceleration.y;
  IMU_linear_z = msg->linear_acceleration.z;

  IMU_roll =
      atan2(2.0 * (IMU_orient_y * IMU_orient_z + IMU_orient_w * IMU_orient_x),
            IMU_orient_w * IMU_orient_w - IMU_orient_x * IMU_orient_x -
                IMU_orient_y * IMU_orient_y + IMU_orient_z * IMU_orient_z);
  IMU_pitch =
      asin(-2.0 * (IMU_orient_x * IMU_orient_z - IMU_orient_w * IMU_orient_y));
  IMU_yaw =
      atan2(2.0 * (IMU_orient_x * IMU_orient_y + IMU_orient_w * IMU_orient_z),
            IMU_orient_w * IMU_orient_w + IMU_orient_x * IMU_orient_x -
                IMU_orient_y * IMU_orient_y - IMU_orient_z * IMU_orient_z);
  IMU_roll *= todegree;
  IMU_pitch *= todegree;
  IMU_yaw *= todegree;

  //  myPlotterFile <<IMU_orient_x << ", " << IMU_orient_y << ", " <<
  //  IMU_orient_z << ", " << IMU_orient_w <<", " << IMU_yaw <<std::endl;
}

void IMU_odom_sync(const sensor_msgs::ImuConstPtr& msg1,
                   const nav_msgs::OdometryConstPtr& msg2)
{
  /*  IMU_orient_x2 = msg1->orientation.x;
    IMU_orient_y2 = msg1->orientation.y;
    IMU_orient_z2 = msg1->orientation.z;
    IMU_orient_w2 = msg1->orientation.w;

    Odom_x = msg2->pose.pose.orientation.x;
    Odom_y = msg2->pose.pose.orientation.y;
    Odom_z = msg2->pose.pose.orientation.z;
    Odom_w = msg2->pose.pose.orientation.w;

    Odom_temp = msg2->twist.twist.angular.z;
    IMU_yaw2 =
      atan2(2.0 * (IMU_orient_x2 * IMU_orient_y2 + IMU_orient_w2 *
    IMU_orient_z2), IMU_orient_w2 * IMU_orient_w2 + IMU_orient_x2 *
    IMU_orient_x2 - IMU_orient_y2 * IMU_orient_y2 - IMU_orient_z2 *
    IMU_orient_z2);

    Odom_theta =
      atan2(2.0 * (Odom_x * Odom_y + Odom_w * Odom_z),
        Odom_w * Odom_w + Odom_x * Odom_x -
          Odom_y * Odom_y - Odom_z * Odom_z);

    IMU_yaw2 *= todegree;
    Odom_theta *= todegree;
    Odom_temp *= todegree;
    std::cout << "IMU_Yaw2 is: " << IMU_yaw2 << "Odom_theta is: " << Odom_theta
    <<std::endl;*/
}

void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
  Odom_x = msg->twist.twist.angular.x;
  Odom_y = msg->twist.twist.angular.y;
  Odom_z = msg->twist.twist.angular.z;

  Odom_x = msg->pose.pose.orientation.x;
  Odom_y = msg->pose.pose.orientation.y;
  Odom_z = msg->pose.pose.orientation.z;
  Odom_w = msg->pose.pose.orientation.w;

  Odom_temp = msg->twist.twist.angular.z;

  Odom_theta = atan2(
      2.0 * (Odom_x * Odom_y + Odom_w * Odom_z),
      Odom_w * Odom_w + Odom_x * Odom_x - Odom_y * Odom_y - Odom_z * Odom_z);

  Odom_theta *= todegree;
  Odom_temp *= todegree;
}

void CANstatus_callback(
    const mid_level_controller::Can_Status_Message::ConstPtr& msg)
{
  mode = msg->value[2];

  steering_angle = msg->value[7];
  if (steering_angle.compare("None") == 0 || steering_angle.compare("off") == 0)
    steering_angle = "0";
  steering_angle_fl = str_to_val<float>(steering_angle);
  steering_direction = msg->value[8];
  //    std::cout<<"Steering direction is: "<<steering_direction<<std::endl;
  if (steering_direction == "right")
    steering_direction_multiplier = 1;
  else if (steering_direction == "left")
    steering_direction_multiplier = -1;
  else if (steering_direction == "None")
    steering_direction_multiplier = steering_direction_multiplier;

  steering_angle_fl = steering_angle_fl * steering_direction_multiplier;

  gateway_velocity = msg->value[14];
  gateway_velocity_fl = str_to_val<float>(gateway_velocity) * 5 / 18;
  double gateway_velocity_db = static_cast<double>(gateway_velocity_fl);

  braking_percentage = msg->value[10];
  accln_percentage = msg->value[11];
  braking_percentage_fl = str_to_val<float>(braking_percentage);
  accln_percentage_fl = str_to_val<float>(accln_percentage);

  mode_check[i] = mode;
  i++;
  i = i % 10;
  for (int j = 0; j < 10; j++)
  {
    if (mode_check[j] == "off") mode_sum += 1;
  }

  if (mode_sum == 10)
    PLOT = false;
  else
    PLOT = true;
  mode_sum = 0;
  double speed = gateway_velocity_db;
  if (start_measuring_pid_parameters = true)
  {
    // assumes a 10 sec cap on calculation
    // printf("1 speed: %f setpoint: %f instance_count %f last_speed: %f\n",
    // speed,
    //        setpoint, instance_count, last_commanded_velocity);
    // printf("1 max overshoot: %f overshoot: %f min_overshoot: %f \n",
    //        max_value_overshoot, overshoot_percentage, min_value_overshoot);
    if (speed >= max_value_overshoot && last_commanded_velocity <= setpoint &&
        instance_count <= 1000)
    {
      double diff = fabs(speed - setpoint);
      max_value_overshoot = gateway_velocity_db;
      overshoot_percentage = diff / setpoint * 100;
      if (speed == 0 && setpoint != 0) overshoot_percentage = 0;
    }
    if (speed <= min_value_overshoot && last_commanded_velocity > setpoint &&
        instance_count <= 1000)
    {
      double diff = fabs(speed - setpoint);
      overshoot_percentage = diff / setpoint * 100;
      min_value_overshoot = gateway_velocity_db;
      if (speed == 0 && setpoint != 0) overshoot_percentage = 0;
    }
    if (speed > 0.9 * setpoint && rise_time_calc_flag == true &&
        last_commanded_velocity < setpoint)
    {
      end_time_pid_rise_time_measurement = get_timestamp();
      rise_time = (end_time_pid_rise_time_measurement -
                   start_time_pid_rise_time_measurement) /
                  1000L;
      rise_time_calc_flag = false;
    }
    if (speed < 1.1 * setpoint && rise_time_calc_flag == true &&
        last_commanded_velocity > setpoint)
    {
      end_time_pid_rise_time_measurement = get_timestamp();
      rise_time = (end_time_pid_rise_time_measurement -
                   start_time_pid_rise_time_measurement) /
                  1000L;
      rise_time_calc_flag = false;
    }
    start_settling_values_array[settling_value_array_counter] = speed;
    settling_value_array_counter = (settling_value_array_counter + 1) % 10;
    bool values_settled = true;
    for (int count = 0; count < settling_value_array_counter_max; count++)
    {
      diff_settling[count] =
          fabs(start_settling_values_array[count] - setpoint) * 100 / setpoint;
      if (diff_settling[count] > settling_percentage)
      {
        values_settled = false;
      }
    }
    if (values_settled == true && first_instance_update == false)
    {
      end_time_pid_settling_time_measurement = get_timestamp();
      settling_time = (end_time_pid_settling_time_measurement -
                       start_time_pid_settling_time_measurement) /
                      1000L;
      first_instance_update = true;
    }
  }
  instance_count = instance_count + 1;
  if (instance_count > 10000) instance_count = 10000;
  // at this point the value are present in
  // overshoot_percentage,rise_time,settling_time in ms they just need to be
  // plotted

  //    std::cout << "Mode Sum is: " << mode_sum << " PLOT is: " <<PLOT <<
  //    "\t";//<< std::endl; std::cout << "Mode is: " << mode << std::endl;
}

void CANcontrol_callback(
    const mid_level_controller::Callback_Msg::ConstPtr& msg)
{
  if (msg->topic_name.compare("Accln_Control_Percntg_Control") == 0)
    accln_ctrl_prcntg = msg->message_data;
  // std::cout << "Accln Control percentage is: " << accln_ctrl_prcntg
  //           << std::endl;

  if (msg->topic_name.compare("Braking_Control_Percntg_Control") == 0)
    braking_ctrl_prcntg = msg->message_data;
  // std::cout << "Braking Control percentage is: " << braking_ctrl_prcntg
  //           << std::endl;
}
int main(int argc, char** argv)
{
  std::vector<std::string> labels_MP1, labels_MP2, labels_SP1,     // NOLINT
      labels_SP2, labels_SP3, labels_SP4, labels_SP5, labels_SP6,  // NOLINT
      labels_SP7, labels_SP8;                                      // NOLINT

  print::init(&argc, &argv, true);

  // Initialize ROS
  ros::init(argc, argv, "plotter_utils", ros::init_options::NoSigintHandler);

  // Create nodehandle
  ros::NodeHandle n;
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);

  // Initialize graph window
  Window.init("Data Plotter", WIN_W, WIN_H, &argc, &argv);

  myPlotterFile.open("PlotterData.txt", std::ios::out);

  // Define Containers/blocks and their span
  Window.add_container("Main", PAF(0.0f), PAF(2 / 3.0f), PAF(0.0f), PAF(1.0f),
                       pangolin::LayoutEqualHorizontal, "");
  Window.add_container("Secondary", PAF(2 / 3.0f), PAF(1.0f), PAF(0.0f),
                       PAF(1.0f), pangolin::LayoutEqualHorizontal, "");

  // Primary objects (Plots)
  Window.add_2d_view("Velocities", "Main");  // M1
  Window.add_2d_view("PID Params", "Main");  // M1
  // Secondary Objects (Plots)
  Window.add_2d_view("Errors", "Secondary");             // S1
  Window.add_2d_view("Del_Errors", "Secondary");         // S2
  Window.add_2d_view("Brake_Accln%", "Secondary");       // S3
  Window.add_2d_view("Steering_Angle", "Secondary");     // S4
  Window.add_2d_view("IMU_data", "Secondary");           // S5
  Window.add_2d_view("GPS_Lat_Long_Diff", "Secondary");  // S6
  Window.add_2d_view("GPS_Covariance", "Secondary");     // S7
  Window.add_2d_view("Odom_Test", "Secondary");          // SP8

  //  ros::Subscriber sub_Wheel_l = n.subscribe("/wheel_encoder/wheel_l", 5,
  //  wheel_l_callback); ros::Subscriber sub_wheel_r =
  //  n.subscribe("/wheel_encoder/wheel_r", 5, wheel_r_callback);
  ros::Subscriber sub_cmdVel =
      n.subscribe("/velocitySteeringAngle", 5, cmdVel_callback);
  ros::Subscriber sub_GPS = n.subscribe("/gps/fix", 5, GPS_callback);
  ros::Subscriber sub_IMU = n.subscribe("/imu/data", 5, IMU_callback);
  ros::Subscriber sub_CANstatus =
      n.subscribe("/canStatus", 5, CANstatus_callback);
  ros::Subscriber sub_CANcontrol =
      n.subscribe("/canControl", 5, CANcontrol_callback);
  ros::Subscriber sub_odom =
      n.subscribe("/odometry/wheel_encoder", 5, odom_callback);

  labels_MP1.push_back("Command Vel(mps)");
  labels_MP1.push_back("Gateway Vel(mps)");
  labels_MP1.push_back("Encoder Vel(mps)");

  labels_MP2.push_back("Rise time(ms)");
  labels_MP2.push_back("Overshoot percentage(For each cmd velocity)");
  labels_MP2.push_back("Settling Time(ms)");

  labels_SP1.push_back("Err GW_Vel");
  labels_SP1.push_back("Err ENC_Vel");

  labels_SP2.push_back("Del Err GW_Vel");
  labels_SP2.push_back("Del Err ENC_Vel");

  labels_SP3.push_back("Braking %");
  labels_SP3.push_back("Accln %");

  labels_SP4.push_back("Steering Angle GW");
  labels_SP4.push_back("Steering Angle WE");

  labels_SP5.push_back("Roll");
  labels_SP5.push_back("Pitch");
  labels_SP5.push_back("Yaw");

  labels_SP6.push_back("GPS Latitude Diff");
  labels_SP6.push_back("GPS Longitutde Diff");

  labels_SP7.push_back("GPS Covariance X");
  labels_SP7.push_back("GPS Covariance Y");
  labels_SP7.push_back("GPS Covariance Z");

  labels_SP8.push_back("Odom_theta");
  // labels_SP8.push_back("Odom_temp");
  labels_SP8.push_back("Yaw");

  DataPlotterObject Obj2d_MP1(labels_MP1), Obj2d_MP2(labels_MP2),
      Obj2d_SP1(labels_SP1), Obj2d_SP2(labels_SP2), Obj2d_SP3(labels_SP3),
      Obj2d_SP4(labels_SP4), Obj2d_SP5(labels_SP5), Obj2d_SP6(labels_SP6),
      Obj2d_SP7(labels_SP7), Obj2d_SP8(labels_SP8);

  Viewnode("Velocities")->add_object("Plot_MP1", &Obj2d_MP1);
  Viewnode("PID Params")->add_object("Plot_MP2", &Obj2d_MP2);
  Viewnode("Errors")->add_object("Plot_SP1", &Obj2d_SP1);
  Viewnode("Del_Errors")->add_object("Plot_SP2", &Obj2d_SP2);
  Viewnode("Brake_Accln%")->add_object("Plot_SP3", &Obj2d_SP3);
  Viewnode("Steering_Angle")->add_object("Plot_SP4", &Obj2d_SP4);
  Viewnode("IMU_data")->add_object("Plot_SP5", &Obj2d_SP5);
  Viewnode("GPS_Lat_Long_Diff")->add_object("Plot_SP6", &Obj2d_SP6);
  Viewnode("GPS_Covariance")->add_object("Plot_SP7", &Obj2d_SP7);
  Viewnode("Odom_Test")->add_object("Plot_SP8", &Obj2d_SP8);

  pangolin::RegisterKeyPressCallback('s', []() {               // NOLINT
    pangolin::DisplayBase().SaveOnRender("Data_Plotter.jpg");  // NOLINT
  });                                                          // NOLINT
  pangolin::RegisterKeyPressCallback('r', []() {               // NOLINT
    pangolin::DisplayBase().RecordOnRender(                    // NOLINT
        "ffmpeg:[unique_filename]//Data_Plotter.avi");         // NOLINT
  });                                                          // NOLINT

  while (!pangolin::ShouldQuit() && ros::ok() && !exit_flag_dataplotter)
  {
    ros::spinOnce();

    std::vector<float> msgM1, msgM1_pre, msgM2, msgM2_pre, msgS1,  // NOLINT
        msgS1_pre, msgS2, msgS2_pre, msgS3, msgS3_pre, msgS4,      // NOLINT
        msgS4_pre, msgS5, msgS5_pre, msgS6, msgS6_pre, msgS7,      // NOLINT
        msgS7_pre, msgS8, msgS8_pre;                               // NOLINT

    lr_speed = (l_speed + r_speed) / 2;
    msgM1.push_back(cmd_vel_target_velocity);
    msgM1.push_back(gateway_velocity_fl);
    msgM1.push_back(lr_speed);
    // plot data for pid params
    msgM2.push_back(rise_time);
    msgM2.push_back(overshoot_percentage_final);
    msgM2.push_back(settling_time);

    Err_lr_speed = lr_speed - lr_speed_pre;
    Err_gateway_velocity = gateway_velocity_fl - gateway_velocity_pre;
    msgS1.push_back(Err_lr_speed);
    msgS1.push_back(Err_gateway_velocity);

    del_lr_speed = Err_lr_speed - Err_lr_speed_pre;
    del_gateway_velocity = Err_gateway_velocity - Err_gateway_velocity_pre;
    msgS2.push_back(del_lr_speed);
    msgS2.push_back(del_gateway_velocity);

    msgS3.push_back(braking_percentage_fl);
    msgS3.push_back(accln_percentage_fl);

    msgS4.push_back((steering_angle_fl));
    msgS4.push_back((Odom_temp));
    //        std::cout<<"Steering angle is: "<< steering_angle <<std::endl;

    msgS5.push_back(IMU_roll);
    msgS5.push_back(IMU_pitch);
    msgS5.push_back(IMU_yaw);

    gps_latitude_del = gps_latitude - gps_latitude_pre;
    gps_longitude_del = gps_longitude - gps_longitude_pre;

    msgS6.push_back(gps_latitude_del);
    msgS6.push_back(gps_longitude_del);

    msgS7.push_back(gps_covariance_1);
    msgS7.push_back(gps_covariance_2);
    msgS7.push_back(gps_covariance_3);

    message_filters::Subscriber<sensor_msgs::Imu> IMU_sub(n, "/imu/data", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "/odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
                                                            nav_msgs::Odometry>
        MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), IMU_sub,
                                                     odom_sub);
    sync.registerCallback(boost::bind(&IMU_odom_sync, _1, _2));

    msgS8.push_back(Odom_theta);
    // msgS8.push_back(Odom_temp);
    msgS8.push_back(IMU_yaw);
    // std::cout << Odom_theta << " , " << IMU_yaw << " , "
    //           << std::abs(Odom_theta) - std::abs(IMU_yaw) << std::endl;
    myPlotterFile << Odom_theta << " , " << IMU_yaw << " , "
                  << std::abs(Odom_theta) - std::abs(IMU_yaw) << std::endl;

    if (PLOT == true)
    {
      Obj2d_MP1.update(msgM1);
      Obj2d_MP2.update(msgM2);
      Obj2d_SP1.update(msgS1);
      Obj2d_SP2.update(msgS2);
      Obj2d_SP3.update(msgS3);
      Obj2d_SP4.update(msgS4);
      Obj2d_SP5.update(msgS5);
      Obj2d_SP6.update(msgS6);
      Obj2d_SP7.update(msgS7);
      Obj2d_SP8.update(msgS8);
    }
    else
    {
      Obj2d_MP1.update(msgM1_pre);
      Obj2d_MP2.update(msgM2_pre);
      Obj2d_SP1.update(msgS1_pre);
      Obj2d_SP2.update(msgS2_pre);
      Obj2d_SP3.update(msgS3_pre);
      Obj2d_SP4.update(msgS4_pre);
      Obj2d_SP5.update(msgS5_pre);
      Obj2d_SP6.update(msgS6_pre);
      Obj2d_SP7.update(msgS7_pre);
      Obj2d_SP8.update(msgS8_pre);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();

    msgM1_pre = msgM1;
    msgM2_pre = msgM2;
    msgS1_pre = msgS1;
    msgS2_pre = msgS2;
    msgS3_pre = msgS3;
    msgS4_pre = msgS4;
    msgS5_pre = msgS5;
    msgS6_pre = msgS6;
    msgS7_pre = msgS7;
    msgS8_pre = msgS8;

    lr_speed_pre = lr_speed;
    gateway_velocity_pre = gateway_velocity_fl;

    Err_lr_speed_pre = Err_lr_speed;
    Err_gateway_velocity_pre = Err_gateway_velocity;

    gps_latitude_pre = gps_latitude;
    gps_longitude_pre = gps_longitude;
  }
  ros::shutdown();

  return 0;
}
#else
/// for clean exit of code
int exit_flag_dataplotter = 0;

/// signal handler to do a clean exit
void mysignalHandler(int signum)
{
  printf("Interrupt signal (%i) received by dataplotter.\n", signum);
  // terminate program
  exit_flag_dataplotter = 1;
  // ros::shutdown();
};

int main(int argc, char** argv)
{
  /// create handles for sigtermination
  signal(SIGINT, mysignalHandler);
  signal(SIGTERM, mysignalHandler);
  // Print status message.
  printf("Pangolin Not Found.\n");
  while (!exit_flag_dataplotter)
  {
  }
  return 0;
}
#endif
