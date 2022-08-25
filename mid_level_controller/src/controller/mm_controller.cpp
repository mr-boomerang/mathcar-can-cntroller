// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file mm_controller.cpp
 * @brief the speed controller for car
 * @author Divyanshu Goel
 * @date 2017-03-29 (yyyy-mm-dd)
 **/
// controller for speed and steering over CAN
#include <controller/mm_controller.h>

#include <algorithm>
#include <string>
#include <vector>

// function to calc amount of pedal press value
int MmController::calc_pedal_press_value()
{
  PRINT_FUNC_ENTER;
  // control loop for acceleration
  error = setpoint_speed - speed;
  error_dt = (error - last_error) / dt;
  last_error = error;
  error_sum = error_sum + error;
  // equation for throttle calculation
  pedal_press = lookup_enable * C_1 + Kp * error + Ki * error_sum +
                Kd * error_dt + slope_enable * K_2 * sin(theta * M_PI / 180);
  // set upper bound on throttle
  if (pedal_press > throttle_limit)
  {
    pedal_press = throttle_limit;
  }
  if (debug_enable) printf("Throttle Value decided %f.\n", pedal_press);
  PRINT_FUNC_EXIT;
  return (pedal_press);
}
// function to calc amount of brake press value
int MmController::calc_brake_press_value()
{
  PRINT_FUNC_ENTER;
  // control loop for braking
  error_brake = fabs(setpoint_speed - speed);
  error_brake_dt = (error_brake - last_error_brake) / dt;
  last_error_brake = error_brake;
  error_brake_sum = error_brake_sum + error_brake;
  C_2 = get_brake_lookup();
  // Equation for brake calculation
  brake_press = lookup_enable * C_2 + Kp_brake * error_brake +
                Ki_brake * error_brake_sum + Kd_brake * error_brake_dt;
  // set higher and lower limits on brake output
  if (brake_press >= 90)
  {
    brake_press = 100;
  }
  if (brake_press <= 10)
  {
    brake_press = 0;
  }
  // print the final calulated brake value
  if (debug_enable) printf("Brake Value decided %f.\n", brake_press);
  PRINT_FUNC_EXIT;
  return (brake_press);
  //  return (input_brake_target0);
}
// function to calculate corresponding base pedal press value
double MmController::get_brake_lookup()
{
  PRINT_FUNC_ENTER;
  int floor_current_speed = 0, floor_target_speed = 0;
  int brake_percentage;
  // adjust values based on the input
  if (velocity_input_units.compare("mps") == 0)
  {
    // convert value to integers
    floor_current_speed = floor(speed * 18 / 5);
    floor_target_speed = floor(setpoint_speed * 18 / 5);
  }
  else
  {
    // convert value to integers
    floor_current_speed = floor(speed);
    floor_target_speed = floor(setpoint_speed);
  }
  // limits check
  if (floor_target_speed >= brake_value_count)
  {
    floor_target_speed = brake_value_count - 1;
  }
  if (floor_current_speed >= brake_value_count)
  {
    floor_current_speed = brake_value_count - 1;
  }
  // get corresponding braking lookup value
  brake_percentage =
      braking_values_lookup_orig[floor_target_speed][floor_current_speed];
  // decide on braking mode
  if (brake_mode.compare("alternate") == 0)
  {
    double diff = floor_current_speed - floor_target_speed;
    brake_percentage = abs(brake_multiplier * diff);
  }
  if (floor_target_speed == floor_current_speed)
  {
    brake_percentage = 45;
  }
  // limits on brake value
  if (setpoint_speed == 0 || brake_percentage >= brake_limit)
  {
    brake_percentage = brake_limit;
  }
  // print the final calulated brake value
  if (debug_enable)
  {
    printf("Brake Value decided %f.\n", brake_press);
    printf("Target speed decided %d.\n", floor_target_speed);
    printf("Current speed decided %d.\n", floor_current_speed);
  }
  // return the brake lookup
  PRINT_FUNC_EXIT;
  return (brake_percentage);
}
// function to calculate corresponding throttle press value
int MmController::polynomial_func(double input_velocity)
{
  PRINT_FUNC_ENTER;
  // Lookup table for velocity control
  int value = 0;
  // braking condition
  if (input_velocity <= 0)
  {
    value = -100;
  }
  else
  {
    // holds temporary value of braking
    int temp_out = 0;
    int target_velocity = floor(input_velocity);
    // calculate the braking value.
    // it is associated as given index contains velocity. this index needs to
    // be offset by 16 to get throttle value.
    for (int i = 0; i < throttle_value_count; i++)
    {
      if (throttle_values[i] == target_velocity)
      {
        temp_out = 16 + i;
        break;
      }
    }
    // copy the value
    value = temp_out;
  }
  // limit the speed to 21 kmph
  if (input_velocity >= 21)
  {
    value = 41;
  }
  // lower limits of throttle for slow speed that are not easy to control
  if (input_velocity < 5 && input_velocity > 0)
  {
    value = 16;
  }
  // return lookup value
  PRINT_FUNC_EXIT;
  return (value);
}
// function to update setpoint ros ind.
void MmController::update_setpoint(double inp_speed, double inp_steer_angle,
                                   double inp_brake_target)
{
  PRINT_FUNC_ENTER;
  // if steer locked allow only src with lock_id to update it

  // if speed locked allow only src with lock_id to update it

  // if steer and speed both locked allow only with lock_id to update it.

  // get the feedback
  if (cm0_.car_state_.auto_mode_count > 50)
  {
    inp_speed = 0;
    inp_steer_angle = 0;
    if (debug_enable)
    {
      printf("Stopping car due to mode switch\n");
    }
  }
  setpoint_speed = inp_speed;
  // used for brake logging of how much braking was to be applied
  input_brake_target0 = inp_brake_target;
  setpoint_steer = steer_direction_multiplier * inp_steer_angle;
  if (fabs(setpoint_speed) <= .05) setpoint_speed = 0;
  // print the setpoint information
  if (debug_enable)
    printf("(Speed, steer_angle) updated to (%0.3f %s,%0.3f %s).\n",
           setpoint_speed, velocity_input_units.c_str(), setpoint_steer,
           steer_input_units.c_str());
  if (setpoint_speed >= 0)
  {
    // change gear to reverse after taking feedback and checking current gear
    cm0_.SetGear("fwddrive");
  }
  if (setpoint_speed < 0)
  {
    // change gear to forward after taking feedback and checking current gear
    cm0_.SetGear("reverse");
  }
  if (setpoint_speed < 0)
  {
    setpoint_speed = -1 * setpoint_speed;
    speed_direction = -1;
  }
  else
  {
    speed_direction = -1;
  }
  // depending on the units adjust the speed input setpoint
  if (velocity_input_units.compare("mps") == 0)
  {
    if (setpoint_speed >= vel_limit) setpoint_speed = vel_limit;
    setpoint_speed = setpoint_speed * 18 / 5;
    // setpoint_speed = floor(setpoint_speed);
  }
  if (velocity_input_units.compare("kmph") == 0)
  {
    if (setpoint_speed >= vel_limit) setpoint_speed = vel_limit;
    setpoint_speed = setpoint_speed;
    // setpoint_speed = floor(setpoint_speed);
  }
  // depending on the units adjust the steer input setpoint
  if (steer_input_units.compare("degree") == 0)
  {
    if (setpoint_steer >= steer_limit) setpoint_steer = steer_limit;
    setpoint_steer = setpoint_steer;
    setpoint_steer = ceil(setpoint_steer);
  }
  if (steer_input_units.compare("radian") == 0)
  {
    if (setpoint_steer >= steer_limit) setpoint_steer = steer_limit;
    setpoint_steer = setpoint_steer * 180 / M_PI;
    setpoint_steer = ceil(setpoint_steer);
  }
  // get lookup value for the given speed
  C_1 = polynomial_func(setpoint_speed);
  // set variable to compute braking distance, watch dog timer
  watch_dog_time_planner_feedback = get_timestamp();
  braking_distance = 0;
  // take action on steering
  update_steer(setpoint_steer);
  // print to terminal the information needed for debugging
  if (debug_enable)
  {
    printf("trying to get to %f kmph\n", setpoint_speed);
    printf("Lookup value %f\n", C_1);
  }
  PRINT_FUNC_EXIT;
}
void MmController::update_steer(int setpoint_steer)
{
  PRINT_FUNC_ENTER;
  // check against steering value and publish the appropriate set of commands
  if (setpoint_steer >= 0)
  {
    cm0_.SendCmdToVehicle("Steering_Direction_Control", "left");
    cm0_.SendCmdToVehicle("Steering_Degree_Control", setpoint_steer);
  }
  else
  {
    cm0_.SendCmdToVehicle("Steering_Direction_Control", "right");
    cm0_.SendCmdToVehicle("Steering_Degree_Control", -1 * setpoint_steer);
  }
  PRINT_FUNC_EXIT;
}
void MmController::read_settings(const std::string settings_file_path)
{
  PRINT_FUNC_ENTER;
  // fetch settings from the cfg file
  // load parameters using utils
  if (!params_loaded) params_utils.load(settings_file_path);
  // store values in local variables
  // gains for speed controller pd loop
  params_utils.link("throttle_kp", &Kp);
  params_utils.link("throttle_ki", &Ki);
  params_utils.link("throttle_kd", &Kd);
  // gains for braking controller pd loop
  params_utils.link("brake_kp", &Kp_brake);
  params_utils.link("brake_ki", &Ki_brake);
  params_utils.link("brake_kd", &Kd_brake);
  // allows printing of debugging information
  debug_enable = params_utils.get_val<bool>("debug_msg_enable", false);
  // allows for setting of units for speed input
  velocity_input_units =
      params_utils.get_val<std::string>("velocity_input_units", "mps");
  // allows for setting of limit for speed input
  vel_limit = params_utils.get_val<double>("vel_limit", 5);
  // allows for setting of units for steer input
  steer_input_units =
      params_utils.get_val<std::string>("steer_input_units", "radian");
  // steer direction multiplier
  steer_direction_multiplier = 1;
  // allows for setting of limit for steer input
  steer_limit = params_utils.get_val<double>("steer_limit", 0.687);
  // allows for setting of limits for throttle.
  throttle_limit = params_utils.get_val<double>("throttle_limit", 60);
  // allows for setting of limits for braking.
  brake_limit = params_utils.get_val<double>("brake_limit", 88);
  // sets mode of operation of the controller
  std::string controller_type;
  controller_type =
      params_utils.get_val<std::string>("controller_type", "lookup");
  if (controller_type.compare("pid") == 0)
    lookup_enable = 0;
  else
    lookup_enable = 1;
  // sets whether slope input is to be used or not. the data linking has to be
  // done yet
  if (params_utils.get_val<bool>("slope_input_enabled", false))
    slope_enable = 1;
  else
    slope_enable = 0;
  // allows for setting of units for speed input
  slope_topic = params_utils.get_val<std::string>("slope_topic", "slope");
  // allows selection of source to use for controller operation
  controller_feedback_source = params_utils.get_val<std::string>(
      "controller_feedback_source", "gateway");
  // requirement to publish speed in a fixed time duration of 30 msec else
  // stop the code and get the feedback in 40 ms
  watch_dog_feedback_planner_enable =
      params_utils.get_val<bool>("watch_dog_planner_enable", true);
  watch_dog_feedback_speed_enable =
      params_utils.get_val<bool>("watch_dog_feedback_speed_enable", true);
  watch_dog_feedback_slope_enable =
      params_utils.get_val<bool>("watch_dog_feedback_slope_enable", false);
  // allows for update of PID online in ROS
  enable_update_pid_flag =
      params_utils.get_val<bool>("allow_pid_update", false);

#ifndef BUILD_WITHOUT_ROS
  if (enable_update_pid_flag)
  {
    params_utils.start_ros_service();
  }
#endif
  // display the data read if debug is enabled
  if (debug_enable || txrx.rx0.debug_enable || txrx.tx0.debug_enable)
  {
    printf("Data read from can settings file:\n");
    std::string lst = params_utils.list_full("\n");
    // params with vales and properties.
    printf("%s\n", lst.c_str());
    printf("*********************************************\n");
  }
  PRINT_FUNC_EXIT;
}
// initialize the state
void MmController::init(int argc, char** argv, const int* exit_flag_arg_)
{
  PRINT_FUNC_ENTER;
  // copy lookup tables
  throttle_value_count = throttle_value_count_orig;
  throttle_value_reverse_count = throttle_value_reverse_count_orig;
  throttle_value_boost_count = throttle_value_boost_count_orig;
  brake_value_count = brake_value_count_orig;

  throttle_values = new int[throttle_value_count];
  throttle_reverse_values = new int[throttle_value_reverse_count];
  throttle_boost_values = new int[throttle_value_boost_count];

  memcpy(throttle_values, &throttle_values_orig,
         throttle_value_count * sizeof(int));
  memcpy(throttle_reverse_values, &throttle_reverse_values_orig,
         throttle_value_reverse_count * sizeof(int));
  memcpy(throttle_boost_values, &throttle_boost_values_orig,
         throttle_value_boost_count * sizeof(int));
  // initialize the variables for the code
  error_sum = 0;
  last_error = 0;
  error = 0;
  setpoint_speed = 0;
  braking_distance = 0;
  watch_dog_time_planner_feedback = get_timestamp();
  watch_dog_time_speed_feedback = get_timestamp();
  watch_dog_time_slope_feedback = get_timestamp();
  exit_flag_pointer_ = exit_flag_arg_;
  txrx.init_parse_flags(argc, argv);
  txrx.init(txrx.control_yaml_file_path, txrx.status_yaml_file_path,
            txrx.log_file_path, txrx.settings_file_path, exit_flag_pointer_);

  if (*exit_flag_pointer_)
  {
    return;
  }
  params_loaded = true;
  init_success = true;
  read_settings(txrx.settings_file_path);
  cm0_.Init(exit_flag_pointer_);
  PRINT_FUNC_EXIT;
}
void MmController::start_controller_thd()
{
  PRINT_FUNC_ENTER;
  controllerIsThreaded = true;
  controller_thread0 = std::thread(&MmController::controller, this);
  if (!*exit_flag_pointer_)
    PN$ PRINT_SUCCESS << ct_grn("Controller Working") pendl;
  PRINT_FUNC_EXIT;
}
// function to run the controller
void MmController::controller()
{
  PRINT_FUNC_ENTER;
  // start transreceiver
  txrx.start_transreceiver();
#ifndef BUILD_WITHOUT_ROS
  setup_ros_connections();
#endif
  cm0_.ResetCar();
  if (!init_success)
  {
    PE$ ct_red(
        "CAN Transmitter. Error in intilization. Please Ensure proper "
        "initialization.") pendl;
    //  return;
  }
  else
  {
    printf("Velocity input max value: %0.2f %s\n", vel_limit,
           velocity_input_units.c_str());
    printf("Steer input max value: %0.2f %s\n", steer_limit,
           steer_input_units.c_str());
  }
  //  base timing
  int brake_handle = 0;
  watch_dog_time_speed_feedback = get_timestamp();
  watch_dog_time_planner_feedback = get_timestamp();
  watch_dog_time_slope_feedback = get_timestamp();
  while (!*exit_flag_pointer_)
  {
    auto time_start = std::chrono::high_resolution_clock::now();
    if (watch_dog_feedback_speed_enable)
    {
      watch_dog_time_now = get_timestamp();
      watch_dog_time_diff =
          (watch_dog_time_now - watch_dog_time_speed_feedback) / 1000L;
      if (watch_dog_time_diff >= 40)
      {
        cm0_.SendCmdToVehicle("Driving_Gear_Control", "neutral");
        cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 88);
        PE$ ct_red(
            "Please restart CAR. No Incoming speed data. Stopping "
            "vehicle. Repeating Message.") pendl;
      }
    }
    if (watch_dog_feedback_planner_enable)
    {
      watch_dog_time_now = get_timestamp();
      watch_dog_time_diff =
          (watch_dog_time_now - watch_dog_time_planner_feedback) / 1000L;
      if (watch_dog_time_diff >= 100)
      {
        cm0_.SendCmdToVehicle("Driving_Gear_Control", "neutral");
        cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 88);
        PE$ ct_red(
            "Please run Planner module no input received. Either Disable "
            "the watch dog flag for planner and then run or ensure that "
            "speed command is sent within 100 mSecs. Stopping vehicle. "
            "Repeating Message.") pendl;
      }
    }
    if (watch_dog_feedback_slope_enable)
    {
      watch_dog_time_now = get_timestamp();
      watch_dog_time_diff =
          (watch_dog_time_now - watch_dog_time_slope_feedback) / 1000L;
      if (watch_dog_time_diff >= 100)
      {
        cm0_.SendCmdToVehicle("Driving_Gear_Control", "neutral");
        cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 88);
        PE$ ct_red(
            "Please run Slope module no input received. Either Disable "
            "the watch dog flag for slope and then run or ensure that "
            "slope feedback is sent within 100 mSecs. Stopping vehicle. "
            "Repeating Message.") pendl;
      }
    }
    if (cm0_.car_state_.auto_mode_count > 50)
    {
      update_setpoint(0, 0, 100);
    }
    speed = cm0_.car_state_.speed_fdbk;
    // get the throttle value to be commanded
    accln_commanded = calc_pedal_press_value();
    if (accln_commanded > 0)
    {
      accln_commanded = ceil(accln_commanded);
      cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 0);
      cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", accln_commanded);
      if (raw_msg_enable)
        printf("Updating Accln to %i and braking to 0\n", accln_commanded);
    }
    else
    {
      // braking part
      if (debug_enable) printf("-ve accln %i\n", accln_commanded);

      brake_handle = calc_brake_press_value();
      cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", 0);
      // limit of braking and in case of zero setpoint
      if (setpoint_speed == 0 || brake_handle >= brake_limit)
        brake_handle = brake_limit;
      cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", brake_handle);

      if (raw_msg_enable)
        printf("Updating Accln to 0 and braking to %i\n", brake_handle);
    }
#ifndef BUILD_WITHOUT_ROS
    if (!controllerIsThreaded && !*exit_flag_pointer_) ros::spinOnce();
#endif
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed_time =
        time_end - time_start;
    usleep(10000 - static_cast<int>(elapsed_time.count()));
  }
  signalHandler(1);
  PRINT_FUNC_EXIT;
}
// Factor in slope
void MmController::slope_update(double slope_ang)
{
  PRINT_FUNC_ENTER;
  // update slope
  theta = slope_ang;
  watch_dog_time_slope_feedback = get_timestamp();
  PRINT_FUNC_EXIT;
}
#ifndef BUILD_WITHOUT_ROS
// function to calc amount of brake press value
void MmController::slope_update_ros(const std_msgs::Float64::ConstPtr& msg)
{
  PRINT_FUNC_ENTER;
  slope_update(msg->data);
  PRINT_FUNC_EXIT;
}
// function to update setpoint
void MmController::update_setpoint(const geometry_msgs::Twist::ConstPtr& msg)
{
  PRINT_FUNC_ENTER;
  // call the update function
  if (!(std::isnan(msg->linear.x)) && !(std::isnan(msg->linear.y)) &&
      !(std::isnan(msg->linear.z)) && !(std::isnan(msg->angular.x)) &&
      !(std::isnan(msg->angular.y)) && !(std::isnan(msg->angular.z)) &&
      !(std::isinf(msg->linear.x)) && !(std::isinf(msg->linear.y)) &&
      !(std::isinf(msg->linear.z)) && !(std::isinf(msg->angular.x)) &&
      !(std::isinf(msg->angular.y)) && !(std::isinf(msg->angular.z)))
    update_setpoint(msg->linear.x, msg->angular.z, msg->linear.z);
  PRINT_FUNC_EXIT;
}
// seup ros connections
void MmController::setup_ros_connections()
{
  PRINT_FUNC_ENTER;
  // create a ros handle
  ros::NodeHandle n;
  // to get commanding value
  input_sub = n.subscribe("velocitySteeringAngle", 10,
                          &MmController::update_setpoint, this);
  PRINT_FUNC_EXIT;
}
#endif
// clean exit
void MmController::controller_exit(int signum)
{
  PRINT_FUNC_ENTER;
  // for clean exit
  cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", 0);
  cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 88);
  cm0_.SetGear("neutral");
  txrx.signalHandler(signum);
  delete[] throttle_reverse_values;
  delete[] throttle_boost_values;
  delete[] throttle_values;

#ifndef BUILD_WITHOUT_ROS
  ros::shutdown();
#endif
  PRINT_FUNC_EXIT;
}
void MmController::signalHandler(int signum)
{
  PRINT_FUNC_ENTER;
  PI$ "Closing controller instance." pendl;
  controller_exit(signum);
  PRINT_FUNC_EXIT;
}
MmController::~MmController()
{
  PRINT_FUNC_ENTER;
  // closing other things
  if (controller_thread0.joinable()) controller_thread0.join();
  PRINT_FUNC_EXIT;
}
