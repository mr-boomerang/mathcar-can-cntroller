// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file redgear.cpp
 * @brief controls for redgear joystick
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#include <interfaces/peripherals/redgear.h>

#include <algorithm>  // for transform
#include <string>     /// for handling strings

/// to execute the hard brake press option of the particular joystick
void Redgear::ExecuteHardBrakePress()
{
  PRINT_FUNC_ENTER;
  cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
  cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 100);

  if (cm0_.GetDebugOption())
  {
    dirty_ = true;
    control_pressed_ = "Hard Brake Pressed";
    printf("Action: %s\n", control_pressed_.c_str());
  }
  car_state_.throttle_cmd = 0;
  car_state_.velocity_target = 0;
  car_state_.brake_cmd = 100;
  PRINT_FUNC_EXIT;
}
/// to execute the hard brake release option of the particular
/// joystick
void Redgear::ExecuteHardBrakeRelease()
{
  PRINT_FUNC_ENTER;
  cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
  cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", "0");
  if (cm0_.GetDebugOption())
  {
    control_pressed_ = "Hard Brake Release";
    dirty_ = true;
    printf("Action: %s\n", control_pressed_.c_str());
  }
  car_state_.throttle_cmd = 0;
  car_state_.velocity_target = 0;
  car_state_.brake_cmd = 0;
  PRINT_FUNC_EXIT;
}
/// to execute the Indicators of the particular joystick
void Redgear::ExecuteIndicators(int type)
{
  PRINT_FUNC_ENTER;
  // execute different types of indicators
  // 0 -> off
  // 1 -> left
  // 2 -> right
  // 3 -> hazard
  if (type == 0)
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "off");
    control_pressed_ = "Indicators off";
  }
  if (type == 1)
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "left");
    control_pressed_ = "Left Indicator";
  }
  if (type == 2)
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "right");
    control_pressed_ = "Right Indicator";
  }
  if (type == 3)
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "hazard");
    control_pressed_ = "Hazard Indicator";
  }

  if (cm0_.GetDebugOption())
  {
    dirty_ = true;
    printf("Action: %s\n", control_pressed_.c_str());
  }
  car_state_.indicator_cmd = control_pressed_;
  PRINT_FUNC_EXIT;
}
/// to execute the Gear Up by the particular joystick
void Redgear::ExecuteGearUp()
{
  PRINT_FUNC_ENTER;
  gear_count_ = (gear_count_ + 1) % 4;
  if (gear_count_ > 3) gear_count_ = 3;
  if (gear_count_ == 0) ExecuteSetGear("reverse");
  if (gear_count_ == 1) ExecuteSetGear("neutral");
  if (gear_count_ == 2) ExecuteSetGear("fwddrive");
  if (gear_count_ == 3) ExecuteSetGear("boost");

  if (cm0_.GetDebugOption())
  {
    control_pressed_ = "Gear Shift Up";
    printf("Action: %s\n", control_pressed_.c_str());
    dirty_ = true;
  }
  PRINT_FUNC_EXIT;
}
/// to execute the Gear Down by the particular joystick
void Redgear::ExecuteGearDown()
{
  PRINT_FUNC_ENTER;
  gear_count_ = (gear_count_ - 1) % 4;
  if (gear_count_ < 1) gear_count_ = 0;
  if (gear_count_ == 0) ExecuteSetGear("reverse");
  if (gear_count_ == 1) ExecuteSetGear("neutral");
  if (gear_count_ == 2) ExecuteSetGear("fwddrive");
  if (gear_count_ == 3) ExecuteSetGear("boost");

  if (cm0_.GetDebugOption())
  {
    control_pressed_ = "Gear Shift Down";
    printf("Action: %s\n", control_pressed_.c_str());
    dirty_ = true;
  }
  PRINT_FUNC_EXIT;
}
/// to set mode of the joystick
void Redgear::SetMode(std::string type)
{
  PRINT_FUNC_ENTER;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);
  if (type.compare("throttle") == 0)
    mode_ = "throttle";
  else
    mode_ = "controller";
  PRINT_FUNC_EXIT;
}
/// to set specific Gear by the particular joystick
void Redgear::ExecuteSetGear(std::string gear_val)
{
  PRINT_FUNC_ENTER;
  std::transform(gear_val.begin(), gear_val.end(), gear_val.begin(), ::tolower);
  printf("gear change mech goes in here \n");
  car_state_.gear_cmd = gear_val;
  PRINT_FUNC_EXIT;
}
/// to enable/disable horn
void Redgear::ExecuteHorn(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
  {
    cm0_.SendCmdToVehicle("Horn_Control", "on");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Horn On";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  else
  {
    cm0_.SendCmdToVehicle("Horn_Control", "off");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Horn Off";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  car_state_.horn_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to enable/disable wiper
void Redgear::ExecuteWiper(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
  {
    cm0_.SendCmdToVehicle("Wiper_Control", "on");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Wiper On";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  else
  {
    cm0_.SendCmdToVehicle("Wiper_Control", "off");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Wiper Off";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  car_state_.wiper_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to enable/disable headlight
void Redgear::ExecuteHeadLight(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Head_Lamp_Control", "high_beam");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Head Light On";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  else
  {
    cm0_.SendCmdToVehicle("Driving_Lamp_Head_Lamp_Control", "off");
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Head Light Off";
      printf("Action: %s\n", control_pressed_.c_str());
      dirty_ = true;
    }
  }
  car_state_.headlight_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to execute the Steer of the particular joystick
void Redgear::ExecuteSteer(double value)
{
  PRINT_FUNC_ENTER;
  if (value != car_state_.steer_target)
  {
    double steer_value_time_now = GetTimestampNow();
    double time_diff =
        (steer_value_time_now - car_state_.steer_header.stamp) / 1000000L;
    if (time_diff > 0.2)
    {
      car_state_.steer_target = value;
      car_state_.steer_header.stamp = steer_value_time_now;
      t_equation_ = 0;
      /// boundary checks
      if (car_state_.steer_target >= car_state_.steer_limit)
        car_state_.steer_target = car_state_.steer_limit;
      if (car_state_.steer_target <= -1 * car_state_.steer_limit)
        car_state_.steer_target = -1 * car_state_.steer_limit;
      if (car_state_.steer_target < 0 && car_state_.steer_target > -0.08)
        car_state_.steer_target = 0;
      if (car_state_.steer_target > 0 && car_state_.steer_target < -0.08)
        car_state_.steer_target = 0;
    }
  }
  if (cm0_.GetDebugOption())
  {
    control_pressed_ = "Steer Control";
    printf("Action: %s\n", control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
void Redgear::ExecuteSteerOffset(double value)
{
  PRINT_FUNC_ENTER;
  if (value > 0.75 || value < -0.75)
  {
    double steer_time_now = GetTimestampNow();
    double time_diff =
        (steer_time_now - car_state_.steer_header.stamp) / 1000000L;
    // check timestamp to avoid large updates so update once in 0.8 secs
    if (time_diff > 0.8)
    {
      car_state_.steer_header.stamp = steer_time_now;
      if (value > 0)
        car_state_.steer_offset = car_state_.steer_offset + step_size_steer_;
      else
        car_state_.steer_offset = car_state_.steer_offset - step_size_steer_;
      /// reset conditiions
      if (car_state_.steer_offset < 0 && car_state_.steer_offset > -0.08)
        car_state_.steer_offset = 0;

      if (car_state_.steer_offset >= car_state_.steer_limit)
        car_state_.steer_offset = car_state_.steer_limit;
      if (car_state_.steer_offset <= -1 * car_state_.steer_limit)
        car_state_.steer_offset = -1 * car_state_.steer_limit;
      t_equation_ = 0;
    }
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Steer Offset Control";
      printf("Action: %s\n", control_pressed_.c_str());
    }
  }
  PRINT_FUNC_EXIT;
}
/// to execute the Speed/Accelaration of the particular joystick
void Redgear::ExecuteSpeed(double value)
{
  PRINT_FUNC_ENTER;
  if (value > 0.75 || value < -0.75)
  {
    double speed_time_now = GetTimestampNow();
    double time_diff =
        (speed_time_now - car_state_.speed_header.stamp) / 1000000L;
    if (time_diff > 0.8)
    {
      car_state_.speed_header.stamp = speed_time_now;
      if (value > 0)
        car_state_.velocity_target =
            car_state_.velocity_target + step_size_speed_;
      else
        car_state_.velocity_target =
            car_state_.velocity_target - step_size_speed_;
      if (car_state_.velocity_target >= car_state_.velocity_limit)
        car_state_.velocity_target = car_state_.velocity_limit;
      if (car_state_.velocity_target <= -1 * car_state_.velocity_limit)
        car_state_.velocity_target = -1 * car_state_.velocity_limit;
    }
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Vel Control";
      printf("Action: %s\n", control_pressed_.c_str());
    }
    PRINT_FUNC_EXIT;
  }
}
/// to print help
void Redgear::PrintState()
{
  PRINT_FUNC_ENTER;
  // if mode is controller control
  if (mode_.compare("controller") == 0)
  {
    printf("(Velocity: %0.3f mps,", car_state_.velocity_target);
    printf("Steer: %0.3f degrees)\n", car_state_.steer_target);
  }
  else
  {
    // if mode is Throttle control
    printf("(Steer: %f degree, ", car_state_.steer_target);
    if (car_state_.throttle_cmd > 0)
      printf("Throttle: %d )\n", car_state_.throttle_cmd);
    else
      printf("Throttle: %d )\n", -1 * car_state_.brake_cmd);
  }
  PRINT_FUNC_EXIT;
}
/// to print help
void Redgear::PrintHelp()
{
  PRINT_FUNC_ENTER;
  printf("---------------------------\n");
  printf(
      "Use below Controls to move the Car. First Change the Gear if in "
      "gateway "
      "mode.\n\n");
  printf("\tToggle Head Light -> R2\t");
  printf("\t\tToggle Wiper -> L2\n");

  printf("\tLEFT Indicator -> L1\t");
  printf("\t\tRIGHT Indicator -> R1\n\n");

  printf("\t\t\t\t(Toggle Horn) \t");
  printf("\t\t\t(Release Hard Brake)\n");
  printf("\t\t\t\tStart Button\t");
  printf("\t\t\t\tY\n\t\n");
  printf("\t Steer Offset\t\t\t\t\t\t\tX\t\tB\n");
  printf(
      "\t <-- \t-->\t\t\t\t\t\t(Gear Down)\t\t\b\b\b\b(Gear "
      "Up)\n\n ");
  printf("\t\t\t\t\t\t\t\t\t\tA\n ");

  printf("\t\t\t\t\t\t\t\t\t\t\b\b\b\b(Hard Brake)\n\n");
  printf(
      "\t\b\b\b(Acclerate)\t\t\t\b(Steer "
      "Left)\t\b\b\b(Reset)\t\b(Steer "
      "Right)\n");
  printf("\t ^\t\t\t\t\t<-- \t\b\b\bPress\t-->\n");
  printf("\t |\n");
  printf("\t\b\b\b Stop(Press)\n");
  printf("\t |\n");
  printf("\t V\n");
  printf("\t\b\b\b(Deacclerate)\n");

  printf("HAZARD Indicator -> L1 + R1 \t");

  printf("\n---------------------------\n");
  PRINT_FUNC_EXIT;
}
/// to process message
void Redgear::ProcessMessage(JoystickMsg msg)
{
  /// For braking
  if (msg.buttons[redgearjoy::hard_brake_press.index] == 1)
  {
    ExecuteHardBrakePress();
  }
  if (msg.buttons[redgearjoy::hard_brake_release.index] == 1)
  {
    ExecuteHardBrakeRelease();
  }
  /// Execute Indicators
  if (msg.buttons[redgearjoy::left_indicator.index] == 1 &&
      msg.buttons[redgearjoy::right_indicator.index] == 0)
  {
    left_indicator_switch_ = (left_indicator_switch_ + 1) % 2;
    if (left_indicator_switch_)
      ExecuteIndicators(1);
    else
      ExecuteIndicators(0);
  }
  if (msg.buttons[redgearjoy::right_indicator.index] == 1 &&
      msg.buttons[redgearjoy::left_indicator.index] == 0)
  {
    right_indicator_switch_ = (right_indicator_switch_ + 1) % 2;
    if (right_indicator_switch_)
      ExecuteIndicators(2);
    else
      ExecuteIndicators(0);
  }
  if (msg.buttons[redgearjoy::right_indicator.index] == 1 &&
      msg.buttons[redgearjoy::left_indicator.index] == 1)
  {
    hazard_indicator_switch_ = (hazard_indicator_switch_ + 1) % 2;
    if (hazard_indicator_switch_)
      ExecuteIndicators(3);
    else
      ExecuteIndicators(0);
  }
  /// execute Horn and off in abence
  if (msg.buttons[redgearjoy::horn.index] == 1)
  {
    car_state_.horn_cmd = 1;
    ExecuteHorn(1);
  }
  else
  {
    if (msg.buttons[redgearjoy::horn.index] == 0 && car_state_.horn_cmd)
    {
      car_state_.horn_cmd = 0;
      ExecuteHorn(0);
    }
    if (car_state_.horn_cmd == 0)
    {
      ExecuteHorn(0);
    }
  }
  /// execute wiper
  if (msg.buttons[redgearjoy::wiper.index] == 1)
  {
    car_state_.wiper_cmd = (car_state_.wiper_cmd + 1) % 2;
    ExecuteWiper(car_state_.wiper_cmd);
  }
  /// execute headlight
  if (msg.buttons[redgearjoy::head_light.index] == 1)
  {
    car_state_.headlight_cmd = (car_state_.headlight_cmd + 1) % 2;
    ExecuteHeadLight(car_state_.headlight_cmd);
  }
  /// Execute gear
  if (msg.buttons[redgearjoy::gear_up.index] == 1)
  {
    ExecuteGearUp();
  }
  if (msg.buttons[redgearjoy::gear_down.index] == 1)
  {
    ExecuteGearDown();
  }
  /// to process steer and its offset
  double incoming_steer_target_value =
      msg.axes[redgearjoy::steer.index] * car_state_.steer_limit / 4;
  // update steer_value so as to go for a cycloidal trajectory
  double offset_input = msg.axes[redgearjoy::offset.index];
  ExecuteSteer(incoming_steer_target_value);
  /// to process speed
  double speed_input = msg.axes[redgearjoy::speed.index];
  ExecuteSpeed(speed_input);

  // steer_target_value0 = msg.axes[3] * steer_degree_limit;
  if (msg.buttons[redgearjoy::vel_set_zero.index] == 1)
  {
    car_state_.velocity_target = 0;
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Vel. Control";
      printf("Action: %s\n", control_pressed_.c_str());
    }
  }
  // update velocity_value so as to go for a cycloidal trajectory
  if (msg.buttons[redgearjoy::steer_set_zero.index] == 1)
  {
    car_state_.steer_target = 0;
    if (cm0_.GetDebugOption())
    {
      control_pressed_ = "Steer Control";
      printf("Action: %s\n", control_pressed_.c_str());
    }
  }
  if (cm0_.GetDebugOption() == true)
  {
    printf("Velocity value (Target) : %0.2f\n", car_state_.velocity_target);
    printf("Steer Value (Target) : %0.4f\n", car_state_.steer_target);
    printf("Steer Offset : %0.4f\n", car_state_.steer_offset);
  }
}
/// to reset the car
void Redgear::ResetState()
{
  cm0_.SendCmdToVehicle("Steering_Degree_Control", "0");
  cm0_.SendCmdToVehicle("Steering_Direction_Control", "right");
  cm0_.SendCmdToVehicle("Driving_Lamp_Head_Lamp_Control", "off");
  cm0_.SendCmdToVehicle("Driving_Lamp_Turn_Indicator_Control", "off");
  cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", "0");
  cm0_.SendCmdToVehicle("Horn_Control", "off");
  cm0_.SendCmdToVehicle("Wiper_Control", "off");
  cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
  cm0_.SendCmdToVehicle("Steering_Homing_Control", "homing_active");
  cm0_.SendCmdToVehicle("Driving_Lamp_Brake_Light_Control", "off");
  car_state_.velocity_target = 0;
  car_state_.steer_target = 0;
  car_state_.steer_offset = 0;
}
/// to reset the car
void Redgear::Setup(const int* exit_flag_pointer)
{
  cm0_.Init(exit_flag_pointer);
}
