// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file keyboard.cpp
 * @brief processing for keyboard
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#include "interfaces/peripherals/keyboard.h"

#include <algorithm>  // for transform
#include <string>     /// for handling strings

/// to execute the hard brake press option of the particular joystick/Keyboard
void Keyboard::ExecuteHardBrakePress()
{
  PRINT_FUNC_ENTER;
  if (mode_.compare("controller") == 0)
  {
    control_input_.linear.x = 0;
    cm0_.SendCmdToController(control_input_);
    car_state_.velocity_target = 0;
  }
  else
  {
    cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
    cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", 100);
    car_state_.velocity_target = 0;
    car_state_.throttle_cmd = 0;
    car_state_.brake_cmd = 100;
  }
  if (cm0_.GetDebugOption())
  {
    key_pressed_ = "Space";
    control_pressed_ = "Hard Brake press";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to execute the hard brake release option of the particular
/// joystick
void Keyboard::ExecuteHardBrakeRelease()
{
  PRINT_FUNC_ENTER;
  if (mode_.compare("controller") == 0)
  {
    control_input_.linear.x = 0;
    cm0_.SendCmdToController(control_input_);
    car_state_.velocity_target = 0;
  }
  else
  {
    cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
    cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", "0");
    car_state_.velocity_target = 0;
    car_state_.throttle_cmd = 0;
    car_state_.brake_cmd = 0;
  }
  if (cm0_.GetDebugOption())
  {
    key_pressed_ = "Space";
    control_pressed_ = "Hard Brake Release";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to execute the Indicators of the particular joystick
void Keyboard::ExecuteIndicators(int type)
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
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  car_state_.indicator_cmd = control_pressed_;
  dirty_ = true;
  PRINT_FUNC_EXIT;
}
/// to execute the Steer of the particular joystick
void Keyboard::ExecuteSteerOffset(double value) {}
/// to execute the Steer of the particular joystick
void Keyboard::ExecuteSteer(double value)
{
  PRINT_FUNC_ENTER;
  if (fabs(car_state_.steer_target) >= car_state_.steer_limit) return;
  car_state_.t_equation = 0;
  car_state_.steer_target = car_state_.steer_target + value;
  if (car_state_.steer_target > car_state_.steer_limit)
    car_state_.steer_target = car_state_.steer_limit;
  if (car_state_.steer_target < -1 * car_state_.steer_limit)
    car_state_.steer_target = -1 * car_state_.steer_limit;

  if (cm0_.GetDebugOption())
  {
    dirty_ = true;
    control_pressed_ = "Steer";
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to execute the Speed/Accelaration of the particular joystick
void Keyboard::ExecuteSpeed(double value)
{
  PRINT_FUNC_ENTER;
  if (input_mode_.compare("throttle") == 0)
  {
    if (cm0_.GetDebugOption()) printf("Accl.");
    car_state_.throttle_cmd = car_state_.throttle_cmd + value;
    if (car_state_.throttle_cmd > car_state_.throttle_limit)
      car_state_.throttle_cmd = car_state_.throttle_limit;
    if (car_state_.throttle_cmd < -1 * car_state_.throttle_limit)
      car_state_.throttle_cmd = -1 * car_state_.throttle_limit;
    if (car_state_.throttle_cmd >= 0)
    {
      cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control", "0");
      cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control",
                            car_state_.throttle_cmd);
      car_state_.brake_cmd = 0;
    }
    else
    {
      cm0_.SendCmdToVehicle("Accln_Control_Percntg_Control", "0");
      cm0_.SendCmdToVehicle("Braking_Control_Percntg_Control",
                            -1 * car_state_.throttle_cmd);
      car_state_.brake_cmd = car_state_.throttle_cmd;
    }
    control_pressed_ = "Throttle";
  }
  else
  {
    car_state_.velocity_target = (last_value_speed_ + value) * 5 / 18;
    last_value_speed_ = last_value_speed_ + value;
    if (car_state_.velocity_target > car_state_.velocity_limit)
      car_state_.velocity_target = car_state_.velocity_limit;
    if (car_state_.velocity_target <= -1 * car_state_.velocity_limit)
      car_state_.velocity_target = -1 * car_state_.velocity_limit;
    control_pressed_ = "Velocity";
  }
  if (cm0_.GetDebugOption())
  {
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to set mode of the joystick
void Keyboard::SetMode(std::string type)
{
  PRINT_FUNC_ENTER;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);
  if (type.compare("throttle") == 0)
    mode_ = "throttle";
  else
    mode_ = "controller";
  PRINT_FUNC_EXIT;
}
/// to execute the Gear Up by the particular joystick
void Keyboard::ExecuteGearUp()
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
    key_pressed_ = "1";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to execute the Gear Down by the particular joystick
void Keyboard::ExecuteGearDown()
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
    key_pressed_ = "2";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  PRINT_FUNC_EXIT;
}
/// to set specific Gear by the particular joystick
void Keyboard::ExecuteSetGear(std::string gear_val)
{
  PRINT_FUNC_ENTER;
  std::transform(gear_val.begin(), gear_val.end(), gear_val.begin(), ::tolower);
  cm0_.SetGear(gear_val);
  car_state_.gear_cmd = gear_val;
  PRINT_FUNC_EXIT;
}
/// to enable/disable horn
void Keyboard::ExecuteHorn(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
    cm0_.SendCmdToVehicle("Horn_Control", "on");
  else
    cm0_.SendCmdToVehicle("Horn_Control", "off");
  if (cm0_.GetDebugOption())
  {
    key_pressed_ = "J";
    control_pressed_ = "Horn";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  car_state_.horn_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to enable/disable wiper
void Keyboard::ExecuteWiper(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
    cm0_.SendCmdToVehicle("Wiper_Control", "high");
  else
    cm0_.SendCmdToVehicle("Wiper_Control", "off");
  if (cm0_.GetDebugOption())
  {
    key_pressed_ = "P";
    control_pressed_ = "Wiper";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  car_state_.wiper_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to enable/disable headlight
void Keyboard::ExecuteHeadLight(bool state)
{
  PRINT_FUNC_ENTER;
  if (state)
    cm0_.SendCmdToVehicle("Driving_Lamp_head_lamp_Control", "high_beam");
  else
    cm0_.SendCmdToVehicle("Driving_Lamp_Head_Lamp_Control", "off");
  if (cm0_.GetDebugOption())
  {
    key_pressed_ = "O";
    control_pressed_ = "Head Light";
    dirty_ = true;
    printf("Key: %s Action: %s\n", key_pressed_.c_str(),
           control_pressed_.c_str());
  }
  car_state_.headlight_cmd = state;
  PRINT_FUNC_EXIT;
}
/// to print help
void Keyboard::PrintHelp()
{
  PRINT_FUNC_ENTER;
  // help on inputs
  printf("Reading from keyboard\n");
  printf("---------------------------\n");
  printf("Use arrow keys to move the Car. First Change the Gear.\n");
  printf("\t\b\b\b\b(GEAR DOWN)");
  printf("\t\t\b\b\b\b(GEAR UP)\n");
  printf("\t1");
  printf("\t\t2\n\n");

  printf("\t\t\b\b(ACCL.)\n");
  printf("\t\tW\n");
  printf("\t\b\b\b(LEFT)");
  printf("\t\b\b\b(BRAKE)");
  printf("\t\b\b\b(RIGHT)\n");
  printf("\tA");
  printf("\tS");
  printf("\tD \n\n");
  printf("\t\t\b\b\b\b\b\b (Hard Brake)");
  printf("\n\t\t\b\bSPACE\n\n");

  printf("LEFT Indicator -> Q \t");
  printf("HAZARD Indicator -> H \t");
  printf("RIGHT Indicator -> E \n\n");

  printf("Toggle Horn -> J \t");
  printf("Toggle Head Light -> O \t");
  printf("Toggle Wiper -> P\n");
  printf("\n---------------------------\n");
  PRINT_FUNC_EXIT;
}
/// to process message
void Keyboard::ProcessMessage(char key_pressed)
{
  PRINT_FUNC_ENTER;
  /// switch case to process input
  switch (key_pressed)
  {
    // HARD_STOP
    case keycode::keycode_SPACE:
      ExecuteHardBrakePress();
      break;
    // Directions -> Left
    case keycode::keycode_A:
    case keycode::keycode_a:
      ExecuteSteer(-1);
      key_pressed_ = "A";
      break;
    // Directions -> Right
    case keycode::keycode_D:
    case keycode::keycode_d:
      ExecuteSteer(1);
      key_pressed_ = "D";
      break;
    // Directions -> Accelerate
    case keycode::keycode_W:
    case keycode::keycode_w:
      ExecuteSpeed(1);
      key_pressed_ = "W";
      break;
    // Directions -> brake
    case keycode::keycode_S:
    case keycode::keycode_s:
      ExecuteSpeed(-1);
      key_pressed_ = "S";
      break;
    // Gears Control
    case keycode::keycode_SHIFT_UP:
      ExecuteGearUp();
      break;
    case keycode::keycode_SHIFT_DOWN:
      ExecuteGearDown();
      break;
    // Indicator Control-> Left
    case keycode::keycode_Q:
    case keycode::keycode_q:
      left_indicator_switch_ = (left_indicator_switch_ + 1) % 2;
      right_indicator_switch_ = 0;
      hazard_indicator_switch_ = 0;
      if (left_indicator_switch_)
        ExecuteIndicators(0);
      else
        ExecuteIndicators(1);
      key_pressed_ = "Q";
      break;
    // Indicator Control-> Right
    case keycode::keycode_E:
    case keycode::keycode_e:
      right_indicator_switch_ = (right_indicator_switch_ + 1) % 2;
      left_indicator_switch_ = 0;
      hazard_indicator_switch_ = 0;
      if (right_indicator_switch_)
        ExecuteIndicators(0);
      else
        ExecuteIndicators(2);
      key_pressed_ = "E";
      break;
    // Indicator Control-> Hazard
    case keycode::keycode_H:
    case keycode::keycode_h:
      hazard_indicator_switch_ = (hazard_indicator_switch_ + 1) % 2;
      left_indicator_switch_ = 0;
      right_indicator_switch_ = 0;
      if (hazard_indicator_switch_)
        ExecuteIndicators(0);
      else
        ExecuteIndicators(3);
      key_pressed_ = "H";
      break;
    // Horn Control
    case keycode::keycode_J:
    case keycode::keycode_j:
      car_state_.horn_cmd = (car_state_.horn_cmd + 1) % 2;
      ExecuteHorn(car_state_.horn_cmd);
      break;
    //  Wiper Control
    case keycode::keycode_P:
    case keycode::keycode_p:
      car_state_.wiper_cmd = (car_state_.wiper_cmd + 1) % 2;
      ExecuteWiper(car_state_.wiper_cmd);
      break;
    // Head Light Control
    case keycode::keycode_O:
    case keycode::keycode_o:
      car_state_.headlight_cmd = (car_state_.headlight_cmd + 1) % 2;
      ExecuteHeadLight(car_state_.headlight_cmd);
      break;
  }
  if (dirty_ &&
      (key_pressed == keycode::keycode_W || key_pressed == keycode::keycode_w ||
       key_pressed == keycode::keycode_A || key_pressed == keycode::keycode_a ||
       key_pressed == keycode::keycode_S || key_pressed == keycode::keycode_s ||
       key_pressed == keycode::keycode_D || key_pressed == keycode::keycode_d))
    PrintState();
  PRINT_FUNC_EXIT;
}
/// to print current state
void Keyboard::PrintState()
{
  PRINT_FUNC_ENTER;
  // if mode is controller control
  if (input_mode_.compare("controller") == 0)
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
/// to reset the car
void Keyboard::ResetState()
{
  PRINT_FUNC_ENTER;
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
  PRINT_FUNC_EXIT;
}
/// to reset the car
void Keyboard::Setup(const int* exit_flag_pointer)
{
  PRINT_FUNC_ENTER;
  cm0_.Init(exit_flag_pointer);
  PRINT_FUNC_EXIT;
}
