// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file joy_struct.h
 * @brief strucutes for joystick
 * @author Divyanshu Goel
 * @date 2018-01-26 (yyyy-mm-dd)
 **/
#ifndef INTERFACES_JOY_STRUCT_H
#define INTERFACES_JOY_STRUCT_H

#include <algorithm>  /// to use transform
#include <chrono>     /// to use timestamps
#include <string>     /// to use strings
#include <vector>     /// to use vectors

#include "can_driver/can_struct.h"    /// for Header
#include "controller/common_utils.h"  /// for car state
#include "linux/joystick.h"

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

/// Sets the input for joystick for each pin
struct JoystickInput
{
  /// takes the value of the index for an operation
  int index = 0;
  /// set the value for particular input src axes by default(0).
  bool mode = 0;
  /// to set the limit on the axes on negative side
  double limit_negative = -1;
  /// to set the limit on the axes on positive side
  double limit_positive = 1;
  /// to set the value back to zero
  void Clear()
  {
    index = 0;
    mode = 0;
    limit_negative = -1;
    limit_positive = 1;
  }
  /// to set the initial values
  JoystickInput(int index_input, bool src)
  {
    SetIndex(index_input);
    SelectSrc(src);
  }
  /// to set the initial values
  JoystickInput(int index_input, std::string src)
  {
    SetIndex(index_input);
    SelectSrc(src);
  }
  /// to set the value to desired values
  void SetIndex(int index_input) { index = index_input; }
  /// to set the value to desired values
  void SetLimitPositive(double limit_positive_input)
  {
    limit_positive = limit_positive_input;
  }
  /// to set the value to desired values
  void SetLimitNegative(double limit_negative_input)
  {
    limit_negative = limit_negative_input;
  }
  /// to set the src for the joystick input
  void SelectSrc(bool src)
  {
    /// 0 for axes
    /// 1 for button
    mode = src;
  }
  /// to set the src for the joystick input
  void SelectSrc(std::string src)
  {
    std::transform(src.begin(), src.end(), src.begin(), ::tolower);
    if (src.compare("axes") == 0) mode = 0;
    if (src.compare("buttons") == 0 || src.compare("button") == 0) mode = 1;
  }
  /// to get the current mode of control
  bool GetMode() { return (mode); }
};
/// Reports the state of a joysticks axes and buttons.
struct JoystickMsg
{
  /// timestamp in the header is the time the data is received from the joystick
  Header header;
  /// the axes measurements from a joystick
  std::vector<double> axes = decltype(axes)(6, -1);
  /// the buttons measurements from a joystick
  std::vector<double> buttons = decltype(buttons)(12, 0);
  /// flag for last update of state
  bool data_updated = false;
  /// to set the value back to zero
  void Clear()
  {
    header.Clear();
    axes.clear();
    buttons.clear();
    data_updated = false;
  }
  /// to set the value of data update
  void SetUpdated(bool data_updated_input) { data_updated = data_updated; }
  /// to get the value of data update
  bool GetUpdated() { return (data_updated); }
};

///  @brief Defines base class for Joystick types and their response for basic
/// functions
class ControllerBase
{
 protected:
  /// parameters for the vehicle
  CarState car_state_;
  /// handles for publishing etc.
  CommonUtils cm0_;
  /// flag for key pressed
  bool dirty_ = 0;
  /// to have control info for the key press
  std::string control_pressed_ = "none";
  /// switch for left indicator
  int left_indicator_switch_ = 0;
  /// switch for left indicator
  int right_indicator_switch_ = 0;
  /// switch for left indicator
  int hazard_indicator_switch_ = 0;
  /// switch for left indicator
  int gear_count_ = 0;
  /// mode of the car
  std::string mode_ = "controller";
  /// increment size for steer offset
  double step_size_steer_ = 5;
  /// increment size for steer offset
  double step_size_speed_ = 0.278;
  /// start time to allow for cycloidal trajectory
  double t_equation_ = 0;

#ifndef BUILD_WITHOUT_ROS
  /// geometry msg for transmission
  geometry_msgs::Twist control_input_;

#endif

 public:
  /// constructor
  ControllerBase() {}
  /// destructor
  ~ControllerBase() {}
  /**
   * @brief to get current car state
   * @param value value to set for the counter
   */
  CarState GetCarState() { return (car_state_); }
  /**
   * @brief to increase value of t_equation
   */
  void SetSteerCounter(int value) { car_state_.t_equation = value; }
  /**
   * @brief to execute the hard brake press option of the particular joystick
   */
  virtual void ExecuteHardBrakePress() = 0;
  /**
   * @brief to execute the hard brake release option of the particular joystick
   */
  virtual void ExecuteHardBrakeRelease() = 0;
  /**
   * @brief to execute the Indicators of the particular joystick
   * @param type option to set ; off =0, left = 1 , right = 2, hazard = 3
   */
  virtual void ExecuteIndicators(int type) = 0;
  /**
   * @brief to execute the Steer of the particular joystick
   */
  virtual void ExecuteSteer(double value) = 0;
  /**
   * @brief to execute the Steer of the particular joystick
   */
  virtual void ExecuteSteerOffset(double value) = 0;
  /**
   * @brief to execute the Speed/Accelaration of the particular joystick
   */
  virtual void ExecuteSpeed(double value) = 0;
  /**
   * @brief to set mode of the joystick
   * @param type option to set
   */
  virtual void SetMode(std::string type) = 0;
  /**
   * @brief to execute the Gear Up by the particular joystick
   */
  virtual void ExecuteGearUp() = 0;
  /**
   * @brief to execute the Gear Down by the particular joystick
   */
  virtual void ExecuteGearDown() = 0;
  /**
   * @brief to set specific Gear by the particular joystick
   * @param gear_val option to set
   */
  virtual void ExecuteSetGear(std::string gear_val) = 0;
  /**
   * @brief to enable/disable horn
   * @param state option to set
   */
  virtual void ExecuteHorn(bool state) = 0;
  /**
   * @brief to enable/disable wiper
   * @param state option to set
   */
  virtual void ExecuteWiper(bool state) = 0;
  /**
   * @brief to enable/disable headlight
   * @param state option to set
   */
  virtual void ExecuteHeadLight(bool state) = 0;
  /**
   * @brief This method will be used to reset car
   */
  virtual void ResetState() = 0;
  /**
   * @brief This method will be used to print status of car based on can
   * feedback
   */
  virtual void PrintState() = 0;
  /**
   * @brief This method will be used to print help for the joystick
   */
  virtual void PrintHelp() = 0;
  /**
   * @brief to setup things for the controller
   * @param exit_flag_pointer exit variable flag
   */
  virtual void Setup(const int* exit_flag_pointer) = 0;
  /**
   * @brief This method will be used to parse incoming message and respond
   * accordingly
   * @param msg incoming message
   */
  virtual void ProcessMessage(JoystickMsg msg) {}
  /**
   * @brief to process incoming messages
   * @param key_pressed incoming pressed key
   */
  virtual void ProcessMessage(char key_pressed) {}
};
#endif  // INTERFACES_JOY_STRUCT_H
