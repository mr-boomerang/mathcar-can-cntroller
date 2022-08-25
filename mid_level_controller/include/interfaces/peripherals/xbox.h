// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file keyboard.h
 * @brief controller instance for keyboard
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#ifndef INTERFACES_PERIPHERALS_XBOX_H
#define INTERFACES_PERIPHERALS_XBOX_H

#include "interfaces/joy_inputs.h"  /// for key maps
#include "interfaces/joy_struct.h"  /// for base class
#include <string>                   /// for strings

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

/// @brief Actions for Xbox joystick
class Xbox : public ControllerBase
{
 private:
 public:
  /// constructor
  Xbox() {}
  /// destructor
  ~Xbox() {}
  /**
   * @brief to set debug option
   * @param value option to set
   */
  void SetDebugOption(bool value);
  /**
   * @brief to execute the hard brake press option of the particular joystick
   */
  void ExecuteHardBrakePress();
  /**
   * @brief to execute the hard brake release option of the particular joystick
   */
  void ExecuteHardBrakeRelease();
  /**
   * @brief to execute the Indicators of the particular joystick
   * @param type option to set ; off =0, left = 1 , right = 2, hazard = 3
   */
  void ExecuteIndicators(int type);
  /**
   * @brief to execute the Steer of the particular joystick
   * @param value value to set or increase by
   */
  void ExecuteSteer(double value);
  /**
   * @brief to execute the Steer offset of the particular joystick
   * @param value value to set or increase by
   */
  void ExecuteSteerOffset(double value);
  /**
   * @brief to execute the Speed/Accelaration of the particular joystick
   * @param value value to set or increase by
   */
  void ExecuteSpeed(double value);
  /**
   * @brief to set mode of the joystick
   * @param type option to set
   */
  void SetMode(std::string type);
  /**
   * @brief to execute the Gear Up by the particular joystick
   */
  void ExecuteGearUp();
  /**
   * @brief to execute the Gear Down by the particular joystick
   */
  void ExecuteGearDown();
  /**
   * @brief to set specific Gear by the particular joystick
   * @param gear_val option to set
   */
  void ExecuteSetGear(std::string gear_val);
  /**
   * @brief to enable/disable horn
   * @param state option to set
   */
  void ExecuteHorn(bool state);
  /**
   * @brief to enable/disable wiper
   * @param state option to set
   */
  void ExecuteWiper(bool state);
  /**
   * @brief to enable/disable headlight
   * @param state option to set
   */
  void ExecuteHeadLight(bool state);
  /**
   * @brief This method will be used to reset car
   */
  void ResetState();
  /**
   * @brief This method will be used to print status of car based on can
   * feedback
   */
  void PrintState();
  /**
   * @brief to process incoming messages
   */
  void ProcessMessage(JoystickMsg msg);
  /**
   * @brief to print Help message corresponding to the hardware
   */
  void PrintHelp();
  /**
   * @brief to setup things for the controller
   * @param exit_flag_pointer exit variable flag
   */
  void Setup(const int* exit_flag_pointer);
};

#endif  // INTERFACES_PERIPHERALS_XBOX_H
