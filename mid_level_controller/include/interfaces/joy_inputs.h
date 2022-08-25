// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file joy_inputs.h
 * @brief inputs for joystick
 * @author Divyanshu Goel
 * @date 2018-04-29 (yyyy-mm-dd)
 **/
#ifndef INTERFACES_JOY_INPUTS_H
#define INTERFACES_JOY_INPUTS_H

#include "interfaces/joy_struct.h"

#include <utils/mode_printing.h>
#include <utils/params.hpp>
#include <utils/str_utils.h>

/// control for the keyboard
namespace keycode
{
/// Accelerate
const int keycode_W = 0x57;
/// Left
const int keycode_A = 0x41;
/// brake
const int keycode_S = 0x53;
/// Right
const int keycode_D = 0x44;
/// Left Indicator , double tap turns off --
const int keycode_Q = 0x51;
/// Right Indicator , double tap turns off --
const int keycode_E = 0x45;
/// Hazard Indicator , double tap turns off --
const int keycode_H = 0x48;
/// for accesories
/// Toggle horn on/off
const int keycode_J = 0x4A;
/// Toggle headlight On/Off --
const int keycode_O = 0x4F;
/// Toggle Wiper On/Off --
const int keycode_P = 0x50;

/// Gear and horn
/// Hard Brake --
const int keycode_SPACE = 0x20;
/// Gear Down --
const int keycode_SHIFT_DOWN = 0x31;
/// Gear up --
const int keycode_SHIFT_UP = 0x32;

// basic navigation
/// Accelerate --
const int keycode_w = 0x77;
/// Left --
const int keycode_a = 0x61;
/// brake --
const int keycode_s = 0x73;
/// Right --
const int keycode_d = 0x64;
/// Left Indicator , double tap turns off --
const int keycode_q = 0x71;
/// Right Indicator , double tap turns off --
const int keycode_e = 0x65;
/// Hazard Indicator , double tap turns off --
const int keycode_h = 0x68;
/// for accesories
/// Toggle horn on/off
const int keycode_j = 0x6A;
/// Toggle headlight On/Off --
const int keycode_o = 0x6F;
/// Toggle Wiper On/Off --
const int keycode_p = 0x70;
}  // namespace keycode
/// JoystickInput(x,y) -> x-> index, y-> button(1)/axes(2)
/// for the old JoyStick
namespace redgearjoy
{
/// Button A for joystick and its corresponding location in joy message, button
/// triangle
const JoystickInput hard_brake_press(0, 1);
/// Button B for joystick and its corresponding location in joy message, button
/// circle
const JoystickInput gear_up(1, 1);
/// Button X for joystick and its corresponding location in joy message, button
/// X
const JoystickInput hard_brake_release(2, 1);
/// Button Y for joystick and its corresponding location in joy message, button
/// square
const JoystickInput gear_down(3, 1);
/// Button LB/L1 for joystick and its corresponding location in joy message,
/// button l1
const JoystickInput left_indicator(4, 1);
/// Button RB/R1 for joystick and its corresponding location in joy message,
/// button r1
const JoystickInput right_indicator(5, 1);
/// Button LB/L2 for joystick and its corresponding location in joy message,
/// button l2
const JoystickInput wiper(6, 1);
/// Button RB/R2 for joystick and its corresponding location in joy message,
/// button r2
const JoystickInput head_light(7, 1);
/// Button Select for joystick and its corresponding location in joy message,
/// button select
const JoystickInput button_select(8, 1);
/// Button Start for joystick and its corresponding location in joy message,
/// button start
const JoystickInput horn(9, 1);
/// Axes Cross X for joystick and its corresponding location in joy message,
/// axes cross x
const JoystickInput offset(4, 0);
/// Axes Cross X for joystick and its corresponding location in joy message,
/// axes cross y
const JoystickInput axes_cross_y(5, 0);
/// Axes Left X for joystick and its corresponding location in joy message, axes
/// left X
const JoystickInput axes_left_x(0, 0);
/// Axes Left Y for joystick and its corresponding location in joy message, axes
/// left Y
const JoystickInput speed(1, 0);
/// Axes Right X for joystick and its corresponding location in joy message,
/// axes right X
const JoystickInput steer(2, 0);
/// Axes Right Y for joystick and its corresponding location in joy message,
/// axes right Y
const JoystickInput axes_right_y(3, 0);
/// Button Stick left for joystick and its corresponding location in joy
/// message, button left stick
const JoystickInput vel_set_zero(10, 1);
/// Button Stick left for joystick and its corresponding location in joy
/// message, button right stick
const JoystickInput steer_set_zero(11, 1);
}  // namespace redgearjoy

/// JoystickInput(x,y) -> x-> index, y-> button(1)/axes(2)
/// for the sidewinder
namespace sidewinder
{
/// Button X for joystick and its corresponding location in joy message
const JoystickInput hard_brake_release(0, 1);
/// Button Y(square) for joystick and its corresponding location in joy message
const JoystickInput gear_down(1, 1);
/// Button B(circle) for joystick and its corresponding location in joy message
const JoystickInput gear_up(2, 1);
/// Button A(Triangle) for joystick and its corresponding location in joy
/// message
const JoystickInput hard_brake_press(3, 1);
/// Button R1 for joystick and its corresponding location in joy message
const JoystickInput right_indicator(4, 1);
/// Button R2 for joystick and its corresponding location in joy message
const JoystickInput wiper(6, 1);
/// Button R3 for joystick and its corresponding location in joy message
const JoystickInput vel_set_zero(10, 1);
/// Button L1 for joystick and its corresponding location in joy message
const JoystickInput left_indicator(5, 1);
/// Button L2 for joystick and its corresponding location in joy message
const JoystickInput head_light(7, 1);
/// Button L3 for joystick and its corresponding location in joy message
const JoystickInput steer_set_zero(11, 1);
/// Button Select for joystick and its corresponding location in joy message
const JoystickInput button_share(8, 1);
/// Button Start for joystick and its corresponding location in joy message
const JoystickInput button_option(9, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput boost_gear_1(12, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput neutral_gear_1(13, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput fwddrive_gear_1(14, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput neutral_gear_2(15, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput fwddrive_gear_2(16, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput neutral_gear_3(17, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput reverse_gear_1(18, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput horn(23, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput button_plus(19, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput button_minus(20, 1);
/// Button Stick left for joystick and its corresponding location in joy message
const JoystickInput button_psp(24, 1);

/// Axes wheel for joystick and its corresponding location in joy message
const JoystickInput steer(0, 0);
/// Axes clutch for joystick and its corresponding location in joy message
const JoystickInput axes_clutch(1, 0);
/// Axes pedal for joystick and its corresponding location in joy message
const JoystickInput throttle(2, 0);
/// Axes brake for joystick and its corresponding location in joy message
const JoystickInput axes_brake(3, 0);
/// Axes Right Y for joystick and its corresponding location in joy message
const JoystickInput offset(4, 0);
/// Axes Right X for joystick and its corresponding location in joy message
const JoystickInput axes_top(5, 0);
}  // namespace sidewinder

/// JoystickInput(x,y) -> x-> index, y-> button(1)/axes(2)
/// defaults for xbox joystick
namespace xbox
{
/// button for braking
const JoystickInput hard_brake_press(0, 1);
/// button to release brake
const JoystickInput hard_brake_release(3, 1);
/// gear shift up
const JoystickInput gear_up(1, 1);
/// gear shift down
const JoystickInput gear_down(2, 1);
/// indicator left
const JoystickInput left_indicator(4, 1);
/// indicator right button
const JoystickInput right_indicator(5, 1);
/// button for axesRight to set velocity zero
const JoystickInput vel_set_zero(9, 1);
/// button for axesLeft to set steer zero
const JoystickInput steer_set_zero(9, 1);
/// horn
const JoystickInput horn(8, 1);
/// wiper
const JoystickInput wiper(2, 1);
/// headLight
const JoystickInput head_light(5, 1);
/// steer control button
const JoystickInput steer(3, 0);
/// speed control button
const JoystickInput speed(1, 0);
/// offset axes button
const JoystickInput offset(6, 0);
}  // namespace xbox

#endif  // INTERFACES_JOY_INPUTS_H
