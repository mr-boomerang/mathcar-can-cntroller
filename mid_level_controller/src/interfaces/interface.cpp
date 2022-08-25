// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file interface.cpp
 * @brief joystick interface for can. this file allows for driving of car though
 * joystick.
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#include <interfaces/interface.h>

#include <algorithm>
#include <string>
#include <vector>

#ifndef UTILS_AVOID_GFLAGS
/// path to config file
DEFINE_string(joy_cfg, "./settings_default.cfg",
              "File path to the configuration file for the package.");
/// input mode selection
DEFINE_string(mode, "keyboard", "Mode of operation");
/// path to joystick
DEFINE_string(joy_path, "/dev/input/js0", "path to joystick");
#else
/// path to config file
std::string FLAGS_joy_cfg = "./cfg/settings.cfg";
/// input mode selection
std::string FLAGS_mode = "keyboard";
/// path to joystick
std::string FLAGS_joy_path = "/dev/input/js1";
#endif

void Interface::SetMode(std::string mode)
{
  PRINT_FUNC_ENTER;
  joystick->SetMode(mode);
  PRINT_FUNC_EXIT;
}

void Interface::StartInterface(int argc, char **argv, const int *exit_flag_arg_)
{
  PRINT_FUNC_ENTER;
  std::string settings_file_path;
  exit_flag_ptr_ = exit_flag_arg_;
#ifndef BUILD_WITHOUT_ROS
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "teleop_node", ros::init_options::NoSigintHandler);
  }
#endif
// gflags or miniflags check
#ifndef UTILS_AVOID_GFLAGS
  // parse commands
  print::init(&argc, &argv);
#else
  miniflags::miniflags_init(argc, argv);
#endif

// for swahana monitor
#ifndef _DEPLOY_SWAHANA_SUITE_
  settings_file_path = PKG_SRC "/cfg/" + FLAGS_joy_cfg;
#else
  settings_file_path =
      CONTROLLER_PREFIX "/../cfg/mid_level_controller/" + FLAGS_joy_cfg;
#endif
  cm0_.Init(exit_flag_ptr_);
  cm0_.SendCmdToVehicle("Driving_Gear_Control", "neutral");
  // 0 for keyboard
  // 1 for joystick
  int input_mode = 0;
  // check against mode
  if (FLAGS_mode.compare("joy") == 0)
  {
    // start a thread to monitor controller if it present else revert to
    // keyboard
    std::string command = "ls -l /dev/input";
    int interface_enable = cm0_.ExecCommand(command);
    if (interface_enable == 1)
    {
      input_mode = 1;
    }
    else
    {
      PE$ ct_red("Can't find a joystick. Reverting back to keyboard") pendl;
      FLAGS_mode = "keyboard";
    }
  }
  if (!input_mode)
  {
    PN$ ct_grn("Starting Keyboard Controller") pendl;
    // keyboard
    InitKeyboard(settings_file_path);
    // start a thread to monitor keyboard
    KeyboardMonitorThd();
  }
  else if (input_mode)
  {
    // joystick
    InitJoystick(settings_file_path);
    // start a thread to process joystick
    JoystickMonitorThd(FLAGS_joy_path);
  }
  PRINT_FUNC_EXIT;
}
// to monitor keyboard
void Interface::KeyboardMonitor()
{
  PRINT_FUNC_ENTER;
  char key_in;
  /* Get current terminal settings. */
  tcgetattr(fileno(stdin), &old_settings_);
  // get the console in raw mode
  tcgetattr(kfd_, &cooked_);
  memcpy(&raw_, &cooked_, sizeof(struct termios));
  raw_.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw_.c_cc[VEOL] = 1;
  raw_.c_cc[VEOF] = 2;
  tcsetattr(kfd_, TCSANOW, &raw_);
  int flags = fcntl(0, F_GETFL, 0); /* get current file status flags */
  flags |= O_NONBLOCK;              /* turn off blocking flag */
  fcntl(0, F_SETFL, flags);         /* set up non-blocking read */
  // help on inputs
  joystick->PrintHelp();
  // run the key loop at 400 hz
  int sleep_time = 2500;
#ifndef BUILD_WITHOUT_ROS
  while (ros::ok() && !*exit_flag_ptr_)
  {
#else
  while (!*exit_flag_ptr_)
  {
#endif
    // get the next event from the keyboard
    key_in = getchar();
    if (cm0_.GetDebugOption()) printf("value: 0x%02X\n", key_in);
    joystick->ProcessMessage(key_in);
    usleep(sleep_time);
  }
  SignalHandler(1);
  PRINT_FUNC_EXIT;
}
// start monitoring joystick in a threaded manner
void Interface::KeyboardMonitorThd()
{
  PRINT_FUNC_ENTER;
  interface0_is_threaded_ = true;
  interface0_ = std::thread(&Interface::KeyboardMonitor, this);
  joy_process0_is_threaded_ = true;
  // start a thread to monitor joystick
  joy_process0_ = std::thread(&Interface::JoystickProcessVelocity, this);
  PN$ PRINT_SUCCESS << ct_grn("Keyboard Interface Working") pendl;
  PRINT_FUNC_EXIT;
}
void Interface::JoystickMonitor(std::string joy_port)
{
  PRINT_FUNC_ENTER;
  j0_.JoyInit(joy_port, exit_flag_ptr_, debug_enable_);
  j0_.JoyMonitorThd();
  while (!*exit_flag_ptr_)
  {
    if (j0_.getJoyMsgUpdated())
    {
      JoystickCallback(j0_.GetJoystickMsg());
      j0_.setJoyMsgUpdated(false);
    }
    usleep(2000);
  }
  PRINT_FUNC_EXIT;
}
void Interface::JoystickProcessVelocity()
{
  PRINT_FUNC_ENTER;
  // program state variables
  double current_output_velocity = 0;
  double current_output_steer = 0, last_output_velocity = 0;
  double last_output_steer = 0;
  teleop_car_velocity_.linear.x = 0;
  teleop_car_velocity_.linear.y = 0;
  teleop_car_velocity_.linear.z = 0;
  teleop_car_velocity_.angular.x = 0;
  teleop_car_velocity_.angular.y = 0;
  teleop_car_velocity_.angular.z = 0;
  speed_control_.publish(teleop_car_velocity_);
  t_equation_ = 0;
#ifndef BUILD_WITHOUT_ROS
  while (ros::ok() && !*exit_flag_ptr_)
  {
#else
  while (!*exit_flag_ptr_)
  {
#endif
    // apply cyloidal trajectory for velocity and steer
    // current_output_steer = teleop_car.current_steering();
    t_equation_ = (joystick->GetCarState()).t_equation;
    current_output_steer = CurrentSteering(
        t_equation_, (joystick->GetCarState()).steer_target,
        (joystick->GetCarState()).steer_fdbk, total_time_ * frequency_);
    current_output_velocity = (joystick->GetCarState()).velocity_target;
    // for trajectory profiling
    t_equation_++;
    if (t_equation_ > total_time_ * frequency_)
      t_equation_ = total_time_ * frequency_;
    joystick->SetSteerCounter(t_equation_);
    // to send message to controller
    teleop_car_velocity_.linear.x = current_output_velocity;
    teleop_car_velocity_.angular.z = current_output_steer * M_PI / 180;
    cm0_.SendCmdToController(teleop_car_velocity_);
    usleep(10000);
  }
  SignalHandler(1);
  PRINT_FUNC_EXIT;
}
// start monitoring joystick in a threaded manner
void Interface::JoystickMonitorThd(std::string joy_path)
{
  PRINT_FUNC_ENTER;
  interface0_is_threaded_ = true;
  // start a thread to monitor joystick
  interface0_ = std::thread(&Interface::JoystickMonitor, this, joy_path);
  if (!(joy_type_.compare("Sidewinder") == 0 ||
        joy_type_.compare("sidewinder") == 0))
  {
    // start a thread to monitor joystick
    joy_process0_is_threaded_ = true;
    joy_process0_ = std::thread(&Interface::JoystickProcessVelocity, this);
  }
  PN$ PRINT_SUCCESS << ct_grn(" Joystick Interface Working") pendl;
  joystick->PrintHelp();
  PRINT_FUNC_EXIT;
}
// start monitoring joystick
void Interface::JoystickCallback(JoystickMsg msg)
{
  PRINT_FUNC_ENTER;
  joystick->ProcessMessage(msg);
  PRINT_FUNC_EXIT;
}
// initialize keyboard
void Interface::InitKeyboard(const std::string controller_settings_file_path)
{
  PRINT_FUNC_ENTER;
  // fetch settings from the cfg file
  // load parameters using utils
  params_utils.load(controller_settings_file_path);
  // allows printing of debugging information
  debug_enable_ = params_utils.get_val("debug_joy_enable", false);
  // copy settings value to local variables
  input_mode_ =
      params_utils.get_val<std::string>("input_control_mode", "controller");
  steer_input_units_ =
      params_utils.get_val<std::string>("steer_input_units", "radian");
  velocity_input_units_ =
      params_utils.get_val<std::string>("velocity_input_units", "mps");
  max_velocity_ = params_utils.get_val<double>("vel_limit", 5);
  // adjust the limits as per units
  if (velocity_input_units_.compare("mps") == 0)
    max_velocity_ = max_velocity_ * 18 / 5;
  /// create a ros handle
  ros::NodeHandle nh0;
  joystick = new Keyboard();
  speed_control_ =
      nh0.advertise<geometry_msgs::Twist>("/velocitySteeringAngle", 10);
  // if debugging is enabled then print all parameters
  if (debug_enable_)
  {
    printf("Data read from controller settings file:\n");
    std::string lst = params_utils.list_full("\n");
    // params with vales and properties.
    printf("%s\n", lst.c_str());
    printf("*********************************************\n");
  }
  joystick->Setup(exit_flag_ptr_);
  joystick->ResetState();
  PRINT_FUNC_EXIT;
}
void Interface::InitJoystick(const std::string controller_settings_file_path)
{
  PRINT_FUNC_ENTER;
  velocity_ = 0;
  steer_input_ = 0;
  // fetch settings from the cfg file
  // load parameters using utils
  params_utils.load(controller_settings_file_path);
  // allows printing of debugging information
  debug_enable_ = params_utils.get_val("debug_msg_enable", false);
  // allows for setting of limit for steer input
  steer_degree_limit_ = params_utils.get_val<double>("steer_limit", 0.687);
  // allows for setting of limits for throttle.
  accl_limit_ = params_utils.get_val<double>("throttle_limit", 60);
  trajectory_ = params_utils.get_val<std::string>(
      "control_intermediate_trajectory_mode", "sin");
  trajectory_following_ = params_utils.get_val("trajectory_followed", true);
  velocity_limit_ = params_utils.get_val<double>("vel_limit", 5);
  printf("velocity limit %f\n", velocity_limit_);
  total_time_ = params_utils.get_val<double>("total_time", 1);
  // frequency = params_utils.get_val<double>("frequency", 100);
  steer_input_units_ =
      params_utils.get_val<std::string>("steer_input_units", "radian");
  velocity_input_units_ =
      params_utils.get_val<std::string>("velocity_input_units", "mps");
  joy_type_ = params_utils.get_val<std::string>("joyType", "redgear");
  joy_expose_topics_ = params_utils.get_val("joyExposeTopics", false);
  /// update as per joystick
  if (joy_type_.compare("redgear") == 0) joystick = new Redgear();
  if (joy_type_.compare("xbox") == 0) joystick = new Xbox();
  if (joy_type_.compare("sidewinder") == 0 ||
      joy_type_.compare("Sidewinder") == 0)
    joystick = new Sidewinder();
  // j0.expose_topics = joyExposeTopics;
  if (velocity_input_units_.compare("mps") == 0)
  {
    step_size_speed_ = step_size_speed_ * 5 / 18;
  }
  if (steer_input_units_.compare("radian") == 0)
  {
    step_size_steer_ = step_size_steer_ * M_PI / 180;
  }
  speed_time_last_ = GetTimestampNow();
  speed_time_now_ = GetTimestampNow();
  steer_time_last_ = GetTimestampNow();
  steer_time_now_ = GetTimestampNow();
  printf("Throttle Limit : %d\n", accl_limit_);
  printf("Steer Limit : %f radians\n", steer_degree_limit_);
#ifndef BUILD_WITHOUT_ROS
  /// create a ros handle
  ros::NodeHandle nh0;
  // initialize the subscriber and publisher for joystick and velocity control
  // later to be replaced by some ros independent thing
  // sub = nh0.subscribe("joy", 1000, &Interface::joystick_callback, this);
  speed_control_ =
      nh0.advertise<geometry_msgs::Twist>("velocitySteeringAngle", 1000);
  control_pub_ =
      nh0.advertise<mid_level_controller::Callback_Msg>("canControl", 10);
#endif
  joystick->Setup(exit_flag_ptr_);
  joystick->ResetState();
  PRINT_FUNC_EXIT;
}
// get intermediate trajectory for steering
double Interface::CurrentSteering(int time_handle, double target,
                                  double starting_position, double time_limit)
{
  PRINT_FUNC_ENTER;
  if (!trajectory_following_) return (target);
  double output = 0;
  if (trajectory_.compare("sin") == 0)
  {
    output = starting_position + (target - starting_position) *
                                     sin(M_PI * time_handle / 2 / time_limit);
  }
  if (trajectory_.compare("ramp") == 0)
  {
    output = starting_position +
             (target - starting_position) * time_handle / time_limit;
  }
  if (trajectory_.compare("cycloidal") == 0)
  {
    output = starting_position +
             ((target - starting_position) / (2 * M_PI)) *
                 (time_handle -
                  (2 * M_PI * (sin(time_handle * 2 * M_PI / time_limit)) /
                   (2 * time_limit)));
  }
  if (trajectory_.compare("step") == 0)
  {
    if (time_handle > 0)
      output = starting_position + (target - starting_position);
    else
      output = starting_position;
  }
  // output = output + steer_offset_;
  // if (output >= steer_degree_limit_) output = steer_degree_limit_;
  // if (output <= -1 * steer_degree_limit_) output = -1 * steer_degree_limit_;

  if (debug_enable_)
  {
    printf("Current Steer : %0.2f\n", output);
  }
  PRINT_FUNC_EXIT;
  return (output);
}
// get intermediate trajectory for speed
double Interface::CurrentSpeed(int time_handle, double target,
                               double starting_position, double time_limit)
{
  PRINT_FUNC_ENTER;
  if (!trajectory_following_) return (target);
  double output = 0;
  if (trajectory_.compare("sin") == 0)
  {
    output = starting_position + (target - starting_position) *
                                     sin(M_PI * time_handle / 2 / time_limit);
  }
  if (trajectory_.compare("ramp") == 0)
  {
    output = starting_position +
             (target - starting_position) * time_handle / time_limit;
  }
  if (trajectory_.compare("cycloidal") == 0)
  {
    output = starting_position +
             ((target - starting_position) / (2 * M_PI)) *
                 (time_handle -
                  (2 * M_PI * (sin(time_handle * 2 * M_PI / time_limit)) /
                   (2 * time_limit)));
  }
  if (trajectory_.compare("step") == 0)
  {
    if (time_handle > 0)
      output = starting_position + (target - starting_position);
    else
      output = starting_position;
  }
  if (debug_enable_ == true)
  {
    printf("Current Speed : %0.2f\n", output);
  }
  PRINT_FUNC_EXIT;
  return (output);
}
/// exit method
Interface::~Interface()
{
  PRINT_FUNC_ENTER;
  // closing other things
  if (interface0_is_threaded_ && interface0_.joinable())
  {
    interface0_.join();
  }
  if (joy_process0_is_threaded_ && joy_process0_.joinable())
  {
    joy_process0_.join();
  }
  PRINT_FUNC_EXIT;
}
/// clean exit
void Interface::SignalHandler(int signum)
{
  PRINT_FUNC_ENTER;
  PI$ ct_grn("Closing interface instance") pendl;
  if (FLAGS_mode.compare("joy") != 0)
    tcsetattr(fileno(stdin), TCSANOW, &old_settings_);
#ifndef UTILS_AVOID_GFLAGS
  gflags::ShutDownCommandLineFlags();
#endif
  delete joystick;
  PRINT_FUNC_EXIT;
}
