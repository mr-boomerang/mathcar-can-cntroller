// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file joy_node.cpp
 * @brief joystick interface
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#include <interfaces/joy_node.h>

#include <string>
#include <vector>
/// to set joymsg update
void Joystick::setJoyMsgUpdated(bool update_option)
{
  // set the option
  joy_msg_.SetUpdated(update_option);
}
/// to set debug enable option
void Joystick::setDebugOption(bool debug_enable)
{
  // set the option
  debug_enable_ = debug_enable;
}
/// to expose joystick message
void Joystick::setExposeTopicsOption(bool expose_topics_enable)
{
  expose_topics_ = expose_topics_enable;
}
/// to set joymsg update
bool Joystick::getJoyMsgUpdated()
{
  // get the option
  return (joy_msg_.GetUpdated());
}
/// to set debug enable option
bool Joystick::getDebugOption()
{
  // get the option
  return (debug_enable_);
}
/// to expose joystick message
bool Joystick::getExposeTopicsOption()
{
  // get the option
  return (expose_topics_);
}
/// to return joystick message
JoystickMsg Joystick::GetJoystickMsg()
{
  // send the message
  return (joy_msg_);
}
/// Opens joystick port, reads from port and publishes while node is
/// active
void Joystick::JoyInit(std::string port, const int *exit_flag_arg,
                       bool debug_enable, double dead_zone,
                       double auto_repeat_rate, double coalesce_interval)
{
  joy_dev_ = port;
  exit_flag_ptr_ = exit_flag_arg;
  dead_zone_ = dead_zone_;
  auto_repeat_rate_ = auto_repeat_rate;
  coalesce_interval_ = coalesce_interval;
  debug_enable_ = debug_enable;
// Parameters
#ifndef BUILD_WITHOUT_ROS

  if (expose_topics_)
  {
    ros::NodeHandle nh0;
    pub_ = nh0.advertise<sensor_msgs::Joy>("joy", 1);
  }
#endif
  // Checks on parameters
  if (auto_repeat_rate_ > 1 / coalesce_interval_)
  {
    printf(
        "joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f "
        "Hz) does not make sense. Timing behavior is not well defined.\n",
        auto_repeat_rate_, 1 / coalesce_interval_);
  }

  if (dead_zone_ >= 1)
  {
    printf(
        "joy_node: deadzone greater than 1 was requested. The semantics "
        "of deadzone have changed. It is now related to the range "
        "[-1:1] instead of [-32767:32767]. For now I am dividing your "
        "deadzone by 32767, but this behavior is deprecated so you need "
        "to update your launch file.\n");
    dead_zone_ /= 32767;
  }

  if (dead_zone_ > 0.9)
  {
    printf("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9\n",
           dead_zone_);
    dead_zone_ = 0.9;
  }

  if (dead_zone_ < 0)
  {
    printf("joy_node: deadzone (%f) less than 0, setting to 0.\n", dead_zone_);
    dead_zone_ = 0;
  }

  if (auto_repeat_rate_ < 0)
  {
    printf("joy_node: autorepeat_rate (%f) less than 0, setting to 0.\n",
           auto_repeat_rate_);
    auto_repeat_rate_ = 0;
  }

  if (coalesce_interval_ < 0)
  {
    printf("joy_node: coalesce_interval (%f) less than 0, setting to 0.\n",
           coalesce_interval_);
    coalesce_interval_ = 0;
  }
}
void Joystick::JoyOpen()
{
  bool first_fault = true;
  joy_fd_ = 0;
  while (joy_fd_ <= 0)
  {
    joy_fd_ = open(joy_dev_.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ > 0)
    {
      ioctl(joy_fd_, JSIOCGAXES, &axes0_);
      ioctl(joy_fd_, JSIOCGBUTTONS, &buttons0_);
      if (debug_enable_)
      {
        printf(" Axes: %d\n", static_cast<int>(axes0_));
        printf(" Buttons: %d\n", static_cast<int>(buttons0_));
      }
      joy_msg_.axes.resize(static_cast<int>(axes0_));
      joy_msg_.buttons.resize(static_cast<int>(buttons0_));
      first_fault = false;
    }
    if (joy_fd_ > 0) break;
    if (first_fault)
    {
      printf("Couldn't open joystick %s. Will retry every second.\n",
             joy_dev_.c_str());
    }
    usleep(1000000);
  }

  printf("Opened joystick: %s. deadzone: %f.\n", joy_dev_.c_str(), dead_zone_);
  open_ = true;
}
int Joystick::JoyEventCheck(js_event event)
{
  int state = -1;
  switch (event.type)
  {
    case JS_EVENT_BUTTON:
    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      if (event.number >= joy_msg_.buttons.size())
      {
        int old_size = joy_msg_.buttons.size();
        joy_msg_.buttons.resize(event.number + 1);
        for (unsigned int i = old_size; i < joy_msg_.buttons.size(); i++)
          joy_msg_.buttons[i] = 0.0;
      }
      joy_msg_.buttons[event.number] = (event.value ? 1 : 0);
      // For initial events, wait a bit before sending to try to catch
      // all the initial events.
      if (!(event.type & JS_EVENT_INIT))
        state = 1;
      else
        state = 2;
      break;
    case JS_EVENT_AXIS:
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      if (event.number >= joy_msg_.axes.size())
      {
        int old_size = joy_msg_.axes.size();
        joy_msg_.axes.resize(event.number + 1);
        for (unsigned int i = old_size; i < joy_msg_.axes.size(); i++)
          joy_msg_.axes[i] = -1.0;
      }
      // Init event.value is wrong.
      if (!(event.type & JS_EVENT_INIT))
      {
        double val = event.value;
        // Allows deadzone to be "smooth"
        if (val > unscaled_dead_zone_)
          val -= unscaled_dead_zone_;
        else if (val < -unscaled_dead_zone_)
          val += unscaled_dead_zone_;
        else
          val = 0;
        joy_msg_.axes[event.number] = val * scale_;
      }
      // Will wait a bit before sending to try to combine events.
      state = 2;
      break;
    default:
      break;
  }
  if (debug_enable_)
  {
    printf("Axes: [");
    for (int i = 0; i < static_cast<int>(axes0_); i++)
    {
      if (i != static_cast<int>(axes0_) - 1)
        printf("%0.3f, ", joy_msg_.axes[i]);
      else
        printf("%0.3f", joy_msg_.axes[i]);
    }
    printf("]\n");
    printf("Buttons: [");
    for (int i = 0; i < static_cast<int>(buttons0_); i++)
    {
      if (i != static_cast<int>(buttons0_) - 1)
        printf("%0.0f, ", joy_msg_.buttons[i]);
      else
        printf("%0.0f", joy_msg_.buttons[i]);
    }
    printf("]\n");
  }
  return (state);
}
void Joystick::JoyMonitorThd()
{
  joy_is_threaded_ = true;
  joy_stick0_ = std::thread(&Joystick::JoyMonitor, this);
}
void Joystick::JoyMonitor()
{
  // Parameter conversions
  autorepeat_interval_ = 1 / auto_repeat_rate_;
  scale_ = -1. / (1. - dead_zone_) / 32767.;
  unscaled_dead_zone_ = 32767. * dead_zone_;
  js_event *event_ = new js_event();
  event_count_ = 0;
  pub_count_ = 0;
  int publish_state = -1;

  auto start = std::chrono::high_resolution_clock::now();
  last_diag_time_ = (start.time_since_epoch().count() / 1000) / 1000000L;
  JoyOpen();
  // Big while loop opens, publishes
  while (!*exit_flag_ptr_)
  {
    auto time_start = std::chrono::high_resolution_clock::now();
    int data_bytes_read = read(joy_fd_, event_, sizeof(*event_));
    if (data_bytes_read > 0)
    {
      joy_msg_.header.stamp = GetTimestampNow();
      event_count_++;
      publish_state = JoyEventCheck(*event_);
      if (publish_state > 0) joy_msg_.data_updated = true;
    }
#ifndef BUILD_WITHOUT_ROS
    if (expose_topics_ && publish_state > 0)
    {
      sensor_msgs::Joy joy_msg_ros;
      joy_msg_ros.header.stamp = ros::Time::now();
      joy_msg_ros.axes.insert(joy_msg_ros.axes.end(), joy_msg_.axes.begin(),
                              joy_msg_.axes.end());
      joy_msg_ros.buttons.insert(joy_msg_ros.buttons.end(),
                                 joy_msg_.buttons.begin(),
                                 joy_msg_.buttons.end());
      pub_.publish(joy_msg_ros);
      pub_count_++;
      publish_state = -1;
    }
#endif
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed_time =
        time_end - time_start;
    usleep(2000 - static_cast<int>(elapsed_time.count()));
  }
  // End of joystick open loop.
  close(joy_fd_);
}
// clean exit
void Joystick::JoyExit() {}
void Joystick::SignalHandler(int signum)
{
  printf("joy_node shut down.\n");
  JoyExit();
}
Joystick::~Joystick()
{
  if (joy_stick0_.joinable())
  {
    joy_stick0_.join();
  }
  // delete event_;
}
