// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file keyboard_node.cpp
 * @brief keyboard interface
 * @author Divyanshu Goel
 * @date 2018-05-02 (yyyy-mm-dd)
 **/
#include <interfaces/joy_node.h>

#include <string>
#include <vector>
/// to set joymsg update
void Keyboard::setJoyMsgUpdated(bool update_option)
{
  // set the option
  joy_msg_.SetUpdated(update_option);
}
/// to set debug enable option
void Keyboard::setDebugOption(bool debug_enable)
{
  // set the option
  debug_enable_ = debug_enable;
}
/// to expose Keyboard message
void Keyboard::setExposeTopicsOption(bool expose_topics_enable)
{
  expose_topics_ = expose_topics_enable;
}
/// to set joymsg update
bool Keyboard::getKeyMsgUpdated()
{
  // get the option
  return (joy_msg_.GetUpdated());
}
/// to set debug enable option
bool Keyboard::getDebugOption()
{
  // get the option
  return (debug_enable_);
}
/// to expose Keyboard message
bool Keyboard::getExposeTopicsOption()
{
  // get the option
  return (expose_topics_);
}
/// to return Keyboard message
char Keyboard::GetKeyboardMsg()
{
  // send the message
  return (key_pressed);
}
/// Opens Keyboard port, reads from port and publishes while node is
/// active
void Keyboard::KeyboardInit(const int *exit_flag_arg, bool debug_enable)
{
  exit_flag_ptr_ = exit_flag_arg;
  debug_enable_ = debug_enable;
// Parameters
#ifndef BUILD_WITHOUT_ROS

  if (expose_topics_)
  {
    ros::NodeHandle nh0;
    pub_ = nh0.advertise<Int64>("Key", 1);
  }
#endif
}
void Keyboard::KeyboardOpen()
{
  bool first_fault = true;
  keyboard_fd_ = 0;
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

  printf("Opened Keyboard.\n");
  open_ = true;
}
int Keyboard::KeyboardEventCheck(char key_pressed)
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
          joy_msg_.axes[i] = 0.0;
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
void Keyboard::JoyMonitorThd()
{
  joy_is_threaded_ = true;
  joy_stick0_ = std::thread(&Keyboard::JoyMonitor, this);
}
void Keyboard::JoyMonitor()
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
  // End of Keyboard open loop.
  close(joy_fd_);
}
// clean exit
void Keyboard::JoyExit() {}
void Keyboard::SignalHandler(int signum)
{
  printf("joy_node shut down.\n");
  JoyExit();
}
Keyboard::~Keyboard()
{
  if (joy_stick0_.joinable())
  {
    joy_stick0_.join();
  }
  // delete event_;
}
