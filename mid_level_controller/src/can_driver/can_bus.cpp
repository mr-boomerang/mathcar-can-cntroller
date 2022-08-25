// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_bus.cpp
 * @brief transreceiver for can. This file can be used to send and recieve
 * messages over can given that it is setup already and store commands in a log
 * file
 * @author Divyanshu Goel
 * @date 2017-02-16 (yyyy-mm-dd)
 **/
#include <can_driver/can_bus.h>

#include <algorithm>
#include <string>
#include <vector>

/// open up a can port
int CanBus::open_port(const std::string port, const std::string logfilepath,
                      bool log_enabler, const int *exit_flag_arg_)
{
  if (*exit_flag_arg_) return (-1);
  log_enabled = log_enabler;
  struct ifreq ifr;
  struct sockaddr_can addr;
  /* open socket */
  soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (soc < 0)
  {
    PE$ ct_red("Error Opening Socket connection") pendl;
    throw socketex;
    return (-1);
  }
  /* update the port index in ioctl*/
  addr.can_family = AF_CAN;
  snprintf(ifr.ifr_name, port.size() + 1, port.c_str());
  if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
  {
    PE$ ct_red(
        "Error IOCTL Socket connection. Please Check if Interface is "
        "up and running.") pendl;
    throw IOCTLex;
    return (-1);
  }
  addr.can_ifindex = ifr.ifr_ifindex;
  fcntl(soc, F_SETFL, O_NONBLOCK);
  /* bind the socket onto a can bus interface*/
  if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    std::cerr << "Error Binding Socket connection to can interface\n"
              << std::endl;
    throw connectex;
    return (-1);
  }
  /// write to log file
  if (log_enabled) log_file.open_file(logfilepath);
  exit_flag_pointer_ = exit_flag_arg_;
  return (0);
}
// to convert canfd_frame to can_frame
can_frame CanBus::copy_fd_frame_to_frame(struct canfd_frame *frame)
{
  can_frame temp;
  temp.can_id = frame->can_id;
  temp.can_dlc = frame->len;
  temp.data[0] = frame->data[0];
  temp.data[1] = frame->data[1];
  temp.data[2] = frame->data[2];
  temp.data[3] = frame->data[3];
  temp.data[4] = frame->data[4];
  temp.data[5] = frame->data[5];
  temp.data[6] = frame->data[6];
  temp.data[7] = frame->data[7];
  return (temp);
}
/// send data over port
int CanBus::send_port(struct canfd_frame *frame)
{
  if (*exit_flag_pointer_) return (-1);
  int retval;
  /// send the given can frame
  can_frame temp = copy_fd_frame_to_frame(frame);
  retval = write(soc, &temp, sizeof(struct can_frame));
  if (retval != sizeof(struct can_frame) && read_exit_flag != 1)
  {
    PE$ ct_red(
        "Failure in sending can frame. Please check connection , turn "
        "on gateway or set up can if connected by using: $ ip link") pendl;
    throw writeex;
    return (-1);
  }
  else
  {
    /// write to log file
    if (log_enabled) log_file.print_canfdframe_to_file(*frame);
    return (0);
  }
}
/// send data over port overloaded
int CanBus::send_port(struct can_frame *frame)
{
  if (*exit_flag_pointer_) return (-1);
  int retval;
  /// send the given can frame
  retval = write(soc, frame, sizeof(struct can_frame));
  if (retval != sizeof(struct can_frame) && read_exit_flag != 1)
  {
    PE$ ct_red(
        "Failure in sending can frame. Please check connection , turn "
        "on gateway or set up can if connected by using: $ ip link") pendl;
    throw writeex;

    return (-1);
  }
  else
  {
    /// write to log file
    if (log_enabled) log_file.print_canframe_to_file(*frame);
    return (0);
  }
}
/// receive data over port blocking call
void CanBus::read_port()
{
  if (*exit_flag_pointer_) return;
  struct canfd_frame frame_rd;
  /// allows me escape sigterm and others
  int recvbytes = 0;
  int read_can_port = 1;
  while (read_can_port && !read_exit_flag && !*exit_flag_pointer_)
  {
    struct timeval timeout = {1, 0};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(soc, &readSet);
    if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
    {
      if (!read_can_port)
      {
        break;
      }
      if (FD_ISSET(soc, &readSet))
      {
        recvbytes = read(soc, &frame_rd, sizeof(struct canfd_frame));
        if (recvbytes)
        {
          read_can_port = 1;
          /// write to log file
          if (log_enabled) log_file.print_canfdframe_to_file(frame_rd);
          frame_temp[frame_count] = frame_rd;
          frame_count++;
          if (frame_count >= max_no_frames)
          {
            frame_count = 0;
            new_incoming_data = 1;
          }
        }
      }
    }
    read_exit_flag = *exit_flag_pointer_;
  }
  read_can_port = 0;
}
/// set filter on the port
int CanBus::set_filter(unsigned int can_id, unsigned int can_filter)
{
  struct can_filter read_filter;
  read_filter.can_id = can_id; /* SFF frame keep 0x700 and 0xF00*/
  read_filter.can_mask = can_filter;
  int rc;
  /// perform a socket operation to set the filter
  rc = setsockopt(soc, SOL_CAN_RAW, CAN_RAW_FILTER, &read_filter,
                  sizeof(read_filter));
  if (rc < 0)
  {
    PE$ ct_red(
        "Error in setting the filter. Either hardware does not support it "
        "or bus is not active.") pendl;
    return (-1);
  }
  return (1);
}
/// close the instance of port
int CanBus::close_port()
{
  read_exit_flag = 1;
  close(soc);
  if (log_enabled) log_file.close_file();
  return 0;
}
