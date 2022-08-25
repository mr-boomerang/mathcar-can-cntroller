// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file logger.cpp
 * @brief print can messages to log file given location including the name of
 * file preferably absolute path.
 * @author Divyanshu Goel
 * @date 2017-02-06 (yyyy-mm-dd)
 */

#include <can_driver/logger.h>

#include <string>
/// This method will be used to create empty log file and enter in that file
/// that log has started.
void Logger::open_file(const std::string filepath)
{
  myfile.open(filepath);
  if (myfile.is_open())
  {
    // get current time
    auto time_now_data = std::chrono::system_clock::now();
    std::time_t time_t1 = std::chrono::system_clock::to_time_t(time_now_data);
    std::tm *now = std::localtime(&time_t1);
    myfile << "********* Log Start [";
    myfile << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
           << now->tm_mday << " " << (now->tm_hour) << ":" << (now->tm_min)
           << ":" << (now->tm_sec) << "] *********\n";
  }
  else
  {
    PE$ ct_red("File Handle create failed.") pendl;
  }
}
// write to file flush
void Logger::flush() { myfile.flush(); }
/// This method will be used to print a single can fd frame to the log file.
void Logger::print_canfdframe_to_file(canfd_frame temp)
{
  auto start = std::chrono::high_resolution_clock::now();
  double ms = start.time_since_epoch().count() / 1000000L;
  myfile << ms;
  myfile << " ";
  myfile << temp.can_id << " " << static_cast<unsigned int>(temp.len) << " ";
  for (int i = 0; i < temp.len; ++i)
  {
    if (i < temp.len - 1)
      myfile << static_cast<unsigned int>(temp.data[i]) << " ";
    else
      myfile << static_cast<unsigned int>(temp.data[i]);
  }
  myfile << "\n";
}
/// This method will be used to print a single can frame to the log file.
void Logger::print_canframe_to_file(can_frame temp)
{
  auto start = std::chrono::high_resolution_clock::now();
  double ms = start.time_since_epoch().count() / 1000000L;
  myfile << ms;
  myfile << " ";
  myfile << temp.can_id << " " << static_cast<unsigned int>(temp.can_dlc)
         << " ";
  for (int i = 0; i < temp.can_dlc; ++i)
  {
    if (i < temp.can_dlc - 1)
      myfile << static_cast<unsigned int>(temp.data[i]) << " ";
    else
      myfile << static_cast<unsigned int>(temp.data[i]);
  }
  myfile << "\n";
}
/// This method will be used to print a single char array to the log file.
void Logger::print_strmessage_to_file(const std::string data)
{
  myfile << data;
}
/// This method will be used to print a single char array to the log file.
void Logger::print_string_message_to_file(std::string data) { myfile << data; }
/// This method will be used to print a new line in the log file.
void Logger::print_newline_to_file() { myfile << "\n"; }
/// This method will be used to close the file safely after entering the closing
/// time and date.
void Logger::close_file()
{
  // get current time
  auto time_now_data = std::chrono::system_clock::now();
  std::time_t time_t1 = std::chrono::system_clock::to_time_t(time_now_data);
  std::tm *now = std::localtime(&time_t1);
  myfile << "********* Log Ends [";
  myfile << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
         << now->tm_mday << " " << (now->tm_hour) << ":" << (now->tm_min) << ":"
         << (now->tm_sec) << "] *********\n";
  myfile.close();
}
