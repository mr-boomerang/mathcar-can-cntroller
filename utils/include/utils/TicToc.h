/****************************************************************************
 **
 ** File: TicToc.h
 ** **
 ** Copyright © 2015, Uurmi Systems                                         **
 ** All rights reserved.                                                    **
 ** http://www.uurmi.com **
 ** **
 ** All information contained herein is property of Uurmi Systems           **
 ** unless otherwise explicitly mentioned.                                  **
 ** **
 ** The intellectual and technical concepts in this file are proprietary    **
 ** to Uurmi Systems and may be covered by granted or in process national   **
 ** and international patents and are protect by trade secrets and          **
 ** copyright law.                                                          **
 ** **
 ** Redistribution and use in source and binary forms of the content in     **
 ** this file, with or without modification are not permitted unless        **
 ** permission is explicitly granted by Uurmi Systems.                      **
 ** **
 ****************************************************************************/
#pragma once

#if __GNUC__ >= 5
#define GCC5
#endif

#include "boostd.h"
#ifdef BOOST_NOT_CPP11
#else
#include <chrono>
#ifdef GCC5
#include <ctime>
#endif
#endif
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <utils/mode_printing.h>

#define NUM_OF_TIMERS 10
#define DEFAULT_CLOCK 0

// Change EMA_N to use that number of operations in impact zone
//#define EMA_N 10
#define EMA_ALPHA 0.18181818 // 2/(EMA_N+1) i.e. 2/11

#define tic utils::Tic(DEFAULT_CLOCK)
#define toc utils::Toc(DEFAULT_CLOCK)
#define clear_clock_stats                                                      \
  {                                                                            \
    utils::dur_ms_[DEFAULT_CLOCK] = 0.0;                                       \
    utils::ma_ms_[DEFAULT_CLOCK] = 0.0;                                        \
    utils::avg_fps_[DEFAULT_CLOCK] = 0.0;                                      \
  }

#define time_now utils_boostd::chrono::steady_clock::now()
#define highres_time_now utils_boostd::chrono::high_resolution_clock::now()

#define print_elapsed                                                          \
  DP$ "time taken[" << DEFAULT_CLOCK                                           \
                    << "]:" << utils::durationInmsec(DEFAULT_CLOCK) pendl

#define elapsed_msec utils::durationInmsec(DEFAULT_CLOCK)
#define elapsed_sec utils::durationInsec(DEFAULT_CLOCK)
#define elapsed_usec utils::durationInusec(DEFAULT_CLOCK)

#define avg_elapsed utils::moving_average(DEFAULT_CLOCK)
#define exp_avg_elapsed utils::exp_moving_average(DEFAULT_CLOCK)
#define tictoc_fps utils::avg_fps(DEFAULT_CLOCK)

#define ticn(n) utils::Tic(n)
#define tocn(n) utils::Toc(n)
#define clear_clock_statsn(n)                                                  \
  {                                                                            \
    utils::dur_ms_[n] = 0.0;                                                   \
    utils::ma_ms_[n] = 0.0;                                                    \
    utils::avg_fps_[n] = 0.0;                                                  \
  }

#define print_elapsedn(n)                                                      \
  DP$ "time taken[" << n << "]:" << utils::durationInmsec(n) pendl
#define elapsed_msecn(n) utils::durationInmsec(n)
#define elapsed_secn(n) utils::durationInsec(n)
#define elapsed_usecn(n) utils::durationInusec(n)

#define avg_elapsedn(n) utils::moving_average(n)
#define exp_avg_elapsedn(n) utils::exp_moving_average(n)
#define tictoc_fpsn(n) utils::avg_fps(n)

namespace utils {

typedef utils_boostd::chrono::steady_clock::time_point chrono_tp;
typedef utils_boostd::chrono::high_resolution_clock::time_point
    highres_chrono_tp;

/*------------------------Tic Toc start---------------------------------------*/
// don't use this variables directly, just don't.
static chrono_tp tic_[NUM_OF_TIMERS];
static chrono_tp toc_[NUM_OF_TIMERS];
static double dur_ms_[NUM_OF_TIMERS]; // ms is for milliseconds
// ma is moving average, ms is for milliseconds
static double ma_ms_[NUM_OF_TIMERS] = {0.0};
static double ema_ms_[NUM_OF_TIMERS] = {0.0};
// if update_mark_ is 0 then duration has been updated but ma hasn't been,
// if it's 1 then fps has been updated by moving_average function,
// if it's 2 then update fps has been read.
// if it's 3 then fps has been updated by exp_moving_average function
static int update_mark_[NUM_OF_TIMERS];
static double avg_fps_[NUM_OF_TIMERS] = {0.0};
static int count_[NUM_OF_TIMERS] = {0};

static inline double moving_average(int n = 0) {

  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;

  if (update_mark_[n] == 1 || update_mark_[n] == 2)
    return ma_ms_[n];

  if (ma_ms_[n] == 0.0) { // first time.
    ma_ms_[n] = dur_ms_[n];
  } else {
    // new average = ((old count * old data) + next data) / next count
    // new average = old average + (next data - old average) / next count
    // using the second formulae.
    // count is increased by toc().
    ma_ms_[n] = ma_ms_[n] + (dur_ms_[n] - ma_ms_[n]) / (double)count_[n];
  }
  avg_fps_[n] = 1000.0 / ma_ms_[n];
  update_mark_[n] = 1;

  return ma_ms_[n];
}

static inline double exp_moving_average(int n = 0) {

  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;

  if (update_mark_[n] == 2 || update_mark_[n] == 3)
    return ema_ms_[n];

  if (ema_ms_[n] == 0.0) { // first time.
    ema_ms_[n] = dur_ms_[n];
  } else {
    // new exp average= (next data * multiplier) + (old exp average *
    // (1-multiplier))
    ema_ms_[n] = (dur_ms_[n] * EMA_ALPHA) + (ema_ms_[n] * (1.0 - EMA_ALPHA));
  }
  avg_fps_[n] = 1000.0 / ema_ms_[n];
  update_mark_[n] = 3;

  return ema_ms_[n];
}

static inline double avg_fps(int n = 0) {
  if (update_mark_[n] == 1 || update_mark_[n] == 2 || update_mark_[n] == 3) {
    // 2 avg_fps_ has been read already.
    // 1 is avg_fps_ is written by moving_average but not read yet.
    // 3 is avg_fps_ is written by exp_moving_average but not read yet
  } else if (update_mark_[n] == 0) {
    // 0 is avg_fps_ is not updated after duration has been updated.
    moving_average(n);
    exp_moving_average(n);
  } else {
    PW$ "update_mark_(" << ct_vred(update_mark_[n])
                        << ") is weird state for clock number: "
                        << ct_red(n) pendl;
    return -1.0;
  }
  update_mark_[n] = 2;
  return avg_fps_[n];
}

static inline double durationInmsec(int n = 0) {
  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;
  utils_boostd::chrono::duration<double, utils_boostd::milli> d =
      toc_[n] - tic_[n];
  dur_ms_[n] = d.count();
  update_mark_[n] = 0;
  //  count_[n]++;
  return dur_ms_[n];
}

static inline double durationInsec(int n = 0) {
  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;
  utils_boostd::chrono::duration<double> d = toc_[n] - tic_[n];
  return d.count();
}

static inline double durationInusec(int n = 0) {
  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;
  utils_boostd::chrono::duration<double, utils_boostd::micro> d =
      toc_[n] - tic_[n];
  return d.count();
}

static inline void Tic(int n = 0) {
  if (n < 0 || n >= NUM_OF_TIMERS)
    return;
  tic_[n] = utils_boostd::chrono::steady_clock::now();
}

static inline double Toc(int n = 0) {
  if (n < 0 || n >= NUM_OF_TIMERS)
    return -1.0;
  toc_[n] = utils_boostd::chrono::steady_clock::now();
  count_[n]++;
  return durationInmsec(n);
}
/*-------------------------Tic Toc end----------------------------------------*/

inline long sec_since_epoch(std::chrono::system_clock::time_point tm) {
  auto dur =
      std::chrono::duration_cast<std::chrono::seconds>(tm.time_since_epoch());
  return dur.count();
}
inline long msec_since_epoch(std::chrono::system_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long usec_since_epoch(std::chrono::system_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long nsec_since_epoch(std::chrono::system_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(
      tm.time_since_epoch());
  return dur.count();
}

inline long sec_since_epoch(std::chrono::steady_clock::time_point tm) {
  auto dur =
      std::chrono::duration_cast<std::chrono::seconds>(tm.time_since_epoch());
  return dur.count();
}
inline long msec_since_epoch(std::chrono::steady_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long usec_since_epoch(std::chrono::steady_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long nsec_since_epoch(std::chrono::steady_clock::time_point tm) {
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(
      tm.time_since_epoch());
  return dur.count();
}

/*
inline long sec_since_epoch(std::chrono::high_resolution_clock::time_point tm) {
  auto dur =
      std::chrono::duration_cast<std::chrono::seconds>(tm.time_since_epoch());
  return dur.count();
}
inline long msec_since_epoch(std::chrono::high_resolution_clock::time_point tm)
{
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long usec_since_epoch(std::chrono::high_resolution_clock::time_point tm)
{
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(
      tm.time_since_epoch());
  return dur.count();
}
inline long nsec_since_epoch(std::chrono::high_resolution_clock::time_point tm)
{
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(
      tm.time_since_epoch());
  return dur.count();
}
*/

#ifdef GCC5

static inline std::string date_time_now() {
  std::time_t timet =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  std::stringstream ss;
  ss << std::put_time(std::localtime(&timet), "%Y%m%d_%H%M%S");
  return ss.str();
}

static inline std::string
date_time(std::chrono::time_point<std::chrono::system_clock> tm) {
  std::time_t timet = std::chrono::system_clock::to_time_t(tm);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&timet), "%Y%m%d_%H%M%S");
  return ss.str();
}
#else
static inline std::string date_time_now() {
  std::time_t timet =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  char buf[100];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&timet));
  return buf;
}

static inline std::string
date_time(std::chrono::time_point<std::chrono::system_clock> tm) {
  std::time_t timet = std::chrono::system_clock::to_time_t(tm);

  char buf[100];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&timet));
  return buf;
}
#endif

} // namespace utils
