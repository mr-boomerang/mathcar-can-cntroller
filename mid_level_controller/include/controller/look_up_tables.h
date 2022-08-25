// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file look_up_tables.h
 * @brief look up table for controller
 * @author Divyanshu Goel
 * @date 2017-05-08 (yyyy-mm-dd)
 */

#ifndef CONTROLLER_LOOK_UP_TABLES_H
#define CONTROLLER_LOOK_UP_TABLES_H
/// size of the throttle lookup table at normal gear
const int throttle_value_count_orig = 27;
/// size of the throttle lookup table at reverse gear
const int throttle_value_reverse_count_orig = 27;
/// size of the throttle lookup table at boost gear
const int throttle_value_boost_count_orig = 27;
/// size of the brake lookup table
const int brake_value_count_orig = 19;
/// throttle values lookup table at normal gear
// clang-format off
const int throttle_values_orig[throttle_value_count_orig] =
{
    0,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
    17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 20, 20, 21
};

/// throttle values lookup table at reverse gear
const int throttle_reverse_values_orig[throttle_value_reverse_count_orig] =
{
    0,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
    17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 20, 20, 21
};

/// throttle values lookup table at boost gear
const int throttle_boost_values_orig[throttle_value_boost_count_orig] =
{
    0,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
    17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 20, 20, 21
};
// clang-format off
/// lookup table braking
const int
    braking_values_lookup_orig[brake_value_count_orig][brake_value_count_orig] =
        {
          {
          0, 42, 42, 46, 46, 46, 46, 46, 50, 50, 50, 50, 40, 54, 54, 54, 54, 54,
            54
        }
        ,
        {
           0, 0, 34, 38, 42, 42, 42, 46, 46, 46, 46, 50, 50, 50, 54, 54, 50, 50,
           50
        }
        ,
         {
           0, 0, 0, 30, 38, 42, 42, 42, 42, 46, 42, 50, 50, 50, 54, 50, 50, 50,
           50
        }
        ,
         {
           0, 0, 0, 0, 34, 38, 38, 42, 42, 46, 42, 50, 50, 50, 54, 50, 50, 50 ,
          50
        }
        ,
         {
           0, 0, 0, 0, 0, 34, 38, 38, 38, 46, 42, 50, 46, 50, 50, 50, 50, 50,
            50
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 34, 34, 38, 38, 38, 50, 46, 50, 46, 46, 46, 46, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 34, 34, 38, 38, 46, 42, 46, 50, 46, 46, 46, 19
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38, 46, 38, 46, 50, 42, 46, 46, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 34, 46, 42, 46, 46, 46, 46, 46, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 42, 38, 42, 42, 42, 42, 42, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 38, 38, 38, 42, 38, 42, 42, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38, 38, 42, 42, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38, 38, 38, 46
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38, 38, 42
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38, 38
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38, 38
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 38
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34
        }
        ,
         {
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         }
       };
// clang-format on
#endif  // CONTROLLER_LOOK_UP_TABLES_H
