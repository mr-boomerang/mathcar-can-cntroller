cmake_minimum_required(VERSION 2.8.3)
project(utils)

option(EXPORT_UTILS_CORE "Export utils-config.cmake for projects outside ROS" OFF)

set(BUILD_WITH_ROS 0)
if(EXISTS ${CATKIN_ENV})
  set(BUILD_WITH_ROS 1)
  find_package(catkin QUIET COMPONENTS roscpp std_msgs genmsg tf visualization_msgs )
  add_definitions(-DBUILT_WITH_ROS)

  ## Declare ROS messages and services
  add_service_files(FILES ParamInfo.srv)
  add_service_files(FILES ParamSet.srv)
  add_service_files(FILES ParamGet.srv)
  add_service_files(FILES OutputInfo.srv)

  ## Generate added messages and services
  generate_messages(DEPENDENCIES std_msgs)

  option(UTILS_USE_BOOST "Use boost and not use c++11 features." OFF)
  if(NOT UTILS_USE_BOOST)
    string(SUBSTRING $ENV{ROS_DISTRO} 0 1 ROS_DIST)
    string(COMPARE LESS ${ROS_DIST} "i" UTILS_USE_BOOST)
  endif()

  if(UTILS_USE_BOOST)
    find_package(Boost COMPONENTS chrono REQUIRED)
    add_definitions(-DBOOST_NOT_CPP11)
    if(Boost_MINOR_VERSION GREATER 46)
      message("Using TicToc since boost >1.46 is available.")
      add_definitions(-DUTILS_USE_TICTOC)
    endif()
  endif()

else()
  set(EXPORT_UTILS_CORE ON)
endif()

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

# Original Util package stuff.
set(CORE_HDRS "")
add_subdirectory(src) 
if(NOT ${BUILD_WITH_ROS})
  return()
endif()

## Declare a catkin package
catkin_package(INCLUDE_DIRS ${CORE_HDRS}
  LIBRARIES ${LIB_NAME}
  CFG_EXTRAS ${CFG_FILE_NAME})

## Build ros package
#include_directories(include ${catkin_INCLUDE_DIRS} ${CORE_HDRS})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})

### Build service client and server
#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

# Configure file to set dir paths.
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/config.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h"
  )

