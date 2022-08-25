cmake_minimum_required(VERSION 2.8.3)
project(mid_level_controller)
# add_definitions(-std=c++11)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 14) 
## only with ROS
set(BUILD_ADDONS 1)


## can library Sources
set(can_lib_SRCS
    src/can_driver/can_bus.cpp
    src/can_driver/can_receiver.cpp
    src/can_driver/can_transmitter.cpp
    src/can_driver/can_yaml_parser.cpp
    src/can_driver/logger.cpp
    src/can_driver/can_transceiver.cpp
    include/can_driver/can_struct.h
)

## teleoperation Sources
set(teleop_SRCS
  src/interfaces/joy_node.cpp
  src/interfaces/interface.cpp
  src/interfaces/peripherals/keyboard.cpp
  src/interfaces/peripherals/redgear.cpp
  src/interfaces/peripherals/sidewinder.cpp
  src/interfaces/peripherals/xbox.cpp
  src/controller/common_utils.cpp
  include/interfaces/joy_struct.h
  include/interfaces/joy_inputs.h
)

## controller Sources
set(controller_SRCS
  src/controller/mm_controller.cpp
  src/controller/common_utils.cpp
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/config.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h"
  )

# check for ROS
set(BUILD_WITHOUT_ROS 1)
if(EXISTS ${CATKIN_ENV})
  set(BUILD_WITHOUT_ROS 0)
  add_definitions("-DBUILD_WITH_ROS")

  # look for pangolin
  find_package(Pangolin QUIET)
  if(Pangolin_FOUND)
   add_definitions("-DPANGOLIN_FOUND")
   message("Pangolin found, adding dataplotter.")
  endif()
  # look for gflags
  find_package(gflags QUIET)
  if(gflags_FOUND)
    message("Gflags found.")
    set(UTILS_AVOID_GFLAGS 0)
  else()
    message("Gflags not found. Using miniflags from utils")
    add_definitions("-DUTILS_AVOID_GFLAGS")
  endif()

  # setup ros packages
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation sensor_msgs geometry_msgs utils roslint)
  add_message_files(FILES Can_Message.msg Callback_Msg.msg Can_Status_Message.msg)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(INCLUDE_DIRS include src add_ons LIBRARIES teleop can_bus controller CATKIN_DEPENDS roscpp std_msgs message_runtime)
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
  roslint_cpp()
  # if(Pangolin_FOUND)
  #   include_directories(include src add_ons ${catkin_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${utils_INCLUDE_DIRS})
  # else()
  # endif()
  include_directories(include src add_ons ${catkin_INCLUDE_DIRS} ${utils_INCLUDE_DIRS})
  ### Libraries
  ## Add can_bus library
  add_library(can_bus SHARED ${can_lib_SRCS})
  ## Add teleoperation library
  add_library(teleop SHARED ${teleop_SRCS})
  ## add controller library
  add_library(controller SHARED ${controller_SRCS} ${can_lib_SRCS})
  # linkages
  target_link_libraries(teleop rt pthread ${utils_LIBRARIES})
  target_link_libraries(can_bus rt pthread ${utils_LIBRARIES})
  target_link_libraries(controller rt pthread gflags ${catkin_LIBRARIES} ${utils_LIBRARIES})
  # add dependency
  add_dependencies(teleop mid_level_controller_generate_messages_cpp)
  add_dependencies(can_bus mid_level_controller_generate_messages_cpp)
  add_dependencies(controller mid_level_controller_generate_messages_cpp)

  # Executables
  # add tele-operation executable
  add_executable(teleop_car src/examples/interface_ros.cpp)
  add_executable(sidewinder_car src/examples/interface_sidewinder_ros.cpp)
  add_executable(mid_level_controller_node src/main/main_ros.cpp)
  add_executable(transceiver src/examples/transceiver_ros.cpp)
  # if(Pangolin_FOUND)
  #   add_executable(dataplotter add_ons/dataplotter.cpp)
  #   target_link_libraries(dataplotter ${catkin_LIBRARIES} ${Pangolin_LIBRARIES})
  #   add_dependencies(dataplotter mid_level_controller_generate_messages_cpp)
  # endif()
  # linkages
  target_link_libraries(teleop_car ${catkin_LIBRARIES} teleop)
  target_link_libraries(sidewinder_car ${catkin_LIBRARIES} teleop can_bus)
  target_link_libraries(mid_level_controller_node ${catkin_LIBRARIES} controller)
  target_link_libraries(transceiver ${catkin_LIBRARIES} controller)
  # dependencies
  add_dependencies(teleop_car mid_level_controller_generate_messages_cpp)
  add_dependencies(sidewinder_car mid_level_controller_generate_messages_cpp)
  add_dependencies(mid_level_controller_node mid_level_controller_generate_messages_cpp)
  add_dependencies(transceiver mid_level_controller_generate_messages_cpp)

  if(BUILD_ADDONS)
    add_executable(pedal_mapper add_ons/logging_scripts/pedal_logger.cpp)
    target_link_libraries(pedal_mapper ${catkin_LIBRARIES})
    add_dependencies(pedal_mapper mid_level_controller_generate_messages_cpp)

    add_executable(brake_mapper add_ons/logging_scripts/brake_logger.cpp)
    target_link_libraries(brake_mapper ${catkin_LIBRARIES})
    add_dependencies(brake_mapper mid_level_controller_generate_messages_cpp)
  endif()
else()
 # loading utils
 set(BUILD_WITHOUT_ROS 1)
 add_definitions("-DBUILD_WITHOUT_ROS")

 # look for utils
 find_package(utils REQUIRED)

 # look for pangolin
 find_package(Pangolin QUIET)
#  if(Pangolin_FOUND)
#    message("Pangolin found, adding dataplotter for mid level controller.")
#  endif()
 # look for gflags
 find_package(gflags QUIET)
 if(gflags_FOUND)
   message("Gflags found.")
   set(UTILS_AVOID_GFLAGS 0)
 else()
   message("Gflags not found. Using miniflags from utils")
   add_definitions("-DUTILS_AVOID_GFLAGS")
   set(UTILS_AVOID_GFLAGS 1)
 endif()

 ### Libraries
 ## Add can_bus library
 add_library(can_bus ${can_lib_SRCS})
 ## add controller library
 add_library(controller ${controller_SRCS} ${can_lib_SRCS})
 ## Add teleoperation library
 add_library(teleop ${teleop_SRCS})

 target_link_libraries(can_bus rt pthread ${utils_LIBRARIES})
 target_link_libraries(teleop rt pthread ${utils_LIBRARIES})
 if(GFLAGS_FOUND)
  target_link_libraries(controller rt pthread gflags ${utils_LIBRARIES})
 else()
  target_link_libraries(controller rt pthread ${utils_LIBRARIES})
 endif()

 ### executables
 # add tele-operation executable
 add_executable(teleop_car src/interfaces/interface.cpp)
 add_executable(sidewinder_car src/interfaces/interface_sidewinder.cpp)
 add_executable(mid_level_controller_node src/main/main.cpp)
 add_executable(transceiver src/examples/transceiver.cpp)
#  if(Pangolin_FOUND)
#   add_executable(dataplotter add_ons/dataplotter.cpp)
#   target_link_libraries(dataplotter ${Pangolin_LIBRARIES})
#  endif()
 # linkages
 target_link_libraries(teleop_car teleop)
 target_link_libraries(sidewinder_car teleop can_bus)
 target_link_libraries(mid_level_controller_node controller)
 target_link_libraries(transceiver controller)

endif()
### Documentation
find_package(Doxygen QUIET)
option(BUILD_DOCUMENTATION "Build MLC documentation and create the HTML based API documentation (requires Doxygen)")

if(${BUILD_DOCUMENTATION})
      if(NOT DOXYGEN_FOUND)
          message("Doxygen is needed to build the documentation.")
      else()
        set(MLC_DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/readme/Doxyfile)
        set(MLC_DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)
        set(MLC_DOXYGEN_LOG_FILE ${CMAKE_CURRENT_BINARY_DIR}/doxylog.txt)
        set(MLC_DOXY_DOC_DEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/readme/doxygenGeneratedDocs CACHE PATH "Path to the doxygen output")
        set(MLC_DOXY_DOC_INPUT_ROOT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path from where to start to parse")
        configure_file(${MLC_DOXYGEN_IN} ${MLC_DOXYGEN_OUT} @ONLY)
        MESSAGE(STATUS "Adding target doc to generate Documentation for mid level controller")
        add_custom_target(mid_level_controller_docs ALL
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen for mid level controller" VERBATIM
        )
      endif()
endif()
# make link for settings
execute_process(COMMAND ln -s ${CMAKE_CURRENT_SOURCE_DIR}/cfg/settings_default.cfg ${CMAKE_CURRENT_SOURCE_DIR}/cfg/settings.cfg
OUTPUT_QUIET
ERROR_QUIET)

# if(Pangolin_FOUND)
#   install(TARGETS
#      can_bus
#      teleop
#      controller
#      teleop_car
#      mid_level_controller_node
#      transceiver
#      dataplotter
#      brake_mapper
#      pedal_mapper
#      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
# else()
# endif()
install(TARGETS
   can_bus
   teleop
   controller
   teleop_car
   mid_level_controller_node
   transceiver
   brake_mapper
   pedal_mapper
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
 include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY
  launch
  yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  cfg/settings_default.cfg
  DESTINATION ${CMAKE_INSTALL_PREFIX}/../cfg/${PROJECT_NAME}
)
install(FILES
  yaml/control_messages.yaml
  yaml/status_messages.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/../cfg/${PROJECT_NAME}/yaml
)
install(TARGETS teleop can_bus controller
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(PROGRAMS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
