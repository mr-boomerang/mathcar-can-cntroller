project(utils)

string(TOLOWER ${PROJECT_NAME} LIB_NAME_TEMP)
set(LIB_NAME "${LIB_NAME_TEMP}" CACHE INTERNAL STRING)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs")

option(UTILS_USE_OPENCV "Use opencv while compiling, used for color.hpp" ON)
option(UTILS_USE_STL
  "If ON then all the stl headers are inculded, i mean ALL" OFF)
option(UTILS_USE_GFLAGS
  "If ON then all the stl headers are inculded, i mean ALL" ON)
option(UTILS_GUI "Compile gui module" ON)

# eg. set(SRC main.cpp)
#keep them in order of dependency. miniflags is used by mode_printing,
#mode_printing is used by fs_utils, otherwise there would be seg fault.
set(SRC ./miniflags.cpp ./mode_printing.cpp ./fs_utils.cpp
	./str_utils.cpp ./params.cpp )

# eg. set(HEADERS ${CMAKE_SOURCE_DIR}/include ${OPENCV_INCLUDE_DIRS} /usr/include/eigen3/)
if(UTILS_USE_GFLAGS)
  find_package(gflags REQUIRED)
else()
  add_definitions(-DUTILS_AVOID_GFLAGS)
endif()

if(UTILS_USE_OPENCV)
  find_package(OpenCV REQUIRED)
  add_definitions(-DUTILS_USE_OPENCV)
endif()

if(UTILS_USE_STL)
  add_definitions(-DUTILS_USE_STL)
endif()

# if(UTILS_GUI)
#   find_package(Pangolin REQUIRED)
#   set(SRC ${SRC} ./gui/view_graph.cpp ./gui/view_node.cpp
#     ./gui/gui_handler.cpp
#     ./gui/graph_objects.cpp
#     ./gui/objects_3d.cpp
#     ./gui/objects_2d.cpp
#     ./gui/radio_button.cpp
#     ./tinyxml2.cpp
#     ./gui/data_plotter_object.cpp
# ./gui/textView.cpp	)
# endif()

#current_src/../ because it works normally and with catkin
set(HEADERS ${HEADERS} ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(HEADERS ${HEADERS} ${gflags_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} #${OPENCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

# eg. set(LIBS ${OPENCV_LDFLAGS} /usr/lib/libblas.so)
set(LIBS gflags #${OPENCV_LDFLAGS}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${Pangolin_LIBRARIES})

if(NOT UTILS_USE_BOOST)
  # set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} --std=c++11)
  set(CMAKE_CXX_STANDARD 14) 
  add_definitions(-DUTILS_USE_TICTOC)
endif()

# Add TARGET utils
include_directories(${HEADERS})
add_library(${LIB_NAME} SHARED ${SRC})
target_link_libraries(${LIB_NAME} ${LIBS})
if(BUILD_WITH_ROS)
  set(CORE_HDRS ${HEADERS} PARENT_SCOPE)
  add_dependencies(${LIB_NAME} ${LIB_NAME}_gencpp)
  target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})
endif()


#if(EXPORT_UTILS_CORE)
#install(TARGETS ${LIB_NAME} EXPORT ${PROJECT_NAME}-targets DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
export(TARGETS ${LIB_NAME} FILE
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-config.cmake.in
  #${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DEIRECTORY}/${PROJECT_NAME}-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  @ONLY)

set(CFG_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    PARENT_SCOPE)

export(PACKAGE ${PROJECT_NAME})
#endif()
 install(TARGETS ${LIB_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
