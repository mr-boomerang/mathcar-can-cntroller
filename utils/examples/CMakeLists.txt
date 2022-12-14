cmake_minimum_required(VERSION 2.8)
project(UtilsExample)

string(TOLOWER ${PROJECT_NAME} EXE_NAME)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

#eg: pkg_check_modules(OPENCV REQUIRED opencv)
#INCLUDE(FindPkgConfig)
#pkg_check_modules()
#To add header OPENCV_INCLUDE_DIRS and libs OPENCV_LDFLAGS
#find_package(Pangolin QUIET)
find_package(utils REQUIRED)

if(UTILS_USE_OPENCV)
  find_package(OpenCV REQUIRED)
  find_package(Pangolin QUIET)
  if(Pangolin_FOUND)
    message("Pangolin Found, adding img_gui_demo.")
    set(HEADERS ${Pangolin_INCLUDE_DIRS} /usr/include/eigen3/) 
    set(LIBS ${Pangolin_LIBRARIES}) 
  endif()
endif()

# eg. set(SRC main.cpp)
#set(SRC ./mode_print_demo.cpp) 

# eg. set(HEADERS ${CMAKE_SOURCE_DIR}/include ${OPENCV_INCLUDE_DIRS} /usr/include/eigen3/)
set(HEADERS ${HEADERS} ${utils_INC_DIRS}) 

# eg. set(LIBS ${OPENCV_LDFLAGS} /usr/lib/libblas.so)
set(LIBS ${LIBS} ${utils_LIBRARIES}) 

include_directories(${HEADERS})

set(EXE1 mode_print_demo)
set(EXE2 color_demo)
set(EXE3 fs_utils_demo)
set(EXE4 str_util_demo)
set(EXE5 params_demo)
set(EXE6 gui_demo)
set(EXE7 pangolin_test)
set(EXE8 traj_demo)
set(EXE10 textview_demo)

add_executable(${EXE1} ./mode_print_demo.cpp) 
add_executable(${EXE2} ./color_demo.cpp) 
add_executable(${EXE3} ./fs_utils_demo.cpp) 
add_executable(${EXE4} ./str_utils_demo.cpp) 
add_executable(${EXE5} ./params_demo.cpp) 
add_executable(${EXE6} ./gui_demo.cpp) 
add_executable(${EXE7} ./pangolin_test.cpp) 
add_executable(${EXE8} ./traj_demo.cpp) 
add_executable(${EXE10} ./textview_demo.cpp) 

target_link_libraries(${EXE1} ${LIBS})
target_link_libraries(${EXE2} ${LIBS})
target_link_libraries(${EXE3} ${LIBS})
target_link_libraries(${EXE4} ${LIBS})
target_link_libraries(${EXE5} ${LIBS})
target_link_libraries(${EXE6} ${LIBS})
target_link_libraries(${EXE7} ${LIBS})
target_link_libraries(${EXE8} ${LIBS})
target_link_libraries(${EXE10} ${LIBS})

#if(UTILS_USE_OPENCV)
  ##find_package(Pangolin QUIET)
  #if(Pangolin_FOUND)
    #message("Pangolin Found, adding img_gui_demo.")
    #set(EXE6 img_gui_demo)
    #add_executable(${EXE6} ./img_gui_demo.cpp) 
    #include_directories(${Pangolin_INC})
    #target_link_libraries(${EXE6} ${LIBS})
  #endif()
#endif()
