cmake_minimum_required(VERSION 2.8.3)

project(openni2_tracker)

find_package(catkin REQUIRED COMPONENTS geometry_msgs
  orocos_kdl
  roscpp
  roslib
  tf)

find_path(OpenNI2_INCLUDEDIR
  NAMES OpenNI.h
  HINTS $ENV{OPENNI2_INCLUDE})

find_library(OpenNI2_LIBRARY
  NAMES OpenNI2
  HINTS $ENV{OPENNI2_REDIST}
  PATH_SUFFIXES lib)

find_path(Nite2_INCLUDEDIR
  NAMES NiTE.h
  HINTS $ENV{NITE2_INCLUDE})

find_library(Nite2_LIBRARY
  NAMES NiTE2
  HINTS $ENV{NITE2_REDIST64}
  PATH_SUFFIXES lib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDEDIR}
  ${Nite2_INCLUDEDIR})

#message(OpenNI2_INCLUDEDIR: ${OpenNI2_INCLUDEDIR})
#message(OpenNI2_LIBRARY: ${OpenNI2_LIBRARY})
#message(Nite2_INCLUDEDIR: ${Nite2_INCLUDEDIR})
#message(Nite2_LIBRARY: ${Nite2_LIBRARY})

catkin_package()

add_executable(openni2_tracker src/openni2_tracker.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARY} ${Nite2_LIBRARY})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(openni2_tracker ${catkin_EXPORTED_TARGETS})
endif()
