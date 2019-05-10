cmake_minimum_required(VERSION 2.8.3)
project(gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set( sources
  src/main.cpp
  src/t_serial.cpp
  src/CoordinateConv.cpp
  src/Sensor_GPS.cpp
)

set( headers
  src/t_serial.h
  src/CoordinateConv.h
  src/Sensor_GPS.h
)

catkin_package(
  LIBRARIES gps
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${headers} ${sources})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
