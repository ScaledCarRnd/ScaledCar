cmake_minimum_required(VERSION 2.8.3)
project(fake_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS
    roscpp
)

## Declare a C++ executable
add_executable(fake_laser_publisher src/fake_laser_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fake_laser_publisher
   ${catkin_LIBRARIES}
 )
