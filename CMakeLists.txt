cmake_minimum_required(VERSION 2.8.3)
project(ebimu_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ebimu_serial src/ebimu_serial.cpp)

target_link_libraries(ebimu_serial
  ${catkin_LIBRARIES}
)

