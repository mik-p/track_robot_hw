cmake_minimum_required(VERSION 2.8.3)
project(track_robot_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES track_robot_hw
  CATKIN_DEPENDS roscpp hardware_interface controller_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/track_robot_hw_node.cpp
# )

add_executable(${PROJECT_NAME}_node
  src/track_robot_hw_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )
