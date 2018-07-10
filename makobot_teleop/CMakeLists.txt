cmake_minimum_required(VERSION 2.8.3)
project(makobot_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  message_generation
  dynamic_reconfigure
  mavros_msgs
  makobot_bridge
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/xbox_joy.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(makobot_teleop src/makobot_teleop.cpp)
add_dependencies(makobot_teleop ${PROJECT_NAME}_generate_messages_cpp makobot_teleop_gencfg)
target_link_libraries(makobot_teleop ${catkin_LIBRARIES})
