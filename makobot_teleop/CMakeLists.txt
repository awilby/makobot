cmake_minimum_required(VERSION 2.8.3)
project(makobot_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  joy
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/teleop_joy.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(joystick_remapper src/joystick_remapper.cpp)
add_dependencies(joystick_remapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_remapper ${catkin_LIBRARIES})

add_executable(teleop_joy src/teleop_joy.cpp)
add_dependencies(teleop_joy ${catkin_EXPORTED_TARGETS} makobot_teleop_gencfg)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

