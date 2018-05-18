cmake_minimum_required(VERSION 2.8.3)
project(makobot_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  joy
  message_generation
  dynamic_reconfigure
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Arm.srv
)

generate_messages()

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(joystick_remapper src/joystick_remapper.cpp)
#add_dependencies(joystick_remapper ${catkin_EXPORTED_TARGETS})
#target_link_libraries(joystick_remapper ${catkin_LIBRARIES})

add_executable(makobot_teleop src/makobot_teleop.cpp)
add_dependencies(makobot_teleop ${PROJECT_NAME}_generate_messages_cpp makobot_teleop_gencfg)
target_link_libraries(makobot_teleop ${catkin_LIBRARIES})

