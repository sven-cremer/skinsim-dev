cmake_minimum_required(VERSION 2.8.3)
project(skinsim_msgs)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

find_package( catkin REQUIRED COMPONENTS
              message_generation
              genmsg
              std_msgs                   )


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files( FILES
   tactileData.msg
   controllerData.msg
   conciseData.msg
   inputData.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs )
catkin_package( CATKIN_DEPENDS message_runtime )