cmake_minimum_required(VERSION 2.8.3)
project(skinsim_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  std_srvs
)

add_message_files(FILES
  SkinJointData.msg
  SkinJointDataArray.msg
  Joint1D.msg
  Joint1DArray.msg
  PointArray.msg
  ControllerType.msg
  TactileData.msg
)

add_service_files(DIRECTORY srv FILES
  Empty.srv
  SetController.srv
  GetLayout.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  std_srvs
)
