cmake_minimum_required(VERSION 2.8.3)
project(skinsim_model)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  skinsim_msgs
  message_generation
  genmsg
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

#find_package(Protobuf REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
			    ${GAZEBO_INCLUDE_DIRS}
			    ${SDFormat_INCLUDE_DIRS} )
							
################################################
## Declare ROS messages, services and actions ##
################################################

# Build whatever you need here

# add_library(plane_joint SHARED src/plane_joint.cc)
# target_link_libraries(plane_joint ${GAZEBO_LIBRARIES})

# add_library(mass_joint SHARED src/mass_joint.cc)
# target_link_libraries(mass_joint ${GAZEBO_LIBRARIES})

# add_library(mass_valid_joint SHARED src/mass_valid_joint.cc)
# target_link_libraries(mass_valid_joint ${GAZEBO_LIBRARIES} yaml-cpp)

#add_library(PlaneJoint SHARED src/PlaneJoint.cc)
#target_link_libraries(PlaneJoint ${GAZEBO_LIBRARIES})

# add_library(SkinJointPlugin SHARED src/SkinJointPlugin.cc)
# target_link_libraries(SkinJointPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

# add_library(TactileSensorPlugin SHARED src/TactileSensorPlugin.cc)
# target_link_libraries(TactileSensorPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

catkin_package(
    CATKIN_DEPENDS message_runtime
    DEPENDS 
      gazebo_ros
      roscpp
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)
