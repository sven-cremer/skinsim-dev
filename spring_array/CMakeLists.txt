cmake_minimum_required(VERSION 2.8.3)
project(spring_array)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  message_generation
  genmsg
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
			    ${GAZEBO_INCLUDE_DIRS}
			    ${SDFormat_INCLUDE_DIRS} )
							
################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
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

generate_messages()
catkin_package( CATKIN_DEPENDS message_runtime )


include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)


# Build whatever you need here
add_library(spring_joint SHARED spring_joint.cc)
target_link_libraries(spring_joint ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

add_library(tactile_joint SHARED tactile_joint.cc)
target_link_libraries(tactile_joint ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

add_library(mass_joint SHARED mass_joint.cc)
target_link_libraries(mass_joint ${GAZEBO_LIBRARIES})

add_library(plane_joint SHARED plane_joint.cc)
target_link_libraries(plane_joint ${GAZEBO_LIBRARIES})

add_library(skin_joint SHARED skin_joint.cc)
target_link_libraries(skin_joint ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

add_library(mass_valid_joint SHARED mass_valid_joint.cc)
target_link_libraries(mass_valid_joint ${GAZEBO_LIBRARIES} yaml-cpp)


#find_package(Protobuf REQUIRED)
#target_link_libraries(central ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})
#target_link_libraries(central ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )
#add_dependencies(central spring_array_generate_message_cpp)


add_executable(central src/central.cpp)
target_link_libraries( central ${catkin_LIBRARIES} )


catkin_package(
    CATKIN_DEPENDS message_runtime
    DEPENDS 
      gazebo_ros
      roscpp
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)
