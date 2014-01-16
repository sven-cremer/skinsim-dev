cmake_minimum_required(VERSION 2.8.3)
project(spring_array)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscpp
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
							${GAZEBO_INCLUDE_DIRS}
							${SDFormat_INCLUDE_DIRS} )

# Build whatever you need here
add_library(spring_joint SHARED spring_joint.cc)
target_link_libraries(spring_joint  ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

add_library(mass_joint SHARED mass_joint.cc)
target_link_libraries(mass_joint ${GAZEBO_LIBRARIES})

add_library(plane_joint SHARED plane_joint.cc)
target_link_libraries(plane_joint ${GAZEBO_LIBRARIES})

add_library(tactile_joint SHARED tactile_joint.cc)
target_link_libraries(tactile_joint ${GAZEBO_LIBRARIES})

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)
add_executable(central central.cc)
target_link_libraries(central ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

catkin_package(
    DEPENDS 
      gazebo_ros
      roscpp
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)


