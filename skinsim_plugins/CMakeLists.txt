cmake_minimum_required(VERSION 2.8.3)
project(skinsim_plugins)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

# Depend on system install of Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

#find_package(Protobuf REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS}
			    ${SDFormat_INCLUDE_DIRS} )
							
# Build

add_library(plane_joint SHARED src/plane_joint.cc)
target_link_libraries(plane_joint ${GAZEBO_LIBRARIES})

add_library(mass_joint SHARED src/mass_joint.cc)
target_link_libraries(mass_joint ${GAZEBO_LIBRARIES})

add_library(mass_valid_joint SHARED src/mass_valid_joint.cc)
target_link_libraries(mass_valid_joint ${GAZEBO_LIBRARIES} yaml-cpp)

#add_library(PlaneJoint SHARED src/PlaneJoint.cc)
#target_link_libraries(PlaneJoint ${GAZEBO_LIBRARIES})

add_library(SkinJointPlugin SHARED src/SkinJointPlugin.cc)
target_link_libraries(SkinJointPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )

add_library(TactileSensorPlugin SHARED src/TactileSensorPlugin.cc)
target_link_libraries(TactileSensorPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp )
