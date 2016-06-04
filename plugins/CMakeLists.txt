cmake_minimum_required(VERSION 2.8.3)
project(plugins)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

# Depend on system install of Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

include_directories(include ${GAZEBO_INCLUDE_DIRS}
                            ${SDFormat_INCLUDE_DIRS} )

include_directories ("${PROJECT_SOURCE_DIR}/../msgs/build")
link_directories($ENV{SKINSIM_PATH}/msgs/build)
							
# Build
add_library(MassJoint SHARED src/MassJoint.cc)
target_link_libraries(MassJoint ${GAZEBO_LIBRARIES})

#add_library(PlaneJoint SHARED src/PlaneJoint.cc)
#target_link_libraries(PlaneJoint ${GAZEBO_LIBRARIES} msgs )

add_library(SkinJointPlugin SHARED src/SkinJointPlugin.cc)
target_link_libraries(SkinJointPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs )

add_library(SkinJointPlugin_V2 SHARED src/SkinJointPlugin_V2.cc)
target_link_libraries(SkinJointPlugin_V2 ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs pthread)

add_library(SkinJointForceDistributionPlugin SHARED src/SkinJointForceDistributionPlugin.cc)
target_link_libraries(SkinJointForceDistributionPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs pthread)

add_library(VisualizeContactPlugin SHARED src/VisualizeContactPlugin.cc)
target_link_libraries(VisualizeContactPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs )

add_library(CylinderTrajectoryPlugin SHARED src/CylinderTrajectoryPlugin.cc)
target_link_libraries(CylinderTrajectoryPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs )

#add_library(TactileSensorPlugin SHARED src/TactileSensorPlugin.cc)
#target_link_libraries(TactileSensorPlugin ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES} yaml-cpp msgs )
