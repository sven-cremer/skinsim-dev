cmake_minimum_required(VERSION 2.8.3)
project(model)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

set (MIN_OGRE_VERSION 1.7.4 CACHE INTERNAL "Ogre version requirement" FORCE)

# find_package( libyaml-cpp REQUIRED )

# Depend on system install of Gazebo
FIND_PACKAGE( PkgConfig REQUIRED )
find_package( gazebo    REQUIRED )
find_package( Eigen3    REQUIRED )
find_package( OGRE      REQUIRED )
find_package( Boost     COMPONENTS
                        system
                        filesystem
                        thread
                        chrono
                        REQUIRED )

include_directories ("${PROJECT_SOURCE_DIR}/../generator/include")
include_directories ("${PROJECT_SOURCE_DIR}/../plugins/include")

include_directories ( ${ODE_INCLUDE_DIRS}
                      ${OPENGL_INCLUDE_DIR}
                      ${OGRE_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${PROTOBUF_INCLUDE_DIR}
                      ${GAZEBO_INCLUDE_DIRS}
                      ${SDFormat_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}    )

link_directories( ${Boost_LIBRARY_DIRS} )

if(HAVE_BULLET)
  include_directories ( ${BULLET_INCLUDE_DIRS} )
  link_directories ( ${BULLET_LIBRARY_DIRS} )
endif()
if(HAVE_DART)
  include_directories ( ${DARTCore_INCLUDE_DIRS} )
  link_directories ( ${DARTCore_LIBRARY_DIRS} )
endif()

# ROS
find_package(roscpp REQUIRED)
find_package(skinsim_ros_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${skinsim_ros_msgs_INCLUDE_DIRS})

# Build library
add_library(SkinSimTestingFramework SHARED src/skinsim_testing_framework.cc)
target_link_libraries(SkinSimTestingFramework ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp msgs)

# Build
add_executable( generate_experiment_specification src/generate_experiment_specification.cc )
target_link_libraries( generate_experiment_specification ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp )

add_executable( auto_generator src/auto_generator.cpp )
target_link_libraries( auto_generator ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp model_builder )

add_executable( auto_experimenter src/auto_experimenter.cpp )
target_link_libraries( auto_experimenter SkinSimTestingFramework )

add_executable( auto_calibrator src/auto_calibrator.cpp )
target_link_libraries( auto_calibrator SkinSimTestingFramework )