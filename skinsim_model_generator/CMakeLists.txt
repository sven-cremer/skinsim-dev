cmake_minimum_required(VERSION 2.8.3)
project(skinsim_model_generator)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscpp
  pcl_ros
  pcl_conversions
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(PCL REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}
							${GAZEBO_INCLUDE_DIRS}
							${SDFormat_INCLUDE_DIRS}
							${PCL_INCLUDE_DIRS}     )

catkin_package(
    DEPENDS 
      gazebo_ros
      roscpp
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)

# Build whatever you need here
add_executable(skinmodel_sdf_generator src/skinmodel_sdf_generator.cc)
target_link_libraries( skinmodel_sdf_generator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp )

#add_executable(only_skinmodel_sdf_generator src/only_skinmodel_sdf_generator.cc)
#target_link_libraries( only_skinmodel_sdf_generator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp )


# Mesh code
add_executable(mesh2pclTest src/mesh2pclTest.cc)
target_link_libraries(mesh2pclTest ${catkin_LIBRARIES})

