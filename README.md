# SkinSim

### About
SkinSim is a multi-modal skin simulation environment based on the Gazebo simulator. It provides functionality for building robot models with robotic skin attached and near real-time realistic skin simulation.

### Dependencies
SkinSim 0.2 is being developed in Ubuntu 14.04 (Trusty) using Gazebo 5. To install the developer tools: 
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgazebo5-dev
```
The following dependencies are needed:
```
sudo apt-get install clang  
sudo apt-get install protobuf-compiler
```
It is recommended to install ROS alonside with Gazebo. Version 5 is supported by ROS Jade and *gazebo_ros_pkgs* provides several wrappers:

- http://wiki.ros.org/jade/Installation
- http://gazebosim.org/tutorials?tut=ros_installing

Currently, *ros_control* has not been released for ROS Jade. 

### Install
1\) Assuming that ROS has been installed, first setup a catkin workspace:

    source /opt/ros/jade/setup.bash
    mkdir ~/skin_ws/src -p
    cd ~/skin_ws/src
    catkin_init_workspace
    cd ~/skin_ws
    catkin_make

Edit your bashrc to source the workspace in every new shell or simply execute

    echo "source ~/skin_ws/devel/setup.bash" >> ~/.bashrc
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

2\) Download the repositry (or fork) to your workspace, e.g. 

    cd ~/skin_ws/src
    git clone https://<user-name>@bitbucket.org/nextgensystems/skinsim.git

3\) Add the repository path to the SKINSIM_PATH environment variable

    echo "export SKINSIM_PATH=~/skin_ws/src/skinsim" >> ~/.bashrc

4\) Export model and plugin folders to the Gazebo environment variables

    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/model/models" >> ~/.bashrc
    echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$SKINSIM_PATH/build" >> ~/.bashrc

5\) Open a new terminal or source the bashrc file

    source ~/.bashrc

6\) Build SkinSim

    cd ~/skin_ws/src/skinsim
    mkdir build
    cd build
    cmake ..
    make

7\) Add the build folder to the path environment variable

    echo "export PATH=$SKINSIM_PATH/build:$PATH" >> ~/.bashrc

and source the bashrc file again. Here is a slightly modified version of what the .bashrc file should look like after the above steps:
```
function skin_ws {
	echo "*** skin_ws ***"
	source /opt/ros/jade/setup.bash
	source ~/skin_ws/devel/setup.bash

	unset GAZEBO_RESOURCE_PATH; unset GAZEBO_PLUGIN_PATH; unset GAZEBO_MODEL_PATH
	source /usr/share/gazebo/setup.sh

	export SKINSIM_PATH=~/skin_ws/src/skinsim-dev
	export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$SKINSIM_PATH/model/models
	export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SKINSIM_PATH/build
	export PATH=$SKINSIM_PATH/build:$PATH
	alias skin='cd $SKINSIM_PATH/build'
}
skin_ws
```
### Troubleshooting
If another version of Gazebo is already installed, then the following errors might occur:

- unable to install *libgazebo5-dev* due to "unmet dependencies"
- *catkin_make* fails because *gazebo_ros* cannot be found
- *make* fails because of missing member in "gazebo::physics::Joint"

For example, the PR2 simulator is still using Gazebo 2. To check the version, execute the following in a terminal
```
gazebo -v
```
If for example version 2 is installed, remove it by executing
```
sudo apt-get remove gazebo2
```
and then try installing *libgazebo5-dev* again.

### Executables
For detailed usage, see the demo descriptions below.

- *skin_model_generator*
    - generates Gazebo models based on the configurtions inside *generator/config/model_params.yaml* 
- *generate_experiment_specification*
    - generates files used by experimenter (deprecated)
- *auto_experimenter*
    - generates files used by experimenter
- *auto_experimenter*
    - runs the experimenter

### Demo 1 - Loading a Skin Array model
First generate the skin array model:

    cd ~/skin_ws/src/skinsim/build
    ./skin_model_generator

Note that it is using the default settings from *generator/config/model_param.yaml*.
Startup Gazebo with the ROS plugins by running 

    roscore
    gazebo -s `catkin_find --first-only libgazebo_ros_paths_plugin.so` -s `catkin_find --first-only libgazebo_ros_api_plugin.so` --verbose --pause

or

    roslaunch gazebo_ros empty_world.launch

Insert the *skin_array* model from the menu. Testing can be done by inserting the *simple_sphere* or *simple_cylinder* and dropping them on the skin array. Do not forget to unpause the simulation!
To apply virtual forces using ROS:
```
rosservice call /gazebo/apply_body_wrench "body_name: 'skin_array::patch_0_spring_24'
reference_frame: 'world'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 0.0, y: 0.0, z: -0.3}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 2, nsecs: 0}" 
```
RVIZ can be used for visualization:

    roslaunch skinsim_ros rviz.launch

*Note*: If the data is not published correctly, then try listening to the Gazebo contact topic.
In Gazebo, press `ctrl+T` and select the */gazebo/default/physics/contacts* topic. Or from Terminal,

    gz topic -e /gazebo/default/physics/contacts
    
This is a bug that needs to be fixed in the skin plugin. Gazebo does not recognize when contact data is being requested via the Contact Manger and will not publish any unless there is a subscription to the contact topic. 

### Demo 2 - Plunger experiment
The *skin_model_generator* also creates a world file with a plunger. To open the world, simply add its path to the Gazebo command:

    gazebo -s `catkin_find --first-only libgazebo_ros_paths_plugin.so` -s `catkin_find --first-only libgazebo_ros_api_plugin.so` --verbose --pause ~/skin_ws/src/skinsim-dev/model/worlds/skin_array.world

To move the plunger, unpause the simulation and send a control service request. For example:
```
rosservice call /skinsim/set_controller "type: {selected: 1}
fb: {selected: 1}
f_des: -2.0
x_des: 0.0
v_des: -0.005
Kp: 0.0
Ki: 0.0
Kd: 0.0"
```
The controller and feedback types are defined in
```
skinsim_ros_msgs/msg/ControllerType.msg
skinsim_ros_msgs/msg/FeedbackType.msg
```
Again, the response can be viewed in RVIZ:

    roslaunch skinsim_ros rviz.launch

### Demo 3 - Automated testing
The SkinSim experimenter enables automatic configuring and testing of several skin array models. Different model and controller specifications can be generated by modifying and compiling *experimenter/src/auto_generator.cpp*. In future, the generator will use a configuration file to test parameters over different ranges:

    - paramter_1: [min, step, max]

To generate the Gazebo models and experiment files:

    cd ~/skin_ws/src/skinsim/build
    ./auto_generator [experiment_name]

where the experiment name is optional. This creates an experiment folder in *data/* containing the model and controller configuration files. To run the experimenter:

    ./auto_experimenter [experiment_name]

which will load the configuration files from the *data/experiment_name/* folder. The topic name specified in the model file will be saved to the same folder. While the experimenter runs, ROS topics can viewed from terminal and RVIZ (however, this will slow down the simulation).

### Data collection with ROS
To view or save the joint data published from the skin array and plunger plugins:

    rostopic list
    rostopic echo /skinsim/joint_data
    rostopic echo /skinsim/joint_data -p > joint_data.csv

To save the joint layout of the skin array execute the following in order:

    rostopic echo /skinsim/layout -n 1 -p > layout_data.csv
    rosservice call /skinsim/publish_layout "selected: 0"

The tactile layout can be requested with

    rosservice call /skinsim/publish_layout "selected: 1"
    
All the CSV files can easily be imported into MATLAB using the *rtpload.m* script.

### Versioning
Semantic versioning 2.0.0 is used in SkinSim. See http://semver.org/

- Current version : 0.1.0
- In development  : 0.2.0

### Release Schedule and Roadmap (OUTDATED)
A new version of SkinSim will be released 1 month after every major Gazebo release.

- 2014-09-30 - SkinSim 0.1.0 : Gazebo 4.0 : ROS I
- 2016-09-01 - SkinSim 0.2.0 : Gazebo 5.0 : ROS J
    - Classes for different skin models
    - Classes for different sensor models - noise models etc.
    - Classes for different sensor data encoding types
    - Automated testing framework
    - Refactoring/Review
- 2016-12-01 - SkinSim 0.3.0 : Gazebo 5.0 : ROS J
    - Automatic skin placement on 3D surfaces
    - Standard tactile message types
- 2016-12-01 - SkinSim 1.0.0 : Gazebo 6.0 : ROS J
    - Robot tailor GUI
- 2016-12-01 - SkinSim 2.0.0 : Gazebo 7.0 : ROS K

### Coding Style

SkinSim tries to adhere to the Google style guide:

- Google C++ Style Guide : http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

### Build Status

[![Build Status](https://drone.io/bitbucket.org/nextgensystems/skinsim/status.png)](https://drone.io/bitbucket.org/nextgensystems/skinsim/latest)
