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

and source the bashrc file again.

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

### Example programs

- *skin_model_generator*
    - generates Gazebo models based on the configurtions inside *generator/config/model_params.yaml* 
- *generate_experiment_specification*
    - generates files used by experimenter
- *auto_experimenter*
    - runs auto experimenter (currently no data is saved) 

### Demo
First generate the spring array model:

    cd ~/skin_ws/src/skinsim/build
    ./skin_model_generator

Startup Gazebo by running 

    gazebo --verbose

or

    roslaunch gazebo_ros empty_world.launch

Insert the spring_array model from the menu. To test: spawn a sphere, resize it, and drop it on the array.
If Gazebo was launched with ROS, virtual forces can be applied:
```
rosservice call /gazebo/apply_body_wrench "body_name: 'spring_2'
reference_frame: 'spring_2'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 0.0, y: 0.0, z: 10.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 1, nsecs: 0}" 
```
To view or save the data published from the spring array plugin:

    rostopic list
    rostopic echo /skinsim/spring_array
    rostopic echo /skinsim/spring_array -p > data_file.csv

The file can easily be importeted into MATLAB using the *rtpload.m* script.

### Versioning
Semantic versioning 2.0.0 is used in SkinSim. See http://semver.org/

- Current version : 0.1.0
- In development  : 0.2.0

### Release Schedule and Roadmap (OUTDATED)
A new version of SkinSim will be released 1 month after every major Gazebo release.

- 2014-09-30 - SkinSim 0.1.0 : Gazebo 4.0 : ROS I
- 2015-07-01 - SkinSim 0.2.0 : Gazebo 5.0 : ROS J
- 2015-09-01 - SkinSim 0.3.0 : Gazebo 5.0 : ROS J
    - Classes for different skin models
    - Classes for different sensor models - noise models etc.
    - Classes for different sensor data encoding types
    - Automated testing framework
    - Refactoring/Review
- 2015-12-01 - SkinSim 0.4.0 : Gazebo 5.0 : ROS J
    - Automatic skin placement on 3D surfaces
    - Standard tactile message types
- 2016-03-01 - SkinSim 1.0.0 : Gazebo 6.0 : ROS J
    - Robot tailor GUI
- 2016-05-01 - SkinSim 2.0.0 : Gazebo 7.0 : ROS K

### Coding Style

SkinSim tries to adhere to the Google style guide:

- Google C++ Style Guide : http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

### Build Status

[![Build Status](https://drone.io/bitbucket.org/nextgensystems/skinsim/status.png)](https://drone.io/bitbucket.org/nextgensystems/skinsim/latest)
