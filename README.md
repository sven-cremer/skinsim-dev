# SkinSim : multi-modal skin simulation for Gazebo

# About
SkinSim is a multi-modal skin simulation environment based on the Gazebo simulator. It provides functionality for building robot models with robotic skin attached and near real-time realistic skin simulation.

# Dependencies
SkinSim 0.2 uses Gazebo 5 which is supported in Ubuntu 14.04 (Trusty). The following needs to be installed:
```
sudo apt-get install libgazebo5-dev
sudo apt-get install clang  
sudo apt-get install protobuf-compiler
```
ROS Jade is optional:
```
http://wiki.ros.org/jade/Installation
```
# Install
- Clone to catkin workspace (for example ~/catkin_ws/src)  

		git clone https://<user-name>@bitbucket.org/nextgensystems/skinsim.git


- Add path to skinsim as SKINSIM_PATH env variable

		echo "export SKINSIM_PATH=~/catkin_ws/src/skinsim" >> ~/.bashrc


- Export model and plugin folders

		echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/model/models" >> ~/.bashrc
		echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$SKINSIM_PATH/build" >> ~/.bashrc
and source the bashrc file

		source ~/.bashrc


- Build SkinSim

		cd ~/catkin_ws/src/skinsim
		mkdir build
		cd build
		cmake ..
		make

- Add the build folder to the path environment variable

		echo "export PATH=$SKINSIM_PATH/build:$PATH" >> ~/.bashrc
and source the bashrc file again.


# Example programs

- *skin_model_generator*: generates Gazebo models based on the configurtions inside generator/config/model_params.yaml 
- *generate_experiment_specification*: generates files used by experimenter
- *auto_experimenter*: runs auto experimenter (currently no data is saved) 

# Versioning
Semantic versioning 2.0.0 is used in SkinSim. See : http://semver.org/

- Current version : 0.1.0

# Release Schedule and Roadmap
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

# Coding Style

SkinSim tries to adhere to the Google style guide:

- Google C++ Style Guide : http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

# Build Status

[![Build Status](https://drone.io/bitbucket.org/rommelAlonzo/skinsim/status.png)](https://drone.io/bitbucket.org/rommelAlonzo/skinsim/latest)