# SkinSim : multi-modal skin simulation for Gazebo

# About
SkinSim is a multi-modal skin simulation environment based on the Gazebo simulator. It provides functionality for building robot models with robotic skin attached and near real-time realistic skin simulation.

# Dependencies
### ROS Hydro
Requires Gazebo 4 in order to compile. To upgrade hydro version, this might be sufficient:
```
sudo apt-get install ros-hydro-gazebo4-ros-pkgs
sudo apt-get install ros-hydro-gazebo4-ros-control
```
If not, follow http://gazebosim.org/tutorials?tut=install_ubuntu&ver=4.0&cat=install:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
sudo apt-get update
sudo apt-get install gazebo4
sudo apt-get install libgazebo4-dev
```
Other packages:
```
sudo apt-get install clang  
sudo apt-get install protobuf-compiler
#sudo apt-get install protobuf-c-compiler ?
```
### ROS Jade
```
sudo apt-get install libgazebo5-dev
```
# Install
- Clone to catkin workspace eg. ~/catkin_ws/src

		git clone https://isura@bitbucket.org/nextgensystems/skinsim.git


- Add path to skinsim as SKINSIM_PATH env variable

		export SKINSIM_PATH=~/catkin_ws/src/skinsim


- Export model and plugin folders

		echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/model/models" >> ~/.bashrc
		echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SKINSIM_PATH/build" >> ~/.bashrc
		source ~/.bashrc


- Build SkinSim

		mkdir build
		cd build
		cmake ..
		make

# Versioning
Semantic versioning 2.0.0 is used in SkinSim. See : http://semver.org/

- Current version : 0.1.0

# Release Schedule and Roadmap
A new version of SkinSim will be released 1 month after every major Gazebo release.

- 2014-09-30 - SkinSim 0.1.0 : Gazebo 4.0 : ROS I
- 2015-02-26 - SkinSim 1.0.0 : Gazebo 5.0 : ROS J
- 2015-08-27 - SkinSim 2.0.0 : Gazebo 6.0 : 
- 2016-02-25 - SkinSim 3.0.0 : Gazebo 7.0 : ROS K

# Coding Style

SkinSim tries to adhere to the Google style guide:

- Google C++ Style Guide : http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

# Build Status

[![Build Status](https://drone.io/bitbucket.org/rommelAlonzo/skinsim/status.png)](https://drone.io/bitbucket.org/rommelAlonzo/skinsim/latest)