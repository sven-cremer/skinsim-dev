# SkinSim : multi-modal skin simulation for Gazebo

# About
SkinSim is a multi-modal skin simulation environment based on the Gazebo simulator. It provides functionality for building robot models with robotic skin attached and near real-time realistic skin simulation.

# Install
- clone to catkin workspace eg.
			~/catkin_ws/src
			git clone https://isura@bitbucket.org/nextgensystems/skinsim.git
- Add path to skinsim as SKINSIM_PATH env variable
'export SKINSIM_PATH=~/catkin_ws/src/skinsim'
- Export model and plugin folders
```
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/skinsim_model/models" >> ~/.bashrc
```
```
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SKINSIM_PATH/skinsim_plugins/build" >> ~/.bashrc
```
```
source ~/.bashrc
```
- Build SkinSim
```
mkdir build
cd build
cmake ..
make
```

# Versioning
Semantic versioning 2.0.0 is used in SkinSim. See : http://semver.org/
- Current version : 0.0.0

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

[![Build Status](https://drone.io/bitbucket.org/nextgensystems/skinsim/status.png)](https://drone.io/bitbucket.org/nextgensystems/skinsim/latest)