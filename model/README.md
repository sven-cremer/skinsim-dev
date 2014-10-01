# Initial README

# Install

1. Export model folder
~~~~
export SKINSIM_PATH=~/catkin_ws/src/skinsim
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/skinsim_model/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SKINSIM_PATH/skinsim_plugins/build" >> ~/.bashrc
source ~/.bashrc
~~~~

2. make
~~~~
cd ~/catkin_ws &&
catkin_make
~~~~

3. run
~~~~
roslaunch skinsim_model skinsim_model.launch
~~~~