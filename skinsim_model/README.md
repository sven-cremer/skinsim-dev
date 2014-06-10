# Initial README

# Install

1. Export model folder

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/skinsimdev/skinsim_model/models" >> ~/.bashrc &&
source ~/.bashrc

2. make

cd ~/catkin_ws &&
catkin_make

3. run

roslaunch skinsim_model skinsim_model.launch
