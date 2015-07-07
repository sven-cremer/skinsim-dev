##########################
# Continuous Integration #
##########################

# Install Gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgazebo5-dev

# Install dependencies
sudo apt-get install clang  
sudo apt-get install protobuf-compiler

# Environment Variables
echo "export SKINSIM_PATH=~/catkin_ws/src/skinsim" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_PATH/model/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$SKINSIM_PATH/build" >> ~/.bashrc
source ~/.bashrc

# Build
mkdir build
cd build
cmake ..
make

