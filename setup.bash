####Setup Environment Varaibles####
echo "**********SkinSim Wksp**********"
export SKINSIM_PATH=~/jade_wksp/src/skinsim
export SKINSIM_MODEL_PATH=$SKINSIM_PATH/model/models
export SKINSIM_WORLD_PATH=$SKINSIM_PATH/model/worlds
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SKINSIM_MODEL_PATH
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SKINSIM_PATH/build

####Generator Variables####
export SKINSIM_GEN_CONFIG_PATH=$SKINSIM_PATH/generator/config
export SKINSIM_GEN_OUTPUT_PATH=$SKINSIM
