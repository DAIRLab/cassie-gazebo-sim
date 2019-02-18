source /usr/share/gazebo/setup.sh
echo $PWD
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/..
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$PWD/../build
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PWD/../build
gazebo cassie.world