# agility-cassie-gazebo-simulator

A simulation library for Agility Robotics' Cassie robot using Gazebo.
Developed and tested on Ubuntu 18.04 with Gazebo 9.9

This has been modified from the original Agility Robotics library to be used alongside **dairlib**.
## Instructions:

### Building the code:
* Download and build **dairlib** (https://github.com/DAIRLab/dairlib). NOTE: this is needed for the LCM types only.
* Download and install Gazebo 9 - http://gazebosim.org/tutorials?tut=install_ubuntu
* Clone this repository into the same root directory as **dairlib**.
* Make a build directory
* `cd build`
* `cmake ../plugin`
* `make`
* `source /usr/share/gazebo/setup.sh` 
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<WORKSPACE>/cassie-gazebo-sim/`
* `export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<WORKSPACE>/cassie-gazebo-sim/build`
* `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<WORKSPACE>/cassie-gazebo-sim/build`
* From the `cassie-gazebo-sim/cassie` directory, run `gazebo cassie.world` to test if the installation is successful. 

Note: The Gazebo plugin and model path needs to be updated to load the meshes and plugins - http://gazebosim.org/tutorials?tut=components

### Running the sim:
* `cd ~/path-to-repo/cassie`
* ` gazebo cassie.world`

The robot should be spawned in Gazebo

### Connecting to the sim with a demo UDP based controller:
* `cd ~/path-to-repo/your-build-folder`
* `./cassiectrl`

The controller should be connected to the simulator.

The examples include cassiectrl, a null controller operating over UDP.
