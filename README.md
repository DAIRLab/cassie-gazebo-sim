# agility-cassie-gazebo-simulator

A simulation library for Agility Robotics' Cassie robot using Gazebo.
Developed and tested on Ubuntu 16.04 with Gazebo 9.

## Instructions:

### Building the code:
* Download and install Gazebo 9 - http://gazebosim.org/tutorials?tut=install_ubuntu
* Clone this repository
* Make a build directory
* `cd build`
* `cmake ..`
* `make`

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
