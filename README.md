communication_node
===================

This repository contains the code for ROS-Gazebo communication_node


Abstract
----------
TBA.

----------

### Preparing Your System
----------
This software uses Gazebo 7.0, Libgazebo7-dev and ROS-kinetic. If you have these already installed and working, skip to the [next section](#using-the-communication-node).

* First, you'll need to install Linux 16.04 LTS. Instructions are available [here](https://help.ubuntu.com/lts/installation-guide/).


* Install ROS Kinetic using [this](http://wiki.ros.org/kinetic/Installation/Ubuntu) tutorial


* Install ROS-gazebo packages:
	```
	sudo apt-get install ros-kinetic-gazebo-\*
	```
* Install Libgazebo7-dev:
	```
    sudo apt-get install libgazebo7-dev 
    ```
    

### Installing the Communication Node
----------
Create a catkin workspace:
```
mkdir ~/communication_node_ws
cd ~/communication_node_ws
```

Clone this repository:
```
git clone https://github.com/SajadAzami/communication_node.git
```

Move all the files in *communication_node/src* to *~/communication_node_ws*:
```
cp -r ~/communication_node_ws/communication_node/src ~/communication_node_ws
```

Export gazebo plugin path(communication_node_ws is the name of the workspace):
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/communication_node_ws/communication_node/devel/lib
```

Now you can make the package:
```
catkin_make
```

Source the variables:
```
source ./devel/setup.sh
```

### Using the Communication Node
----------
TBA.


### Contribution on the node
----------
TBA.

#### Lisence
----------
BSD-3-Clause
