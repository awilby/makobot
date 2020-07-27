# makobot

Tools and ROS packages for the Makobot underwater robot, a Hovering Autonomous Underwater Vehicle (HAUV) built on the BlueROV2.


## Quickstart

This package has primarily been tested with ROS Noetic on Ubuntu 20.04, but should work with ROS Kinetic and Melodic with few issues.

Detailed system documentation, including hardware modifications, networking setup, and all other instructions can be found at: [https://makobot-hauv.github.io/](https://makobot-hauv.github.io/).

### Install Dependencies

For ROS Melodic and below:

```
sudo apt install python-catkin-tools
```

For ROS Noetic:

```
sudo apt install python3-catkin-tools
```

### Install Makobot software

Clone this repository and all submodules into the source folder of your existing ROS workspace:

```
git clone --recurse-submodules --remote-submodules https://github.com/awilby/makobot.git
```

From the root of your ROS workspace, run:

```
catkin build
```

Make sure to re-source the `devel/setup.bash` file of your ROS workpace.


### Networking Setup

Your topside system should be running ROS and Ubuntu and using NetworkManager for these setup instructions. These instructions assume the vehicle has already been properly set up with the instructions at

TODO add detailed instructions for these steps:

* Follow instructions for setting up SSH Keys
* Set up static IP and share internet
* set ROS_Master_URI to fitlet (fitlet is ROS master)



## Running

If everything has worked thus far, you should be able to simply run:

```
roslaunch makobot makobot.launch
```

and everything should work.

TODO: do we need to run this from the fitlet as it's master or can we launch from topside even when fitlet is master?
