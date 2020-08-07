bmflight Simulation Gazebo-ROS Interface
===============
# work in progress!!!
This ROS package is based on the CrazyS extension for the ROS package RotorS.

For installing follow instruction under:
https://github.com/gsilano/CrazyS/blob/master/README.md

Here the excerpt for my setup:

Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-octomap-mapping
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ sudo apt install ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control
$ sudo apt install ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface
$ sudo apt install ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface
$ rosdep update
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5. Update the pre-installed Gazebo version. This fix the issue with the `error in REST request for accessing api.ignition.org`

```console
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ sudo apt install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event that the simulation does not start, the problem may be related to Gazebo and missing packages. Therefore, run the following commands. More details are reported in [#25](https://github.com/gsilano/CrazyS/issues/25).

```console
$ sudo apt-get remove ros-melodic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

# building ROS package
```
$ catkin build
```
Source the package so it is available to ros
```
soure devel/setup.bash
```
# start simulation interface
to start the testdrone use:
```
$ roslaunch bmflight_simulation test.launch
```
