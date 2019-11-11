# Kobuki
Documentation for Kobuki Robot on Nvidia Jetson with rplidar and some other peripherals.

## Step 0 - Setup Nvidia Jetson Nano on SD card

You'll first want to follow the [setup instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) for setting up the Ubuntu SD card image. At time of writing image is Ubuntu 18.04.

## Install ROS Melodic

You'll first want to install ROS Melodic so follow the [instructions here](http://wiki.ros.org/melodic/Installation/Ubuntu).
If you don't have a lot of experience with ROS check out the [start guide](http://wiki.ros.org/ROS/StartGuide) and follow the links there for an [intro](http://wiki.ros.org/ROS/Introduction) and the [concepts](http://wiki.ros.org/ROS/Concepts) of ROS.
The [tutorials](http://wiki.ros.org/ROS/Tutorials) are also very good, so check them out.

## Installing the Kobuki Package

The following assumes that you've created a catkin workspace in the default directory as in the [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
We'll have to first clone the package from github into the catkin workspace. 
```
cd ~/catkin_ws/src
git clone https://github.com/yujinrobot/kobuki.git
```
Next, build the package(s):

```
cd ~/catkin_ws
catkin_make
```
Now the package should have compiled. You can now look at the [kobuki tutorials](https://wiki.ros.org/kobuki/Tutorials) for what the robot can do. For example check out [this](https://wiki.ros.org/kobuki/Tutorials/Examine%20Kobuki) to listen in on some of the sensor nodes. Just remember to source your setup file before running any `.launch` files.
```
source ~/catkin_ws/devel/setup.bash
```
