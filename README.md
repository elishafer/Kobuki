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

## Installing rplidar

On the Kobuki at our lab, we have an rplidar (version A1 or A2). So we need to install the rplidar package in the same way as before, namely cloning the git file:
```
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
```
After cloning we build the catkin:
```
cd ~/catkin_ws
catkin_make
```

### Running rplidar
After installing the lidar package, to run it we need to first set the read/write authorisations. First we check where it is connected. We run the following command:## TroubleShooting
When you get problems with tf use the following command:

    rosrun rqt_tf_tree rqt_tf_tree

Checkout the [ros tf tutorial](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) for more info
```
ls -l /dev |grep ttyUSB
```
The output should be something like:
```console
jetson0@jetson-nano:~/catkin_ws$ ls -l /dev |grep ttyUSB
lrwxrwxrwx  1 root    root           7 Nov  5 11:45 gps0 -> ttyUSB0
lrwxrwxrwx  1 root    root           7 Nov  5 11:45 kobuki -> ttyUSB1
crw-rw-rw-  1 root    dialout 188,   0 Nov  5 13:13 ttyUSB0
crw-rw-rw-  1 root    dialout 188,   1 Nov  5 11:45 ttyUSB1
```
our lidar has the name of `gps0` so we need to change the `ttyUSB0` permissions:
```
sudo chmod a+rw /dev/ttyUSB0
```
Now we can execute an example launch file:
```
roslaunch rplidar_ros view_rplidar.launch
```

For more info checkout the [ros wiki page](http://wiki.ros.org/rplidar)

## SLAM
You can use [hector mapping](http://wiki.ros.org/hector_mapping).
To install do the regular drill: 
```
cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```
I used the `catktin` branch. 
```
git checkout catkin
```
Otherwise if you want, you can use the `melodic-devel` branch:
```
git checkout melodic-devel
```

### Change tf names

You'll need to change the tf names in the launch files so that they can work with each other.
In hector slam the odom_frame is called nav, and the base_frame is base_stabilized. Change these names in any launch file that you'll want to use. for example in `/home/jetson0/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch` you'll want to change:

```
    <param name="base_frame" value="base_stabilized" />
    <param name="odom_frame" value="nav"/>
```

to:
```
    <param name="base_frame" value="base_footprint" />
    <param name="odom_frame" value="odom"/>
```

You'll also want to change the rplidar launch to take the tf in consideration. In `/home/jetson0/catkin_ws/src/rplidar_ros/launch/rplidar.launch` add the following line before `</launch>`:
```
  <node pkg="tf" type="static_transform_publisher" name="base_frame_2_laser"
   args="0 0 0.05 0 0 0 1 /base_link /laser 50" />
```

## Run Everything together
First open a new terminal and launch kobuki node with tf:

    roslaunch kobuki_node robot_with_tf.launch --screen
    
Next open a new terminal and launch the rplidar:

    roslaunch rplidar_ros rplidar.launch
    
Now you can open a new terminal and launch hectorslam:

    roslaunch hector_slam_launch tutorial.launch


## Run From another PC on network
You'll want the robot to be mobile and control/monitor it from afar. Thus you'll have to setup ROS on another pc on the same network as the robot. get a dedicated wifi router for that. For setup check [this](http://users.umiacs.umd.edu/~cteo/umd-erratic-ros-data/README-rvis-remote).
Here's an excerpt from there:
```
*** ssh into remote computer ***
1. ssh -X erratic@192.168.1.0

At the remote terminal:
2. export ROS_MASTER_URI=http://192.168.1.0:11311   //this ensures that we do not use localhost, but the real IP address as master node

3. export ROS_IP=192.168.1.0 		//this ensures that ROS knows that we cannot use hostname directly (due to DHCP firewall issues)

4. roscore

At the local terminal:
1. export ROS_MASTER_URI=http://192.168.1.0:11311   //tells local computer to look for the remote here

2. export ROS_IP=192.168.1.1		//this ensures that ROS knows that we cannot use hostname directly (due to DHCP firewall issues)

3. rosrun rviz rviz 			// fires up rviz on local computer. It will attach to the master node of the remote computer
```
Add the source commands to `~/.bashrc` or `~/.zshrc` depending on what you're using.


## TroubleShooting
When you get problems with tf use the following command:

    rosrun rqt_tf_tree rqt_tf_tree

Checkout the [ros tf tutorial](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) for more info


