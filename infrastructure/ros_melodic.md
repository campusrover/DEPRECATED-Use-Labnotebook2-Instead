## Ubuntu 18.04 and ROS Melodic Install Notes

#### Gen 4 - Going to Ubuntu 18.04
* We are planning to move students' and teachers' computers to Ubuntu 18.04
* It has a nicer to use Desktop and is also the currently supported Ubuntu
* It supports ROS Melodic (and ROS Melodic does not support the old Ubuntu)

#### Installation notes
* Follow Ubuntu's own installation instructions to install a clean copy of Ubuntu
* As Robotis is still on Ubuntu 16.04 and ROS Kinetic we have to amend their instructions a little
* Our baseline is http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/

#### Commands

##### Core install of Melodic
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh
```

##### Further install of needed melodic packages

```
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-turtle-tf2 ros-melodic-tf2-tools ros-melodic-tf

```

##### Further install of ROS Packages

```
$ cd ~/catkin_ws/src/

# Robotis
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Cosi119a PRR Samples
$ git clone https://github.com/campusrover/prrexamples.git
$ exit
```
#### Important: OPEN A NEW SHELL!
```
$ cd ~/catkin_ws && catkin_make

```
