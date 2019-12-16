# Robotics Lab Robot Networking Configuration

### Robot Bashrc File 

alias gp='git pull'

alias cw='cd ~/catkin_ws'

alias cs='cd ~/catkin_ws/src'

alias cm='cd ~/catkin_ws && catkin_make'

source /opt/ros/melodic/setup.bash

source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://mutant.dyn.brandeis.edu:11311

export ROS_HOSTNAME=mutant.dyn.brandeis.edu

export TURTLEBOT3_MODEL=burger

### PC Bashrc File

alias gp='git pull'

alias cw='cd ~/catkin_ws'

alias cs='cd ~/catkin_ws/src'

alias cm='cd ~/catkin_ws && catkin_make'

alias bu='roslaunch turtlebot3_bringup turtlebot3_robot.launch'

source /opt/ros/kinetic/setup.bash

source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://<ROBOT_NAME>.dyn.brandeis.edu:11311

export TB3_MODEL=burger

export TURTLEBOT3_MODEL=burger

export ROS_HOSTNAME=<ROBOT_NAME>.dyn.brandeis.edu

# Robotics Lab Mac Addresses
* One entry for each robot or roscore.
* All these devices live in the Robotics Lab in Gzang 006

### Name, mac address, IP address, dns name
* Wifi Connected Rasberry Pi's
  * b8:27:eb:e3:9b:a2 - **mutant** - mutant.dyn.brandeis.edu
  * b8:27:eb:75:89:b1 - **roba** - roba.dyn.brandeis.edu
  * b8:27:eb:01:e7:69 - **robb** - robb.dyn.brandeis.edu
  * b8:27:eb:83:21:e4 - **robc** - robc.dyn.brandeis.edu
  * b8:27:eb:8c:90:c5 - **donatello** - donatello.dyn.brandeis.edu
  * b8:27:eb:a4:d5:ec - **rafael** - rafael.dyn.brandeis.edju
  * 74:40:bb:d5:ea:2f - **alien** - alien.dyn.brandeis.edu
* Wifi connected Linux Notebook
  * 5c:ff:35:0f:ef:6d - **roscore2** 
* Wired Linux Desktop
  * 44:37:e6:b7:4b:d1 - **roscore1**

### Internal DNS names
* Currently the names are formed as <hostname>.dyn.brandeis.edu
* <hostname> is literally the `hostname` of the particular robot
* You can check and set it with the `$ hostname` command
* 