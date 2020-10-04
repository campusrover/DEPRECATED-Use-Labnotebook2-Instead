
## Using the MiniRover

### Turning it on

It is important that you follow a rigid procedure when turning the robot on and off.

Assuming it is totally off:

1. Make sure the battery pack is correctly connected to the robot.

1. Switch on the battery pack

1. Click the micro button on the red board

![Button to reboot](button.jpg)

1. Look at the Raspberry Pi (not the "red board") You will see tiny red and green LEDs blinking. Wait until the green one has settled down to slow flickering.

### Turning it off

1. From your ssh command line on the remote computer, type `sudo shutdown now`

1. Once the Raspberry Pi has stopped blinking you can turn off the power switch on the battery pack.

### Charging the Battery

The Battery pack came with a charger. It has a light on which is red while charging and green when fully charged. *Note that the battery will not charge when its switch is set to off*.

### rset command

We've implemented a very simple tool to set up the IP addresses correctly. It will be changing as I figure out how to make it better. So for now...

1. If you have a cloud desktop and want to run simulations without a actual miniRover" `rset cloud`
1. If you have a cloud desktop add a real robot, run `rset robot` on your cloud desktop and `rset pi` on the  actual miniRover (over ssh)
1. If you have a local docker based desktop, run `rset docker` there.

rset by itself displays the current satus

(I know a combination is missing and plan a revision of this)

### Starting the MiniRover ROS applications

Note that all we run on the MiniRover itself are roscore, and the nodes needed for the motors, lidar and camera. Everything else runs on your "remote". The following commands are to be done on the MiniRover from ~/catkin_ws

```
# launch the motor controller and lidar
roslaunch gpg_bran gpg_ydlidar.launch 

# launch camera (optional if you need the camera)
roslaunch gpg_bran raspicam.launch
```

### Web Desktop tools

Note that this includes all flavors, cloud based, local docker based, and gpu based browser desktops. If you just want to use the simulators on their own and are not using an actual miniRover, then: `rset cloud` is enough. At that point you can run your ROS programs.

There are numerous scripts, programs and launch files that are preinstalled on your ROS web desktop. I will document only some of them here but you can look around and find more that are interesting. All of them from the book are here. I have not renamed any of them for that reason.

```
# Fun prr examples
rosrun prrexamples red_light_green_light.py
rosrun prrexamples wander.py

# Visualize robot with rviz
roslaunch gopigo3_navigation gopigo3_rviz.launch 

# Or with gazebo (not as useful)
roslaunch gopigo3_navigation gopigo3_gazebo.launch 

# Run different simulated stages to experiment with roslaunch 
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
roslaunch turtlebot3_gazebo turtlebot3_stage_2.launch
roslaunch turtlebot3_gazebo turtlebot3_stage_3.launch
roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
```

### Rset Command

In order to facilitate working in all the combinations of environments we have these commands:

* rset pi - declare that this is the raspberry pi
* rset cloud - declare that this is a cloud desktop working with sim
* rset robot - declare that this is a cloud desktop working with a real robot
* rset - display settings

### Aliases

There are a bunch of handy aliases:

* `myip` - this computer's regular local ip
* `myvpnip` - this computer's vpn ip if it has one
* `stopnow` - immediately stop the robot
* `teleop` - run a simple teleop tool
* `ru` - connect to rosutils
* `cm` - catkin_make
* `cw` - cd ~/catkin_ws
* `cs` - cd ~/catkin_ws/src



### Accounts and passwords

* miniRover
  * hostname `gopigo3`
  * default account `pi`
  * default password raspberry
* Cloud or Docker Desktop
  * default password `dev@ros`
  * url: <unetid>.ros.campusrover.org:6080/vnc.html (desktop)
  * url: <unetid>.ros.campusrover.org:8080 (cloud vscode)

### Key Environment Variables

`ROS_MASTER_URI=http:/100.94.206.80:11311` (example!) should always be set to the computer where roscore is running. If you are using a physical robot, then roscore runs on the robot itself. If you are in the web deskop and working just with simulation then roscore would run there.

#### Robot

ROS_MASTER_URI = robot's own ip address
ROS_IP = robot's own ip address

##### Remote Computer 

ROS_MASTER_URI = robots ip address 
ROS_IP = remote computer's own IP address

These IP addresses are on different networks and cannot access each other. So instead we've created what is called a "virtual private network" that connects them together. Both your robot and your cloud desktop have an *alternate* ip address which they can both see.

### IP Addresseses

* `myip` returns your local ip address
* `myvpnip` returns your vpn ip address (if you have one)


