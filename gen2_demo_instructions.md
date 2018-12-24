# Gen2 Demo Instructions

## Intro
Basically all the steps we think are needed to get the [Fall 208 Demo Script](./demo_script_fall) to work. This was gathered by the team and then experimented and revised further by Pito during the post-semester break.

### Onboard Laptop Steps:
* SSH into the robot’s onboard laptop (turtlebot@129.64.243.64)
* Start a roscore
* Do a bringup for the onboard laptop: “roslaunch cr_ros campus_rover.launch”
* Wait until the bringup finishes. Look for “Odom received” message

### On another machine (needs to be more powerful for this than for the web app)
* Run “roslaunch cr_ros off_board.launch”
* Wait until the bringup finishes. Look for “Odom received” message

### On a third machine
* Install Flask
* Clone cr_web repo
* cd into cr_web
* Make sure to bring up TB2 (with cr_ros), or run a local roscore
* export ROS_MASTER_URI=http://________
* export FLASK_APP=rover_app
* flask run --no-reload --host=0.0.0.0 --with-threads

*Note: --host=0.0.0.0 and --with-threads are only needed if other client machines need to access the hosted app.*

* Go to localhost:5000/, or replace localhost with the other machine’s IP if you’re accessing it from a client machine. (edited)


### Known dependencies:
* aruco_detect (for fiducial_msgs import error)
* pip install face_recognition
* Kobuki docking might be part of the general TB2 install or may need to be separate

### On offboard computer
* sudo apt-get install ros-kinetic-moveit-ros-visualization
* make sure that ROS_MASTER_URI=http://129.64.243.64:11311 (or whereever roscore is runnung)