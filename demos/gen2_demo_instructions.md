# Demo Setup

## Intro

Basically all the steps we think are needed to get the [Fall 208 Demo Script](https://github.com/campusrover/labnotebook/tree/dc56aa6edc981881eef46d234f70ff1c55b767ce/demo_script_fall/README.md) to work. This was gathered by the team and then experimented and revised further by Pito during the post-semester break.

### Onboard Laptop Steps:

* SSH into the robot’s onboard laptop \(turtlebot@129.64.243.64\)
* Start a roscore
* Do a bringup for the onboard laptop: “roslaunch cr\_ros campus\_rover.launch”
* Wait until the bringup finishes. Look for “Odom received” message

### On another machine \(needs to be more powerful for this than for the web app\)

* Run “roslaunch cr\_ros off\_board.launch”
* Wait until the bringup finishes. Look for “Odom received” message

### On a third machine

* Install Flask
* Clone cr\_web repo
* cd into cr\_web
* Make sure to bring up TB2 \(with cr\_ros\), or run a local roscore
* export ROS_MASTERURI=_[http://\_\_\_\_\_\_](http://______)
* export FLASK\_APP=rover\_app
* flask run --no-reload --host=0.0.0.0 --with-threads

_Note: --host=0.0.0.0 and --with-threads are only needed if other client machines need to access the hosted app._

* Go to localhost:5000/, or replace localhost with the other machine’s IP if you’re accessing it from a client machine. \(edited\)

### Known dependencies:

* aruco\_detect \(for fiducial\_msgs import error\)
* pip install face\_recognition
* Kobuki docking might be part of the general TB2 install or may need to be separate

### On offboard computer

* sudo apt-get install ros-kinetic-moveit-ros-visualization
* make sure that ROS\_MASTER\_URI=[http://129.64.243.64:11311](http://129.64.243.64:11311) \(or whereever roscore is runnung\)

