### Hints on how to set things up

* Reflects the "new" multi robot setup


#### .bashrc ON BOTH your laptop and your Robot
* This shell script is run automatically whenever you open a new terminal or tab in a terminal
* This means that if you edit it and save it, it still does nothing until you open a new terminal
* If you want to apply it without opening a new terminal, do `source ~/.bashrc`
* These lines should be on your laptop as well as your robot, the same way.

````
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export ROS_NAMESPACE=roba
export TB3_MODEL=burger
````

* ROS_HOSTNAME: the IP address of this computer. localhost stands for "my own ip"
* ROS_MASTER: the IP address of whereever ROSCORE is running. We have roscore always running on the little computer by the door. It happens to be called roscore1.cs.brandeis.edu
* ROS_NAMESPACE: Indicates a unique name for a specific robot. That way when we send a /cmd_vel it is turned into, e.g. /roba/cmd_vel. This way all the robots can coexist on one ROSCORE. It also means you don't have to run your own ROSCORE anymore.

#### ON THE ROBOT
* SSH into the robot
* Run the following command to launch ROS on the robot

````
roslaunch turtlebot3_bringup turtlebot3_robot.launch
````

#### Test the configuration
* Can you run rostopic list? If not, check your IP addresses and whether the ROSCORE computer is running. Try pinging roscore1.cs.brandeis.edu
* Does it list /x/cmd_vel among them? If not, check your IP addresses. Also make sure you ran
* Can you teleop the robot? `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`