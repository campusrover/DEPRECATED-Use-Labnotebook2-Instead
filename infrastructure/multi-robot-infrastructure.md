# Multi Robot Infrastrcucture

## How to namespace multiple robots and have them on the same roscore

To have multiple robots on the same ROS core and each of them listen to a separate node, namespacing would be an easy and efficient way to help.

### Set namespace on robots' onboard computers with environment variable

* Boot up the robot and ssh into it
* On the robot's onboard computer, open the terminal and type in

````
nano ~/.bashrc`
````

Then add the following line to the end of the file, using the robots name as the namespace

````
export ROS_NAMESPACE={namespace_you_choose}`
````

Also make the following other changes:

````
alias bu='roslaunch turtlebot3_bringup turtlebot3_robot.launch'
export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export ROS_NAMESPACE=roba
export ROS_IP=<ip address of computer where this .bashrc is stored>
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger

````
* Save the file and don't forget to do `source ~/.bashrc`

## Set namespace on your laptop with environment variable
* Now that the robot is configured properly with its own unique name space, how do we talk to it?
* There are three ways:
    1. Configure your laptop to be permanently associated with the same name space
    1. Set a temporary environment variable that specifies the name space
    1. Add the namespace to a launch file
    1. Use the __ns parameter for roslaunch or rosrun (not recommended)

### Permanently associate your laptop with the name space

* Use the same steps above. Make sure your namespace is exactly the same as the namespace of the robot you want to control. 
* From now on, whenever you do, e.g. a cmd_vel, it will be directed just to your robot.

### Use an environment variable

* Set namespace for a termimal with temporary environment variable.
* To set a namespace temporarily for a terminal, which will be gone when you close the termial, just type in `export ROS_NAMESPACE={namespace_you_choose}` directly in your terminal window. 
* You can use `echo $ROS_NAMESPACE` to check it.

### Use the .launch file

* Set namespace for a node in launch file/ Use the attribute `ns`, for example:

`<node name="listener1" pkg="rospy_tutorials" type="listener.py" ns="{namespace_you_choose}" />`

### Not recommended: add __ns to the run or launch command
1. Launch/Run a file/node with namespace in terminal. Add a special key `__ns` (double underscore here!) to your command, for example: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch __ns:={namespace_you_choose}` However,
* Use of this keyword is generally not encouraged as it is provided for special cases where environment variables cannot be set. (http://wiki.ros.org/Nodes)

### Publishing/Subsribing topics in other namespace

Do a `rostopic list`, you will find out topics under a namespace will be listed as `/{namespace}/{topic_name}`
