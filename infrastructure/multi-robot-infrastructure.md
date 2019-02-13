# Multi Robot Infrastrcucture

## How to namespace multiple robots and have them on the same roscore

To have multiple robots on the same ROS core and each of them listen to a separate node, namespacing would be an easy and efficient way to help.

### Set namespace on robots' onboard computers with environment variable

On the robot's onboard computer, open the terminal and type in

`nano ~/.bashrc`

Then add the following line to the end of the file:

`export ROS_NAMESPACE={namespace_you_choose}`

Save the file and don't forget to do

`source ~/.bashrc`

Since you're already on the robot's onboard terminal, it's a good idea to change the `ROS_MASTER_URI` and bringup the robot now.

### Set namespace on your laptop with environment variable

Use the same steps above. Make sure your namespace is exactly the same as the namespace of the robot you want to control.

### Set namespace for a termimal with temporary environment variable

To set a namespace temporarily for a terminal, which will be gone when you close the termial, just type in

`export ROS_NAMESPACE={namespace_you_choose}`

directly in your terminal window. You can use

`echo $ROS_NAMESPACE`

to check it.

### Launch/Run a file/node with namespace in terminal

Add a special key `__ns` (double underscore here!) to your command, for example:

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch __ns:={namespace_you_choose}`

However,

> Use of this keyword is generally not encouraged as it is provided for special cases where environment variables cannot be set. (http://wiki.ros.org/Nodes)

### Set namespace for a node in launch file

Use the attribute `ns`, for example:

`<node name="listener1" pkg="rospy_tutorials" type="listener.py" ns="{namespace_you_choose}" />`

### Publishing/Subsribing topics in other namespace

Do a `rostopic list`, you will find out topics under a namespace will be listed as `/{namespace}/{topic_name}`
