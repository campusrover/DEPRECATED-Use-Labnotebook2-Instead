# Running multiple robots on a single roscore

Joshua Liu

# Normal behavior

Normally, running the bringup launch file will create a new roscore running on that robot. This, of course, is not desired.

# Step 0: Decide on your desired structure

These configurations should both be possible:

* Run roscore on one robot 

* Run roscore on a seperate computer (probably your VNC environment)

We will assume the first configuration here.       
I have not actually testes the second one, but I see no reason it wouldn't work.

# Setp 1: Edit .bashrc file of your "auxiliary" robot(s)

SSH into your auxiliary robot (We will use "auxiliary" here to mean "not running roscore").

Open the .bashrc file in your editor of choice

Navigate to where it says

```
$(bru name ROBOTNAME -m 127.0.0.1)
```

Change the ROBOTNAME to the name of your computer (robot is computer too) that will run roscore. Change the IP address to the IP address of your core robot.    
Save your changes.

# Step 2: Copy and edit the bringup launch file

Do this for **each** robot, even your "core" (running roscore) one.

While still SSH'ed into that robot, run `roscd bringup`    
Navigate to the `launch` directory    
Run `cp turtlebot3_robot.launch turtlebot3_multi_robot.launch` to copy the file. Please do this.    

Open the newly created `turtlebot3_multi_robot.launch` file in your editor of choice.    

At the top, there should be two `<arg>` tags.

After them, put on a new line: `<group ns="$(arg multi_robot_name)">`

Then, right before the very bottom `</launch>`, add a closing `</group>` tag.    
The result should look something like:

```
<launch>
	<arg> lorem ipsum dolor sit amet ... </launch>
	<arg> lorem ipsum dolor sit amet ... </launch>
	<group ns="$(arg multi_robot_name)">
		<stuff> </stuff>
		<stuff> </stuff>
		<stuff> </stuff>
		<stuff> </stuff>
	</group>
</launch>
```

Repeat the two above steps on all your auxiliary robots.

What does this do? While it is not necessary, this will prefix all the nodes created by bringup with the robot's name.     
If you don't do this, there will be many problems.

# Launching the robots

When SSH'ed into a robot, you can't just run `bringup` anymore. Use this command instead:

`roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=THEROBOTNAME`

This will cause the bringup nodes to be properly prefixed with the robot's name.    
For example, `raspicam_node/images/compressed` becomes `name/raspicam_node/images/compressed`.

As long as you launch the core robot first, this should work.

See what it looks like by running `rostopic list` from your (properly configured to point at the core computer) vnc environment.

# Acknowledgments

Very many thanks to August Soderberg, who figured most of this out.