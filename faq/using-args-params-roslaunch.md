---
title: How do I use Parameters and Arguments in ROS?
author: Evalyn Berleant, Kelly Duan
description: Arguments and parameters are important tags for roslaunch files that are similar, but not quite the same.

date: may-2021
status: OK
type: FAQ
---
#### Evalyn Berleant, Kelly Duan

Arguments and parameters are important tags for roslaunch files that are similar, but not quite the same.

### What are parameters?

Parameters are either set within a launch file or taken from the command line and passed to the launch file, and then used within scripts themselves.

#### Getting parameters

Parameters can be called inside their nodes by doing

``` python
# get a global parameter
rospy.get_param('/global_param_name')

# get a parameter from our parent namespace
rospy.get_param('param_name')

# get a parameter from our private namespace
rospy.get_param('~private_param_name')
```
Example from [ROS parameter tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters).

#### Adding parameters to launch files

#### Setting Parameters

Parameters can be set inside nodes like such (python):

``` python
rospy.set_param('some_numbers', [1., 2., 3., 4.])
rospy.set_param('truth', True)
rospy.set_param('~private_bar', 1+2)
```

For instance, if you wanted to generate a random number for some parameter, you could do as follows:

``` xml
<param name="robot_position" command="$(find some_package)/scripts/generate_random_position.py"/>
```
which would generate a random position for the parameter.

Be careful that if you are setting parameters in more than one place that they are set in order correctly, or one file may overwrite the parameter’s value set by another file. (See links in resources for more detail).

### What are arguments?

While parameters can pass values from a launch file into a node, arguments (that look like `<arg name=”name”/>` in the launch file) are passed from the terminal to the launch file, or from launch file to launch file. You can put arguments directly into the launch file like such and give it a value (or in this case a default value):

```xml
<launch>
  <arg name="x_pos" default="0.0" />
  <arg name="y_pos" default="0.0" />
  <arg name="z_pos" default="0.0" />
...
```

Or you can pass arguments into “included” files (launch files included in other launch files that will run):
``` xml
<!-- start world and launch gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find swarmbots)/worlds/$(arg world).world"/>
    <arg name="paused" value="true"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
```

### Substitution args

Substitution args, recognized by the `$` and parentheses surrounding the value, are used to pass values between arguments.
Setting the value of a parameter or argument as `value=”$(arg argument_name)”` will get the value of argument_name in the same launch file.
Using `$(eval some_expression)` will set the value to what the python expression at `some_expression` evaluates to.
Using `$(find pkg)` will get the location of a package recognized by the catkin workspace (very often used).

The `if` attribute can be used on the group tag, node tag, or include tag and work like an if statement that will execute what is inside the tag if true. By using `eval` and `if` together, it is possible to create loops to run files recursively. For example, running a launch file an arbitrary number of times can be done by specifying the number of times to be run in the launch file, including the launch file within itself, and decrementing the number of times to be run for each recursive `include` launch, stopping at some value checked by the `if` attribute.
Here is an example of a recursive launch file called `follower.launch` to spawn in robots.

``` xml
<launch>
  <arg name="followers" />
  <arg name="ns" />

  <!-- BEGIN robot[#] -->
  <group ns="$(arg ns)">

    <param name="tf_prefix" value="$(arg ns)" />
    <include file="$(find swarmbots)/launch/one_robot.launch">
      <arg name="robot_name" value="$(arg ns)" />
      <arg name="followers" value="$(arg followers)" />
    </include>
  </group>

  <!-- recursively start robot[#-1] -->
  <arg name="new_followers" value="$(eval arg('followers') - 1)" />
  <include file="$(find swarmbots)/launch/follower.launch" if="$(eval arg('new_followers') >= 0)">
    <arg name="followers" value="$(arg new_followers)" />
    <arg name="ns" value="robot$(arg new_followers)" />
  </include>
</launch>
```

`followers` here will impact the number of times the launch file is recursively called. `$(eval arg('followers') - 1)` will decrement the value of `followers` inside each recursive launch, and the `if` attribute

``` xml
 if="$(eval arg('new_followers') >= 0)"
```

checks that once the new number is below 0, it will not call the launch file again.

### Differences between arguments and parameters (important!)

Both arguments and parameters can make use of substitution args. However, arguments cannot be changed by nodes like parameters are with `rospy.set_param()`. Because of the limits of substitution, you cannot take the value of a parameter and bring it to an argument.
If you want to use the same value between two params that require generating a specific value with `rospy.set_param()` then you should create another node that sets both parameters at once.

For example, this script

``` python
#!/usr/bin/env python
import random, rospy, roslib

rospy.init_node("generate_random_x")
pos_range = float(rospy.get_param('pos_range', 3))

x_pos = random.uniform(-pos_range / 2, pos_range / 2)
rospy.set_param('map_merge/init_pose_x',x_pos)

print(x_pos)
```

is called within a parameter using the `command` attribute.

``` xml
<param name="x_pos" command="$(find swarmbots)/src/generate_random_x.py" />
```

The command attribute sets the value of the parameter to whatever is printed by `stdout` in the script. In this case, the script generates a random number for `x_pos`. In the same file, `rospy.setparam` is called to set another parameter to the same value of `x_pos`. In that way, both parameters can be set at once.

### Too many parameters? Use rosparam!

If you have too many parameters and/or groups of parameters, not only is it inefficient to write them into a launch file, but is also prone to many more errors. That is when a rosparam file comes in handy--a rosparam file is a YAML file that stores parameters in an easier-to-read format.
A good example of the utility of rosparam is the parameters for move_base, which uses the command
``` xml
<rosparam file="$(find turtlebot3_navigation)/param/local_costmap_params.yaml" command="load" />
```
which loads the parameters from the yaml file here:
``` yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint

  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5  

  static_map: false  
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```



### References
- [Ros Parameters Tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters)
- [Substitution args](http://wiki.ros.org/roslaunch/XML#substitution_args)
- [Order of launch files](https://answers.ros.org/question/199608/roslaunch-order-of-rosparams/)
