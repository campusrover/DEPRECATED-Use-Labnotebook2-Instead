# Launch Files

This page will serve as a one stop shop for understanding the onboard and offboard launch files used for campus rover mark 3: 'Mutant' and 'Alien'. Along the way, it will also serve as a launch file tutorial in general.

This is a simple walkthrough of how to create a launch file: 
labnotebook/faq/launch_file_create.md

## On-board launch file

On-board is a short and rather simple launch file. Generally, a node should be run on-board if it meets one of two criteria:

1. The node interfaces directly with the hardware of the robot, and therefore must be onboard
2. The node is lightweight enough that it can run on the raspberry pi without causing too much strain to the cpu.

### Nodes that must be on-board for hardware purposes

```xml
<include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch"/>
```

The line above launches the normal turtlebot bringup script. This makes the `include` tag very useful, because it is the equivalent of a `roslaunch` terminal command. In short - by using the include tag, a launch file can launch other launch files. The `file` argument is the path to the launch file you want to include. `$(find <package name>)` does what it says - finds the path to the specified package in your catkin workspace.

```xml
<node pkg="raspicam_node" type="raspicam_node" name="raspicam_node" output="screen">
  <param name="camera_info_url" value="package://turtlebot3_bringup/camera_info/turtlebot3_rpicamera.yaml"/>
  <param name="width" value="640"/>
  <param name="height" value="480"/>
  <param name="framerate" value="50"/>
  <param name="enable_raw" value="true"/>
  <param name="camera_frame_id" value="camera"/>
</node>
```

Here we see a node with parameters. This snippet launches the raspberry pi camera, which must be on-board in order to publish images captured by the camera. Often, the documentation on nodes like this will inform you of all the parameters and what their values mean.

The cpu checker node is also on-board, because it uses a python module to monitor the current cpu usage at any given time, then publish it as a ROS topic.

the talk service must be on-board so that it can produce audio through the robot's audio port.

### Nodes that are onboard because they are lightweight

```xml
<node pkg="cr_ros_2" type="scan_filter.py" name="scan_filter" output="screen"></node>
<node pkg="cr_ros_2" type="detect_pickup.py" name="pickup_checker" output="screen"></node>
<node pkg="cr_ros_2" type="rover_controller.py" name="rover_controller" output="screen"></node>
<node pkg="cr_ros_2" type="state.py" name="state" output="screen"></node>
```

pickup\_checker and scan\_filter are both lightweight nodes that are ideal for being included on-board. The state manager is also a rather small node, if you believe it or not - all it has to do is store the current state and make state changes.

rover\_controller is arguably the anomaly - it does quite a bit to communicate with the web app and deal with navigation goals and completion. It could easily be moved off-board.

## Off-board launch file

```xml
<arg name="map_file" default="$(find cr_ros_2)/files/basement_map.yaml"/>
```

Here we see an arg defined on it's own, outside of a node launch. This behaves the same way as assigning a variable. The value can be accessed at any point in the launch file, as demonstrated below:

```xml
<include file="$(find cr_ros_2)/launch/mutant_navigation.launch">
  <arg name="map_file" value="$(arg map_file)"/>
  <arg name="scan_topic" value="scan_filter"/>
  <arg name="open_rviz" value="true"/>
  <arg name="move_forward_only" value="false"/>
</include>
```

This snippet launches our custom navigation launch file, and you can see on line 2 how `$(arg <arg name>)` passes the value. Since this value is only passed once, you could say in this case is is redundant, but you can image how if you had to use the value multiple times how it could be very useful.

```xml
<node pkg="topic_tools" type="throttle" name="cam_throttle" args="messages /$(env ROS_NAMESPACE)/raspicam_node/image/compressed 2" />
```

This line uses a provided topic tool to throttle the publish rate of the given topic down to 2 hz. More importantly, `$(env <var>)` is used to get the value of the given environment variable, which must be defined in .bashrc.

``` xml
<node pkg="aruco_detect" name="aruco_detect"
  type="aruco_detect" respawn="false">
  <param name="image_transport" value="$(arg transport)"/>
  <param name="publish_images" value="true" />
  <param name="fiducial_len" value="$(arg fiducial_len)"/>
  <param name="dictionary" value="$(arg dictionary)"/>
  <param name="do_pose_estimation" value="true"/>
  <remap from="/camera/compressed"
      to="$(arg camera)/$(arg image)/$(arg transport)_throttle"/> <!-- removed throttle -->
  <remap from="/camera_info" to="$(arg camera)/camera_info"/>
  <remap from="/fiducial_transforms" to="/$(env ROS_NAMESPACE)/fiducial_transforms" />
</node>
```

Now we see the use of remapping topics. This is very useful particularly in namespacing, as well as ensuring that a node is subscribed to the right topic for a certain kind of data - such as camera feed, in this case.

``` xml
<node pkg="tf" type="static_transform_publisher" name="fid_153" args="19.6 21.8 0.825 0 3.14159 0 /map /fid_153 100" /> <!-- charging dock -->
```

The final part of the file are all of the static transforms - these are entities that do not move with respect to their reference frame. The six numerical args are, in order, from left to right, x, y, z, yaw, pitch, roll. **NOTE:** the orientation of fiducials needs to be reviewed and corrected - though the positions are very accurate

## Additional Tricks

### Args vs Params

Here is the key difference:

1. args are self-contined within launch files. They are defined and accessed only within launch files.
2. Params are how to pass values (such as args) into an executable node. They send values to the ROS parameter server. For a param to be accepted within a node, it must get the paramter from the server. Here is an example from rover_controller.py:

``` python
# This is how the python file gets the parameter. notice the paramter is namespaced to the name of the node
cam_topic = rospy.get_param("/rover_controller/cam_topic")
```

``` xml
<!-- This is how the paramter is passed into the node-->
<node pkg="cr_ros_3" type="rover_controller.py" name="rover_controller" output="screen">
    <param name="cam_topic" value="/usb_cam_node/image_raw/compressed"/>
  </node>
```

### Grouping and Logic

The `<group>` tag allows for clusters of node to only be launched if the value passed to the group tag is true. A basic example is seen with modules like voice:

``` xml
<arg name="voice" default="true"/>
<group if="$(arg voice)">
  <include file="$(find cr_ros_3)/launch/voice.launch"/>
</group>
```

You can see that voice.launch is included only if the voice arg is true. the voice arg can be manually set to false to diable voice, as documented in the cr_ros_3 readme.

But what if your condition is based on something else? There is a way to evaluate operations in launch to a boolean. Observe:

``` xml
<group if="$(eval arg('robot') == 'ALIEN')">
```

`eval` will take a python-evaluable boolean expression and evaluate it to true or false. This group above will only launch when the campus rover model is set to "ALIEN".


### File path shortcut

As you know, `$(find <package name>)` will return the absolute path to the specified package, which allows for launch files from eternal packages to be included in your launch. A handy shortcut exists for launch files in the same package. Suppose A.launch and B.launch are in the same launch folder. A.launch can include B.launch like so using `$(dirname)`:

``` xml
<include file="$(dirname)/B.launch">
```

This trick only works in ROS versions that are newer than Lunar.

### Shell scripts

Shell scripts can be run in roslaunch the same way nodes are launched. Here's an example:

``` xml
<node pkg="cr_ros_3" type="camera_reconfigure.sh" name="camera_reconfigure" output="screen"/>
```

The above example runs a script that will dynamically flip upside down raspberry pi camera's video output.
