title: Campus Rover V 3
# Campus Rover Code Base 3 Project Summary

## Mission

To improve upon version 2 of the code base, focusing on changes to:

* Ease of expansion - building infrastructure that makes adding functional modules to the code base easy
* Reclaiming functionality of some nodes from version 1, that were unused in version 2
* Improving upon existing nodes
* Cleaning up the code base by throwing away unused files

Continue reading for details on each aspect of this project.

## Ease of Expansion

### Launch File Modularity

Rather than lumping all nodes into either "onboard" or "offboard" launch files, we have created a series of launch files that can be added to either main launch file. Furthermore, these launch modules can be disabled in the command line. Here is how this was achieved:

Here is the launch module for alexa voice integration:

``` xml
<launch>
  <!-- nodes that work with alexa to take orders and deliver items-->
  <node pkg="cr_ros_3" type="ngrok_launch.sh" name="ngrok_launch" output="screen"/>
  <node pkg="cr_ros_3" type="voice_webhook.py" name="voice_webhook" output="screen"></node>
  <node pkg="cr_ros_3" type="voice_destination_pub.py" name="voice_destination_pub" output="screen"></node>
</launch>
```

It is added to the offboard launch like so:

``` xml
<arg name="voice" default="true"/>
<group if="$(arg voice)">
  <include file="$(find cr_ros_3)/launch/voice.launch"/>
</group>
```

SO if you wish to launch offboard without voice, use this command:

``` sh
roslaunch cr_ros_3 mutant_offboard_rpicam.launch voice:=false
```

Going forward, all new features should be added to their own launch module, where the module contains all nodes that are required for the feature to work properly. The only excepition is if a new feature is added to the core of the package, and is required for the package to function properly.

### State Tools for State Manager Interfacing

A handful of changes have been made to the state manager interface. They include

* get_state now communicates via the state query service, rather than directly accessing the current_state field.
* get_state and change_state now have error handling to deal for the event that the state serivces are unavailable.
* New functions have been added to the state_tools file which should make interfacing with the state manager easier.

### Install Scripts

To make it easier for new users to begin working with the existing campus rover platform, we have created a set of install scripts that will make the setup time significantly faster, and remove all the guesswork. Refer to the cr_ros_3 package readme for installation instructions.

### Alien Compatibilty

Prior to cr_ros_3, the campus roverr platform only worked on robots running ROS Kinetic. Now, the platform will run on robot running ROS Melodic.

## Reclaiming Functionality

### Talk Queue

Formerly known as "message_switch", the talking queue was an overlooked node that did not recieve much attention until now, as it was overlooked for directly interfacing with the talk service. As the code base grows and more features may wish to use the on-board text to speech feature, a queue for all talking requests is a nessecary feature.

Using the talk queue from another node is easy. First, import the following:

``` python
from cr_ros_3.msg import ThingsToSay
from state_tools import talker
```

Set up a publisher to the /things_to_say topic

``` python
talker_pub = rospy.Publisher('/things_to_say', ThingsToSay, queue_size=1)
```

Whenever your node wishes to produce speech, utilize a line of code similar to this:

``` python
talker("I can talk!", talker_pub)
```

### Narrate Location

The location narrator is a novelty feature that has been reclaimed by:

* indtroducing a new set of waypoints in files/waypoints.json that correspond to the basement demo area
* modifying the existing script to narrate whenever the nearest waypoint or the state changes.

## Improving Existing Nodes

### Clearing Costmaps

Innaccuracies in costmaps can lead to difficulty in navigation. As such, we have implemented costmap clearing under the following circumstances:

* Navigation returns the 'aborted' code
* The robot exits the flying state and enters the lost state

Here's why:

* When navigation fails it is often because the costmap is muddled in some way that prevents a new route from being plotted. If the costmap is cleared and the goal is re-sent, then navigation may become successful.
* A human lifting the robot create a significant amount of lidar noise that is innacurate because the robot is not on the ground. Therefore, when the robot is place back on the ground it deserves a clean slate.

Here's how:

First, the Empty service topic must be impported

``` python
from std_srvs.srv import Empty
```

Then establish a serivce proxy

``` python
costmap_clearer = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
```

Finally, any time a node requires the costmap cleared, ultilize this line:

``` python
costmap_clearer()
```

### Automatic Dynamic Camera Reconfigure

The Rasbperry Pi camera that is mounted to Mutant is mounted such that by default, it is upside down. Fortunately, Ubiqutiy's rapicam_node that we ultize supports dynamic reconfiguration. This can be done manually using the following shell command:

``` sh
rosrun rqt_reconfigure rqt_reconfigure
```

This will bring up a GUI which will allow various aspects of the camera configuration to be manipulated. It also allows for a given configuration to be saved in a yaml file. Unfortunately, changes made through dynamic reconfiguration are temporary, and using a GUI to reconfigure every time is tedium that we strive to eliminate. The solution lies in camera_reconfigure.sh. We saved our desired configuration to camera.yaml from rqt_reconfigure. camera_reconfigure.sh uses that yaml file in this crucial line:

``` sh
roscd cr_ros_3 && rosrun dynamic_reconfigure dynparam load raspicam_node camera.yaml
```

Thus, automatic reconfiguration is acheived.

### Fiducials

Transforms are tricky. Version 2 never quite had accurate fiducial localizations due to bad static fiducial transforms. Through trial and error, static fiducial transforms have been edited and the pose esitmates have become more accurate as a result. We found it useful to refer to this static transform documentation to help remember the order of the arguments supplied to the static transform publisher:

``` xml
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
```

### Rviz Settings

If Rviz is not supplied the right settings file, it can produce a lot of frustrating errors. This was the case with our old settings file. We have generated a new settings file. How? Here are the steps:

1. launch default turtlebot navigation
2. save the settings from Rviz as a new file
3. set Rviz in mutant_navigation.launch to use the new settings file
4. make the necessary changes to the left bar to make Rviz visualize navigation properly (ex. using scan_filter as the scan topic rather than scan)
5. save your changes to your settings file

### Ngrok

We have added ngrok_launcher.sh which will bringup an ngrok tunnel to the alexa webhook to allow the alexa voice nodes to function.

### Updates to cr_web

Minor updates have been made to cr_web to accomodate a few changes to the code base. They are:

* localhost port changed from 5000 (default) to 3000, to accomodate cr web app and ngrok webhook running on the same machine
* Updated the map file to match current demo area layout.

## Throw Away Unused Files

Files related to the following were removed:

* gen2 facial recognition
* gen2 package delivery
* gen3 facial recognition
* gen3 hand detection

Fortunately, all these files still exist in version 1 and/or version 2 of the code base, so if they wish to be salvaged, they could be.

## Conclusion

The campus rover code base is arguably in it's best state that it has ever been in. We eagerly look forward to how it will continue to grow in the future.
