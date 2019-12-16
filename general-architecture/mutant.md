# Mutant

To launch Mutant, follow these steps:

1. Ensure that mutant has the most recent version of `cr_ros_2`. This can be accomplished by running `roscd cr_ros_2` and then `gp`.
2. SSH into the mutant and run `bu-mutant`. This will launch the mutant onboard bringup.
3. On your local machine \(again after making sure that you have the most recent version of `cr_ros_2`, run `roslaunch cr_ros_2 mtnt_onb_rpicam.launch`. This command will start the web app, and you can proceed from there.

## Longterm Troubleshooting

This section should be updated as problems arise and their solutions are discovered

### SSH is slow/unresponsive/won't connect, etc.

We've found that forcing powercycles with the switch on the OpenCR board can be detrimental to the robot's ability to SSH. We recommend running `sudo poweroff` every time you want to powercycle, if possible. Give the robot ample time to fully shut down before turning it back on. Usually, when turning the robot back on, waiting for the Echo Dot to fully turn on is a good indicator of when the robot will be ready to SSH - if the Echo is powered through the Raspberry pi board. Another indicator is that if the echo is setup to communicate with the robot via bluetooth, then the Echo will say "now connected to mutant". This means the robot is ready. \(please note - even though the Echo says it is connected via bluetooth, the raspberry pi does not default to use the Echo as an audio sink, so it will not play audio from the robot.\)

Another solution we have found is to disable DNS in SSH settings on the robot. Go to `etc/ssh` and then open the config file with `sudo nano sshd_config`. If there is a line that says `UseDNS yes` then change the `yes` to `no`. If `UseDNS` is not present in the file, then add the line `UseDNS no` to the bottom of the file.

### The Amazon Echo attached to the robot is in an infinite boot loop!

This is probably because the volume was turned up too high, and the raspberry pi cannot supply enough power. Plug the Echo into a wall outlet, turn the volume down, and then plug it back into the robot. Rule of thumb: keep the echo's volume no greater than 70%.

### One of mutant's wheels isn't spinning!

Turn the robot all the way off \(this means powering off the reaspberry pi, then switching the openCR board off as well\), then turn it back on. If the wheel continues to not spin after this, then consult the lab's resident roboticist, Charlie.

### The robot isn't working and I have literally no idea why!

1. Shut down and restart roscore.
2. Make sure everything is namespaced correctly \(on your local machine\) - see above for namespacing troubleshooting.
3. Check that the battery is fully charged.
4. Your node may have crashed right off the bat - check `rqt_graph` and check that all nodes that are supposed to be communicating with each other are actually communicating.

### In RVIZ, the robot appears to be jumping around and won't get any sort of pose!

This is usually caused by interference from another robot. Even with namespacing, another robot running on the roscore can cause interference specifically on rviz. We have determined that the likely cause of this is because the odom tf \(transform\) is not namespaced.

### The camera feed from the raspicam is flipped

1. Type `rosrun rqt_reconfigure rqt_reconfigure` into the command line.
2. Click the boxes to flip the image hortizontally/vertically.
3. To check whether the image is flipped correctly, just run `rqt_imageview`.

**For further details on the raspicam, look at Hardware - Raspberry Pi Camera page in the lab notebook.**

### Nodes aren't subscribing to the right topics!

The best way to debug topic communication is through rqt\_graph in the terminal. This will create a visual representation of all nodes and topics currently in use. There are two setting to toggle:

* First, in the upper left, select Nodes/Topics \(all\) \(the default setting is Nodes only\)
* Next, in the check box bar, make sure it is grouped by namespace and that dead sinks, leaf topics and unreachable topics are un-hidden.

  Now it's important to note that nodes appear on the graph as ovals, and topics appear in the graph as boxes. hovering over a node or topic will highlight it and all topics/ nodes connected to it. This will help to show whether a node is subscribing to and publishing all the nodes that it is expected to.

  Based on the rqt graph information, update your topic names in your nodes and launch files to match what you expect.

### Namespacing a topic from a node that you can't edit

Some nodes automatically namespace their published topics, but some don't. This can be an annoyance when you desire a mutli-robot setup. Fear not, it is possible to namespace topics that aren't auto-namespaced. There are two ways to do this, and both require some launch file trickery. One way is to declare a topic name, then to remap the default topic name to your new one. The below example is used in the file move\_base\_mutant.launch

```text
<arg name="cmd_vel_topic" default="/$(env ROS_NAMESPACE)/cmd_vel" />
...
<remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
```

The new topic name argument can be declared anywhere in the launch file. The remapping must occur nested within the node tag of the node which publishes the topic you want renamed. This method has been used and approved by the students of gen3.

The other way is to use a group tag, and declare a namespace for the entire group. gen3 has not tested this method, but [there does exist some documentation about it on the ROS wiki.](http://wiki.ros.org/roslaunch/XML/group)

Gen3's preferred method of namespacing nodes that we have written ourselves is as follows: Before initializing any publishers or subscribers, make sure this line of code is present: `ns = rospy.get_namespace()` Then, use python string formatting in all your publishers and subscribers to namespace your topics: `cmd_pub = rospy.Publisher('{}cmd_vel'.format(ns), . . .)` `ns` will contain both backslashes needed for the topic to conform to ROS's topic formatting guidelines, e.g. the publisher above will publish to the topic `/namespace/cmd_vel`.

#### list of known nodes that don't get automatically namespaced:

* /fiducial\_transforms \(from the node aruco\_detect\)
* /diagnostics \(from turtlebot3\_core\)
* /cmd\_vel \(from move\_base\)
* Any and all transforms

### Configuring the size of the wheels within the OpenCR firmware

Many components of Turtlebot3 software depend on knowing the size of wheels of the robot. Some examples include Odometry, cmd\_vel \(the turtlebot core\), and move\_base. By default, turtlebot3 wheels are 6.6cm in diameter. Mutant has 10cm diameter wheels. If you use larger wheels, but the software believes it has smaller wheels, then movement behavior will not be as expected.

On your remote pc, follow steps 4.1.1 through 4.1.6 [of this Robotis e-manual guide](http://emanual.robotis.com/docs/en/parts/controller/opencr10/) to install the arduino IDE and configure it to work with the OpenCR board. Please note that as of May 2019, the latest version of the Arduino IDE is 1.8.9, but the guide displays version 1.6.4.

In the IDE, go to `File` --&gt; `Examples` --&gt; `TurtleBot3` --&gt; `turtlebot3_waffle` \(or `turtlebot3_burger` if that better fits the form factor of your mutant TB3\) --&gt; `turtlebot3_core`. This will open three files: `turtlebot3_core`, `turtlebot3_core_config.h` and `turtlebot3_waffle.h`. Go to `turtlebot3_waffle.h`. You will see that this file defines a number of characteristics of a robot, including wheel radius! Edit your wheel radius variable, then save your work - two windows will pop up. In the first one, click "ok" and in the second, click "save" - don't edit anything!

Now it's time to upload the edited firmware to the OpenCR board. This is actually not too difficult - first, unplug the usb cable that connects the OpenCR board to the Raspberry Pi \(or whatever SBC your mutant is using\), and plug it into your remote PC. You may have noticed that you weren't able to select a port in step 4.1.5.3 of the emanual instructions - now that the OpenCR board is connected, you should be able to select the port. Once you've done that, click the upload button at the top of the IDE to upload the firmware to your robot. Once the upload is complete, your firmware should be updated and your robot should behave as expected in movement-based tasks. To test, make the robot move forward at 0.1 m/s for 10 seconds - it should travel about a meter. If not, your firmware may not have been updated properly or your wheel measurements may be incorrect.

### What if .bashrc \(accidentally\) gets deleted?

Restoring `.bashrc` is not very difficult.

First, type `/bin/cp /etc/skel/.bashrc ~/` into your terminal. This will replace a corrupt .bashrc file with the default .bashrc. Then, try `ls -a` in your user directory to view all files, including hidden ones, such as .bashrc. If a file such as `.bashrc.swp` appears, delete it with `rm .bashrc.swp`. Continue removing files that look related to .bashrc until `nano ~/.bashrc` properly opens the file without giving an error. Now you'll have to restore all the lines at the bottom of .bashrc that were added when ROS was installed. Fortunately, there are many computers in the lab and they all have almost identical .bashrc files! ssh from one of those computers to the one that you are restoring, copy the lines that need to be re-added, and then edit them to fit the computer that the .bashrc file is located on \(things to consider: namespace, ip address, aliasas that are useful on the given machine\) Don't forget to `source ~/.bashrc` after you are done editing the file, so the changes you made can take effect in the terminal!

Now don't make the same mistake twice.

