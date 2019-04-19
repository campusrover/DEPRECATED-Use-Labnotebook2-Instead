# Setting up Mutant
In the event that a new mutant ever has to be set up, or software problems require mutant to be reset, here are the necessary tips to installing everything that is needed to operate using the most recent campus rover ros package.

## Installing ros
[The Robotis Emanual](http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-ubuntu-mate) serves as an adequate guide to getting 95% of the way to setting up ROS on a mutant turtlebot. There are a few divergences from their instructions, though:
* In part 3 of section 6.2.1.1, you could easily miss the note about installing what is needed to use a raspi camera. Either do not miss it (it is right below the first block of terminal commands) or check out [our labnotebook page on configuring the raspberry pi camera](RaspiCam.md).
* just below the camera hint, the emanual instructs to use these commands to remove unneeded packages:
`cd ~/catkin_ws/src/turtlebot3`
`sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/`
However, slam and navigation are actually useful to us, so use these commands instead:
`cd ~/catkin_ws/src/turtlebot3`
`sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_example/`
* Once you have finished the emanual SBC setup, you still need to install a few dependencies that Robotis assumes you will only use on a remote pc. For your convenience, run this command:
`sudo apt-get install ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers`
If you are curious as to where this came from, it is an edited version of the first command of emanual section 6.1.3, under PC setup.


## On the remote PC
you will need:
* the fiducial package `sudo apt-get install ros-kinetic-fiducials`
* cr_ros_2 github repo on branch mutant_transfer
* cr_web github repo on branch mutant
* the turtlebot3_navigation package, which should come with turtlebot3.
* google chrome (for the web app - you can use another browser, but that would require editing a shell script in the cr_web package)
* flask module for python

### Using Git and Github
If this class is your first time using github - don't worry! Though it may seem mildly confusing and daunting at first, it will eventually become your best friend. There's a pretty good guide called [git - the simple guide - no deep shit!](http://rogerdudler.github.io/git-guide/) which can walk you through the basics of using git in the terminal. Here's a bit of a short guide to the commands we have used most in gen3:
* `git clone` is how you will initially pull a repository off of Github
* in a repository's main directory in your terminal, `gs` (or `git status`) is a super useful command that will show you all the files you have edited.
* `git add` --> `git commit -m` --> `git push` is how you will be updating your repos on github. **Pro Tip:** if you've edited multiple files in a subdirectory, for example in src, then you can type `git add src` to add all modified files in src, rather than typing each file individually.
* always do a pull before a push if you think someone else has made edits to your repo.
* if you've made changes locally that you don't want to keep, `git reset --hard` will revert back to your last pull or clone.  

## Longterm Troubleshooting
This section should be updated as problems arise and their solutions are discovered

### What if .bashrc (accidentally) gets deleted?
Restoring `.bashrc` is not very difficult.

First, type `/bin/cp /etc/skel/.bashrc ~/` into your terminal. This will replace a corrupt .bashrc file with the default .bashrc.
Then, try `ls -a` in your user directory to view all files, including hidden ones, such as .bashrc. If a file such as `.bashrc.swp` appears, delete it with `rm .bashrc.swp`. Continue removing files that look related to .bashrc until `nano ~/.bashrc` properly opens the file without giving an error.
Now you'll have to restore all the lines at the bottom of .bashrc that were added when ROS was installed. Fortunately, there are many computers in the lab and they all have almost identical .bashrc files! ssh from one of those computers to the one that you are restoring, copy the lines that need to be re-added, and then edit them to fit the computer that the .bashrc file is located on (things to consider: namespace, ip address, aliasas that are useful on the given machine)
Don't forget to `source ~/.bashrc` after you are done editing the file, so the changes you made can take effect in the terminal!

Now don't make the same mistake twice.

### The Amazon Echo attached to the robot is in an infinite boot loop!
This is probably because the volume was turned up too high, and the raspberry pi cannot supply enough power. Plug the Echo into a wall outlet, turn the volume down, and then plug it back into the robot. Rule of thumb: keep the echo's volume no greater than 70%.

## One of mutant's wheels isn't spinning!
Turn the robot all the way off (this means powering off the reaspberry pi, then switching the openCR board off as well), then turn it back on. If the wheel continues to not spin after this, then consult the lab's resident roboticist, Charlie.

### Nodes aren't subscribing to the right topics!
The best way to debug topic communication is through rqt_graph in the terminal. This will create a visual representation of all nodes and topics currently in use. There are two setting to toggle:
- First, in the upper left, select Nodes/Topics (all) (the default setting is Nodes only)
- Next, in the check box bar, make sure it is grouped by namespace and that dead sinks, leaf topics and unreachable topics are un-hidden.
Now it's important to note that nodes appear on the graph as ovals, and topics appear in the graph as boxes. hovering over a node or topic will highlight it and all topics/ nodes connected to it. This will help to show whether a node is subscribing to and publishing all the nodes that it is expected to.
Based on the rqt graph information, update your topic names in your nodes and launch files to match what you expect.

### Namespacing a topic from a node that you can't edit
Some nodes automatically namespace their published topics, but some don't. This can be an annoyance when you desire a mutli-robot setup. Fear not, it is possible to namespace topics that aren't auto-namespaced. There are two ways to do this, and both require some launch file trickery.
One way is to declare a topic name, then to remap the default topic name to your new one. The below example is used in the file move_base_mutant.launch
```
<arg name="cmd_vel_topic" default="/$(env ROS_NAMESPACE)/cmd_vel" />
...
<remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>

```
The new topic name argument can be declared anywhere in the launch file. The remapping must occur nested within the node tag of the node which publishes the topic you want renamed. This method has been used and approved by the students of gen3.

The other way is to use a group tag, and declare a namespace for the entire group. gen3 has not tested this method, but [there does exist some documentation about it on the ROS wiki.](http://wiki.ros.org/roslaunch/XML/group)

Gen3's preferred method of namespacing nodes that we have written ourselves is as follows:
Before initializing any publishers or subscribers, make sure this line of code is present:
`ns = rospy.get_namespace()`
Then, use python string formatting in all your publishers and subscribers to namespace your topics:
`cmd_pub = rospy.Publisher('{}cmd_vel'.format(ns), . . .)`
`ns` will contain both backslashes needed for the topic to conform to ROS's topic formatting guidelines, e.g. the publisher above will publish to the topic `/namespace/cmd_vel`.

### list of known nodes that don't get automatically namespaced:
* /fiducial_transforms (from the node aruco_detect)
* /diagnostics (from turtlebot3_core)
* /cmd_vel (from move_base)
