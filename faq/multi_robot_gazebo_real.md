---
title: Running Multi Robot in Gazebo and Real Robot
author: Karen Mai
description: Are you interested in knowing how to get multiple robots running in gazebo and in the real world on the turtlebots? 
status: new
date: may-2023
---

<pre><code>
# export ROS_MASTER_URI=http://100.74.60.34:11311
export ROS_MASTER_URI=http://100.66.118.56:11311
# export ROS_IP=100.66.118.56
export ROS_IP=100.74.60.34
# Settings for a physical robot 
$(bru mode real)
# $(bru name robb -m 100.99.186.125)
# $(bru name robc -m 100.117.252.97)
$(bru name robc -m $(myvpnip))
# $(bru name robb -m $(myvpnip))
</code></pre>

If you are interested in launching on the real turtlebot3, you are going to have to ssh into it and then once you have that ssh then you will be able to all bringup on it. There is more detail about this in other FAQs that can be searched up. When you are running multirobots, be aware that it can be a quite bit slow because of concurrency issues.

These 3 files are needed to run multiple robots on Gazebo. In the object.launch that is what you will be running roslaunch. Within the robots you need to spawn multiple one_robot and give the position and naming of it.
