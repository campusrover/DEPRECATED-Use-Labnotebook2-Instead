---
title: Running Multi Robot in Gazebo and Real Robot
author: Karen Mai
description: Are you interested in knowing how to get multiple robots running in gazebo and in the real world on the turtlebots? 
status: new
date: may-2023
---

<h2> Launching on Gazebo </h2>
If you are interested in launching on the real turtlebot3, you are going to have to ssh into it and then once you have that ssh then you will be able to all bringup on it. There is more detail about this in other FAQs that can be searched up. When you are running multirobots, be aware that it can be a quite bit slow because of concurrency issues.

These 3 files are needed to run multiple robots on Gazebo. In the object.launch that is what you will be running roslaunch. Within the robots you need to spawn multiple one_robot and give the position and naming of it.

<img width="190" alt="Screen Shot 2023-05-06 at 4 55 10 PM" src="https://user-images.githubusercontent.com/89604161/236646236-c7dd85ab-3c2b-4901-b6a6-58fdc1937613.png">

Within the object.launch of line 5, it spawns an empty world. Then when you have launched it you want to throw in the guard_world which is the one with the multiple different colors and an object to project in the middle. Then you want to include the file of robots.launch because that is going to be spawning the robots. 

<img width="575" alt="Screen Shot 2023-05-06 at 4 55 45 PM" src="https://user-images.githubusercontent.com/89604161/236646252-7c480a02-de15-4329-925f-4496ce140233.png">


For each robot, tell it to spawn. We need to say that it takes in a robot name and the init_pose. And then we would specify what node that it uses.

<img width="628" alt="Screen Shot 2023-05-06 at 4 56 08 PM" src="https://user-images.githubusercontent.com/89604161/236646266-d71ddf41-e15e-4b14-94af-cab2416c06e1.png">


Within the robots.launch, we are going to have it spawn with specified position and name. 
<img width="576" alt="Screen Shot 2023-05-06 at 4 56 43 PM" src="https://user-images.githubusercontent.com/89604161/236646283-87e8b496-dd4c-4199-93e5-c729a0fe95ff.png">

<h2> Launching Real </h2>

Make sure your bashrc looks something like this:
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

Then you need to ssh into the robots to start up the connection. You will need to know the ip of the robot and you can get this by doing a tailscale grep on the the robot of interest, and that can be roba, robb, robc, rafael, or donanotella as only these turtlebot are set up for that. 

Then you will need to then call on (bru mode real) and (bru name -m <ip address>) and multibringup. The multibringup is a script that Adam Rings wrote so that these connectiosn can set up this behavior. 
