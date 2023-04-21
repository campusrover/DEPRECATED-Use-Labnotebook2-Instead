---
title: BRU Concepts
---
# BRU Concepts

## Types of robots

BRU knows about types of robots. Currently:

1. platform
1. bullet
1. minirover
1. cat

Bru knows about all the instances of those types, for example, that platform2 is a specific robot of category platform. This knowledge is encoded in the bru.py script. This script is modified when creating a new type or instance of robot.

## Our software stack for all robots and vVMs

Currently [gpg_bran4](https://github.com/campusrover/gpg_bran4) is the repo containing all the custom ROS bits and pieces to make our robots go. There you will find, among other things, a directory for each BRU robot type. Each of those directories is structures as a standard ROS package, including in particular, a launch subdirectory.

## How each robot type is set up in BRU

As mentioned, under `~/catkin_ws/src/gpg_bran4` there are standard ROS package directories named after each of the robot types. For example, `~/catkin_ws/src/gpg_bran4/platform`. The most important subdirectory below that would be the launch directory, so for example `~/catkin_ws/src/gpg_bran4/platform/launch`. BRU defines a set of standard launch file names that should exist, and do the same thing, for each type of robot

**NB: This is a work in progress. Treat this description more as a spec than as a description of reality. We are working to this goal**

* `bringup_mini.launch` - Will launch the minimum nodes to allow the robot to move and have odometry.
* `bringup_full.launch` - Will launch the full set of nodes, for motion, odometry, lidar, and camera`

## Robot and "remote computer"

A key idea in ROS is that it is a distributed operating system. Separate computers, who can access each other over the network, together form a ROS system. ROS `nodes` are run on each of the computers, and they cooperate with each other. There is a key node called `roscore` which acts essentially as the traffic cop between them.

ROS nodes are always running at least on "the robot". But there can be other computers involved. In the Brandeis scenario, typically, the remote computer is a VM running on our cluster. And so you will have ROS nodes on the robot and additional ros nodes on the "remote computer" - the VM.

The coordination relies on the network and in particular that each side knows about the otjers' IP addresses. That is where BRU comes in.




