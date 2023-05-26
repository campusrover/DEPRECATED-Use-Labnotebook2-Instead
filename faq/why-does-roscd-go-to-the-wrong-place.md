---
title: Why does roscd go wrong?
description: When you run a .launch it grabs packages from the wrong place
type: faq
status: new
author: Pito Salas (with Stackoverlow)
---
The issue is with your ROS_PACKAGE_PATH. roscd brings you to the first workspace on the path. So they should be the other way around on you package path.

I highly discourage from manually fiddling with the ROS_PACKAGE_PATH variable (if you've done that or plan on doing that). Also note that you don't need to have multiple source statements for workspaces in your .bashrc. With catkin, the last setting wins.

* clean your workspace (i.e. remove build, devel and logs folder, if they exist; if you've built with catkin-tools, you can use the catkin clean command)
* clean the bashrc (i.e. remove all source commands that source a ROS workspace)
* start a new terminal (without any ROS environment sourced)
* manually source /opt/ros/melodic/setup.bash
* build your workspace again and source the workspaces setup.bash

