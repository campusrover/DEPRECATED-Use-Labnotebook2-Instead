---
title: "ROS Message Types"
description: A cheatsheet of the important message types (structures)
author: Pito Salas
date: may-2023
status: new
type: faq
---
# Package common_msgs
## geometry_msgs/Twist.msg
* Vector3 linear
* Vector3 angular

## geometry_msgs/Pose2D.msg
* float64 x
* float64 y
* float64 theta

## geometry_msgs/Pose.msg
* Point position
* Quaternion orientation

## geometry_msgs/Quaternion.msg
* float64 x
* float64 y
* float64 z
* float64 w

## geometry_msgs/QuaternionStamped.msg
* Header header
* Quaternion quaternion

## std_msgs/Header.msg
* uint32 seq
* time stamp
* string frame_id


## geometry_msgs/Vector3.msg
* float64 x
* float64 y
* float64 z

## geometry_msgs/Point.msg
* float64 x
* float64 y
* float64 z






