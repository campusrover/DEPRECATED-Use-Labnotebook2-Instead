---
title: How do I visualize the contents of a bag
description: Webviz is an advanced online visualization tool
status: new
type: FAQ
date: may-2023
author: Pito Salas
---
# Webviz is an advanced online visualization tool

I have spent many hours tracking down a tricky tf tool and in doing that came across some new techniques for troubleshooting. In this FAQ I introduce using rosbag with Webviz

## Rosbag

This CLI from ROS monitors all topic publications and records their data in a timestamped file called a bag or rosbag. This bag can be used for many purposes. For example the recording can be "played back" which allows you to test the effects that a certain run of the robot would have had. You can select which topics to collect.

One other use is to analyze the rosbag moment to moment.

## Webiz

Is an online tool (https://webviz.io). You need to understand topics and messages to use it and the UI is a little bit obscure. But it's easy enough. You supply your bag file (drag and drop) and then arrange a series of panes to visualize all of them in very fancy ways.

