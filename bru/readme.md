# BRU - Brandeis Robotics Utilities

## What it is
First of all, BRU is not a finished product, it is constantly under development. You should expect inconsistencies and incompleteness. You are invited to fix and improve it. BRU embodies a series of python scripts, bash shells, and conventions, to make it more convenient to operate robots in the Brandeis Robotics Lab. We have many different robots that are all similiar but a little different. BRU attempts to unify and organize things in a sane manner.

## What is the problem BRU solves?
While all our robots run ROS (so far, just ROS1) they are all a little different. They use different SBC (single board computers) and different controllers. They have different peripherals. Also students use the robots either from their own laptop but more commonly from a virtual machine in our cluster environment. ROS has very partoicular requirements about IP addresses and environment variables. All together this can become very confusing.
