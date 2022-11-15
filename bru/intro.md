# Concepts

## Types of robots

BRU knows about types of robots. Currently:

1. platform
1. bullet
1. minirover
1. cat

Bru knows about all the instances of those types, for example, that platform2 is a specific robot of category platform. This knowledge is encoded in the bru.py script. This script is modified when creating a new type or instance of robot.

## Our software stack for all robots and vVMs

Currently [gpg_bran4](https://github.com/campusrover/gpg_bran4) is the repo containing all the custom ROS bits and pieces to make our robots go. There you will find, among other things, a directory for each BRU robot type. Each of those directories is structures as a standard ROS package, including in particular, a launch subdirectory.

## Standard Launch Files

**this is definitely a work in progress**




