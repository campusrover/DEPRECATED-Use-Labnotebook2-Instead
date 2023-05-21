---
title: "Advanced: Help me troubleshoot weird build problems"
description: For advanced users who are trying to tix weird problems
author: Pito Salas
date: may-2023
status: new
type: faq
---
# Advanced troubleshooting of build problems

## Notes

You really need to know what you are doing when you are dealing at this level. 

## Radical Cleanup

* It turns out that it is safe to delete the build and devel subdirectories in ROS. 
* Sometimes this helps:

```
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make
source devel/setup.bash
```

## Another way

* First: lean your build by running "catkin_make clean" in the root of your workspace.
* Second: remake your project with "catkin_make"
* Third: re-source the devel/setup.bash in your workspace.

