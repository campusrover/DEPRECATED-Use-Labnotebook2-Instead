# Modifying LaserScan message (Or How to Access Turtlebot Files)
### by Harry Zhu (courtesy of TA Adam Ring) 

This guide showcases how you can modify the definitions (e.g. range_min) of the LaserScan message published to the /scan topic. It is also serves as a brief “guide” on how to access and modify files on a Turtlebot. 

Turtlebot’s LaserScan message is specified by the ld08_driver.cpp file from this repo: https://github.com/ROBOTIS-GIT/ld08_driver/tree/develop/src. 

To modify this file on a Turtlebot, you can either unplug its SD card and mount it to your computer or do it through terminals in vnc. Here, we will do it through vnc. 

## Step 1: ssh into your robot, do not do bringup 
## Step 2: cd catkin_ws/src/ld08_driver/src
## Step 3: nano ld08_driver.cpp
You will enter a editor 
## Step 4: go to line 63 scan.range_min = 0.0; 
## Step 5: modify the value as you see fit
## Step 6: ctrl + S to save, ctrl + X to exit
## Step 7: cm 
