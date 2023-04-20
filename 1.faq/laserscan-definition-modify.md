# Modifying LaserScan Message Definition(Or How to Access Turtlebot Files)
### by Harry Zhu (courtesy of TA Adam Ring) 

This guide showcases how you can modify the definitions (e.g. range_min) of the LaserScan message published to the /scan topic. It is also a brief “guide” on how to access and modify files on a Turtlebot. 

Turtlebot’s LaserScan message is specified by the ld08_driver.cpp file from this repo: https://github.com/ROBOTIS-GIT/ld08_driver/tree/develop/src. You can modify this file on your Turtlebot through a terminal on vnc. 

## Step 1: ssh into your robot, do not do bringup 
## Step 2: cd to the right directory
```cd catkin_ws/src/ld08_driver/src```
## Step 3: enter nano editor
```nano ld08_driver.cpp```
## Step 4: go to line 63 and fine 
```scan.range_min = 0.0;```
## Step 5: modify the value as you see fit
## Step 6: save and exit
ctrl + S to save, ctrl + X to exit the editor
## Step 7: remember to cm
```cm```
