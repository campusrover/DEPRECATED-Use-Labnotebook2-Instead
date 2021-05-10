# TF in Gazebo

MinJun Song

## Introduction

When we manipulate a robot in the gazebo world, there are times when the odometry of the robot does not correspond correctly with the actual position of the robot.  This might be due to the robot running into objects which leads to mismatch between the rotation of the wheels and the calculation for the location derived from it.  So it will be **useful to get the absolute position of the robot that is given by the gazebo environment.**

## Getting Started

For the robot to navigate autonomously, it has to know its **exact location**. To manifest this process of localization, the simplest method is to receive values from the wheel encoder that calculates the odometry of the robot from the starting position.

In terms of mobile robots like turtlebot that we use for most of the projects, localization is impossible when the robot collides with other objects such as walls, obstacles, or other mobile robots.

So in this case, we can use the `p3d_base_controller` plugin that gazebo provides to receive the correct odometry data of the robot to localize in the environment.


## Code

What I needed was relative positions of the robots.  Originally we need to get the odometry data of the two robots and use rotation, translation matrix to find the relative coordinate values and yaw by ourselves.

In `gazebo.xacro` file paste the following commands:

``` xml
<gazebo>
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>50.0</updateRate>
    <bodyName>base_footprint</bodyName>
    <topicName>base_footprint_odom</topicName>
    <gaussianNoise>0.01</gaussianNoise>
    <frameName>world</frameName>
    <xyzOffsets>0 0 0</xyzOffsets>
    <rpyOffsets>0 0 0</rpyOffsets>
    </plugin>
</gazebo>
```

Explaination of the Code

- bodyName: the name of the body that we want the location of
- topicName: the name of odometry topic

With the code above, we can represent two robot’s location.  To find robot 1’s location relative to robot 2’s location, we can use the following code:

``` python
try:
    robot_trans = tfBuffer.lookup_transform(‘robot1’, ‘robot2’, rospy.Time())
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    continue
```

