# Spawn Object to Gazebo via Terminal ROS Service Call

By Frank Hu

Have Gazebo Simulation started, and open a different terminal tab, and enter the following command to spawn a model (.urdf)

```
rosrun gazebo_ros spawn_model -file `rospack find MYROBOT_description`/urdf/MYROBOT.urdf -urdf -x 0 -y 0 -z 1 -model MYROBOT
```

This command can also been used for `.sdf` files by replacing flag `-urdf` with `-sdf` 

For `.xacro` files, `.xacro` first need to be converted to `.xml` with:

```
rosrun xacro xacro `rospack find rrbot_description`/urdf/MYROBOT.xacro >> `rospack find rrbot_description`/urdf/MYROBOT.xml
```

then

```
rosrun gazebo_ros spawn_model -file `rospack find rrbot_description`/urdf/MYROBOT.xml -urdf -y 1 -model rrbot1 -robot_namespace rrbot1
```

For more information, enter

`rosrun gazebo_ros spawn_model -h`

