# Teleport Model within Gazebo Simulation

by Frank Hu

If you want to move existing model within Gazebo simulation, you can do so via `rosservice call`.

Open a new terminal tab, and enter

```
rosservice call /gazebo/set_model_state '{model_state: { model_name: MODEL+NAME, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
```

Special Note:

Some spaces in the above command CANNOT be deleted, if there is an error when using the above command, check you command syntax first.

E.g: Right: `{ x: 0, y: 0 ,z: 0 }` Wrong:  `{ x:0, y:0 ,z:0 }` (missing space after `:`)

You can also experiment with this command to set robot arm's state, to make it into a certain orientation like this rrbot

![rrbot_default](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/ros_comm/Coke_Can.png)

`rosservice call /gazebo/set_model_state '{model_state: { model_name: rrbot, pose: { position: { x: 1, y: 1 ,z: 10 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'`

![rrbot_swing](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/ros_comm/Coke_Can_Flying.png)
