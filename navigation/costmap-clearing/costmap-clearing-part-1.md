# Costmap Clearing Part 1

Our objective of this iteration is to find a way to clean Turtlebot's costmap, so any long gone obstacle will not stay on the map, but the Turtlebot should also not run into a transient obstacle like a person or a chair.

## Packages & Enviroments

We are using `roslaunch cr_ros campus_rover.launch` command to bring up Turtlebot. This launch file launches amcl with a configuration similar to `amcl_demo.launch` file in `turtlebot_navigation` package. Then we run rviz to visualize the static floor plan map and costmap.

## Known Issues Before This Iteration

In previous demos, we found that Turtlebot would successfully mark transient obstacles, such as a person passing by, on its costmap and avoid them. But it failed to unmark them even after they are gone. These marks of long gone obstacles would cause the path planner to avoid them. Eventually Turtlebot would stuck because it cannot find a valid path to its goal.

## Research and Findings

We found a possible pattern and cause for this problem. In [**this post thread**](http://ros-users.122217.n3.nabble.com/Clear-cells-in-costmap-with-max-laser-range-td973150.html), someone mentions that:

> "Costmap2D seems not to "clear" the space around the robot if the laser scan range is max."

We tested this claim. Indeed, when an obstacle in front of Turtlebot is out of max range of its camera sensor, someone who pass through the empty space between the obstacle and Turtlebot's camera would be marked permanently on the costmap. However, if an obstacle is within max range of camera sensor, someone pass through will be marked and then unmarked immediately once this person is gone.

The above post thread also mentions an explanation for this behavior:

> "Its actually not the costmap that is ignoring max\_scan\_range values from the laser, its the laser projector that takes a laser scan and turns it into a point cloud. The reason for this is that there is no guarantee that max\_scan\_range actually corresponds to the laser not seeing anything. It could be due to min\_range, a dark object, or another error condition... all the laser knows is that it didn't get a return for one reason or another. "

Based on our experiment and this explanation, a possible solution for the max\_range problem could be setting up a [**scan filter chain**](http://wiki.ros.org/laser_filters). Theoretically when a scan value is "max\_range", we could replace it with a big number such as 100 \(Turtlebot's scan range is 10 meters\). However we could not make it work this week, so we will do more experiments in the coming week.

## Our Implementation -- Recovery Behavior

The `campus_rover.launch` file includes another launch file `move_base.launch.xml` from `turtlebot_navigation` package. In `move_base.launch.xml`, a `move_base` node is launched with a bunch of parameters stored in yaml files. This node basically runs navigation stack for Turtlebot and also includes all the costmap drawing/clearing behaviors.

What we found out was that in `turtlebot_navigation/param/move_base_params.yaml`, all the parameters for recovery behaviors were commented out. According to [**ros documentation on expected robot behaviors**](http://wiki.ros.org/move_base#Expected_Robot_Behavior), recovery behaviors are an essential part of robot's navigation. When the robot perceive itself as stuck \(unable to find a valid path to its goal\), it should perform recovery behaviors to clear its costmap and then replan a path.

Therefore, we brought back an recovery behavior with the code:

```text
recovery_behaviors:
- name: 'aggressive_reset2'
  type: 'clear_costmap_recovery/ClearCostmapRecovery'

aggressive_reset2:
reset_distance: 0.0
```

`reset_distance: 0.0` means Turtlebot clears its costmap outside of 0.0 radius, so it will clear all the costmaps when it perceive itself as stuck. Based on our experiments and previous experience, Turtlebot was pretty good at dodging obstacles that were not on its costmap, so this "aggressive" reset is safe for most occasions, unless Turtlebot is physically surrounded by obstacles that are very close to it, but in this extreme circumstance, "conservative" costmap clearing would also be useless because clearing costmaps several meters away would not be enough to unstuck it.

We also specified the costmap layer name since `obstacle layer` is the one contains all the marks of dynamic obstacles:

```text
layer_names: ["obstacle_layer"]
```

These changes would ensure the clearing of costmap when Turtlebot perceive itself as stuck, and it will no longer get stuck by marks of long-gone obstacles.

## Next Step

We will look more into implementing the scan filter, so Turtlebot would immediately unmark a gone obstacle even if the scan is out of max range.

### _Huaigu Lin & Jacky Chen 11/11/2018_

