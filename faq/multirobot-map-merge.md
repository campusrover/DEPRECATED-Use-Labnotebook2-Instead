### Multi-robot Map Merge

This FAQ section assumes understanding of creating TF listeners/broadcasters, using the TF tree, namespacing, and launching of multiple robots. This tutorial also relies on the ROS [gmapping](http://wiki.ros.org/gmapping) package although different SLAM methods can be substituted.

### Setting up launch files for gmapping

Gmapping will require namespaces for each robot.
If you want to use the launch file `turtlebot3_gmapping.launch` from the package `turtlebot3_slam`, be sure to pass values to the arguments `set_base_frame`, `set_odom_frame`, and `set_map_frame` to use the robot namespaces. For instance, if the default value is `base_footprint`, the namespaced value will be `$(arg ns)/base_footprint`.
You can also directly call the node for slam_gmapping inside the namespace (below it is `$(aarg ns)`) with

``` xml
<node pkg="gmapping" type="slam_gmapping" name="turtlebot3_slam_gmapping" output="log">
    <param name="base_frame" value="$(arg ns)/base_footprint"/>
    <param name="odom_frame" value="$(arg ns)/odom"/>
    <param name="map_frame"  value="$(arg ns)/map"/>
</node>
```

### Using multirobot_map_merge package

1. First navigate to the [ros documentation for multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge). Install depending on the version (newer ROS versions may need to clone directly from the respective branches of m-explore).
1. `catkin_make`
1. If not using initial poses of robots, simply use `map_merge.launch` as is. In the initial file, `map_merge` has a namespace but by removing the namespace the merged map will be directl published to `/map`.
1. If using initial poses of robots, you must add parameters within the robot namespace named `map_merge/init_pose_x`, `map_merge/init_pose_y`, `map_merge/init_pose_z`, and `map_merge/init_pose_yaw`.

#### Using merged map for cost map

The current map merge package does not publish a tf for the map. As such, one must create a TF frame for the map and connect it to the existing tree, making sure that the base_footprints of each robot can be reached from the map, before using things such as move_base.


### Saving multiple/namespaced maps

Maps can be saved directly through the command line, but multiple maps can also be saved at once by creating a launch file that runs the `map_saver` node. If running the `map_saver` node for multiple maps in one launch, each node will also have to be namespaced.

When using `map_saver`, be sure to set the map parameter to the dynamic map of the robot (will look like `namespace/dynamic_map`, where `namespace` is the name of the robot’s namespace), instead of just the at `namespace/map` topic.

Example:

``` xml
<node name="map_saver" pkg="map_server" type="map_saver" 
    args="-f $(find swarmbots)/maps/robot$(arg newrobots)" output="screen">
        <param name="map" value="/robot$(arg newrobots)/dynamic_map"/>
    </node>
```

will set the map to be saved as the one specified in the parameter, meanwhile the argument `-f $(find swarmbots)/maps/robot$(arg newrobots)` will set the save destination for the map files as the `maps` folder, and the names of the files as `robot$(arg newrobots).pgm` and `robot$(arg newrobots).yaml`.

To save a map merged by the map_merge package, do not namespace the map_merge launch file and instead have it publish directly to `/map`, and map saver can be used on `/map`.

### References

- [Multirobot map merge package](http://wiki.ros.org/multirobot_map_merge)