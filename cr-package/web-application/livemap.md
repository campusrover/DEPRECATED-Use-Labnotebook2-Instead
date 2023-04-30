# livemap.md

## Campus Rover Live Map

The objective is to implement a 2D map in the CR\_Web application that depicts:

* The floorplan Campus Rover is currently using to navigate
* Campus Rover's "real-time" location as it navigates
* The goal destination, toward which Campus Rover is navigating

### First Iteration

Our first implementation was based on a [tutorial](http://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap) that relied on a websocket connection between the robot and web client, and had the following dependencies on 3rd party libraries:

* [RosBridge](http://wiki.ros.org/rosbridge\_suite)
* [2Djs](https://www.npmjs.com/package/2djs)
* [RosLibJs](http://wiki.ros.org/roslibjs)
* [EaselJs](https://www.createjs.com/easeljs)
* [EventEmitter2](https://www.npmjs.com/package/eventemitter2)

This initial implementation ([repo here](https://github.com/campusrover/Campus-Rover-Web-Tools/tree/master/CR%20Live%20Map)) was successful, but presented several issues:

* Building upon 3rd party dependencies risked future breaks and maintenance.
* As discussed [here](../../Flask%20&%20ROS.md), it entailed "ROS-like" programming in _JavaScript_ instead of Python.
* The implementation described in the [tutorial](http://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap) generates a 2D map image from an amcl occupancy grid. This is unecessary for our purposes, because Campus Rover uses a _pre-generated_ floorplan image; re-generating it is redundant and thus computationally wasteful.
* Generating the map and loading the 4 JavaScript libraries mentioned above on every page load created noticeable performance issues, limiting any additional page content.

### Current Iteration

The current iteration resolves the issues identified through the first iteration and enables additional map features:

* Instead of generating a map image from an occupancy grid, an existing floorplan image file is rendered.
* Instead of using 3rd-party JavaScript libraries, the map is rendered using HTML5's Canvas element.
* Instead of writing "ROS-like" JavaScript in the front end as before, all ROS code is implemented with regular ROS Python programming in the Flask layer of the application.
* Unlike the initial iteration, the current map includes the option to "track" the robot as it traverses the map, automatically scrolling to keep up with the robot as it moves.
* The current iteration now displays the robot's goal location, too.

### Next Steps

Support for:

* Multiple floorplans/maps
* Switching between different floorplans
* Adjusting the size and scale of a map (for zooming in/out, resizing, etc.)

## Follow-up Iteration

> Brad Nesbitt 11/18/2018

### Overview of the LiveMap class

After several preceding iterations of "live" 2D maps, it became clear that a single abstraction for such mapping would be appropriate. An instance of the `LiveMap` class maps waypoints, the robot's current pose, and its goal poses onto 2D floorplan for display within a web application.

The `static` directory in `rover_app` now contains `map_files`, which contains the local files needed to generate a given map, including a JSON file with parameters specific to each map. For example:

\--

#### `all_maps.json`

```
"Gerstenzang Basement": {
    "files": {
        "path": "rover_app/static/map_files/basement/",
        "png_file": {
            "file_name": "basement.png",
            "cm_per_pixel": 1
        },
        "waypoint_file": "basement_waypoints.json"
    },
    "yaml_parameters": {
        "resolution":  0.01,
        "origin": [0.0, 0.0, 0.0]
    }
```

The JSON object for a map includes references to local files comprising the map's floorplan `.png` file, a JSON file of the map's waypoint data, and a copy of the yaml parameters used for amcl navigation of the `.png`-based map.

\--

#### `live_map.py`

Initializing a LiveMap object requires 2 parameters:

1. The name/String corresponding to a map in `all_maps.json`, such as "Gerstenzang Basement"
2. The desired centimeters per pixel ratio to be used when displaying the map.
3. An optional parameter is the centimeter diameter of the robot, which is the Turtlebot2's spec of 35.4 by default.

For example, `live_map = LiveMap("Gerstenzang Basement", 2)` initializes a LiveMap object of the Gerstenzang Basement floorplan with a 2cm/pixel scale. The object maintains the following abstraction representing the state of the map, including the robot's current place within it and it's goal destination:

```
    self.map_state = {
        "map_parameters": {
            "map_name": map_name_string,
            "files": {
                "path": path,
                "png": {
                    "file_name": map_json["files"]["png_file"]["file_name"],
                    "cm_per_pixel": map_json["files"]["png_file"]["cm_per_pixel"],
                    "pixel_width": png_height,
                    "pixel_height": png_width,
                },
                "yaml": map_json["yaml_parameters"]
            },
            "bot_radius": bot_cm_diameter/2,
            "cm_per_pixel": scale_cm_per_pixel, # Desired scale
            "waypoints": waypoints,
            "current_pose": {},
            "goal_pose": {}
        },
        "scaled_pixel_values": {
            "bot_radius": (bot_cm_diameter / 2) * png_cm_per_pixel / scale_cm_per_pixel,
            "cm_per_pixel": scale_cm_per_pixel,
            "png_pixel_width": png_width * png_cm_per_pixel / scale_cm_per_pixel,
            "png_pixel_height": png_height * png_cm_per_pixel / scale_cm_per_pixel,
            "current_pose": {},
            "goal_pose": {}
        },
        "subscribers": {
            "current_pose_sub": rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_current_pose),
            "goal_pose_sub": rospy.Subscriber('/move_base/current_goal', PoseStamped, self.update_goal_pose),
            "rviz_goal_pose_sub": rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal_pose)
        }
}
```

Note that a nested dictionary of ROS subscribers continually updates the scaled pixel value equivalents of the current and goal poses.

Implementing 2D mapping in this way aims to achieve two main advantages:

1. The LiveMap class allows the initialization of multiple, differing maps, with custom scales in the web application. For instance, a small, "thumbnail" map could be implemented on one page, while large map could be displayed somewhere else. This also makes switching between maps is also possible.
2. Representing a `map_state` as a Python dictionary (shown above) makes it easy to send the data needed to work with a live 2D map as JSON. For instance, a map route or endpoint could be implemented to return a `map_state` JSON object which could, in turn, be used to render or update a map in the UI.

## Brad Nesbitt & Huaigu Lin 11/10/2018
