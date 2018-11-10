###### Brad Nesbitt & Huaigu Lin 11/10/2018

---

# Campus Rover Live Map

The objective is to implement a 2D map in the CR_Web application that depicts:

* The floorplan Campus Rover is currently using to navigate
* Campus Rover's "real-time" location as it navigates
* The goal destination, toward which Campus Rover is navigating

---

### First Iteration

Our first implementation was based on a [tutorial](http://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap) that relied on a websocket connection between the robot and web client, and had the following dependencies on 3rd party libraries:

* [RosBridge](http://wiki.ros.org/rosbridge_suite)
* [2Djs](https://www.npmjs.com/package/2djs)
* [RosLibJs](http://wiki.ros.org/roslibjs)
* [EaselJs](https://www.createjs.com/easeljs)
* [EventEmitter2](https://www.npmjs.com/package/eventemitter2)

This initial implementation ([repo here](https://github.com/campusrover/Campus-Rover-Web-Tools/tree/master/CR%20Live%20Map)) was successful, but presented several issues: 

* Building upon 3rd party dependencies risked future breaks and maintenance.

* As discussed [here](https://github.com/campusrover/labnotebook/blob/master/Flask%20%26%20ROS.md), it entailed "ROS-like" programming in _JavaScript_  instead of Python.

* The implementation described in the [tutorial](http://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap) generates a 2D map image from an amcl occupancy grid. This is unecessary for our purposes, because Campus Rover uses a _pre-generated_ floorplan image; re-generating it is redundant and thus computationally wasteful.

* Generating the map and loading the 4 JavaScript libraries mentioned above on every page load created noticeable performance issues, limiting any additional page content.

---

### Current Iteration

The current iteration resolves the issues identified through the first iteration and enables additional map features:

* Instead of generating a map image from an occupancy grid, an existing floorplan image file is rendered.

* Instead of using 3rd-party JavaScript libraries, the map is rendered using HTML5's Canvas element.

* Instead of writing "ROS-like" JavaScript in the front end as before, all ROS code is implemented with regular ROS Python programming in the Flask layer of the application.

* Unlike the initial iteration, the current map includes the option to "track" the robot as it traverses the map, automatically scrolling to keep up with the robot as it moves.

* The current iteration now displays the robot's goal location, too.


---

### Next Steps

Support for:

* Switching between different floorplans
* Adjusting the size and scale of a map (for zooming in/out, resizing, etc.)
