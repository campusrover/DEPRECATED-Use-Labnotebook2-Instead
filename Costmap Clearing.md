###### Huaigu Lin & Jacky Chen 11/11/2018

---

# Amcl Costmap Clearing

Our objective of this iteration is to find a way to clean Turtlebot's costmap, so any long gone obstacle will not stay on the map, but the Turtlebot should also not run into a transient obstacle like a person or a chair.

---

### Packages & Enviroments

We are using 'roslaunch cr_ros bringup.launch' command to bring up Turtlebot. This launch file launches amcl with a configuration similar to 'amcl_demo.launch' file in 'turtlebot_navigation' package. Then we run rviz to visualize the static floor plan map and costmap.

---

### Known Issues Before This Iteration

In previous demos, we found that Turtlebot would successfully mark transient obstacles, such as a person passing by, on its costmap and avoid them. But it failed to unmark them even after they are gone. These marks of long gone obstacles would cause the path planner to avoid them. Sometimes Turtlebot would stuck because it cannot find a valid path to its goal.

---

### Research and Findings

We found a possible pattern and cause for this problem. In [**this post thread**](http://ros-users.122217.n3.nabble.com/Clear-cells-in-costmap-with-max-laser-range-td973150.html), someone mentions that:

> "Costmap2D seems not to "clear" the space around the robot if the laser scan range is max."

We tested this claim. Indeed, when an obstacle in front of Turtlebot is out of max range of its camera sensor, someone who pass through the empty space between the obstacle and Turtlebot's camera would be marked permanently on the costmap. However, if an obstacle is within max range of camera sensor, someone pass through will be marked and then unmarked immediately once this person is gone.

The above post thread also mentions an explanation for this behavior:

> "Its actually not the costmap that is ignoring max_scan_range values from the laser, its the laser projector that takes a laser scan and turns it into a point cloud. The reason for this is that there is no guarantee that max_scan_range actually corresponds to the laser not seeing anything. It could be due to min_range, a dark object, or another error condition... all the laser knows is that it didn't get a return for one reason or another. "

Based on this explanation, a possible solution could be:

> "Write a filter for the laser scan which takes all the max_range values and sets them to something slightly less than max_range. Then, you can configure the costmap to use that filtered scan for clearing operations."

We decided not to use this because laser readings represent distance from obstacles. This implementation would make costmap mark a line of nonexistent obstacles at the sensor's max range and affect path planning.

---

###
