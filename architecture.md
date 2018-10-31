@Alexander Feldman, feldmanay@gmail.com

# Campus Rover Mark 1 Design

[Architecture diagram here](https://docs.google.com/drawings/d/1K8Bq4vd7oYqD6yXStrwqmTUtPrRfLSoVaKmgR9WnArc/edit?usp=sharing)

The most complex software controlling our rover is the navigation algorithm we would use. It must efficiently process sensor data and plan routes over long range maps and short distance obstacles. A full-fledged campus rover must also handle complex obstacles like doors, elevators, and road crossings which would each require special navigation decision making. A fully functioning rover would also incorporate a unique localization algorithm to combine sensor data from fiducials, GPS, wifi signals, and camera/lidar inputs. 

Currently, reliability is a more pressing concern in delivering a rover to meet the mark 1 requirements. While a solution which provides more control in navigation would allow us to better expand our solution beyond Volen 1, it is not feasible at this stage. The turtlebot navigation package has been well tested and is proven to carefully plan routes and avoid obstacles. It also incorporates a powerful adaptive Monte Carlo localization algorithm which draws on data points from a single-beam lidar or depth camera.

A better solution for the mark 1 rover is to make use of the robust turtlebot navigation code and supplement the AMCL localization with a separate fiducial localization system which will supply poses to the existing algorithm by publishing pose estimates. A navigation_controller will also wrap the standard navigation and localization nodes to provide additional control and feedback where possible.

It is also beneficial to wrap the cmd_vel outputs of the navigation code in a motor_control node which will allow for distinct wandering to refine an initial pose and for teleop commands. This will allow teleop commands to block navigation directions which in turn will block localization commands.

At the top level, a central control node will manage core tasks like interacting with the tablet controls, publishing a map stored on disk, and deciding when the robot must return to recharge. Keeping these simple high level tasks combined will allow for rapid development but could be spun off into independent ROS nodes if the functionality required grows in complexity.

fiducial_localize also publishes acts primarily as a transform from the camera-relative pose from the built-in aruco_detect node, to a map relative pose based on it’s knowledge of the locations of fiducials in the map. The node is always running, but only when it sees a fiducial will it publish a cur_pose message to assist the AMCL localization.

#### Central control:
Publishes: 
cmd_vel/teleop + drive_state because it communicates with the tablet
robot_state because it unifies information from the navigation controller and the hardware level controllers
Calls:
navigation_controller as an action to execute a nav goal. It passes the coordinates, and navigation_controller sends feedback on status of navigation.

#### Navigation controller:
Publishes:
drive_state to distinguish between localization movement and navigation movement.
goal_pose to allow navigation algorithm to find path to goal
Calls:
get_initial_pose to drive while determining pose with little information
Feedback:
Compares current pose to goal pose to determine when navigation is complete. Also can return feedback regularly about distance traveled over time, expected delivery time, confidence in pose, etc.

