# Campusrover Node List


## [adjust_position](https://github.com/campusrover/cr_ros/blob/master/src/adjust_position.py)

> Passes on pose information between topics

**Publications**
- /initialpose

**Subscriptions**
- /fid_pose


## [check_docked](https://github.com/campusrover/cr_ros/blob/master/src/check_docked.py)*

> Updates the robot's state to reflect whether it is currently being charged at its dock

**Subscriptions**
- /mobile_base/sensors/core_throttle


## [cpu_checker](https://github.com/campusrover/cr_ros/blob/master/src/cpu_checker.py)*

> Publishes the robot's CPU usage.

**Publications**
- /laptop_cpu_usage


## [greeter](https://github.com/campusrover/cr_ros/blob/master/src/greet.py)*

> Recognizes known faces from the camera and greets them by their first name

**Subscriptions**
- /camera/rgb/image_raw/compressed_throttle


## [lost_and_found](https://github.com/campusrover/cr_ros/blob/master/src/lost_and_found.py)*

> Detects when the robot is kidnapped, lost, and found again and adjusts its state accordingly

**Publications**
- /initialpose
- cmd_vel_mux/input/teleop
- /destination

**Subscriptions**
- /mobile_base/events/wheel_drop
- /destination


## [message_switch](https://github.com/campusrover/cr_ros/blob/master/src/message_switch.py)*

> Receives messages of what to say and has them said at the appropriate time

**Subscriptions**
- /things_to_say


## [location_narration](https://github.com/campusrover/cr_ros/blob/master/src/narrate_location.py)

> Publishes messages for the robot to say pertaining to where it is and what it is doing

**Publications**
- /things_to_say

**Subscriptions**
- /nearest_waypoint


## [navigation_controller](https://github.com/campusrover/cr_ros/blob/master/src/navigation_controller.py)

> `Defunct` All functionality was moved to [rover_controller](https://github.com/campusrover/cr_ros/blob/master/src/rover_controller.py)*

**Publications**
- /cmd_vel_mux/input/navi

**Subscriptions**
- /amcl_pose


## [package_handler](https://github.com/campusrover/cr_ros/blob/master/src/package_handler.py)*

> Adjusts robot's state and manages verbal responses pertaining to the robot's package delivery functionality

**Publications**
- /release_package
- /record_start
- /record_stop
- /physical_package
- /destination

**Subscriptions**
- /release_package
- /receive_package
- /mobile_base/events/button
- /mobile_base/events/digital_input
- /destination


## [package_sender](https://github.com/campusrover/cr_ros/blob/master/src/package_sender.py)*

> Stores information about physical packages the robot receives

**Publications**
- /receive_package

**Subscriptions**
- /physical_package


## [pose_converter](https://github.com/campusrover/cr_ros/blob/master/src/pose_converter.py)

> Provides scripts for automatically converting from different pose types


## [process_fid_tfs](https://github.com/campusrover/cr_ros/blob/master/src/process_fid_transforms.py)*

> Uses fiducials to determine the robot's position

**Publications**
- initialpose

**Subscriptions**
- fiducial_transforms


## [recording_sender](https://github.com/campusrover/cr_ros/blob/master/src/recording_sender.py)*

> Records and publishes audio instructions the robot receives a package

**Publications**
- /receive_package

**Subscriptions**
- /record_start
- /record_stop


## [rover_controller](https://github.com/campusrover/cr_ros/blob/master/src/rover_controller.py)

> Controls the robot and adjusts its state and verbal messages in response to its status and goals

**Publications**
- temp_pose
- /teleop_keypress
- /destination
- /web/camera
- /web/state
- /web/map
- /cmd_vel_mux/input/navi

**Subscriptions**
- /camera/rgb/image_rect_color/compressed
- /web/teleop
- /web/destination
- /destination


## [scan_filter](https://github.com/campusrover/cr_ros/blob/master/src/scan_filter.py)*

> Adjusts scan messages to substitute out nan values

**Publications**
- /scan_filtered

**Subscriptions**
- scan


## [state_manager](https://github.com/campusrover/cr_ros/blob/master/src/state.py)*

> Handles robot state changes

**Publications**
- /move_base_simple/goal
- /initialpose
- /goal_pose_for_fids
- /state


## [talk](https://github.com/campusrover/cr_ros/blob/master/src/talk.py)*

> Plays sound files of what the robot is supposed to say


## [turtlebot_teleop](https://github.com/campusrover/cr_ros/blob/master/src/turtlebot_teleop_key.py)*

> Cancels existing robot goals and allows for manual control of the robot

**Publications**
- /cmd_vel_mux/input/teleop

**Subscriptions**
- /web/teleop
- initialpose


## [whereabouts](https://github.com/campusrover/cr_ros/blob/master/src/whereabouts.py)

> Publishes the name of the nearest waypoint when it changes

**Publications**
- /nearest_waypoint

**Subscriptions**
- /amcl_pose


---
\* Included in launch
