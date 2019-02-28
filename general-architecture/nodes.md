# Campusrover Node List


## [adjust_position](https://github.com/campusrover/cr_ros/blob/master/src/adjust_position.py)

`Dormant` Converts the Pose messages it receives from its subscription to PoseWithCovarianceStamped messages and passes them on via its publication

**Publications**
- /initialpose

**Subscriptions**
- /fid_pose


## [check_docked](https://github.com/campusrover/cr_ros/blob/master/src/check_docked.py)

`Current` Updates the robot's state to reflect whether it is currently being charged at its dock based on charging data from its subscription

**Subscriptions**
- /mobile_base/sensors/core_throttle


## [cpu_checker](https://github.com/campusrover/cr_ros/blob/master/src/cpu_checker.py)

`Current` Publishes CPU usage data and prints it to the warning log if it is high or otherwise to the debug log based on data from process and system utilities

**Publications**
- /laptop_cpu_usage


## [greeter](https://github.com/campusrover/cr_ros/blob/master/src/greet.py)

`Current` Uses facial recognition to detect and recognize known faces in the camera feed based on provided data and greets them appropriately by name via a vocal service

**Subscriptions**
- /camera/rgb/image_raw/compressed_throttle


## [lost_and_found](https://github.com/campusrover/cr_ros/blob/master/src/lost_and_found.py)

`Current` Detects when the robot is flying based on whether its wheels are in contact with the ground, lost upon regaining contact with the ground, and navigating upon recognizing its location and adjusts its state and goals accordingly and publishes or logs reverent information

**Publications**
- /initialpose
- cmd_vel_mux/input/teleop
- /destination

**Subscriptions**
- /mobile_base/events/wheel_drop
- /destination


## [message_switch](https://github.com/campusrover/cr_ros/blob/master/src/message_switch.py)

`Current` Organizes speech messages chronologically and feeds them to the speech service at appropriate times

**Subscriptions**
- /things_to_say


## [location_narration](https://github.com/campusrover/cr_ros/blob/master/src/narrate_location.py)

`Dormant` Publishes speech messages narrating the robot's behavior current and proximate location based on its state and on data from its subscription

**Publications**
- /things_to_say

**Subscriptions**
- /nearest_waypoint


## [navigation_controller](https://github.com/campusrover/cr_ros/blob/master/src/navigation_controller.py)

`Defunct` All functionality was moved to [rover_controller](https://github.com/campusrover/cr_ros/blob/master/src/rover_controller.py)

**Publications**
- /cmd_vel_mux/input/navi

**Subscriptions**
- /amcl_pose


## [package_handler](https://github.com/campusrover/cr_ros/blob/master/src/package_handler.py)

`Current` Detects the presence of a physical package via its publications and converses with a user to determine goals and to communicate successes and errors while updating its goals to respond to expected and unexpected changes

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


## [package_sender](https://github.com/campusrover/cr_ros/blob/master/src/package_sender.py)

`Current` Publishes filename of appropriate prerecorded message for the robot to play based on data from its subscription

**Publications**
- /receive_package

**Subscriptions**
- /physical_package


## [pose_converter](https://github.com/campusrover/cr_ros/blob/master/src/pose_converter.py)

`Dormant` Provides scripts for automatically converting from different pose types


## [process_fid_tfs](https://github.com/campusrover/cr_ros/blob/master/src/process_fid_transforms.py)

`Current` Uses fiducial data from its subscription to to determine and publish the robot's position relative to the map

**Publications**
- initialpose

**Subscriptions**
- fiducial_transforms


## [recording_sender](https://github.com/campusrover/cr_ros/blob/master/src/recording_sender.py)

`Current` Records short audio clips featuring user instructions to a file and publishes its name

**Publications**
- /receive_package

**Subscriptions**
- /record_start
- /record_stop


## [rover_controller](https://github.com/campusrover/cr_ros/blob/master/src/rover_controller.py)

`Dormant` Controls the robot and its state with respect to a wide range of input sources and publishes a wide range of data for other nodes to use

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


## [scan_filter](https://github.com/campusrover/cr_ros/blob/master/src/scan_filter.py)

`Current` Replaces the 'nan' value in the /scan topic with 9.90 and publishes to /scan_filtered

**Publications**
- /scan_filtered

**Subscriptions**
- scan


## [state_manager](https://github.com/campusrover/cr_ros/blob/master/src/state.py)

`Current` Handles and validates requested state changes for legality and publishes relevant information accordingly

**Publications**
- /move_base_simple/goal
- /initialpose
- /goal_pose_for_fids
- /state


## [talk](https://github.com/campusrover/cr_ros/blob/master/src/talk.py)

`Current` Plays sound files of what the robot is supposed to say


## [turtlebot_teleop](https://github.com/campusrover/cr_ros/blob/master/src/turtlebot_teleop_key.py)

`Current` Cancels existing robot goals and allows for manual control of the robot via teleoperation

**Publications**
- /cmd_vel_mux/input/teleop

**Subscriptions**
- /web/teleop
- initialpose


## [whereabouts](https://github.com/campusrover/cr_ros/blob/master/src/whereabouts.py)

`Dormant` Publishes the name of the nearest waypoint when it changes based on data from its subscription

**Publications**
- /nearest_waypoint

**Subscriptions**
- /amcl_pose
