# Nodes

Updated May 2019 with progress following gen3 and mutant mark 1.

## [adjust\_position](https://github.com/campusrover/cr_ros/blob/master/src/adjust_position.py)

`Dormant` Converts the Pose messages it receives from its subscription to PoseWithCovarianceStamped messages and passes them on via its publication

**Publications**

* /initialpose

**Subscriptions**

* /fid\_pose

## [check\_docked](https://github.com/campusrover/cr_ros/blob/master/src/check_docked.py)

`Defunct` Updates the robot's state to reflect whether it is currently being charged at its dock based on charging data from its subscription

Now defunct - mutant does not dock, because it is not based on the kobuki base.

**Subscriptions**

* /mobile\_base/sensors/core\_throttle

## [cpu\_checker](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/cpu_checker.py)

`Current` Publishes CPU usage data and prints it to the warning log if it is high or otherwise to the debug log based on data from process and system utilities

**Publications**

* /laptop\_cpu\_usage

## [greeter](https://github.com/campusrover/cr_ros/blob/master/src/greet.py)

`Dormant` Uses facial recognition to detect and recognize known faces in the camera feed based on provided data and greets them appropriately by name via a vocal service

**Subscriptions**

* /camera/rgb/image\_raw/compressed\_throttle

## [lost\_and\_found](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/lost_and_found.py)

`Current` Uses pickup detector data to determine whether the robot is flying or not. Handles localization recovery upon returning to the ground.

**Publications**

* /initialpose
* /cmd\_vel
* /destination

**Subscriptions**

* /airborne
* /destination

## [message\_switch](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/message_switch.py)

`Dormant` Organizes speech messages chronologically and feeds them to the speech service at appropriate times

**Subscriptions**

* /things\_to\_say

## [location\_narration](https://github.com/campusrover/cr_ros/blob/master/src/narrate_location.py)

`Dormant` Publishes speech messages narrating the robot's behavior current and proximate location based on its state and on data from its subscription

**Publications**

* /things\_to\_say

**Subscriptions**

* /nearest\_waypoint

## [navigation\_controller](https://github.com/campusrover/cr_ros/blob/master/src/navigation_controller.py)

`Defunct` All functionality was moved to [rover\_controller](https://github.com/campusrover/cr_ros/blob/master/src/rover_controller.py)

**Publications**

* /cmd\_vel\_mux/input/navi

**Subscriptions**

* /amcl\_pose

## [package\_handler](https://github.com/campusrover/cr_ros/blob/master/src/package_handler.py)

`Dormant` Detects the presence of a physical package via its publications and converses with a user to determine goals and to communicate successes and errors while updating its goals to respond to expected and unexpected changes.

Currently not in use due to the lack of a sensor to detect packages on gen3's mutant.

**Publications**

* /release\_package
* /record\_start
* /record\_stop
* /physical\_package
* /destination

**Subscriptions**

* /release\_package
* /receive\_package
* /mobile\_base/events/button
* /mobile\_base/events/digital\_input
* /destination

## [package\_sender](https://github.com/campusrover/cr_ros/blob/master/src/package_sender.py)

`Dormant` Publishes filename of appropriate prerecorded message for the robot to play based on data from its subscription

Dormant for same reason as package\_handler

**Publications**

* /receive\_package

**Subscriptions**

* /physical\_package

## [pose\_converter](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/pose_converter.py)

`Current` Provides scripts for automatically converting from different pose types

## [process\_fid\_tfs](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/process_fid_transforms.py)

`Current` Uses fiducial data from its subscription to to determine and publish the robot's position relative to the map

**Publications**

* initialpose

**Subscriptions**

* fiducial\_transforms

## [recording\_sender](https://github.com/campusrover/cr_ros/blob/master/src/recording_sender.py)

`Current` Records short audio clips featuring user instructions to a file and publishes its name

**Publications**

* /receive\_package

**Subscriptions**

* /record\_start
* /record\_stop

## [rover\_controller](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/rover_controller.py)

`Current` Controls the robot and its state with respect to a wide range of input sources and publishes a wide range of data for other nodes to use

**Publications**

* temp\_pose
* /teleop\_keypress
* /destination
* /web/camera
* /web/state
* /web/map
* /cmd\_vel

**Subscriptions**

* /raspicam\_node/image/compressed
* /web/teleop
* /web/destination
* /destination

## [scan\_filter](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/scan_filter.py)

`Current` applies a filter to scan data to ignore the structural posts of the mutant

**Publications**

* /scan\_filter

**Subscriptions**

* scan

## [state](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/state.py)

`Current` Handles and validates requested state changes for legality and publishes relevant information accordingly

**Publications**

* /move\_base\_simple/goal
* /initialpose
* /goal\_pose\_for\_fids
* /state

## [talk](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/talk.py)

`Current` Uses text to speech to turn strings into audio output

## [turtlebot\_teleop](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/turtlebot_teleop_key.py)

`Current` Cancels existing robot goals and allows for manual control of the robot via teleoperation

**Publications**

* /cmd\_vel\_mux/input/teleop

**Subscriptions**

* /web/teleop
* initialpose

## [whereabouts](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/whereabouts.py)

`Dormant` Publishes the name of the nearest waypoint when it changes based on data from its subscription

**Publications**

* /nearest\_waypoint

**Subscriptions**

* /amcl\_pose

## [detect\_pickup](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/detect_pickup.py)

`Current` Uses IMU accelerometer data to decide whether the robot has been lifted, and when it has been placed on the ground.

**Publications**

* /airborne

**Subcriptions**

* /imu

## [voice\_destination\_pub](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/voice_destination_pub.py)

`Current` takes information from the alexa webhook, and if it involves going to a destination, publishes the goal pose of the specified destination.

**Publications**

* /destination

**Subscriptions**

* /voice\_intents

## [hand\_gesture](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/hand_gesture.py)

`Current` _only slightly usable in demo_ pauses navigation for ten seconds if it receives signal that a hand is in view of the camera.

**Publications**

* /destination

**Subscriptions**

* /destination
* /hand\_command

## [go\_to\_person](https://github.com/campusrover/cr_ros_2/blob/mutant_transfer/src/go_to_person.py)

`Current` _only slightly usable in demo_ spins, searching for recognized person, then stops.

**Publications**

* /destination
* /cmd\_vel

**Subscriptions**

* /odom
* /face\_detection
* /has\_package

