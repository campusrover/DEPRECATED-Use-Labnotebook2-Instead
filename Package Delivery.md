# Package Delivery
## Related Nodes
* `package_handler.py`
* `package_sender.py`
* `recording_sender.py`

## Overview
The package delivery stack is comprised of one main node, `package_handler.py`, and two secondary nodes `package_sender.py` and `recording_sender.py`.  Each secondary node corresponds to a type of package that can be handled by the handler (the current implementation supports physical packages on top of the robot, and voice recordings recorded on the on-board laptop.  Each secondary node (when pinged on its respective topic) generates a file with a specific suffix (`.package` and `.wav` respectively) that will be processed accordingly by the package handler.  Each file type has an associated package release protocol, which removes the package filename from the `packages` queue (maintained by the handler node).

## Demo
* Run `recording_sender.py` on-board (so computer microphone is accessed)
* Run `package_handler.py` and `package_sender.py`
* After navigating to the package sender (wait for WAITING state) place package on the robot
* After being prompted, hold down B0, B1, or B2 on the robot's base to record a message
* Send the robot a delivery navigation goal
* Once the robot has reached the goal (wait for WAITING state) pick up the package.  The robot will also play back the audio recording
* After the 

###### _Ben Albert 12/16/18_
