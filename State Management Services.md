# *State Management Services*

## Overview

This week, we built an enum, `all_states.py` to represent the varying states of the robot, and two ROS services: _StateChange_ manages state changes, as well as any associated publishes, and _StateQuery_ reports the current state of the robot. Both services are hosted in the `state.py` node.

While the robot's states aren't too complicated yet, our goal was to create a modular architecture to make it easy to add new states as the robot's functionality expands.

### StateQuery

`StateQuery` is a service that takes no arguments, and returns the state of the robot as the string value of the appropriate enum.

### all_states.py

`all_states.py` serves two purposes: It defines a series of enums for the various states of the robot, and it defines methods to act as the service clients. In any node that needs to request a state change or get the current state of the robot, add `from all_states import *` to gain access to the enums (type `State`) and the two methods.

- `get_state` acts as a client method for the `StateQuery` service, taking no parameters and returning the string value of the robot's state.
- `change_state` acts as a client method for the `StateChange` service, taking up to three parameters and returning a boolean value of whether or not the state change was legal:
	- `new_state`: The string value of the desired new state
	- `to_say`: A message to be said by `talk.py` upon the state change
	- `pose_to_pub`: A `Pose` to be published, if the new state is either `NAVIGATING` or `LOCALIZING`

### state.py

`state.py` contains the code needed to keep track of the robot's current state and facilitate state changes. It includes the `is_legal` method, which contains a dict mapping every state to an array of the states that could legally follow it.

If an illegal state change is requested, the current state of the robot is set to `States.ILLEGAL_STATE_CHANGE`.

---
###### _Ari Carr and Ben Albert 11/14/2018_

