# *lost\_and\_found.py*

## Overview

This week, we built a node to handle the problem of fiducial detection - namely, the question of what the robot does when it doesn't know where it is. This would happen in two cases:

* **Bringup:** When the robot is initially started up, it won't know where it is unless it happens to have a fiducial in view already.
* **The "kidnapped robot problem":** When someone picks up the robot and moves it, its localization won't recognize the new position, so the robot needs to identify that it's been moved.

In both of these cases, the robot must act on its state of being lost by semi-intelligently wandering around the area and avoiding obstacles until it sees a fiducial and can re-localize.

## New States

To solve this problem, we first added two new states to `state.py`: **FLYING** and **LOST**. These states will be used to identify when the robot is being picked up and when the robot doesn't know where it is, respectively.

## The `lost_and_found` node

### State Changes

**Becoming lost:**

The node includes a subscriber to the `/mobile_base/events/wheel_drop` topic, which publishes a message every time the robot's wheels move up or down. As the wheels' natural state is to be pushed up as the body of the TurtleBot rests on top of them, a message that the wheels have moved down indicates that the robot has been picked up, triggering a change into the FLYING state. Similarly, once the robot is flying, a message that the wheels are up again indicates that the robot has been set down, such that the robot is now LOST and able to start looking for fiducials.

**Becoming found:**

The while loop of our wandering is controlled by an if statement designed to catch `state != States.LOST`. Ideally, the fiducial detection will trigger the localization of the TurtleBot, which will change the state of the 
TurtleBot to LOCALIZING. Once the state changes, the while loop will break and the node will stop making the TurtleBot wander until LOST is detected again. 

### The wander algorithm

We ensure maximum camera coverage, for the best odds of finding a fiducial, by having the robot drive in a rectangular outward spiral away from where it had been:

* The robot starts by spinning 360º, then driving a set distance to its right
* The robot then spins 360º, turns 90º to the left, and drives that same distance
* Until a fiducial is found:
	* The robot spins 360º
	* The robot turns left and drives for a slightly further distance than last time
	* The robot spins 360º
	* The robot turns left and drives the same distance as the previous step
	* Repeat, increasing the distance to be driven for the next two turns.

**Implementation:**

In order to ensure the best possible obstacle avoidance in this algorithm, rather than implement the driving ourselves, we send the movements described above to the robot as a series of AMCL goals using the following algorithm:

```
initial_pose = a point in the map's whitespace area
publish initial_pose
goal = initial_pose
offset = 1
polarity = -1
while not shutdown:
	if not lost:
		reset offset and polarity
		continue
	
	spin 360 degrees at a rate of 72 degrees/second
	
	if goal.x == goal.y:
		goal.x += offset
	else:
		goal.y += offset
		offset = polarity * (|offset| + 1)
		polarity *= -1
	publish goal
	wait for amcl to complete or a 5-second timeout
```

### Goal saving

One potential bug arising from AMCL-based wandering is that the robot would forget any AMCL goal it had been working towards when it was kidnapped. To fix this, we have included a `/move_base_simple/goal` subscriber. Whenever it receives a message, indicating a new AMCL goal, it saves that goal in this node as `current_goal`.

In our `flying_or_lost` method, which recognizes wheel drops as described above, we have included a check for if the robot's state at the moment of kidnapping was `FLYING`. If the state was `NAVIGATING`, such that the robot was in the middle of AMCL, we set `lock_current_goal` to True, which acts as a flag to indicate that our node should stop saving new incoming goals because our AMCL-based wandering is about to start.

Finally, our `if get_state() != States.LOST` block, which is responsible for resetting the node once wandering is complete, includes a check for `lock_current_goal`. If `lock_current_goal` is True, then the robot must have been working towards an AMCL goal prior to the kidnapping, so our node re-publishes that goal with an updated timestamp and the robot can continue its journey.

###### _Ari Carr and Jacky Chen 11/28/2018, updated 12/1/2018 with a new wander algorithm_