###### _Ari Carr and Jacky Chen 11/28/2018_

---
# *lost\_and\_found.py*
---

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

The following algorithm is used to dictate the robot's random wandering, while also ensuring it avoids obstacles. It is inspired by the Roomba algorithms we wrote at the beginning of the semester.

The TurtleBot has four possible movements: It can drive forwards, drive backwards, turn left, and turn right. 

* It turns left and moves forward when the closest obstacle is on its right.
* It turns right and moves forward when the closest obstacle is on its left.
* It backs up for a while when the closest obstacle is directly in front of the robot.
* It drives forwards, combined with a certain angular velocity, when no obstacle within a set distance is detected. An angular velocity is included in order to let the robot circle around and explore as much space as it can.

**Implementation:**

The TurtleBot publishes an array of  640 elements to the `/scan` topic, where each element represents a pixel of the camera's width and is a number represents the distance from the nearest object to the corresponding pixel.

We divided the camera into three parts: front, left, and right. From there, we identified which part of the camera has the closest obstacle and then reacted to that information as described above.

### Known issues

**`NaN` values**

There are two thresholds for which the TurtleBot's `/scan` array will contain `NaN` for the scan number. The min threshold, for which any lower value will be replaced with `NaN`, and the max threshold, for which any higher value will be replaced with `NaN`. In our implementation, we filtered `NaN` out and only take actual numbers into count when evaluating the array of values. However, this poses a problem when the TurtleBot is too close to an obstacle to see it; it may not notice the obstacle at all.

**Backing up**

We found that the making the robot drive backwards is always risky, since unlike the LIDAR-equipped TurtleBot 3 on which our original Roomba algorithm was implemented, the TurtleBot 2 doesn't have 360º vision, and thus cannot necessarily trust that it is safe to back up. We addressed this by limiting how much time the robot spends backing up upon seeing a forwards obstacle, to minimize the risk of hitting something.

**Fiducial integration**

The node responsible for processing fiducials has not yet been fully integrated with the state management system, such that seeing a fiducial does not publish a state change so we weren't able to test that aspect of the code. However, by manually publishing a `LOST` -> `LOCALIZING` state change, we can verify that our loop does break when a state change is noticed, and thus can say that it should break successfully once the fiducial node gains full state integration.