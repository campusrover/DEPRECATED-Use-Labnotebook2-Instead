# Smart Rotation for Navigation
### Author - Eyal Cohen

#### Given two coordinates in a 2-dimensional plane and your robots' current direction, the smart rotation algorithm will calculate your target angle and return whether your robot should to turn left, turn right, or go straight ahead to reach its navigation goal.

## How It Works:
Inputs: `posex1` is a float that represents the  x-axis coordinate of your robot, `posey1` is a float that represents your robot's y-axis coordinate. `posex2` and `posey2` are floats that represent the x and y of your robot's goal. Lastly, `theta` represents your robot's current pose angle.

``` python 
from math import atan2
import math


def smartRotate(posex1, posey1, posex2, posey2, theta):
    inc_x = posex2 -posex1
    inc_y = posey2 -posey1
    angle_goal = atan2(inc_y, inc_x)
    # if angle_to_goal < 0:
    #     angle_to_goal = (2* math.pi) + angle_to_goal
    print("angle goal = ",angle_goal)
    goal_range = theta + math.pi
    wrapped = goal_range - (2 * math.pi)
    if abs(angle_goal - theta) > 0.1:
        print(theta)
        print("goal_range = ",goal_range)
        if (goal_range) > (2 * math.pi) and (theta < angle_goal or angle_goal < wrapped):
            print("go left")
        elif (goal_range) < (2 * math.pi) and (theta < angle_goal and angle_goal < goal_range):
            print("go left")
        else:
            print("go right")

    else:
        print("go straight")
```

Firstly, the `angle_goal` from your robot's coordinate (not taking its current angle into account) is calculated by finding the arc tangent of the difference between the robot's coordinates and the goal coordinates. 

In order to decide whether your robot should go left or right, we must determine where the `angle_goal` is relative to its current rotational direction. If the `angle_goal` is on the robot's left rotational hemisphere, the robot should rotate left, otherwise it should rotate right. Since we are working in Radians, π is equivilant to 180 degrees. 
To check whether the `angle_goal` is within the left hemisphere of the robot, we must add π to `theta` (the robot's current direction) to get the upperbound of the range of values we want to check the target may be included in. If the `angle_goal` is between `theta` and that upper bound, then the robot must turn in that direction to most efficiently reach its goal.

## Consider This Example:
``` python
    smartRotation(0,0,2,2,0)
```

If your robot is at (0,0), its rotational direction is 0, and it's target is at (2,2), then its `angle_goal` would equal = 0.785. First we check whether its current angle's deviation from the `angle_goal` is significant by finding the difference and seeing if its larger than 0.1. If the difference between the angles is insignificant the robot should go straight towards its goal. In this case however, `angle_goal` - `theta` (0.785 - 0) is greater than 0.1, so we know that we must turn left or right to near our `angle_goal`. To find out whether this angle is to the left or the right of the robot's current angle, we must add π to its current angle to discover the point between its left and right hemispheres. In this case, if the `angle_goal` is between `theta` and its goal_range, 3.14 (0(`theta`) + π), then we would know that the robot must turn left to reach its goal. 

However, if `theta` (your robot's current direction) + π is greater than 2π (maximum radians in a circle) then the left hemisphere of your robot is partially across the 0 radian point of the circle. To account for that case, we must calculate how far the goal range wraps around the circle passed the origin. If there is a remainder, we check whether the `angle_goal` is between `theta` and 2π or if the `angle_goal` is present within the remainder of the range that wraps around the origin. If either of these conditions are met then we know that your robot should turn left to most efficiently arrive at its goal, otherwise it should turn right.

