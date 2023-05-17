---
title: How do I control the Arm
desc: Converting Arm Coordinates
date: may-2023
status: new
author: Elliot Siegal
type: faq
---
# Arm Motion Control

## Problem

The InterbotixPincherX100 can rotate, move up and down, and extend. To get the arm to go to a specific point in space given (x, y, z) coordinates, the x and y compenents must be converted to polar coordinates.

## Solution
<img width="318" alt="Screen Shot 2023-05-06 at 10 54 42 AM" src="https://user-images.githubusercontent.com/62267188/236631562-4e0a5d32-b811-4736-9ebe-0f4b7ebd5d9c.png">
<i>Moving the arm to point (x, y), top-down view</i><br><br>

Since the arm can move out/back and up/down to specific points without conversion, consider only a top-down view of the arm (x-y axis). To get the rotation and extension of the arm at a specific point, consider the triangle shown above. For rotation θ: $θ=atan(x/y)$. For extension r: $r=y/cos(θ)$. The up-down movement of the arm remains the same as the given z coordinate.

The arm can be moved to the desired point using set_ee_cartesian_trajectory. In this method, the extension and motion up/down is relative so the current extension and vertical position of the arm must be known as current_r and current_z.

```
bot = InterbotixManipulatorXS("px100", "arm", "gripper")
theta = math.atan(x/y)
dr = y / (math.cos(theta)) - current_r
dz = z - current_z
bot.arm.set_single_joint_position("waist", theta)
bot.arm.set_ee_cartesian_trajectory(r = dr, z = dz)
```