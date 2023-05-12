---
title: Robot Arm Transforms
desc: Determine arm transforms using a real coordinate system
author: Vibhu Singh
date: may-2023
status: new
type: faq
---
# Determine arm transforms using a real coordinate system

## By Vibhu Singh

## A couple important things about the arm

1. Moving the arm in multiple steps is not the same as moving the arm with just one transform because of the motors in the arm
    - For example, moving the arm -0.03 and then 0.01 in the x direction is not the same as moving -0.02
    - The farther the arm moves in any direction, especially the y direction
2. The arm needs to be firmly secured on a surface, otherwise the whole apparatus will move around when the arm moves
3. The arm should be returned to the sleep position frequently, one team found that the sweet spot was every 5-6 transforms. If that isn't enough, you should return the arm to the sleep position after one full cycle of transforms.
4. Brush up on polar coordinates since those are the best way to think about the arm and arm transforms. The best way to think about the transforms is that the y transform is the theta and the x transform is the radius of the unit circle.

## How to set up

The arm set up doesn't have to be exactly as the image below, but having a setup that follows the same principles is going to be ideal.
- Having the camera positioned over the arm so that the arm is at the edge of the frame will make the math that you will have to do easier
- As mentioned above, the arm should be fixed on a stable surface
- For reproducability of the results, you will need to have the exact same, or as close as you can get to the exact same, set up so that all the math you do works correctly.
    - Depending on what you are trying to grab using the arm, a difference of even 1-2 centimeters is enough to mess up the equations

![An example setup that works](../../images/cargoclaw/cargoclaw_setup.jpg)

## How to get X transforms

As mentioned at the start of this guide it's a good idea to brush up on polar coordinates for this project. The x transform is basically the radius portion of a polar coordinate. The arm has a very limited range of motion for the X transforms, ranging from about -0.08 to .04 and the transforms can only go to the second decimal place. The best way to find a relationship between the X transforms and the real coordinate system is to use a stepwise function. 

It is important to remember to transform the coordinates so that the origin is at. This is why it was important to position the camera so that the arm was at one edge of the frame, it would make it easier to transform the coordinates the camera gives since there will be less transforms to make.

To get the "bins" for the stepwise function, the best way so far to do it is to have the arm extend or retract to all possible positions and record the coordinates, apply the transforms, and then convert to polar coordinates. What this will do is allow you to find a way to get ranges of radii that correspond with certain X transforms.

Below is an example of a graph that the Cargo Claw team made to illustrate their X transforms for their project. There are more details available about the project in the reports section under the cargoclaw project.

![x transform graph](../../images/stepwise-for-x-transform.png)

## How to get Y transforms

The best way to think about the Y transforms is to imagine them as angles of rotation on a unit circle. The first step to getting a good idea of the relationship between the Y transforms is to record what the coordinates are at even intervals of the Y transform locations you are trying to reach. Once you have that, apply the transforms to all the coordinates and then save the theta values. From there, it'll be easy work to graph the Y transforms against the theta values.

From here, to get an actual equation to convert from the coordinates to transforms, it'll be a good idea make a graph of the Y transforms against the theta values so that you can see what sort of pattern the relationship follows. A cubic regression works well because it can account for some of the problems that come with the arm's movement and the inaccuracies that. You might still have to do some fine tuning in your code, for example:

```
    if (y_transform > .5):
        y_transform += .05
    elif (y_transform > .6):
        y_tranfrom += .09
```

Alternatively, you can use two different cubic regressions, one for if the Y transform is positive and one for if the Y transform is negative. You can also try higher degree regressions, but you will have to decide if the additional complexity is worth it.

Below is an example of a graph that the Cargo Claw team made to show the relationship between the theta and the y transform. There are more details available about the project in the reports section under the cargoclaw project.

![y transform graph](../../images/y-transform-graph.png)

_[To get an applied and working example of everything described, you can look at the code for the cargoclaw project in the box_pickup.py file within the src directory of the git.](../reports/cargoclaw.md)_

