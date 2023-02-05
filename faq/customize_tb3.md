# Customizing Turtlebot3

## Pito Salas / April 2022

## Objective

When we customize a TB3 with bigger wheels or a larger wheelbase, it is necesary to update the software in the OpenCr board.

* The steps required by OpenCR when changing wheel size or distance between wheels can be summarized as follows

* change WHEEL_RADIUS, WHEEL_SEPARATION and TURNING_RADIUS in turtlebot3_burger.h

* Change the circumference of the wheel in turtlebot3_core.ino. Here, the circumference can be calculated by 2Pi * WHEEl_RADIUS. The default value of the circumference of burger is 0.207.

* Install Arduino IDE on your PC and write the edited file (turtlebot3_core.ino). Please refer to the following e-Manual for this detail.
https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup
3.3. At the end of OpenCR Setup, there are instructions for using the Arduino IDE, click to expand and see the details.

