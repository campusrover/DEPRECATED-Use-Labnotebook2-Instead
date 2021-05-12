# Hardware

![Basic hardware Layout](pupper_titan.png)

The robot was donated by the [Hands-On Robotics Initiative](https://handsonrobotics.org/). Due to time constraints and some delays, a prebuilt robot was delivered instead of a parts kits. The build instructions can be found on this [Google Doc](https://docs.google.com/document/d/1ztO-zyF31r9wYJvHZEMbAYyhtuHloNoHOm8AScbr3wc). It also includes other relevant instructions that will be referenced later, such as motor calibration, software installation and run instructions.

## Hardware Overview

![RaspberryPi](rpi.png)

### Default hardware (robot)
The robot consists of 12 C610 motor controllers, 12 M2006 motors, a Teensy board, cables and the chassis.

### Additional hardware
* Raspberry Pi
* Raspberry Pi Camera
* [6S Lipo Battery 22.2V](https://www.amazon.com/dp/B08BZ9P469?psc=1&pldnSite=1)
* USB Battery Pack for Raspberry Pi
* Keyboard (for emergency stop)

## Battery Charging Settings
The battery charger in the lab supports charging the 6S Lipo Battery. Settings can be found below:

```
Chemistry:    LiPo
Voltage:      22 Volts
Capacity:     1300mAh
Charge Rate:  5C
Series Count: 6S
```

The batteries took around 1 hour to charge and last around 2-3 hours per charge.

The USB Battery Pack used was a generic one that provided enough voltage for the Pi. These are usually battery packs that support fast charge technology for smart phones. An appropriate cable is needed (e.g: USB 3.0 to Micro-USB or otherwise)

## Calibration Tips
* Motor calibration can be found in the [doc linked earlier](https://docs.google.com/document/d/1ztO-zyF31r9wYJvHZEMbAYyhtuHloNoHOm8AScbr3wc). The robot's casing may have to be opened to access some of the motor controllers.
* Instructions for leg calibration can be found [here](https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/dji/README.md). Best results were achieved by supporting the robot with a box and adding paddings (made of paper towels and tape) between the legs and the box to get desired motor angles.

![Pupper Calibration Setup](Pupper2Calibration.png)