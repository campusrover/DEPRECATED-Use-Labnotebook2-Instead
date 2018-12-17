There are currently two versions of the mutant in existence. They both have the same footprint, that of a Turtlebot 3 Waffle.

One of those robots, has large green wheels but operates as a standard Turtlebot3 with ROS etc. The only difference is in the OpenCR firmware, where the wheel diameter, robot radius, etc. fields are modified to account for the different chassis and wheels. It is fully operational.

The other robot, has blue wheels of similar size to the standard TB3 wheels. This model is where new motors/motor controllers are tested. The current combination are Uxcell 600 RPM encoder gear motors. When installing/wiring new motors ALWAYS GO OFF OF THE PCB (printed circuit board). Meaning follow whatever is actually written on the back of the motor. They are plugged in and controlled by a RoboClaw motor controller. For our purposes, it is communicating over packet serial with an Arduino, which gives it serial commands. The RoboClaw is well documented and has a functional Arduino library that is used for most of its operation. The RoboClaw comes with a Windows software interface that allows for simple initial programming and Autotuning of the PID control. The robot is equipped with a latching Estop button. The robot does not currently drive particularly straight but that does not seem to be the result of the speed control (the encoder counts stay similar). Rather it is likely a result of the alignment of the wheels. To attach the larger wheels to these motors, an adapter needs to be made or found from the smaller hex of the driver pin to a larger size. The RoboClaw controller is top of the line across the board. To interface with ROS, the best approaches would be to do so through an Arduino using ROSSerial or directly over MicroUSB, using the existing ROS-RoboClaw libraries.

Uxcell Motors: https://www.amazon.com/uxcell-600RPM-Encoder-Mounting-Bracket/dp/B078HYRPZM/ref=sr_1_3?ie=UTF8&qid=1545081920&sr=8-3&keywords=uxcell+motor+wheel

RoboClaw Manual: http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf 

Wiring diagrams and pictures:
