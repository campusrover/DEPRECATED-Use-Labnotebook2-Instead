# Communicating over Rosserial

Rosserial is a libaray of ROS tools that allow a ROS node to run on devices like Arduinos and other non-linux devices. Because Arduino and clones like Tivac are programmed usinc C++, writing a Rosserial node is very similar to writing a ROScpp node. For you, the programmer, you can program your rosserial node to publish and subscribe in the same way as a normal ROS node. The topics are transferred across the USB serial port using the Rosserial_python serial_node on whichever device the Rosserial device is tethered to - so to be clear, a Rosserial node cannot properly communicate with the rest of ROS without the serial_node to act as a middle man to pass the messages between ROS and the microcontroller.

This page assumes you have followed the rosserial installation instructions for your [device and IDE](http://wiki.ros.org/rosserial#Client_Libraries).

So, you've written a node on your Tiva C and You'd like to make sure that it is working. Upload your code to the board, then proceed.

First, enable your machine to read from the board's serial connection. Run this command in the terminal **Only if you have not enabled this device in udev rules**:

``` sh
sudo chmod a+rw /dev/ttyACM0
```

Now, start up a `roscore`, then run this command:

``` sh
rosrun rosserial_python serial_node.py _port="/dev/ttyACM0" _baud=57600
```

This tells the serial node to look for ros topics on port `/dev/ttyACM0` and to communicate at a rate of 57600. If the baudrate is defined as a diferrent value in your embedded source code, then you must use that value instead!

* there is also a rosserial_server

If there is no red error text printing in the terminal, then you should be good to go! Open a `rostopic list` to see if your topics from the board are appearing, and `rostopic echo` your topics to ensure that the correct information is being published.
