# Communicating over Rosserial

So, you've written a node on your Tiva C and You'd like to make sure that it is working. Upload your code to the board, then proceed.

First, enable your machine to read from the board's serial connection. Run this command in the terminal:

``` sh
sudo chmod a+rw /dev/ttyACM0
```

Now, start up a `roscore`, then run this command:

``` sh
rosrun rosserial_python serial_node.py _port="/dev/ttyACM0" _baud=57600
```

This tells the serial node to look for ros topics on port `/dev/ttyACM0` and to communicate at a rate of 57600.

If there is no red error text printing in the terminal, then you should be good to go! Open a `rostopic list` to see if your topics from the board are appearing, and `rostopic echo` your topics to ensure that the correct information is being published.
