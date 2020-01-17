# Working with the low-level of "chefbot"

The campus rover 4 prototype is based off of the "chefbot" from the textbook "Learning Robotics Using Python" by Lentin Joseph. Rather than using an OpenCR board like a turtlebot3, it uses a Tiva C Launchpad. To write code for the Tiva C, we need two things:

1. The Energia IDE to write our code, compile it and upload it to the board.
2. rosserial_tivac to allow the Tiva C to act like a ROS node.

## Step 1. Download the Energia IDE

go to [The Energia Download Page](https://energia.nu/download/) to get the IDE. Then, choose where to extract the files. You can put them wherever you like, I chose to keep them in my Downlaods folder, so for me the path to the IDE and it's related files/ folders is `~/Downloads/energia-1.8.10E23-linux64/energia-1.8.10E23`, because my version of energia is 1.8.10E23. In the future, your version may be different.

To make it easy to open the Energia IDE, I added this alias to my `.bashrc`:

``` sh
alias energiaIDE='cd ~/Downloads/energia-1.8.10E23-linux64/energia-1.8.10E23 && ./energia'
```

## Step 2: Download and install rosserial_tivac

We need to download, build and install the rosserial_tivac package.

Run these five commands in your terminal:

``` sh
cd ~/catkin_ws/src
git clone https://github.com/vmatos/rosserial_tivac.git
cd ~/catkin_ws
catkin_make
catkin_make install
```

Add this line to your `.bashrc` (preferably near similar lines that were automatically added when ROS was installed):

``` sh
source ~/catkin_ws/install/setup.bash
```

## Step 3: Enable energia to complie with ROS

In the terminal, cd to where energia is. So, because my Energia is in the Downloads directory, I input:

``` sh
cd ~/Downloads/energia-1.8.10E23-linux64/energia-1.8.10E23
```

`ls` to see the files and subdirectories in the folder. There should be a `libraries` directory. Run this command:

``` sh
rosrun rosserial_tivac make_libraries_energia libraries/
```

If that command completes without error, you should find the directory `ros_lib` inside `libraries`. Congratulations! You can now turn your Tiva C launchpad into a ROS node.

This guide was adapted from [here](http://wiki.ros.org/rosserial_tivac/Tutorials/Energia%20Setup)

## Step 4: running the node and communicating with ROS

For the node on your Tiva C to communicate with ROS, the computer it is attached to must be running the `serial_node` from the `rosserial_python` package. This node is installed by default along with ROS. Run it independantly with `rosrun`, or add it to your project's launch file.
