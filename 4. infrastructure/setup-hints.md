# Setup Troubleshooting

## Hints on how to set things up

* Reflects the "new" multi robot setup

### .bashrc ON BOTH your laptop and your Robot

* This shell script is run automatically whenever you open a new terminal or tab in a terminal
* This means that if you edit it and save it, it still does nothing until you open a new terminal
* If you want to apply it without opening a new terminal, do `source ~/.bashrc`
* These lines should be on your laptop as well as your robot, the same way.

```sh
alias bu='roslaunch turtlebot3_bringup turtlebot3_robot.launch'
export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export ROS_NAMESPACE=roba
export ROS_IP=<ip address of computer where this .bashrc is stored>
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
```

* ROS\_HOSTNAME: the IP address of this computer.
* ROS\_MASTER: the IP address of whereever ROSCORE is running. We have roscore always running on the little computer by the door. It happens to be called roscore1.cs.brandeis.edu
* ROS\_NAMESPACE: Indicates a unique name for a specific robot. That way when we send a /cmd\_vel it is turned into, e.g. /roba/cmd\_vel. This way all the robots can coexist on one ROSCORE. It also means you don't have to run your own ROSCORE anymore.

### ON THE ROBOT

* SSH into the robot
* Run the following command to launch ROS on the robot

```sh
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Test the configuration

* Did you do bringup on the ROBOT \(NOT ON YOUR COMPUTER!
  * if not, do so!
* Did bringup work
  * if not, check your environment variables
  * `printenv | grep ROS`
  * It should include all your settings and also env variables for the path to ROS
  * Make sure you run `source ~/.bashrc`
* Can you run rostopic list? `rostopic list`
  * If not, check your IP addresses
  * Check whether the ROSCORE computer is running.
  * Try pinging roscore1.cs.brandeis.edu
* Does it list /x/cmd\_vel among them?
  * If not, check your IP addresses. Also make sure you ran
* Can you teleop the robot? `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
  * if not, try a manual cmd\_vel: rostopic pub /rob\*/cmd\_vel

## Additional Intermediate Bash Tips

### Aliases: Live Better, Type Less\(TM\)

You may notice that the ros installation process added a few Aliases to your .bashrc file. These serve to make your life easier, and you can add more of your own custom aliases to make your life even easier. Here are a few recommended changes to make:

#### New Editor in eb

You'll notice eb uses nano by default. nano is fine, if you enjoy living in the dark ages. To make eb open a different text editor, just replace nano with whatever your favorite installed editor is - code \(vscode\), atom, vim, emacs, etc. Bonus tip: if you're not already using `eb` as a shortcut to edit the .bashrc, start using eb!

#### Networking Shortcuts

Try using this chunk in your bashrc:

```sh
alias connect='ssh $ROBOT@$ROBOT.dyn.brandeis.edu'
export ROBOT=robc
export ROS_MASTER_URI=http://$ROBOT.dyn.brandeis.edu:11311
export IP="$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')"
export ROS_IP=$IP
```

_NB_ the snippet of code above uses robc as the current robot, robc can be replaced with any valid robot name Here's what this chunk does: 1. The alias "connect" will ssh to your desired preconfigured robot \(saves quite a bit of typing\) 1. The variable ROBOT is used to reduce the redundancy of typing the robot name anythime you need to ssh or change your ROS\_MASTER\_URI 1. The last two lines will automatically obtain your IP address anytime you start a new terminal window Some additional things to note: 1. The connect alias grabs ROBOT when it is called, so it can be defined before ROBOT, but ROS\_MASTER\_URI grabs ROBOT when it is defined, so ROBOT must be defined first 1. The command `export ROBOT=<name>` can temporarily change the value of ROBOT - anything that accesses ROBOT will grab the temporary value \(Think about what this means given the last point: connect wil ssh to the temporary robot, but ROS\_MASTER\_URI will not update\)

#### Universally Useful Time Savers

One of the most common problems is that when you kill a ROS script, the most recent cmd\_vel latches and the robot will continue in that manner therefore, it is useful to have some sort of emergency stop procedures. 2 viable and common options: 1. create an alias for `rostopic pub` that publishes a blank twist to cmd\_vel 1. create an alias that launches teleop - this take longer, but will immediately stop the robot as soon as it is running.

#### The Possibilities Don't Stop Here

There's obviously still much more that could done to make a more efficient terminal experience. Don't be afraid to become a bash master. Here are some resources to help get started becoming more proficient in bash: [Bash Cheatsheet](https://devhints.io/bash)
