# Multi Robot Setup

## How to namespace multiple robots and have them on the same roscore

To have multiple robots on the same ROS core and each of them listen to a separate node, namespacing would be an easy and efficient way to help.

### Set namespace on robots' onboard computers with environment variable

* Boot up the robot and ssh into it
* On the robot's onboard computer, open the terminal and type in

```sh
nano ~/.bashrc
```

Then add the following line to the end of the file, using the robots name as the namespace

```sh
export ROS_NAMESPACE={namespace_you_choose}
```

Also make the following other changes:

```sh
alias bu='roslaunch turtlebot3_bringup turtlebot3_robot.launch'
export IP="$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')"
export ROS_IP=$IP
export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export ROS_NAMESPACE=roba
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
```

* Save the file and don't forget to do `source ~/.bashrc`

## Set namespace on your laptop with environment variable

* Now that the robot is configured properly with its own unique name space, how do we talk to it?
* There are three ways:
  1. Configure your laptop to be permanently associated with the same name space
  2. Set a temporary environment variable that specifies the name space
  3. Add the namespace to a launch file
  4. Use the \_\_ns parameter for roslaunch or rosrun \(not recommended\)

### Permanently associate your laptop with the name space

* Use the same steps above. Make sure your namespace is exactly the same as the namespace of the robot you want to control.
* From now on, whenever you do, e.g. a cmd\_vel, it will be directed just to your robot.

### Use an environment variable

* Set namespace for a termimal with temporary environment variable.
* To set a namespace temporarily for a terminal, which will be gone when you close the termial, just type in `export ROS_NAMESPACE={namespace_you_choose}` directly in your terminal window.
* You can use `echo $ROS_NAMESPACE` to check it.

### Use the .launch file

* To set namespace for a node in launch file, Use the attribute `ns`, for example:

`<node name="listener1" pkg="rospy_tutorials" type="listener.py" ns="{namespace_you_choose}" />`

### Not recommended: add \_\_ns to the run or launch command

1. Launch/Run a file/node with namespace in terminal. Add a special key `__ns` \(double underscore here!\) to your command, for example: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch __ns:={namespace_you_choose}` However,
2. Use of this keyword is generally not encouraged as it is provided for special cases where environment variables cannot be set. \([http://wiki.ros.org/Nodes](http://wiki.ros.org/Nodes)\)

### Publishing/Subsribing topics in other namespace

Do a `rostopic list`, you will find out topics under a namespace will be listed as `/{namespace}/{topic_name}`

## Make changes to turtlebot3\_navigation package on your laptop

* Type `roscd turtlebot3_navigation` to go to the package directory.
* Type `cd launch` to go to the folder that stores .launch files.
* Open the `turtlebot3_navigation.launch` file with nano or your favorite code editor.
* You will see the scripts for launching move base and rviz:

```xml
  <!-- move_base -->
  <include file="$(find turtlebot3_navigation)/launch/move_base.launch">
    <arg name="model" value="$(arg model)" />
    <arg name="move_forward_only" value="$(arg move_forward_only)"/>
  </include>

  <!-- rviz -->
  <group if="$(arg open_rviz)">
    <node pkg="rviz" type="rviz" name="rviz" required="true"
          args="-d $(find turtlebot3_navigation)/rviz/turtlebot3_navigation.rviz"/>
  </group>
```

### Change move base arguments

* Add another argument `cmd_vel_topic` to the four arguments in the `<!-- Arguments -->` section:

  ```xml
    <!-- Arguments -->
    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>
    <arg name="open_rviz" default="true"/>
    <arg name="move_forward_only" default="false"/>
    <arg name="cmd_vel_topic" default="/{namespace_you_choose}/cmd_vel"/>
  ```

  * Pass the new argument to move base launch file:

    ```xml
    <!-- move_base -->
    <include file="$(find turtlebot3_navigation)/launch/move_base.launch">
     <arg name="model" value="$(arg model)" />
     <arg name="move_forward_only" value="$(arg move_forward_only)"/>
     <arg name="cmd_vel_topic" value="$(arg cmd_vel_topic)"/>
    </include>
    ```

* These changes will make move base publish to `/{namespace_you_choose}/cmd_vel` instead of `/cmd_vel`. Open the `move_base.launch` file in the launch folder, you will see why it worked.

### Change rviz arguments

* Type `roscd turtlebot3_navigation` to go to the package directory.
* Type `cd rviz` to go to the folder that has `turtlebot3_navigation.rviz`. It stores all the arguments for rviz.
* Open `turtlebot3_navigation.rviz` with VSCode or other realiable code editors, since this file is long and a bit messy.
* In the .rviz file, search for all the lines that has the part `Topic:`
* Add your namespace to the topics you found. For example, change

  ```sh
    Topic: /move_base/local_costmap/footprint
    `
  ```

    to

  ```sh
    Topic: /roba/move_base/local_costmap/footprint
  ```

* These changes will make rviz subsribe to topics in your namespace.
