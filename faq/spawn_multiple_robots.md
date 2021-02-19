# Spawning Multiple Robots 
## Author: Belle Scott

In order to spawn multiple robots in a gazebo launch file (in this example, there are two robots, seeker and hider), you must define their x,y,z positions, as shown below. Also shown is setting the yaw, which is used to spin the robot. For example, if one robot was desired to face forward and the other backward, ones yaw would be 0 while the others would be 3.14. 

``` xml
    <arg name="model" default="burger" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="seeker_pos_x" default="0"/>
    <arg name="seeker_pos_y" default=" -0.5"/>
    <arg name="seeker_pos_z" default=" 0.0"/>
    <arg name="seeker_yaw" default="0"/>
    <arg name="hider_pos_x" default="0"/>
    <arg name="hider_pos_y" default="0.5"/>
    <arg name="hider_pos_z" default=" 0.0"/>
    <arg name="hider_yaw" default="0" />
```

After setting their initial positions, you must use grouping for each robot. Here, you put the logic so the launch file knows which python files correspond to which robot. It is also very important to change the all the parameters to match your robots name (ex: $(arg seeker_pos_y)). Without these details, the launch file will not spawn your robots correctly. 

``` xml
    <group ns="seeker">
        <param name="robot_description" command="$(find xacro)/xacro.py $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
        <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
            <param name="publish_frequency" type="double" value="50.0" />
            <param name="tf_prefix" value="seeker" />
        </node>
        <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model seeker -x $(arg seeker_pos_x) -y $(arg seeker_pos_y) -z $(arg seeker_pos_z)          -Y $(arg seeker_yaw) -param robot_description" />
    </group>
    <group ns="hider">
        <param name="robot_description" command="$(find xacro)/xacro.py $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
        <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
            <param name="publish_frequency" type="double" value="50.0" />
            <param name="tf_prefix" value="hider" />
        </node>
        <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model hider -x $(arg hider_pos_x) -y $(arg hider_pos_y) -z $(arg hider_pos_z) -Y               $(arg hider_yaw) -param robot_description" />
    </group>
```
### References

* We also found this [Launch Multiple Robots in Gazebo](https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/)