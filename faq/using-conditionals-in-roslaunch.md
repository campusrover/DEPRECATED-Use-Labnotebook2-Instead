# Using the Group Tag to Make Conditional Statements Within Launch Files

## Author: Lisandro Mayancela

When making launch files you may sometimes want aspects of your launch (Such as the urdf file that is used) to be dependent on certain conditions

![All of these robots use the same launch file but the urdf file that is loaded is different based on robot team and type](https://gyazo.com/630de797bf9efc23a45847f3cc37a1fc)

In order to have this functionality you can use the <group> tag with an if parameter like so:

---
<group if="condition goes here">
    If statement body
</group>
---

## Examples

For a better example let's look at a launch file which spawns a robot into a gazebo world:

---
<launch>
    <arg name="robot_name"/>
    <arg name="init_pose"/>
    <arg name="team"/>
    <arg name="type"/>

    <group if="$(eval team == 'Red')">
        <group if="$(eval type == 'painter')">
            <param name="robot_description" 
                command="$(find xacro)/xacro.py $(find robopaint)/urdf/red/painterbot_red_burger.urdf.xacro" />
        </group>
        <group if="$(eval type == 'attacker')">
            <param name="robot_description" 
                command="$(find xacro)/xacro.py $(find robopaint)/urdf/red/attackerbot_red_burger.urdf.xacro" />
        </group>
    </group>

    <group if="$(eval team == 'Blue')">
        <group if="$(eval type == 'painter')">
            <param name="robot_description" 
                command="$(find xacro)/xacro.py $(find robopaint)/urdf/blue/painterbot_blue_burger.urdf.xacro" />
        </group>
        <group if="$(eval type == 'attacker')">
            <param name="robot_description" 
                command="$(find xacro)/xacro.py $(find robopaint)/urdf/blue/attackerbot_blue_burger.urdf.xacro" />
        </group>
    </group>

    node name="spawn_minibot_model" pkg="gazebo_ros" type="spawn_model"
     args="$(arg init_pose) -urdf -param robot_description -model $(arg robot_name)"
     respawn="false" output="screen" />

    <node pkg="robot_state_publisher" type="robot_state_publisher" 
          name="robot_state_publisher" output="screen"/>
</launch>
---


