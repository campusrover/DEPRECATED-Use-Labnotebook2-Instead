# How to change joint control type

Each joint in ros has a type. These types determine the degrees of freedom of a joint. For example, a continuous joint can spin around a single axis, while a fixed joint has zero degrees of freedom and cannot move. At a low level, when publishing commands to a joint in ros you are telling it how you want it to move about its degrees of freedom.

The form these commands take, however, is not determined by the joint itself, but by its joint controller. The most common joint controllers (which are provided in ros by default) are effort controllers. These controllers come in three varieties. The first is the very simple "joint_effort_controller" which just takes in the amount of force/torque you want the joint to exert.
The second, and likely more useful type is the "joint_position_controller", which takes commands in the form of a desired position. In a continuous joint this would be a radian measurement for it to rotate to. The position controller uses a pid algorithm to bring the joint smoothly to the desired positon.
Finally, there is the "joint_velocity_controller", which takes in a velocity command and once again uses a pid algorithm to maintain this speed (this is most similar to the twist commands used in class, though rotational velocity is given in radians per second. Twist commands take care of the conversion to linear velocity for you).

In order to set up one of these joint controllers, you simply need to add a transmission block to your urdf file. This takes the following form: 

``` xml
<transmission name="tran">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```

In this section of xml code there are a number of variables which will need to be changed to fit your environment

1. transmission name. The specific name you choose doesn't matter, but it must be unique from any other transmissions in your urdf.
2. joint name. This must match the name of the joint you wish to control.
3. actuator name. See transmission name.
4. <hardwareInterface> This section must be changed to match the type of controller you wish to use.
    1. note that this appears twice. Once underneath the joint name, and again under actuator name. They must match each other.
    2. If you want to use a position controler instead of velocity replace "VelocityJointInterface" with "PositionJointInterface" or "EffortJointInterface" for an effort joint.

By making these modifications you can correctly set up a joint controller in your homemade urdf.
