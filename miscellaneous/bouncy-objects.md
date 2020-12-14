# Creating Bouncy Objects in Gazebo

### August Soderberg & Joe Pickens

This guide will give you all the tools you need to create a bouncy surface in Gazebo whether you want to make a basketball bounce on the ground or to make a hammer rebound off of an anvil.

First you will need to create the URDF file for your model, examine any of the URDF files given to you for help with this, I will put a generic example of a ball I created below.

```xml
<robot name="ball">
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <sphere radius=".5"/>
            </geometry>
        </visual>

        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <sphere radius=".5"/>
            </geometry>
        </collision>

        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="1"/>
            <inertia ixx="8e-03" ixy="-4-05" ixz="1e-04"
                                 iyy="8e-03" iyz="-4-06"
                                             izz="8e-03" />
        </inertial>
    </link>
    <gazebo reference="base_link">
        <mu1>1</mu1>
        <mu2>1</mu2>
        <kp>500000</kp>
        <kd>0</kd>
        <minDepth>0</minDepth>
        <maxVel>1000</maxVel>
    </gazebo>
</robot>
```

A few notes on the sample URDF we will be discussing above: the model is a 0.5m radius sphere, the inertia tensor matrix is arbitrary as far as we are concerned (a third-party program will generate an accurate tensor for your models if you designed them outside of Gazebo), and the friction coefficients "mu1" and "mu2" are also arbitrary for this demonstration.

To create a good bounce we will be editing the values within the "gazebo" tags, specifically the "kp", "kd", "minDepth", and "maxVel".

## maxVel
When objects collide in Gazebo, the simulation imparts a force on both objects opposite to the normal vector of their surfaces in order to stop the objects from going inside of one another when we have said they should collide. The "maxVel" parameter specifies the velocity (in meters/second) at which you will allow Gazebo to move your object to simulate surfaces colliding, as you will see later, this value is unimportant as long as it is greater than the maximum velocity your object will be traveling.

## minDepth
Gazebo only imparts this corrective force on objects to stop them from merging together if the objects are touching. The "minDepth" value specifies how far (in meters) you will allow the objects to pass through each other before this corrective velocity is applied; again we can set this to 0 and forget about it.

## kd
The "kd" value is the most important for our work, this can be thought of as the coefficient of elasticity of collision. A value of 0 represents a perfectly elastic collision (no kinetic energy is lost during collision) and as the values get the collision becomes more inelastic (more kinetic energy is converted to other forms during collision). For a realistic bouncing ball, just start at 0 and work your way up to higher values until it returns to a height that feels appropriate for your application, a value of 100 is more like dropping a block of wood on the ground for reference.

## kp
The "kp" value can be thought of as the coefficient of deformation. A value of 500,000 is good for truly solid objects like metals; keep in mind that an object with a "kd" value of 0 and a "kp" value of 500,000 will still bounce all around but will be hard as metal. A "kp" of 500 is pretty ridiculously malleable, like a really underinflated yoga ball, for reference. Also, keep in mind that low "kp" values can often cause weird effects when objects are just sitting on surfaces so be careful with this and err on the side of closer to 500,000 when in doubt.

There you go, now you know how to create objects in Gazebo that actually bounce!

## Tips When Debugging
Here are a bunch of random tips for debugging:

When testing how much objects bounce, try it on the grey floor of Gazebo and not on custom created models since those models will also affect how your object bounces.

Keep in mind all external forces, especially friction. If a robot is driving into a wall and you want it to bounce off, that means the wheels are able to maintain traction at the speed the robot is travelling so even if that speed is quickly reversed, your robot will simply stop at the wall as the wheels stop all bouncing movement.

When editing other URDF files, they will often have a section at the top like:

```xml
<xacro:include filename="$(find turtlebot3_description)/urdf/common_properties.xacro"/>
<xacro:include filename="$(find turtlebot3_description)/urdf/turtlebot3_waffle.gazebo.xacro"/>
```

Make sure these files aren't affecting the behavior of your model, they could be specifying values of which you were unaware.

If your simulation is running at several times real time speed, you could miss the bouncing behavior of the ball, make sure simulation is running at a normal "real-time-factor".

If any small part has a low "maxVel" value, it could change the behavior of the entire model.

Good luck!