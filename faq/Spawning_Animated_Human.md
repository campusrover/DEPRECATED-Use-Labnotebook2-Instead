---
title: Spawning and Animated Humans
---
#### Nathan Cai

This FAQ section assumes understanding of creating a basic Gazebo world and how to manipulate a XML file. This tutorial relies on the assets native to the Gazebo ecosystem.

### Setting up empty Gazebo world

By Nathan Cai

(If you have a prexisting Gazebo world you want to place an actor you can skip this part)
Empty Gazebo worlds often lack a proper ground plane so it must be added in manually. You can directly paste this code into the world file.

```xml

<?xml version="1.0" ?>
<sdf version="1.6">
   <world name="default">
      <!-- A ground plane -->
      <include>
         <uri>model://ground_plane</uri>
      </include>
      <!-- A global light source -->
      <include>
         <uri>model://sun</uri>
      </include>
```

### Placing an actor in the world

## TL:DR Quick Setup

Here is the quick setup of everything, one can simply copy and paste this code and change the values to suit the need:

(If you do not have a Plugin for the model, please delete the Plugin section)

```xml
<actor name="actor1">
    <pose>0 0 0 0 0 0</pose>
    <skin>
        <filename>PERSON_MESH.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="NAME">
        <filename>ANIMATION_FILE_NAME.dae</filename>
        <scale>1.000000</scale>
        <interpolate_x>true</interpolate_x>
    </animation>
    <plugin name="PLUGIN NAME" filename="PLUGIN_FILE_NAME.so">
        ...
        ...
        ...
    </plugin>
    <script>
        <trajectory id="0" type="NAME">
            <waypoint>
                <time>0</time>
                <pose>0 2 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>2</time>
                <pose>0 -2 0 0 0 -1.57</pose>
            </waypoint>
            ...
                ...
                ...
            ...
        </trajectory>
      </script>
</actor>
```

## Defining an actor

Human or animated models in Gazebo are called actors, which contains all the information of an actor. The information can include: `pose`, `skin`, `animation`, or any `plugins`. Each actor needs a unique name. It uses the syntax:
```xml
<actor name="actor">
    ...
    ...
    ...
</actor>
```

## Change the actor pose

The pose of the is determined using the pose parameter of an actor. The syntax is:
(x_pos, y_pos, z_pos, x_rot, y_rot, z_rot)

```xml
<actor name="actor">
    <pose>0 0 0 0 0 0</pose>
    ...
    ...
</actor>
```

## Add in Skins

The skin is the mesh of the actor or model that you want to place into the world. it is placed in the actor group and takes in the input of the filename. The syntax is:

```xml
<actor name="actor">
    <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
    </skin>
</actor>
```
The mesh scale is also adjustable by changing the scale parameter.

## Add in Animations

Though the actor can operate without animations, it is preferable for you to add one, especially if the model is to move, as it would make the enviorment more interesting and realistic.

To add an animation to the actor, all it needs is a name fore the animation, and the file that contains the animation. The syntax for this is:
NOTE: THE FILE BECOMES BUGGY OR WILL NOT WORK IF THERE IS NO SKIN.

# IMPORTANT: IN ORDER FOR THE ANIMATION TO WORK, THE SKELETON OF THE SKIN MUST BE COMPATABLE WITH THE ANIMATION!!!!

```xml
<actor name="actor">
    <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.000000</scale>
    </animation>
</actor>
```

The animation can also be scaled.

## Scripts

Scripts are tasks that you can assign an actor to do, in this case it is to make the actor walk around to specific points at specific times. The syntax for this is:

```xml
<actor name="actor">
    <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.000000</scale>
    </animation>
    <script>
        <trajectory id="0" type="NAME">
            <waypoint>
                <time>0</time>
                <pose>0 2 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>2</time>
                <pose>0 -2 0 0 0 -1.57</pose>
            </waypoint>
            ...
                ...
                ...
            ...
        </trajectory>
    </script>
</actor>
```
You can add as many waypoitns as you want so long as they are at different times. The actor will navigate directly to that point at the specified time of arrive in `<time>0</time>` and pose using `<pose>0 0 0 0 0 0</pose>`.

## Plugin addons

The actor can also take on plugins such as obstacle avoidance, random navigation, and potentially teleop. The parameters for each plugin may be different, but the general syntax to give an actor a plugin is: 

```xml
<actor name="actor">
    <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.000000</scale>
    </animation>
    <plugin name="PLUGIN_NAME" filename="NAME_OF_PLUGIN_FILE">
        ...
    </plugin>
</actor>
```



With all of this you should be able to place a human or any model of actor within any Gazebo world. For reference, you can refer to the Gazebo actor tutorial for demonstration material.

## References
- [Make an animated model (actor)](http://gazebosim.org/tutorials?tut=actor&cat=build_robot)
