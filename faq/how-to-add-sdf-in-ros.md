---
title: How to add an SDF Model
author: Shuo Shi
---

## How to add SDF model in ROS
Author: Shuo Shi
### Overview
This tutorial describes the details of a SDF Model Object.

SDF Models can range from simple shapes to complex robots. It refers to the <model> SDF tag, and is essentially a collection of links, joints, collision objects, visuals, and plugins. Generating a model file can be difficult depending on the complexity of the desired model. This page will offer some tips on how to build your models.

### Components of SDF Models
Links: A link contains the physical properties of one body of the model. This can be a wheel, or a link in a joint chain. Each link may contain many collision and visual elements. Try to reduce the number of links in your models in order to improve performance and stability. For example, a table model could consist of 5 links (4 for the legs and 1 for the top) connected via joints. However, this is overly complex, especially since the joints will never move. Instead, create the table with 1 link and 5 collision elements.

Collision: A collision element encapsulates a geometry that is used for collision checking. This can be a simple shape (which is preferred), or a triangle mesh (which consumes greater resources). A link may contain many collision elements.

Visual: A visual element is used to visualize parts of a link. A link may contain 0 or more visual elements.

Inertial: The inertial element describes the dynamic properties of the link, such as mass and rotational inertia matrix.

Sensor: A sensor collects data from the world for use in plugins. A link may contain 0 or more sensors.

Light: A light element describes a light source attached to a link. A link may contain 0 or more lights.

Joints: A joint connects two links. A parent and child relationship is established along with other parameters such as axis of rotation, and joint limits.

Plugins: A plugin is a shared library created by a third party to control a model.

### Building a Model
#### Step 1: Collect your meshes
This step involves gathering all the necessary 3D mesh files that are required to build your model. Gazebo provides a set of simple shapes: box, sphere, and cylinder. If your model needs something more complex, then continue reading.

Meshes come from a number of places. Google's 3D warehouse is a good repository of 3D models. Alternatively, you may already have the necessary files. Finally, you can make your own meshes using a 3D modeler such as Blender or Sketchup.

Gazebo requires that mesh files be formatted as STL, Collada or OBJ, with Collada and OBJ being the preferred formats.

#### Step 2: Make your model SDF file
Start by creating an extremely simple model file, or copy an existing model file. The key here is to start with something that you know works, or can debug very easily.

Here is a very rudimentary minimum box model file with just a unit sized box shape as a collision geometry and the same unit box visual with unit inertias:

Create the box.sdf model file

gedit box.sdf
Copy the following contents into box.sdf:
```html
<?xml version='1.0'?>
<sdf version="1.4">
  <model name="my_model">
    <pose>0 0 0.5 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```
Note that the origin of the Box-geometry is at the geometric center of the box, so in order to have the bottom of the box flush with the ground plane, an origin of <pose>0 0 0.5 0 0 0</pose> is added to raise the box above the ground plane.


#### Step 3: Add to the model SDF file
With a working .sdf file, slowly start adding in more complexity.

Under the &lt;geometry&gt; label, add your .stl model file
```html
<geometry>
        <mesh filename="package://package_name/model_path/model.stl" scale="0.01 0.01 0.01"/>
</geometry>
```

The &lt;geometry&gt; label can be added below &lt;collision&gt; and &lt;visual&gt; label.

### Import the model in you .world file
In your .world file, import the sdf file using &lt; include &gt; tag
```html
<include>
      <uri>model://my_model</uri>
</include>
```

Then open termial to add the model path to Gazebo variable.
```bash
export GAZEBO_MODEL_PATH=~/catkin_ws/src/package_name/model_path/
```
