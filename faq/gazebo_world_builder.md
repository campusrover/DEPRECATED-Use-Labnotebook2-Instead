# Creating Gazebo world from scratch

### Muthhukumar Malaiiyyappan (Malai)


Building a gazebo world might be a little daunting of a task when you are getting started. One might want to edit existing gazebo worlds but I will save you the trouble and state that its not going to work.

1. Open a vnc terminal

```
gazebo
```
This would open a gazebo terminal and once you get in there you would want to bring your cursor to the top left hand corner and find the Building Editor in the Edit tab.

Once in the building editor click on the wall to create the boundaries on the top half of the editor. Left click to get out of the building mode. If you would like to create walls without standard increments, press shift while dragging the wall. 

If you would like to increase or decrease the thickness of the wall. Click on the walls you would like to change and it will open up a modal with options to change. 

After you are satisfied with their boundaries, save it as a model into your models folder within your project.

When the models have been saved. You would be brought back to gazebo with the model that you have built.

2. Change the pose of the model if need be according to your needs then save the world.

3. Upload your new model into this part of the launch file

```
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find {project_name})/worlds/{world_name}.world"/>

        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
    </include>
```

If you find that your robot is not in the right place. Open the launch file, make the changes to the model accordingly and save it again as the world again.


Thats how you can build a world from scratch, hope this helped. 