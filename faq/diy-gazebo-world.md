# Building Your Own Custom Gazebo World with DIY models from Blender
### by Al Colon

**FOR YOUR CONSIDERATION:**
This tutorial is for .dae files. It would probably work with .stl or .obj files but since I was using Blender, it was the best mesh format as it was really easy to transfer, update and adjust. It also includes material within itself so you don't have to worry about adding colors through gazebo. If you are looking for a solid, rigid object, I definitely recommend using Blender as it is very flexible. There are plenty of tutorials to make pretty much any shape so be creative with your inquiries and you should be able to create whatever you need.

## Steps!
### Step 1! Creating your model

Making a model can be difficult if you are not sure where to start. My advice is to go as simple as possible. Not only will it make gazebo simulation simpler, it will save you time when you inevitable need to make changes.

Make sure that the object you are designing looks exactly like you want it to in Gazebo. Obviously, this speaks to overall design, but almost more important than that, make sure that your orientation is correct. This means making sure that your model is centralized the way you want it on the x, y and *z–axis* Trying to update the pose though code never worked and it is one adjustment away through editing your model. Note: you can add collisions but if you want to create a surface for the robots to drive on, it will be easier to just make it as thin as possible.

### Step 2! Importing your model
>*Learning from my mistakes*
>It might be tempting to use the Add function via the Custom Shapes but this was probably the most frustrating part of my project. What I suggest, as it would have save me weeks of work, it would behoove you to add the mesh (the 3d model files) to a world file so that you can get you measurement and adjustments.

The best tutorial that I found was http://gazebosim.org/tutorials/?tut=import_mesh It is fairly simple and the most reliable way to get your model to show up. From here, however, save a new world from this file as it will have a more complete .world file. Robots should be launched from roslaunch but double check how they interact by adding the model via the Insert tab. It will be white and more boxy but it will help you with positioning, interation and scale.

### Step 3! Setup your files
>It is important to be incredibly vigilant while doing this part as you will be editing some files in the gazebo-9 folder. You do not want to accidental delete or edit the wrong file.

Using one of the models in gazebo-9 or my_ros_data/.gazebo/models as a template, change the information in the model.sdf and model.config files. The hierarchy should look something like:

```xml
<your_model_name>
    <material>
        <material1.png>
        <material2.png>
        <etc...>
    <meshes>
        <your_model.dae>
    <model.sdf>
    <model.config>
```
According to your model, you might need to edit more but for me, it sufficed to edit the mesh uri with my dae file, update the .sdf and .config files and add png versions of the colors i used in blender.

In order for Gazebo to open files through roslaunch, they must be in the proper spot. You can double check where exactly Gazebo is looking by checking the $GAZEBO_RESOURCE_PATH and $GAZEBO_MODEL_PATH in the setup.sh in the gazebo-9 folder. It will probably be something like usr/share/gazebo-9 but just **copy and paste** the entire file into this folder !! after !! you !! have !! confirmed !! that !! it !! works !! in !! your !! world !! file !! *(To check without a roslaunch, navigate to the directory that holds your .world file AND .dae file and in the terminal $ gazebo your_world.world)*

You can also add your world file to the ~/gazebo-9/worlds.

### Step 3! ROSLAUNCH
Launch files are pretty straightforward actually. Having an understanding about how args work is really key and a quick google search will probably get a better explanation for your needs. You can also take a gander at launch files you have already used for other assignments.

**KEY**
>1. Make sure the launch file is looking in the right places for your models.
>
>2. Debug as often as you can
>
>3. Reference other launch files to know what you need for your file.
>4. Update arg fields as needed





















# Getting the namespace


# References
