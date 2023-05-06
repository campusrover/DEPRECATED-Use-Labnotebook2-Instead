# Creating and Executing Launch Files
by Helen Lin
edited by Jeremy Huey

## Introduction

This guide shows you how to create launch files for your code so that you can launch multiple nodes at once rather than running each node individually.

## Step 1

Open a new terminal window and navigate to a package you want to create a launch file in. Create a folder called 'launch'.

```mkdir launch```

## Step 2

Navigate to the launch directory and create a new launch file ending in .launch. Replace 'name' with the name of your launch file.

```touch name.launch```

## Step 3

Add the following code and fill in the parameters within the double quotes.

```
<launch>
  <node name=" " pkg=" " type=" " output=" "/>
  <node name=" " pkg=" " type=" " output=" "/>
</launch>
```

Here is an example of what the node will look like filled in, using code from the Mini Scouter project:

```xml
<launch>
  <node name="MiniScoutMain" pkg="mini_scouter" type="MiniScoutMain.py" output="screen"></node>
  <node name="laser_scan" pkg="mini_scouter" type="scan_arouund.py" output="screen"></node>
</launch>
```
The pkg name can be found in the package.xml. Eg. 
```xml
<?xml version="1.0"?>
<package format="2">
  <name>object_sorter</name>
  <version>0.0.0</version>
```

## Step 4

Make sure you have run the following command on all of the files used in the launch file so they can all be found by ROS launch. Replace 'name' with the name of the python file.

```chmod +x name.py```

Change the permissions of the launch file as well by going to the launch directory and running the following command. Replace 'name' with the name of the launch file.

```chmmod +x name.launch```

## Step 5

Open a new terminal window and run the following command. Replace 'package_name' with the name of the package and 'name' with the name of the launch file.

```roslaunch package_name name.launch```

For example, to run the Mini Scouter launch file:

```roslaunch mini_scouter teleop.launch```

All of the nodes you specified in the launch file should now be running.

### Optional, launch process in new window
To have a node launch and open in a new window, such as to run things like key_publisher.py, you can modify the line to include this: 
```<node pkg="object_sorter" type="key_publisher.py" name="key" output="screen" launch-prefix="xterm -e"/>```
You must then run in terminal: 
```bash
sudo apt-get install xterm
```

### More info on launch files
To continue to get more information on launch files, go to here: 
labnotebook/faq/launch-files.md
