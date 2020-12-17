### Author: Mahima Devanahalli

### Creating a Gazebo World 

To create a gazebo world you can first create new models on gazebo, mannually and save those as sdf files. You can also use premade gazebo models. Once you do this, you can make a world file which incorporates all the models you need. 

### What is the difference between a .sdf file and .world file?

Both these files share the same format. However, the .sdf files define models that you can create in gazebo. When these models are in their own file, they can be reused in multiple worlds. A .world file describes an entire scene with objects and various models. You can copy the contents of an sdf file into your world file to incorporate that model into your world. 

When you create your own models on gazebo and save them, they automatically save as sdf files which can then be used as mentioned above. 
To do this:

Open an empty world in gazebo
Click edit -> building editor or model editor -> then save it in your desired location

### Helpful Links

https://www.youtube.com/watch?v=3YhW04wIjEc
http://gazebosim.org/tutorials?cat=build_world
https://nlamprian.me/blog/software/ros/2019/10/06/gazebo-virtual-worlds/

