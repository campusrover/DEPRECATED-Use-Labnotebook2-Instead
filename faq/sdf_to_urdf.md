### Creating a Gazebo Model 

For some users it is easier to create a model through the given Gazebo simulator in the Model Editor than having to write a custom file. The problem that arises is using objects and specific services else where in a project can be difficult due to the extension names of the file. 

### Building  Saving a model 

When you want to buld a model in Gazebo it is best that you do it in the Model Editor that you can get you with CTRl m

Once the model is built and you save it to the right directory it is saved as an .sdf file 

### Why is .sdf not useful for Autonomous Pacman and how it conflicts with Services?

For the sake of the autonomous pacman project, our goal was to implement collectibles that pacman could pick up to make him "invincible". Though ,the gazebo SpawnModel & DeleteModel services expect a .urdf file in order to track the model down in order to Spawn or Delete. 
Example of spawning a custom .urdf is below. 

``` python 
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='TESTCYLINDER',
    model_xml=open('/my_ros_data/catkin_ws/src/ros-autonomous-pacman/models/TESTCYLINDER/TESTCYLINDER.urdf', 'r').read(), 
    initial_pose=Pose(),
    reference_frame='world'
    )
```


### Solution 

After reasearching it was with great pleasure that we found an open source library that allows you put input the name of the .sdf file and then converts it to .urdf as output to where you specify. It is a really straight forward process that can be done by using the following links below 

Github repo - https://github.com/andreasBihlmaier/pysdf
Youtube video instruction - https://www.youtube.com/watch?v=8g5nMxhi_Pw



