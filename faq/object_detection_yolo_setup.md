# Setting Up Object Detection Using yolo and darknet_ros
## By Peter Zhao

### Introduction
The FAQ section describes how to integrate YOLO with ros that allows the turtlebot to be able to detect object using its camera. [YOLO](https://arxiv.org/abs/1506.02640) (You only look once) is a widely used computer vision algorith that makes use of convolutional neural networks (CNN) to detect and label an object. Typically before a machine learning algorithm can be used, one needs to train the algorithm with a large data set. Fortunately, YOLO provides many pre-trained model that we can use. For example, yolov2-tiny weight can classify around 80 objects including person, car, bus, bir, cat, dog, and so on.

### darknet_ros
In order to integrate YOLO with ROS easily, one can use this third party package known as [darknet_ros](https://github.com/leggedrobotics/darknet_ros), which uses darknet (a neural network library written in C) to run YOLO and operates as a ros node. The node basically subscribes to the topics that has Image message type, and publishes bounding box information as [BoundingBoxes](https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros_msgs/msg/BoundingBoxes.msg) message type to the /darknet_ros/bounding_boxes topic. 

### How to use darknet_ros
To use darknet_ros, first clone the github repo, and place this directory somewhere inside your catkin_workspace so that when you run catkin_make, this can be built:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/leggedrobotics/darknet_ros
```

Afterward, run catkin_make to build the darknet_ros package
```bash
cd ~/catkin_ws
catkin_make
```

You are now ready to run darknet_ros!

### How to run darknet_ros
darknet_ros can be run aloneside with your other nodes. In your launch file include the following lines

```
<include file="$(find darknet_ros)/launch/darknet_ros.launch">
         <arg name="image" value="rapicam_node/image/raw"/>
</include>
```

Next you need to download the pretrained weight or put the weight you've trained yourself into the darknet_ros/darknet_ros/yolo_network_config/ folder. To download a pretrained weight, follow the insturction given [here](https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros/yolo_network_config/weights/how_to_download_weights.txt). Essentially run the following commands

```
cd DIRECTORY_TO_DARKNET_ROS/darknet_ros/yolo_network_config/weights
wget http://pjreddie.com/media/files/yolov2-tiny.weights
```

Then run this launch file

```
roslaunch [your_package_name] [your_launch_file.launch]
```

You should be able to see the darknet_ros node running with a window that displays the image with bounding boxes placed around objects. If you don't see any GUI window, try to check if the topic passed in in the "image" arg is publishing a valid image.


### Subscribing to darknet_ros's topics
To receive information from the darknet_ros topics, we need to subscribes to the topic it is publishing. Here is an example usage:

```python
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class ObjectRecognizer

    def __init__(self):
        self.boxes = []
        self.box_sub = box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes, self.get_bounding_box_cb())

    def get_bounding_box_cb(self)
        
        def cb(msg: BoundingBoxes):
            self.boxes = msg.bounding_boxes
            for box in self.boxes:
                print("box_class: {}".format(box.Class))
                print("box_x_min: {}".format(box.xmin))
                print("box_x_max: {}".format(box.xmax))
                print("box_y_min: {}".format(box.ymin))
                print("box_y_max: {}".format(box.ymax))
                print()
```

The detailed description of the message types can be found in [darknet_ros/darknet_ros_msg/msg](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros_msgs/msg) folder. 

### Working with CompressedImage
You might have noticed that darknet_ros expects to get raw image from the topic it subscribes to. Sometimes, we may want to use CompressedImage in order to reduce network latency so that the image can be published at a higher frame rate. This can be done by slightly modifying the source code of darknet_ros.

I have forked the original darknet_ros repository and made the modification myself. If you wish to use it, you can simply clone this [repo](https://github.com/zhaoy17/darknet_ros). Now you can modify the launch file to ensure that the darknet_ros subscirbes to a topic that publishes CompressedImage message type:


```
<include file="$(find darknet_ros)/launch/darknet_ros.launch">
         <arg name="image" value="rapicam_node/image/compressed"/>
</include>
```

The changes I've made can be found in darknet_ros/darknet_ros/include/darknet_rosYoloObjectDetector.hpp and darknet_ros/darknet_ros/src/YoloObjectDetector.cpp.

Essentially, you need to change the 

```cpp
void YoloObjectDetector::cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  ROS_DEBUG("[YoloObjectDetector] USB image received.");
```

method to accept msg of type sensor_msgs::ImageConstPtr& instead of sensor_msgs::CompressedImageConstPtr&. You also need to change the corresponding header file so that the method signature matches.