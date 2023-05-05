# Some tips for using OpenCV and Turtlebot3 Camera
## Read CompressedImage type
When using Turtlebot in the lab, it only publishs the CompressedImage from '/raspicam_node/image/compressed'. Here's how to read CompressedImage and Raw if you need.
~~~Python
from cv_bridge import CvBridge
def __init__(self):
   self.bridge = CvBridge()
def image_callback(self, msg):
        # get raw image
        # image = self.bridge.imgmsg_to_cv2(msg)
        
        # get compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(img_np, cv2.COLOR_BGR2HSV)
~~~

## Limit frame rate 
When the computation resource and the bandwidtn of robot are restricted, you need limit the frame rate.

~~~Python
def __init__(self):
   self.counter = 1
def image_callback(self, msg):
        # set frame rate 1/3
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
~~~

## Republish the image
Again, when you have multiple nodes need image from robot, you can republish it to save bandwidth.

~~~Python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def image_callback(msg):
    # Convert the compressed image message to a cv2 image
    bridge = CvBridge()
    cv2_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Process the image (e.g., resize, blur, etc.)
    processed_image = process_image(cv2_image)

    # Convert the processed cv2 image back to a compressed image message
    compressed_image_msg = bridge.cv2_to_compressed_imgmsg(processed_image)

    # Publish the processed image on the new topic
    processed_image_pub.publish(compressed_image_msg)

def process_image(cv2_image):
    # Perform image processing (e.g., resize, blur, etc.) and return the processed image
    # Example: resized_image = cv2.resize(cv2_image, (new_width, new_height))
    return cv2_image

if __name__ == '__main__':
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/robot/camera_topic', CompressedImage, image_callback)
    processed_image_pub = rospy.Publisher('/processed_image_topic', CompressedImage, queue_size=1)

    rospy.spin()
~~~





