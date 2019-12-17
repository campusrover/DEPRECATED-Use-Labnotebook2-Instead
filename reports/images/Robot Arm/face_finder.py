#!/usr/bin/env python

#importing Python labraies
import face_recognition
import cv2
import numpy as np
import rospy
#importing ROS labraies and messesge/data type
from sensor_msgs.msg import Image, CameraInfo,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# prints to console that the node has begun the start up proccess
print'start up has begun'

# Loading facesfrom jpg files to store them
# find the faces in the pictures uploaded and encode the faces
luis_image = face_recognition.load_image_file("face_pictures/luis.jpg")
luis_face_encoding = face_recognition.face_encodings(luis_image)[0]

pito_image = face_recognition.load_image_file("face_pictures/pito.jpg")
pito_face_encoding = face_recognition.face_encodings(pito_image)[0]

# Declaring global lists
global known_face_encodings
global known_face_names
global face_names
frame_counter = 1


#add face encoding to a list of all face = encodings
known_face_encodings = [
    luis_face_encoding,
    pito_face_encoding,
]
#have the names of the people corispound to the face encodings
known_face_names = [
    "luis andino",
    "pito",
]
#declaring lists
face_locations = []
face_encodings = []
face_names = []


# converting ros published image to an open cv format
# We are also flipping the image becouse the img published is upside down
def convert_from_ros(img):
    try:
        # create the cvBridge object
        bridge = CvBridge()
        # Convert the compressed ros image to open CV rgb8
        cv_image = bridge.compressed_imgmsg_to_cv2(img, "rgb8")
        # rotate the the image
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        return cv_image
    except CvBridgeError as e:
        print(e)

def callback(data):

    global frame_counter
    #process 1 every 6 frames
    if frame_counter == 6:
        frame_counter = 0
        # Get converted Open_CV image
        frame = convert_from_ros(data)
        # Resizes the Image so it os easir to processed
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # Takes the small image and the face locations list amd fines all the faces
        # In the image
        face_locations = face_recognition.face_locations(small_frame)
        face_encodings = face_recognition.face_encodings(small_frame, face_locations)
        # if face_encoding array is empty no faces were detected
        if len(face_encodings) == 0:
            # publish message for any node listening
            pub.publish('cant_find_face')
        #  if there are faces in the image
        else:
            # iterate all the face encoding found
            for face_encoding in face_encodings:
                #test to see if faces where found in the img that it recognizes
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"
                # gets the face distances between each faces to latter draw bounding boz
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                # gets the index of the best matches
                best_match_index = np.argmin(face_distances)
                # if there is a name that matches the faces
                if matches[best_match_index]:
                    # Sets name to one in the face index
                    name = known_face_names[best_match_index]
                # Publish the name over the ros Node
                pub.publish(name)
                print name

    else:
        # adds 1 to the frame counter 
        frame_counter = frame_counter +1

    # Sleep for  sencound
    rate_image_processsing.sleep()

# Creates the ROS Node
rospy.init_node('face_recognition_v2')
# Sets publisher variable with the message being a String
pub = rospy.Publisher('face_recognition', String, queue_size=1)
# Subscribesto the Raspicam on the robots compressed image
image_sub = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, callback)
# Sets the rate to .75
rate_image_processsing  = rospy.Rate(2)
# prints to console that the node has finished the setup
print 'start up has finished'

rospy.spin()
