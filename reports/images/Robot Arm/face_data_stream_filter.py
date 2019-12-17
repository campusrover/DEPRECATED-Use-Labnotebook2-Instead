#!/usr/bin/env python

#importing Python labraies
import numpy as np
import rospy
from std_msgs.msg import String

frame_counter = 1
list_of_names  = []

def callback(name):
    
    global get_name
    global frame_counter
    get_name = name
    list_of_names.append(name)
    frame_counter = + frame_counter + 1

print 'start up has begun in the node'
rospy.init_node('face_recognition_v2')
face_finder = rospy.Subscriber("face_recognition", String, callback)
pub = rospy.Publisher('face_recognition/data_clean_up', String, queue_size=1)

rate = rospy.Rate(.1)
while not rospy.is_shutdown():
    if frame_counter >= 10:
        frame_counter = 0
        if get_name != 'cant_find_face':
            pub.publish(get_name)

        print get_name, frame_counter
        rate.sleep()













