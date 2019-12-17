#!/usr/bin/env python
import os,sys
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 1) # line buffering

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

import rospy
import time
import serial
from std_msgs.msg import String

# define function is called each time the message is published (by some other node)
def callback(msg):
  	print ("\tPI:recieved command: " + str(msg.data))
   	sendCommand(msg.data)

#Command the Arm to do an action given a command
def sendCommand (characterCommand):
	ser.write(characterCommand)
	response=ser.readline()
	#if there was no response, update variable to default message
	if not response:
		response="\tElement did not respond in time"
	serialPub.publish(response)
	print(response)
	print("")

#Initialize the usb connection between the arm and the rasberry pi 
def initSerial():
	#intialize serial connection, updating the global variable
	global ser
	ser=serial.Serial("/dev/ttyUSB0",9600,timeout=3)	
	ser.baudrate=9600
	print("Starting Serial Connection")

	print("PI:Starting Sequence")
	#consume Arduino wellcome message
	print(ser.readline())
	print(ser.readline())
	print(ser.readline())
	

#provides error handling for initSerial method
def initSerialWrapper():
	try:
		initSerial()
	except serial.SerialException as ex:
		print ("USB Not Plugged in")
		time.sleep(5)
		#if the usb is unplugged, wait x seconds and restart connection process
		initSerial()
	except Exception as ex:
		template = "An exception of type {0} occurred. Arguments:\n{1!r}"
		message = template.format(type(ex).__name__, ex.args)
		print message
		time.sleep(5)

# Make this into a ROS node.
rospy.init_node('arminterface')
# Prepare to publish topic `counter` and publish
serialPub = rospy.Publisher('armresponse', String, queue_size=10)
#once here, there are no usb connections and robot can start up
serialPub.publish("First Publish")

#make global keyword 
global ser
#conect the arm to the rasberry pi with error handling
initSerialWrapper()

#subscribe to arm command topic
sub = rospy.Subscriber('armcommand', String, callback)

#spin robot
rospy.spin()