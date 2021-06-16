#!/usr/bin/env python
'''

Team ID        VD#2373
Theme          Vitarana Drone
Author List    Atharva Chandak, Srujan Deolasse, Naitik Khandelwal, Ayush Agrawal
Filename       Task_6_VD_2373_qr_detect.py
Functions      rgb2gray,image_callback
Global Variables    None

'''

'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from vitarana_drone.msg import *
from pyzbar.pyzbar import decode

class image_proc():

	# Initialise everything
	def __init__(self):

		rospy.init_node('scanned_barcode') #Initialise rosnode
		self.scanned_coords= qr_scanner()
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.qr_publisher=rospy.Publisher("/edrone/qr_scanner",qr_scanner,queue_size=1)
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

	def rgb2gray(self,rgb):
	    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

	# Callback function of amera topic
	def image_callback(self, data):
		
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.img = self.rgb2gray(self.img)
			out=decode(self.img)
			if len(out)!=0:
				print(out)
				try:
					self.destination_setpoint=map(float,out[0][0].split(','))
					print(self.destination_setpoint)
					self.destination_setpoint=list(self.destination_setpoint)
					self.scanned_coords.lat_x=self.destination_setpoint[0]
					self.scanned_coords.long_y=self.destination_setpoint[1]
					self.scanned_coords.alt_z=self.destination_setpoint[2]
				except Exception as e:
					print(e)
					
		except CvBridgeError as e:
			print(e)
			return

def main():
	image_proc_obj = image_proc()
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		image_proc_obj.qr_publisher.publish(image_proc_obj.scanned_coords)
		r.sleep()

if __name__ == "__main__":
    main()
