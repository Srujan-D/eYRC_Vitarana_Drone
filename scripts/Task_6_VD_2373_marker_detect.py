#!/usr/bin/env python
'''
Team ID        VD#2373
Theme          Vitarana Drone
Author List    Atharva Chandak, Srujan Deolasse, Naitik Khandelwal, Ayush Agrawal
Filename       Task_6_VD_2373_marker_detector.py
Functions      range_finder_bottom_callback,gps_callback,setpoint_callback,check_proximity_setpoint,marker_detect_callback,get_coords_from_img,detect,main
Global Variables     none
'''
import cv2,os
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

import rospy
from vitarana_drone.msg import *
from sensor_msgs.msg import Image,LaserScan,NavSatFix
from std_msgs.msg import Float32

class MarkerDetect():

    def __init__(self):

        rospy.init_node("marker_detect")

        cc_path='/data/cascade.xml'
        abs_cc_path=os.path.dirname(__file__)+cc_path
        self.logo_cascade = cv2.CascadeClassifier(abs_cc_path)
        
        self.bottom_sensor_dist=None

        self.img=None
        self.bridge = CvBridge()
       
        self.iterations = 0
        self.target=[0,0,0]
        self.drone_position=[0,0,0]
        self.pub_center_pixels = center_x_y()
        
        self.center_x_y = rospy.Publisher("/edrone/center_lat_long", center_x_y, queue_size=1)
      
        rospy.Subscriber("/edrone/camera/image_raw", Image, self.marker_detect_callback) #Subscribing to the camera topic
        rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.range_finder_bottom_callback)
        rospy.Subscriber("/edrone/setpoint",destination,self.setpoint_callback)
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)


    def range_finder_bottom_callback(self,msg):
        self.bottom_sensor_dist=msg.ranges[0]


    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude



    def setpoint_callback(self,msg):
        self.target[0] = msg.lat
        self.target[1] = msg.long
        self.target[2] = msg.alt
        
    def check_proximity_setpoint(self,target):
        if (
            (
                abs(self.drone_position[0] - target[0])
                <= 0.0000018 #0.000004517
            )
            and (
                abs(self.drone_position[1] - target[1])
                <= 0.0000019 #0.0000047487
            )
            and (
                abs(self.drone_position[2] - target[2])
                <= 0.2
            )
        ):
            if self.iterations>=5:
                self.iterations=0
                return True
            else:
                self.iterations+=1
                return False
        else:
            self.iterations = 0
            return False


    def marker_detect_callback(self,msg):
        if self.check_proximity_setpoint(self.target):
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image

            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
            print(logo)
            
            for (x, y, w, h) in logo:
                cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            
            plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            
            if len(logo)!=0:
                self.get_coords_from_img(logo)
                # plt.show()
            else:
                pass
                # self.pub_center_pixels = center_x_y()            
            

    def get_coords_from_img(self,rect):
        
        self.centre_x_pixel=rect[0][0]+rect[0][2]/2
        self.pub_center_pixels.x=self.centre_x_pixel

        self.centre_y_pixel=rect[0][1]+rect[0][3]/2
        self.pub_center_pixels.y=self.centre_y_pixel

        self.pub_center_pixels.square_size=(rect[0][2]+rect[0][3])/2
        
    
    def detect(self):
        print(self.pub_center_pixels)
        self.center_x_y.publish(self.pub_center_pixels)

    def reset(self):
        self.center_x_y.publish(center_x_y())
        

def main():
    marker=MarkerDetect()
    r=rospy.Rate(1)
    rospy.on_shutdown(marker.reset)
    while not rospy.is_shutdown():
        marker.detect()
        r.sleep()

if __name__ == "__main__":
    main()