#!/usr/bin/env python

from setpoint_control import SetpointControl
import rospy
import cv2,os,math
import numpy as np
from vitarana_drone.msg import *
from sensor_msgs.msg import Image,LaserScan,NavSatFix
from std_msgs.msg import Float32
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

class MarkerDetect():

    def __init__(self):

        rospy.init_node("marker_detect")

        cc_path='/data/cascade.xml'
        abs_cc_path=os.path.dirname(__file__)+cc_path
        self.logo_cascade = cv2.CascadeClassifier(abs_cc_path)
        
        self.bottom_sensor_dist=None

        self.img=None
        self.bridge = CvBridge()

        self.img_width=400
        self.hfov_rad=1.3962634
        self.focal_length = (self.img_width/2)/math.tan(self.hfov_rad/2)
       
        self.target=[0,0,0]
        self.drone_position=[0,0,0]
        self.pub_center_x_y = center_x_y()
        
        self.center_x_y=rospy.Publisher("/edrone/center_lat_long", center_x_y, queue_size=1)
      
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
            return True
        else:
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
                print("detected")
                self.get_coords_from_img(logo)
                plt.show()
            else:
                print("-")
                # self.pub_center_x_y = center_x_y()            
            

    def get_coords_from_img(self,rect):
        
        self.centre_x_pixel=rect[0][0]+rect[0][2]/2
        self.pub_center_x_y.x=self.centre_x_pixel*self.bottom_sensor_dist/self.focal_length

        self.centre_y_pixel=rect[0][1]+rect[0][3]/2
        self.pub_center_x_y.y=self.centre_y_pixel*self.bottom_sensor_dist/self.focal_length
        
    
    def detect(self):
        self.center_x_y.publish(self.pub_center_x_y)

        
        

def main():
    marker=MarkerDetect()
    r=rospy.Rate(1)
    while not rospy.is_shutdown():
        marker.detect()
        r.sleep()

if __name__ == "__main__":
    main()

# rosrun vitarana_drone marker_detect.py
# [[ 28 328  51  51]]