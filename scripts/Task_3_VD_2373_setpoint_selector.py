#!/usr/bin/env python

import rospy
import math
import time
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu, NavSatFix,LaserScan
from vitarana_drone.srv import Gripper

class Selector():
    def __init__(self):

        rospy.init_node('setpoint_selector')

        self.obs_msg = destination()

        # self.obs_msg.lat=0
        # self.obs_msg.long=0
        # self.obs_msg.alt=0
        # self.obs_msg.obstacle_detected=False


        self.setpoint_control_msg = destination()
        
        # self.setpoint_control_msg.lat=0
        # self.setpoint_control_msg.long=0
        # self.setpoint_control_msg.alt=0
        # self.setpoint_control_msg.obstacle_detected=False

        
        self.setpoint_pub = rospy.Publisher("/edrone/setpoint", destination, queue_size=2)


        rospy.Subscriber("/edrone/obstacle_setpoint",destination,self.obstacle_setpoint_callback)
        rospy.Subscriber("/edrone/setpoint_control",destination,self.setpoint_control_callback)

    def obstacle_setpoint_callback(self,msg):
        
        self.obs_msg = msg

        # self.obs_msg_lat=msg.lat
        # self.obs_msg_long=msg.long
        # self.obs_msg_alt=msg.alt
        # self.obs_msg_obstacle_detected=msg.obstacle_detected


    def setpoint_control_callback(self,msg):
        
        self.setpoint_control_msg = msg 
        
        # self.setpoint_control_msg.lat=msg.lat
        # self.setpoint_control_msg.long=msg.long
        # self.setpoint_control_msg.alt=msg.alt
        # self.setpoint_control_msg.obstacle_detected=msg.obstacle_detected


    def check(self):
        
        pub_msg=destination()
        
        if self.obs_msg.obstacle_detected:
            print("avoiding obs")
            pub_msg = self.obs_msg

            # pub_msg.lat=self.obs_msg.lat
            # pub_msg.long=self.obs_msg.long
            # pub_msg.alt=self.obs_msg.alt
            # pub_msg.obstacle_detected=self.ob._msg.obstacle_detected
            
        else:
            
            pub_msg = self.setpoint_control_msg
            print("Going to setpoint",pub_msg)

            # pub_msg.lat=self.setpoint_control_msg.lat
            # pub_msg.long=self.setpoint_control_msg.long
            # pub_msg.alt=self.setpoint_control_msg.alt
            # pub_msg.obstacle_detected=self.setpoint_control_msg.obstacle_detected
        
        self.setpoint_pub.publish(pub_msg)



        # if self.setpoint[0]==self.parcel_setpoint[0] and self.setpoint[1]==self.parcel_setpoint[1] and self.setpoint[2]==self.parcel_setpoint[2]:
        #     self.dest_msg.landing=True
        #     print("Landing")
        # else:
        #     self.dest_msg.landing=False

def main():

    selector_obj=Selector()
    r = rospy.Rate(50)  # rate in Hz 
    
    while not rospy.is_shutdown():
        selector_obj.check()
        r.sleep()

if __name__ == "__main__":
    main()
