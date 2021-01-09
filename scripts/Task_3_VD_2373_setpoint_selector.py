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

        self.setpoint_control_msg = destination()
                
        self.setpoint_pub = rospy.Publisher("/edrone/setpoint", destination, queue_size=2)

        rospy.Subscriber("/edrone/obstacle_setpoint",destination,self.obstacle_setpoint_callback)
        rospy.Subscriber("/edrone/setpoint_control",destination,self.setpoint_control_callback)


    def obstacle_setpoint_callback(self,msg):

        self.obs_msg = msg


    def setpoint_control_callback(self,msg):
        
        self.setpoint_control_msg = msg 


    def check(self):
        
        pub_msg=destination()
        
        if self.obs_msg.obstacle_detected:
            print("avoiding obs")
            pub_msg = self.obs_msg
            
        else:
            pub_msg = self.setpoint_control_msg
            print("Going to setpoint",pub_msg)

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
