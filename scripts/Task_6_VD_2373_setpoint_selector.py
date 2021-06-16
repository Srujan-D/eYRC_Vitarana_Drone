#!/usr/bin/env python
'''
Team ID        VD#2373
Theme          Vitarana Drone
Author List    Atharva Chandak, Srujan Deolasse, Naitik Khandelwal, Ayush Agrawal
Filename       Task_6_VD_2373_setpoint_selector.py
Functions      check,obstacle_setpoint_callback,setpoint_control_callback
Global Variables None


'''
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
            # Avoiding obs
            pub_msg = self.obs_msg
            
        else:
            # Going to setpoint
            pub_msg = self.setpoint_control_msg

        self.setpoint_pub.publish(pub_msg)

def main():

    selector_obj=Selector()
    r = rospy.Rate(50)  # rate in Hz 
    
    while not rospy.is_shutdown():
        selector_obj.check()
        r.sleep()

if __name__ == "__main__":
    main()
