#!/usr/bin/env python

'''
File for experienting / testing certain stuff
'''

import rospy
from vitarana_drone.srv import Gripper

class Test():

    def __init__(self):
        rospy.init_node('test')
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper())

    def leave(self):
        print("drop")
        res=self.gripper(False)
        print("dropped")
        print("res",res)
        

if __name__ == "__main__":
    image_proc_obj = Test()
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        image_proc_obj.leave()
        r.sleep()