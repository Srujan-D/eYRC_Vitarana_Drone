#!/usr/bin/env python

'''
File for experienting / testing certain stuff
'''

import rospy
from vitarana_drone.srv import Gripper
from vitarana_drone.msg import *

class Test():

    def __init__(self):
        rospy.init_node('test')
        # self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper())
        #TODO:yaw publish
        self.yaw_pub= rospy.Publisher('/drone_command', edrone_cmd, self.drone_command_callback)

    # def leave(self):
    #     print("drop")
    #     res=self.gripper(False)
    #     print("dropped")
    #     print("res",res)

    def drone_command_callback(self):
        drone_pub=edrone_cmd()
         #TODO:yaw publish

if __name__ == "__main__":
    image_proc_obj = Test()
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        image_proc_obj.leave()
        r.sleep()