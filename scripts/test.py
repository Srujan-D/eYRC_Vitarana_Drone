#!/usr/bin/env python

'''
File for experienting / testing certain stuff
'''

import rospy
from vitarana_drone.srv import Gripper
from vitarana_drone.msg import *
# from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
import tf

class Test():

    def __init__(self):
        rospy.init_node('test')
        self.drone_position=[0,0,0]
        self.drone_orientation_euler=[0,0,0]
        self.drone_orientation_quaternion=[0,0,0,0]
        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber("/drone_command", edrone_cmd, self.drone_command_callback)
        rospy.Subscriber("/edrone/imu/data", Imu, self.imu_callback)
        # rospy.Subscriber("/edrone/setpoint",destination,self.setpoint_callback)

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w


    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    # def leave(self):
    #     print("drop")
    #     res=self.gripper(False)
    #     print("dropped")
    #     print("res",res)

    def drone_command_callback(self):
        drone_pub=edrone_cmd()
         #TODO:yaw publish

if __name__ == "__main__":
    test = Test()
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        (
            test.drone_orientation_euler[1],
            test.drone_orientation_euler[0],
            test.drone_orientation_euler[2],
        ) = tf.transformations.euler_from_quaternion(
            [
                test.drone_orientation_quaternion[0],
                test.drone_orientation_quaternion[1],
                test.drone_orientation_quaternion[2],
                test.drone_orientation_quaternion[3],
            ]
        )
        print(test.drone_orientation_euler[2])
        r.sleep()