#!/usr/bin/env python

'''

PID position controller for the drone
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /drone_command          /edrone/gps
                                /edrone/setpoint

'''

# Importing the required libraries
import time

import rospy
import tf
import math
import threading
from Task_4_VD_2373_utils import *

# from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix,LaserScan
from std_msgs.msg import Float32
from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper



lock=threading.Lock()

class Edrone:
    """docstring for Edrone"""

    def __init__(self):
        
        rospy.init_node("position_controller")  # initializing ros node with name position_controller
       
        # The latitude, longitude and altitude of the drone
        self.drone_position = [0.0, 0.0, 0.0]

        # Format for drone_command
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 0

        
        # Initial settings for the values of Kp, Ki and Kd for roll, pitch and throttle
        self.Kp = [1000000*15, 1000000*15,1000]
        self.Ki = [0, 0, -0.138]
        self.Kd = [10000000*11.5, 10000000*11.5, 2300]
        
        self.parcel_setpoints=[19.0007046575, 71.9998955286, 21]
        self.target=[0,0,0]
        self.subscribed_target=[0,0,0]
        self.last_point=[0,0,0]
        self.roll_setpoint_queue=[]
        self.pitch_setpoint_queue=[]
        self.setpoint_changed=False
        
        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_diff = [0.0, 0.0, 0.0]

        self.rpt = [0.0, 0.0, 0.0]

        # self.scaling_factor=0.0000451704

        
        # minimum and maximum values for roll, pitch and throttle
        self.min_value = [1375, 1375, 1000]
        self.max_value = [1625, 1625, 2000]

        # Sample time in which pid is run. The stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        #  ROS Publishers
        self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
        # self.throttle_pub = rospy.Publisher("/throttle_error", Float32, queue_size=1)
        # self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        # self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        # self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        # self.zero_pub = rospy.Publisher("/zero", Float32, queue_size=1)        

        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/setpoint",destination,self.setpoint_callback)

        # ROS Services 
        # rospy.wait_for_service('/edrone/activate_gripper')

        # ------------------------------------------------------------------------------------------------------------

    def gps_callback(self, msg):
            self.drone_position[0] = msg.latitude
            self.drone_position[1] = msg.longitude
            self.drone_position[2] = msg.altitude


    def setpoint_callback(self,msg):
        if msg.lat and msg.long and msg.alt:
            if self.subscribed_target[0] != msg.lat or self.subscribed_target[1] != msg.long or self.subscribed_target[2] != msg.alt:
                self.setpoint_changed=True
                print('setpoint_changed')
                self.roll_setpoint_queue=[]
                self.pitch_setpoint_queue=[]
            else:
                self.setpoint_changed=False
                

            self.subscribed_target[0] = msg.lat
            self.subscribed_target[1] = msg.long
            self.subscribed_target[2] = msg.alt

            self.target[2]=self.subscribed_target[2]        

    # if you use this for control, you may have to change the relevant pitch   direction because of the sign
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)
    
    def x_to_lat(self, input_x):
        return input_x/110692.0702932625 + 19

    def y_to_long(self, input_y):
        return -input_y/(105292.0089353767 )+ 72

    def get_current_coords(self):
        return (self.lat_to_x(self.drone_position[0]),self.long_to_y(self.drone_position[1]),self.drone_position[2])


    # TODO: use x,y values and get coordinates of a point 10 meters towards the destination direction
    def create_next_setpoint(self):
                 
        # err=list(self.target)
        # if err[1]!=0:
        #     tan_theta = abs(err[0]/err[1])  #TODO:remove abs() here
        #     # if err[0]>err[1]:
        #     if abs(err[0])>=self.scaling_factor:
        #         if abs(err[1])>self.scaling_factor and abs(tan_theta)<1:
        #             err[1]=self.scaling_factor if err[1]>0 else -self.scaling_factor
        #             err[0]=self.scaling_factor*tan_theta
        #         else:
        #             err[0]=self.scaling_factor if err[0]>0 else -self.scaling_factor
        #             err[1]=self.scaling_factor/tan_theta

        #     elif abs(err[1])>=self.scaling_factor:
        #         err[1]=self.scaling_factor if err[1]>0 else -self.scaling_factor
        #         err[0]=self.scaling_factor*tan_theta
        #     self.error=list(err)
        if abs(self.error[0])>0.0000451704:
            self.error[0]= 0.0000451704 if self.error[0]>0 else -0.0000451704 

        if abs(self.error[1])>0.000047487:
            self.error[1]= 0.000047487 if self.error[1]>0 else -0.000047487 
    
    def check_proximity(self):

        if( len(self.roll_setpoint_queue)>0 and abs(self.drone_position[0] - self.roll_setpoint_queue[0])<= 0.000004517): #0.000004517
            self.roll_setpoint_queue.pop(0)

        if( len(self.pitch_setpoint_queue)>0 and abs(self.drone_position[1] - self.pitch_setpoint_queue[0])<= 0.0000047487): #0.000004517
            self.pitch_setpoint_queue.pop(0)


    def pid(self,select_rpt,):
        # self.target = list(target_point)       
        # Calculating the error
        if self.setpoint_changed and select_rpt!=2:
            # self.roll_setpoint_queue=[]
            # self.pitch_setpoint_queue=[]
            print("generating...")
            err= ( self.subscribed_target[select_rpt] - self.drone_position[select_rpt] )
            print(select_rpt,err)
            print("int(err/0.0000451704)",int(err/0.0000451704))
            if select_rpt==0 and abs(err)>0.0000451704:
                print("Creating roll path")
                for i in range(1,1+int(abs(err)/0.0000451704)):
                    self.roll_setpoint_queue.append(self.drone_position[0]+i*0.0000451704 if err>0 else self.drone_position[0]-i*0.0000451704 )
                self.target[0]=err%0.0000451704

            elif select_rpt==1 and abs(err)>0.000047487:
                print("Creating pitch path")
                for i in range(1,1+int(abs(err)/0.000047487)):
                    self.pitch_setpoint_queue.append(self.drone_position[1]+i*0.000047487 if err>0 else self.drone_position[1]-i*0.000047487 ) 
                self.target[1]=err%0.000047487
                self.setpoint_changed = False #Backup if self.setpoint_changed is published late
        
        if select_rpt==0:
            if len(self.roll_setpoint_queue)!=0:
                self.target[0]=self.roll_setpoint_queue[0]
            else:
                self.target[0]=self.subscribed_target[0]

        if select_rpt==1:
            if len(self.pitch_setpoint_queue)!=0:
                self.target[1]=self.pitch_setpoint_queue[0]
            else:
                self.target[1]=self.subscribed_target[1]
        
        self.check_proximity()

        self.error[select_rpt] = ( self.target[select_rpt] - self.drone_position[select_rpt] )

        if not select_rpt:
            print("")
            print("Target",self.target)
            # print("Drone_pos",self.drone_position)
            print("errrrrrr",self.error)
            print('roll_setpoint_queue',self.roll_setpoint_queue)
            print('pitch_setpoint_queue',self.pitch_setpoint_queue)
        # self.create_next_setpoint()

        #     print("")

        self.error_sum[select_rpt] = self.error_sum[select_rpt] + self.error[select_rpt]
        self.error_diff[select_rpt] = (self.error[select_rpt] - self.prev_error[select_rpt])


        # Calculating pid values
        self.rpt[select_rpt] = (
            (self.Kp[select_rpt] * self.error[select_rpt])
            + (self.Ki[select_rpt] * self.error_sum[select_rpt]) *
            self.sample_time
            + (self.Kd[select_rpt] *
            (self.error_diff[select_rpt]) / self.sample_time)
        )

        # Changing the previous error values
        self.prev_error[select_rpt] = self.error[select_rpt]

        #------------------------------------------------#
        self.cmd_drone.rcRoll = 1500 + self.rpt[0]
        self.cmd_drone.rcPitch = 1500 + self.rpt[1]
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 1500 + self.rpt[2]

        self.cmd_drone.rcRoll = limit_value(
            self.cmd_drone.rcRoll, self.min_value[0], self.max_value[0]
        )
        self.cmd_drone.rcPitch = limit_value(
            self.cmd_drone.rcPitch, self.min_value[1], self.max_value[1]
        )
        self.cmd_drone.rcThrottle = limit_value(
            self.cmd_drone.rcThrottle, self.min_value[2], self.max_value[2]
        )

        # limiting the values
        self.cmd_drone.rcRoll = limit_value(
            self.cmd_drone.rcRoll, self.min_value[0], self.max_value[0]
        )
        self.cmd_drone.rcPitch = limit_value(
            self.cmd_drone.rcPitch, self.min_value[1], self.max_value[1]
        )
        self.cmd_drone.rcThrottle = limit_value(
            self.cmd_drone.rcThrottle, self.min_value[2], self.max_value[2]
        )

        self.cmd_pub.publish(self.cmd_drone)

        # self.throttle_pub.publish(self.error[2])
        # self.roll_pub.publish(self.error[0])
        # self.pitch_pub.publish(self.error[1])
        # self.zero_pub.publish(0)


        if (self.drone_position[0]-self.last_point[0]>=0.000001) or (self.drone_position[1]-self.last_point[1]>=0.000001) or  (self.drone_position[2]-self.last_point[2]>=0.02): 
            self.last_point=list(self.drone_position)


if __name__ == "__main__":

    e_drone = Edrone()
    r = rospy.Rate(50)  # rate in Hz 

    while not rospy.is_shutdown():

        if all(e_drone.drone_position):# and not e_drone.obstacle_detected_bottom and not  e_drone.obstacle_detected_top :
            print(time.strftime("%H:%M:%S"))
            e_drone.pid(0)
            e_drone.pid(1)
            e_drone.pid(2)

        r.sleep()
