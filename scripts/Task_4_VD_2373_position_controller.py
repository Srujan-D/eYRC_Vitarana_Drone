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

# util functions
from Task_4_VD_2373_utils import *

# from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix,LaserScan
from std_msgs.msg import Float32
from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper


# util functions
from Task_4_VD_2373_utils import *

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
        self.Kp = [ 65, 65, 1000]
        self.Ki = [0, 0, -0.138]
        self.Kd = [ 550, 550, 2300 ]       


        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.target=[0,0,0]
        self.subscribed_target=[0,0,0]
        self.last_point=[0,0,0]
        self.roll_setpoint_queue=[]
        self.pitch_setpoint_queue=[]
        self.rp_queue_max_length=0
        self.setpoint_changed=False
        self.created_next_setpoints=False
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_diff = [0.0, 0.0, 0.0]

        self.rpt = [0.0, 0.0, 0.0]

        # self.scaling_factor=0.0000451704
        self.min_lat_limit =  0.0000451704 * 2
        self.min_long_limit = 0.000047487  * 2
        
        # minimum and maximum values for roll, pitch and throttle
        self.min_value = [1000, 1000, 1000]
        self.max_value = [2000, 2000, 2000]

        # Sample time in which pid is run. The stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds
        # self.obs_tuning=False


        #  ROS Publishers
        self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
        # self.throttle_pub = rospy.Publisher("/throttle_error", Float32, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.zero_pub = rospy.Publisher("/zero", Float32, queue_size=1)        

        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/setpoint",destination,self.setpoint_callback)
        # rospy.Subscriber("/edrone/range_finder_top",LaserScan,self.range_finder_top_callback)

        # ROS Services 
        # rospy.wait_for_service('/edrone/activate_gripper')

        # ------------------------------------------------------------------------------------------------------------

    def gps_callback(self, msg):
            self.drone_position[0] = msg.latitude
            self.drone_position[1] = msg.longitude
            self.drone_position[2] = msg.altitude

    # def range_finder_top_callback(self,msg):
    #     if any((dist<=25 and dist>0.5) for dist in msg.ranges):
    #         self.obs_tuning=True
    #         print()
    #     else:
    #         self.obs_tuning=False

    def setpoint_callback(self,msg):
        if msg.lat!=0 and msg.long!=0 and msg.alt!=0 and all(self.drone_position):
            if (self.subscribed_target[0] != msg.lat or self.subscribed_target[1] != msg.long or self.subscribed_target[2] != msg.alt) :
                # self.setpoint_changed=True
                print('setpoint_changed')
                print(msg)
                self.subscribed_target[0] = msg.lat
                self.subscribed_target[1] = msg.long
                self.subscribed_target[2] = msg.alt
                self.obs = msg.obstacle_detected
                self.roll_setpoint_queue=[]
                self.pitch_setpoint_queue=[]
                self.create_next_linear_setpoints()

            else:
                self.setpoint_changed=False
                self.subscribed_target[0] = msg.lat
                self.subscribed_target[1] = msg.long
                self.subscribed_target[2] = msg.alt
                self.obs = msg.obstacle_detected

            

            self.target[2]=self.subscribed_target[2]        

    # method to break down long distance travels into ~5 meter travels in both roll and pitch direction        
    def create_next_linear_setpoints(self):      
        print("generating...")
        print("self.subscribed_target",self.subscribed_target)
        print("drone_position",self.drone_position)
        lat_err = self.subscribed_target[0] - self.drone_position[0] 
        long_err = self.subscribed_target[1] - self.drone_position[1] 
        try: 
            tan_theta = abs(long_err/lat_err)
        except Exception as e:
            tan_theta =9999999999

        if abs( lat_err ) > abs(long_err):
            if abs(lat_err)>(self.min_lat_limit):
                print("Creating roll path")
                for i in range(1,1+int(abs(lat_err)/(self.min_lat_limit))):
                    self.roll_setpoint_queue.append(self.drone_position[0]+i*(self.min_lat_limit) if lat_err>0 else self.drone_position[0]-i*(self.min_lat_limit) )

                print("Creating pitch path")
                for i in range(1,1+int(abs(lat_err)/(self.min_lat_limit))):
                # for i in range(1,1+int(abs(long_err)/(self.min_long_limit*tan_theta))):
                    self.pitch_setpoint_queue.append(self.drone_position[1]+i*(self.min_long_limit)*tan_theta if long_err>0 else self.drone_position[1]-i*(self.min_long_limit)*tan_theta ) 
                
                self.rp_queue_max_length=int(abs(lat_err)/(self.min_lat_limit))
                self.target[0]=lat_err%((self.min_lat_limit))
                self.target[1]=long_err%((self.min_long_limit*tan_theta))
                self.setpoint_changed = False #Backup if self.setpoint_changed is published late
        else:
            if abs(long_err)>(self.min_long_limit):
                print("Creating roll path")
                for i in range(1,1+int(abs(long_err)/(self.min_long_limit))):
                # for i in range(1,1+int(abs(lat_err)/(self.min_lat_limit*tan_theta))):
                    self.roll_setpoint_queue.append(self.drone_position[0]+i*(self.min_lat_limit)/tan_theta if lat_err>0 else self.drone_position[0]-i*(self.min_lat_limit)/tan_theta )

                print("Creating pitch path")
                for i in range(1,1+int(abs(long_err)/(self.min_long_limit))):
                    self.pitch_setpoint_queue.append(self.drone_position[1]+i*(self.min_long_limit) if long_err>0 else self.drone_position[1]-i*(self.min_long_limit) ) 
                
                self.rp_queue_max_length=int(abs(long_err)/(self.min_long_limit))
                self.target[0]=lat_err%((self.min_lat_limit*tan_theta))
                self.target[1]=long_err%((self.min_long_limit))
                self.setpoint_changed = False #Backup if self.setpoint_changed is published late
        self.created_next_setpoints=True    
        print("SETPOINTS broken down")
    
    def check_proximity(self):

        if( len(self.roll_setpoint_queue)>0 and abs(self.drone_position[0] - self.roll_setpoint_queue[0])<= 0.000004517): #0.000004517
            self.roll_setpoint_queue.pop(0)

        if( len(self.pitch_setpoint_queue)>0 and abs(self.drone_position[1] - self.pitch_setpoint_queue[0])<= 0.0000047487): #0.000004517
            self.pitch_setpoint_queue.pop(0)


    def pid(self,):

        if len(self.roll_setpoint_queue)!=0:
            self.target[0]=self.roll_setpoint_queue[0]
        else:
            self.target[0]=self.subscribed_target[0]

        if len(self.pitch_setpoint_queue)!=0:
            self.target[1]=self.pitch_setpoint_queue[0]
        else:
            self.target[1]=self.subscribed_target[1]


        if ((len(self.roll_setpoint_queue)<=1 and len(self.pitch_setpoint_queue)<=1 and not self.obs) or (self.rp_queue_max_length-len(self.roll_setpoint_queue)<1)):
            # More slow & stable Tuning for smaller distances
            self.Kp = [18, 18,1000]
            self.Ki = [0, 0, -0.138]
            self.Kd = [500, 500, 2300]
            # self.error=[0,0,0]
            # self.Kp = [10, 10,1000]
            # self.Ki = [0, 0, -0.138]
            # self.Kd = [200, 200, 2300]
            
            print("using slow tuning")
        else:
            # Faster better uning for larger distances            
            self.Kp = [ 70, 70, 1000]
            self.Ki = [0, 0, -0.138]
            self.Kd = [ 600, 600, 2300 ]

            print("USING FAST TUNING, max_rpq=", self.rp_queue_max_length,len(self.roll_setpoint_queue))

        self.check_proximity()

        self.error[0] = ( self.target[0] - self.drone_position[0] )*100000
        self.error[1] = ( self.target[1] - self.drone_position[1] )*100000
        self.error[2] = ( self.target[2] - self.drone_position[2] )

        print("")
        print("Target",self.target)
        print("Drone_pos",self.drone_position)
        print("errrrrrr",self.error)
        print('roll_setpoint_queue',self.roll_setpoint_queue)
        print('pitch_setpoint_queue',self.pitch_setpoint_queue)


        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_diff[0] = (self.error[0] - self.prev_error[0])
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_diff[1] = (self.error[1] - self.prev_error[1])
        self.error_sum[2] = self.error_sum[2] + self.error[2]
        self.error_diff[2] = (self.error[2] - self.prev_error[2])


        # Calculating pid values
        self.rpt[0] = (
            (self.Kp[0] * self.error[0])
            + (self.Ki[0] * self.error_sum[0]) * self.sample_time
            + (self.Kd[0] * (self.error_diff[0]) / self.sample_time)
        )
        self.rpt[1] = (
            (self.Kp[1] * self.error[1])
            + (self.Ki[1] * self.error_sum[1]) * self.sample_time
            + (self.Kd[1] * (self.error_diff[1]) / self.sample_time)
        )
        self.rpt[2] = (
            (self.Kp[2] * self.error[2])
            + (self.Ki[2] * self.error_sum[2]) * self.sample_time
            + (self.Kd[2] * (self.error_diff[2]) / self.sample_time)
        )

        # Changing the previous error values
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        #------------------------------------------------#
        self.cmd_drone.rcRoll = 1500 + self.rpt[0]
        self.cmd_drone.rcPitch = 1500 + self.rpt[1]
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 1500 + self.rpt[2]

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
        self.roll_pub.publish(self.error[0])
        self.pitch_pub.publish(self.error[1])
        self.zero_pub.publish(0)


        if (self.drone_position[0]-self.last_point[0]>=0.000001) or (self.drone_position[1]-self.last_point[1]>=0.000001) or  (self.drone_position[2]-self.last_point[2]>=0.02): 
            self.last_point=list(self.drone_position)


if __name__ == "__main__":

    e_drone = Edrone()
    r = rospy.Rate(50)  # rate in Hz 

    while not rospy.is_shutdown():

        if all(e_drone.drone_position) and all(e_drone.subscribed_target):# and not e_drone.obstacle_detected_bottom and not  e_drone.obstacle_detected_top :
            print(time.strftime("%H:%M:%S"))
            e_drone.pid()

        r.sleep()
