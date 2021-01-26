#!/usr/bin/env python


# Importing the required libraries

import time

import rospy
import tf
import math
import threading
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix,LaserScan
from std_msgs.msg import Float32
from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper



lock=threading.Lock()

class Edrone:
    """docstring for Edrone"""

    def __init__(self):
        rospy.init_node("position_controller")  # initializing ros node with name position_controller

        # Format for drone_command
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 0

        # The latitude, longitude and altitude of the drone
        self.drone_position = [0.0, 0.0, 0.0]

        # The coordinates in the target postion vector is in the order latitude, longitude and altitude
        # self.set_points = [[19.0009248718, 71.9998318945, 25],[19.0007046575, 71.9998955286, 25],[19.0007046575, 71.9998955286, 21]] #[19.00000, 72.000003, 3.31]
        
        self.parcel_setpoints=[19.0007046575, 71.9998955286, 21]
        self.target=[0,0,0]
        self.next_target=None
        self.last_point=[0,0,0]
        self.stop_coords=[19.0005646575,71.9998955286,24]
        self.last_set_point=[0,0,0]
        # Initial settings for the values of Kp, Ki and Kd for roll, pitch and throttle
        self.Kp = [400000, 400000, 30]
        self.Ki = [0, 0, 0.0001]
        self.Kd = [19000000, 19000000, 2300]
        
        
        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_diff = [0.0, 0.0, 0.0]

        self.rpt = [0.0, 0.0, 0.0]

        self.obstacle_detected_bottom=False
        self.obstacle_detected_top=False

        self.go_to_parcel=False
        self.scanned_destination_setpoint=None
        self.destination_set=False
        self.parcel_picked=False
        # self.go_down=False
        self.reached_status=False

        self.go_up_counter=0
        self.go_down_counter=0
        self.go_to_parcel_counter=0
        # self.go__counter=0
        self.scaling_factor=0.00005
        # self.go_up_counter=0

        self.go_up_setpoint=None
        self.go_down_limit=0.0000005
        self.process_counter=None
        self.prev_process_counter=None
        self.stop_drone_counter=0
        # minimum and maximum values for roll, pitch and throttle
        self.min_value = [1375, 1375, 1000]
        self.max_value = [1625, 1625, 2000]


        self.top_sensor_dist=None
        self.bottom_sensor_dist=None

        # Sample time in which pid is run. The stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        #  ROS Publishers
        self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
        self.throttle_pub = rospy.Publisher("/throttle_error", Float32, queue_size=1)
        # self.zero_pub = rospy.Publisher("zero", Float32, queue_size=1)
        
        self.pwm_pub = rospy.Publisher("/edrone/pwm", prop_speed, queue_size=1) # for shutdown hook
        

        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/range_finder_top",LaserScan,self.range_finder_top_callback)
        rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.range_finder_bottom_callback)
        rospy.Subscriber("/edrone/qr_scanner",qr_scanner,self.qr_callback)
       
        # ROS Services 
        rospy.wait_for_service('/edrone/activate_gripper')
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper())

        # ------------------------------------------------------------------------------------------------------------

    def gps_callback(self, msg):
            self.drone_position[0] = msg.latitude
            self.drone_position[1] = msg.longitude
            self.drone_position[2] = msg.altitude

    def range_finder_top_callback(self,msg):
            self.top_sensor_dist=msg.ranges[:]
            print('dist is',self.top_sensor_dist)
            if (self.top_sensor_dist[3]<=5):
                self.stop()
                print("stopping")
            # if (self.top_sensor_dist[3]<=10 and self.parcel_picked==True):
                self.obstacle_detected_top=True
            #     self.go_left()
            #     print("left")
            else:
                self.obstacle_detected_top=False
    
    def range_finder_bottom_callback(self,msg): 
            self.bottom_sensor_dist=msg.ranges[0]
            if (self.bottom_sensor_dist<=1 and (not self.parcel_picked) and not(abs(self.drone_position[0] -self.parcel_setpoints[0])<= 0.00001 and abs(self.drone_position[1] -self.parcel_setpoints[1])<= 0.00001)):
                self.obstacle_detected_bottom=True
                self.process_counter=2
                self.go_up()
                print("go up")
            else:
                self.obstacle_detected_bottom=False

    def qr_callback(self, data):
        try:
            print(data.lat_x)
            print(data)
            if data.alt_z and data.lat_x and data.long_y:
                self.scanned_destination_setpoint=[data.lat_x,data.long_y,data.alt_z]
                self.destination_set=True
        except Exception as e:
            print(e)


    def limit_value(self, current, min, max):
        if current < min:
            return min
        elif current > max:
            return max
        else:
            return current

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

    def go_up(self):
        if self.go_up_counter==0:
            self.go_up_counter+=1
            self.go_up_setpoint=self.drone_position[2]+2.5
        self.next_target=[self.drone_position[0],self.drone_position[1],self.go_up_setpoint]#self.target[2]+0.1] #TODO:change
        self.pid(0,self.next_target)
        self.pid(1,self.next_target)
        self.pid(2,self.next_target)
        print("going up")
    
    def go_down(self):
        # if self.go_up_counter==0:
        #     self.go_up_counter+=1
        #     self.go_up_setpoint=self.drone_position[2]-1
        self.next_target=[self.drone_position[0],self.drone_position[1],21] #TODO:change
        self.pid(0,self.next_target)
        self.pid(1,self.next_target)
        self.pid(2,self.next_target)
        print("going down")

    def get_side_point(self):
        x2=self.lat_to_x(self.drone_position[0])
        x1=self.lat_to_x(self.last_point[0])
        y2=self.long_to_y(self.drone_position[1])
        y1=self.long_to_y(self.last_point[1])
        print('x2,x1,y2,y1')
        print(x2,x1,y2,y1)
        if x2==x1:
            x3=x2+1
            y3=y2
        else:
            m=(y2-y1)/(x2-x1)
            x3=x2-m*math.sqrt(1/(1+m**2))
            y3=y2-m*math.sqrt(1/(1+m**2))
        set_lat=self.x_to_lat(x3)
        set_long=self.y_to_long(y3)
        return(set_lat,set_long)

    def stop(self):
        if self.stop_drone_counter==0:
            self.stop_drone_counter+=1
            self.stop_coords=[self.stop_coords[0],self.drone_position[0]+0.000003,self.drone_position[2]]
        self.next_target=self.stop_coords #TODO:change
        self.pid(0,self.next_target)
        self.pid(1,self.next_target)
        self.pid(2,self.next_target)
        print("stop func")

    def go_right(self):
        self.next_target=[self.target[0],self.target[1],self.target[2]] #TODO:change
        self.pid(0,self.next_target)
        self.pid(1,self.next_target)
        self.pid(2,self.next_target)
        print("going right")

    def go_left(self):
        set_coord=self.get_side_point()
        self.next_target=[set_coord[0],set_coord[1],self.drone_position[2]] #TODO:change
        self.pid(0,self.next_target)
        self.pid(1,self.next_target)
        self.pid(2,self.next_target)
        print("going left")

    def move_drone(self):
 
            process_number=0

            if(
                (abs(self.drone_position[0] -self.parcel_setpoints[0])<= self.go_down_limit)
                and
                (abs(self.drone_position[1] -self.parcel_setpoints[1])<= self.go_down_limit)
                and
                not self.parcel_picked
            ):
                self.go_down()
                self.process_counter=1
                print("down")

            elif not self.parcel_picked:
                self.process_counter=3
                self.next_target=[self.parcel_setpoints[0],self.parcel_setpoints[1],self.target[2]]
                self.pid(0,self.next_target)
                self.pid(1,self.next_target)
                self.pid(2,self.next_target)
                process_number=3
                print("parcel")

            elif self.parcel_picked==True and self.drone_position[2]<=23.5:
                print("pickup")
                self.process_counter=4
                self.go_up()
                process_number=4
                
            elif self.destination_set==True and self.drone_position[2]>=24.5:
                print("GOING TO DESTINATION!")
                self.process_counter=5
                self.scanned_destination_setpoint[2]=24.5
                self.pid(0,self.scanned_destination_setpoint)
                self.pid(1,self.scanned_destination_setpoint)
                self.pid(2,self.scanned_destination_setpoint)
                process_number=5

            self.prev_process_counter=self.process_counter

            try:
                resp=self.gripper(True)

                if str(resp).split(' ')[1] == 'True':
                    self.parcel_picked=True
            except Exception as e:
                print(e)

            if (self.drone_position[0]-self.last_point[0]>=0.000001) or  (self.drone_position[1]-self.last_point[1]>=0.000001) or  (self.drone_position[2]-self.last_point[2]>=0.02): 
                self.last_point=list(self.drone_position)

            self.reset_flags(process_number)

            
    def reset_flags(self,process_number):
        if process_number==0:
            self.go_up_counter=0
            self.go_to_parcel_counter=0
        elif process_number==1:
            self.go_down_counter=0
            self.go_to_parcel_counter=0
        elif process_number==2:
            self.go_down_counter=0
            self.go_to_parcel_counter=0
            self.go_up_counter=0

        elif process_number==3:
            self.go_down_counter=0
            self.go_up_counter=0

    # TODO: use x,y values and get coordinates of a point 10 meters towards the destination direction
    def create_next_setpoint(self):
        # Breaking down long distance travels to smaller one
        if abs(self.drone_position[0]-self.last_point[0])<=0.000004 and abs(self.drone_position[1]-self.last_point[1])<0.00004 and abs(self.drone_position[2]-self.last_point[2])<0.1:
            self.last_set_point=self.target
            err=list(self.error)
            if err[1]!=0:
                tan_theta = abs(err[0]/err[1])  #TODO:remove abs() here
                # if err[0]>err[1]:
                if abs(err[0])>=self.scaling_factor:
                    if abs(err[1])>self.scaling_factor and abs(tan_theta)<1:
                        err[1]=self.scaling_factor if err[1]>0 else -self.scaling_factor
                        err[0]=self.scaling_factor*tan_theta
                    else:  
                        err[0]=self.scaling_factor if err[0]>0 else -self.scaling_factor
                        err[1]=self.scaling_factor/tan_theta

                elif abs(err[1])>=self.scaling_factor:
                    err[1]=self.scaling_factor if err[1]>0 else -self.scaling_factor
                    err[0]=self.scaling_factor*tan_theta
                self.error=list(err)
            

    def pid(self,select_rpt,target_point):
        self.target = list(target_point)
        # Just for the sake of completeness, no real significance
        # else:
        #     self.target=[self.drone_position[0],self.drone_position[1],self.drone_position[2]+2]
        #     print("Something's wrong!")
        # if select_rpt==0: 
        #     print('target',str(self.target))
        #     print('position',str(self.drone_position))

        

        # Calculating the error
        self.error[select_rpt] = (
            self.target[select_rpt] - self.drone_position[select_rpt]
        )
        print("errrrrrr",self.error)
       
        self.create_next_setpoint()

        print("scaled errrrrrrr",self.error)

        self.error_sum[select_rpt] = self.error_sum[select_rpt] + \
            self.error[select_rpt]
        self.error_diff[select_rpt] = (
            self.error[select_rpt] - self.prev_error[select_rpt]
        )
        




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

        self.cmd_drone.rcRoll = self.limit_value(
            self.cmd_drone.rcRoll, self.min_value[0], self.max_value[0]
        )
        self.cmd_drone.rcPitch = self.limit_value(
            self.cmd_drone.rcPitch, self.min_value[1], self.max_value[1]
        )
        self.cmd_drone.rcThrottle = self.limit_value(
            self.cmd_drone.rcThrottle, self.min_value[2], self.max_value[2]
        )

        # limiting the values
        self.cmd_drone.rcRoll = self.limit_value(
            self.cmd_drone.rcRoll, self.min_value[0], self.max_value[0]
        )
        self.cmd_drone.rcPitch = self.limit_value(
            self.cmd_drone.rcPitch, self.min_value[1], self.max_value[1]
        )
        self.cmd_drone.rcThrottle = self.limit_value(
            self.cmd_drone.rcThrottle, self.min_value[2], self.max_value[2]
        )
        self.cmd_pub.publish(self.cmd_drone)

        if (
            (
                abs(self.drone_position[0] -self.target[0])
                <= 0.000007#0.000004517
            )
            and (
                abs(self.drone_position[1] -self.target[1])
                <= 0.000007#0.0000047487
            )
            and (
                abs(self.drone_position[2] -self.target[2])
                <= 0.1
            )
        ):
            return True
        else:
            return False

    def reset(self):
        pwm=prop_speed()
        pwm.prop1=0
        pwm.prop2=0
        pwm.prop3=0
        pwm.prop4=0
        self.pwm_pub.publish(pwm)


if __name__ == "__main__":

    e_drone = Edrone()
    r = rospy.Rate(50)  # rate in Hz 
    rospy.on_shutdown(e_drone.reset)

    while not rospy.is_shutdown():
        if e_drone.drone_position[0]!=0 and not e_drone.obstacle_detected_bottom and not  e_drone.obstacle_detected_top :
            e_drone.move_drone()
    
        r.sleep()
