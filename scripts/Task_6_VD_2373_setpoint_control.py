#!/usr/bin/env python

'''

Team ID        VD#2373
Theme          Vitarana Drone
Author List    Atharva Chandak, Srujan Deolasse, Naitik Khandelwal, Ayush Agrawal
Filename       Task_6_VD_2373_Setpoint_Control.py
Functions      gps_callback,range_finder_bottom_callback,check_lat_long_proximity,setpoint_control,drop,leave_parcel,check_proximity_with_iter,search_delivery_marker,reset,setpoint_control,add_setpoint_to_queue
Global Variables None


Node which publishes the setpoints to go to (if no obstacle is detected )
Controls setpoint of all pickups and drops too.

This node publishes and subsribes the following topics:
    PUBLICATIONS               SUBSCRIPTIONS                SERVICES
    /edrone/setpoint_control   /edrone/gps                  /edrone/activate_gripper
    /edrone/marker_data        /edrone/center_lat_long


'''

import rospy
import time,os,math
from vitarana_drone.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import NavSatFix,LaserScan
from vitarana_drone.srv import Gripper

# util functions
from Task_6_VD_2373_utils import *

# Marker Detection
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt

class SetpointControl():

    def __init__(self):

        rospy.init_node('setpoint_control')


        self.drone_position=[0,0,0]
        self.drone_height=0.31

        # Task_4
        # self.setpoint_queue=[[19,72,10]]#[[18.99988879058862, 72.00021844012868, 18.757980880739165]]
        # self.start_coords=[19.0,72.0,8.44099749139]#[18.99988879058862, 72.00021844012868, 16.757980880739165]
        # self.parcels_delivery_coords=[[19.0007030405,71.9999429002,22.1600026799],[18.9993676146,71.9999999999,10.65496],[19.0004681325,72.0000949773,16.660019864],]
        # self.parcels_coords=[[18.9999864489,71.9999430161,8.44099749139-0.1],[18.9999864489+0.000013552,71.9999430161+0.000014245,8.44099749139-0.1],[18.9999864489+2*0.000013552,71.9999430161,8.44099749139-0.1],[19.0,72.0,8.44099749139]]
        self.instructions=['DELIVERY','DELIVERY','DELIVERY']
        
        self.start_coords= [18.99988879058862, 72.00021844012868, 16.757980880739165]
        self.setpoint_queue=[[self.start_coords[0],self.start_coords[1],self.start_coords[2]+2]]
        self.parcels_delivery_coords=[]
        self.parcels_coords=[]#[[18.9999864489,71.9999430161,8.44099749139],[18.9999864489+0.000013552,71.9999430161+0.000014245,8.44099749139],[18.9999864489+2*0.000013552,71.9999430161,8.44099749139],[19.0,72.0,8.44099749139]]
        
        self.delivered=[]   # includes the ongoing setpoint too
        self.picked_up=[]   # includes the ongoing setpoint too
        self.instructions=[]
        
        
        self.proximity_iterations=0
        self.lat_long_proximity_iterations=0
        self.bottom_sensor_dist=None

        # Task 1 final point
        # [[19.0004,71.9997,10]]

        # Task 2 obs avoid testing:
        # [19.0001646575,71.9999,26.1599967919]
        # start coord:
        # [19.0009248718,71.9998318945,24.1600061035]

        # Task 3
        # start point [18.99924113805385, 71.99981954948407, 16.660023269655206]
        # [[18.9993675932, 72.0000569892, 10.7+17],[18.9990965928,72.0000664814,10.75+10],[18.9990965925, 71.9999050292, 22.2+10]]
        # [[19.0007046580, 71.9998955286, 24.6600061035167],[19.0007046575, 71.9998955286, 22],[19.0007046575, 71.9998955286, 25.6600061035167],[19.0006046575, 71.9998955286, 25.6600061035167]]

        self.parcel_picked=False
        self.picking_parcel=False

        self.marker_setpoints=[]
        self.setpoint=list(self.setpoint_queue[0])
        self.iterations=0
        self.dest_msg=destination()
        cc_path='/data/cascade.xml'
        abs_cc_path=os.path.dirname(__file__)+cc_path
        self.logo_cascade = cv2.CascadeClassifier(abs_cc_path)

        self.pub_marker_data=MarkerData()
        self.marker_detected=False

        # QR scanned setpoints
        self.scanned_destination_setpoint=[0,0,0]
        self.destination_set = False
        self.go_to_marker=False
        self.marker_point=-1   #activate from 0
        self.popped=False
        self.pub_marker_data.marker_id=3
        self.publish=True
        self.last_spq_popped=None
        self.checked_return_pickup_point=False

        #  ROS Publishers
        self.setpoint_pub = rospy.Publisher("/edrone/setpoint_control", destination, queue_size=2)
        self.marker_data=rospy.Publisher("/edrone/marker_data", MarkerData, queue_size=1)


        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/center_lat_long", center_x_y, self.center_lat_long)
        rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.range_finder_bottom_callback)

        # rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

        # rospy.Subscriber("/edrone/qr_scanner",qr_scanner,self.qr_callback)
        rospy.wait_for_service('/edrone/activate_gripper')
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper(),persistent=True)

        # Marker Detection
        self.img_width=400
        self.hfov_rad=1.3962634
        self.focal_length = (self.img_width/2)/math.tan(self.hfov_rad/2)
        self.prev_maker_data = center_x_y()
        self.searching_marker=False

    # Callback to set current drone location
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    # Callback to add QR scanned 
    def qr_callback(self, msg):
        
        try:
            if msg.alt_z and msg.lat_x and msg.long_y and not self.destination_set:

                drop_point=[msg.lat_x,msg.long_y, 25 ]  #msg.alt_z]
                self.add_setpoint_to_queue(drop_point)
                
                self.scanned_destination_setpoint=[msg.lat_x,msg.long_y,msg.alt_z]
                self.add_setpoint_to_queue(self.scanned_destination_setpoint)
                
                self.destination_set=True
        
        except Exception as e:
            print(e)

    def range_finder_bottom_callback(self,msg):

        self.bottom_sensor_dist=msg.ranges[0]

    
    def center_lat_long(self,msg):
        # Marker detection should only work if drone near vicinity of the setpoint
        print("CENTER LAT LONG:",msg)
        if self.prev_maker_data.x!=msg.x and self.prev_maker_data.y!=msg.y:
            if self.searching_marker:
                # The following are deduced as best fitted keeping in mind the parallax effect & numerous trials
                x_center = msg.x - (self.img_width / 2 )
                y_center = msg.y - (self.img_width / 2 )
                if x_center >= 100:
                    setpoint_x_pixel = (x_center + (msg.square_size/2))
                elif x_center <= -100:
                    setpoint_x_pixel = (x_center - (msg.square_size/2))
                else:
                    setpoint_x_pixel = x_center #- (msg.square_size/2)
                
                if y_center >= 100:
                    setpoint_y_pixel = (y_center + (msg.square_size/2))
                elif y_center <= -100:
                    setpoint_y_pixel = (y_center - (msg.square_size/2))
                else:
                    setpoint_y_pixel = y_center #- (msg.square_size/2)
                    

                # setpoint_y_pixel = (y_center + (msg.square_size/2)) if y_center >= 0 else (y_center - (msg.square_size/2)) 
                
                print(setpoint_x_pixel)
                print(setpoint_y_pixel)
                                                # -msg.square_size                                                 
                marker_x_m = setpoint_x_pixel * (self.drone_position[2]-self.delivered[-1][2]-1)/self.focal_length # + 0.5# - 1 from self.delivered[-1][2] is done coz while reading, I have added 1 so as to maintain a buffer height from the marker 
                marker_y_m = - (setpoint_y_pixel * (self.drone_position[2]-self.delivered[-1][2]-1)/self.focal_length ) #+0.25  # the "-" is dependent on yaw angle i.e. orientation of drone w.r.t. 3d world
                # marker_x_m = setpoint_x_pixel * (self.bottom_sensor_dist-1)/self.focal_length # + 0.5# - 1 from self.delivered[-1][2] is done coz while reading, I have added 1 so as to maintain a buffer height from the marker 
                # marker_y_m = - (setpoint_y_pixel * (self.bottom_sensor_dist-1)/self.focal_length ) #+0.25  # the "-" is dependent on yaw angle i.e. orientation of drone w.r.t. 3d world

                lat_x = x_to_lat(marker_x_m + lat_to_x(self.drone_position[0]))#+9.03e-6/2 
                long_y = y_to_long(marker_y_m + long_to_y(self.drone_position[1]))#+ 0.0000047487/2

                marker_setpoint= [lat_x,long_y,self.delivered[-1][2]+1]
                print(self.marker_setpoints)
                
                # if marker_setpoint is near setpoint_queue, only then add the point marker_queue
                # if abs(marker_setpoint[0]-self.setpoint_queue[0])
                try:

                    if (abs(marker_setpoint[0]-self.marker_setpoints[self.marker_point][0])>=0.000004517
                        and (abs(marker_setpoint[0]-self.marker_setpoints[self.marker_point][0])<=0.00002 
                        and abs(marker_setpoint[1]-self.marker_setpoints[self.marker_point][1])<=0.00002) 
                        and abs(marker_setpoint[1]-self.marker_setpoints[self.marker_point][1])>=0.0000047487):
                        self.marker_setpoints[self.marker_point]=marker_setpoint
                        # print("updated marker_detected point")
                        self.go_to_marker=True
                    elif abs(marker_setpoint[0]-self.marker_setpoints[self.marker_point][0])>=0.00002 and abs(marker_setpoint[1]-self.marker_setpoints[self.marker_point][1])>=0.00002:
                        self.marker_setpoints.append(marker_setpoint)
                        # print("ADDED marker_detected point")
                        self.marker_point+=1
                        self.go_to_marker=True
                        self.last_spq_popped=self.setpoint_queue.pop(0) # pop the marker searching setpoints 
                except:
                    self.marker_setpoints.append(marker_setpoint)
                    self.go_to_marker=True
                    self.marker_point+=1
                    self.last_spq_popped=self.setpoint_queue.pop(0) # pop the marker searching setpoints 

                self.pub_marker_data.err_x_m = lat_to_x(self.drone_position[0])+marker_x_m
                self.pub_marker_data.err_y_m = long_to_y(self.drone_position[1])+marker_y_m
        else:
            # print("no_new_msg",msg)
            self.prev_maker_data=msg

    def check_lat_long_proximity(self,target,current=None):
        
        if current is None:
            current = self.drone_position

        if (
            (
                abs(current[0] - target[0])<= 0.000004517/4 #0.000004517
            )
            and (
                abs(current[1] - target[1])<= 0.0000047487/4 #0.0000047487
            )
        ):  
            return True
        else:
            return False


    def check_proximity_with_iter(self,target,current=None):
        
        if current is None:
            current = self.drone_position

        if (
            (
                abs(current[0] - target[0])
                <= 0.000004517/4 #0.000004517
            )
            and (
                abs(current[1] - target[1])
                <= 0.0000047487/4 #0.0000047487
            )
            and (
                abs(current[2] - target[2])
                <= 0.2
            )
        ):
            if self.proximity_iterations>=20:
                self.popped=False
                self.check_gripper()
                self.proximity_iterations=0
                return True
            else:
                self.proximity_iterations+=1
                return False
        else:
            self.iterations = 0
            return False


    def check_proximity(self,target,current=None):
        
        if current is None:
            current = self.drone_position

        if (
            (
                abs(current[0] - target[0])
                <= 0.000004517/4 #0.000004517
            )
            and (
                abs(current[1] - target[1])
                <= 0.0000047487/4 #0.0000047487
            )
            and (
                abs(current[2] - target[2])
                <= 0.2
            )
        ):           
            return True
        else:
            return False

    def leave_parcel(self):
        print("Dropped parcel")
        for i in range(15):
            resp=self.gripper(False)
        print(resp)
        print(self.drone_position)
        self.go_to_marker=False
        self.parcel_picked=False
        self.searching_marker=False
        self.checked_return_pickup_point=False
        instruction_status=self.instructions[0]
        self.instructions.pop(0)
        if instruction_status=='DELIVERY':      #since for return deliveries. setpoints of setpoint_queue is used while dropping the parcel
            self.setpoint_queue=[]


    def drop(self):
        
        if self.instructions[0]=='DELIVERY' and( abs(self.delivered[-1][2]+1-self.marker_setpoints[self.marker_point][2])<=0.05):
            self.marker_setpoints.insert(self.marker_point,[self.marker_setpoints[self.marker_point][0],self.marker_setpoints[self.marker_point][1],self.marker_setpoints[self.marker_point][2]-self.bottom_sensor_dist+0.05+self.drone_height])
            self.marker_setpoints.pop(self.marker_point+1)
        else:
           self.leave_parcel()


    def check_gripper(self):
        try:
            resp = self.gripper(True)
            if str(resp).split(' ')[1] == 'True':
                self.picking_parcel=False
                self.parcel_picked=True
                if not self.popped:
                    self.popped=True
                    self.last_spq_popped=self.setpoint_queue.pop(0)
                    self.setpoint=list(self.setpoint_queue[0])
            else:
                self.parcel_picked=False
                self.popped=False
        except Exception as e:
            print('ERROR IN check_gripper',e)


    def search_delivery_marker(self):
        
        if len(self.setpoint_queue)==0:
            self.setpoint_queue.insert(0,[self.delivered[-1][0],self.delivered[-1][1],self.drone_position[2]+1]) 
        
        elif len(self.setpoint_queue)>0 and self.check_proximity_with_iter(self.setpoint_queue[0]):
            self.last_spq_popped=self.setpoint_queue.pop(0)
            self.setpoint_queue.insert(0,[self.delivered[-1][0],self.delivered[-1][1],self.drone_position[2]+1]) 
        self.searching_marker=True

    def check_setpoint_queue(self):
        try:
            if len(self.setpoint_queue)==0 or self.check_proximity(self.setpoint_queue[0]):
                # GOING TO PICKUP POINT
                if not self.parcel_picked and len(self.parcels_coords)>0:
                    if len(self.picked_up)>0 and (self.picked_up[-1] not in self.setpoint_queue) and len(self.setpoint_queue)==0 :

                        if self.parcels_coords[0][2]>self.drone_position[2]:
                            self.add_setpoint_to_queue(list([self.drone_position[0],self.drone_position[1],self.parcels_coords[0][2]+8]))
                            self.add_setpoint_to_queue(list([self.parcels_coords[0][0],self.parcels_coords[0][1],self.parcels_coords[0][2]+8]))
                        else:
                            self.add_setpoint_to_queue(list([self.drone_position[0],self.drone_position[1],self.drone_position[2]+8]))
                            self.add_setpoint_to_queue(list([self.parcels_coords[0][0],self.parcels_coords[0][1],self.drone_position[2]+8]))
                        self.add_setpoint_to_queue(list(self.parcels_coords[0]))
                        self.picked_up.append(list(self.parcels_coords[0]))
                        self.parcels_coords.pop(0)
                        # print("len(self.parcels_coords)",len(self.parcels_coords))
                    elif len(self.picked_up)==0:
                        # print("Going to 1st pickup point")
                        self.add_setpoint_to_queue(list(self.parcels_coords[0]))
                        self.picked_up.append(list(self.parcels_coords[0]))
                        self.parcels_coords.pop(0)
                
                # GOING TO DELIVERY POINT
                elif self.parcel_picked and len(self.parcels_delivery_coords)>0 and len(self.picked_up)==len(self.delivered)+1:
                    if self.parcels_delivery_coords[0][2]>self.drone_position[2]:
                        # print("deliver to higher coord")
                        self.add_setpoint_to_queue(list([self.drone_position[0],self.drone_position[1],self.parcels_delivery_coords[0][2]+8]))
                        self.add_setpoint_to_queue(list([self.parcels_delivery_coords[0][0],self.parcels_delivery_coords[0][1],self.parcels_delivery_coords[0][2]+8]))
                    else:
                        # print("deliver to lower coord")
                        self.add_setpoint_to_queue(list([self.drone_position[0],self.drone_position[1],self.drone_position[2]+8]))
                        self.add_setpoint_to_queue(list([self.parcels_delivery_coords[0][0],self.parcels_delivery_coords[0][1],self.drone_position[2]+8]))
                    self.add_setpoint_to_queue(list(self.parcels_delivery_coords[0]))
                    self.delivered.append(list(self.parcels_delivery_coords[0]))
                    self.parcels_delivery_coords.pop(0)
                elif  not self.parcel_picked and len(self.parcels_delivery_coords)==0 and len(self.delivered)==len(self.picked_up):
                    self.add_setpoint_to_queue(list(self.start_coords))
                else:
                    # Wandering
                   pass


            if len(self.setpoint_queue)>0 and self.check_proximity_with_iter(self.setpoint_queue[0]):

                if len(self.setpoint_queue)>1 and abs(self.setpoint_queue[1][2] - self.drone_position[2])>=0.1 and (self.setpoint_queue[0][0]!=self.setpoint_queue[1][0] and self.setpoint_queue[0][1]!=self.setpoint_queue[1][1]):

                    if self.drone_position[2]>self.setpoint_queue[1][2]:
                        self.setpoint_queue.insert(0,[self.setpoint_queue[1][0],self.setpoint_queue[1][1],self.drone_position[2]])
                    else:
                        self.setpoint_queue.insert(0,[self.drone_position[0],self.drone_position[1],self.setpoint_queue[1][2]])

                    self.last_spq_popped=self.setpoint_queue.pop(1)

                elif not self.picking_parcel :
                    if len(self.setpoint_queue)>1 and self.check_lat_long_proximity(self.setpoint_queue[0],self.setpoint_queue[1]) and self.setpoint_queue[0][2]>self.setpoint_queue[1][2] and not self.parcel_picked:
                        if (not self.checked_return_pickup_point) and self.check_lat_long_proximity(self.setpoint_queue[0]):
                            if (self.bottom_sensor_dist-(self.setpoint_queue[0][2]-self.setpoint_queue[1][2])>=0):
                                self.setpoint_queue[1][2]= self.drone_position[2]-self.bottom_sensor_dist+0.31
                            else:
                                self.checked_return_pickup_point= True

                        if (self.setpoint_queue[0][2]-self.setpoint_queue[1][2])>3:
                            self.setpoint_queue.insert(1,[self.setpoint_queue[0][0],self.setpoint_queue[0][1],self.setpoint_queue[0][2]-7])
                        else:
                            self.picking_parcel=True

                    else:
                        self.picking_parcel=False


                    if self.check_proximity(self.setpoint_queue[0]):
                        self.last_spq_popped=self.setpoint_queue.pop(0)

            try:
                if self.parcel_picked and self.check_lat_long_proximity(self.delivered[-1]) and not self.go_to_marker and self.instructions[0]=='DELIVERY':
                    self.search_delivery_marker()
                elif self.parcel_picked and self.check_proximity_with_iter(self.delivered[-1]) and not self.go_to_marker and self.instructions[0]=='RETURN':
                    self.drop()
                elif self.parcel_picked and self.check_lat_long_proximity(self.delivered[-1]) and not self.go_to_marker and self.instructions[0]=='RETURN':
                    if self.check_proximity_with_iter(self.last_spq_popped):
                        self.setpoint_queue.insert(0,[self.setpoint_queue[0][0],self.setpoint_queue[0][1],self.setpoint_queue[0][2]-7])

            except Exception as e:
                print("SEARCHING MARKER ERROR",e)

            
            try:
                self.setpoint=list(self.setpoint_queue[0])
            except:
                print("CANT ASSIGN EMPTY SETPOINT QUEUE")
                self.publish=False
            self.publish=True
        except Exception as e:
            print("ERROR IN POPPING",e)
            self.publish=False


    def check_marker_queue(self):

        try:
            if self.check_proximity_with_iter(self.marker_setpoints[self.marker_point]):
                # self.marker_setpoints.pop(0)
                self.drop()
            self.setpoint=list(self.marker_setpoints[self.marker_point])
            self.publish=True
        except:
            print("NO MARKER SETPOINTS")
            self.setpoint=list(self.drone_position)
            self.publish=False
            

    def add_setpoint_to_queue(self, setpoint):
        if len(self.setpoint_queue):
            if abs(self.setpoint_queue[-1][2]-setpoint[2])>0.1:
                self.setpoint_queue.append(list(setpoint))
        self.setpoint_queue.append(list(setpoint))
        return
    

    def setpoint_control(self):
        
        pub_msg=destination()
        if self.go_to_marker and len(self.marker_setpoints)>0:
            self.check_marker_queue()
            pub_msg.lat=self.marker_setpoints[self.marker_point][0]
            pub_msg.long=self.marker_setpoints[self.marker_point][1]
            pub_msg.alt=self.marker_setpoints[self.marker_point][2]

        else:
            self.check_setpoint_queue()
            pub_msg.lat=self.setpoint[0]
            pub_msg.long=self.setpoint[1]
            pub_msg.alt=self.setpoint[2]

        self.dest_msg=pub_msg
        if self.publish:
            self.setpoint_pub.publish(pub_msg)


    def reset(self):
        self.setpoint_pub.publish(destination())



def main():

    delivery_control = SetpointControl()
    
    # delivery_control.setpoint_queue
    delivery_control.parcels_coords,delivery_control.parcels_delivery_coords,delivery_control.instructions=get_set_point_sequence()

    r = rospy.Rate(50)
    counter = 0
    rospy.on_shutdown(delivery_control.reset)
    while not rospy.is_shutdown():
        delivery_control.setpoint_control()
        r.sleep()

if __name__ == "__main__":
    main()