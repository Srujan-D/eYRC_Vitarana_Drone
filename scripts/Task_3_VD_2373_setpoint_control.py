#!/usr/bin/env python

'''

Node which publishes the setpoints to go to (if no obstacle is detected )
Controls setpoint of all pickups and drops too.

This node publishes and subsribes the following topics:
    PUBLICATIONS               SUBSCRIPTIONS                SERVICES
    /edrone/setpoint_control   /edrone/gps                  /edrone/activate_gripper
    /edrone/marker_data        /edrone/center_lat_long


'''

import rospy
import time,os,math,csv
from vitarana_drone.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from vitarana_drone.srv import Gripper

# util functions
from Task_4_VD_2373_utils import *

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

        
        self.setpoint_queue=[[19.0,72.0,10.0]]
        self.start_coords=[19.0,72.0,8.44099749139]
        self.parcels_delivery_coords=[]
        self.parcels_coords=[[18.9999864489,71.9999430161,8.44099749139-0.15],[18.9999864489+2*0.000013552,71.9999430161,8.44099749139],[18.9999864489+0.000013552,71.9999430161+0.000014245,8.44099749139]]
        
        self.delivered=[]   # includes the ongoing setpoint too
        self.picked_up=[]   # includes the ongoing setpoint too
        
        
        # [19.000924871823383, 71.99983189451873, 25.6600061035167]
                    # 71.99987938144884
        # Task 1 final point
        # [19.0000451704, 72.0, 0.31]
        # [[19.0000451704, 72.0, 3.0],[19.0+2*0.0000451704, 72.0, 3.0],]
        
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
        self.prev_scanner_setpoint=None
        self.go_to_marker=False
        self.marker_point=-1   #activate from 0
        self.popped=False
        self.pub_marker_data.marker_id=3
        self.publish=True

        #  ROS Publishers
        self.setpoint_pub = rospy.Publisher("/edrone/setpoint_control", destination, queue_size=2)
        self.marker_data=rospy.Publisher("/edrone/marker_data", MarkerData, queue_size=1)


        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/center_lat_long", center_x_y, self.center_lat_long)

        # rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

        # rospy.Subscriber("/edrone/qr_scanner",qr_scanner,self.qr_callback)
        rospy.wait_for_service('/edrone/activate_gripper')
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper(),persistent=True)

        # Marker Detection
        self.img_width=400
        self.hfov_rad=1.3962634
        self.focal_length = (self.img_width/2)/math.tan(self.hfov_rad/2)
        self.prev_maker_data = center_x_y()
    
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


    
    def center_lat_long(self,msg):
        # self.marker_detected=False
        print("CENTER LAT LONG:",msg)
        if self.prev_maker_data.x!=msg.x and self.prev_maker_data.y!=msg.y :
            
            print("YAAAAYYYYY")
                                                                                               
            marker_x_m=(msg.x-self.img_width/2)*(self.drone_position[2]-self.delivered[-1][2]-1)/self.focal_length  # - 1 from self.delivered[-1][2] is done coz while reading, I have added 1 so as to maintain a buffer height from the marker 
            marker_y_m=-(msg.y-self.img_width/2)*(self.drone_position[2]-self.delivered[-1][2]-1)/self.focal_length    # the "-" is dependent on yaw angle i.e. orientation of drone w.r.t. 3d world

            lat_x = x_to_lat(marker_x_m + lat_to_x(self.drone_position[0]))
            long_y = y_to_long(marker_y_m + long_to_y(self.drone_position[1]))

            marker_setpoint= [lat_x,long_y,self.delivered[-1][2]+1]
            print(self.marker_setpoints)
            try:
                if not self.check_lat_long_proximity(marker_setpoint,self.marker_setpoints[self.marker_point]):
                    self.marker_setpoints.append(marker_setpoint)
                    print("ADDED marker_detected point")
                    self.marker_point+=1
                    self.go_to_marker=True
                    self.setpoint_queue.pop(0) # pop the marker searching setpoints 
            except:
                self.marker_setpoints.append(marker_setpoint)
                self.go_to_marker=True
                self.marker_point+=1
                self.setpoint_queue.pop(0) # pop the marker searching setpoints 
                print("First ADDED")

            self.pub_marker_data.err_x_m = lat_to_x(self.drone_position[0])+marker_x_m
            self.pub_marker_data.err_y_m = long_to_y(self.drone_position[1])+marker_y_m
        else:
            print("no_new_msg",msg)
        self.prev_maker_data=msg

    def check_lat_long_proximity(self,target,current=None):
        
        if current is None:
            current = self.drone_position

        if (
            (
                abs(current[0] - target[0])<= 0.000004517/3 #0.000004517
            )
            and (
                abs(current[1] - target[1])<= 0.0000047487/3 #0.0000047487
            )
        ):           
            return True
        else:
            return False


    def check_proximity(self,target,current=None):
        
        if current is None:
            current = self.drone_position

        if (
            (
                abs(current[0] - target[0])
                <= 0.000004517/3 #0.000004517
            )
            and (
                abs(current[1] - target[1])
                <= 0.0000047487/3 #0.0000047487
            )
            and (
                abs(current[2] - target[2])
                <= 0.2
            )
        ):
            # if self.iterations>=30:
                # self.popped=False
            self.check_gripper()
            print("INSIDE PROXIMITY")
            print('self.target',target)
            print('self.dronepos',current)

            return True
            # else:
            #     self.iterations+=1
            #     return False
        else:
            # self.iterations = 0
            return False


    def drop(self):

        if abs(self.delivered[-1][2]+1-self.marker_setpoints[self.marker_point][2])<=0.05:
            print("GOING DOWN FOR DROPPING")
            self.marker_setpoints.insert(self.marker_point,[self.marker_setpoints[self.marker_point][0],self.marker_setpoints[self.marker_point][1],self.marker_setpoints[self.marker_point][2]-2+self.drone_height])
            self.marker_setpoints.pop(self.marker_point+1)
        else:
            print("Dropped parcel")
            for i in range(10):
                resp=self.gripper(False)
            print(resp)
            print(self.drone_position)
            self.go_to_marker=False
            self.parcel_picked=False


    def check_gripper(self):
        print("CHECKING GRIPPER")
        try:
            resp = self.gripper(True)
            if str(resp).split(' ')[1] == 'True':
                self.picking_parcel=False
                self.parcel_picked=True
                print("PICKED UP!!")
                # print("Self.popped",self.popped)
                if not self.popped:
                    self.popped=True
                    # self.drop()
                    print("POPPED FROM CHECK GRIPPER")
                    self.setpoint_queue.pop(0)
                    print('after',self.setpoint_queue)
                    self.setpoint=list(self.setpoint_queue[0])
            else:
                self.parcel_picked=False
                self.popped=False
                print("NOT PICKED YET")
        except Exception as e:
            print('errr',e)

    def search_delivery_marker(self):
        
        if len(self.setpoint_queue)==0:
            self.setpoint_queue.insert(0,[self.delivered[-1][0],self.delivered[-1][1],self.drone_position[2]+1]) 
        
        elif len(self.setpoint_queue)>0 and self.check_proximity(self.setpoint_queue[0]):
            print("GOING UP")
            self.setpoint_queue.pop(0)
            self.setpoint_queue.insert(0,[self.delivered[-1][0],self.delivered[-1][1],self.drone_position[2]+1]) 



    def check_setpoint_queue(self):
        try:

            print(len(self.setpoint_queue),self.setpoint_queue)
            
            if len(self.setpoint_queue)==0 or self.check_proximity(self.setpoint_queue[0]):
                if not self.parcel_picked and len(self.parcels_coords)>0:
                    if len(self.picked_up)>0 and (self.picked_up[-1] not in self.setpoint_queue):
                        print("GOING TO PICKUP POINT")
                        if self.parcels_coords[0][2]+2>self.drone_position:
                            self.add_setpoint_to_queue(list([self.parcels_coords[0][0],self.parcels_coords[0][1],self.parcels_coords[0][2]+2]))
                        else:
                            self.add_setpoint_to_queue(list([self.parcels_coords[0][0],self.parcels_coords[0][1],self.drone_position[2]+2]))
                        self.add_setpoint_to_queue(list(self.parcels_coords[0]))
                        self.picked_up.append(list(self.parcels_coords[0]))
                        self.parcels_coords.pop(0)
                        print("len(self.parcels_coords)",len(self.parcels_coords))
                    elif len(self.picked_up)==0:
                        print("GOING TO 1st PICKUP POINT")
                        self.add_setpoint_to_queue(list(self.parcels_coords[0]))
                        self.picked_up.append(list(self.parcels_coords[0]))
                        self.parcels_coords.pop(0)
                        print("len(self.parcels_coords)",len(self.parcels_coords))
                    
                elif self.parcel_picked and len(self.parcels_delivery_coords)>0 and len(self.picked_up)==len(self.delivered)+1:
                    print("GOING TO DELIVERY POINT")
                    self.add_setpoint_to_queue(list(self.parcels_delivery_coords[0]))
                    self.delivered.append(list(self.parcels_delivery_coords[0]))
                    self.parcels_delivery_coords.pop(0)
                
                else:
                    print("Wandering",self.setpoint_queue)
                    print("SELF.PICKING",self.picking_parcel)

            # if len(self.setpoint_queue)>0 and self.check_proximity(self.setpoint):
            #     if self.drone_position[2]>self.setpoint[2]:
            #         self.setpoint_queue.insert(0,[self.setpoint_queue[0][0],self.setpoint_queue[0][1],self.drone_position[2]])
            #     else:
            #         self.setpoint_queue.insert(0,[self.drone_position[0],self.drone_position[1],self.setpoint_queue[1][2]])

            if len(self.setpoint_queue)>0 and self.check_proximity(self.setpoint_queue[0]):

                if len(self.setpoint_queue)>1 and abs(self.setpoint_queue[1][2] - self.drone_position[2])>=0.1 and (self.setpoint_queue[0][0]!=self.setpoint_queue[1][0] and self.setpoint_queue[0][1]!=self.setpoint_queue[1][1]):
                    print("Getting ready")

                    if self.drone_position[2]>self.setpoint_queue[1][2]:
                        self.setpoint_queue.insert(0,[self.setpoint_queue[1][0],self.setpoint_queue[1][1],self.drone_position[2]])
                    else:
                        self.setpoint_queue.insert(0,[self.drone_position[0],self.drone_position[1],self.setpoint_queue[1][2]])

                    self.setpoint_queue.pop(1)
                    # self.setpoint=list(self.setpoint_queue[0])
                    # return
                    # print("Going to",self.setpoint_queue)
                    # elif len(self.setpoint_queue)==0:
                    #     print("reached")
                elif not self.picking_parcel:
                    if len(self.setpoint_queue)>1 and self.check_lat_long_proximity(self.setpoint_queue[0],self.setpoint_queue[1]) and self.setpoint_queue[0][2]>self.setpoint_queue[1][2]:
                        print("self.picking_parcel=True")
                        self.picking_parcel=True
                    else:
                        print("self.picking_parcel=False")
                        self.picking_parcel=False
                    print("Default checkpoint reached popping")
                    self.setpoint_queue.pop(0)
                    print("Updated spq",self.setpoint_queue)

            try:
                if self.parcel_picked and self.check_lat_long_proximity(self.delivered[-1]) and not self.go_to_marker:
                    print("SEARCHING MARKER")
                    self.search_delivery_marker()
            except Exception as e:
                print("SEARCHING MARKER ERROR",e)

            
            try:
                self.setpoint=list(self.setpoint_queue[0])
            except:
                print("CANT ASSIGN EMPTY SETPOINT QUEUE")
                self.publish=False
            self.publish=True
        except Exception as e:
            print("error in popping",e)
            self.publish=False


    def check_marker_queue(self):

        try:
            print("CHECKING MARKER Q")
            if self.check_proximity(self.marker_setpoints[self.marker_point]):
                # self.marker_setpoints.pop(0)
                self.drop()
            self.setpoint=list(self.marker_setpoints[self.marker_point])
            self.publish=True
        except:
            print("NO MARKER SETPOINTS")
            self.setpoint=list(self.drone_position)
            self.publish=False
            

    def add_setpoint_to_queue(self, setpoint):
        print("ADDED", setpoint)
        self.setpoint_queue.append(list(setpoint))
        return
    

    def setpoint_control(self):
        
        print("")
        print(time.strftime("%H:%M:%S"))
        print("")
        pub_msg=destination()
        if self.go_to_marker and len(self.marker_setpoints)>0:
            print("Going to marker")
            self.check_marker_queue()
            pub_msg.lat=self.marker_setpoints[self.marker_point][0]
            pub_msg.long=self.marker_setpoints[self.marker_point][1]
            pub_msg.alt=self.marker_setpoints[self.marker_point][2]
            print("marker sp",self.marker_setpoints)
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

    def publish_marker_data(self):
        self.marker_data.publish(self.pub_marker_data)


def main():

    delivery_control = SetpointControl()
    
    with open('/home/atharva/catkin_ws/src/vitarana_drone/scripts/manifest.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            delivery_control.parcels_delivery_coords.append([float(row[1]),float(row[2]),float(row[3])+1]) # 1m height buffer 

    r = rospy.Rate(50)
    counter = 0
    rospy.on_shutdown(delivery_control.reset)
    while not rospy.is_shutdown():
        # if all(delivery_control.drone_position):
        delivery_control.setpoint_control()
        # publish marker_data
        if counter==0:
            delivery_control.publish_marker_data()
            counter+=1
        else:
            counter+=1
            if counter ==50:
                counter=0

        r.sleep()

if __name__ == "__main__":
    main()