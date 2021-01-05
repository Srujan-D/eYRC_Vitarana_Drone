#!/usr/bin/env python

import rospy
import time,os
from vitarana_drone.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from vitarana_drone.srv import Gripper
import csv

# Marker Detection
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

class SetpointControl():

    def __init__(self):

        rospy.init_node('setpoint_control')


        self.drone_position=[0,0,0]
        
        self.setpoint_queue=[[19.0,72.0,10.0]]
        self.start_coords=[19.0,72.0,8.44099749139]
        self.parcels_delivery_coords=[]
        self.parcels_coords=[[18.9999864489,71.9999430161,8.44099749139],[18.9999864489+2*0.000013552,71.9999430161,8.44099749139],[18.9999864489+0.000013552,71.9999430161-0.000014245,8.44099749139]]
        # [19.000924871823383, 71.99983189451873, 25.6600061035167]
                    # 71.99987938144884
        # Task 1 final point
        # [19.0000451704, 72.0, 0.31]
        # [[19.0000451704, 72.0, 3.0],[19.0+2*0.0000451704, 72.0, 3.0],]
        
        # Task 3
        # start point [18.99924113805385, 71.99981954948407, 16.660023269655206]
        # [[18.9993675932, 72.0000569892, 10.7+17],[18.9990965928,72.0000664814,10.75+10],[18.9990965925, 71.9999050292, 22.2+10]]
        # [[19.0007046580, 71.9998955286, 24.6600061035167],[19.0007046575, 71.9998955286, 22],[19.0007046575, 71.9998955286, 25.6600061035167],[19.0006046575, 71.9998955286, 25.6600061035167]]

        # self.building_ids=[3,1,2,0,0,0,0,0]
        self.parcel_picked=False
        self.delivering=False

        self.marker_setpoints=[[0,0,0]]
        self.setpoint=list(self.setpoint_queue[0])
        self.iterations=0
        self.dest_msg=destination()
        cc_path='/data/cascade.xml'
        abs_cc_path=os.path.dirname(__file__)+cc_path
        self.logo_cascade = cv2.CascadeClassifier(abs_cc_path)
        
        self.img=None
        self.bridge = CvBridge()
        

        self.pub_marker_data=MarkerData()
        self.marker_detected=False

        # QR scanned setpoints
        self.scanned_destination_setpoint=[0,0,0]
        self.destination_set = False
        self.prev_scanner_setpoint=None
        self.go_to_marker=False
        self.marker_point=0
        self.popped=False
        self.pub_marker_data.marker_id=3


        #  ROS Publishers
        self.setpoint_pub = rospy.Publisher("/edrone/setpoint_control", destination, queue_size=2)
        self.marker_data=rospy.Publisher("/edrone/marker_data", MarkerData, queue_size=1)


        # ROS Subscribers
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        # rospy.Subscriber("/edrone/center_lat_long", center_x_y, self.center_lat_long)

        # rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

        rospy.Subscriber("/edrone/qr_scanner",qr_scanner,self.qr_callback)
        rospy.wait_for_service('/edrone/activate_gripper')
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper',Gripper())


    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude


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


    def check_range(self,dist,target):
        return (abs(dist - target)>=0.001)

    
    def center_lat_long(self,msg):
        # self.marker_detected=False
        if msg.x and msg.y :
            # self.marker_detected = True
            lat_x = self.x_to_lat(msg.y + self.lat_to_x(self.drone_position[0]))
            long_y = self.y_to_long(msg.x + self.long_to_y(self.drone_position[1]))
            # print(lat_x,long_y)
            marker_setpoint= [lat_x,long_y,self.drone_position[2]]
            print("YAAAAYYYYY")
            print(self.marker_setpoints)
            try:
                if self.check_range(marker_setpoint[0],self.marker_setpoints[self.marker_point][0]) and  self.check_range(marker_setpoint[1],self.marker_setpoints[self.marker_point][1]) :
                    self.marker_setpoints.append(marker_setpoint)
                    print("ADDED")
                    self.marker_point+=1
                    self.go_to_marker=True
            except:
                self.marker_setpoints.append(marker_setpoint)
                print("First ADDED")

            self.pub_marker_data.err_x_m = self.lat_to_x(self.drone_position[0])+msg.x
            self.pub_marker_data.err_y_m = self.long_to_y(self.drone_position[1])+msg.y
            # self.pub_marker_data.marker_id=self.building_ids[0]
        else:
            print("no_new_msg",msg)


    def check_proximity(self,target):
        if (
            (
                abs(self.drone_position[0] - target[0])
                <= 0.000004517/3 #0.000004517
            )
            and (
                abs(self.drone_position[1] - target[1])
                <= 0.0000047487/3 #0.0000047487
            )
            and (
                abs(self.drone_position[2] - target[2])
                <= 0.2
            )
        ):
            if self.iterations>=60:
                return True
            else:
                self.iterations+=1
                return False
        else:
            self.iterations = 0
            return False


    def drop(self):
        # print("",self.parcels_delivery_coords)
        if self.parcel_picked and len(self.parcels_delivery_coords)>0 and len(self.setpoint_queue)<=1 :
            self.add_setpoint_to_queue(list(self.parcels_delivery_coords[0]))
            self.parcels_delivery_coords.pop(0)
            print("GOING TO DELIVERY POINT")
            # self.delivering=True
        return


    def pickup(self):
        if not self.parcel_picked and len(self.parcels_coords)>0 and len(self.setpoint_queue)<=1:
            self.add_setpoint_to_queue(list(self.parcels_coords[0]))
            self.parcels_coords.pop(0)
            print("GOING TO PICKUP POINT")
            print("len(self.parcels_coords)",len(self.parcels_coords))
        return
            # self.delivering=False
        # elif len(self.parcels_coords)==0:
        #     self.add_setpoint_to_queue(self.start_coords)



    def check_gripper(self):
        try:
            resp=self.gripper(True)
            if str(resp).split(' ')[1] == 'True':
                self.parcel_picked=True
                print("PICKED UP!!")
                if not self.popped:
                    self.setpoint_queue.pop(0)
                    self.setpoint=list(self.setpoint_queue[0])
                    self.popped=True
            else:
                self.parcel_picked=False
                self.popped=False
                print("NOT PICKED YET")
        except Exception as e:
            print(e)


    def check_setpoint_queue(self):
        self.drop()
        self.pickup()
        self.check_gripper()

        print(len(self.setpoint_queue),self.setpoint_queue)
        if self.go_to_marker and self.check_proximity(self.marker_setpoints[self.marker_point]):
            # self.marker_setpoints.pop(0)
            print("marker popped")
            self.go_to_marker=False
        
        elif len(self.setpoint_queue)>0 and self.check_proximity(self.setpoint_queue[0]):
            try:
                if len(self.setpoint_queue)>1 and abs(self.setpoint_queue[1][2] - self.drone_position[2])>=0.1 and (self.setpoint_queue[0][0]!=self.setpoint_queue[1][0] and self.setpoint_queue[0][1]!=self.setpoint_queue[1][1]):
                    print("Getting ready")

                    if self.drone_position[2]>self.setpoint_queue[1][2]:
                        self.setpoint_queue.insert(0,[self.setpoint_queue[1][0],self.setpoint_queue[1][1],self.drone_position[2]])
                    else:
                        self.setpoint_queue.insert(0,[self.drone_position[0],self.drone_position[1],self.setpoint_queue[1][2]])
            
                    self.setpoint_queue.pop(1)
                    # print("Going to",self.setpoint_queue)
                elif len(self.setpoint_queue)==1:
                    print("reached")
                else:
                    print("POPPing")
                    self.setpoint_queue.pop(0)
                    print("POPPED")
                    # self.building_ids.pop(0)
                    # self.pub_marker_data.marker_id = self.building_ids[0]
                self.setpoint=list(self.setpoint_queue[0])
            except Exception as e:
                print("error in popping",e)
                self.setpoint=list(self.drone_position)
        
        # elif len(self.setpoint_queue)==1:
        #     print("reached")
        
        else:   
            # print("GOING",self.iterations)
            pass


    def add_setpoint_to_queue(self, setpoint):
        print("ADDED", setpoint)
        self.setpoint_queue.append(list(setpoint))
        return
    

    def setpoint_control(self):
        self.check_setpoint_queue()
        print("")
        print(time.strftime("%H:%M:%S"))
        print("")
        pub_msg=destination()
        if self.go_to_marker:
            pub_msg.lat=self.marker_setpoints[self.marker_point][0]
            pub_msg.long=self.marker_setpoints[self.marker_point][1]
            pub_msg.alt=self.marker_setpoints[self.marker_point][2]
            print("Going to marker")
        else:
            pub_msg.lat=self.setpoint[0]
            pub_msg.long=self.setpoint[1]
            pub_msg.alt=self.setpoint[2]

        self.dest_msg=pub_msg
        self.setpoint_pub.publish(pub_msg)


    def x_to_lat(self, input_x):
        return input_x/110692.0702932625 + 19

    def y_to_long(self, input_y):
        return -input_y/(105292.0089353767 )+ 72

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def reset(self):
        self.setpoint_pub.publish(destination())

    def publish_marker_data(self):
        self.marker_data.publish(self.pub_marker_data)


def main():

    delivery_control=SetpointControl()
    
    with open('/home/atharva/catkin_ws/src/vitarana_drone/scripts/manifest.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            delivery_control.parcels_delivery_coords.append([float(row[1]),float(row[2]),float(row[3])+0.2])

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


# Equations of lines

# till parcel
# y= -0.2889644*x + 77.4904233

# till delivery
# y= -0.14825840*x + 74.816909775

if __name__ == "__main__":
    main()