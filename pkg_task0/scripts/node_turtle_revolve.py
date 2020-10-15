#!/usr/bin/env python
from math import pow, sqrt

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class turtlebot:
    def __init__(self):
        rospy.init_node("turtle_revolve", anonymous=True)
        self.velpub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.posesub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)
        self.pose = Pose()  # Current Pose
        self.dist = 0  # Distace to goal
        self.dist_tol = 0.05  # Tolerable distance between turtle pose and goal
        self.goal_pose = Pose()  # Goal Pose (same as start pose here)
        self.goal_pose.x = 5.544
        self.goal_pose.y = 5.544
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)

    # Callback function for turtle pose subscriber
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move2goal(self):
        cnt = 0  # Compare dist and dist_tol after few iterations
        # otherwise turtle will break from loop earlier

        while not rospy.is_shutdown():

            rospy.loginfo("Moving in a circle")
            # linear velocity in the x-axis:
            self.vel_msg.linear.x = 1

            # angular velocity in the z-axis:
            self.vel_msg.angular.z = 1

            # Publishing vel_msg
            self.velpub.publish(self.vel_msg)
            self.dist = sqrt(
                pow((self.goal_pose.x - self.pose.x), 2)
                + pow((self.goal_pose.y - self.pose.y), 2)
            )
            if cnt > 10 and self.dist <= self.dist_tol:
                rospy.loginfo("goal reached")
                break
            cnt += 1
            self.rate.sleep()

        # Stopping our robot after the movement is over
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velpub.publish(self.vel_msg)
        rospy.spin()


if __name__ == "__main__":
    try:
        o = turtlebot()
        o.move2goal()

    except rospy.ROSInterruptException:
        pass
