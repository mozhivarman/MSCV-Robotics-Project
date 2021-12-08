#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/control_lane', Float64, self.cbFollowLane, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.lastError = 0
        self.MAX_VEL = 0.14

        rospy.on_shutdown(self.fnShutDown)


    def cbFollowLane(self, desired_center):
        center = desired_center.data
        error = center - 320
        print('The Error in the centroid : {}'.format(error))
        Kp = 0.003
        Kd = 0.009

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        twist = Twist()
        twist.linear.x = 0.05
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2) if angular_z < 0 else -min(angular_z, 2)
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
