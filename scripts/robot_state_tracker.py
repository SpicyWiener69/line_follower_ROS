#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist
import numpy as np
import cv2
import copy
class RobotStateTracker:
    def __init__(self):
        rospy.init_node('robot_state_tracker')
        self.map = cv2.imread('route.png')
        self.x = rospy.get_param("~x0", 80)
        self.y = rospy.get_param("~y0", 900)
        self.theta = rospy.get_param("~theta0", np.pi)  

        # pixels/sec && rad/sec
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.pub = rospy.Publisher('/sim/pose', Pose2D, queue_size=1)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

    def cmd_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        print(f'command:{msg.linear.x,msg.angular.z}')

    def run(self):
        rospy.loginfo("Starting RobotStateTracker")
        dt = 1.0 / 10.0  

        while not rospy.is_shutdown():
            self.theta += self.angular_velocity * dt
            # Keep  within [0, 2π)
            self.theta = self.theta % (2 * np.pi)
            self.x +=int( self.linear_velocity * np.sin(self.theta) * dt)
            self.y += int(self.linear_velocity * np.cos(self.theta) * dt)

            map_copy = copy.copy(self.map)
            cv2.circle(map_copy, (self.x,self.y), 20, (0,255,0),thickness=-1)
            cv2.imshow('results',map_copy)
            cv2.waitKey(1)
            pose = Pose2D(x=self.x, y=self.y, theta=self.theta)
            self.pub.publish(pose)
            print(f'{self.x} {self.y}')
            self.rate.sleep()

if __name__ == '__main__':
    tracker = RobotStateTracker()
    tracker.run()
