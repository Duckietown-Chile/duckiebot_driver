#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from sensor_msgs.msg import Joy
import numpy as np

class LookAtMe(object):
    def __init__(self):
        # Sub datos laser
        self.laser_sub = rospy.Subscriber('/joy', Joy, self.process_laser)
        # Pub comando base
        # self.duck = rospy.Publisher('/duck/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.duck = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.command = Twist2DStamped()
        
    def process_laser(self, msg):
        self.command.v = msg.axes[1]
        self.command.omega = 100.0*msg.axes[0]

        self.duck.publish(self.command)

if __name__ == '__main__':
    rospy.init_node('t1')
    t1 = LookAtMe()
    rospy.spin()

