#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Based on https://github.com/duckietown/Software/blob/master/catkin_ws/src/dagu_car/src/wheels_driver_node.py
__author__ = 'Rodrigo Muñoz'


import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from duckietown_driver.serial_interface import DuckietownSerial
from duckietown_driver.message import DuckietownCommand, DuckietownStatus

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False


        # Setup driver
        self.driver = DuckietownSerial('/dev/ttyUSB0', baudrate = 115200)
        self.cmd = DuckietownCommand()
        self.status = DuckietownStatus()
        
        # Add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed", WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheel_command_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.emergency_stop_cb, queue_size=1)

    def wheel_command_cb(self, msg):
        if self.estop:
            self.cmd.pwm_ch1 = 0
            self.cmd.pwm_ch2 = 0
            self.driver.send_command(self.cmd)
            return
        # Velocity conversion @TODO
        
        # Command saturation
        msg.vel_left = min(max(msg.vel_left,-1.0),1.0)
        msg.vel_right = min(max(msg.vel_right,-1.0),1.0)

        # L9110 use inverse logic
        if msg.vel_left >= 0.0:
            self.cmd.pwm_ch1 = 255 - int(255*msg.vel_left)
        else:
            self.cmd.pwm_ch1 = int(255*msg.vel_left)

        if msg.vel_right >= 0.0:
            self.cmd.pwm_ch2 = 255 - int(255*msg.vel_right)
        else:
            self.cmd.pwm_ch2 = int(255*msg.vel_right)

        self.driver.send_command(self.cmd)
        
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()  
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def emergency_stop_cb(self,msg):
        self.estop = not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.cmd.pwm_ch1 = 0
        self.cmd.pwm_ch2 = 0
        self.driver.send_command(self.cmd)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
