#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float64

def pwm_callback(percentage_vel_motor):
    dc_max = 0.1
    dc_min = 0.05

    dc = dc_min + ((dc_max - dc_min)/100) * percentage_vel_motor
    pass


sub_pwm = rospy.Subscriber("/control_node", Float64,callback = pwm_callback)
pub_esc = rospy.Publisher("/esc", Float64, queue_size=10)

