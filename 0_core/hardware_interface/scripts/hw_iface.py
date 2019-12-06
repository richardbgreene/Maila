#!/usr/bin/env python

from __future__ import division


import Adafruit_PCA9685 # Import the PCA9685 module.

# Import ros modules and msg
import rospy
from custom_msgs.msg import ActuatorControl, ActuatorControlInfo

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)



info_pub = None
max_steer_angle = 1 #30*3.1415/180.

steer_center_cmd = 640
steer_range = 150 #200

throttle_center_cmd = 600
throttle_range = 200

pwm_freq = 100

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

def set_steering_angle(val):
    control_steering(val/max_steer_angle)

def control_velocity(val):
    pwm.set_pwm(1, 0, int(throttle_range * val + throttle_center_cmd))

def control_steering(val):
    pwm.set_pwm(0, 0, int(steer_range * val + steer_center_cmd))

def initialize_controls():
    # FrSky D4R-II period is 18 mS ~ 55 hz
    pwm.set_pwm_freq(pwm_freq)     # Set frequency to 60hz, good for servos.
    control_velocity(0)
    set_steering_angle(0)

def callback(data):
    global info_pub
    rospy.loginfo(rospy.get_caller_id() + " recived control cmd")
    set_steering_angle(data.delta)
    control_velocity(data.dc)

    # Publish node information
    info = ActuatorControlInfo()
    info.cmd_recived_stamp = rospy.Time()
    info.ctrl_set_point = data
    info_pub.publish(info)

def listener():
    global info_pub

    initialize_controls()

    rospy.init_node('hw_iface', anonymous=True)
    info_pub = rospy.Publisher('hw/info', ActuatorControlInfo, queue_size=1)
    rospy.Subscriber("hw/cmd", ActuatorControl, callback)

    rospy.spin()

if __name__ == '__main__':
    #global pwm

    listener()
