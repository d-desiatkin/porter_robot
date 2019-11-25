#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import odrive
from odrive.enums import *
from time import sleep
import math
	

def callback(data, odrv0):
	L = 0.5
	# rospy.loginfo(rospy.get_caller_id() + "I heard: \n %s", data)
	V = data.linear.x
	W = data.angular.z
	left = V - L/2 * W
	left = left/(2*math.pi*0.127)*90*(-1)
	right = V + L/2 * W
	right = right/(2*math.pi*0.127)*90
	odrv0.axis0.controller.vel_setpoint = left
	odrv0.axis1.controller.vel_setpoint = right
	rospy.loginfo(rospy.get_caller_id() + "I heard: \n %s \n %s", right, left)


def transfer(odrv0):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('transfer', anonymous=False)

    rospy.Subscriber("/porter/diff_drive_controller/cmd_vel", Twist, callback, odrv0)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	odrv0 = odrive.find_any()
	print(odrv0.vbus_voltage)
	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	transfer(odrv0)
