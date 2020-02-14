#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TransformStamped, Vector3
import tf_conversions
import tf2_ros
from nav_msgs.msg import Odometry
import odrive
from odrive.enums import *
from time import sleep
import math


encoder_cpr = None
tyre_circumference = None
m_s_to_value = None
wheel_radius = None
wheel_track = None
odom_msg = Odometry()
tf_msg = TransformStamped()
odom_frame = None
base_frame = None
odom_pub = None
tf_pub = None
cur_x = 0.0
cur_y = 0.0
cur_theta = 0.0
new_pos_l = 0
new_pos_r = 0
old_pos_l = 0
old_pos_r = 0


def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val


def callback(data, odrv0):
    L = 0.5
    # rospy.loginfo(rospy.get_caller_id() + "I heard: \n %s", data)
    V = data.linear.x
    W = data.angular.z
    left = V - L / 2.0 * W
    left = left / tyre_circumference * encoder_cpr * (-1.0)
    right = V + L / 2.0 * W
    right = right / tyre_circumference * encoder_cpr
    odrv0.axis0.controller.vel_setpoint = left
    odrv0.axis1.controller.vel_setpoint = right
    rospy.loginfo(rospy.get_caller_id() + "I heard: \n %s \n %s", right, left)


def odometry(odrv0):
    global new_pos_l, new_pos_r, old_pos_l, old_pos_r, cur_x, cur_y, cur_theta
    right_vel = -odrv0.axis0.encoder.vel_estimate
    left_vel = odrv0.axis1.encoder.vel_estimate

    new_pos_r = odrv0.axis0.encoder.pos_cpr
    new_pos_l = odrv0.axis1.encoder.pos_cpr

    now = rospy.Time.now()
    odom_msg.header.stamp = now
    tf_msg.header.stamp = now

    # Twist/velocity: calculated from motor values only
    s = tyre_circumference * (left_vel + right_vel) / (2.0 * encoder_cpr)
    w = tyre_circumference * (right_vel - left_vel) / (
                wheel_track * encoder_cpr)  # angle: vel_r*tyre_radius - vel_l*tyre_radius
    odom_msg.twist.twist.linear.x = s
    odom_msg.twist.twist.angular.z = w

    delta_pos_l = new_pos_l - old_pos_l
    delta_pos_r = new_pos_r - old_pos_r

    old_pos_l = new_pos_l
    old_pos_r = new_pos_r

    # counts to metres
    delta_pos_l_m = delta_pos_l / m_s_to_value
    delta_pos_r_m = delta_pos_r / m_s_to_value

    # Distance travelled
    d = (delta_pos_l_m + delta_pos_r_m) / 2.0  # delta_ps
    th = (delta_pos_r_m - delta_pos_l_m) / wheel_track  # works for small angles

    xd = math.cos(th) * d
    yd = -math.sin(th) * d

    # Pose: updated from previous pose + position delta
    cur_x += math.cos(cur_theta) * xd - math.sin(cur_theta) * yd
    cur_y += math.sin(cur_theta) * xd + math.cos(cur_theta) * yd
    cur_theta = (cur_theta + th) % (2 * math.pi)

    # fill odom message and publish
    odom_msg.pose.pose.position.x = cur_x
    odom_msg.pose.pose.position.y = cur_y
    q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, cur_theta)
    odom_msg.pose.pose.orientation.z = q[2]  # math.sin(self.theta)/2
    odom_msg.pose.pose.orientation.w = q[3]  # math.cos(self.theta)/2

    # self.odom_msg.pose.covariance

    tf_msg.transform.translation.x = cur_x
    tf_msg.transform.translation.y = cur_y
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]

    # ... and publish!
    odom_pub.publish(odom_msg)
    tf_pub.sendTransform(tf_msg)


def transfer(odrv0):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    odometry(odrv0)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def terminate():
    None


if __name__ == '__main__':
    odrv0 = odrive.find_any()
    print(odrv0.vbus_voltage)
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    if odrv0.axis0.encoder.config.cpr == odrv0.axis1.encoder.config.cpr:
        encoder_cpr = odrv0.axis0.encoder.config.cpr
    else:
        rospy.loginfo("Your encoders have different cpr. Are you sure that it is correct?")
    tyre_circumference = float(get_param('tyre_circumference', None))
    wheel_radius = float(get_param('wheel_radius', 0.127))
    if not tyre_circumference:
        tyre_circumference = 2 * math.pi * wheel_radius
    m_s_to_value = encoder_cpr / tyre_circumference
    wheel_track = float(get_param('wheel_track', 0.88))

    odom_frame = get_param('odom_frame', "odom")
    base_frame = get_param('base_frame', "base_link")

    rospy.init_node('odrive', anonymous=False)
    rospy.on_shutdown(terminate)

    rospy.Subscriber("/diff_drive_controller/cmd_vel", Twist, callback, odrv0)

    odom_pub = rospy.Publisher("/diff_drive_controller/odom", Odometry, tcp_nodelay=True, queue_size=2)
    odom_msg.header.frame_id = odom_frame
    odom_msg.child_frame_id = base_frame
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    odom_msg.pose.pose.position.z = 0.0  # always on the ground, we hope
    odom_msg.pose.pose.orientation.x = 0.0  # always vertical
    odom_msg.pose.pose.orientation.y = 0.0  # always vertical
    odom_msg.pose.pose.orientation.z = 0.0
    odom_msg.pose.pose.orientation.w = 1.0
    odom_msg.twist.twist.linear.x = 0.0
    odom_msg.twist.twist.linear.y = 0.0  # no sideways
    odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
    odom_msg.twist.twist.angular.x = 0.0  # or roll
    odom_msg.twist.twist.angular.y = 0.0  # or pitch... only yaw
    odom_msg.twist.twist.angular.z = 0.0

    tf_pub = tf2_ros.TransformBroadcaster()

    tf_msg.header.frame_id = odom_frame
    tf_msg.child_frame_id = base_frame
    tf_msg.transform.translation.x = 0.0
    tf_msg.transform.translation.y = 0.0
    tf_msg.transform.translation.z = 0.0
    tf_msg.transform.rotation.x = 0.0
    tf_msg.transform.rotation.y = 0.0
    tf_msg.transform.rotation.w = 0.0
    tf_msg.transform.rotation.z = 1.0

    transfer(odrv0)
