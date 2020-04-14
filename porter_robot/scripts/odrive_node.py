#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, TransformStamped, Vector3
import tf_conversions
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv
import odrive
from odrive.enums import *
import math
import serial


def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val


class ODriveNode(object):
    encoder_cpr = None
    tyre_circumference = None
    fast_timer_comms_active = False
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
    left_cmd = 0.0
    left_vel = 0.0
    right_cmd = 0.0
    right_vel = 0.0
    driver = None
    ser = None

    def __init__(self):

        rospy.init_node('odrive_node', anonymous=False)
        rospy.Rate(20)
        rospy.on_shutdown(self.terminate)
        self.ser = serial.Serial('/dev/ttyTHS2', 115200)  # open serial port
        rospy.loginfo("Used port: %s", str(self.ser.name))

        rospy.Subscriber("diff_drive_controller/cmd_vel", Twist, self.drive, queue_size=1)
        self.odom_pub = rospy.Publisher("diff_drive_controller/odom", Odometry, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

        rospy.Service('reset_odometry', std_srvs.srv.Trigger, self.reset_odometry)

        vbus_voltage = self.driver_read(b'vbus_voltage')

        rospy.loginfo("Current plate voltage: %s", vbus_voltage)

        self.driver_write(b'axis0.requested_state {}'.format(AXIS_STATE_CLOSED_LOOP_CONTROL))
        self.driver_write(b'axis1.requested_state {}'.format(AXIS_STATE_CLOSED_LOOP_CONTROL))

        axis0_encoder_config_cpr = self.driver_read(b'axis0.encoder.config.cpr')
        axis1_encoder_config_cpr = self.driver_read(b'axis1.encoder.config.cpr')

        if axis0_encoder_config_cpr == axis1_encoder_config_cpr:
            self.encoder_cpr = float(axis0_encoder_config_cpr)
            rospy.loginfo("cpr = {}".format(self.encoder_cpr))
        else:
            rospy.loginfo("Your encoders have different cpr. Are you sure that it is correct?")
            rospy.signal_shutdown("I can't work with different encoders cpr")

        nm = rospy.get_name()
        self.tyre_circumference = float(get_param(nm + '/' + 'tyre_circumference', 0.0))
        self.wheel_radius = float(get_param(nm + '/' + 'wheel_radius', 0.127))
        self.wheel_track = float(get_param(nm + '/' + 'wheel_track', 0.88))
        self.odom_frame = get_param(nm + '/' + 'odom_frame', "odom")
        self.base_frame = get_param(nm + '/' + 'base_frame', "base_link")
        self.odom_calc_hz = float(get_param(nm + '/' + 'odom_calc_hz', 20.0))

        if not self.tyre_circumference:
            self.tyre_circumference = 2 * math.pi * self.wheel_radius
        self.m_s_to_value = self.encoder_cpr / self.tyre_circumference

        self.fast_timer = rospy.Timer(rospy.Duration(1.0 / self.odom_calc_hz), self.publish)

        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0  # always on the ground, we hope
        self.odom_msg.pose.pose.orientation.x = 0.0  # always vertical
        self.odom_msg.pose.pose.orientation.y = 0.0  # always vertical
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
        self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
        self.odom_msg.twist.twist.angular.x = 0.0  # or roll
        self.odom_msg.twist.twist.angular.y = 0.0  # or pitch... only yaw
        self.odom_msg.twist.twist.angular.z = 0.0

        self.tf_msg.header.frame_id = self.odom_frame
        self.tf_msg.child_frame_id = self.base_frame
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.x = 0.0
        self.tf_msg.transform.rotation.y = 0.0
        self.tf_msg.transform.rotation.w = 0.0
        self.tf_msg.transform.rotation.z = 1.0

        jsm = JointState()
        jsm.name = ['chasiss_to_left_actuated_wheel', 'chasiss_to_right_actuated_wheel']
        jsm.position = [0.0, 0.0]
        self.joint_state_msg = jsm

    def odometry(self):

        now = rospy.Time.now()
        self.odom_msg.header.stamp = now
        self.tf_msg.header.stamp = now

        # Twist/velocity: calculated from motor values only
        s = self.tyre_circumference * (self.left_vel + self.right_vel) / (2.0 * self.encoder_cpr)
        w = self.tyre_circumference * (self.right_vel - self.left_vel) / (
                self.wheel_track * self.encoder_cpr)  # angle: vel_r*tyre_radius - vel_l*tyre_radius

        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w

        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r

        half_cpr = self.encoder_cpr / 2.0
        if delta_pos_l > half_cpr:
            delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr:
            delta_pos_l = delta_pos_l + self.encoder_cpr
        if delta_pos_r > half_cpr:
            delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr:
            delta_pos_r = delta_pos_r + self.encoder_cpr

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value

        # Distance travelled
        d = (delta_pos_l_m + delta_pos_r_m) / 2.0  # delta_ps
        th = (delta_pos_r_m - delta_pos_l_m) / self.wheel_track  # works for small angles

        xd = math.cos(th) * d
        yd = -math.sin(th) * d

        # Pose: updated from previous pose + position delta
        self.cur_x += math.cos(self.cur_theta) * xd - math.sin(self.cur_theta) * yd
        self.cur_y += math.sin(self.cur_theta) * xd + math.cos(self.cur_theta) * yd
        self.cur_theta = (self.cur_theta + th) % (2 * math.pi)

        # fill odom message and publish
        self.odom_msg.pose.pose.position.x = self.cur_x
        self.odom_msg.pose.pose.position.y = self.cur_y
        # rospy.loginfo('  %s: %s', str(self.odom_msg.pose.pose.position.x), str(self.odom_msg.pose.pose.position.y))
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.cur_theta)
        self.odom_msg.pose.pose.orientation.z = q[2]  # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3]  # math.cos(self.theta)/2

        # self.odom_msg.pose.covariance

        self.tf_msg.transform.translation.x = self.cur_x
        self.tf_msg.transform.translation.y = self.cur_y
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        # ... and publish!
        self.odom_pub.publish(self.odom_msg)
        self.tf_pub.sendTransform(self.tf_msg)

    def joint_angles(self):
        self.joint_state_msg.header.stamp = rospy.Time.now()
        if self.ser:
            self.joint_state_msg.position[0] = 2 * math.pi * self.new_pos_l / self.encoder_cpr
            self.joint_state_msg.position[1] = 2 * math.pi * self.new_pos_r / self.encoder_cpr
        self.joint_state_publisher.publish(self.joint_state_msg)

    def publish(self, timer_event):
        if self.fast_timer_comms_active:
            self.driver_write_vel(0, self.right_cmd)
            self.driver_write_vel(1, self.left_cmd)
            self.new_pos_r, self.right_vel = self.driver_read_enc(0)
            self.new_pos_l, self.left_vel = self.driver_read_enc(1)
            self.new_pos_l = -self.new_pos_l
            self.left_vel = -self.left_vel
            self.odometry()
            self.joint_angles()

    def drive(self, data):
        L = 0.5
        # rospy.loginfo(rospy.get_caller_id() + "I heard: \n %s", data)
        V = data.linear.x
        W = data.angular.z
        self.left_cmd = V - self.wheel_track / 2.0 * W
        self.left_cmd = self.left_cmd / self.tyre_circumference * self.encoder_cpr * (-1.0)
        self.right_cmd = V + self.wheel_track / 2.0 * W
        self.right_cmd = self.right_cmd / self.tyre_circumference * self.encoder_cpr

    def terminate(self):
        self.ser.cancel_read()
        self.ser.cancel_write()
        self.ser.flush()
        self.driver_write(b'axis0.requested_state {}'.format(AXIS_STATE_IDLE))
        self.driver_write(b'axis1.requested_state {}'.format(AXIS_STATE_IDLE))
        self.ser.close()

    def run(self):
        self.fast_timer_comms_active = True
        rospy.spin()

    def reset_odometry(self, request):
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        return (True, "Odometry reset.")

    def ser_read(self):
        c = ''
        value = ''
        while not c == '\n':
            value = value + c
            if self.ser.readable():
                c = self.ser.read()
        return value

    def ser_write(self, buf):
        # rospy.loginfo(buf + '\n')
        counter = 0
        size = len(buf)
        while (1):
            if counter >= size:
                break
            if self.ser.writable():
                self.ser.write(buf[counter])
                counter = counter + 1

    def driver_write(self, prop):
        buf = b'w ' + prop + b'\n'
        self.ser_write(buf)

    def driver_write_vel(self, axis, vel):
        buf = b'v ' + str(axis) + ' ' + str(vel) + b'\n'
        self.ser_write(buf)

    def driver_read(self, prop):
        buf = b'r ' + prop + b'\n'
        self.ser_write(buf)
        value = self.ser_read()
        
        if (not value.find('invalid') == -1) or (not value.find('unknown') == -1):
            value = self.driver_read(prop)
        
        return value

    def driver_read_enc(self, axis):
        buf = b'f ' + str(axis) + b'\n'
        self.ser_write(buf)
        value = self.ser_read()
        try:
            pos, vel = value.split(' ') 
        except:
            pos, vel = self.driver_read_enc(axis)
        return float(pos), float(vel)


if __name__ == '__main__':
    ODriveNode().run()
