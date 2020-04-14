#!/usr/bin/env python
import rospy
from Queue import Queue, Empty
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from numpy import abs


class Synchronisation_node():

    def __init__(self):
        self.left_queue = Queue(4)
        self.right_queue = Queue(4)
        self.left = list()
        self.right = list()
        rospy.init_node("synchronisation_node")
        rospy.Subscriber("left/image_raw", Image, self.image_left_callback, queue_size=4)
        rospy.Subscriber("right/image_raw", Image, self.image_right_callback, queue_size=4)
        self.left_pub = rospy.Publisher("stereo/left/image_raw", Image, queue_size=4)
        self.right_pub = rospy.Publisher("stereo/right/image_raw", Image, queue_size=4)
        self.fast_timer = rospy.Timer(rospy.Duration(1.0 / 300.0), self.synchronisation)
        rospy.spin()


    def image_left_callback(self, msg):
        self.left_queue.put(msg)

    def image_right_callback(self, msg):
        self.right_queue.put(msg)

    def synchronisation(self, timer_event):
        try:
            self.left.append(self.left_queue.get())
        except Empty:
            pass
        try:
            self.right.append(self.right_queue.get())
        except Empty:
            pass
        if len(self.right) >= 3 or len(self.left) >= 3:
            min_d = 1.0
            ind_l = 0
            ind_r = 0
            for i in range(len(self.left)):
                for j in range(len(self.right)):
                    diff = self.left[i].header.stamp.secs + self.left[i].header.stamp.nsecs * 10**(-9) - self.right[j].header.stamp.secs - self.right[j].header.stamp.nsecs * 10**(-9)
                    diff = abs(diff)
                    if diff <= min_d:
                        min_d = diff
                        ind_l = i
                        ind_r = j
            now = rospy.get_rostime()
            self.left[ind_l].header.stamp.secs = now.secs
            self.left[ind_l].header.stamp.nsecs = now.nsecs
            self.right[ind_r].header.stamp.secs = now.secs
            self.right[ind_r].header.stamp.nsecs = now.nsecs
            self.left_pub.publish(self.left.pop(ind_l))
            self.right_pub.publish(self.right.pop(ind_r))
            del self.left[0:ind_l]
            del self.right[0:ind_r]


if __name__ == '__main__':
    Synchronisation_node()