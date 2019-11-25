#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2


bridge = CvBridge()


def image_callback(msg):
	try:
		# Convert your ROS Image message to OpenCV2
		cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
	except CvBridgeError, e:
		print(e)
	else:
		# Save your OpenCV2 image as a jpeg 
		cv2.imshow('frame', cv2_img)
    
    
def main_camera_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('main_camera_node', anonymous=True)
    rospy.Subscriber("porter/camera_main/image_raw", Image, image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	cv2.namedWindow('frame', flags=cv2.WINDOW_AUTOSIZE)
	cv2.startWindowThread()
	main_camera_node()
	cv2.destroyWindow("frame")
