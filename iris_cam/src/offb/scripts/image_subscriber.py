#!/usr/bin/env python3

'''
This is a boilerplate script that contains an example of how to subscribe to a rostopic containing camera frames
and store it into an OpenCV image for further image processing tasks.
You can use this code snippet as a starting point and add your own code.

This python file runs a ROS node named "marker_detection" which detects a moving ArUco marker.
This node subscribes to and publishes the following topics:
	Subscriptions:					Publications:
	/camera/rgb/image_raw			/marker_info
'''

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
#from offb.msg import Marker


class ImageProc:

    def __init__(self):
        rospy.init_node('marker_detection')  # Initialize ROS node

        self.marker_pub = rospy.Publisher('/marker_info', Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        self.img = np.empty([])  # This will contain the image frame from the camera
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Convert the image to OpenCV standard image
        except CvBridgeError as e:
            print(e)
            return

        # Perform marker detection on self.img
        # Your marker detection code goes here

        # Assuming you have obtained the marker information as a result of detection
        marker_info = "Marker detected"  # Replace this with the actual marker information

        # Create a new image with the marker info overlay
        img_with_overlay = self.img.copy()
        cv2.putText(img_with_overlay, marker_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Publish the image with overlay
        try:
            image_msg = self.bridge.cv2_to_imgmsg(img_with_overlay, "bgr8")
            self.marker_pub.publish(image_msg)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    image_proc_obj = ImageProc()
    rospy.spin()
