#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoMarkerDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/aruco_marker_detection/image", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/aruco_marker_detection/marker", Point, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Annotate the image with the corners and IDs of the detected markers
        annotated_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if ids is not None:
            for i, marker_id in enumerate(ids):
                # Draw the corners of the marker
                marker_corners = corners[i][0]
                cv2.line(annotated_image, tuple(marker_corners[0]), tuple(marker_corners[1]), (0, 255, 0), 2)
                cv2.line(annotated_image, tuple(marker_corners[1]), tuple(marker_corners[2]), (0, 255, 0), 2)
                cv2.line(annotated_image, tuple(marker_corners[2]), tuple(marker_corners[3]), (0, 255, 0), 2)
                cv2.line(annotated_image, tuple(marker_corners[3]), tuple(marker_corners[0]), (0, 255, 0), 2)

                # Draw the ID of the marker
                marker_center = np.mean(marker_corners, axis=0)
                cv2.putText(annotated_image, str(marker_id), (int(marker_center[0]), int(marker_center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Publish the center point of the marker
                marker_point = Point()
                marker_point.x = marker_center[0]
                marker_point.y = marker_center[1]
                marker_point.z = 0
                self.marker_pub.publish(marker_point)

        # Publish the annotated image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main():
    rospy.init_node("aruco_marker_detector", anonymous=True)
    detector = ArucoMarkerDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

