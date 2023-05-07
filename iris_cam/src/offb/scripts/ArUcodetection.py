import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from visualization_msgs.msg import Marker, MarkerArray



bridge = CvBridge()

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Display the image in a separate window
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('arucodetection', anonymous=True)
    image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    marker_pub = rospy.Publisher('marker_positions', MarkerArray, queue_size=10)

    # Initialize OpenCV camera capture
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Initialize ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters_create()

    while not rospy.is_shutdown():
        # Capture frame from camera
        ret, frame = cap.read()

        # Detect ArUco markers in the frame
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        # Publish marker positions to ROS topic
        marker_array = MarkerArray()
        for i in range(len(marker_ids)):
            marker = Marker()
            marker.id = marker_ids[i][0]
            marker.header.frame_id = "camera"
            marker.pose.position.x = marker_corners[i][0][0][0]
            marker.pose.position.y = marker_corners[i][0][0][1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        marker_pub.publish(marker_array)

        # Display the image in a separate window
        cv2.imshow("Image Window", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()

