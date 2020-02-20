#!/usr/bin/env python2
import sys
import cv2

# CHANGED: added to process info from rosbag
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# CHANGED: added to make Subscriber node
import rospy
from sensor_msgs.msg import Image

# NOTE: run with: $ rosrun iarc_forebrain helmet_tracker_logic.py webcam


# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
# def detect_helmet_coordinates(image, visualize=True):
#     """
#     Detects the pixel coordinates of the helmet in the image feed
#
#     Input: OpenCV image
#     Output: tuple of x- and y- coordinates of the center of the green helmet in the display
#     """
#     if visualize:
#         cv2.imshow('Raw Image', image)
#
#     # TODO(you) Fill this out to "threshold" the image to show only the color you want, display that thresholded image,
#     # and figure out the coordinates of the center of the blob of pixels.
#
#     return (0, 0)  # (x, y) measured in pixels


#####################
# DETECT CHESSBOARD #
#####################

# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
def detect_chessboard_coordinates(image, visualize=True):
    """
    Detects the chessboard in the image feed

    Input: OpenCV image
    Output: tuple of x- and y- coordinates of the corners of the chessboard in the display
    """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # processing
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if visualize:
        cv2.imshow('Raw Image', gray_img)

    # Find the chessboard corners
    # ret, corners = cv2.findChessboardCorners(gray, (8, 8), None)
    # corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


    corners = cv2.goodFeaturesToTrack(gray_img,50,0.01,10)
    corners = np.int0(corners)

    for i in corners:
        x,y = i.ravel()
        cv2.circle(image,(x,y),3,(0,0,255),-1)

    cv2.imshow('Corners',image)
    cv2.waitKey(1)

    return corners
    # return (0, 0)  # (x, y) measured in pixels


###################
# SUBSCRIBER NODE #
###################

# get bag data
# types: sensor_msgs/Image
# topics: /usb_cam/image_raw

def callback(data):
    try:
        bridge = CvBridge()
        # to convert a ROS image message into an cv::Mat,
        # module cv_bridge.CvBridge provides the following function:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Process it
        coordinates = detect_chessboard_coordinates(cv_image)
        print("rosbag", coordinates)

    except CvBridgeError as e:
        print(e)



def listener():
    rospy.init_node('node_name')
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



########
# MAIN #
########

if __name__ == '__main__':
    listener()




### Test code for processing data directly from the computer webcam or a saved image file
# if __name__ == '__main__':
#     if len(sys.argv) == 2:
#         filename = sys.argv[1]
#         if filename == 'webcam':
#             # Process the webcam in realtime
#             camera = cv2.VideoCapture(0)  # "0" here means "the first webcam on your system"
#
#             while True:
#                 # Get the latest image from the webcam
#                 ok, image = camera.read()
#                 if not ok:
#                     print("Unable to open webcam...")
#                     break
#
#                 # Process it
#                 coordinates = detect_helmet_coordinates(image)
#                 print(coordinates)
#
#                 # Exit if the "Esc" key is pressed
#                 key = cv2.waitKey(1)
#                 if key == 27:
#                     break
#
#         # CHANGED: added to process info from rosbag
#         elif filename.endswith(".bag"):
#
#             # get bag data
#             # types: sensor_msgs/Image
#             # topics: /usb_cam/image_raw
#             # while True:
#             bag = rosbag.Bag(filename)
#
#             bridge = CvBridge()
#
#             # TODO: in real time
#             for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
#
#                 try:
#                     # to convert a ROS image message into an cv::Mat,
#                     # module cv_bridge.CvBridge provides the following function:
#                     cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#
#                     # Process it
#                     coordinates = detect_chessboard_coordinates(cv_image)
#                     print("rosbag", coordinates)
#
#                 except CvBridgeError as e:
#                     print(e)
#
#                 # Exit if the "Esc" key is pressed
#                 key = cv2.waitKey(1)
#                 if key == 27:
#                     bag.close()
#                     break
#
#             bag.close()
#
#         else:
#             # Read data from an image file
#             image = cv2.imread(filename)
#             coordinates = detect_helmet_coordinates(image)
#             print(coordinates)
#             cv2.waitKey(0)
#
#     else:
#         print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')
#         # CHANGED: added to process info from rosbag
#         print('Usage: rosrun iarc_forebrain gati_cv_identify_chessboard.py <bag-file.bag>')
