#!/usr/bin/env python2
import sys
import cv2

# CHANGED: added to process info from rosbag
import rosbag
from cv_bridge import CvBridge, CvBridgeError

# CHANGED: added to make Subscriber node
import rospy
from sensor_msgs.msg import Image

# CHANGED: added for processing step
import numpy as np
import math

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

    #################
    # OTHER METHODS #
    #################

    # # ADAPTIVE BINARIZATION
    # adaptive_thresh = cv2.adaptiveThreshold(gray_image,255, \
    #                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

    #####################
    # WHERE ARE CORNERS #
    #####################

    # GREY
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # GLOBAL BINARIZATION
    ret,global_thresh = cv2.threshold(gray_image,170,235,cv2.THRESH_BINARY)

    # ADAPTIVE BINARIZATION
    # adaptive_thresh = cv2.adaptiveThreshold(gray_image,255, \
    #                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

    ############ OR DO

    # # # GET CORNERS -- GOOD FEATURES TO TRACK
    corners = cv2.goodFeaturesToTrack(gray_image,50,0.01,10)
    corners = np.int0(corners)

    ############

    # FIND CONTOURS
    im2, contours, hierarchy = cv2.findContours(global_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # MAKE BLANK BLACK IMAGE
    mask = np.zeros_like(gray_image)
    # DRAW CONTOURS ONTO MASK
    cv2.drawContours(mask, contours, -1, 255, 10)

    ############ again

    # FIND CONTOURS
    im2, contours2, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # MAKE BLANK BLACK IMAGE
    mask2 = np.zeros_like(gray_image)
    # DRAW FILLED CONTOUR ONTO MASK
    cnts = contours2[1]
    cv2.drawContours(mask2, contours2, 1, 255, -1)

    ############

    # PASTE CHESSBOARD ONTO BLACK BACKGROUND
    out = np.zeros_like(gray_image)
    out[mask2 == 255] = gray_image[mask2 == 255]

    ############

    # REDO BINARY THRESHOLD -- CAN REMOVE STEP LATER
    ret, new_thresh = cv2.threshold(out,170,235,cv2.THRESH_BINARY)
    # CANNY EDGE DETECTION
    # image, minVal, maxVal, size of Sobel kernel, L2gradient boolean
    canny_edges = cv2.Canny(new_thresh, 200, 255)

    # HOUGH LINE TRANSFORM (INFER LINES IN IMAGE, MAP GRID)
    #  Standard Hough Line Transform
    rho_h, theta_h, thresh_h = 1, np.pi / 180, 110
    lines = cv2.HoughLines(canny_edges, rho_h, theta_h, thresh_h)

    intersections = None

    # Draw the lines
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(out, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        # CALCULATE INTERSECTIONS
        # Segment your lines into two classes based on their angle.
        segmented = segment_by_angle_kmeans(lines)
        # Calculate the intersections of each line in one class to the lines in the other classes.
        intersections = segmented_intersections(segmented)

    ###############
    # GET SQUARES #
    ###############


    #########################
    # ARE PIECES IN SQUARES #
    #########################



    ##############################
    # IDENTIFY PIECES IN SQUARES #
    ##############################



    #############
    # VISUALIZE #
    #############

    visual = out
    # visual = gray_image
    # visual = adaptive_thresh

    visual = cv2.cvtColor(visual, cv2.COLOR_GRAY2BGR)
    # visual = out

    # if visualize:
    #     cv2.imshow('Image', visual)
    #     cv2.waitKey(1)

    if visualize:
        for i in corners:
            x,y = i.ravel()
            cv2.circle(visual,(x,y),3,(0,255,0),-1)
        if intersections is not None:
            for i in intersections:
                x,y = i[0]
                cv2.circle(visual,(x,y),3,(0,0,255),1)
        cv2.imshow('Corners',visual)
        cv2.waitKey(10)

    ##########
    # RETURN #
    ##########

    return 0

    ####################
    # HELPER FUNCTIONS #
    ####################

from collections import defaultdict
def segment_by_angle_kmeans(lines, k=2, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle
    to segment `k` angles inside `lines`.

    the theta returned is between 0 and 180 degrees,
    and lines around 180 and 0 degrees are similar
    (they are both close to horizontal lines),
    so we need some way to get this periodicity in kmeans

    So we plot the angle on the unit circle,
    but multiply the angle by two,
    then the angles originally around 180 degrees will become
    close to 360 degrees and thus will have x, y values on the unit circle
    near the same for angles at 0

    https://stackoverflow.com/questions/46565975/
    find-intersection-point-of-two-lines-drawn-using-houghlines-opencv/49590801
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get('attempts', 10)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
        for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented


def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def segmented_intersections(lines):
    """Finds the intersections between groups of lines."""

    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2))

    return intersections



###################
# SUBSCRIBER NODE #
###################

# get bag data
# types: sensor_msgs/Image
# topics: /usb_cam/image_raw, /alexa/image_raw

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
    # rospy.Subscriber('/alexa/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



########
# MAIN #
########

if __name__ == '__main__':
    listener()
