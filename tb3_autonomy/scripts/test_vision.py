#!/usr/bin/env python3

"""
Script that tests a simple color thresholding vision approach.

Example usage:
  Python: python3 test_vision.py --min_h 0 --max_h 30 --min_s 0 --max_s 255 --min_v 0 --max_v 255
  rosrun: rosrun tb3_autonomy test_vision.py --min_h 0 --max_h 30 --min_s 220 --max_s 255 --min_v 0 --max_v 255

For reference, these are some approximate thresholds on the H channel.
  Red: 0 - 30
  Green: 40 - 80
  Blue: 100 - 140
Since the blocks are highly saturated, the default S channel thresholds are tight (220-255)
"""

import cv2
import cv_bridge
import rospy
import argparse
from sensor_msgs.msg import Image

# Parse command-line arguments
parser = argparse.ArgumentParser(description="HSV Color Thresholding test script")
parser.add_argument("--min_h", type=int, default=100)
parser.add_argument("--max_h", type=int, default=140)
parser.add_argument("--min_s", type=int, default=220)
parser.add_argument("--max_s", type=int, default=255)
parser.add_argument("--min_v", type=int, default=0)
parser.add_argument("--max_v", type=int, default=255)
args = parser.parse_args()
min_bounds = (args.min_h, args.min_s, args.min_v)
max_bounds = (args.max_h, args.max_s, args.max_v)

# Define vision related variables
bridge = cv_bridge.CvBridge()
cv2.namedWindow("Image with Detections", cv2.WINDOW_NORMAL)
params = cv2.SimpleBlobDetector_Params()
params.minArea = 100
params.maxArea = 100000
params.filterByArea = True
params.filterByColor = False
params.filterByInertia = False
params.filterByConvexity = False
params.thresholdStep = 50
detector = cv2.SimpleBlobDetector_create(params)

def img_callback(msg):
    """ Image topic subscriber callback """
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, min_bounds, max_bounds)
    keypoints = detector.detect(mask)
    labeled_img = cv2.drawKeypoints(img, keypoints, None, (255,0,0), 
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Image with Detections", labeled_img)
    cv2.waitKey(10)

if __name__=="__main__":
    # Start ROS node and subscriber
    rospy.init_node("test_vision")
    rospy.Subscriber("/camera/rgb/image_raw", Image, img_callback)
    rospy.spin()
