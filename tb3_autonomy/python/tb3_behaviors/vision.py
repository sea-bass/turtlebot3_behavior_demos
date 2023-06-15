"""
Vision behaviors for TurtleBot3
"""

import cv2
import cv_bridge
import rclpy
from rclpy.duration import Duration
import py_trees
from sensor_msgs.msg import Image

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Define HSV color space thresholds
hsv_threshold_dict = {
    "red": ((160, 220, 0), (180, 255, 255)),
    "green": ((40, 220, 0), (90, 255, 255)),
    "blue": ((100, 220, 0), (150, 255, 255)),
}


class LookForObject(py_trees.behaviour.Behaviour):
    """
    Gets images from the robot and looks for object using
    simple HSV color space thresholding and blob detection.
    """

    def __init__(self, name, color, node, img_timeout=3.0, visualize=True):
        super(LookForObject, self).__init__(name)
        self.color = color
        self.node = node
        self.hsv_min = hsv_threshold_dict[color][0]
        self.hsv_max = hsv_threshold_dict[color][1]
        self.img_timeout = Duration(nanoseconds=img_timeout * 1e9)
        self.viz_window_name = "Image with Detections"
        self.visualize = visualize
        if self.visualize:
            plt.figure(1)
            plt.axis("off")
            plt.title(self.viz_window_name)
            plt.ion()

    def initialise(self):
        """Starts all the vision related objects"""
        self.bridge = cv_bridge.CvBridge()
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = 100
        params.maxArea = 100000
        params.filterByArea = True
        params.filterByColor = False
        params.filterByInertia = False
        params.filterByConvexity = False
        params.thresholdStep = 50
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.start_time = self.node.get_clock().now()
        self.latest_img_msg = None
        self.img_sub = self.node.create_subscription(
            Image, "/camera/image_raw", self.img_callback, 10
        )

    def update(self):
        """Looks for at least one object detection using HSV thresholding"""
        # Wait for an image message and handle failure case
        now = self.node.get_clock().now()
        if self.latest_img_msg is None:
            if now - self.start_time < self.img_timeout:
                return py_trees.common.Status.RUNNING
            else:
                self.logger.info("Image timeout exceeded")
                return py_trees.common.Status.FAILURE

        # Process the image
        img = self.bridge.imgmsg_to_cv2(self.latest_img_msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        keypoints = self.detector.detect(mask)

        # Visualize, if enabled
        if self.visualize:
            labeled_img = cv2.drawKeypoints(
                img,
                keypoints,
                None,
                (255, 0, 0),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )
            # OpenCV visualization
            cv2.destroyAllWindows()
            cv2.imshow(self.viz_window_name, labeled_img)
            cv2.waitKey(100)

            # Matplotlib visualization
            # plt.imshow(labeled_img[:,:,::-1])
            # plt.pause(0.1)

        # If there were no failures along the way, the behavior was successful
        if len(keypoints) == 0:
            self.logger.info("No objects detected")
            return py_trees.common.Status.FAILURE
        for k in keypoints:
            self.logger.info(f"Detected object at [{k.pt[0]}, {k.pt[1]}]")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        self.img_sub = None
        self.latest_img_msg = None

    def img_callback(self, msg):
        # self.logger.info("Image received")
        self.latest_img_msg = msg
