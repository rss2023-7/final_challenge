#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector:

    def __init__(self):
        # Initialize the node and create a publisher for processed images
        rospy.init_node('lane_detection_node')
        self.processed_image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.bridge = CvBridge()

        # Subscribe to the raw image and lane assignment topics
        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.image_callback)

    def color_segmentation(self, img):
        bounding_box = ((0,0),(0,0))

        # Convert the image from BGR to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([18, 255, 255])

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Erode the mask to remove noise
        kernel = np.ones((3, 3), np.uint8)
        eroded_mask = cv2.erode(mask, kernel, iterations=2)

        # Dilate the mask to improve the shape
        dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=2)



        opencv_major_version = int(cv2.__version__.split('.')[0])

        # Call cv2.findContours with the appropriate output format based on the major version
        if opencv_major_version >= 4:
            contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the bounding box of the largest contour
        max_area = 0
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if area > max_area:
                max_area = area
                bounding_box = ((x, y), (x + w, y + h))

        return bounding_box

    def preprocess_image(self, img):
        # Resize the image if necessary
        img = cv2.resize(img, (640, 480))

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use Canny edge detection to find edges in the image
        edges = cv2.Canny(blur, 50, 150)

        return edges

    def detect_lane_lines(self, edges):
        # Apply HoughLinesP to detect lines in the image
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=50)

        return lines

    def segment_lane_lines(self, lines, lane_assignment):
        # Filter the lines based on slope, length, and position
        # Customize the filtering criteria based on your lane assignment
        segmented_lines = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 < lane_assignment * 100 and x2 < (lane_assignment + 1) * 100:
                    slope = (y2 - y1) / (x2 - x1)
                    if abs(slope) < 0.5:
                        segmented_lines.append(line)

        return segmented_lines

    def image_callback(self, image_message):
        try:
            img = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Image preprocessing
        edges = self.preprocess_image(img)

        # Lane detection
        lines = self.detect_lane_lines(edges)

        # Visualize the detected lines
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    img = cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

                    #show the image with the lines drawn
                    cv2.imshow("lane lines", img)

   






if __name__ == '__main__':
    try:
        lane_detector = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

