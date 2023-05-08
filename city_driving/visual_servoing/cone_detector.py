#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        
        # for final challenge
        
        # 1. crop img to be bottom 15-40%
        cropped_img = image[int(image_height*.4):int(image_height*.85),:,:]
        
        # 2. apply Hough transform to img
        dst = cv.Canny(image, 50, 200, None, 3)
        lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
        
        # 4. determine goal pixel point

        image_height = image.shape[0]
        image_crop = image[int(image_height*.5):int(image_height*.85),:,:]

        # Call your color segmentation algorithm
        bbox = cd_color_segmentation(image_crop, None)

        # Get the center pixel on the bottom
        u = (bbox[0][0] + bbox[1][0]) / 2
        v = bbox[1][1] + int(image_height*.5)

        # Publish the pixel (u, v) to the /relative_cone_px topic
        cone_msg = ConeLocationPixel()
        cone_msg.u = u
        cone_msg.v = v
        self.cone_pub.publish(cone_msg)    
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
<<<<<<< HEAD
        debug_image = cv2.rectangle(image_crop, bbox[0], bbox[1], (0, 255, 0), 2)
=======
        debug_image = cv2.rectangle(image, bbox[0], bbox[1], (0, 255, 0), 2)
>>>>>>> b8b68970bf578f79149fdd14bb5c5c0d8c4995aa

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        #rospy.loginfo("ConeDetector: publishing debug image")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        #rospy.loginfo("spinning")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
