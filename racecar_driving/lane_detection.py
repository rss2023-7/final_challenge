#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from final_challenge.msg import Point2D #To check: change msg file name

class LaneDetector:

    run_once = False

    def __init__(self):
        # Initialize the node and create a publisher for processed images
        rospy.init_node('lane_detection_node')
        self.bridge = CvBridge()
        # Subscribe to the raw image
        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.image_callback)
        #Pulish Line Coordinates
        self.line_coordinates_pub = rospy.Publisher('/lane_error', Float32, queue_size=1)
        self.cropped_img_pub = rospy.Publisher('/cropped_img', Image, queue_size=1)
        COORDINATE_CONVERTER_TOPIC = "/coordinate_converter"
        self.midpoint_pub = rospy.Publisher(COORDINATE_CONVERTER_TOPIC, Point2D, queue_size = 1)

        

    def read_image(self, image_path):
        image = cv2.imread(image_path)
        #cv2.imshow('original', image)
        return image


    def perform_hough_transform(self, src, dst, cdst, cdstP):
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        #cv2.imshow("Source", src)
        #cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        return linesP

    def preprocess_image(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the color white
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([179, 30, 255])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Bitwise-AND the mask and the original image
        result = cv2.bitwise_and(image, image, mask=mask)
        return result
    def crop_image(self, image):
        height, width, _ = image.shape
        return image[int(0.4*height):int(0.85*height), 0:width]

    def perform_canny_edge_detection(self, image):

        # Canny on masked image
        canny_masked = cv2.Canny(image, 50, 200, None, 3)

        cdst_masked = cv2.cvtColor(canny_masked, cv2.COLOR_GRAY2BGR)

        cdstP_masked = np.copy(cdst_masked)

        return canny_masked, cdst_masked, cdstP_masked


    def image_callback(self, image_message):
        
        if image_message is None:
            return
        
        try:
            image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        preprocessed_image = self.preprocess_image(image)
        result_cropped = self.crop_image(preprocessed_image)

        canny_masked, cdst_masked, cdstP_masked = self.perform_canny_edge_detection(result_cropped)

        #publish line coordinates output from perform_hough_transform function to /line_coordinates topic
        lines = self.perform_hough_transform(result_cropped, canny_masked, cdst_masked, cdstP_masked)

        # don't try any processing if no lines detected
        if lines is None:
            return


        ########################### EXPERIMENTAL ############################

        [img_height, img_width] = [result_cropped.shape[0], result_cropped.shape[1]]
        current_left_line_index = None

        # this sweep checks all lines to find the best match for left case 1
        for i in range(len(lines)):

            line = lines[i][0]

            # if line starts in the left side of the image...
            if line[0] <= img_width / 2:

                # if line starts somewhere along the bottom of the image
                if abs(img_height - line[1]) < 5:

                    if current_left_line_index is None:
                        current_left_line_index = i
                
                    # if there are multiple case 1 lines (there shouldn't be), break tie
                    # by selecting the innermost line (greater x value)
                    else:
                        if line[0] > lines[current_left_line_index][0][0]:
                            current_left_line_index = i

        # if we didn't find a case 1, we must have a left case 2
        # --> perform a sweep to find best line
        if current_left_line_index is None:


            for i in range(len(lines)):

                line = lines[i][0]

                # if line starts in the bottom half of the image...
                if line[1] >= img_height / 2:


                    # if line starts somewhere along the left edge of the image (5 is tunable threshold)
                    if line[0] < 5:


                        if current_left_line_index is None:
                            current_left_line_index = i
                    
                        # if there are multiple case 1 lines (there shouldn't be), break tie
                        # by selecting the innermost line (greater y value)
                        else:
                            if line[1] > lines[current_left_line_index][0][1]:
                                current_left_line_index = i

        current_right_line_index = None

        # this sweep checks all lines to find the best match for right case 1
        for i in range(len(lines)):

            line = lines[i][0]

            # if line ends in the right side of the image...
            if line[2] >= img_width / 2:

                # if line starts somewhere along the bottom of the image
                if abs(img_height - line[3]) < 5:

                    if current_right_line_index is None:
                        current_right_line_index = i
                
                    # if there are multiple case 1 lines (there shouldn't be), break tie
                    # by selecting the innermost line (lesser x value)
                    else:
                        if line[2] < lines[current_right_line_index][0][2]:
                            current_right_line_index = i

        # if we didn't find a case 1, we must have a left case 2
        # --> perform a sweep to find best line
        if current_right_line_index is None:

            for i in range(len(lines)):

                line = lines[i][0]

                # if line ends in the bottom half of the image...
                if line[2] > img_height / 2:

                    # if line ends somewhere along the right edge of the image
                    if abs(img_width - line[2]) < 5:

                        if current_right_line_index is None:
                            current_right_line_index = i
                    
                        # if there are multiple case 1 lines (there shouldn't be), break tie
                        # by selecting the innermost line (lesser y value)
                        else:
                            if line[3] > lines[current_right_line_index][0][3]:
                                current_right_line_index = i


        ########################### EXPERIMENTAL ############################


        if current_left_line_index is None:
            #rospy.loginfo("LEFT line was None, returning")
            return
        if current_left_line_index is None or current_right_line_index is None:
            #rospy.loginfo("line was None, returning")
            return

        #img_with_line = cv2.line(result_cropped, (lines[current_left_line_index][0][0],lines[current_left_line_index][0][1]), (lines[current_left_line_index][0][2], lines[current_left_line_index][0][3]), (255,0,0), 2)
        #img_with_line = cv2.line(img_with_line, (lines[current_right_line_index][0][0],lines[current_right_line_index][0][1]), (lines[current_right_line_index][0][2], lines[current_right_line_index][0][3]), (255,0,0), 2)
        #img_with_line= self.bridge.cv2_to_imgmsg(img_with_line, "bgr8")
        #self.cropped_img_pub.publish(img_with_line)

        # extract the x and y coordinates of the left and right lines
        left_x1 = lines[current_left_line_index][0][0]
        left_y1 = lines[current_left_line_index][0][1]
        left_x2 = lines[current_left_line_index][0][2]
        left_y2 = lines[current_left_line_index][0][3]

        right_x1 = lines[current_right_line_index][0][0]
        right_y1 = lines[current_right_line_index][0][1]
        right_x2 = lines[current_right_line_index][0][2]
        right_y2 = lines[current_right_line_index][0][3]


        def get_intersection_with_horizon(x1, y1, x2, y2):
            if x2 - x1 == 0:  # To avoid division by zero
                return None

            m = (y2 - y1) / (x2 - x1)  # Calculate slope
            c = y1 - m * x1  # Calculate y-intercept

            x_intersection = -c / m  # Calculate x when y=0
            return x_intersection

        def get_simple_intersect(line_1_endpoints, line_2_endpoints):

            one_x_1, one_y_1, one_x_2, one_y_2 = line_1_endpoints
            two_x_1, two_y_1, two_x_2, two_y_2 = line_2_endpoints

            # check for division by 0
            if one_x_2 - one_x_1 == 0 or two_x_2 - two_x_1 == 0:
                return

            line_1_slope = float(one_y_2 - one_y_1) / float(one_x_2 - one_x_1)
            line_2_slope = float(two_y_2 - two_y_1) / float(two_x_2 - two_x_1)

            line_1_intercept = one_y_1 - line_1_slope * one_x_1
            line_2_intercept = two_y_1 - line_2_slope * two_x_1

            midpoint_coordinate = (line_1_intercept - line_2_intercept) / (line_2_slope - line_1_slope)

            # add offset based on slope difference if needed
            if abs(line_1_slope) > 2 * abs(line_2_slope):
                midpoint_coordinate += 15
            elif abs(line_2_slope) > 2 * abs(line_1_slope):
                midpoint_coordinate -= 15

            # offset due to using the left camera
            midpoint_coordinate -= 0

            return midpoint_coordinate


        x_midpoint_coordinate = int(get_simple_intersect(lines[current_left_line_index][0], lines[current_right_line_index][0]))

        original_img_height = image.shape[1]
        y_midpoint_coordinate = int (original_img_height * .05)

        #result_cropped_midpoint = cv2.circle(img_with_line, (x_midpoint_coordinate, y_midpoint_coordinate), radius=2, color=(0, 0, 255), thickness=-1)
        #img_with_midpoint = self.bridge.cv2_to_imgmsg(result_cropped_midpoint, "bgr8")
        #self.cropped_img_pub.publish(img_with_midpoint)   

        y_midpoint_coordinate += original_img_height

        point_message = Point2D()

        point_message.x_pos = x_midpoint_coordinate
        point_message.y_pos = y_midpoint_coordinate

        #rospy.loginfo("we have a midpoint = "+str(x_midpoint_coordinate)+", "+str(y_midpoint_coordinate))
        self.midpoint_pub.publish(point_message)
    
if __name__ == '__main__':
    try:
        lane_detector = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

