#!/usr/bin/env python

import cv2
import numpy as np



def color_segmentation(img):
    bounding_box = ((0,0),(0,0))

    # Convert the image from BGR to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([179, 30, 255])

    mask = cv2.inRange(hsv, lower_white, upper_white)

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

def preprocess_image(img):
    # Resize the image if necessary
    img = cv2.resize(img, (640, 480))

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use Canny edge detection to find edges in the image
    edges = cv2.Canny(blur, 50, 150)

    return edges

def detect_lane_lines(edges):
    # Apply HoughLinesP to detect lines in the image
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=50)

    return lines


image = cv2.imread("../media/test_images/IMG_8485.png")
#display the image
cv2.imshow('original', image)

# # apply and show color segmentation
# bounding_box = color_segmentation(image)
# cv2.rectangle(image, bounding_box[0], bounding_box[1], (0, 255, 0), 5)
# cv2.imshow('color segmentation', image)


hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds for the color white
# Hue can be any value, so we set a wide range (0-179)
# Low saturation (0-30) and high value (200-255) for white
lower_white = np.array([0, 0, 200])
upper_white = np.array([179, 30, 255])

# Threshold the HSV image to get only white colors
mask = cv2.inRange(hsv_image, lower_white, upper_white)

# Bitwise-AND the mask and the original image
result = cv2.bitwise_and(image, image, mask=mask)


# crop the resulting image to only focus on the right half of the image and the bottom 65%
height, width, _ = result.shape
result_cropped = result[int(height/2):height, int(width/2):width]


hough_image = image.copy()
preprocess_image = preprocess_image(hough_image)
lines = detect_lane_lines(preprocess_image)
if lines is not None:
    for line in lines:
        for x1, y1, x2, y2 in line:
            hough_image = cv2.line(hough_image, (x1, y1), (x2, y2), (0, 0, 255), 5)

            #show the image with the lines drawn
            cv2.imshow("lane lines", hough_image)
            


# Display the images
cv2.imshow('Original Image', image)
cv2.imshow('HSV Image', hsv_image)
cv2.imshow('Mask', mask)
cv2.imshow('Result', result)
cv2.imshow('Result Cropped', result_cropped)



cv2.waitKey(0)



def image_callback(image_message):
    try:
        img = bridge.imgmsg_to_cv2(image_message, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Image preprocessing
    edges = preprocess_image(img)

    # Lane detection
    lines = detect_lane_lines(edges)

    # Visualize the detected lines
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                img = cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

                #show the image with the lines drawn
                cv2.imshow("lane lines", img)

   








