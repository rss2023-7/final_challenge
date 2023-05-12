#!/usr/bin/env python

import cv2
import numpy as np
import math



def read_image(image_path):
    image = cv2.imread(image_path)
    print(image)
    cv2.imshow('original', image)
    return image


def perform_hough_transform(src, dst, cdst, cdstP):
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            print(l)
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
    cv2.imshow("Source", src)
    cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    return linesP

def preprocess_image(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the color white
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([179, 30, 255])

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv_image, lower_white, upper_white)

    # Bitwise-AND the mask and the original image
    result = cv2.bitwise_and(image, image, mask=mask)
    return result
def crop_image(image):
    height, width, _ = image.shape
    return image[int(height/2):height, int(width/2):width]

def perform_canny_edge_detection(image):

    # Canny on masked image
    canny_masked = cv2.Canny(image, 50, 200, None, 3)

    cdst_masked = cv2.cvtColor(canny_masked, cv2.COLOR_GRAY2BGR)

    cdstP_masked = np.copy(cdst_masked)

    return canny_masked, cdst_masked, cdstP_masked

image = read_image("/home/racecar/racecar_ws/src/final_challenge/media/test_images/img_from_car.png")
#image = read_image("IMG_8500.png")

#image = cv2.LoadImage("../media/test_images/IMG_8485.png")




preprocessed_image = preprocess_image(image)



result_cropped = crop_image(preprocessed_image)
# crop the resulting image to only focus on the right half of the image and the bottom 50%




canny_masked, cdst_masked, cdstP_masked = perform_canny_edge_detection(result_cropped)


print(perform_hough_transform(result_cropped, canny_masked, cdst_masked, cdstP_masked))



cv2.waitKey(0)



   








