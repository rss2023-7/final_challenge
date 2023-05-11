#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from final_challenge.msg import Point2D
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

# pixels
PTS_IMAGE_PLANE = [[420, 329],
                   [252, 329],
                   [200, 292],
                   [420, 235]]
# inches
PTS_GROUND_PLANE = [[19.5, -5.25],
                    [19.5, 5.0],
                    [29.0, 10.5],
                    [70.75, -16.5]]

METERS_PER_INCH = 0.0254

class CoordinateConverter:

    # coordinate converter topic
    # --> fed by lane_detection
    COORDINATE_CONVERTER_TOPIC = "/coordinate_converter"

    # goal point topic
    # --> feeds driver
    GOAL_POINT_TOPIC = "/goal_point_topic"

    def __init__(self):

        self.click_sub = rospy.Subscriber(self.COORDINATE_CONVERTER_TOPIC, Point2D, self.coordinate_callback)
        self.goal_point_pub = rospy.Publisher(self.GOAL_POINT_TOPIC, Point2D, queue_size=1)
        np_pts_ground = np.float32((np.array(PTS_GROUND_PLANE) * METERS_PER_INCH)[:, np.newaxis, :])
        np_pts_image = np.float32((np.array(PTS_IMAGE_PLANE) * 1.0)[:, np.newaxis, :])
        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)
        #print("ready to homograph")


    def coordinate_callback(self, coordinate_msg):
        #print("converting coords")
        try:
            u = coordinate_msg.x_pos
            v = coordinate_msg.y_pos
        except:
            u = coordinate_msg.x
            v = coordinate_msg.y
        #rospy.loginfo("image coordinates")
        #rospy.loginfo(u)
        #rospy.loginfo(v)
        x, y = self.transformUvToXy(u, v)
        #rospy.loginfo("output coordinates")
        #rospy.loginfo(x)
        #rospy.loginfo(y)
        relative_xy_msg = Point2D()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y
        self.goal_point_pub.publish(relative_xy_msg)


    def transformUvToXy(self, u, v):
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y


if __name__ == '__main__':
    try:
        rospy.init_node("coordinate_converter")
        coordinate_converter = CoordinateConverter()
        print("spinning")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("o no")
        pass
