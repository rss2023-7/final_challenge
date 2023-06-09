#!/usr/bin/env python

import math
import rospy
import numpy as np
import time

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

from final_challenge.msg import StopSign


class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
                         self.relative_cone_callback)

        # set in launch file; different for simulator vs racecar
        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
                                         AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
                                         ParkingError, queue_size=10)
        self.x_offset = 0.0 # rospy.get_param("visual_servoing/x_offset")
        self.y_offset = 0.0 # rospy.get_param("visual_servoing/y_offset")

        self.parking_distance = 0.1  # meters; try playing with this number!
        self.error_threshold = 0 # how close or far from the goal we can be without needing to adjust
        self.relative_x = 0
        self.relative_y = 0

        # NEW SHIT FOR FINAL CHALLENGE - CITY DRIVING
        rospy.Subscriber("/stop_signs", StopSign, self.stop_sign_callback)
        self.detect_stop = True
        self.sign_visible = False
        self.distance_to_stop_sign = float('inf')


    def relative_cone_callback(self, msg):

        self.relative_x = msg.x_pos - self.x_offset
        self.relative_y = msg.y_pos + self.y_offset
        if self.relative_y < 0:
            self.relative_y += .2
        else:
            self.relative_y -= .1
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        angle = np.arctan2(self.relative_y, self.relative_x)

        # determine euclidean distance to cone
        dist_to_cone = math.sqrt(self.relative_x ** 2 + self.relative_y ** 2)
        rospy.loginfo("x = "+str(self.relative_x)+", y = "+str(self.relative_y))

        # NEW SHIT FOR FINAL CHALLENGE - STOP SIGNS
        if not self.sign_visible or self.distance_to_stop_sign > 1.4:  # resets the sign detecting
            self.detect_stop = True

        if self.detect_stop and self.sign_visible and 0.75 <= self.distance_to_stop_sign <= 1.0:
            # MAY WANT TO TWEAK THE STOPPING THRESHOLD DEPENDING ON IRL PERFORMANCE
            drive_cmd.drive.steering_angle = 0.0
            drive_cmd.drive.speed = 0.0
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()  # prolly don't need this
            rospy.sleep(.5)  # wait for half a second at the sign
            self.detect_stop = False  # this keeps us from stopping forever

        # OG PARKING CONTROLLER
        else:
            # what to do when the cone (or line) is not in view
            if(self.relative_x < 0 or self.relative_x > 3):
                drive_cmd.drive.steering_angle = 0.0
                drive_cmd.drive.speed = 0.0
            else:
                if (self.relative_x - self.parking_distance) > self.error_threshold:
                    drive_cmd.drive.speed = 1.5
                    drive_cmd.drive.steering_angle = angle
                elif (self.relative_x - self.parking_distance) < -self.error_threshold:
                    drive_cmd.drive.speed = -0.5
                    drive_cmd.drive.steering_angle = -angle
                else:
                    drive_cmd.drive.speed = 0
                    drive_cmd.drive.steering_angle = 0
            #rospy.loginfo("angle to car = "+str(angle)+", x = "+str(self.relative_x)+", y = "+str(self.relative_y))
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()

    def stop_sign_callback(self, msg):
        if msg.visible:
            self.sign_visible = True
            self.distance_to_stop_sign = msg.distance

    def drive_in_circle(self):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = 0.73
        drive_cmd.drive.speed = 0.5
        self.drive_pub.publish(drive_cmd)


    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt(self.relative_x ** 2 + self.relative_y ** 2)
        self.error_pub.publish(error_msg)

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################

        self.error_pub.publish(error_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
