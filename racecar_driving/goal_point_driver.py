#!/usr/bin/env python

import math
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped

from final_challenge.msg import StopSign, Point2D


class GoalPointDriver():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        rospy.Subscriber("/goal_point_topic", Point2D,
                         self.relative_cone_callback)

        # set in launch file; different for simulator vs racecar
        DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
                                         AckermannDriveStamped, queue_size=10)
        self.x_offset = 0.0 #rospy.get_param("visual_servoing/x_offset")
        self.y_offset = 0.0 #rospy.get_param("visual_servoing/y_offset")

        self.parking_distance = 0.1  # meters; try playing with this number!
        self.error_threshold = 0 # how close or far from the goal we can be without needing to adjust
        self.relative_x = 0
        self.relative_y = 0

        # NEW SHIT FOR FINAL CHALLENGE
        self.detect_stop = True
        rospy.Subscriber("/stop_signs", StopSign, queue_size=10)
        self.cumulative_error = 0.0


    def relative_cone_callback(self, msg):


        # testing PD control parameters
        kp = 0.6#0.4
        kd = 0.6#0.35
        ki = 0#0.1

        self.relative_x = msg.x_pos
        self.relative_x = 0.35
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.cumulative_error += self.relative_y
        angle = np.arctan2(self.relative_y, self.relative_x)
        drive_angle = kp * self.relative_y + kd * angle + ki * self.cumulative_error


        #rospy.loginfo("x = "+str(self.relative_x)+", y = "+str(self.relative_y))
        #rospy.loginfo("car moves at angle "+str(angle)+" and speed "+str(1))

        drive_cmd.drive.steering_angle = drive_angle
        drive_cmd.drive.speed = 4
        self.drive_pub.publish(drive_cmd)
        #self.error_publisher()



if __name__ == '__main__':
    try:
        rospy.init_node('GoalPointDriver', anonymous=True)
        GoalPointDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
