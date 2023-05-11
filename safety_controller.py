#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class SafetyController:

    SCAN_TOPIC = "/scan"
    DRIVE_INPUT_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"
    DRIVE_OUTPUT_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"
    LASER_DATA = None
    INSTR_CYCLES = 0

    def __init__(self):
        rospy.Subscriber(self.DRIVE_INPUT_TOPIC, AckermannDriveStamped, self.enforce_safety)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.save_laser_data)
        self.safety_controller_pub = rospy.Publisher(self.DRIVE_OUTPUT_TOPIC, AckermannDriveStamped, queue_size=1)
        
        #braking command
        msg = AckermannDriveStamped()
        instr = msg.drive
        instr.steering_angle = 0
        instr.steering_angle_velocity = 0
        instr.speed = 0
        instr.acceleration = 0
        instr.jerk = 0
        self.brake_msg = msg

    def enforce_safety(self, drive_input):
        instr = drive_input.drive
        if instr.speed > 0:
            angle = instr.angle
            scan = self.LASER_DATA
            pts = scan.ranges
            angle_min = scan.angle_min
            angle_increment = scan.angle_increment
            #get dir car is facing and check 10% around it
            midpoint = round((angle - angle_min)/angle_increment)
            start = max(0, round(midpoint - len(pts)/10))
            end = min(len(pts) - 1, round(midpoint + len(pts)/10))
            #check if car is > 1 sec from object
            closest_dist = min(pts[start:end])
            time = 1/instr.speed * closest_dist
            if time <= 1:
                self.safety_controller_pub.publish(self.brake_msg)

    def save_laser_data(self, laser_data):
        self.LASER_DATA = laser_data


if __name__ == "__main__":
    rospy.init_node("safety_controller")
    safety_controller = SafetyController()
    rospy.spin()