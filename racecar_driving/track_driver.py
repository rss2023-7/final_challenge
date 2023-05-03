import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class TrackDriver:
    """
    Class to handle driver controls for car on race track.

    Uses PD control on provided images to adjust steering angle and speed.
    """

    # topic which publishes by how much the car has deviated from the center of the
    # race track lane.
    LANE_ERROR_TOPIC = rospy.get_param("final_challenge2023/lane_error_topic")

    # topic which receives drive commands for the car
    DRIVE_TOPIC = rospy.get_param("final_challenge2023/drive_topic")

    # proportional gain constant
    KP_GAIN = 0

    # integral gain constant
    KI_GAIN = 0

    # derivative gain constant
    KD_GAIN = 0

    # loop interval time constant
    DT = 0.05

    # saved previous error for PID control
    previous_error = 0

    # saved previous speed used in calculating new speed
    previous_speed = 0
    

    def __init__(self):
        
        # set up the subscriber that listens to the error
        rospy.Subscriber(self.LANE_ERROR_TOPIC, Float32, self.lane_error_callback)

        # set up the drive command publisher
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def lane_error_callback(self, lane_error_msg):
        """
        Handles each cycle of driving. Determines driving angle and speed based on
        the error of the car within its lane. Publishes the drive command directly.

        Speed could be dependent on curve of race track. How could that be known here?
        Do we just expect error ∝ curviness => speed ∝ 1 / error?

            Parameters:
                lane_error_msg (Float32): A float 32 msg
        """

        # calculate the drive angle using PID
        drive_angle = self.calculate_pid_value(lane_error_msg.data)

        # calculate speed based on heuristics
        drive_speed = self.calculate_drive_speed(lane_error_msg.data)

        drive_cmd = AckermannDriveStamped()
        drive_cmd.speed = drive_speed
        drive_cmd.angle = drive_angle
        self.drive_pub.publish(drive_cmd)

    def calculate_pid_value(self, error):
        """
        Computes PID control output. Based on the pseudocode here:
        https://en.wikipedia.org/wiki/PID_controller#Pseudocode

            Parameters:
                error (float): the measured value for PID control

            Returns:
                pid_output (float): the result of performing one iteration of PID
                on inputs.
        """

        proportional = error
        self.integral = self.integral + error * self.DT
        derivative = (error - self.previous_error) / self.DT
        output = self.KP_GAIN * proportional + self.KI_GAIN * self.integral + self.KD_GAIN * derivative

        self.previous_error - error

        return output
    
    def calculate_drive_speed(self, current_speed, error):
        """
        Determines the correct drive speed based on the error of the car
        in its lane.

        I'm thinking we can use simple heuristics here to start. Maybe just
        slow down when our error is above a threshold and speed up when the
        error drops lower?

            Parameters:
                current_speed (float): the current speed of the car
                error (float): the error of the car in its lane

            Returns:
                drive_speed (float): the speed to drive the car
        
        """

        drive_speed = current_speed
        max_speed = 4

        # threshold for error above which speed is decreased and below whic
        # speed is increased
        threshold = 0.1

        if error > threshold:
            drive_speed = 0.9 * current_speed
        elif error < threshold:
            drive_speed = min(1.1 * current_speed, max_speed)

        return drive_speed

if __name__ == "__main__":
    rospy.init_node('track_driver')
    wall_follower = TrackDriver()
    rospy.spin()
