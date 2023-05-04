import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
# from DetectStop.msg import Bool
from std_msgs.msg import Bool
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = None # TODO probably want to publish to Ackermann Drive
        self.camera_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.camera_callback)

        self.detect_stop_sub = rospy.Subscriber("/detect_stop", Bool, self.flag_callback)
        self.detect_stop = True

    def camera_callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        # reset self.detect_stop if no sign detected
        # might need to make this more robust but this is a good starting point
        if not self.detector.predict(img_msg):
            self.detect_stop = True

        #TODO: add logic for calculating distance to the stop sign
        if self.detect_stop:
            #TODO

    def flag_callback(self, detect_stop):
        self.detect_stop = detect_stop

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()

