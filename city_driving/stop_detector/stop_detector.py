import cv2
import rospy

import sys

sys.path.append('..')

import numpy as np
from sensor_msgs.msg import Image
from final_challenge.msg import StopSign  # TODO check if this is working :(
from detector import StopSignDetector
from homography import HomographyTransformer

a = HomographyTransformer()
print(type(a))


class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/stop_signs", StopSign, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

        self.detect_stop = True

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:, :, :-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        # reset self.detect_stop if no sign detected
        # might need to make this more robust but this is a good starting point
        if not self.detector.predict(img_msg):
            self.detect_stop = True

        # logic for calculating distance to the stop sign
        if self.detect_stop:
            visible, bounding_box = self.detector.predict(rgb_img)
            # in_sight = self.detector.predict(rgb_img)[0]
            # bounding_box = self.detector.draw_box(rgb_img)

            if visible:
                u = (bounding_box[0][0] + bounding_box[1][0]) / 2
                v = (bounding_box[0][1] + bounding_box[1][1]) / 2
                transformer = HomographyTransformer()
                distance = transformer.transformUvToXy(u,v)

                Sign = StopSign()
                Sign.visible = True
                Sign.distance = distance
                self.publisher.publish(Sign)

            else:
                Sign = StopSign()
                Sign.visible = False
                Sign.distance = 999.9
                self.publisher.publish(Sign)




if __name__ == "__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
