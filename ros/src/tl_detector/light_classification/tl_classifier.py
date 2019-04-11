import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # Transform to HSV and simply count the number of color within the range 
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # red has hue 0 - 10 & 160 - 180 add another filter 
        # TODO  use Guassian mask
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])        

        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask_red_lower = cv2.inRange(hsv_img, lower_red1, upper_red1) 
        mask_red_upper = cv2.inRange(hsv_img, lower_red2, upper_red2) 
        if cv2.countNonZero(mask_red_lower) + cv2.countNonZero(mask_red_upper) > 70:
            return TrafficLight.RED

        lower_yellow = np.array([40.0/360*255, 100, 100])
        upper_yellow = np.array([66.0/360*255, 255, 255])
        mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        if cv2.countNonZero(mask_yellow) > 70:
            return TrafficLight.YELLOW

        lower_green = np.array([90.0/360*255, 100, 100])
        upper_green = np.array([140.0/360*255, 255, 255])
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        if cv2.countNonZero(mask_green) > 70:
            return TrafficLight.GREEN


        return TrafficLight.UNKNOWN