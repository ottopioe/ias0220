import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy

class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('camera_calibration')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        
cap = cv.VideoCapture(0)
 
while(1):
 
    # Take each frame
    frame = cap.read()
 
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
 
    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
 
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
 
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
 
    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
 
cv.destroyAllWindows()




def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognition()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()