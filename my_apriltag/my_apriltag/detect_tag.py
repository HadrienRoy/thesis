#!/usr/bin/env python3

# detect_tag.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from AprilRobotics
# Outputs:
#
# Purpose: To detect AprilTag

import rclpy
from rclpy.node import Node

from apriltag import apriltag
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectTagNode(Node):
    def __init__(self):
        super().__init__("detect_tag")

        self.counter = 0

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.callback_image, 1
        )

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Construct argument parser and pase the arguments
        # self.ap = argparse.ArgumentParser()
        # self.ap.add_argument("-i", "--image", required=True,
        #                 help="path to input image")
        # args = vars(self.ap.parse_args())
        

    def callback_image(self, msg):
        self.counter += 1
        if self.counter % 50 == 0:
            
            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # show_img(cv_image)
  
            except CvBridgeError as e:
                print(e)
            
            
            self.get_logger().info("Loading Image...")

            # Load input image and convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Define AprilTag detector
            self.get_logger().info("Detecting AprilTags...")
            detector = apriltag("tag36h11")
            detections = detector.detect(gray)
         
            #self.get_logger().info("total AprilTags detected" + str(detections))
            
            #err = apriltag.estimate_tag_pose()

            self.get_logger().info("Tag # " + str(detections[0]["id"]))
            


def main(args=None):
    rclpy.init(args=args)
    node = DetectTagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
