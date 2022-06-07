#!/usr/bin/env python3

# detect_tag_pupil.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from Pupil Labs
# Outputs:
#
# Purpose: To detect AprilTag

from torch import det
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from mathutils import Vector
from mathutils import Matrix
from mathutils import Quaternion



import pupil_apriltags

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Pose



class DetectTagPupilNode(Node):
    def __init__(self):
        super().__init__("detect_tag_pupil")

        self.counter = 0

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.callback_image, 1
        )

        self.detections_publisher = self.create_publisher(Pose, "detections", 10)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Camera parameters, {fx, fy}: focal length (px), {cx, cy}: focal center (px)
        self.fx = 2714.3
        self.fy = 2714.3
        self.cx = 1640
        self.cy = 1232
        self.tag_size = 0.3
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

    def callback_image(self, msg):
        self.counter += 1
        if self.counter % 50 == 0:
            self.counter = 0

            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # show_img(cv_image)

            except CvBridgeError as e:
                print(e)

            #self.get_logger().info("Loading Image...")

            # Load input image and convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Define AprilTag detector
            #self.get_logger().info("Detecting AprilTags...")
            detector = pupil_apriltags.Detector(families='tag36h11')

            detections = detector.detect(
                gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

            #self.get_logger().info("total AprilTags detected" + str(detections))

            #err = apriltag.estimate_tag_pose()
            
            # if detections not empty
            if len(detections) >= 1:       
                # self.get_logger().info("Tag family: " + str(detections[0].tag_family))
                # self.get_logger().info("Tag #: " + str(detections[0].tag_id))
                # self.get_logger().info("Pose_R: " + str(detections[0].pose_R))
                # self.get_logger().info("Pose_t: " + str(detections[0].pose_t))
                self.publish_detections_data(detections)

    def publish_detections_data(self, detections):
        detections_msg = Pose()

        mat = Matrix()
        mat[0][0] = detections[0].pose_R[0,0]
        mat[0][1] = detections[0].pose_R[0,1]
        mat[0][2] = detections[0].pose_R[0,2]
        mat[1][0] = detections[0].pose_R[1,0]
        mat[1][1] = detections[0].pose_R[1,1]
        mat[1][2] = detections[0].pose_R[1,2]
        mat[2][0] = detections[0].pose_R[2,0]
        mat[2][1] = detections[0].pose_R[2,1]
        mat[2][2] = detections[0].pose_R[2,2]
        q = mat.to_quaternion()

        # Put data into pose msg
        detections_msg.orientation.w = q[0]
        detections_msg.orientation.x = q[1]
        detections_msg.orientation.y = q[2]
        detections_msg.orientation.z = q[3]
        detections_msg.position.x = detections[0].pose_t[0,0]
        detections_msg.position.y = detections[0].pose_t[1,0]
        detections_msg.position.z = detections[0].pose_t[2,0]

        self.detections_publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectTagPupilNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
