#!/usr/bin/env python3

# detect_tag_pupil.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from Pupil Labs
# Outputs:
#
# Purpose: To detect AprilTag

from re import M
from torch import det
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
import pupil_apriltags
from my_robot_interfaces.msg import AprilTagDetection
from my_robot_interfaces.msg import AprilTagDetectionArray

from mathutils import Vector
from mathutils import Matrix
from mathutils import Quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from docking_interfaces.srv import StartAprilTagDetection, Docking


from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class DetectTagPupilNode(Node):
    def __init__(self):
        super().__init__("detect_tag_pupil")

        self.counter = 0

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.callback_image, qos_profile_sensor_data)

        self.detections_publisher = self.create_publisher(
            Pose, "detections", 10)

        self.start_tag_detection_service = self.create_service(
            StartAprilTagDetection, 'detect_apriltag_pupil/start_apriltag_detection', self.start_apriltag_detection_server)

        # Initialize the transform broadcaster
        # self.br = TransformBroadcaster(self)
        self.br = StaticTransformBroadcaster(self)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.start_tag_detection = False

        # Camera parameters, {fx, fy}: focal length (px), {cx, cy}: focal center (px)
        self.fx = 530.4669406576809
        self.fy = 530.4669406576809
        self.cx = 320.5
        self.cy = 240.5
        self.tag_size = 0.24545
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

        # Define AprilTag detector
        self.detector = pupil_apriltags.Detector(families='tag36h11',
                                                 nthreads=1,
                                                 quad_decimate=1.0,
                                                 quad_sigma=0.0,
                                                 refine_edges=1,
                                                 decode_sharpening=0.25,
                                                 debug=0)

        self.get_logger().info("AprilTag Detection Node has been started.")

    
    def start_apriltag_detection_server(self, request, response):
        if request.service == 'start':
            self.start_tag_detection = True

        response.success = True

        return response
    
    
    def callback_image(self, msg):
        self.counter += 1
        if self.counter % 25 == 0:
            self.counter = 0

            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            except Exception as e:
                self.get_logger().error(e)

            # Load input image and convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(
                gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

            # if detections not empty
            if len(detections) >= 1:
                self.publish_detections_data(detections)

    def publish_detections_data(self, detections):
        if (self.start_tag_detection):
            detections_msg = Pose()

            mat = Matrix()
            mat[0][0] = detections[0].pose_R[0, 0]
            mat[0][1] = detections[0].pose_R[0, 1]
            mat[0][2] = detections[0].pose_R[0, 2]
            mat[1][0] = detections[0].pose_R[1, 0]
            mat[1][1] = detections[0].pose_R[1, 1]
            mat[1][2] = detections[0].pose_R[1, 2]
            mat[2][0] = detections[0].pose_R[2, 0]
            mat[2][1] = detections[0].pose_R[2, 1]
            mat[2][2] = detections[0].pose_R[2, 2]
            q = mat.to_quaternion()
            q.normalize()

            # q0=qw, q1=qx, q2=qy, q3=qz
            # detections_msg.orientation.w = q[0]
            # detections_msg.orientation.x = q[1]
            # detections_msg.orientation.y = q[2]
            # detections_msg.orientation.z = q[3]

            # detections_msg.position.x = detections[0].pose_t[0, 0]
            # detections_msg.position.y = detections[0].pose_t[1, 0]
            # detections_msg.position.z = detections[0].pose_t[2, 0]

            # # Send the pose of the detected tag
            # self.detections_publisher.publish(detections_msg)

            # tag_detection = AprilTagDetection()
            # tag_detection_array = AprilTagDetectionArray()

            # tag_detection.id = detections[0].tag_id
            # tag_detection.size = self.tag_size
            # tag_detection.pose.position.x = detections[0].pose_t[0, 0]
            # tag_detection.pose.position.y = detections[0].pose_t[1, 0]
            # tag_detection.pose.position.z = detections[0].pose_t[2, 0]
            # tag_detection.orientation.w = q[0]
            # tag_detection.orientation.x = q[1]
            # tag_detection.orientation.y = q[2]
            # tag_detection.orientation.z = q[3]

            # tag_detection_array.detections.push_back(tag_detection)

            t = TransformStamped()

            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_rgb_optical_frame'
            t.child_frame_id = "tag_36h11_00408"

            t.transform.translation.x = detections[0].pose_t[0, 0]
            t.transform.translation.y = detections[0].pose_t[1, 0]
            t.transform.translation.z = detections[0].pose_t[2, 0]

            # q0=qw, q1=qx, q2=qy, q3=qz
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]

            # Send the transformation
            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DetectTagPupilNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
