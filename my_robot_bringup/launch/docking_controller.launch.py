import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    camera_config = os.path.join(get_package_share_directory("picam_node"),
                          "config", "camerav2_1280x960.yaml")

    detect_tag_pupil = Node(
        package="my_apriltag",
        executable="detect_tag_pupil",
        name="detect_tag_pupil"
    )

    docking_controller = Node(
        package="docking_controller",
        executable="docking_controller",
        name="docking_controller"
    )

    docking_client = Node(
        package="docking_controller",
        executable="docking_client",
        name="docking_client"
        # parameters=[
        #     {"start_docking_controller": False},
        # ]
    )

    camera_info = Node(
        package="picam_node",
        executable="camera_info_node",
        name="camera_info_node"
        #parameters=[camera_config]
    )

    ld.add_action(detect_tag_pupil)
    ld.add_action(docking_controller)
    # ld.add_action(docking_client)
    # ld.add_action(camera_info)

    return ld
