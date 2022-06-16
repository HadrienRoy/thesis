# thesis: Multiple autonomous robot docking

THIS PROJECT IS ONGOING.

The goal for this project is to be able to dock multiple Turtlebot3 robots autonomously with only one docking station by comparing charging needs.

It leverages ROS2 Foxy and the Pupil Labs apriltag package.

Current progress: Single Turtlebot3 apriltag docking.


The docking manager is inspired from https://github.com/Adlink-ROS/apriltag_docking.

## Quickstart

```sh
pip install pupil-apriltags

mkdir thesis_ros2_ws/src
cd ~/thesis_ros2_ws/src
git clone https://github.com/HadrienRoy/thesis.git
cd ..
colcon build --symlink-install
```

## Usage

Launch the gazebo simulator and fake battery node
```sh
ros2 launch my_simulations my_world.py
```
Launch the docking controller and docking client
```sh
ros2 launch my_robot_bringup docking_controller.launch.py
```
Launch the docking client
```sh
ros2 launch my_robot_bringup docking_controller.launch.py
```

Move the robot manually using teleop to starting position
```sh
ros2 run turtlebot3_teleop teleop_keyboard
```

In the current battery configuration, the robot will be triggered to start docking after 1 minute. To extend this time, change the inequality in docking_client.cpp. The battery percentage will drop by 0.1 every minute for reference.