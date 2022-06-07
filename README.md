# thesis: Multiple autonomous robot docking

THIS PROJECT IS NOT FINISHED

This repository holds the code for my thesis project.



The goal for this project is to be able to dock multiple Turtlebot3 robots autonomously when there is only one docking station. 

I use the Pupil Labs AprilTag library.
This docking controller is based off of https://github.com/Adlink-ROS/apriltag_docking.

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
