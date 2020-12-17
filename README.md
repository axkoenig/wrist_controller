# Gazebo Wrist Controller

[![Build Status](https://travis-ci.com/axkoenig/wrist_controller.svg?token=KeJradpJgXCJqZfQ8pwB&branch=develop)](https://travis-ci.com/axkoenig/wrist_controller)

This ROS package mimics a wrist pose controller of a robot manipulator in Gazebo and allows you to conveniently control the 6D pose of your robot hand attached to the wrist. The project aims at accelerating robotic grasping research by reducing the computational effort of simulating the robot manipulator. This wrist controller package was initially based on the repository [jsbruglie/grasp](https://github.com/jsbruglie/grasp).

## Method

The package creates five virtual links between the world frame and your robot base frame. It then connects these links with 6 joints - 3 prismatic joints for translation in X,Y,Z and 3 revolute joints for rotation in roll, pitch, yaw. I then add actuators to the joints via the [ROS transmissions](http://wiki.ros.org/urdf/XML/Transmission). Upon launch, each of the actuators is controlled via the `gazebo_ros_control` plug-in. I set the PID and effort values such that the hand quickly reaches the target pose.

## Setup Instructions

1. Install Gazebo 11 with the DART 6 physics engine (DART has proven to work better at simulating grasping than the default physics engine (ODE)). You will need to build Gazebo from source to work with DART. Follow the [official instructions](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) for doing so.

2. Install ROS noetic by following the [official instructions](https://wiki.ros.org/noetic/Installation/Ubuntu). The `ros-noetic-desktop` is recommended. The `ros-noetic-desktop-full` would also install Gazebo and this might conflict with the installation from step 1. 

3. Install ROS controllers for robot control and the Gazebo ROS integration.
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
``` 

4. Setup new catkin workspace.

```bash
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
```

5. Clone repository.

```bash
git clone https://github.com/axkoenig/wrist_controller.git
```

6. Build workspace.

```bash
cd ~/catkin_ws
catkin_make
```

7. Source this workspace and, if you like, add it to your `.bashrc`.

```bash
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

8. Change the name of the robot base link to fit your robot URDF model.

```

```

9. Start your Gazebo simulation and check if everything works by using `static_transform_publisher`.

```

```

10. You might need to adapt the PID gains, effort values or execution velocities to your use case.


## Acknowledgements

- The author of the [jsbruglie/grasp](https://github.com/jsbruglie/grasp) repository.