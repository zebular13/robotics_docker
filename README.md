## Overview

Docker container scripts for ROS2 development.

Contents of Docker image:
- Ubuntu 24.04 (Noble)
- ROS2 (Jazzy)
- Gazebo (Harmonic)

Additional content:
- MoveIt2
    - built from source, for moveit_py package
- ASL Controller
   - https://github.com/AlbertaBeef/asl_mediapipe_pointnet
- MyCobot-280 simulation
   - https://github.com/automaticaddison/mycobot_ros2 (branch=jazzy)
   - https://github.com/moveit/moveit_task_constructor (branch=jazzy)
   - https://github.com/moveit/warehouse_ros_mongo (branch=ros2)
- Yahboom ROSMASTER-X3 simulation
   - https://github.com/automaticaddison/yahboom_rosmaster
- MOGI-ROS simulation
   - https://github.com/MOGI-ROS/mogi_trajectory_server
   - https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics
   - https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
   - https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation
   - https://github.com/MOGI-ROS/Week-9-10-Simple-arm

## Cloning the repo

- git clone https://github.com/AlbertaBeef/robotics_docker


## Building the Docker image

The pre-built Docker image can be accessed on Docker Hub:
- https://hub.docker.com/repository/docker/albertabeef/robotics_docker

If you prefer to build the Docker image locally,

Start by downloading the following archive to the "robotics_docker" directory:
- gazebo_models.zip [Google Drive](https://drive.google.com/uc?export=download&id=1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_)

Then launch the Docker build script as follows:
- cd robotics_docker/docker
- source ./build.sh


## Launching the Docker image

If not done so already, build Docker image, or download the image from Docker Hub:
- docker pull albertabeef/robotics_docker:latest

Create a directory for shared content, and indicate its location in the following file:
- robotics_docker/compose/docker-compose.yml 

Launch Docker image using compose functionnality:
- cd robotics_docker/compose
- docker compose up -d robotics_demo

Making the local host's GUI available to the Docker image:
- xhost +
- docker compose exec robotics_demo bash


## Running the ASL-controlled Turtlesim demo

Launch the asl_controller_twist node with turtlesim:

   - ros2 launch asl_mediapipe_pointnet asl_mediapipe_pointnet_turtlesim.launch.py

Control Turtle with Hand Signs

   - A : Advance
   - B : Backup
   - L : Turn Left
   - R : Turn Right
   
![](https://github.com/AlbertaBeef/asl_mediapipe_pointnet/blob/main/images/asl_mediapipe_pointnet_demo01_ros2_turtlesim.gif)


## Running the ASL-controlled MOGI-ROS wheeled vehicle demo

Launch the asl_controller_twist node with MOGI-ROS vehicle:

   - ros2 launch asl_mediapipe_pointnet asl_mediapipe_pointnet_mogiros_car.launch.py

Control Vehicle with Hand Signs

   - A : Advance
   - B : Backup
   - L : Turn Left
   - R : Turn Right

![](https://github.com/AlbertaBeef/asl_mediapipe_pointnet/blob/main/images/asl_mediapipe_pointnet_demo01_ros2_gazebo.gif)


## Running the ASL-controlled Yahboom ROSMASTER-X3 demo

Launch the asl_controller_twist node with ROSMASTER-X3 vehicle:

   - ros2 launch asl_mediapipe_pointnet asl_mediapipe_pointnet_rosmaster.launch.py

Control Vehicle with Hand Signs

   - A : Advance
   - B : Backup
   - L : Turn Left
   - R : Turn Right


![](https://github.com/AlbertaBeef/asl_mediapipe_pointnet/blob/main/images/asl_mediapipe_pointnet_demo02_ros2_gazebo_rosmaster.gif)


## Running the ASL-controlled MOGI-ROS robotic arm demo

Launch the asl_controller_pose node with MOGI-ROS simple robotic arm:

   - ros2 launch asl_mediapipe_pointnet asl_mediapipe_pointnet_mogiros_arm.launch.py

Control Robotic Arm with Left/Right Hands:

   - Left Hand
      - L : Turn Arm Left
      - R : Turn Arm Right
      - A : Advance Arm (shoulder joint)
      - B : Backup Arm (shoulder joint)
      - U : Lift Arm (elbow joint)
      - D : Lower Arm (elbow joint)

   - Right Hand
      - A : Close Gripper
      - B : Open Gripper

![](https://github.com/AlbertaBeef/asl_mediapipe_pointnet/blob/main/images/asl_mediapipe_pointnet_demo04_ros2_gazebo_mogiros_arm.gif)


## Running the ASL-controlled MYCOBOT-280 robotic arm demo

Launch the asl_controller_pose node with MYCOBOT-280 robotic arm:

   - moveit &
   - ros2 launch asl_mediapipe_pointnet asl_mediapipe_pointnet_mycobot.launch.py


Control Robotic Arm with Hand Signs

   - L : Move Left
   - R : Move Right
   - A : Move Forward
   - B : Move Backward
   - U : Move Up
   - D : Move Down

![](https://github.com/AlbertaBeef/asl_mediapipe_pointnet/blob/main/images/asl_mediapipe_pointnet_demo03_ros2_gazebo_mycobot.gif)



## References

The Complete Guide to Docker for ROS 2 Jazzy Projects
   - https://automaticaddison.com/the-complete-guide-to-docker-for-ros-2-jazzy-projects/

Automatic Addison on-line Tutorials:
   - https://automaticaddison.com/tutorials
   - https://github.com/automaticaddison/mycobot_ros2 (branch=jazzy)
   - https://github.com/automaticaddison/yahboom_rosmaster   

MOGI-ROS on-line Tutorials:
   - https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS2
   - https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics
   - https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
   - https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation
   - https://github.com/MOGI-ROS/Week-9-10-Simple-arm

ASL Recognition using PointNet
   - Article [Medium](https://medium.com/@er_95882/asl-recognition-using-pointnet-and-mediapipe-f2efda78d089)
   - Dataset [Kaggle](https://www.kaggle.com/datasets/ayuraj/asl-dataset)
   - Source [Github](https://github.com/e-roe/pointnet_hands/tree/main)

