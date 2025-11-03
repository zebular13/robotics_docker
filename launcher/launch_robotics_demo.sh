#!/bin/bash

# Navigate to the compose directory
cd /home/monica/robotics_docker/compose # change to your path where docker-compose.yml is located

# Launch the docker container
echo "Starting Docker container..."
docker compose up -d robotics_demo

# Wait a moment for container to be ready
sleep 2

# Allow X-Windows on host
echo "Configuring X-Windows access..."
xhost +

# Check if workspace is already built, if not build it
echo "Checking workspace setup..."
docker compose exec robotics_demo bash -ic '
if [ ! -f /root/hand_controller/ros2_ws/install/setup.bash ]; then
    echo "First time setup - building workspace..."
    cd /root/hand_controller
    git pull
    cd ros2_ws
    colcon build
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
    echo "source /root/hand_controller/ros2_ws/install/setup.bash >> /root/.bashrc" 
    echo "Workspace built successfully!"
else
    echo "Workspace already built, skipping build step."
fi
echo "Launching the demo..."
ros2 launch hand_controller demo11_mogiros_car_part2.launch.py use_imshow:=False 
'
